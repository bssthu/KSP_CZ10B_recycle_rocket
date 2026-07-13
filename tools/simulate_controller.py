"""Deterministic point-mass regression for constrained terminal guidance.

This mirrors the fixed height-indexed Hermite reference, acceleration/tilt constraints,
near-field PID handover and continuously tracking cable wait in ``main.ks``. It is not a
high-fidelity KSP atmosphere or propulsion simulation.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass


DT = 0.02
G = 9.81
CAPTURE_SPEED = 0.65
NET_MAX_VERTICAL = 7.5
NET_MAX_LATERAL = 4.0
NET_HALF_WIDTH = 12.0
LOW_FUEL_FRACTION = 0.0025
LOW_FUEL_DESCENT_SCALE = 1.28
LOW_FUEL_CAPTURE_SPEED = 1.0
RECOVERY_START_FRACTION = 0.20
DESCENT_SPEED_PER_METER = 0.100
DESCENT_MAX_SPEED = 700.0
PLAN_POSITION_GAIN = 0.035
PLAN_VELOCITY_GAIN = 0.65
PLAN_STOP_ACCEL = 3.0
MAX_HORIZONTAL_ACCEL = 55.0
PID_SWITCH_HEIGHT = 18.0
HORIZONTAL_CORRIDOR_HEIGHT = 1000.0
HORIZONTAL_CORRIDOR_RANGE = 700.0
CENTERING_HOLD_HEIGHT = 16.0
CENTERING_HOLD_ERROR = 7.0
WIRE_HOLD_HEIGHT = 12.0
WIRE_HOLD_HORIZONTAL_RANGE = 20.0
WIRE_HOLD_MAX_SECONDS = 2.5
POST_WIRE_CROSSING_SPEED = 1.5
CABLE_TRIGGER_HEIGHT = 100.0
CABLE_TRIGGER_HALF_WIDTH = 12.0
CABLE_CLOSE_SECONDS = (8.0 - 2.15) / 15.0
CABLE_TRACKING_SPEED = 10.0
CABLE_SETTLE_ERROR = 0.75
CABLE_DETECTION_DEPTH = 6.0
CLOSED_CABLE_INSET = 2.15
TERMINAL_ACCEL_FILTER = 0.10
FINAL_HORIZONTAL_SPEED = 0.5
HORIZONTAL_CORRIDOR_SPEED = 45.0
HORIZONTAL_STOP_ACCEL = 1.5
HORIZONTAL_DEADBAND = 3.0
HORIZONTAL_VELOCITY_GAIN = 1.0
HORIZONTAL_ALIGN_RANGE = 300.0
HORIZONTAL_ALIGN_SPEED = 12.0
HORIZONTAL_ALIGN_POSITION_GAIN = 0.10
HORIZONTAL_ALIGN_VELOCITY_GAIN = 0.65
FINAL_ALIGN_HEIGHT = 45.0
FINAL_ALIGN_RANGE = 100.0
FINAL_ALIGN_HOLD_SECONDS = 12.0
FINAL_ALIGN_SPEED = 3.0
FINAL_ALIGN_POSITION_GAIN = 0.05
FINAL_ALIGN_VELOCITY_GAIN = 0.40
FINAL_ALIGN_READY_ERROR = 6.0
FINAL_ALIGN_READY_SPEED = 0.75
FINAL_ALIGN_READY_TILT = math.radians(1.0)
FINAL_CAPTURE_VELOCITY_GAIN = 0.40
FINAL_CAPTURE_MAX_ACCEL = 1.0
ENTRY_MAX_TILT = 55.0
LANDING_MAX_TILT = 12.0
VERTICAL_DRAG_FACTOR = 2.5e-5
HORIZONTAL_DRAG_FACTOR = 1.5e-4
ATMOSPHERE_SCALE_HEIGHT = 5000.0
MIDCOURSE_START_HEIGHT = 30000.0
TERMINAL_GUIDANCE_START_HEIGHT = 30000.0
MIDCOURSE_END_MARGIN = 600.0
MIDCOURSE_PREDICTED_ERROR = 999999999.0
MIDCOURSE_MAX_HORIZONTAL_ACCEL = 8.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_vector(x: float, y: float, limit: float) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= limit or magnitude == 0:
        return x, y
    scale = limit / magnitude
    return x * scale, y * scale


@dataclass
class Case:
    height: float
    vertical_speed: float
    x: float
    z: float
    vx: float
    vz: float
    available_accel: float
    sensor_delay: float


@dataclass
class Result:
    captured: bool
    time: float
    vertical_speed: float
    lateral_speed: float
    lateral_error: float
    minimum_height: float
    hover_time: float
    cable_error: float
    ignition_height: float
    full_throttle_seconds: float
    rebound_after_center: float


def run(case: Case) -> Result:
    h = case.height
    vv = case.vertical_speed
    x, z, vx, vz = case.x, case.z, case.vx, case.vz
    integral = 0.0
    was_pid_mode = False
    capture_align_mode = False
    final_align_mode = False
    final_align_started_at = None
    final_descent_armed = False
    wire_hold_started_at = None
    cable_close_elapsed = 0.0
    net_closed = False
    cable_x = 0.0
    cable_z = 0.0
    actual_ax = 0.0
    actual_az = 0.0
    filtered_ax = 0.0
    filtered_az = 0.0
    history: list[tuple[float, float, float, float, float, float]] = []
    minimum_height = h
    hover_time = 0.0
    ignition_height = -1.0
    full_throttle_seconds = 0.0
    entered_center = False
    rebound_after_center = 0.0
    powered = False
    plan_height = 0.0
    plan_x = plan_z = 0.0
    plan_error_slope_x = plan_error_slope_z = 0.0

    for step in range(int(480 / DT)):
        now = step * DT
        history.append((h, vv, x, z, vx, vz))
        delay_steps = min(len(history) - 1, int(case.sensor_delay / DT))
        sensed_h, sensed_vv, sensed_x, sensed_z, sensed_vx, sensed_vz = history[-1 - delay_steps]

        density = math.exp(-max(h, 0.0) / ATMOSPHERE_SCALE_HEIGHT)
        vertical_drag = -VERTICAL_DRAG_FACTOR * density * vv * abs(vv)
        horizontal_speed = math.hypot(vx, vz)
        drag_ax = -HORIZONTAL_DRAG_FACTOR * density * vx * horizontal_speed
        drag_az = -HORIZONTAL_DRAG_FACTOR * density * vz * horizontal_speed

        if not powered:
            available_net_decel = max(
                case.available_accel * 0.94 - G + max(vertical_drag, 0.0), 0.1
            )
            stop_distance = max(sensed_vv**2 - CAPTURE_SPEED**2, 0.0) / (
                2.0 * available_net_decel
            ) * 1.50
            if sensed_vv < 0 and sensed_h <= stop_distance + 250.0:
                powered = True
                ignition_height = sensed_h
                plan_height = max(sensed_h, 1.0)
                plan_x, plan_z = sensed_x, sensed_z
                progress_rate0 = max(-sensed_vv, CAPTURE_SPEED) / plan_height
                plan_error_slope_x = -sensed_vx / max(progress_rate0, 0.0001)
                plan_error_slope_z = -sensed_vz / max(progress_rate0, 0.0001)
            else:
                effective_g = max(G - max(vertical_drag, 0.0), 1.0)
                ballistic_tgo = (
                    sensed_vv
                    + math.sqrt(max(sensed_vv**2 + 2.0 * effective_g * sensed_h, 0.0))
                ) / effective_g
                predicted_x = sensed_vx * ballistic_tgo + 0.5 * drag_ax * ballistic_tgo**2
                predicted_z = sensed_vz * ballistic_tgo + 0.5 * drag_az * ballistic_tgo**2
                predicted_error = math.hypot(
                    sensed_x - predicted_x, sensed_z - predicted_z
                )
                midcourse = (
                    sensed_h <= MIDCOURSE_START_HEIGHT
                    and sensed_h > stop_distance + MIDCOURSE_END_MARGIN
                    and predicted_error > MIDCOURSE_PREDICTED_ERROR
                )
                target_ax = 0.0
                target_az = 0.0
                vertical_thrust = 0.0
                if midcourse:
                    desired_vx = (sensed_x - 0.5 * drag_ax * ballistic_tgo**2) / max(
                        ballistic_tgo, 1.0
                    )
                    desired_vz = (sensed_z - 0.5 * drag_az * ballistic_tgo**2) / max(
                        ballistic_tgo, 1.0
                    )
                    target_ax, target_az = clamp_vector(
                        (desired_vx - sensed_vx) * 0.30,
                        (desired_vz - sensed_vz) * 0.30,
                        MIDCOURSE_MAX_HORIZONTAL_ACCEL,
                    )
                    vertical_thrust = 0.0
                    full_throttle_seconds += (
                        math.sqrt(vertical_thrust**2 + target_ax**2 + target_az**2)
                        / max(case.available_accel, 0.001)
                        * DT
                    )
                response = DT / (0.4 + DT)
                actual_ax += (target_ax - actual_ax) * response
                actual_az += (target_az - actual_az) * response
                vv += (vertical_thrust - G + vertical_drag) * DT
                vx += (actual_ax + drag_ax) * DT
                vz += (actual_az + drag_az) * DT
                h += vv * DT
                x -= vx * DT
                z -= vz * DT
                minimum_height = min(minimum_height, h)
                if h <= 0:
                    return Result(
                        False, now, vv, math.hypot(vx, vz), math.hypot(x, z),
                        minimum_height, hover_time,
                        math.hypot(cable_x - x, cable_z - z),
                        ignition_height, full_throttle_seconds,
                        rebound_after_center,
                    )
                continue

        error = math.hypot(sensed_x, sensed_z)
        pid_mode = sensed_h <= PID_SWITCH_HEIGHT
        horizontal_corridor_mode = (
            sensed_h <= HORIZONTAL_CORRIDOR_HEIGHT
            and error <= HORIZONTAL_CORRIDOR_RANGE
        )
        if horizontal_corridor_mode and error <= HORIZONTAL_ALIGN_RANGE:
            capture_align_mode = True
        if (
            not final_align_mode
            and sensed_h <= FINAL_ALIGN_HEIGHT
            and error <= FINAL_ALIGN_RANGE
        ):
            final_align_mode = True
            final_align_started_at = now
            integral = 0.0
        if pid_mode and not was_pid_mode:
            integral = 0.0
        was_pid_mode = pid_mode
        centering_required = (
            pid_mode
            and sensed_h < CENTERING_HOLD_HEIGHT
            and error > CENTERING_HOLD_ERROR
        )
        wire_geometry_required = (
            sensed_h < WIRE_HOLD_HEIGHT
            and error <= WIRE_HOLD_HORIZONTAL_RANGE
            and not net_closed
        )
        if wire_geometry_required and wire_hold_started_at is None:
            wire_hold_started_at = now
        wire_required = (
            wire_geometry_required
            and now - wire_hold_started_at < WIRE_HOLD_MAX_SECONDS
            and not final_align_mode
        )

        corridor_down_speed = max(
            CAPTURE_SPEED,
            min(DESCENT_MAX_SPEED, max(sensed_h, 0.0) * DESCENT_SPEED_PER_METER),
        )
        estimated_fraction = RECOVERY_START_FRACTION * max(
            0.0, 1.0 - full_throttle_seconds / 31.0
        )
        if estimated_fraction < LOW_FUEL_FRACTION:
            corridor_down_speed *= LOW_FUEL_DESCENT_SCALE
        target_vertical_speed = -corridor_down_speed
        vertical_net_accel = 2.0 * (target_vertical_speed - sensed_vv)
        if sensed_vv < target_vertical_speed:
            brake_safety_blend = clamp(
                sensed_h / TERMINAL_GUIDANCE_START_HEIGHT, 0.0, 1.0
            )
            brake_safety = 1.0 + (1.50 - 1.0) * brake_safety_blend
            safe_braking_accel = (
                sensed_vv**2 / (2.0 * max(sensed_h, 1.0)) * brake_safety
            )
            vertical_net_accel = max(vertical_net_accel, safe_braking_accel)
        vertical_thrust_command = G + vertical_net_accel
        progress = clamp(1.0 - sensed_h / max(plan_height, 1.0), 0.0, 1.0)
        progress2 = progress**2
        progress3 = progress**3
        h00 = 2.0 * progress3 - 3.0 * progress2 + 1.0
        h10 = progress3 - 2.0 * progress2 + progress
        reference_x = plan_x * h00 + plan_error_slope_x * h10
        reference_z = plan_z * h00 + plan_error_slope_z * h10
        d_error_ds_x = (
            plan_x * (6.0 * progress2 - 6.0 * progress)
            + plan_error_slope_x * (3.0 * progress2 - 4.0 * progress + 1.0)
        )
        d_error_ds_z = (
            plan_z * (6.0 * progress2 - 6.0 * progress)
            + plan_error_slope_z * (3.0 * progress2 - 4.0 * progress + 1.0)
        )
        d2_error_ds2_x = (
            plan_x * (12.0 * progress - 6.0)
            + plan_error_slope_x * (6.0 * progress - 4.0)
        )
        d2_error_ds2_z = (
            plan_z * (12.0 * progress - 6.0)
            + plan_error_slope_z * (6.0 * progress - 4.0)
        )
        progress_rate = max(-sensed_vv, CAPTURE_SPEED) / max(plan_height, 1.0)
        reference_vx = -d_error_ds_x * progress_rate
        reference_vz = -d_error_ds_z * progress_rate
        plan_stop_speed = math.sqrt(
            2.0 * PLAN_STOP_ACCEL * max(error - HORIZONTAL_DEADBAND, 0.0)
        )
        reference_vx, reference_vz = clamp_vector(
            reference_vx, reference_vz, plan_stop_speed
        )
        ax = (
            -d2_error_ds2_x * progress_rate**2
            + (sensed_x - reference_x) * PLAN_POSITION_GAIN
            + (reference_vx - sensed_vx) * PLAN_VELOCITY_GAIN
        )
        az = (
            -d2_error_ds2_z * progress_rate**2
            + (sensed_z - reference_z) * PLAN_POSITION_GAIN
            + (reference_vz - sensed_vz) * PLAN_VELOCITY_GAIN
        )
        ax, az = clamp_vector(ax, az, MAX_HORIZONTAL_ACCEL)

        if horizontal_corridor_mode:
            stop_range = max(error - HORIZONTAL_DEADBAND, 0.0)
            stop_speed = math.sqrt(2.0 * HORIZONTAL_STOP_ACCEL * stop_range)
            pid_horizontal_speed = min(stop_speed, HORIZONTAL_CORRIDOR_SPEED)
            if capture_align_mode:
                pid_horizontal_speed = min(
                    HORIZONTAL_ALIGN_SPEED,
                    stop_range * HORIZONTAL_ALIGN_POSITION_GAIN,
                )
            if error > HORIZONTAL_DEADBAND:
                desired_vx = sensed_x / error * pid_horizontal_speed
                desired_vz = sensed_z / error * pid_horizontal_speed
            else:
                desired_vx, desired_vz = 0.0, 0.0
            velocity_gain = HORIZONTAL_VELOCITY_GAIN
            if capture_align_mode:
                velocity_gain = HORIZONTAL_ALIGN_VELOCITY_GAIN
            ax, az = clamp_vector(
                (desired_vx - sensed_vx) * velocity_gain,
                (desired_vz - sensed_vz) * velocity_gain,
                MAX_HORIZONTAL_ACCEL,
            )

        if final_align_mode and not final_descent_armed:
            final_stop_range = error
            final_horizontal_speed = min(
                FINAL_ALIGN_SPEED,
                final_stop_range * FINAL_ALIGN_POSITION_GAIN,
            )
            if error > 0.25:
                desired_vx = sensed_x / error * final_horizontal_speed
                desired_vz = sensed_z / error * final_horizontal_speed
            else:
                desired_vx, desired_vz = 0.0, 0.0
            ax, az = clamp_vector(
                (desired_vx - sensed_vx) * FINAL_ALIGN_VELOCITY_GAIN,
                (desired_vz - sensed_vz) * FINAL_ALIGN_VELOCITY_GAIN,
                MAX_HORIZONTAL_ACCEL,
            )

        estimated_tilt = math.atan2(math.hypot(actual_ax, actual_az), G)
        if (
            final_align_mode
            and not final_descent_armed
            and error <= FINAL_ALIGN_READY_ERROR
            and math.hypot(sensed_vx, sensed_vz) <= FINAL_ALIGN_READY_SPEED
            and estimated_tilt <= FINAL_ALIGN_READY_TILT
        ):
            final_descent_armed = True
            filtered_ax = filtered_az = 0.0

        if pid_mode:
            pid_target_v = -max(
                CAPTURE_SPEED, min(8.0, max(sensed_h, 0.0) * 0.04)
            )
            if wire_required and estimated_fraction >= LOW_FUEL_FRACTION:
                pid_target_v = 0.0
            if wire_hold_started_at is not None and not wire_required:
                pid_target_v = -max(POST_WIRE_CROSSING_SPEED, -pid_target_v)
            if estimated_fraction < LOW_FUEL_FRACTION:
                pid_target_v = -max(LOW_FUEL_CAPTURE_SPEED, -pid_target_v)
            integral = clamp(
                integral + (pid_target_v - sensed_vv) * DT, -10.0, 10.0
            )
            vertical_thrust_command = (
                G
                + 2.00 * (pid_target_v - sensed_vv)
                + 0.020 * integral
            )

        if final_align_mode and not final_descent_armed:
            final_target_v = 0.0
            assert final_align_started_at is not None
            if now - final_align_started_at >= FINAL_ALIGN_HOLD_SECONDS:
                final_target_v = -POST_WIRE_CROSSING_SPEED
            integral = clamp(
                integral + (final_target_v - sensed_vv) * DT, -10.0, 10.0
            )
            vertical_thrust_command = (
                G
                + 2.00 * (final_target_v - sensed_vv)
                + 0.020 * integral
            )

        if final_descent_armed:
            ax, az = clamp_vector(
                -sensed_vx * FINAL_CAPTURE_VELOCITY_GAIN,
                -sensed_vz * FINAL_CAPTURE_VELOCITY_GAIN,
                FINAL_CAPTURE_MAX_ACCEL,
            )

        filtered_ax = (
            filtered_ax * (1.0 - TERMINAL_ACCEL_FILTER)
            + ax * TERMINAL_ACCEL_FILTER
        )
        filtered_az = (
            filtered_az * (1.0 - TERMINAL_ACCEL_FILTER)
            + az * TERMINAL_ACCEL_FILTER
        )
        ax, az = filtered_ax, filtered_az

        vertical_thrust_command = clamp(
            vertical_thrust_command, 0.0, case.available_accel * 0.96
        )
        full_throttle_seconds += (
            vertical_thrust_command / max(case.available_accel, 0.001) * DT
        )
        altitude_blend = clamp(sensed_h / 800.0, 0.0, 1.0)
        tilt_limit = math.radians(
            ENTRY_MAX_TILT * altitude_blend
            + LANDING_MAX_TILT * (1.0 - altitude_blend)
        )
        max_horizontal_accel = max(vertical_thrust_command, G * 0.2) * math.tan(tilt_limit)
        ax, az = clamp_vector(ax, az, max_horizontal_accel)

        vertical_accel = vertical_thrust_command - G

        # The full-size KSP stage takes roughly 1.5 s to turn a lateral command
        # into acceleration. This deliberately harsh lag reproduces the phase
        # delay which the earlier point-mass regression hid.
        response = DT / (1.5 + DT)
        actual_ax += (ax - actual_ax) * response
        actual_az += (az - actual_az) * response

        vv += (vertical_accel + vertical_drag) * DT
        vx += (actual_ax + drag_ax) * DT
        vz += (actual_az + drag_az) * DT
        h += vv * DT
        x -= vx * DT  # positive velocity is toward the target
        z -= vz * DT
        minimum_height = min(minimum_height, h)
        current_error = math.hypot(x, z)
        if current_error <= 5.0:
            entered_center = True
        elif entered_center:
            rebound_after_center = max(rebound_after_center, current_error)

        if h < 150.0 and h > WIRE_HOLD_HEIGHT and abs(vv) < 0.3:
            hover_time += DT

        if h <= CABLE_TRIGGER_HEIGHT and math.hypot(x, z) <= CABLE_TRIGGER_HALF_WIDTH:
            cable_close_elapsed += DT
            close_fraction = clamp(cable_close_elapsed / CABLE_CLOSE_SECONDS, 0.0, 1.0)
            cable_inset = 8.0 + (CLOSED_CABLE_INSET - 8.0) * close_fraction
            cable_limit = NET_HALF_WIDTH - cable_inset
            target_cable_x = clamp(x, -cable_limit, cable_limit)
            target_cable_z = clamp(z, -cable_limit, cable_limit)
            cable_dx, cable_dz = target_cable_x - cable_x, target_cable_z - cable_z
            cable_step_x, cable_step_z = clamp_vector(
                cable_dx, cable_dz, CABLE_TRACKING_SPEED * DT
            )
            cable_x += cable_step_x
            cable_z += cable_step_z
            net_closed = (
                close_fraction >= 1.0
                and math.hypot(cable_x - x, cable_z - z) <= CABLE_SETTLE_ERROR
            )
        elif not net_closed:
            cable_close_elapsed = 0.0
            cable_dx, cable_dz = -cable_x, -cable_z
            cable_step_x, cable_step_z = clamp_vector(
                cable_dx, cable_dz, CABLE_TRACKING_SPEED * DT
            )
            cable_x += cable_step_x
            cable_z += cable_step_z

        if h <= 0:
            lateral_speed = math.hypot(vx, vz)
            lateral_error = math.hypot(x, z)
            captured = (
                abs(vv) <= NET_MAX_VERTICAL
                and lateral_speed <= NET_MAX_LATERAL
                and lateral_error <= NET_HALF_WIDTH
                and net_closed
            )
            if captured or h <= -CABLE_DETECTION_DEPTH:
                return Result(
                    captured, now, vv, lateral_speed, lateral_error, minimum_height,
                    hover_time, math.hypot(cable_x - x, cable_z - z),
                    ignition_height, full_throttle_seconds,
                    rebound_after_center,
                )

    return Result(
        False, 480.0, vv, math.hypot(vx, vz), math.hypot(x, z),
        minimum_height, hover_time, math.hypot(cable_x - x, cable_z - z),
        ignition_height, full_throttle_seconds,
        rebound_after_center,
    )


def cases(seed: int = 20260710, count: int = 250) -> list[Case]:
    rng = random.Random(seed)
    low_count = count // 3
    medium_count = count // 3
    result: list[Case] = []
    # The ship is parked near the drag-aware ballistic footprint. Perturb that
    # planned corridor rather than asking an 18 m PID handover to repair an
    # intentionally impossible kilometre-scale targeting error.
    for _ in range(low_count):
        height = rng.uniform(450, 1800)
        vertical_speed = rng.uniform(-95, -20)
        vx = rng.uniform(-18, 18)
        vz = rng.uniform(-18, 18)
        tgo = (
            vertical_speed
            + math.sqrt(vertical_speed**2 + 2.0 * G * height)
        ) / G
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=vx * tgo + rng.uniform(-35, 35),
            z=vz * tgo + rng.uniform(-35, 35),
            vx=vx,
            vz=vz,
            available_accel=rng.uniform(45, 65),
            sensor_delay=rng.uniform(0, 0.08),
        ))
    for _ in range(medium_count):
        angle = rng.uniform(-math.pi, math.pi)
        height = rng.uniform(9000, 10000)
        vertical_speed = rng.uniform(-540, -430)
        along_speed = rng.uniform(90, 140)
        cross_speed = rng.uniform(-15, 15)
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        tgo = (
            vertical_speed
            + math.sqrt(vertical_speed**2 + 2.0 * G * height)
        ) / G
        # Dense lower air removes part of this displacement.  The online
        # predictor measures the exact KSP value; 0.8 plus noise is the offline
        # uncertainty envelope around that estimate.
        distance = along_speed * tgo * 0.8 + rng.uniform(-250, 250)
        cross_offset = rng.uniform(-120, 120)
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=distance * direction_x - cross_offset * direction_z,
            z=distance * direction_z + cross_offset * direction_x,
            vx=along_speed * direction_x - cross_speed * direction_z,
            vz=along_speed * direction_z + cross_speed * direction_x,
            available_accel=rng.uniform(45, 65),
            sensor_delay=rng.uniform(0, 0.08),
        ))
    # High-energy cases are centred on the measured 0.8 trajectory entry after
    # the 40 km retrograde burn: about 30 km, -0.46 km/s vertical, 1.0 km/s
    # horizontal and a ship placed 30--45 km ahead near the natural footprint.
    for _ in range(count - low_count - medium_count):
        angle = rng.uniform(-math.pi, math.pi)
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        height = rng.uniform(28500, 30000)
        vertical_speed = rng.uniform(-550, -400)
        along_speed = rng.uniform(950, 1050)
        cross_speed = rng.uniform(-35, 35)
        ballistic_tgo = (
            vertical_speed
            + math.sqrt(vertical_speed**2 + 2.0 * G * height)
        ) / G
        distance = along_speed * ballistic_tgo * 0.91 + rng.uniform(-1500, 1500)
        cross_offset = rng.uniform(-500, 500)
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=distance * direction_x - cross_offset * direction_z,
            z=distance * direction_z + cross_offset * direction_x,
            vx=along_speed * direction_x - cross_speed * direction_z,
            vz=along_speed * direction_z + cross_speed * direction_x,
            available_accel=rng.uniform(60, 72),
            sensor_delay=rng.uniform(0, 0.08),
        ))
    return result


def main() -> int:
    results = [run(case) for case in cases()]
    successes = [result for result in results if result.captured]
    print(f"capture_rate={len(successes)}/{len(results)} ({len(successes) / len(results):.1%})")
    if successes:
        print(f"worst_entry_vertical={max(abs(r.vertical_speed) for r in successes):.3f} m/s")
        print(f"worst_entry_lateral={max(r.lateral_speed for r in successes):.3f} m/s")
        print(f"worst_entry_error={max(r.lateral_error for r in successes):.3f} m")
        print(f"worst_hover_time_below_150m={max(r.hover_time for r in successes):.3f} s")
        print(f"worst_cable_tracking_error={max(r.cable_error for r in successes):.3f} m")
        print(f"highest_ignition={max(r.ignition_height for r in successes):.1f} m")
        print(f"worst_full_throttle_equivalent={max(r.full_throttle_seconds for r in successes):.2f} s")
        print(f"worst_rebound_after_center={max(r.rebound_after_center for r in successes):.3f} m")
    failures = [result for result in results if not result.captured]
    for result in failures[:5]:
        print("failure", result)
    # This broad Monte-Carlo gate is intentionally below 100%; point-mass cases at
    # the low-TWR/high-delay corner identify the envelope rather than hide it.
    stable_successes = [r for r in successes if r.rebound_after_center <= 8.0]
    print(f"stable_capture_rate={len(stable_successes)}/{len(results)}")
    return 0 if len(stable_successes) >= int(len(results) * 0.90) else 1


if __name__ == "__main__":
    raise SystemExit(main())
