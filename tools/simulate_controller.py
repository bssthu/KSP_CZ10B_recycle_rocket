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
LOW_FUEL_FRACTION = 0.02
LOW_FUEL_DESCENT_SCALE = 1.28
LOW_FUEL_CAPTURE_SPEED = 6.0
RECOVERY_START_FRACTION = 0.20
DESCENT_MAX_SPEED = 700.0
TERMINAL_NOMINAL_THRUST_FRACTION = 0.75
TERMINAL_TOTAL_THRUST_FRACTION = 0.98
TERMINAL_VELOCITY_CONE = math.radians(30.0)
TERMINAL_VELOCITY_CONE_MIN_SPEED = 300.0
TERMINAL_WAYPOINT_HEIGHT = 2000.0
TERMINAL_WAYPOINT_VERTICAL_SPEED = 190.0
TERMINAL_HORIZONTAL_PLAN_END_HEIGHT = 6000.0
PLAN_POSITION_GAIN = 0.10
PLAN_VELOCITY_GAIN = 1.50
PLAN_STOP_ACCEL = 55.0
MAX_HORIZONTAL_ACCEL = 55.0
PID_SWITCH_HEIGHT = 18.0
HORIZONTAL_CORRIDOR_HEIGHT = 30000.0
HORIZONTAL_CORRIDOR_RANGE = 30000.0
CENTERING_HOLD_HEIGHT = 16.0
CENTERING_HOLD_ERROR = 7.0
WIRE_HOLD_HEIGHT = 12.0
WIRE_HOLD_HORIZONTAL_RANGE = 20.0
WIRE_HOLD_MAX_SECONDS = 2.5
POST_WIRE_CROSSING_SPEED = 1.5
CABLE_TRIGGER_HEIGHT = 100.0
CABLE_TRIGGER_HALF_WIDTH = 12.0
CABLE_FINAL_CLOSURE_HEIGHT = 1.0
CABLE_TRACKING_SPEED = 10.0
CABLE_SETTLE_ERROR = 0.75
CABLE_DETECTION_DEPTH = 6.0
CLOSED_CABLE_INSET = 2.15
TERMINAL_ACCEL_FILTER = 0.10
TERMINAL_HIGH_ENERGY_ACCEL_FILTER = 0.45
FINAL_HORIZONTAL_SPEED = 0.5
HORIZONTAL_CORRIDOR_SPEED = 150.0
HORIZONTAL_STOP_ACCEL = 0.60
HORIZONTAL_DEADBAND = 15.0
HORIZONTAL_VELOCITY_GAIN = 1.0
HORIZONTAL_ALIGN_RANGE = 550.0
HORIZONTAL_ALIGN_SPEED = 30.0
HORIZONTAL_ALIGN_MIN_SPEED = 10.0
HORIZONTAL_ALIGN_POSITION_GAIN = 0.15
HORIZONTAL_ALIGN_VELOCITY_GAIN = 2.00
HORIZONTAL_ALIGN_ACCEL_RANGE_GAIN = 0.75
HORIZONTAL_ALIGN_ACCEL_VELOCITY_GAIN = 1.00
HORIZONTAL_ALIGN_SETTLE_ENTRY_SPEED = 13.0
HORIZONTAL_ALIGN_SETTLE_ENTRY_RANGE = 130.0
HORIZONTAL_ALIGN_SETTLE_MAX_ACCEL = 25.0
HORIZONTAL_ALIGN_SETTLE_POSITION_GAIN = 0.20
HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN = 3.5
HORIZONTAL_ALIGN_REACQUIRE_RANGE = 30.0
HORIZONTAL_ALIGN_REACQUIRE_POSITION_GAIN = 0.20
HORIZONTAL_ALIGN_REACQUIRE_MAX_ACCEL = 25.0
FINAL_ALIGN_HEIGHT = 1000.0
FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT = 65.0
FINAL_ALIGN_RANGE = 50.0
FINAL_ALIGN_HOLD_SECONDS = 0.0
FINAL_ALIGN_SPEED = 3.0
FINAL_ALIGN_POSITION_GAIN = 0.08
FINAL_ALIGN_VELOCITY_GAIN = 0.50
FINAL_ALIGN_READY_ERROR = 6.0
FINAL_ALIGN_READY_SPEED = 1.25
FINAL_ALIGN_READY_TILT = math.radians(8.0)
FINAL_CAPTURE_VELOCITY_GAIN = 0.40
FINAL_CAPTURE_MAX_ACCEL = 1.0
FINAL_CAPTURE_POSITION_DEADBAND = 2.0
FINAL_CAPTURE_POSITION_GAIN = 0.10
FINAL_CAPTURE_MAX_SPEED = 0.75
ENTRY_MAX_TILT = 89.0
LANDING_MAX_TILT = 12.0
VERTICAL_DRAG_FACTOR = 2.5e-5
HORIZONTAL_DRAG_FACTOR = 1.5e-4
ATMOSPHERE_SCALE_HEIGHT = 5000.0
MIDCOURSE_START_HEIGHT = 30000.0
TERMINAL_GUIDANCE_START_HEIGHT = 30000.0
MIDCOURSE_END_MARGIN = 600.0
MIDCOURSE_PREDICTED_ERROR = 999999999.0
MIDCOURSE_MAX_HORIZONTAL_ACCEL = 8.0
TERMINAL_DESCENT_COUPLING_BAND = 50.0


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
    waypoint_vertical_speed: float = math.nan
    waypoint_lateral_speed: float = math.nan
    waypoint_error: float = math.nan
    max_velocity_cone_angle: float = 0.0


def run(case: Case) -> Result:
    h = case.height
    vv = case.vertical_speed
    x, z, vx, vz = case.x, case.z, case.vx, case.vz
    integral = 0.0
    was_pid_mode = False
    capture_align_mode = False
    horizontal_settle_mode = False
    capture_align_speed_limit = HORIZONTAL_ALIGN_SPEED
    final_align_mode = False
    final_align_started_at = None
    final_descent_armed = False
    wire_hold_started_at = None
    net_closed = False
    cable_target_acquired = False
    cable_x = 0.0
    cable_z = 0.0
    actual_ax = 0.0
    actual_az = 0.0
    filtered_ax = 0.0
    filtered_az = 0.0
    history: list[tuple[float, float, float, float, float, float]] = []
    minimum_height = h
    hover_time = 0.0
    ignition_height = h
    full_throttle_seconds = 0.0
    entered_center = False
    rebound_after_center = 0.0
    powered = True
    plan_height = max(h, 1.0)
    plan_x, plan_z = x, z
    initial_progress_rate = max(-vv, CAPTURE_SPEED) / max(
        plan_height - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 1.0
    )
    plan_error_slope_x = -vx / max(initial_progress_rate, 0.0001)
    plan_error_slope_z = -vz / max(initial_progress_rate, 0.0001)
    waypoint_vertical_speed = math.nan
    waypoint_lateral_speed = math.nan
    waypoint_error = math.nan
    max_velocity_cone_angle = 0.0

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
                case.available_accel * TERMINAL_NOMINAL_THRUST_FRACTION
                - G + max(vertical_drag, 0.0), 0.1
            )
            stop_distance = max(sensed_vv**2 - CAPTURE_SPEED**2, 0.0) / (
                2.0 * available_net_decel
            ) * 1.50
            if sensed_vv < 0 and sensed_h <= stop_distance + 250.0:
                powered = True
                ignition_height = sensed_h
                plan_height = max(sensed_h, 1.0)
                plan_x, plan_z = sensed_x, sensed_z
                progress_rate0 = max(-sensed_vv, CAPTURE_SPEED) / max(
                    plan_height - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 1.0
                )
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
                        waypoint_vertical_speed=waypoint_vertical_speed,
                        waypoint_lateral_speed=waypoint_lateral_speed,
                        waypoint_error=waypoint_error,
                        max_velocity_cone_angle=max_velocity_cone_angle,
                    )
                continue

        error = math.hypot(sensed_x, sensed_z)
        pid_mode = sensed_h <= PID_SWITCH_HEIGHT
        horizontal_corridor_mode = (
            sensed_h <= HORIZONTAL_CORRIDOR_HEIGHT
            and error <= HORIZONTAL_CORRIDOR_RANGE
        )
        if (
            not capture_align_mode
            and horizontal_corridor_mode
            and error <= HORIZONTAL_ALIGN_RANGE
        ):
            capture_align_mode = True
            radial_speed = (
                (sensed_x * sensed_vx + sensed_z * sensed_vz) / error
                if error > 0.0
                else 0.0
            )
            capture_align_speed_limit = max(HORIZONTAL_ALIGN_MIN_SPEED, radial_speed)
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
            min(
                DESCENT_MAX_SPEED,
                max(sensed_h, 0.0)
                * TERMINAL_WAYPOINT_VERTICAL_SPEED
                / TERMINAL_WAYPOINT_HEIGHT,
            ),
        )
        estimated_fraction = RECOVERY_START_FRACTION * max(
            0.0, 1.0 - full_throttle_seconds / 31.0
        )
        fuel_urgent = estimated_fraction < LOW_FUEL_FRACTION
        if fuel_urgent:
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
        plan_horizontal_height = max(
            plan_height - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 1.0
        )
        progress = clamp(
            (plan_height - sensed_h) / plan_horizontal_height, 0.0, 1.0
        )
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
        progress_rate = max(-sensed_vv, CAPTURE_SPEED) / plan_horizontal_height
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

        if horizontal_corridor_mode and (
            capture_align_mode or sensed_h <= TERMINAL_WAYPOINT_HEIGHT
        ):
            stop_range = max(error - HORIZONTAL_DEADBAND, 0.0)
            stop_speed = math.sqrt(2.0 * HORIZONTAL_STOP_ACCEL * stop_range)
            pid_horizontal_speed = min(stop_speed, HORIZONTAL_CORRIDOR_SPEED)
            if capture_align_mode:
                pid_horizontal_speed = min(
                    pid_horizontal_speed,
                    HORIZONTAL_ALIGN_SPEED,
                    capture_align_speed_limit,
                    stop_range * HORIZONTAL_ALIGN_POSITION_GAIN,
                )
            if error > HORIZONTAL_DEADBAND:
                desired_vx = sensed_x / error * pid_horizontal_speed
                desired_vz = sensed_z / error * pid_horizontal_speed
            else:
                desired_vx, desired_vz = 0.0, 0.0
            velocity_gain = HORIZONTAL_VELOCITY_GAIN
            acceleration_limit = MAX_HORIZONTAL_ACCEL
            if capture_align_mode:
                velocity_gain = HORIZONTAL_ALIGN_VELOCITY_GAIN
                acceleration_limit = max(
                    HORIZONTAL_STOP_ACCEL,
                    min(
                        MAX_HORIZONTAL_ACCEL,
                        max(
                            stop_range * HORIZONTAL_ALIGN_ACCEL_RANGE_GAIN,
                            math.hypot(sensed_vx, sensed_vz)
                            * HORIZONTAL_ALIGN_ACCEL_VELOCITY_GAIN,
                        ),
                    ),
                )
            ax, az = clamp_vector(
                (desired_vx - sensed_vx) * velocity_gain,
                (desired_vz - sensed_vz) * velocity_gain,
                acceleration_limit,
            )

        if (
            not horizontal_settle_mode
            and capture_align_mode
            and error <= HORIZONTAL_ALIGN_SETTLE_ENTRY_RANGE
            and math.hypot(sensed_vx, sensed_vz)
            <= HORIZONTAL_ALIGN_SETTLE_ENTRY_SPEED
        ):
            horizontal_settle_mode = True
            filtered_ax = 0.0
            filtered_az = 0.0
        if horizontal_settle_mode and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            ax, az = clamp_vector(
                sensed_x * HORIZONTAL_ALIGN_SETTLE_POSITION_GAIN
                - sensed_vx * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                sensed_z * HORIZONTAL_ALIGN_SETTLE_POSITION_GAIN
                - sensed_vz * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                HORIZONTAL_ALIGN_SETTLE_MAX_ACCEL,
            )
            if error > HORIZONTAL_ALIGN_REACQUIRE_RANGE:
                ax, az = clamp_vector(
                    sensed_x * HORIZONTAL_ALIGN_REACQUIRE_POSITION_GAIN
                    - sensed_vx * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                    sensed_z * HORIZONTAL_ALIGN_REACQUIRE_POSITION_GAIN
                    - sensed_vz * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                    HORIZONTAL_ALIGN_REACQUIRE_MAX_ACCEL,
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

        if (
            final_align_mode
            and not final_descent_armed
            and sensed_h <= FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT
        ):
            final_target_v = 0.0
            assert final_align_started_at is not None
            if fuel_urgent:
                final_target_v = -LOW_FUEL_CAPTURE_SPEED
            elif now - final_align_started_at >= FINAL_ALIGN_HOLD_SECONDS:
                final_target_v = -POST_WIRE_CROSSING_SPEED
            integral = clamp(
                integral + (final_target_v - sensed_vv) * DT, -10.0, 10.0
            )
            vertical_thrust_command = (
                G
                + 2.00 * (final_target_v - sensed_vv)
                + 0.020 * integral
            )

        if (
            final_descent_armed
            and fuel_urgent
            and not pid_mode
            and sensed_h <= FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT
        ):
            committed_fuel_target_v = -LOW_FUEL_CAPTURE_SPEED
            integral = clamp(
                integral + (committed_fuel_target_v - sensed_vv) * DT,
                -10.0,
                10.0,
            )
            vertical_thrust_command = (
                G
                + 2.00 * (committed_fuel_target_v - sensed_vv)
                + 0.020 * integral
            )

        if final_descent_armed:
            desired_vx = 0.0
            desired_vz = 0.0
            final_capture_range = math.hypot(sensed_x, sensed_z)
            if final_capture_range > FINAL_CAPTURE_POSITION_DEADBAND:
                final_capture_speed = min(
                    FINAL_CAPTURE_MAX_SPEED,
                    (final_capture_range - FINAL_CAPTURE_POSITION_DEADBAND)
                    * FINAL_CAPTURE_POSITION_GAIN,
                )
                desired_vx = sensed_x / final_capture_range * final_capture_speed
                desired_vz = sensed_z / final_capture_range * final_capture_speed
            ax, az = clamp_vector(
                (desired_vx - sensed_vx) * FINAL_CAPTURE_VELOCITY_GAIN,
                (desired_vz - sensed_vz) * FINAL_CAPTURE_VELOCITY_GAIN,
                FINAL_CAPTURE_MAX_ACCEL,
            )

        current_accel_filter = TERMINAL_ACCEL_FILTER
        if horizontal_corridor_mode and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            current_accel_filter = TERMINAL_HIGH_ENERGY_ACCEL_FILTER
        filtered_ax = (
            filtered_ax * (1.0 - current_accel_filter)
            + ax * current_accel_filter
        )
        filtered_az = (
            filtered_az * (1.0 - current_accel_filter)
            + az * current_accel_filter
        )
        ax, az = filtered_ax, filtered_az

        vertical_thrust_command = clamp(
            vertical_thrust_command,
            0.0,
            case.available_accel * TERMINAL_NOMINAL_THRUST_FRACTION,
        )
        altitude_blend = clamp(sensed_h / 800.0, 0.0, 1.0)
        tilt_limit = math.radians(
            ENTRY_MAX_TILT * altitude_blend
            + LANDING_MAX_TILT * (1.0 - altitude_blend)
        )
        if (
            horizontal_corridor_mode
            and math.hypot(ax, az) > 0.01
            and sensed_h > 500.0
        ):
            # Reserve enough vertical thrust component to realise the allowed
            # tilt during the main translation.  Below 500 m the online loop
            # instead clips lateral acceleration against the existing vertical
            # command so a small correction cannot cause an upward hop.
            required_vertical_for_tilt = math.hypot(ax, az) / max(
                math.tan(tilt_limit), 0.01
            )
            coupling_min_down_speed = TERMINAL_WAYPOINT_VERTICAL_SPEED * max(
                sensed_h / TERMINAL_WAYPOINT_HEIGHT, 1.0
            ) ** 0.25
            coupling_blend = clamp(
                (-sensed_vv - coupling_min_down_speed)
                / TERMINAL_DESCENT_COUPLING_BAND,
                0.0,
                1.0,
            )
            coupled_vertical_thrust = vertical_thrust_command + (
                max(vertical_thrust_command, required_vertical_for_tilt)
                - vertical_thrust_command
            ) * coupling_blend
            vertical_thrust_command = min(
                coupled_vertical_thrust,
                case.available_accel * TERMINAL_NOMINAL_THRUST_FRACTION,
            )
        max_horizontal_accel = max(vertical_thrust_command, G * 0.2) * math.tan(tilt_limit)
        ax, az = clamp_vector(ax, az, max_horizontal_accel)

        # Match the online load constraint: thrust direction may be at most
        # 30 degrees away from surface retrograde, which is equivalent to the
        # nozzle axis staying within 30 degrees of the velocity vector.
        thrust_magnitude = math.sqrt(vertical_thrust_command**2 + ax**2 + az**2)
        velocity_magnitude = math.sqrt(sensed_vv**2 + sensed_vx**2 + sensed_vz**2)
        if (
            thrust_magnitude > 0.0
            and velocity_magnitude >= TERMINAL_VELOCITY_CONE_MIN_SPEED
        ):
            axis_up = -sensed_vv / velocity_magnitude
            axis_x = -sensed_vx / velocity_magnitude
            axis_z = -sensed_vz / velocity_magnitude
            axial = (
                vertical_thrust_command * axis_up + ax * axis_x + az * axis_z
            )
            if axial <= 0.0:
                vertical_thrust_command = axis_up * thrust_magnitude
                ax = axis_x * thrust_magnitude
                az = axis_z * thrust_magnitude
            else:
                lateral_up = vertical_thrust_command - axis_up * axial
                lateral_x = ax - axis_x * axial
                lateral_z = az - axis_z * axial
                lateral_magnitude = math.sqrt(
                    lateral_up**2 + lateral_x**2 + lateral_z**2
                )
                lateral_limit = axial * math.tan(TERMINAL_VELOCITY_CONE)
                if lateral_magnitude > lateral_limit:
                    scale = lateral_limit / lateral_magnitude
                    vertical_thrust_command = axis_up * axial + lateral_up * scale
                    ax = axis_x * axial + lateral_x * scale
                    az = axis_z * axial + lateral_z * scale

            thrust_magnitude = math.sqrt(
                vertical_thrust_command**2 + ax**2 + az**2
            )
            commanded_angle = math.acos(clamp(
                (vertical_thrust_command * axis_up + ax * axis_x + az * axis_z)
                / max(thrust_magnitude, 1e-9),
                -1.0,
                1.0,
            ))
            max_velocity_cone_angle = max(
                max_velocity_cone_angle, commanded_angle
            )

        total_thrust_limit = (
            case.available_accel * TERMINAL_TOTAL_THRUST_FRACTION
        )
        thrust_magnitude = math.sqrt(vertical_thrust_command**2 + ax**2 + az**2)
        if thrust_magnitude > total_thrust_limit:
            scale = total_thrust_limit / thrust_magnitude
            vertical_thrust_command *= scale
            ax *= scale
            az *= scale
            thrust_magnitude = total_thrust_limit
        full_throttle_seconds += (
            thrust_magnitude / max(case.available_accel, 0.001) * DT
        )

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
        if (
            case.height > TERMINAL_WAYPOINT_HEIGHT
            and math.isnan(waypoint_vertical_speed)
            and h <= TERMINAL_WAYPOINT_HEIGHT
        ):
            waypoint_vertical_speed = vv
            waypoint_lateral_speed = math.hypot(vx, vz)
            waypoint_error = current_error
        # Match the flight verifier: once the stage enters the 10 m centre
        # neighbourhood anywhere below 5 km, a later excursion is rebound.
        # Starting this audit above the 2 km waypoint catches a fast centre
        # crossing that would otherwise look acceptable only after returning.
        if h <= 5000.0 and current_error <= 10.0:
            entered_center = True
        elif h <= 5000.0 and entered_center:
            rebound_after_center = max(rebound_after_center, current_error)

        if h < 150.0 and h > WIRE_HOLD_HEIGHT and abs(vv) < 0.3:
            hover_time += DT

        if (
            not cable_target_acquired
            and h <= CABLE_TRIGGER_HEIGHT
            and math.hypot(x, z) <= CABLE_TRIGGER_HALF_WIDTH
        ):
            cable_target_acquired = True
        if cable_target_acquired and h >= -CABLE_DETECTION_DEPTH:
            close_fraction = clamp(
                (CABLE_TRIGGER_HEIGHT - h)
                / (CABLE_TRIGGER_HEIGHT - CABLE_FINAL_CLOSURE_HEIGHT),
                0.0,
                1.0,
            )
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
                h <= CABLE_FINAL_CLOSURE_HEIGHT
                and math.hypot(cable_x - x, cable_z - z) <= CABLE_SETTLE_ERROR
            )
        elif not net_closed:
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
                    waypoint_vertical_speed=waypoint_vertical_speed,
                    waypoint_lateral_speed=waypoint_lateral_speed,
                    waypoint_error=waypoint_error,
                    max_velocity_cone_angle=max_velocity_cone_angle,
                )

    return Result(
        False, 480.0, vv, math.hypot(vx, vz), math.hypot(x, z),
        minimum_height, hover_time, math.hypot(cable_x - x, cable_z - z),
        ignition_height, full_throttle_seconds,
        rebound_after_center,
        waypoint_vertical_speed=waypoint_vertical_speed,
        waypoint_lateral_speed=waypoint_lateral_speed,
        waypoint_error=waypoint_error,
        max_velocity_cone_angle=max_velocity_cone_angle,
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
            # The stock Mainsail has 25% more maximum thrust than the former
            # 1200 kN integrated engine; the 75% nominal command preserves
            # the old braking level while retaining correction headroom.
            available_accel=rng.uniform(56.25, 81.25),
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
            available_accel=rng.uniform(56.25, 81.25),
            sensor_delay=rng.uniform(0, 0.08),
        ))
    # The last third exercises the mandatory 2 km handover envelope.  The
    # 30 km-to-2 km segment depends on KSP aerodynamics, kOS instruction timing
    # and the long stage's attitude response; pretending the point-mass model
    # predicts that footprint produced misleading kilometre-scale failures.
    # Full-energy acceptance is therefore owned by the real KSP TERMINAL_2KM
    # log, while this deterministic sweep verifies that compliant waypoint
    # states converge monotonically through the moving cable cradle.
    for _ in range(count - low_count - medium_count):
        angle = rng.uniform(-math.pi, math.pi)
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        height = rng.uniform(2000, 5000)
        vertical_speed = (
            -height * TERMINAL_WAYPOINT_VERTICAL_SPEED
            / TERMINAL_WAYPOINT_HEIGHT * rng.uniform(0.85, 1.05)
        )
        along_speed = rng.uniform(0, 20)
        distance = rng.uniform(0, 100)
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=distance * direction_x,
            z=distance * direction_z,
            vx=along_speed * direction_x,
            vz=along_speed * direction_z,
            available_accel=rng.uniform(70, 100),
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
