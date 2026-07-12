"""Deterministic point-mass regression for constrained terminal guidance.

This mirrors the fixed-arrival cubic reference, acceleration/tilt constraints,
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
NET_HALF_WIDTH = 10.0
TGO_MIN = 35.0
TGO_MAX = 90.0
MAX_HORIZONTAL_ACCEL = 8.0
PID_SWITCH_HEIGHT = 500.0
CENTERING_HOLD_HEIGHT = 30.0
CENTERING_HOLD_ERROR = 7.0
WIRE_HOLD_HEIGHT = 12.0
CABLE_TRIGGER_HEIGHT = 16.0
CABLE_TRIGGER_HALF_WIDTH = 8.0
CABLE_CLOSE_SECONDS = (8.0 - 2.15) / 2.5
CABLE_TRACKING_SPEED = 5.0
CABLE_SETTLE_ERROR = 0.5
CLOSED_CABLE_INSET = 2.15
TERMINAL_ACCEL_FILTER = 0.10
FINAL_HORIZONTAL_SPEED = 2.5
ENTRY_MAX_TILT = 28.0
LANDING_MAX_TILT = 12.0
DRAG_FACTOR = 2.5e-5
ATMOSPHERE_SCALE_HEIGHT = 5000.0


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


def run(case: Case) -> Result:
    h = case.height
    vv = case.vertical_speed
    x, z, vx, vz = case.x, case.z, case.vx, case.vz
    integral = 0.0
    was_pid_mode = False
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
    initial_error = math.hypot(x, z)
    plan_vertical_tgo = 2.0 * max(h, 1.0) / max(-vv + CAPTURE_SPEED, 1.0)
    plan_horizontal_tgo = 2.0 * initial_error / max(
        math.hypot(vx, vz) + FINAL_HORIZONTAL_SPEED, 10.0
    )
    plan_arrival = clamp(max(plan_vertical_tgo, plan_horizontal_tgo), TGO_MIN, TGO_MAX)

    for step in range(int(480 / DT)):
        now = step * DT
        history.append((h, vv, x, z, vx, vz))
        delay_steps = min(len(history) - 1, int(case.sensor_delay / DT))
        sensed_h, sensed_vv, sensed_x, sensed_z, sensed_vx, sensed_vz = history[-1 - delay_steps]

        error = math.hypot(sensed_x, sensed_z)
        pid_mode = sensed_h <= PID_SWITCH_HEIGHT
        if pid_mode and not was_pid_mode:
            integral = 0.0
        was_pid_mode = pid_mode
        centering_required = (
            pid_mode
            and sensed_h < CENTERING_HOLD_HEIGHT
            and error > CENTERING_HOLD_ERROR
        )
        wire_required = sensed_h < WIRE_HOLD_HEIGHT and not net_closed

        tgo = max(plan_arrival - now, 1.5)
        vertical_net_accel = (
            -6.0 * max(sensed_h, 0.0) / (tgo**2)
            - (4.0 * sensed_vv - 2.0 * CAPTURE_SPEED) / tgo
        )
        if sensed_vv < 0:
            safe_braking_accel = sensed_vv**2 / (2.0 * max(sensed_h, 1.0)) * 1.18
            vertical_net_accel = max(vertical_net_accel, safe_braking_accel)
        vertical_thrust_command = G + vertical_net_accel
        ax = 6.0 * sensed_x / (tgo**2) - 4.0 * sensed_vx / tgo
        az = 6.0 * sensed_z / (tgo**2) - 4.0 * sensed_vz / tgo
        ax, az = clamp_vector(ax, az, MAX_HORIZONTAL_ACCEL)

        if pid_mode:
            pid_horizontal_speed = max(
                FINAL_HORIZONTAL_SPEED, min(10.0, max(sensed_h, 0.0) * 0.03)
            )
            desired_vx, desired_vz = clamp_vector(
                sensed_x * 0.050, sensed_z * 0.050, pid_horizontal_speed
            )
            ax, az = clamp_vector(
                (desired_vx - sensed_vx) * 0.20,
                (desired_vz - sensed_vz) * 0.20,
                MAX_HORIZONTAL_ACCEL,
            )
            pid_target_v = -max(
                CAPTURE_SPEED, min(8.0, max(sensed_h, 0.0) * 0.04)
            )
            if centering_required:
                pid_target_v = -CAPTURE_SPEED
            if wire_required:
                pid_target_v = 0.0
            integral = clamp(
                integral + (pid_target_v - sensed_vv) * DT, -10.0, 10.0
            )
            vertical_thrust_command = (
                G
                + 0.62 * (pid_target_v - sensed_vv)
                + 0.045 * integral
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
        altitude_blend = clamp(sensed_h / 800.0, 0.0, 1.0)
        tilt_limit = math.radians(
            ENTRY_MAX_TILT * altitude_blend
            + LANDING_MAX_TILT * (1.0 - altitude_blend)
        )
        max_horizontal_accel = max(vertical_thrust_command, G * 0.2) * math.tan(tilt_limit)
        ax, az = clamp_vector(ax, az, max_horizontal_accel)

        vertical_accel = vertical_thrust_command - G

        # KSP must rotate a long stage before the commanded lateral thrust is
        # realised. A first-order 0.4 s lag catches near-field gains that look
        # stable with instantaneous point-mass acceleration but oscillate in game.
        response = DT / (0.4 + DT)
        actual_ax += (ax - actual_ax) * response
        actual_az += (az - actual_az) * response

        density = math.exp(-max(h, 0.0) / ATMOSPHERE_SCALE_HEIGHT)
        vertical_drag = -DRAG_FACTOR * density * vv * abs(vv)
        horizontal_speed = math.hypot(vx, vz)
        drag_ax = -DRAG_FACTOR * density * vx * horizontal_speed
        drag_az = -DRAG_FACTOR * density * vz * horizontal_speed

        vv += (vertical_accel + vertical_drag) * DT
        vx += (actual_ax + drag_ax) * DT
        vz += (actual_az + drag_az) * DT
        h += vv * DT
        x -= vx * DT  # positive velocity is toward the target
        z -= vz * DT
        minimum_height = min(minimum_height, h)

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
            return Result(
                captured, now, vv, lateral_speed, lateral_error, minimum_height,
                hover_time, math.hypot(cable_x - x, cable_z - z),
            )

    return Result(
        False, 480.0, vv, math.hypot(vx, vz), math.hypot(x, z),
        minimum_height, hover_time, math.hypot(cable_x - x, cable_z - z),
    )


def cases(seed: int = 20260710, count: int = 250) -> list[Case]:
    rng = random.Random(seed)
    low_count = count // 2
    result = [
        Case(
            height=rng.uniform(450, 1800),
            vertical_speed=rng.uniform(-95, -20),
            x=rng.uniform(-130, 130),
            z=rng.uniform(-130, 130),
            vx=rng.uniform(-18, 18),
            vz=rng.uniform(-18, 18),
            available_accel=rng.uniform(22, 36),
            sensor_delay=rng.uniform(0, 0.08),
        )
        for _ in range(low_count)
    ]
    for _ in range(count - low_count):
        angle = rng.uniform(-math.pi, math.pi)
        distance = rng.uniform(400, 2500)
        along_speed = rng.uniform(90, 140)
        cross_speed = rng.uniform(-15, 15)
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        result.append(Case(
            height=rng.uniform(9000, 10000),
            vertical_speed=rng.uniform(-540, -430),
            x=distance * direction_x,
            z=distance * direction_z,
            vx=along_speed * direction_x - cross_speed * direction_z,
            vz=along_speed * direction_z + cross_speed * direction_x,
            available_accel=rng.uniform(34, 40),
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
    failures = [result for result in results if not result.captured]
    for result in failures[:5]:
        print("failure", result)
    # This broad Monte-Carlo gate is intentionally below 100%; point-mass cases at
    # the low-TWR/high-delay corner identify the envelope rather than hide it.
    return 0 if len(successes) >= int(len(results) * 0.90) else 1


if __name__ == "__main__":
    raise SystemExit(main())
