"""Deterministic point-mass regression test for the kOS terminal controller.

This is deliberately not a high-fidelity KSP simulator.  It catches gain changes
that cause late ignition, excessive cable-entry speed, or lateral divergence
before spending time on another game run.
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


def run(case: Case) -> Result:
    h = case.height
    vv = case.vertical_speed
    x, z, vx, vz = case.x, case.z, case.vx, case.vz
    integral = 0.0
    was_centering = False
    history: list[tuple[float, float, float, float, float, float]] = []
    minimum_height = h

    for step in range(int(180 / DT)):
        now = step * DT
        history.append((h, vv, x, z, vx, vz))
        delay_steps = min(len(history) - 1, int(case.sensor_delay / DT))
        sensed_h, sensed_vv, sensed_x, sensed_z, sensed_vx, sensed_vz = history[-1 - delay_steps]

        max_net_accel = max(case.available_accel - G, 0.1)
        stop_distance = (sensed_vv**2) / (2 * max_net_accel) * 1.18 if sensed_vv < 0 else 0.0
        altitude_blend = clamp(sensed_h / 800.0, 0.0, 1.0)
        speed_limit = 8.0 + (150.0 - 8.0) * altitude_blend
        if sensed_h < 150:
            speed_limit = 3.0
        desired_vx, desired_vz = clamp_vector(
            sensed_x * (0.050 + 0.030 * (1.0 - altitude_blend)),
            sensed_z * (0.050 + 0.030 * (1.0 - altitude_blend)),
            speed_limit,
        )
        ax, az = clamp_vector((desired_vx - sensed_vx) * 0.55,
                              (desired_vz - sensed_vz) * 0.55, 15.0)

        desired_vv = -clamp(0.42 * math.sqrt(2 * G * max(sensed_h, 0.2)),
                            CAPTURE_SPEED, 70.0)
        if sensed_h < 35:
            desired_vv = -clamp(sensed_h * 0.13, CAPTURE_SPEED, 4.0)
        if sensed_h < 5:
            desired_vv = -CAPTURE_SPEED
        centering_hold = sensed_h < 150 and math.hypot(sensed_x, sensed_z) > 5
        if centering_hold:
            desired_vv = 0.0
        if was_centering and not centering_hold:
            integral = 0.0
        was_centering = centering_hold

        horizontal_burn = math.hypot(ax, az) > 0.5 and (
            math.hypot(sensed_x, sensed_z) > 150
            or math.hypot(sensed_vx, sensed_vz) > 10
            or sensed_h < 5000
        )
        should_burn = sensed_h <= stop_distance + 80 or sensed_vv > -2 or centering_hold or horizontal_burn
        vertical_accel_command = 0.0
        if should_burn:
            integral = clamp(integral + (desired_vv - sensed_vv) * DT, -15.0, 15.0)
            vertical_accel_command = clamp(
                G + 0.62 * (desired_vv - sensed_vv) + 0.045 * integral,
                0.0,
                case.available_accel * 0.96,
            )

        tilt_limit = math.radians(28 * altitude_blend + 12 * (1 - altitude_blend))
        vertical_thrust_command = vertical_accel_command
        if horizontal_burn and vertical_thrust_command < G:
            vertical_thrust_command = G
        max_horizontal_accel = max(vertical_thrust_command, G * 0.2) * math.tan(tilt_limit)
        ax, az = clamp_vector(ax, az, max_horizontal_accel)

        # A non-burning ballistic phase has no commanded horizontal acceleration.
        if not should_burn:
            ax = az = 0.0
        vertical_accel = vertical_thrust_command - G if should_burn else -G

        vv += vertical_accel * DT
        vx += ax * DT
        vz += az * DT
        h += vv * DT
        x -= vx * DT  # positive velocity is toward the target
        z -= vz * DT
        minimum_height = min(minimum_height, h)

        if h <= 0:
            lateral_speed = math.hypot(vx, vz)
            lateral_error = math.hypot(x, z)
            captured = (
                abs(vv) <= NET_MAX_VERTICAL
                and lateral_speed <= NET_MAX_LATERAL
                and lateral_error <= NET_HALF_WIDTH
            )
            return Result(captured, now, vv, lateral_speed, lateral_error, minimum_height)

    return Result(False, 180.0, vv, math.hypot(vx, vz), math.hypot(x, z), minimum_height)


def cases(seed: int = 20260710, count: int = 250) -> list[Case]:
    rng = random.Random(seed)
    return [
        Case(
            height=rng.uniform(450, 1400),
            vertical_speed=rng.uniform(-80, -25),
            x=rng.uniform(-130, 130),
            z=rng.uniform(-130, 130),
            vx=rng.uniform(-18, 18),
            vz=rng.uniform(-18, 18),
            available_accel=rng.uniform(18, 32),
            sensor_delay=rng.uniform(0, 0.08),
        )
        for _ in range(count)
    ]


def main() -> int:
    results = [run(case) for case in cases()]
    successes = [result for result in results if result.captured]
    print(f"capture_rate={len(successes)}/{len(results)} ({len(successes) / len(results):.1%})")
    if successes:
        print(f"worst_entry_vertical={max(abs(r.vertical_speed) for r in successes):.3f} m/s")
        print(f"worst_entry_lateral={max(r.lateral_speed for r in successes):.3f} m/s")
        print(f"worst_entry_error={max(r.lateral_error for r in successes):.3f} m")
    failures = [result for result in results if not result.captured]
    for result in failures[:5]:
        print("failure", result)
    # This broad Monte-Carlo gate is intentionally below 100%; point-mass cases at
    # the low-TWR/high-delay corner identify the envelope rather than hide it.
    return 0 if len(successes) >= int(len(results) * 0.90) else 1


if __name__ == "__main__":
    raise SystemExit(main())
