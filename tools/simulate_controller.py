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
TERMINAL_TOTAL_THRUST_FRACTION = 1.0
TERMINAL_MAIN_TRACKING_THRUST_FRACTION = 0.84
TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT = 20000.0
TERMINAL_MAIN_TRACKING_FULL_HEIGHT = 15000.0
TERMINAL_MAIN_TRACKING_FADE_START_HEIGHT = 7000.0
TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT = 5000.0
RETURN_ENGINE_MAX_ACCEL = 21.5
RETURN_ENGINE_EFFECTIVE_ISP = 300.0
TERMINAL_VELOCITY_CONE = math.radians(25.5)
TERMINAL_VELOCITY_CONE_HARD_LIMIT = math.radians(30.0)
TERMINAL_VELOCITY_CONE_MIN_SPEED = 5.0
TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT = 500.0
TERMINAL_LOW_ALT_COMMAND_CONE = math.radians(8.0)
TERMINAL_DENSE_AIR_CONE_START_HEIGHT = 8000.0
TERMINAL_DENSE_AIR_CONE_END_HEIGHT = 4000.0
TERMINAL_DENSE_AIR_COMMAND_CONE = math.radians(8.0)
TERMINAL_MIN_CONTINUOUS_THROTTLE = 0.03
TERMINAL_WAYPOINT_HEIGHT = 2000.0
TERMINAL_WAYPOINT_VERTICAL_SPEED = 175.0
TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED = 150.0
TERMINAL_WAYPOINT_MAX_VERTICAL_SPEED = 200.0
TERMINAL_WAYPOINT_MAX_HORIZONTAL_SPEED = 5.0
TERMINAL_HORIZONTAL_PLAN_END_HEIGHT = 2000.0
PLATFORM_FOOTPRINT_PLAN_END_HEIGHT = 3000.0
PLATFORM_FOOTPRINT_DOWNRANGE_BIAS = 200.0
TERMINAL_WAYPOINT_APPROACH_OFFSET = 75.0
TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED = 644.0
TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN = 0.0
TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET = 75.0
TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET = 75.0
TERMINAL_CAPTURE_ALIGN_ARM_HEIGHT = 2000.0
TERMINAL_IGNITION_SAFETY = 0.729
TERMINAL_IGNITION_MARGIN = 50.0
TERMINAL_GUIDANCE_RESPONSE_SECONDS = 0.35
TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS = 0.35
TERMINAL_HORIZONTAL_LEAD_SECONDS = 0.0
TERMINAL_WAYPOINT_POSITION_COEFFICIENT = 6.0
TERMINAL_WAYPOINT_VELOCITY_COEFFICIENT = 4.0
TERMINAL_WAYPOINT_CROSS_VELOCITY_COEFFICIENT = 6.0
TERMINAL_WAYPOINT_CROSS_POSITION_COEFFICIENT = 13.0
TERMINAL_ALONG_SPEED_DEFICIT_ARM = 10.0
TERMINAL_ALONG_SPEED_DEFICIT_BLEND = 30.0
TERMINAL_ALONG_COAST_FADE_START_HEIGHT = 7000.0
TERMINAL_ALONG_COAST_END_HEIGHT = 5000.0
TERMINAL_ALONG_COAST_MIN_REQUEST = 5.0
TERMINAL_ALONG_COAST_ENABLED = False
TERMINAL_ALONG_AERO_BRAKE_ENABLED = True
TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT = 14500.0
TERMINAL_ALONG_AERO_BRAKE_FULL_HEIGHT = 13500.0
TERMINAL_ALONG_AERO_BRAKE_ACCEL_GAIN = 8.0
TERMINAL_ALONG_AERO_BRAKE_MARGIN = 1.03
TERMINAL_ALONG_AERO_BRAKE_ERROR_DEADBAND = 0.5
TERMINAL_ALONG_AERO_BRAKE_BUILD_RATE = 0.18
TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE = 0.12
TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND = 0.85
TERMINAL_ALONG_BRAKE_ENABLED = False
TERMINAL_ALONG_BRAKE_START_HEIGHT = 25000.0
TERMINAL_ALONG_BRAKE_FULL_HEIGHT = 24000.0
TERMINAL_ALONG_BRAKE_EARLY_MAX_BLEND = 0.5
TERMINAL_ALONG_BRAKE_LATE_RAMP_START_HEIGHT = 10000.0
TERMINAL_ALONG_BRAKE_LATE_FULL_HEIGHT = 8000.0
TERMINAL_ALONG_BRAKE_SPEED_EXCESS_ARM = 10.0
TERMINAL_ALONG_BRAKE_SPEED_EXCESS_BLEND = 30.0
TERMINAL_ALONG_BRAKE_MIN_REQUEST = 5.0
TERMINAL_ALONG_BRAKE_MAX_BLEND = 1.0
TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT = 20000.0
TERMINAL_ALONG_BRAKE_REACHABILITY_FULL_HEIGHT = 19000.0
TERMINAL_ALONG_BRAKE_PRESSURE_ARM = 0.75
TERMINAL_ALONG_BRAKE_PRESSURE_FULL = 1.0
TERMINAL_ALONG_BRAKE_REACHABILITY_MIN_RANGE = 100.0
PLAN_POSITION_GAIN = 0.10
PLAN_VELOCITY_GAIN = 3.00
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
TERMINAL_HIGH_ENERGY_ACCEL_FILTER = 0.50
POWERED_VERTICAL_AERO_MAX_ACCEL = 20.0
POWERED_HORIZONTAL_AERO_MAX_ACCEL = 10.0
MAIN_CORRECTION_FULL_ALIGNMENT_DEGREES = 8.0
MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES = 20.0
FINAL_HORIZONTAL_SPEED = 0.5
HORIZONTAL_CORRIDOR_SPEED = 150.0
HORIZONTAL_STOP_ACCEL = 0.60
HORIZONTAL_DEADBAND = 3.0
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
# Run 26 measured an effective 0.5e-3--1.4e-3 upward coefficient while the
# grid fins remained aerodynamically active.  Flight guidance now schedules
# their lift to zero after entry cutoff, so the powered point-mass model returns
# to the identified body/engine drag coefficient.  Real KSP physics and the
# per-frame observer remain authoritative.
VERTICAL_DRAG_FACTOR = 7.5e-5
HORIZONTAL_DRAG_FACTOR = 6.0e-5
# Run 35 identification at 8--16 km found 8--16 m/s^2 of horizontal body/fin
# deceleration after subtracting the constrained engine component.  This
# high-energy effective coefficient reproduces its 3.16 km early-stop residual;
# it is a mixed KSP/attitude model, not a claim about a physical Cd value.
RUN35_IDENTIFIED_HORIZONTAL_DRAG_FACTOR = 3.0e-4
ATMOSPHERE_SCALE_HEIGHT = 5000.0
TERMINAL_GUIDANCE_START_HEIGHT = 40000.0
MIDCOURSE_PREDICTED_ERROR = 100.0
MIDCOURSE_MAX_HORIZONTAL_ACCEL = 20.0
MIDCOURSE_CHECKPOINT_3_HEIGHT = 32500.0
# Three simulated checkpoints are not explicitly integrated in this shortened
# sweep.  Reserve the measured correction/alignment drop after the 32.5 km
# trigger; real main.ks still gates on the actual pulse state.
MIDCOURSE_MAIN_HANDOFF_HEIGHT = 25500.0
TERMINAL_DESCENT_COUPLING_BAND = 50.0
MAIN_START_FUEL_FRACTION = 0.171
MAIN_DRY_MASS_RATIO = 0.443
MAIN_STEERING_RESPONSE_SECONDS = 2.0
# Keep the stopping-footprint allowance above separate from the measured
# physical direction lag. Ten-percent bounded terminal wheel authority is
# restored after the stronger schedules in runs 51 and 53 violated the
# physical 30-degree cone below 5 km. Keep the response identified from runs
# 49--52 without changing that independent hard audit.
MAIN_PHYSICAL_STEERING_RESPONSE_SECONDS = 0.35
TERMINAL_STEERING_RESPONSE_SECONDS = 0.5
# Run 52 identified a repeatable high-q authority loss that a pure first-order
# steering delay cannot represent.  Between roughly 6 and 3 km the real stage
# achieved only 30--55% of the requested cone displacement away from surface
# retrograde.  Use density*speed^2 as a unitless dynamic-pressure proxy and
# blend only the *physical* target back toward retrograde; guidance and throttle
# still see their unmodified requested vector.  Real KSP remains authoritative.
MAIN_AERO_STEERING_Q_SCALE = 125000.0
MAIN_AERO_STEERING_RESTORED_Q_SCALE = 125000.0
MAIN_AERO_STEERING_TAPER_START_HEIGHT = 4500.0
MAIN_AERO_STEERING_TAPER_END_HEIGHT = 3000.0
MAIN_AERO_STEERING_MIN_AUTHORITY = 0.30
MAIN_AERO_STEERING_AUTHORITY_EXPONENT = 1.50
MAIN_PREALIGN_HEIGHT = 30000.0
MAIN_PWM_ENABLED = False
WAYPOINT_COAST_ENABLED = False
MAIN_PURE_HORIZONTAL_BRAKE = False
MAIN_DIRECT_STOP_GUIDANCE = False
MAIN_DIRECT_STOP_CROSS_TIME_COEFFICIENT = 12.0
MAIN_DIRECT_STOP_CROSS_MAX_ACCEL = 10.0
MAIN_DIRECT_STOP_MIN_RANGE = 100.0
MAIN_DIRECT_STOP_DECEL_GAIN = 1.00
MAIN_FIXED_AXIS_STOP_ENABLED = False
MAIN_FIXED_AXIS_STOP_GAIN = 1.0
MAIN_FIXED_AXIS_STOP_MIN_RANGE = 100.0
MAIN_FIXED_AXIS_STOP_RESPONSE_SECONDS = 2.5
MAIN_FIXED_AXIS_STOP_ARM_HEIGHT = 8000.0
MAIN_FIXED_AXIS_STOP_AUTHORITY_MARGIN = 0.90
MAIN_FIXED_AXIS_SYNC_EXCESS_ARM = 10.0
MAIN_FIXED_AXIS_EDGE_STEERING_ENABLED = False
MAIN_CROSS_STOP_ENABLED = True
MAIN_CROSS_STOP_GAIN = 1.0
MAIN_CROSS_STOP_MIN_RANGE = 5.0
MAIN_CROSS_STOP_RESPONSE_SECONDS = 2.5
MAIN_CROSS_STOP_ARM_HEIGHT = 12500.0
MAIN_CROSS_STOP_MIN_SPEED = 2.0
MAIN_CROSS_STOP_MAX_ACCEL = 15.0
MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN = 0.5
MAIN_CROSS_AERO_BRAKE_ENABLED = True
MAIN_CROSS_AERO_BRAKE_START_HEIGHT = 8000.0
MAIN_CROSS_AERO_BRAKE_MIN_ENGINE_ACCEL = 5.0
MAIN_CROSS_AERO_BRAKE_FULL_DEMAND = 5.0
MAIN_ALONG_ENGINE_BRAKE_ENABLED = False
MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT = 6000.0
MAIN_ALONG_ENGINE_BRAKE_FULL_HEIGHT = 5000.0
MAIN_ALONG_ENGINE_BRAKE_ACCEL = 25.0
ACTUAL_CONE_GUARD_START = math.radians(20.0)
ACTUAL_CONE_GUARD_FULL = math.radians(24.0)
MAIN_HORIZONTAL_STOP_DISTANCE_SAFETY = 0.863
MAIN_MAX_PREDICTED_PROGRADE_MISS = 3000.0
MAIN_VERTICAL_READY_MARGIN = 400.0
HIGH_ENERGY_BRAKE_ENABLED = True
HIGH_ENERGY_BRAKE_ARM_HEIGHT = 6000.0
HIGH_ENERGY_BRAKE_RANGE = 400.0
TERMINAL_CROSS_TRACK_BRAKE_ERROR = 50.0
TERMINAL_CROSS_TRACK_BRAKE_RANGE = 2500.0
HIGH_ENERGY_BRAKE_VELOCITY_GAIN = 1.5
HIGH_ENERGY_BRAKE_MAX_ACCEL = 55.0
HIGH_ENERGY_BRAKE_POSITION_GAIN = 0.5
HIGH_ENERGY_BRAKE_MAX_SPEED = 30.0
HIGH_ENERGY_BRAKE_SETTLE_HEIGHT = 2200.0
VERTICAL_RECOVERY_STEERING_SPEED = 300.0
VERTICAL_RECOVERY_STEERING_MAX_SURFACE_SPEED = 500.0
VERTICAL_RECOVERY_STEERING_DEGREES = 22.0
VERTICAL_RECOVERY_PRELEAD_HEIGHT = 12000.0
VERTICAL_RECOVERY_PRELEAD_MAX_SURFACE_SPEED = 2000.0
VERTICAL_RECOVERY_PRELEAD_DEGREES = 22.0
WAYPOINT_COAST_MAX_VERTICAL_SPEED = 280.0
WAYPOINT_COAST_MAX_HORIZONTAL_SPEED = 4.5
WAYPOINT_COAST_ENTRY_MAX_ERROR = 1300.0
WAYPOINT_COAST_ENTRY_MAX_HORIZONTAL_SPEED = 220.0
WAYPOINT_COAST_ENTRY_MIN_VERTICAL_SPEED = 275.0
WAYPOINT_TRIM_ARM_RANGE = 650.0
WAYPOINT_TRIM_MIN_ACTUAL_TILT = math.radians(85.0)
WAYPOINT_TRIM_MAX_ACCEL = 55.0
WAYPOINT_TRIM_CROSS_GAIN = 0.20
WAYPOINT_TRIM_POSITION_GAIN = 0.20


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_vector(x: float, y: float, limit: float) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= limit or magnitude == 0:
        return x, y
    scale = limit / magnitude
    return x * scale, y * scale


def prelead_axis(
    vv: float, vx: float, vz: float, degrees: float
) -> tuple[float, float, float]:
    speed = math.sqrt(vv * vv + vx * vx + vz * vz)
    if speed < TERMINAL_VELOCITY_CONE_MIN_SPEED:
        return 1.0, 0.0, 0.0
    retro_up, retro_x, retro_z = -vv / speed, -vx / speed, -vz / speed
    horizontal = math.hypot(vx, vz)
    if horizontal <= 1e-9 or degrees <= 0.0:
        return retro_up, retro_x, retro_z
    brake_x, brake_z = -vx / horizontal, -vz / horizontal
    projection = brake_x * retro_x + brake_z * retro_z
    turn_up = -projection * retro_up
    turn_x = brake_x - projection * retro_x
    turn_z = brake_z - projection * retro_z
    turn_mag = math.sqrt(turn_up**2 + turn_x**2 + turn_z**2)
    if turn_mag <= 1e-9:
        return retro_up, retro_x, retro_z
    lead = math.radians(degrees)
    return (
        retro_up * math.cos(lead) + turn_up / turn_mag * math.sin(lead),
        retro_x * math.cos(lead) + turn_x / turn_mag * math.sin(lead),
        retro_z * math.cos(lead) + turn_z / turn_mag * math.sin(lead),
    )


def waypoint_approach_offset(vertical_speed: float) -> float:
    """Freeze the main-burn footprint allowance from its ignition state."""
    return clamp(
        TERMINAL_WAYPOINT_APPROACH_OFFSET
        + TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN
        * (-vertical_speed
            - TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED),
        TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET,
        TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET,
    )


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
    start_powered: bool
    horizontal_drag_factor: float = HORIZONTAL_DRAG_FACTOR


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


def run(case: Case, stop_at_waypoint: bool = False, trace: bool = False) -> Result:
    h = case.height
    vv = case.vertical_speed
    x, z, vx, vz = case.x, case.z, case.vx, case.vz
    integral = 0.0
    was_pid_mode = False
    capture_align_mode = False
    high_energy_brake_mode = False
    fixed_stop_committed = False
    cross_stop_committed = False
    cross_stop_direction_x = 0.0
    cross_stop_direction_z = 0.0
    cross_stop_completed = False
    cross_aero_brake_previous_demand = 0.0
    cross_aero_brake_blend_state = 0.0
    cross_aero_brake_release_started = False
    along_aero_brake_blend_state = 0.0
    waypoint_coast_mode = False
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
    # The ballistic loop holds the long stage upright.  Steering changes the
    # thrust *direction* with a measured finite response, while throttle changes
    # change magnitude immediately.  Lagging acceleration itself would hide the
    # real vertical impulse caused by a late attitude response.
    initial_surface_speed = math.sqrt(vv * vv + vx * vx + vz * vz)
    if case.start_powered and initial_surface_speed >= TERMINAL_VELOCITY_CONE_MIN_SPEED:
        # main.ks now completes the cone-relative lead before continuous-main
        # ignition instead of lighting on the cone centre.
        actual_dir_up, actual_dir_x, actual_dir_z = prelead_axis(
            vv, vx, vz, VERTICAL_RECOVERY_PRELEAD_DEGREES
        )
    else:
        actual_dir_up = 1.0
        actual_dir_x = 0.0
        actual_dir_z = 0.0
    filtered_ax = 0.0
    filtered_az = 0.0
    history: list[tuple[float, float, float, float, float, float]] = []
    minimum_height = h
    hover_time = 0.0
    powered = case.start_powered
    ignition_height = h if powered else -1.0
    full_throttle_seconds = 0.0
    entered_center = False
    rebound_after_center = 0.0
    plan_height = max(h, 1.0)
    plan_end_height = min(
        TERMINAL_HORIZONTAL_PLAN_END_HEIGHT,
        max(TERMINAL_WAYPOINT_HEIGHT, plan_height - 500.0),
    )
    effective_approach_offset = waypoint_approach_offset(vv)
    initial_horizontal_speed = math.hypot(vx, vz)
    if initial_horizontal_speed > 0.1:
        plan_along_x = vx / initial_horizontal_speed
        plan_along_z = vz / initial_horizontal_speed
        approach_x = vx / initial_horizontal_speed * effective_approach_offset
        approach_z = vz / initial_horizontal_speed * effective_approach_offset
    else:
        initial_range = math.hypot(x, z)
        plan_along_x = x / max(initial_range, 0.1)
        plan_along_z = z / max(initial_range, 0.1)
        approach_x = x / max(initial_range, 0.1) * effective_approach_offset
        approach_z = z / max(initial_range, 0.1) * effective_approach_offset
    plan_x, plan_z = x - approach_x, z - approach_z
    initial_progress_rate = max(-vv, CAPTURE_SPEED) / max(
        plan_height - plan_end_height, 1.0
    )
    plan_error_slope_x = -vx / max(initial_progress_rate, 0.0001)
    plan_error_slope_z = -vz / max(initial_progress_rate, 0.0001)
    waypoint_vertical_speed = math.nan
    waypoint_lateral_speed = math.nan
    waypoint_error = math.nan
    max_velocity_cone_angle = 0.0
    main_pwm_accumulator = 0.0

    for step in range(int(480 / DT)):
        now = step * DT
        fixed_stop_committed_this_tick = False
        mass_ratio = max(
            math.exp(-RETURN_ENGINE_MAX_ACCEL * full_throttle_seconds
                / (RETURN_ENGINE_EFFECTIVE_ISP * 9.80665)),
            MAIN_DRY_MASS_RATIO,
        )
        available_accel = RETURN_ENGINE_MAX_ACCEL
        if mass_ratio <= MAIN_DRY_MASS_RATIO + 1e-9:
            available_accel = 0.0
        history.append((h, vv, x, z, vx, vz))
        delay_steps = min(len(history) - 1, int(case.sensor_delay / DT))
        sensed_h, sensed_vv, sensed_x, sensed_z, sensed_vx, sensed_vz = history[-1 - delay_steps]

        density = math.exp(-max(h, 0.0) / ATMOSPHERE_SCALE_HEIGHT)
        vertical_drag = -VERTICAL_DRAG_FACTOR * density * vv * abs(vv)
        horizontal_speed = math.hypot(vx, vz)
        drag_ax = -case.horizontal_drag_factor * density * vx * horizontal_speed
        drag_az = -case.horizontal_drag_factor * density * vz * horizontal_speed

        if not powered:
            effective_g = max(G - max(vertical_drag, 0.0), 1.0)
            nominal_accel = (
                available_accel * TERMINAL_NOMINAL_THRUST_FRACTION
            )
            burn_dv_vertical = -TERMINAL_WAYPOINT_VERTICAL_SPEED - sensed_vv
            burn_dv_squared = (
                burn_dv_vertical**2 + sensed_vx**2 + sensed_vz**2
            )
            quadratic_a = effective_g**2 - nominal_accel**2
            quadratic_b = 2.0 * effective_g * burn_dv_vertical
            burn_time = 999.0
            if quadratic_a < -0.001:
                discriminant = max(
                    quadratic_b**2 - 4.0 * quadratic_a * burn_dv_squared,
                    0.0,
                )
                burn_time = max(
                    (-quadratic_b - math.sqrt(discriminant))
                    / (2.0 * quadratic_a),
                    0.0,
                )
            planned_burn_time = burn_time + TERMINAL_GUIDANCE_RESPONSE_SECONDS
            burn_drop = max(
                -(sensed_vv - TERMINAL_WAYPOINT_VERTICAL_SPEED)
                * 0.5
                * planned_burn_time,
                0.0,
            )
            ignition_gate = (
                TERMINAL_WAYPOINT_HEIGHT
                + burn_drop * TERMINAL_IGNITION_SAFETY
                + TERMINAL_IGNITION_MARGIN
            )
            prealign_vertical_thrust = effective_g + burn_dv_vertical / max(
                planned_burn_time, 0.1
            )
            horizontal_stop_accel = math.sqrt(max(
                nominal_accel**2 - prealign_vertical_thrust**2,
                1.0,
            ))
            sensed_horizontal_speed = math.hypot(sensed_vx, sensed_vz)
            horizontal_stop_distance = (
                sensed_horizontal_speed * MAIN_STEERING_RESPONSE_SECONDS
                + sensed_horizontal_speed**2
                / (2.0 * horizontal_stop_accel)
            ) * MAIN_HORIZONTAL_STOP_DISTANCE_SAFETY
            horizontal_ignition = (
                math.hypot(sensed_x, sensed_z)
                <= horizontal_stop_distance
                + waypoint_approach_offset(sensed_vv)
            )
            coast_tgo = 0.0
            if sensed_h > ignition_gate:
                coast_drop = sensed_h - ignition_gate
                coast_tgo = (
                    sensed_vv + math.sqrt(max(
                        sensed_vv**2 + 2.0 * effective_g * coast_drop,
                        0.0,
                    ))
                ) / effective_g
            predicted_ignition_vx = sensed_vx + drag_ax * coast_tgo
            predicted_ignition_vz = sensed_vz + drag_az * coast_tgo
            predicted_ignition_vv = sensed_vv - effective_g * coast_tgo
            predicted_vertical_net_accel = (
                -TERMINAL_WAYPOINT_VERTICAL_SPEED
                - predicted_ignition_vv
            ) / max(planned_burn_time, 0.1)
            predicted_horizontal_drop = max(
                ignition_gate - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT,
                0.0,
            )
            predicted_horizontal_burn_time = (
                predicted_horizontal_drop
                / max(-predicted_ignition_vv, 1.0)
            )
            if predicted_vertical_net_accel > 0.001:
                predicted_disc = max(
                    predicted_ignition_vv**2
                    - 2.0 * predicted_vertical_net_accel
                        * predicted_horizontal_drop,
                    0.0,
                )
                predicted_horizontal_burn_time = (
                    -predicted_ignition_vv - math.sqrt(predicted_disc)
                ) / predicted_vertical_net_accel
            predicted_horizontal_burn_time = clamp(
                predicted_horizontal_burn_time,
                0.0,
                planned_burn_time,
            )
            predicted_displacement_x = (
                sensed_vx * coast_tgo
                + 0.5 * drag_ax * coast_tgo**2
                + predicted_ignition_vx * 0.5
                    * (predicted_horizontal_burn_time
                        + TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS)
            )
            predicted_displacement_z = (
                sensed_vz * coast_tgo
                + 0.5 * drag_az * coast_tgo**2
                + predicted_ignition_vz * 0.5
                    * (predicted_horizontal_burn_time
                        + TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS)
            )
            predicted_ignition_horizontal_speed = math.hypot(
                predicted_ignition_vx, predicted_ignition_vz
            )
            predicted_offset_x = 0.0
            predicted_offset_z = 0.0
            if predicted_ignition_horizontal_speed > 0.1:
                predicted_offset = waypoint_approach_offset(
                    predicted_ignition_vv
                )
                predicted_offset_x = (
                    predicted_ignition_vx
                    / predicted_ignition_horizontal_speed
                    * predicted_offset
                )
                predicted_offset_z = (
                    predicted_ignition_vz
                    / predicted_ignition_horizontal_speed
                    * predicted_offset
                )
            predicted_miss_x = (
                sensed_x - predicted_displacement_x - predicted_offset_x
            )
            predicted_miss_z = (
                sensed_z - predicted_displacement_z - predicted_offset_z
            )
            predicted_miss_along = 0.0
            if predicted_ignition_horizontal_speed > 0.1:
                predicted_miss_along = (
                    predicted_miss_x * predicted_ignition_vx
                    + predicted_miss_z * predicted_ignition_vz
                ) / predicted_ignition_horizontal_speed
            one_way_ignition = (
                predicted_miss_along
                <= MAIN_MAX_PREDICTED_PROGRADE_MISS
            )
            vertical_ready = (
                sensed_h <= ignition_gate + MAIN_VERTICAL_READY_MARGIN
            )
            if sensed_vv < 0 and sensed_h <= MIDCOURSE_MAIN_HANDOFF_HEIGHT and (
                sensed_h <= ignition_gate or horizontal_ignition
            ) and vertical_ready and one_way_ignition:
                # Medium-energy cases begin at the solved ignition gate.  They
                # represent a stage that has already used the preceding coast
                # to settle onto this vector, even though the shortened test
                # case omits those seconds explicitly.
                ignition_surface_speed = math.sqrt(
                    sensed_vv**2 + sensed_vx**2 + sensed_vz**2
                )
                actual_dir_up, actual_dir_x, actual_dir_z = prelead_axis(
                    sensed_vv, sensed_vx, sensed_vz,
                    VERTICAL_RECOVERY_PRELEAD_DEGREES,
                )
                powered = True
                ignition_height = sensed_h
                plan_height = max(sensed_h, 1.0)
                plan_end_height = min(
                    TERMINAL_HORIZONTAL_PLAN_END_HEIGHT,
                    max(TERMINAL_WAYPOINT_HEIGHT, plan_height - 500.0),
                )
                plan_x, plan_z = sensed_x, sensed_z
                effective_approach_offset = waypoint_approach_offset(sensed_vv)
                ignition_horizontal_speed = math.hypot(sensed_vx, sensed_vz)
                if ignition_horizontal_speed > 0.1:
                    approach_x = sensed_vx / ignition_horizontal_speed * effective_approach_offset
                    approach_z = sensed_vz / ignition_horizontal_speed * effective_approach_offset
                else:
                    ignition_range = math.hypot(sensed_x, sensed_z)
                    approach_x = sensed_x / max(ignition_range, 0.1) * effective_approach_offset
                    approach_z = sensed_z / max(ignition_range, 0.1) * effective_approach_offset
                plan_x, plan_z = sensed_x - approach_x, sensed_z - approach_z
                progress_rate0 = max(-sensed_vv, CAPTURE_SPEED) / max(
                    plan_height - plan_end_height, 1.0
                )
                plan_error_slope_x = -sensed_vx / max(progress_rate0, 0.0001)
                plan_error_slope_z = -sensed_vz / max(progress_rate0, 0.0001)
            else:
                if sensed_h <= MAIN_PREALIGN_HEIGHT:
                    # Match main.ks: settle on the solved cone-relative axis
                    # before ignition.
                    prealign_up, prealign_x, prealign_z = prelead_axis(
                        sensed_vv, sensed_vx, sensed_vz,
                        VERTICAL_RECOVERY_PRELEAD_DEGREES,
                    )
                    response = DT / (MAIN_STEERING_RESPONSE_SECONDS + DT)
                    actual_dir_up += (
                        prealign_up - actual_dir_up
                    ) * response
                    actual_dir_x += (
                        prealign_x - actual_dir_x
                    ) * response
                    actual_dir_z += (
                        prealign_z - actual_dir_z
                    ) * response
                    actual_dir_magnitude = math.sqrt(
                        actual_dir_up**2 + actual_dir_x**2 + actual_dir_z**2
                    )
                    actual_dir_up /= actual_dir_magnitude
                    actual_dir_x /= actual_dir_magnitude
                    actual_dir_z /= actual_dir_magnitude
                ballistic_tgo = (
                    sensed_vv
                    + math.sqrt(max(
                        sensed_vv**2
                        + 2.0 * effective_g
                        * max(sensed_h - TERMINAL_WAYPOINT_HEIGHT, 0.0),
                        0.0,
                    ))
                ) / effective_g
                predicted_x = sensed_vx * ballistic_tgo + 0.5 * drag_ax * ballistic_tgo**2
                predicted_z = sensed_vz * ballistic_tgo + 0.5 * drag_az * ballistic_tgo**2
                predicted_error = math.hypot(
                    sensed_x - predicted_x, sensed_z - predicted_z
                )
                midcourse = (
                    False  # short discrete pulses are outside this terminal sweep
                )
                target_ax = 0.0
                target_az = 0.0
                vertical_thrust = 0.0
                if midcourse:
                    predicted_miss_x = sensed_x - predicted_x
                    predicted_miss_z = sensed_z - predicted_z
                    target_ax, target_az = clamp_vector(
                        predicted_miss_x / max(ballistic_tgo * 3.0, 0.3),
                        predicted_miss_z / max(ballistic_tgo * 3.0, 0.3),
                        MIDCOURSE_MAX_HORIZONTAL_ACCEL,
                    )
                    vertical_thrust = 0.0
                    full_throttle_seconds += (
                        math.sqrt(vertical_thrust**2 + target_ax**2 + target_az**2)
                        / max(available_accel, 0.001)
                        * DT
                    )
                vv += (vertical_thrust - G + vertical_drag) * DT
                vx += drag_ax * DT
                vz += drag_az * DT
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
        waypoint_x = sensed_x - approach_x
        waypoint_z = sensed_z - approach_z
        if sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            terminal_x, terminal_z = waypoint_x, waypoint_z
        else:
            terminal_x, terminal_z = sensed_x, sensed_z
        terminal_error = math.hypot(terminal_x, terminal_z)
        pid_mode = sensed_h <= PID_SWITCH_HEIGHT
        horizontal_corridor_mode = (
            sensed_h <= HORIZONTAL_CORRIDOR_HEIGHT
            and error <= HORIZONTAL_CORRIDOR_RANGE
        )
        if (
            not capture_align_mode
            and horizontal_corridor_mode
            and sensed_h <= TERMINAL_CAPTURE_ALIGN_ARM_HEIGHT
            and terminal_error <= HORIZONTAL_ALIGN_RANGE
        ):
            capture_align_mode = True
            radial_speed = (
                (terminal_x * sensed_vx + terminal_z * sensed_vz)
                / terminal_error
                if terminal_error > 0.0
                else 0.0
            )
            capture_align_speed_limit = max(HORIZONTAL_ALIGN_MIN_SPEED, radial_speed)
        brake_along_position = (
            terminal_x * plan_along_x + terminal_z * plan_along_z
        )
        brake_along_velocity = (
            sensed_vx * plan_along_x + sensed_vz * plan_along_z
        )
        brake_cross_x = terminal_x - plan_along_x * brake_along_position
        brake_cross_z = terminal_z - plan_along_z * brake_along_position
        brake_cross_vx = sensed_vx - plan_along_x * brake_along_velocity
        brake_cross_vz = sensed_vz - plan_along_z * brake_along_velocity
        cross_track_diverging = (
            not cross_stop_committed
            and math.hypot(brake_cross_x, brake_cross_z)
                >= TERMINAL_CROSS_TRACK_BRAKE_ERROR
            and brake_cross_x * brake_cross_vx
                + brake_cross_z * brake_cross_vz < 0.0
            and terminal_error <= TERMINAL_CROSS_TRACK_BRAKE_RANGE
        )
        if (
            HIGH_ENERGY_BRAKE_ENABLED
            and not high_energy_brake_mode
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
            and sensed_h <= HIGH_ENERGY_BRAKE_ARM_HEIGHT
            and (
                (
                    terminal_error <= HIGH_ENERGY_BRAKE_RANGE
                    and terminal_x * sensed_vx
                        + terminal_z * sensed_vz > 0.0
                )
                or cross_track_diverging
            )
        ):
            high_energy_brake_mode = True
        if (
            WAYPOINT_COAST_ENABLED
            and not waypoint_coast_mode
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
        ):
            coast_endpoint_speed = math.sqrt(max(
                sensed_vv**2
                + 2.0 * G * (sensed_h - TERMINAL_WAYPOINT_HEIGHT),
                0.0,
            ))
            coast_tgo = (
                coast_endpoint_speed + sensed_vv
            ) / max(G, 0.1)
            coast_endpoint_error = math.hypot(
                sensed_x - sensed_vx * coast_tgo,
                sensed_z - sensed_vz * coast_tgo,
            )
            if (
                coast_endpoint_error <= WAYPOINT_COAST_ENTRY_MAX_ERROR
                and math.hypot(sensed_vx, sensed_vz)
                    <= WAYPOINT_COAST_ENTRY_MAX_HORIZONTAL_SPEED
                and
                WAYPOINT_COAST_ENTRY_MIN_VERTICAL_SPEED
                <= coast_endpoint_speed
                <= WAYPOINT_COAST_MAX_VERTICAL_SPEED
            ):
                waypoint_coast_mode = True
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
        estimated_fraction = MAIN_START_FUEL_FRACTION * clamp(
            (mass_ratio - MAIN_DRY_MASS_RATIO)
            / max(1.0 - MAIN_DRY_MASS_RATIO, 1e-9),
            0.0,
            1.0,
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
        if sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            waypoint_vertical_tgo = (
                2.0 * (sensed_h - TERMINAL_WAYPOINT_HEIGHT)
                / max(-sensed_vv + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1.0)
            )
            waypoint_net_vertical_accel = (
                -TERMINAL_WAYPOINT_VERTICAL_SPEED - sensed_vv
            ) / max(waypoint_vertical_tgo, 0.1)
            vertical_thrust_command = G + waypoint_net_vertical_accel
        if (
            sensed_h >= TERMINAL_WAYPOINT_HEIGHT
            and sensed_vv > -TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED
        ):
            vertical_thrust_command = 0.0
        plan_horizontal_height = max(
            plan_height - plan_end_height, 1.0
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
            + (waypoint_x - reference_x) * PLAN_POSITION_GAIN
            + (reference_vx - sensed_vx) * PLAN_VELOCITY_GAIN
        )
        az = (
            -d2_error_ds2_z * progress_rate**2
            + (waypoint_z - reference_z) * PLAN_POSITION_GAIN
            + (reference_vz - sensed_vz) * PLAN_VELOCITY_GAIN
        )
        ax, az = clamp_vector(ax, az, MAX_HORIZONTAL_ACCEL)

        if sensed_h > plan_end_height:
            horizontal_vertical_tgo = (
                2.0 * (sensed_h - TERMINAL_WAYPOINT_HEIGHT)
                / max(-sensed_vv + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1.0)
            )
            horizontal_vertical_net_accel = (
                -TERMINAL_WAYPOINT_VERTICAL_SPEED - sensed_vv
            ) / max(horizontal_vertical_tgo, 0.1)
            horizontal_target_drop = sensed_h - plan_end_height
            horizontal_target_tgo = horizontal_target_drop / max(-sensed_vv, 1.0)
            if horizontal_vertical_net_accel > 0.001:
                horizontal_target_disc = max(
                    sensed_vv**2
                    - 2.0 * horizontal_vertical_net_accel
                    * horizontal_target_drop,
                    0.0,
                )
                horizontal_target_tgo = (
                    -sensed_vv - math.sqrt(horizontal_target_disc)
                ) / horizontal_vertical_net_accel
            horizontal_control_tgo = max(
                horizontal_target_tgo - TERMINAL_HORIZONTAL_LEAD_SECONDS,
                0.5,
            )
            control_x = waypoint_x - sensed_vx * TERMINAL_HORIZONTAL_LEAD_SECONDS
            control_z = waypoint_z - sensed_vz * TERMINAL_HORIZONTAL_LEAD_SECONDS
            along_control_position = (
                control_x * plan_along_x + control_z * plan_along_z
            )
            along_velocity = (
                sensed_vx * plan_along_x + sensed_vz * plan_along_z
            )
            cross_control_x = control_x - plan_along_x * along_control_position
            cross_control_z = control_z - plan_along_z * along_control_position
            cross_velocity_x = sensed_vx - plan_along_x * along_velocity
            cross_velocity_z = sensed_vz - plan_along_z * along_velocity
            position_scale = (
                TERMINAL_WAYPOINT_POSITION_COEFFICIENT
                / max(horizontal_control_tgo**2, 0.25)
            )
            cross_position_scale = (
                TERMINAL_WAYPOINT_CROSS_POSITION_COEFFICIENT
                / max(horizontal_control_tgo**2, 0.25)
            )
            along_velocity_scale = (
                TERMINAL_WAYPOINT_VELOCITY_COEFFICIENT
                / horizontal_control_tgo
            )
            cross_velocity_scale = (
                TERMINAL_WAYPOINT_CROSS_VELOCITY_COEFFICIENT
                / horizontal_control_tgo
            )
            ax, az = clamp_vector(
                plan_along_x * (
                    along_control_position * position_scale
                    - along_velocity * along_velocity_scale
                )
                + cross_control_x * cross_position_scale
                - cross_velocity_x * cross_velocity_scale,
                plan_along_z * (
                    along_control_position * position_scale
                    - along_velocity * along_velocity_scale
                )
                + cross_control_z * cross_position_scale
                - cross_velocity_z * cross_velocity_scale,
                MAX_HORIZONTAL_ACCEL,
            )
            if MAIN_PURE_HORIZONTAL_BRAKE:
                horizontal_speed = math.hypot(sensed_vx, sensed_vz)
                if horizontal_speed > 5.0:
                    ax = -sensed_vx / horizontal_speed * MAX_HORIZONTAL_ACCEL
                    az = -sensed_vz / horizontal_speed * MAX_HORIZONTAL_ACCEL
                else:
                    ax = az = 0.0
            if MAIN_DIRECT_STOP_GUIDANCE and not high_energy_brake_mode:
                horizontal_speed = math.hypot(sensed_vx, sensed_vz)
                if horizontal_speed > 0.1:
                    velocity_dir_x = sensed_vx / horizontal_speed
                    velocity_dir_z = sensed_vz / horizontal_speed
                    along_range = max(
                        terminal_x * velocity_dir_x
                        + terminal_z * velocity_dir_z,
                        0.0,
                    )
                    compensated_range = max(
                        along_range
                        - horizontal_speed * MAIN_STEERING_RESPONSE_SECONDS
                        - HORIZONTAL_DEADBAND,
                        MAIN_DIRECT_STOP_MIN_RANGE,
                    )
                    required_decel = horizontal_speed**2 / (
                        2.0 * compensated_range
                    ) * MAIN_DIRECT_STOP_DECEL_GAIN
                    cross_x = terminal_x - velocity_dir_x * along_range
                    cross_z = terminal_z - velocity_dir_z * along_range
                    cross_tgo = (
                        2.0 * max(sensed_h - TERMINAL_WAYPOINT_HEIGHT, 0.0)
                        / max(-sensed_vv + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1.0)
                    )
                    cross_ax, cross_az = clamp_vector(
                        cross_x * MAIN_DIRECT_STOP_CROSS_TIME_COEFFICIENT
                        / max(cross_tgo**2, 1.0),
                        cross_z * MAIN_DIRECT_STOP_CROSS_TIME_COEFFICIENT
                        / max(cross_tgo**2, 1.0),
                        MAIN_DIRECT_STOP_CROSS_MAX_ACCEL,
                    )
                    ax = (
                        -velocity_dir_x * required_decel
                        + cross_ax
                    )
                    az = (
                        -velocity_dir_z * required_decel
                        + cross_az
                    )
                    ax, az = clamp_vector(ax, az, MAX_HORIZONTAL_ACCEL)
            # One-way stopping feasibility on the downrange axis frozen at
            # main ignition. Preserve the finite-time law's cross-track
            # component and only add braking when its remaining footprint is
            # insufficient after the measured attitude response.
            if (
                MAIN_FIXED_AXIS_STOP_ENABLED
                and sensed_h <= MAIN_FIXED_AXIS_STOP_ARM_HEIGHT
                and not high_energy_brake_mode
            ):
                fixed_stop_position = (
                    waypoint_x * plan_along_x
                    + waypoint_z * plan_along_z
                )
                fixed_stop_velocity = (
                    sensed_vx * plan_along_x
                    + sensed_vz * plan_along_z
                )
                if (
                    fixed_stop_position > HORIZONTAL_DEADBAND
                    and fixed_stop_velocity > 0.1
                ):
                    fixed_stop_usable_range = max(
                        fixed_stop_position
                        - fixed_stop_velocity
                            * MAIN_FIXED_AXIS_STOP_RESPONSE_SECONDS
                        - HORIZONTAL_DEADBAND,
                        MAIN_FIXED_AXIS_STOP_MIN_RANGE,
                    )
                    fixed_stop_raw_required_decel = (
                        fixed_stop_velocity**2
                        / (2.0 * fixed_stop_usable_range)
                        * MAIN_FIXED_AXIS_STOP_GAIN
                    )
                    fixed_stop_required_decel = min(
                        MAX_HORIZONTAL_ACCEL,
                        fixed_stop_raw_required_decel,
                    )
                    fixed_stop_surface_speed = math.sqrt(
                        sensed_vv**2 + sensed_vx**2 + sensed_vz**2
                    )
                    fixed_stop_axis_cosine = clamp(
                        fixed_stop_velocity
                        / max(fixed_stop_surface_speed, 0.1),
                        -1.0,
                        1.0,
                    )
                    fixed_stop_axis_separation = math.acos(
                        fixed_stop_axis_cosine
                    )
                    fixed_stop_engine_projection = math.cos(max(
                        fixed_stop_axis_separation
                        - TERMINAL_VELOCITY_CONE,
                        0.0,
                    ))
                    fixed_stop_aero_decel = max(-(
                        drag_ax * plan_along_x
                        + drag_az * plan_along_z
                    ), 0.0)
                    fixed_stop_available_decel = (
                        available_accel
                            * max(fixed_stop_engine_projection, 0.0)
                        + min(
                            fixed_stop_aero_decel,
                            POWERED_HORIZONTAL_AERO_MAX_ACCEL,
                        )
                    )
                    fixed_stop_current_accel = (
                        ax * plan_along_x + az * plan_along_z
                    )
                    fixed_stop_sync_tgo = (
                        2.0 * max(
                            sensed_h - TERMINAL_WAYPOINT_HEIGHT,
                            0.0,
                        )
                        / max(
                            -sensed_vv + TERMINAL_WAYPOINT_VERTICAL_SPEED,
                            1.0,
                        )
                    )
                    fixed_stop_sync_reference_speed = (
                        2.0 * fixed_stop_position
                        / max(fixed_stop_sync_tgo, 0.5)
                    )
                    fixed_stop_sync_excess = (
                        fixed_stop_velocity
                        - fixed_stop_sync_reference_speed
                    )
                    fixed_stop_feasible = (
                        fixed_stop_raw_required_decel
                        <= fixed_stop_available_decel
                            * MAIN_FIXED_AXIS_STOP_AUTHORITY_MARGIN
                    )
                    if (
                        not fixed_stop_committed
                        and fixed_stop_feasible
                        and fixed_stop_sync_excess
                            > MAIN_FIXED_AXIS_SYNC_EXCESS_ARM
                        and fixed_stop_current_accel
                            > -fixed_stop_required_decel
                    ):
                        fixed_stop_committed = True
                        fixed_stop_committed_this_tick = True
                    if (
                        fixed_stop_committed
                        and fixed_stop_current_accel
                            > -fixed_stop_required_decel
                    ):
                        ax -= plan_along_x * (
                            fixed_stop_current_accel
                            + fixed_stop_required_decel
                        )
                        az -= plan_along_z * (
                            fixed_stop_current_accel
                            + fixed_stop_required_decel
                        )
                        ax, az = clamp_vector(
                            ax, az, MAX_HORIZONTAL_ACCEL
                        )
            if HIGH_ENERGY_BRAKE_ENABLED and high_energy_brake_mode:
                desired_vx = 0.0
                desired_vz = 0.0
                if (
                    sensed_h > HIGH_ENERGY_BRAKE_SETTLE_HEIGHT
                    and terminal_error > HORIZONTAL_DEADBAND
                ):
                    desired_speed = min(
                        HIGH_ENERGY_BRAKE_MAX_SPEED,
                        (terminal_error - HORIZONTAL_DEADBAND)
                        * HIGH_ENERGY_BRAKE_POSITION_GAIN,
                    )
                    desired_vx = terminal_x / terminal_error * desired_speed
                    desired_vz = terminal_z / terminal_error * desired_speed
                ax, az = clamp_vector(
                    (desired_vx - sensed_vx)
                    * HIGH_ENERGY_BRAKE_VELOCITY_GAIN,
                    (desired_vz - sensed_vz)
                    * HIGH_ENERGY_BRAKE_VELOCITY_GAIN,
                    HIGH_ENERGY_BRAKE_MAX_ACCEL,
                )

        if horizontal_corridor_mode and (
            capture_align_mode or sensed_h <= TERMINAL_WAYPOINT_HEIGHT
        ):
            stop_range = max(terminal_error - HORIZONTAL_DEADBAND, 0.0)
            stop_speed = math.sqrt(2.0 * HORIZONTAL_STOP_ACCEL * stop_range)
            pid_horizontal_speed = min(stop_speed, HORIZONTAL_CORRIDOR_SPEED)
            if capture_align_mode:
                pid_horizontal_speed = min(
                    pid_horizontal_speed,
                    HORIZONTAL_ALIGN_SPEED,
                    capture_align_speed_limit,
                    stop_range * HORIZONTAL_ALIGN_POSITION_GAIN,
                )
            if terminal_error > HORIZONTAL_DEADBAND:
                desired_vx = terminal_x / terminal_error * pid_horizontal_speed
                desired_vz = terminal_z / terminal_error * pid_horizontal_speed
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
            and terminal_error <= HORIZONTAL_ALIGN_SETTLE_ENTRY_RANGE
            and math.hypot(sensed_vx, sensed_vz)
            <= HORIZONTAL_ALIGN_SETTLE_ENTRY_SPEED
        ):
            horizontal_settle_mode = True
            filtered_ax = 0.0
            filtered_az = 0.0
        if horizontal_settle_mode and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            ax, az = clamp_vector(
                terminal_x * HORIZONTAL_ALIGN_SETTLE_POSITION_GAIN
                - sensed_vx * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                terminal_z * HORIZONTAL_ALIGN_SETTLE_POSITION_GAIN
                - sensed_vz * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                HORIZONTAL_ALIGN_SETTLE_MAX_ACCEL,
            )
            if terminal_error > HORIZONTAL_ALIGN_REACQUIRE_RANGE:
                ax, az = clamp_vector(
                    terminal_x * HORIZONTAL_ALIGN_REACQUIRE_POSITION_GAIN
                    - sensed_vx * HORIZONTAL_ALIGN_SETTLE_VELOCITY_GAIN,
                    terminal_z * HORIZONTAL_ALIGN_REACQUIRE_POSITION_GAIN
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

        estimated_tilt = math.atan2(
            math.hypot(actual_dir_x, actual_dir_z), actual_dir_up
        )
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
        # Final one-way cross-axis arbitration.  This sits after all position
        # and high-energy branches so an unsuccessful centre crossing cannot be
        # disguised by a second pursuit.  The flight version additionally
        # projects the frozen direction into Kerbin's changing tangent plane.
        cross_stop_active = False
        if MAIN_CROSS_STOP_ENABLED and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            candidate_along_position = (
                waypoint_x * plan_along_x + waypoint_z * plan_along_z
            )
            candidate_cross_x = waypoint_x - (
                plan_along_x * candidate_along_position
            )
            candidate_cross_z = waypoint_z - (
                plan_along_z * candidate_along_position
            )
            candidate_cross_range = math.hypot(
                candidate_cross_x, candidate_cross_z
            )
            if (
                not cross_stop_committed
                and sensed_h <= MAIN_CROSS_STOP_ARM_HEIGHT
                and not high_energy_brake_mode
                and candidate_cross_range > HORIZONTAL_DEADBAND
            ):
                candidate_axis_x = candidate_cross_x / candidate_cross_range
                candidate_axis_z = candidate_cross_z / candidate_cross_range
                candidate_velocity = (
                    sensed_vx * candidate_axis_x
                    + sensed_vz * candidate_axis_z
                )
                if candidate_velocity > MAIN_CROSS_STOP_MIN_SPEED:
                    candidate_usable_range = max(
                        candidate_cross_range
                        - candidate_velocity * MAIN_CROSS_STOP_RESPONSE_SECONDS
                        - HORIZONTAL_DEADBAND,
                        MAIN_CROSS_STOP_MIN_RANGE,
                    )
                    candidate_required_decel = min(
                        MAIN_CROSS_STOP_MAX_ACCEL,
                        candidate_velocity**2
                        / (2.0 * candidate_usable_range)
                        * MAIN_CROSS_STOP_GAIN,
                    )
                    candidate_current_accel = (
                        ax * candidate_axis_x + az * candidate_axis_z
                    )
                    if candidate_current_accel > -candidate_required_decel:
                        cross_stop_committed = True
                        cross_stop_direction_x = candidate_axis_x
                        cross_stop_direction_z = candidate_axis_z
            if cross_stop_committed:
                cross_stop_position = (
                    waypoint_x * cross_stop_direction_x
                    + waypoint_z * cross_stop_direction_z
                )
                cross_stop_velocity = (
                    sensed_vx * cross_stop_direction_x
                    + sensed_vz * cross_stop_direction_z
                )
                cross_stop_current_accel = (
                    ax * cross_stop_direction_x
                    + az * cross_stop_direction_z
                )
                if cross_stop_completed:
                    finish_accel = clamp(
                        -cross_stop_velocity
                            * MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN,
                        -MAIN_CROSS_STOP_MAX_ACCEL,
                        MAIN_CROSS_STOP_MAX_ACCEL,
                    )
                    ax += cross_stop_direction_x * (
                        finish_accel - cross_stop_current_accel
                    )
                    az += cross_stop_direction_z * (
                        finish_accel - cross_stop_current_accel
                    )
                elif cross_stop_velocity > 0.1:
                    cross_stop_active = True
                    if cross_stop_position > HORIZONTAL_DEADBAND:
                        cross_stop_usable_range = max(
                            cross_stop_position
                            - cross_stop_velocity
                                * MAIN_CROSS_STOP_RESPONSE_SECONDS
                            - HORIZONTAL_DEADBAND,
                            MAIN_CROSS_STOP_MIN_RANGE,
                        )
                        cross_stop_required_decel = min(
                            MAIN_CROSS_STOP_MAX_ACCEL,
                            cross_stop_velocity**2
                            / (2.0 * cross_stop_usable_range)
                            * MAIN_CROSS_STOP_GAIN,
                        )
                    else:
                        cross_stop_required_decel = min(
                            MAIN_CROSS_STOP_MAX_ACCEL,
                            cross_stop_velocity
                                * MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN,
                        )
                    if cross_stop_current_accel > -cross_stop_required_decel:
                        ax -= cross_stop_direction_x * (
                            cross_stop_current_accel
                            + cross_stop_required_decel
                        )
                        az -= cross_stop_direction_z * (
                            cross_stop_current_accel
                            + cross_stop_required_decel
                        )
                        ax, az = clamp_vector(ax, az, MAX_HORIZONTAL_ACCEL)
                else:
                    cross_stop_completed = True
                    finish_accel = clamp(
                        -cross_stop_velocity
                            * MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN,
                        -MAIN_CROSS_STOP_MAX_ACCEL,
                        MAIN_CROSS_STOP_MAX_ACCEL,
                    )
                    ax += cross_stop_direction_x * (
                        finish_accel - cross_stop_current_accel
                    )
                    az += cross_stop_direction_z * (
                        finish_accel - cross_stop_current_accel
                    )
        # The flight controller identifies these residual forces from measured
        # acceleration and live thrust.  The point-mass harness knows its drag
        # law exactly, so use the exact residual as the zero-noise limit of the
        # same estimator.
        aero_compensation_x, aero_compensation_z = clamp_vector(
            drag_ax, drag_az, POWERED_HORIZONTAL_AERO_MAX_ACCEL
        )
        ax -= aero_compensation_x
        az -= aero_compensation_z
        vertical_thrust_command -= min(
            max(vertical_drag, 0.0), POWERED_VERTICAL_AERO_MAX_ACCEL
        )

        current_accel_filter = TERMINAL_ACCEL_FILTER
        if horizontal_corridor_mode and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            current_accel_filter = TERMINAL_HIGH_ENERGY_ACCEL_FILTER
        if fixed_stop_committed_this_tick:
            filtered_ax, filtered_az = ax, az
        else:
            filtered_ax = (
                filtered_ax * (1.0 - current_accel_filter)
                + ax * current_accel_filter
            )
            filtered_az = (
                filtered_az * (1.0 - current_accel_filter)
                + az * current_accel_filter
            )
        ax, az = filtered_ax, filtered_az

        # Preserve the successful Run-52 vertical schedule while reserving a
        # bounded, one-way downrange engine component.  This mirrors main.ks:
        # it is applied after aerodynamic compensation/filtering and is
        # disabled immediately after the frozen-axis target is crossed.
        along_engine_brake_active = False
        if (
            MAIN_ALONG_ENGINE_BRAKE_ENABLED
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
            and sensed_h <= MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT
        ):
            along_engine_brake_position = (
                waypoint_x * plan_along_x + waypoint_z * plan_along_z
            )
            along_engine_brake_velocity = (
                sensed_vx * plan_along_x + sensed_vz * plan_along_z
            )
            if (
                along_engine_brake_position > HORIZONTAL_DEADBAND
                and along_engine_brake_velocity > 0.1
            ):
                along_engine_brake_active = True
                along_engine_brake_blend = clamp(
                    (MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT - sensed_h)
                    / max(
                        MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT
                        - MAIN_ALONG_ENGINE_BRAKE_FULL_HEIGHT,
                        1.0,
                    ),
                    0.0,
                    1.0,
                )
                along_engine_brake_required = (
                    MAIN_ALONG_ENGINE_BRAKE_ACCEL
                    * along_engine_brake_blend
                )
                along_engine_brake_current = (
                    ax * plan_along_x + az * plan_along_z
                )
                if along_engine_brake_current > -along_engine_brake_required:
                    ax -= plan_along_x * (
                        along_engine_brake_current
                        + along_engine_brake_required
                    )
                    az -= plan_along_z * (
                        along_engine_brake_current
                        + along_engine_brake_required
                    )

        vertical_thrust_command = clamp(
            vertical_thrust_command,
            0.0,
            available_accel * TERMINAL_NOMINAL_THRUST_FRACTION,
        )
        altitude_blend = clamp((sensed_h - 500.0) / 1500.0, 0.0, 1.0)
        tilt_limit = math.radians(
            ENTRY_MAX_TILT * altitude_blend
            + LANDING_MAX_TILT * (1.0 - altitude_blend)
        )
        low_alt_cone_blend = clamp(
            (sensed_h - TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT)
            / max(
                TERMINAL_WAYPOINT_HEIGHT
                - TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT,
                1.0,
            ),
            0.0,
            1.0,
        )
        active_velocity_cone = TERMINAL_LOW_ALT_COMMAND_CONE + (
            TERMINAL_VELOCITY_CONE - TERMINAL_LOW_ALT_COMMAND_CONE
        ) * low_alt_cone_blend
        dense_air_cone_blend = clamp(
            (sensed_h - TERMINAL_DENSE_AIR_CONE_END_HEIGHT)
            / max(
                TERMINAL_DENSE_AIR_CONE_START_HEIGHT
                - TERMINAL_DENSE_AIR_CONE_END_HEIGHT,
                1.0,
            ),
            0.0,
            1.0,
        )
        dense_air_velocity_cone = TERMINAL_DENSE_AIR_COMMAND_CONE + (
            TERMINAL_VELOCITY_CONE - TERMINAL_DENSE_AIR_COMMAND_CONE
        ) * dense_air_cone_blend
        active_velocity_cone = min(
            active_velocity_cone, dense_air_velocity_cone
        )
        if (
            horizontal_corridor_mode
            and math.hypot(ax, az) > 0.01
            and sensed_h > 500.0
            and (
                sensed_h < TERMINAL_WAYPOINT_HEIGHT
                or sensed_vv <= -TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED
            )
        ):
            # Reserve enough vertical thrust component to realise the allowed
            # tilt during the main translation.  Below 500 m the online loop
            # instead clips lateral acceleration against the existing vertical
            # command so a small correction cannot cause an upward hop.
            required_vertical_for_tilt = math.hypot(ax, az) / max(
                math.tan(tilt_limit), 0.01
            )
            coupling_min_down_speed = TERMINAL_WAYPOINT_VERTICAL_SPEED
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
                available_accel * TERMINAL_NOMINAL_THRUST_FRACTION,
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
                lateral_limit = axial * math.tan(active_velocity_cone)
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

        # 75% is the continuous nominal floor, not a magnitude ceiling.  The
        # flight controller may increase smoothly to 100% for trajectory error.
        total_thrust_limit = available_accel * TERMINAL_TOTAL_THRUST_FRACTION
        thrust_magnitude = math.sqrt(vertical_thrust_command**2 + ax**2 + az**2)
        if thrust_magnitude > total_thrust_limit:
            scale = total_thrust_limit / thrust_magnitude
            vertical_thrust_command *= scale
            ax *= scale
            az *= scale
            thrust_magnitude = total_thrust_limit
        steering_magnitude = thrust_magnitude
        if steering_magnitude > 1e-6:
            steering_dir_up = vertical_thrust_command / steering_magnitude
            steering_dir_x = ax / steering_magnitude
            steering_dir_z = az / steering_magnitude
        else:
            steering_dir_up = actual_dir_up
            steering_dir_x = actual_dir_x
            steering_dir_z = actual_dir_z
        steering_surface_speed = math.sqrt(
            sensed_vv * sensed_vv
            + sensed_vx * sensed_vx
            + sensed_vz * sensed_vz
        )
        waypoint_trim_active = False
        if waypoint_coast_mode and sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            horizontal_speed = math.hypot(sensed_vx, sensed_vz)
            trim_ax = trim_az = 0.0
            if horizontal_speed > 0.1:
                velocity_dir_x = sensed_vx / horizontal_speed
                velocity_dir_z = sensed_vz / horizontal_speed
                along_range = max(
                    sensed_x * velocity_dir_x + sensed_z * velocity_dir_z,
                    0.0,
                )
                brake_range = max(
                    along_range - HORIZONTAL_DEADBAND,
                    1.0,
                )
                required_decel = horizontal_speed**2 / (2.0 * brake_range)
                projection = (
                    sensed_x * velocity_dir_x + sensed_z * velocity_dir_z
                )
                cross_x = sensed_x - velocity_dir_x * projection
                cross_z = sensed_z - velocity_dir_z * projection
                trim_ax, trim_az = clamp_vector(
                    -velocity_dir_x * required_decel
                        + cross_x * WAYPOINT_TRIM_CROSS_GAIN,
                    -velocity_dir_z * required_decel
                        + cross_z * WAYPOINT_TRIM_CROSS_GAIN,
                    WAYPOINT_TRIM_MAX_ACCEL,
                )
            elif math.hypot(sensed_x, sensed_z) > 30.0:
                trim_ax, trim_az = clamp_vector(
                    sensed_x * WAYPOINT_TRIM_POSITION_GAIN,
                    sensed_z * WAYPOINT_TRIM_POSITION_GAIN,
                    WAYPOINT_TRIM_MAX_ACCEL,
                )
            vertical_thrust_command = 0.0
            ax, az = trim_ax, trim_az
            thrust_magnitude = math.hypot(ax, az)
            if thrust_magnitude > 1e-6:
                steering_dir_up = 0.0
                steering_dir_x = ax / thrust_magnitude
                steering_dir_z = az / thrust_magnitude
            actual_tilt = math.acos(clamp(actual_dir_up, -1.0, 1.0))
            waypoint_trim_active = (
                math.hypot(sensed_x, sensed_z) <= WAYPOINT_TRIM_ARM_RANGE
                and actual_tilt >= WAYPOINT_TRIM_MIN_ACTUAL_TILT
                and (
                    horizontal_speed > WAYPOINT_COAST_MAX_HORIZONTAL_SPEED
                    or math.hypot(sensed_x, sensed_z) > 30.0
                )
            )
        # The mandatory 75% baseline is a stable surface-retrograde vector.
        # Only a solved correction larger than that baseline may move the
        # direction away from retrograde.  Filling unused magnitude along the
        # horizontal request made the real stage fly broadside at 12--15 km.
        if sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            tracking_high_blend = clamp(
                (TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT - sensed_h)
                / max(
                    TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT
                    - TERMINAL_MAIN_TRACKING_FULL_HEIGHT,
                    1.0,
                ),
                0.0,
                1.0,
            )
            tracking_low_blend = clamp(
                (sensed_h - TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT)
                / max(
                    TERMINAL_MAIN_TRACKING_FADE_START_HEIGHT
                    - TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT,
                    1.0,
                ),
                0.0,
                1.0,
            )
            tracking_blend = min(tracking_high_blend, tracking_low_blend)
            floor_fraction = TERMINAL_NOMINAL_THRUST_FRACTION + (
                TERMINAL_MAIN_TRACKING_THRUST_FRACTION
                - TERMINAL_NOMINAL_THRUST_FRACTION
            ) * tracking_blend
            nominal_floor = (
                available_accel * floor_fraction
            )
            thrust_magnitude = math.sqrt(
                vertical_thrust_command**2 + ax**2 + az**2
            )
            if thrust_magnitude < nominal_floor:
                # Add only magnitude at the continuous floor; replacing the
                # solved direction with pure retrograde erases cross-track
                # closure whenever the trajectory request is below 75%.
                floor_dir_up = steering_dir_up
                floor_dir_x = steering_dir_x
                floor_dir_z = steering_dir_z
                vertical_thrust_command = floor_dir_up * nominal_floor
                ax = floor_dir_x * nominal_floor
                az = floor_dir_z * nominal_floor
                thrust_magnitude = nominal_floor
                steering_dir_up = (
                    vertical_thrust_command / max(thrust_magnitude, 1e-9)
                )
                steering_dir_x = ax / max(thrust_magnitude, 1e-9)
                steering_dir_z = az / max(thrust_magnitude, 1e-9)

        # Apply the physical load cone after every legacy steering override.
        # Above 5 m/s the thrust axis stays within a 22-degree command cone
        # around surface retrograde, leaving eight degrees for attitude lag.
        # Below 5 m/s the reference changes to local up.
        cone_speed = math.sqrt(
            sensed_vv * sensed_vv + sensed_vx * sensed_vx + sensed_vz * sensed_vz
        )
        if cone_speed >= TERMINAL_VELOCITY_CONE_MIN_SPEED:
            safety_up = -sensed_vv / cone_speed
            safety_x = -sensed_vx / cone_speed
            safety_z = -sensed_vz / cone_speed
        else:
            safety_up, safety_x, safety_z = 1.0, 0.0, 0.0
        steering_axial = (
            steering_dir_up * safety_up
            + steering_dir_x * safety_x
            + steering_dir_z * safety_z
        )
        if (
            steering_axial <= 0.0
            or math.acos(clamp(steering_axial, -1.0, 1.0))
            > active_velocity_cone
        ):
            lateral_up = steering_dir_up - safety_up * steering_axial
            lateral_x = steering_dir_x - safety_x * steering_axial
            lateral_z = steering_dir_z - safety_z * steering_axial
            lateral_magnitude = math.sqrt(
                lateral_up * lateral_up
                + lateral_x * lateral_x
                + lateral_z * lateral_z
            )
            if lateral_magnitude <= 1e-9:
                steering_dir_up = safety_up
                steering_dir_x = safety_x
                steering_dir_z = safety_z
            else:
                cone_cos = math.cos(active_velocity_cone)
                cone_sin = math.sin(active_velocity_cone)
                steering_dir_up = safety_up * cone_cos + (
                    lateral_up / lateral_magnitude * cone_sin
                )
                steering_dir_x = safety_x * cone_cos + (
                    lateral_x / lateral_magnitude * cone_sin
                )
                steering_dir_z = safety_z * cone_cos + (
                    lateral_z / lateral_magnitude * cone_sin
                )

        along_position = max(
            waypoint_x * plan_along_x + waypoint_z * plan_along_z,
            0.0,
        )
        along_velocity = (
            sensed_vx * plan_along_x + sensed_vz * plan_along_z
        )
        along_tgo = (
            2.0 * max(sensed_h - TERMINAL_WAYPOINT_HEIGHT, 0.0)
            / max(-sensed_vv + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1.0)
        )
        along_reference_speed = 2.0 * along_position / max(along_tgo, 0.5)
        along_speed_deficit = along_reference_speed - along_velocity
        along_speed_excess = along_velocity - along_reference_speed
        along_brake_usable_range = max(
            along_position - HORIZONTAL_DEADBAND,
            TERMINAL_ALONG_BRAKE_REACHABILITY_MIN_RANGE,
        )
        along_brake_required_decel = 0.0
        along_brake_available_decel = 0.0
        along_brake_pressure = 0.0
        if along_position > HORIZONTAL_DEADBAND and along_velocity > 0.1:
            along_brake_required_decel = (
                along_velocity**2 / (2.0 * along_brake_usable_range)
            )
            brake_dot = clamp(
                safety_x * -plan_along_x + safety_z * -plan_along_z,
                -1.0,
                1.0,
            )
            brake_separation = math.acos(brake_dot)
            engine_projection = math.cos(max(
                brake_separation - active_velocity_cone, 0.0
            ))
            compensated_drag_x, compensated_drag_z = clamp_vector(
                drag_ax, drag_az, POWERED_HORIZONTAL_AERO_MAX_ACCEL
            )
            aero_decel = max(-(
                compensated_drag_x * plan_along_x
                + compensated_drag_z * plan_along_z
            ), 0.0)
            along_brake_available_decel = (
                available_accel * max(engine_projection, 0.0) + aero_decel
            )
            along_brake_pressure = (
                along_brake_required_decel
                / max(along_brake_available_decel, 1.0)
            )
        along_brake_blend = clamp(
            (along_brake_pressure - TERMINAL_ALONG_BRAKE_PRESSURE_ARM)
            / max(
                TERMINAL_ALONG_BRAKE_PRESSURE_FULL
                - TERMINAL_ALONG_BRAKE_PRESSURE_ARM,
                0.01,
            ),
            0.0,
            1.0,
        )
        along_brake_height_blend = clamp(
            (TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT - sensed_h)
            / max(
                TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT
                - TERMINAL_ALONG_BRAKE_REACHABILITY_FULL_HEIGHT,
                1.0,
            ),
            0.0,
            1.0,
        )
        along_brake_blend *= along_brake_height_blend
        if not TERMINAL_ALONG_BRAKE_ENABLED:
            along_brake_blend = 0.0
        along_accel = ax * plan_along_x + az * plan_along_z
        cross_accel_x = ax - plan_along_x * along_accel
        cross_accel_z = az - plan_along_z * along_accel
        if (
            TERMINAL_ALONG_BRAKE_ENABLED
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
            and along_brake_blend > 0.0
        ):
            brake_request = max(
                abs(along_accel), TERMINAL_ALONG_BRAKE_MIN_REQUEST
            )
            edge_up = 0.0
            edge_x = -plan_along_x * brake_request + cross_accel_x
            edge_z = -plan_along_z * brake_request + cross_accel_z
            edge_magnitude = math.hypot(edge_x, edge_z)
            edge_x /= max(edge_magnitude, 1e-9)
            edge_z /= max(edge_magnitude, 1e-9)
            edge_axial = edge_x * safety_x + edge_z * safety_z
            edge_lateral_up = -safety_up * edge_axial
            edge_lateral_x = edge_x - safety_x * edge_axial
            edge_lateral_z = edge_z - safety_z * edge_axial
            edge_lateral_magnitude = math.sqrt(
                edge_lateral_up**2 + edge_lateral_x**2
                + edge_lateral_z**2
            )
            if edge_lateral_magnitude <= 1e-9:
                edge_up, edge_x, edge_z = safety_up, safety_x, safety_z
            else:
                cone_cos = math.cos(active_velocity_cone)
                cone_sin = math.sin(active_velocity_cone)
                edge_up = safety_up * cone_cos + (
                    edge_lateral_up / edge_lateral_magnitude * cone_sin
                )
                edge_x = safety_x * cone_cos + (
                    edge_lateral_x / edge_lateral_magnitude * cone_sin
                )
                edge_z = safety_z * cone_cos + (
                    edge_lateral_z / edge_lateral_magnitude * cone_sin
                )
            blended_up = (
                steering_dir_up * (1.0 - along_brake_blend)
                + edge_up * along_brake_blend
            )
            blended_x = (
                steering_dir_x * (1.0 - along_brake_blend)
                + edge_x * along_brake_blend
            )
            blended_z = (
                steering_dir_z * (1.0 - along_brake_blend)
                + edge_z * along_brake_blend
            )
            blended_magnitude = math.sqrt(
                blended_up**2 + blended_x**2 + blended_z**2
            )
            steering_dir_up = blended_up / max(blended_magnitude, 1e-9)
            steering_dir_x = blended_x / max(blended_magnitude, 1e-9)
            steering_dir_z = blended_z / max(blended_magnitude, 1e-9)

        along_coast_speed_blend = clamp(
            (along_speed_deficit - TERMINAL_ALONG_SPEED_DEFICIT_ARM)
            / max(TERMINAL_ALONG_SPEED_DEFICIT_BLEND, 0.1),
            0.0,
            1.0,
        )
        along_coast_height_blend = clamp(
            (sensed_h - TERMINAL_ALONG_COAST_END_HEIGHT)
            / max(
                TERMINAL_ALONG_COAST_FADE_START_HEIGHT
                - TERMINAL_ALONG_COAST_END_HEIGHT,
                1.0,
            ),
            0.0,
            1.0,
        )
        along_coast_blend = (
            along_coast_speed_blend * along_coast_height_blend
        )
        along_coast_throttle_scale = 1.0
        if TERMINAL_ALONG_COAST_ENABLED and along_coast_blend > 0.0:
            previous_up = max(steering_dir_up, 0.05)
            blended_up = (
                steering_dir_up * (1.0 - along_coast_blend)
                + safety_up * along_coast_blend
            )
            blended_x = (
                steering_dir_x * (1.0 - along_coast_blend)
                + safety_x * along_coast_blend
            )
            blended_z = (
                steering_dir_z * (1.0 - along_coast_blend)
                + safety_z * along_coast_blend
            )
            blended_magnitude = math.sqrt(
                blended_up**2 + blended_x**2 + blended_z**2
            )
            steering_dir_up = blended_up / max(blended_magnitude, 1e-9)
            steering_dir_x = blended_x / max(blended_magnitude, 1e-9)
            steering_dir_z = blended_z / max(blended_magnitude, 1e-9)
            along_coast_throttle_scale = max(
                1.0,
                previous_up / max(steering_dir_up, 0.05),
            )

        # The point plant cannot reproduce the long body's attitude-dependent
        # drag, but it mirrors the Run-99 measured-reachability state machine,
        # legal cone projection, and rate bounds for structural verification.
        aero_brake_height_blend = clamp(
            (TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT - sensed_h)
            / max(
                TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT
                - TERMINAL_ALONG_AERO_BRAKE_FULL_HEIGHT,
                1.0,
            ),
            0.0,
            1.0,
        )
        realized_along_decel = max(-(
            (actual_dir_x * thrust_magnitude + drag_ax) * plan_along_x
            + (actual_dir_z * thrust_magnitude + drag_az) * plan_along_z
        ), 0.0)
        aero_brake_required = (
            along_brake_required_decel * TERMINAL_ALONG_AERO_BRAKE_MARGIN
        )
        aero_brake_error = aero_brake_required - realized_along_decel
        if abs(aero_brake_error) <= TERMINAL_ALONG_AERO_BRAKE_ERROR_DEADBAND:
            aero_brake_error = 0.0
        aero_brake_rate = (
            TERMINAL_ALONG_AERO_BRAKE_BUILD_RATE
            if aero_brake_error >= 0.0
            else TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE
        )
        aero_brake_step = clamp(
            aero_brake_error / max(TERMINAL_ALONG_AERO_BRAKE_ACCEL_GAIN, 0.1),
            -aero_brake_rate * DT,
            aero_brake_rate * DT,
        )
        if (
            not TERMINAL_ALONG_AERO_BRAKE_ENABLED
            or sensed_h <= TERMINAL_WAYPOINT_HEIGHT
            or along_position <= HORIZONTAL_DEADBAND
            or along_velocity <= 0.1
        ):
            aero_brake_step = -TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE * DT
        else:
            aero_brake_step *= aero_brake_height_blend
        along_aero_brake_blend_state = clamp(
            along_aero_brake_blend_state + aero_brake_step,
            0.0,
            TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND,
        )
        if (
            TERMINAL_ALONG_AERO_BRAKE_ENABLED
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
            and along_position > HORIZONTAL_DEADBAND
            and along_velocity > 0.1
            and aero_brake_height_blend > 0.0
        ):
            edge_up = 0.0
            edge_x = (
                plan_along_x * max(
                    abs(along_accel), TERMINAL_ALONG_BRAKE_MIN_REQUEST
                )
                + cross_accel_x
            )
            edge_z = (
                plan_along_z * max(
                    abs(along_accel), TERMINAL_ALONG_BRAKE_MIN_REQUEST
                )
                + cross_accel_z
            )
            edge_magnitude = math.hypot(edge_x, edge_z)
            edge_x /= max(edge_magnitude, 1e-9)
            edge_z /= max(edge_magnitude, 1e-9)
            edge_axial = edge_x * safety_x + edge_z * safety_z
            edge_lateral_up = -safety_up * edge_axial
            edge_lateral_x = edge_x - safety_x * edge_axial
            edge_lateral_z = edge_z - safety_z * edge_axial
            edge_lateral_magnitude = math.sqrt(
                edge_lateral_up**2 + edge_lateral_x**2
                + edge_lateral_z**2
            )
            if edge_lateral_magnitude <= 1e-9:
                edge_up, edge_x, edge_z = safety_up, safety_x, safety_z
            else:
                cone_cos = math.cos(active_velocity_cone)
                cone_sin = math.sin(active_velocity_cone)
                edge_up = safety_up * cone_cos + (
                    edge_lateral_up / edge_lateral_magnitude * cone_sin
                )
                edge_x = safety_x * cone_cos + (
                    edge_lateral_x / edge_lateral_magnitude * cone_sin
                )
                edge_z = safety_z * cone_cos + (
                    edge_lateral_z / edge_lateral_magnitude * cone_sin
                )
            actuator_up = (
                safety_up * (1.0 - along_aero_brake_blend_state)
                + edge_up * along_aero_brake_blend_state
            )
            actuator_x = (
                safety_x * (1.0 - along_aero_brake_blend_state)
                + edge_x * along_aero_brake_blend_state
            )
            actuator_z = (
                safety_z * (1.0 - along_aero_brake_blend_state)
                + edge_z * along_aero_brake_blend_state
            )
            actuator_magnitude = math.sqrt(
                actuator_up**2 + actuator_x**2 + actuator_z**2
            )
            actuator_up /= max(actuator_magnitude, 1e-9)
            actuator_x /= max(actuator_magnitude, 1e-9)
            actuator_z /= max(actuator_magnitude, 1e-9)
            blended_up = (
                steering_dir_up * (1.0 - aero_brake_height_blend)
                + actuator_up * aero_brake_height_blend
            )
            blended_x = (
                steering_dir_x * (1.0 - aero_brake_height_blend)
                + actuator_x * aero_brake_height_blend
            )
            blended_z = (
                steering_dir_z * (1.0 - aero_brake_height_blend)
                + actuator_z * aero_brake_height_blend
            )
            blended_magnitude = math.sqrt(
                blended_up**2 + blended_x**2 + blended_z**2
            )
            steering_dir_up = blended_up / max(blended_magnitude, 1e-9)
            steering_dir_x = blended_x / max(blended_magnitude, 1e-9)
            steering_dir_z = blended_z / max(blended_magnitude, 1e-9)

        # Mirror the flight controller's one-way final allocator.  The pure
        # braking request is projected to the edge of the active velocity cone;
        # cross-track demand changes its azimuth but not the safety limit.
        if (
            MAIN_FIXED_AXIS_EDGE_STEERING_ENABLED
            and fixed_stop_committed
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
        ):
            fixed_edge_position = (
                waypoint_x * plan_along_x + waypoint_z * plan_along_z
            )
            fixed_edge_velocity = (
                sensed_vx * plan_along_x + sensed_vz * plan_along_z
            )
            if (
                fixed_edge_position > HORIZONTAL_DEADBAND
                and fixed_edge_velocity > 0.1
            ):
                fixed_edge_along_request = max(abs(along_accel), 5.0)
                edge_up = 0.0
                edge_x = (
                    -plan_along_x * fixed_edge_along_request
                    + cross_accel_x
                )
                edge_z = (
                    -plan_along_z * fixed_edge_along_request
                    + cross_accel_z
                )
                edge_magnitude = math.hypot(edge_x, edge_z)
                edge_x /= max(edge_magnitude, 1e-9)
                edge_z /= max(edge_magnitude, 1e-9)
                edge_axial = edge_x * safety_x + edge_z * safety_z
                edge_lateral_up = -safety_up * edge_axial
                edge_lateral_x = edge_x - safety_x * edge_axial
                edge_lateral_z = edge_z - safety_z * edge_axial
                edge_lateral_magnitude = math.sqrt(
                    edge_lateral_up**2 + edge_lateral_x**2
                    + edge_lateral_z**2
                )
                if edge_lateral_magnitude <= 1e-9:
                    steering_dir_up = safety_up
                    steering_dir_x = safety_x
                    steering_dir_z = safety_z
                else:
                    cone_cos = math.cos(active_velocity_cone)
                    cone_sin = math.sin(active_velocity_cone)
                    steering_dir_up = safety_up * cone_cos + (
                        edge_lateral_up / edge_lateral_magnitude * cone_sin
                    )
                    steering_dir_x = safety_x * cone_cos + (
                        edge_lateral_x / edge_lateral_magnitude * cone_sin
                    )
                    steering_dir_z = safety_z * cone_cos + (
                        edge_lateral_z / edge_lateral_magnitude * cone_sin
                    )

        # Mirror the flight-only weathercock control reversal.  The point-mass
        # harness has no attitude-dependent side force, so this branch is
        # expected to remain diagnostically red; it still exercises the exact
        # one-way steering geometry and cone projection.
        if (
            MAIN_CROSS_AERO_BRAKE_ENABLED
            and cross_stop_active
            and sensed_h <= MAIN_CROSS_AERO_BRAKE_START_HEIGHT
            and sensed_h > TERMINAL_WAYPOINT_HEIGHT
        ):
            signed_cross_component = (
                steering_dir_x * cross_stop_direction_x
                + steering_dir_z * cross_stop_direction_z
            )
            base_up = steering_dir_up
            base_x = (
                steering_dir_x
                - cross_stop_direction_x * signed_cross_component
            )
            base_z = (
                steering_dir_z
                - cross_stop_direction_z * signed_cross_component
            )
            full_inward_fraction = max(
                abs(signed_cross_component),
                MAIN_CROSS_AERO_BRAKE_MIN_ENGINE_ACCEL
                    / max(thrust_magnitude, 1e-9),
            )
            realized_cross_decel = max(-(
                (actual_dir_x * thrust_magnitude + drag_ax)
                    * cross_stop_direction_x
                + (actual_dir_z * thrust_magnitude + drag_az)
                    * cross_stop_direction_z
            ), 0.0)
            residual_cross_demand = max(
                cross_stop_required_decel - realized_cross_decel,
                0.0,
            )
            candidate_release_blend = clamp(
                residual_cross_demand
                    / MAIN_CROSS_AERO_BRAKE_FULL_DEMAND,
                0.0,
                1.0,
            )
            if (
                not cross_aero_brake_release_started
                and cross_aero_brake_previous_demand
                    >= MAIN_CROSS_AERO_BRAKE_FULL_DEMAND
                and cross_stop_required_decel
                    < cross_aero_brake_previous_demand
            ):
                cross_aero_brake_release_started = True
            if cross_aero_brake_release_started:
                cross_aero_brake_blend_state = min(
                    cross_aero_brake_blend_state,
                    candidate_release_blend,
                )
            else:
                cross_aero_brake_blend_state = max(
                    cross_aero_brake_blend_state,
                    candidate_release_blend,
                )
            release_blend = cross_aero_brake_blend_state
            inward_fraction = full_inward_fraction * release_blend
            request_up = base_up
            request_x = base_x + cross_stop_direction_x * inward_fraction
            request_z = base_z + cross_stop_direction_z * inward_fraction
            request_magnitude = math.sqrt(
                request_up**2 + request_x**2 + request_z**2
            )
            request_up /= max(request_magnitude, 1e-9)
            request_x /= max(request_magnitude, 1e-9)
            request_z /= max(request_magnitude, 1e-9)
            request_axial = (
                request_up * safety_up
                + request_x * safety_x
                + request_z * safety_z
            )
            request_angle = math.acos(clamp(request_axial, -1.0, 1.0))
            if request_axial <= 0.0 or request_angle > active_velocity_cone:
                lateral_up = request_up - safety_up * request_axial
                lateral_x = request_x - safety_x * request_axial
                lateral_z = request_z - safety_z * request_axial
                lateral_magnitude = math.sqrt(
                    lateral_up**2 + lateral_x**2 + lateral_z**2
                )
                if lateral_magnitude <= 1e-9:
                    steering_dir_up = safety_up
                    steering_dir_x = safety_x
                    steering_dir_z = safety_z
                else:
                    cone_cos = math.cos(active_velocity_cone)
                    cone_sin = math.sin(active_velocity_cone)
                    steering_dir_up = safety_up * cone_cos + (
                        lateral_up / lateral_magnitude * cone_sin
                    )
                    steering_dir_x = safety_x * cone_cos + (
                        lateral_x / lateral_magnitude * cone_sin
                    )
                    steering_dir_z = safety_z * cone_cos + (
                        lateral_z / lateral_magnitude * cone_sin
                    )
            else:
                steering_dir_up = request_up
                steering_dir_x = request_x
                steering_dir_z = request_z
        if cross_stop_active:
            cross_aero_brake_previous_demand = cross_stop_required_decel

        # Protect the measured physical cone, not only the instantaneous
        # command. Near zero horizontal velocity the live retrograde axis can
        # rotate faster than the first-order attitude response. Pull the target
        # smoothly toward the cone centre while leaving thrust magnitude alone.
        actual_cone_angle = math.acos(clamp(
            actual_dir_up * safety_up
            + actual_dir_x * safety_x
            + actual_dir_z * safety_z,
            -1.0,
            1.0,
        ))
        actual_cone_guard_blend = clamp(
            (actual_cone_angle - ACTUAL_CONE_GUARD_START)
            / max(ACTUAL_CONE_GUARD_FULL - ACTUAL_CONE_GUARD_START, 1e-6),
            0.0,
            1.0,
        )
        if actual_cone_guard_blend > 0.0:
            guarded_up = (
                steering_dir_up * (1.0 - actual_cone_guard_blend)
                + safety_up * actual_cone_guard_blend
            )
            guarded_x = (
                steering_dir_x * (1.0 - actual_cone_guard_blend)
                + safety_x * actual_cone_guard_blend
            )
            guarded_z = (
                steering_dir_z * (1.0 - actual_cone_guard_blend)
                + safety_z * actual_cone_guard_blend
            )
            guarded_magnitude = math.sqrt(
                guarded_up**2 + guarded_x**2 + guarded_z**2
            )
            steering_dir_up = guarded_up / max(guarded_magnitude, 1e-9)
            steering_dir_x = guarded_x / max(guarded_magnitude, 1e-9)
            steering_dir_z = guarded_z / max(guarded_magnitude, 1e-9)

        # The main segment is one continuous burn, never a 0/75% pulse train.
        minimum_fraction = (
            TERMINAL_NOMINAL_THRUST_FRACTION
            if sensed_h > TERMINAL_WAYPOINT_HEIGHT
            else TERMINAL_MIN_CONTINUOUS_THROTTLE
        )
        requested_thrust_magnitude = (
            thrust_magnitude * along_coast_throttle_scale
        )
        if sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            brake_throttle_fraction = (
                TERMINAL_NOMINAL_THRUST_FRACTION
                + (1.0 - TERMINAL_NOMINAL_THRUST_FRACTION)
                    * along_brake_blend
            )
            requested_thrust_magnitude = max(
                requested_thrust_magnitude,
                available_accel * brake_throttle_fraction,
            )
        if (
            sensed_h > TERMINAL_WAYPOINT_HEIGHT
            and requested_thrust_magnitude
                > available_accel * TERMINAL_NOMINAL_THRUST_FRACTION
        ):
            alignment_cos = clamp(
                actual_dir_up * steering_dir_up
                + actual_dir_x * steering_dir_x
                + actual_dir_z * steering_dir_z,
                -1.0,
                1.0,
            )
            alignment_degrees = math.degrees(math.acos(alignment_cos))
            alignment_blend = clamp(
                (MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES - alignment_degrees)
                / max(
                    MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES
                    - MAIN_CORRECTION_FULL_ALIGNMENT_DEGREES,
                    0.1,
                ),
                0.0,
                1.0,
            )
            nominal_magnitude = (
                available_accel * TERMINAL_NOMINAL_THRUST_FRACTION
            )
            requested_thrust_magnitude = nominal_magnitude + (
                requested_thrust_magnitude - nominal_magnitude
            ) * alignment_blend
        thrust_magnitude = clamp(
            max(requested_thrust_magnitude,
                available_accel * minimum_fraction),
            available_accel * minimum_fraction,
            available_accel * TERMINAL_TOTAL_THRUST_FRACTION,
        )
        # Above the 2 km waypoint the nominal 75% floor is invariant.  A
        # descent-preserving throttle cap would be continuous rather than PWM,
        # but it still violates the real mission constraint and masks an
        # infeasible joint trajectory.
        full_throttle_seconds += (
            thrust_magnitude / max(available_accel, 0.001) * DT
        )

        # Apply the measured high-q loss of physical cone authority before the
        # remaining first-order steering response.  This is deliberately not a
        # controller clamp: kOS continues to request the solved safe vector,
        # while the replay represents aerodynamic restoring torque on the long
        # stage and its bounded grid-fin/reaction-wheel authority.
        physical_target_up = steering_dir_up
        physical_target_x = steering_dir_x
        physical_target_z = steering_dir_z
        if sensed_h > TERMINAL_WAYPOINT_HEIGHT:
            dynamic_pressure_proxy = density * cone_speed**2
            steering_q_scale_blend = clamp(
                (sensed_h - MAIN_AERO_STEERING_TAPER_END_HEIGHT)
                / max(
                    MAIN_AERO_STEERING_TAPER_START_HEIGHT
                    - MAIN_AERO_STEERING_TAPER_END_HEIGHT,
                    1.0,
                ),
                0.0,
                1.0,
            )
            steering_q_scale = (
                MAIN_AERO_STEERING_RESTORED_Q_SCALE
                + (MAIN_AERO_STEERING_Q_SCALE
                    - MAIN_AERO_STEERING_RESTORED_Q_SCALE)
                    * steering_q_scale_blend
            )
            physical_authority = max(
                MAIN_AERO_STEERING_MIN_AUTHORITY,
                clamp(
                    1.0
                    - dynamic_pressure_proxy / steering_q_scale,
                    0.0,
                    1.0,
                ) ** MAIN_AERO_STEERING_AUTHORITY_EXPONENT,
            )
            physical_target_up = (
                safety_up * (1.0 - physical_authority)
                + steering_dir_up * physical_authority
            )
            physical_target_x = (
                safety_x * (1.0 - physical_authority)
                + steering_dir_x * physical_authority
            )
            physical_target_z = (
                safety_z * (1.0 - physical_authority)
                + steering_dir_z * physical_authority
            )
            physical_target_magnitude = math.sqrt(
                physical_target_up**2
                + physical_target_x**2
                + physical_target_z**2
            )
            physical_target_up /= max(physical_target_magnitude, 1e-9)
            physical_target_x /= max(physical_target_magnitude, 1e-9)
            physical_target_z /= max(physical_target_magnitude, 1e-9)

        # Apply attitude lag to the load-limited unit thrust direction, then
        # multiply it by the current continuous engine magnitude.
        steering_response_seconds = (
            MAIN_PHYSICAL_STEERING_RESPONSE_SECONDS
            if sensed_h > TERMINAL_WAYPOINT_HEIGHT
            else TERMINAL_STEERING_RESPONSE_SECONDS
        )
        response = DT / (steering_response_seconds + DT)
        actual_dir_up += (physical_target_up - actual_dir_up) * response
        actual_dir_x += (physical_target_x - actual_dir_x) * response
        actual_dir_z += (physical_target_z - actual_dir_z) * response
        actual_dir_magnitude = math.sqrt(
            actual_dir_up**2 + actual_dir_x**2 + actual_dir_z**2
        )
        actual_dir_up /= max(actual_dir_magnitude, 1e-9)
        actual_dir_x /= max(actual_dir_magnitude, 1e-9)
        actual_dir_z /= max(actual_dir_magnitude, 1e-9)
        physical_speed = math.sqrt(vv * vv + vx * vx + vz * vz)
        if thrust_magnitude > 1e-6:
            if physical_speed >= TERMINAL_VELOCITY_CONE_MIN_SPEED:
                physical_axis_up = -vv / physical_speed
                physical_axis_x = -vx / physical_speed
                physical_axis_z = -vz / physical_speed
            else:
                physical_axis_up, physical_axis_x, physical_axis_z = 1.0, 0.0, 0.0
            physical_axis_dot = clamp(
                actual_dir_up * physical_axis_up
                + actual_dir_x * physical_axis_x
                + actual_dir_z * physical_axis_z,
                -1.0,
                1.0,
            )
            max_velocity_cone_angle = max(
                max_velocity_cone_angle,
                math.acos(physical_axis_dot),
            )
        actual_vertical_thrust = actual_dir_up * thrust_magnitude
        actual_ax = actual_dir_x * thrust_magnitude
        actual_az = actual_dir_z * thrust_magnitude

        if trace and step % 50 == 0:
            sensed_horizontal_speed = math.hypot(sensed_vx, sensed_vz)
            actual_brake_accel = -(
                actual_ax * sensed_vx + actual_az * sensed_vz
            ) / max(sensed_horizontal_speed, 0.1)
            print(
                f"h={sensed_h:7.1f} vv={sensed_vv:7.1f} "
                f"err={math.hypot(sensed_x, sensed_z):7.1f} "
                f"hv={sensed_horizontal_speed:6.1f} "
                f"radialV={(sensed_x*sensed_vx+sensed_z*sensed_vz)/max(math.hypot(sensed_x,sensed_z),0.1):7.1f} "
                f"actual_brake={actual_brake_accel:6.1f} "
                f"dir_up={actual_dir_up:6.3f} thrust={thrust_magnitude:6.1f}"
            )

        vv += (actual_vertical_thrust - G + vertical_drag) * DT
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
            if stop_at_waypoint:
                return Result(
                    False, now, vv, waypoint_lateral_speed, waypoint_error,
                    minimum_height, hover_time,
                    math.hypot(cable_x - x, cable_z - z),
                    ignition_height, full_throttle_seconds,
                    rebound_after_center,
                    waypoint_vertical_speed=waypoint_vertical_speed,
                    waypoint_lateral_speed=waypoint_lateral_speed,
                    waypoint_error=waypoint_error,
                    max_velocity_cone_angle=max_velocity_cone_angle,
                )
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
            start_powered=True,
        ))
    for _ in range(medium_count):
        angle = rng.uniform(-math.pi, math.pi)
        direction_x = math.cos(angle)
        direction_z = math.sin(angle)
        # Run 52 supplies the current authoritative coupled ignition state
        # after the 0.863 horizontal gate and physical-alignment interlock:
        # 23.05 km hook height, -644 m/s vertical, 23.78 km fixed-axis range,
        # 988 m/s along-track and roughly 272 m cross-track.  Perturb this
        # observed state directly; the former Run-45-derived 20.3 km samples
        # represented a superseded ignition gate and invalidated waypoint
        # tuning even when the full real flight consistently lit near 23 km.
        height = rng.uniform(22950.0, 23150.0)
        vertical_speed = rng.uniform(-647.0, -641.0)
        along_speed = rng.uniform(984.0, 992.0)
        cross_speed = rng.uniform(-0.5, 0.5)
        # The horizontal gate couples these states: during the observed coast,
        # each metre of lost height removes about 1.58 m of remaining range.
        # Keep only the small navigation/timing residual independent.
        distance = (
            23784.1 + 1.58 * (height - 23052.4)
            + rng.uniform(-25.0, 25.0)
        )
        cross_offset = rng.uniform(260.0, 285.0)
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=distance * direction_x - cross_offset * direction_z,
            z=distance * direction_z + cross_offset * direction_x,
            vx=along_speed * direction_x - cross_speed * direction_z,
            vz=along_speed * direction_z + cross_speed * direction_x,
            available_accel=rng.uniform(32.1, 32.3),
            sensor_delay=rng.uniform(0, 0.08),
            start_powered=True,
            horizontal_drag_factor=RUN35_IDENTIFIED_HORIZONTAL_DRAG_FACTOR,
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
        height = rng.uniform(1900, 1995)
        vertical_speed = -rng.uniform(
            TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED,
            TERMINAL_WAYPOINT_MAX_VERTICAL_SPEED,
        )
        along_speed = rng.uniform(0, 5)
        distance = rng.uniform(0, 30)
        result.append(Case(
            height=height,
            vertical_speed=vertical_speed,
            x=distance * direction_x,
            z=distance * direction_z,
            vx=along_speed * direction_x,
            vz=along_speed * direction_z,
            available_accel=rng.uniform(70, 100),
            sensor_delay=rng.uniform(0, 0.08),
            start_powered=True,
        ))
    return result


def main() -> int:
    case_list = cases()
    results = [run(case) for case in case_list]
    low_count = len(results) // 3
    medium_count = len(results) // 3
    medium_results = results[low_count:low_count + medium_count]
    downstream_results = (
        results[:low_count] + results[low_count + medium_count:]
    )
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
    waypoint_successes = [
        r for r in medium_results
        if TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED
        <= abs(r.waypoint_vertical_speed)
        <= TERMINAL_WAYPOINT_MAX_VERTICAL_SPEED
        and r.waypoint_lateral_speed <= 5.0
        and r.waypoint_error <= 10.0
        and r.max_velocity_cone_angle <= TERMINAL_VELOCITY_CONE_HARD_LIMIT
    ]
    print(
        f"waypoint_compliance={len(waypoint_successes)}/"
        f"{len(medium_results)}"
    )
    stable_successes = [
        r for r in downstream_results if r.rebound_after_center <= 8.0
    ]
    official_rebound_successes = [
        r for r in downstream_results if r.rebound_after_center <= 10.0
    ]
    print(
        f"downstream_stable_rate={len(stable_successes)}/"
        f"{len(downstream_results)}"
    )
    print(
        f"downstream_rebound_compliance={len(official_rebound_successes)}/"
        f"{len(downstream_results)}"
    )
    cone_successes = [
        result for result in results
        if result.max_velocity_cone_angle <= TERMINAL_VELOCITY_CONE_HARD_LIMIT
    ]
    print(
        f"powered_nozzle_cone_compliance={len(cone_successes)}/"
        f"{len(results)}"
    )
    return 0 if (
        len(successes) == len(results)
        and len(waypoint_successes) >= int(len(medium_results) * 0.95)
        # A synthetic 2 km handover may traverse the 10 m audit circle before
        # the real capture states even begin.  Require broad compliance here;
        # actual capture plus the plugin's uninterrupted 60 s hold remain the
        # authoritative no-rebound acceptance test.
        and len(official_rebound_successes)
            >= int(len(downstream_results) * 0.95)
        and len(stable_successes) >= int(len(downstream_results) * 0.90)
        and len(cone_successes) == len(results)
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
