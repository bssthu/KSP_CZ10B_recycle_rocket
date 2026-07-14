// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
SET BOOSTER_ENGINE_PART_NAME TO "liquidEngineMainsail.v2".
// The stock Mainsail is limited to the former 1200 kN during ascent. After
// staging its full 1500 kN envelope becomes available to recovery guidance.
SET ASCENT_ENGINE_THRUST_LIMIT TO 80.
// Separation is fuel-budget driven: the first stage keeps at most one fifth of
// its original propellant for recovery.  Altitude is only a lower safety gate.
SET ASCENT_RESERVE_FRACTION TO 0.20.
SET ASCENT_MIN_SEPARATION_ALTITUDE TO 20000.
SET ASCENT_MAX_SPEED TO 720.      // 20% lower low-atmosphere speed limit
SET ASCENT_HIGH_SPEED_THROTTLE TO 0.65.
SET ASCENT_SPEED_LIMIT_END TO 35000. // stay throttled until the slower turn is complete
SET ASCENT_TURN_START TO 500.     // clear the pad before beginning the turn
SET ASCENT_TURN_END TO 35000.     // finish the turn above the densest atmosphere
SET ASCENT_TURN_DEGREES TO 75.    // final commanded pitch is 15 degrees above level
SET ASCENT_HEADING TO 90.         // automated sea target is deployed due east
// Fallback for test stages. The full mission computes this offset from the
// tank part and live vessel CoM because the stock engine is a separate mass.
SET HOOK_ABOVE_COM TO -27.0.
SET BOOSTER_HOOK_LOCAL_Y TO 3.0.
SET NET_PLANE_OFFSET TO 30.0.
SET CAPTURE_FINAL_SPEED TO 0.65.  // commanded downward m/s through net
SET PID_SWITCH_HEIGHT TO 18.      // vertical near-field PI only at the frame mouth
// Enter the one-way stopping corridor at 13.5 km while the fixed 30 km
// reference is still well behaved.  With tilt/thrust coupling enabled, the
// controller then holds the planned 75% main-braking envelope instead of
// silently reducing horizontal authority to a few m/s^2.
SET HORIZONTAL_CORRIDOR_HEIGHT TO 13500.
SET HORIZONTAL_CORRIDOR_RANGE TO 30000. // accept the measured powered-footprint envelope
SET CENTERING_HOLD_ALTITUDE TO 16. // emergency correction only at the frame mouth
SET CENTERING_HOLD_ERROR TO 7.0.  // moving cables can follow anything inside this radius
SET TERMINAL_HORIZONTAL_SPEED TO 0.5. // minimum speed used by the outer stopping corridor
SET TERMINAL_HORIZONTAL_CORRIDOR_SPEED TO 150. // high-altitude cap; stop envelope tapers it to zero
SET TERMINAL_HORIZONTAL_STOP_ACCEL TO 1.0. // conservative stop envelope absorbs the measured attitude lag
SET TERMINAL_HORIZONTAL_DEADBAND TO 3.0. // the moving cradle handles the final few metres
SET TERMINAL_ALIGN_RANGE TO 300.      // keep the range-indexed stop corridor active until the final approach
SET TERMINAL_ALIGN_SPEED TO 30.0.     // cover the far half before the 2 km gate; position field then tapers for braking
SET TERMINAL_ALIGN_POSITION_GAIN TO 0.15. // retain inward travel until the 2 km position gate, then taper before centre
SET TERMINAL_ALIGN_VELOCITY_GAIN TO 2.00. // measured long-stage lag needs decisive damping before crossing
// Final capture is two-stage.  Align while descending, then latch a
// vertical-only descent so attitude lag cannot create another lateral pass.
SET FINAL_ALIGN_HEIGHT TO 65.         // latch before the low-speed vertical loop can hover just above the old gate
SET FINAL_ALIGN_RANGE TO 100.
SET FINAL_ALIGN_HOLD_SECONDS TO 0.    // alignment continues during descent; do not spend the landing reserve hovering
SET FINAL_ALIGN_SPEED TO 3.0.
SET FINAL_ALIGN_POSITION_GAIN TO 0.05.
SET FINAL_ALIGN_VELOCITY_GAIN TO 0.40.
SET FINAL_ALIGN_READY_ERROR TO 7.0.  // keeps all four 1.65 m hooks inside the 20 m short edge
SET FINAL_ALIGN_READY_SPEED TO 1.25. // commit before attitude lag starts another centre pass
SET FINAL_ALIGN_READY_TILT TO 8.0.   // safely inside the net's 15-degree capture envelope
SET FINAL_CAPTURE_VELOCITY_GAIN TO 0.40.
SET FINAL_CAPTURE_MAX_ACCEL TO 1.0.
SET WIRE_HOLD_HEIGHT TO 12.       // wait here while the four cables close around the stage
SET WIRE_HOLD_HORIZONTAL_RANGE TO 20. // do not spend landing fuel hovering while still far from the frame
SET WIRE_HOLD_MAX_SECONDS TO 2.5. // short final settle; never a prolonged hover
SET POST_WIRE_CROSSING_SPEED TO 1.5. // decisive controlled crossing before fuel depletion
SET LANDING_MAX_TILT TO 12.       // degrees
SET ENTRY_MAX_TILT TO 55.         // high-energy braking; blends to landing limit
SET MAX_HORIZONTAL_SPEED TO 150.  // high-altitude boost-back limit
SET TELEMETRY_PERIOD TO 0.20.

// Return phase scheduling. High-energy attitude changes are deliberately kept
// out of ascent and atmospheric coast.
SET COAST_TRACK_DESCENT_SPEED TO -25. // hold inertial attitude through the fast apex rotation
SET ENTRY_RETROGRADE_HEIGHT TO 50000. // nozzle points along velocity below this height
SET ENTRY_DECEL_HEIGHT TO 40000.      // dedicated thermal/load reduction burn
SET ENTRY_HORIZONTAL_SPEED TO 1000.
SET ENTRY_ATTITUDE_SLEW_SECONDS TO 8.
SET TERMINAL_GUIDANCE_START_HEIGHT TO 30000.
SET TERMINAL_MAX_HORIZONTAL_ACCEL TO 55.
SET TERMINAL_ACCEL_FILTER TO 0.10.
// kOS needs roughly 0.3 s to execute the complete guidance loop on the loaded
// two-vessel scene. Use a faster filter above the waypoint so a maximum-energy
// stopping command does not spend several seconds ramping from 10% authority.
// The slower value remains active below 2 km to suppress final oscillation.
SET TERMINAL_HIGH_ENERGY_ACCEL_FILTER TO 0.45.
// Plan the main landing burn around 75% of the stock engine's maximum. The
// unused vector magnitude remains available for trajectory-error correction.
SET TERMINAL_NOMINAL_THRUST_FRACTION TO 0.75.
SET TERMINAL_TOTAL_THRUST_FRACTION TO 0.98.
SET TERMINAL_VELOCITY_CONE_DEGREES TO 30.
// Below this speed aerodynamic/load direction is negligible and the velocity
// vector becomes numerically unstable; use direct vertical-capture damping.
SET TERMINAL_VELOCITY_CONE_MIN_SPEED TO 30.
SET TERMINAL_WAYPOINT_HEIGHT TO 2000.
// Aim below the 200 m/s acceptance ceiling to leave sensor/attitude margin.
SET TERMINAL_WAYPOINT_VERTICAL_SPEED TO 180.
// Finish the planned lateral translation above the 2 km acceptance waypoint,
// leaving altitude to damp model/attitude lag before the straight-down leg.
SET TERMINAL_HORIZONTAL_PLAN_END_HEIGHT TO 5000.
// At 30 km, create one height-indexed Hermite path from the measured position
// and velocity to the recovery frame. Tracking a fixed spatial path avoids
// the aim-point jumps and alternating corrections caused by rolling replans.
SET TERMINAL_PLAN_POSITION_GAIN TO 0.10.
SET TERMINAL_PLAN_VELOCITY_GAIN TO 1.50.
SET TERMINAL_PLAN_STOP_ACCEL TO 3.0. // lag-aware no-overshoot velocity envelope
// With little propellant left, keep moving through the frame rather than hover.
SET TERMINAL_LOW_FUEL_FRACTION TO 0.02. // begin the decisive crossing before the last two percent is spent hovering
SET TERMINAL_LOW_FUEL_DESCENT_SCALE TO 1.28.
SET TERMINAL_LOW_FUEL_CAPTURE_SPEED TO 4.0. // cross decisively inside the 7.5 m/s capture envelope before fuel depletion
SET TERMINAL_DESCENT_MAX_SPEED TO 700.
SET MIDCOURSE_START_HEIGHT TO 30000.
SET MIDCOURSE_END_MARGIN TO 600.
SET MIDCOURSE_PREDICTED_ERROR TO 999999999. // ship is on measured footprint; disable noisy micro-corrections
SET MIDCOURSE_MAX_HORIZONTAL_ACCEL TO 8.
SET MIDCOURSE_VELOCITY_GAIN TO 0.30.
SET MIDCOURSE_VERTICAL_THRUST_G TO 0. // horizontal-only footprint correction; preserve landing fuel

// Controller gains. Keep these in one file so log-driven iterations are small,
// reviewable diffs. The shipped values passed the offline point-mass sweep.
SET STOP_DISTANCE_SAFETY TO 1.50.
SET BURN_ALTITUDE_MARGIN TO 250.
SET DRAG_ACCEL_FILTER TO 0.08.
SET H_POS_KP_HIGH TO 0.050.
SET H_POS_KP_LOW TO 0.030.
SET H_VEL_KP TO 1.00.
SET V_VEL_KP TO 2.00.
SET V_VEL_KI TO 0.020.
