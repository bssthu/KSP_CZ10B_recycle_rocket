// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
// Separation is fuel-budget driven: the first stage keeps at most one fifth of
// its original propellant for recovery.  Altitude is only a lower safety gate.
SET ASCENT_RESERVE_FRACTION TO 0.20.
SET ASCENT_MIN_SEPARATION_ALTITUDE TO 20000.
SET ASCENT_MAX_SPEED TO 720.      // 20% lower low-atmosphere speed limit
SET ASCENT_HIGH_SPEED_THROTTLE TO 0.65.
SET ASCENT_SPEED_LIMIT_END TO 25000. // stay throttled through the dense-air turn
SET ASCENT_TURN_START TO 500.     // clear the pad before beginning the turn
SET ASCENT_TURN_END TO 25000.     // slower gravity turn; avoid dense-air horizontal acceleration
SET ASCENT_TURN_DEGREES TO 75.    // final commanded pitch is 15 degrees above level
SET ASCENT_HEADING TO 90.         // automated sea target is deployed due east
// Integrated hook Y (3.0 m) minus the top-girder cable plane (30.0 m).
SET HOOK_ABOVE_COM TO -27.0.
SET CAPTURE_FINAL_SPEED TO 0.65.  // commanded downward m/s through net
SET PID_SWITCH_HEIGHT TO 18.      // vertical near-field PI only at the frame mouth
SET HORIZONTAL_CORRIDOR_HEIGHT TO 1000. // final damped corridor after rolling guidance
SET HORIZONTAL_CORRIDOR_RANGE TO 700. // include long-stage attitude lag in the braking distance
SET CENTERING_HOLD_ALTITUDE TO 16. // emergency correction only at the frame mouth
SET CENTERING_HOLD_ERROR TO 7.0.  // moving cables can follow anything inside this radius
SET TERMINAL_HORIZONTAL_SPEED TO 0.5. // minimum speed used by the outer stopping corridor
SET TERMINAL_HORIZONTAL_CORRIDOR_SPEED TO 45. // cap while following the stopping profile
SET TERMINAL_HORIZONTAL_STOP_ACCEL TO 1.5. // measured low-altitude steering authority, with lag margin
SET TERMINAL_HORIZONTAL_DEADBAND TO 3.0. // the moving cradle handles the final few metres
SET TERMINAL_ALIGN_RANGE TO 300.      // one-way handover before attitude-lag overshoot begins
SET TERMINAL_ALIGN_SPEED TO 12.0.     // fast enough to converge before the low-fuel crossing
SET TERMINAL_ALIGN_POSITION_GAIN TO 0.10. // direct-path taper below the frame
SET TERMINAL_ALIGN_VELOCITY_GAIN TO 0.65. // stronger final damping before crossing
// Final capture is two-stage.  Settle above the cable plane first, then latch
// a vertical-only descent so attitude lag cannot create another lateral pass.
SET FINAL_ALIGN_HEIGHT TO 45.
SET FINAL_ALIGN_RANGE TO 100.
SET FINAL_ALIGN_HOLD_SECONDS TO 12.
SET FINAL_ALIGN_SPEED TO 3.0.
SET FINAL_ALIGN_POSITION_GAIN TO 0.05.
SET FINAL_ALIGN_VELOCITY_GAIN TO 0.40.
SET FINAL_ALIGN_READY_ERROR TO 6.0.
SET FINAL_ALIGN_READY_SPEED TO 0.75.
SET FINAL_ALIGN_READY_TILT TO 1.0.
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
// At 30 km, create one height-indexed Hermite path from the measured position
// and velocity to the recovery frame.  Tracking a fixed spatial path avoids
// the aim-point jumps and alternating corrections caused by rolling replans.
SET TERMINAL_PLAN_POSITION_GAIN TO 0.035.
SET TERMINAL_PLAN_VELOCITY_GAIN TO 0.65.
SET TERMINAL_PLAN_STOP_ACCEL TO 3.0. // lag-aware no-overshoot velocity envelope
// With little propellant left, keep moving through the frame rather than hover.
SET TERMINAL_LOW_FUEL_FRACTION TO 0.0025. // preserve a short cable-closing hold at normal arrival fuel
SET TERMINAL_LOW_FUEL_DESCENT_SCALE TO 1.28.
SET TERMINAL_LOW_FUEL_CAPTURE_SPEED TO 1.0. // keep crossing the net instead of hovering dry
SET TERMINAL_DESCENT_SPEED_PER_METER TO 0.100.
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
