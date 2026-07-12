// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
SET SEPARATION_ALTITUDE TO 18000. // metres above sea level
SET ASCENT_MAX_SPEED TO 500.      // Kerbin's atmosphere ends at only 70 km
SET ASCENT_TURN_START TO 500.     // clear the pad before beginning the turn
SET ASCENT_TURN_END TO 16000.     // finish before staging so attitude is settled
SET ASCENT_TURN_DEGREES TO 45.    // Kerbin-scaled gravity turn; 45 deg pitch at completion
SET ASCENT_HEADING TO 90.         // automated sea target is deployed due east
// Integrated hook Y (3.0 m) minus the elevated platform cable plane (24.0 m).
SET HOOK_ABOVE_COM TO -21.0.
SET CAPTURE_FINAL_SPEED TO 0.65.  // commanded downward m/s through net
SET PID_SWITCH_HEIGHT TO 500.     // fixed trajectory above, local feedback below
SET CENTERING_HOLD_ALTITUDE TO 30. // emergency correction only at the frame mouth
SET CENTERING_HOLD_ERROR TO 7.0.  // moving cables can follow anything inside this radius
SET TERMINAL_HORIZONTAL_SPEED TO 2.5. // damped final translation; below capture limit
SET WIRE_HOLD_HEIGHT TO 12.       // wait here while the four cables close around the stage
SET LANDING_MAX_TILT TO 12.       // degrees
SET ENTRY_MAX_TILT TO 28.         // degrees
SET MAX_HORIZONTAL_SPEED TO 150.  // high-altitude boost-back limit
SET TELEMETRY_PERIOD TO 0.20.

// Return phase scheduling. High-energy attitude changes are deliberately kept
// out of ascent and atmospheric coast.
SET COAST_TRACK_DESCENT_SPEED TO -25. // hold inertial attitude through the fast apex rotation
SET POST_APOGEE_FLIP_SECONDS TO 6.
SET RETURN_BURN_MAX_TIME TO 45.
SET RETURN_MAX_HORIZONTAL_SPEED TO 180.
SET RETURN_MAX_HORIZONTAL_ACCEL TO 10.
SET RETURN_BURN_MAX_TILT TO 65.
SET RETURN_BURN_END_ERROR TO 350.
SET RETURN_BURN_END_SPEED TO 15.
SET ENTRY_BURN_ALTITUDE TO 25000.
SET ENTRY_BURN_MAX_TIME TO 14.
SET ENTRY_BURN_THROTTLE TO 0.75.
SET ENTRY_TARGET_VERTICAL_SPEED TO -230.
SET ENTRY_ATTITUDE_SLEW_SECONDS TO 6.
SET ENTRY_THROTTLE_RAMP_SECONDS TO 1.
SET TERMINAL_START_ALTITUDE TO 10000.
SET TERMINAL_TGO_MIN TO 35.
SET TERMINAL_TGO_MAX TO 90.
SET TERMINAL_MAX_HORIZONTAL_ACCEL TO 8.
SET TERMINAL_ACCEL_FILTER TO 0.10.

// Controller gains. Keep these in one file so log-driven iterations are small,
// reviewable diffs. The shipped values passed the offline point-mass sweep.
SET STOP_DISTANCE_SAFETY TO 1.18.
SET BURN_ALTITUDE_MARGIN TO 80.
SET H_POS_KP_HIGH TO 0.050.
SET H_POS_KP_LOW TO 0.050.
SET H_VEL_KP TO 0.20.
SET V_VEL_KP TO 0.62.
SET V_VEL_KI TO 0.045.
