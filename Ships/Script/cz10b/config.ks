// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
SET SEPARATION_ALTITUDE TO 18000. // metres above sea level
SET ASCENT_MAX_SPEED TO 430.      // m/s; keeps the demonstrator suborbital
// Integrated hook Y (3.0 m) minus the platform cable plane (3.2 m).
SET HOOK_ABOVE_COM TO -0.2.
SET CAPTURE_FINAL_SPEED TO 0.65.  // commanded downward m/s through net
SET CENTERING_HOLD_ALTITUDE TO 150. // stop descending early if the net is not centered
SET TERMINAL_HORIZONTAL_SPEED TO 3. // damped final translation; also below capture limit
SET LANDING_MAX_TILT TO 12.       // degrees
SET ENTRY_MAX_TILT TO 28.         // degrees
SET MAX_HORIZONTAL_SPEED TO 150.  // high-altitude boost-back limit
SET TELEMETRY_PERIOD TO 0.20.

// Controller gains. Keep these in one file so log-driven iterations are small,
// reviewable diffs. The shipped values passed the offline point-mass sweep.
SET STOP_DISTANCE_SAFETY TO 1.18.
SET BURN_ALTITUDE_MARGIN TO 80.
SET H_POS_KP_HIGH TO 0.050.
SET H_POS_KP_LOW TO 0.080.
SET H_VEL_KP TO 0.55.
SET V_VEL_KP TO 0.62.
SET V_VEL_KI TO 0.045.
