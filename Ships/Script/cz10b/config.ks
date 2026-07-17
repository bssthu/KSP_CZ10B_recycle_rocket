// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
SET BOOSTER_ENGINE_PART_NAME TO "liquidEngineMainsail.v2".
// The stock Mainsail is limited to the former 1200 kN during ascent. After
// staging its full 1500 kN envelope becomes available to recovery guidance.
SET ASCENT_ENGINE_THRUST_LIMIT TO 80.
// Keep the vehicle at zero commanded throttle until ignition, then increase
// throttle linearly.  The clamps may release during the ramp, but only after
// measured thrust has produced the required hold-down TWR.
SET ASCENT_IGNITION_RAMP_SECONDS TO 5.
SET ASCENT_CLAMP_RELEASE_TWR TO 1.05.
// Separation is fuel-budget driven: the first stage keeps at most one fifth of
// its original propellant for recovery.  Altitude is only a lower safety gate.
SET ASCENT_RESERVE_FRACTION TO 0.20.
SET ASCENT_MIN_SEPARATION_ALTITUDE TO 20000.
SET ASCENT_MAX_SPEED TO 720.      // 20% lower low-atmosphere speed limit
SET ASCENT_HIGH_SPEED_THROTTLE TO 0.65.
SET ASCENT_SPEED_LIMIT_END TO 32000. // keep the dense-air limit through the turn
// The part models combined grid-fin/cold-gas authority.  Keep the historical
// 20/20/10 kN-m response through ascent and entry.  Main-burn guidance releases
// the full authority so the real thrust vector can follow the load-constrained
// command instead of retaining a large upward component through the stop.
SET BOOSTER_FLIGHT_REACTION_WHEEL_AUTHORITY TO 0.625.
SET BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY TO 100.
SET ASCENT_TURN_START TO 500.     // clear the pad before beginning the turn
// Finish a little earlier and three degrees flatter than the previous ascent.
// The old 15-degree tail of the turn raised apoapsis past the upper-stage
// target before separation, leaving the second stage dormant until apoapsis.
SET ASCENT_TURN_END TO 32000.
SET ASCENT_TURN_DEGREES TO 78.    // final commanded pitch is 12 degrees above level
SET ASCENT_HEADING TO 90.         // automated sea target is deployed due east
// Fallback for test stages. The full mission computes this offset from the
// tank part and live vessel CoM because the stock engine is a separate mass.
SET HOOK_ABOVE_COM TO -27.0.
SET BOOSTER_HOOK_LOCAL_Y TO 3.0.
SET NET_PLANE_OFFSET TO 30.0.
SET CAPTURE_FINAL_SPEED TO 0.65.  // commanded downward m/s through net
SET PID_SWITCH_HEIGHT TO 18.      // vertical near-field PI only at the frame mouth
// Arm horizontal/vertical thrust coupling with the one-shot 30 km trajectory.
// The range-indexed near-field controller itself is gated to the 2 km endpoint
// in main.ks, so it cannot cross the ship high and then reverse toward it.
SET HORIZONTAL_CORRIDOR_HEIGHT TO 30000.
SET HORIZONTAL_CORRIDOR_RANGE TO 30000. // accept the measured powered-footprint envelope
SET CENTERING_HOLD_ALTITUDE TO 16. // emergency correction only at the frame mouth
SET CENTERING_HOLD_ERROR TO 7.0.  // moving cables can follow anything inside this radius
SET TERMINAL_HORIZONTAL_SPEED TO 0.5. // minimum speed used by the outer stopping corridor
SET TERMINAL_HORIZONTAL_CORRIDOR_SPEED TO 150. // high-altitude cap; stop envelope tapers it to zero
SET TERMINAL_HORIZONTAL_STOP_ACCEL TO 0.60. // measured lag-aware stop envelope remains a hard bound inside capture-align
// Stop the medium-altitude position loop only after the hooks are comfortably
// inside the formal 30 m waypoint envelope.  Chasing the last few metres above 2 km held the
// long stage nearly sideways, so aerodynamic drag removed about 90 m/s of
// vertical speed.  Final-align and the moving cradle still remove this offset.
// Keep a small position field alive at the 2 km gate.  With a 20 m deadband,
// the measured 20.9 m waypoint state received almost pure velocity damping
// and coasted out to 35.4 m before the range field regained authority.
SET TERMINAL_HORIZONTAL_DEADBAND TO 15.0.
SET TERMINAL_ALIGN_RANGE TO 550.      // begin the measured 250-300 m stopping transient in thin air, before centre crossing
SET TERMINAL_ALIGN_SPEED TO 30.0.     // absolute ceiling; the one-way latch normally records a much lower entry-speed cap
SET TERMINAL_ALIGN_MIN_SPEED TO 10.0. // retain convergence for near-tangential entries without re-accelerating to 30 m/s
SET TERMINAL_ALIGN_POSITION_GAIN TO 0.15. // retain inward travel until the 2 km position gate, then taper before centre
SET TERMINAL_ALIGN_VELOCITY_GAIN TO 2.00. // measured long-stage lag needs decisive damping before crossing
// Keep full braking authority on entering the 300 m neighbourhood, but taper
// it to the 1 m/s^2 stop-envelope floor at the 40 m deadband. A fixed 5 m/s^2
// command for only 2.5 m/s of residual drift held the long stage sideways and
// cost roughly 90 m/s of vertical speed through aerodynamic drag.
SET TERMINAL_ALIGN_ACCEL_RANGE_GAIN TO 0.75.
// Even inside the position deadband, retain one time-constant of pure velocity
// damping so a 5-6 m/s centre crossing cannot coast back out of the envelope.
// The limit falls with measured speed and therefore cannot sustain a large tilt.
SET TERMINAL_ALIGN_ACCEL_VELOCITY_GAIN TO 1.00.
// Arm the one-way finish while there is still enough measured stopping room.
// The 100 m / 10 m/s gate let the stage enter at 62.5 m and 9.85 m/s; steering
// lag then carried a 2.3 m/s centre crossing from 3.6 km down to a 35.4 m
// rebound.  Conversely, a 150 m / 15 m/s gate with a reduced position term
// stopped at 37.1 m instead of reaching the formal 30 m waypoint.  The
// measured 130 m / 13 m/s point lies between those two real-flight bounds:
// it adds braking altitude without cancelling the final inward approach.
SET TERMINAL_ALIGN_SETTLE_ENTRY_RANGE TO 130.0.
SET TERMINAL_ALIGN_SETTLE_ENTRY_SPEED TO 13.0.
SET TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN TO 2.0.
SET TERMINAL_ALIGN_SETTLE_MAX_ACCEL TO 25.0.
SET TERMINAL_ALIGN_SETTLE_POSITION_GAIN TO 0.20.
// Reject a genuine centre escape without restoring the full pursuit profile.
SET TERMINAL_ALIGN_REACQUIRE_RANGE TO 30.0.
SET TERMINAL_ALIGN_REACQUIRE_POSITION_GAIN TO 0.20.
SET TERMINAL_ALIGN_REACQUIRE_MAX_ACCEL TO 25.0.
// Final capture is two-stage.  Start the slow horizontal settle early enough
// to absorb the measured multi-second steering lag, then latch a vertical-only
// descent so a centre crossing cannot create another lateral pass.  Keep the
// vertical slowdown at 65 m: moving the horizontal handoff upward must not
// consume the last landing reserve in a long low-speed descent.
// Keep the decisive post-centre braking loop through the 2 km gate. The softer
// final field takes over only below 1 km; handing it control at 2.2-5 km slowed
// the attitude response and increased the measured rebound instead of reducing
// it. Vertical slowdown remains independently gated at 65 m.
SET FINAL_ALIGN_HEIGHT TO 1000.
SET FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT TO 65.
SET FINAL_ALIGN_RANGE TO 50.
SET FINAL_ALIGN_HOLD_SECONDS TO 0.    // alignment continues during descent; do not spend the landing reserve hovering
SET FINAL_ALIGN_SPEED TO 3.0.
SET FINAL_ALIGN_POSITION_GAIN TO 0.08.
SET FINAL_ALIGN_VELOCITY_GAIN TO 0.50.
SET FINAL_ALIGN_READY_ERROR TO 6.0.  // keeps all four 1.65 m hooks inside the 20 m short edge
SET FINAL_ALIGN_READY_SPEED TO 1.25. // commit before attitude lag starts another centre pass
SET FINAL_ALIGN_READY_TILT TO 8.0.   // safely inside the net's 15-degree capture envelope
SET FINAL_CAPTURE_VELOCITY_GAIN TO 0.40.
SET FINAL_CAPTURE_MAX_ACCEL TO 1.0.
// Retain a sub-metre-per-second centring field after vertical capture commits.
// Pure velocity damping let the last few metres of aerodynamic drift carry a
// valid 6-7 m alignment outside the short-axis cable cradle before contact.
SET FINAL_CAPTURE_POSITION_DEADBAND TO 2.0.
SET FINAL_CAPTURE_POSITION_GAIN TO 0.10.
SET FINAL_CAPTURE_MAX_SPEED TO 0.75.
// The long, nearly empty stage needs a deliberately soft cooked-steering
// torque loop near the ship.  The default loop follows sub-degree thrust-vector
// changes too aggressively and produced measured 12.8-18 deg/s rate spikes.
// Preserve the fast cooked-steering response while the high-energy pulse is
// still turning the stage.  Switch to the deliberately soft torque loop before
// the 2 km handoff so bounded drift correction cannot create a rate spike in
// the powered-flight audit below 1 km.
SET TERMINAL_STEERING_TUNE_HEIGHT TO 2200.
SET TERMINAL_STEERING_PITCH_YAW_TS TO 8.0.
SET TERMINAL_STEERING_ROLL_TS TO 5.0.
SET TERMINAL_STEERING_MAX_STOPPING_TIME TO 2.0.
SET TERMINAL_STEERING_TORQUE_FACTOR TO 1.0.
// The high-dynamic-pressure waypoint handoff needs a short, forceful turn.
// These values are restored to the soft profile above before final capture.
SET TERMINAL_COAST_STEERING_PITCH_YAW_TS TO 0.5.
SET TERMINAL_COAST_STEERING_ROLL_TS TO 1.0.
SET TERMINAL_COAST_STEERING_MAX_STOPPING_TIME TO 50.0.
SET TERMINAL_COAST_STEERING_TORQUE_FACTOR TO 0.1.
SET WIRE_HOLD_HEIGHT TO 12.       // wait here while the four cables close around the stage
SET WIRE_HOLD_HORIZONTAL_RANGE TO 20. // do not spend landing fuel hovering while still far from the frame
SET WIRE_HOLD_MAX_SECONDS TO 2.5. // short final settle; never a prolonged hover
SET POST_WIRE_CROSSING_SPEED TO 1.5. // decisive controlled crossing before fuel depletion
SET LANDING_MAX_TILT TO 12.       // degrees
SET ENTRY_MAX_TILT TO 89.         // near-horizontal low-speed braking; >=300 m/s load cone remains the hard limit
SET MAX_HORIZONTAL_SPEED TO 150.  // high-altitude boost-back limit
SET TELEMETRY_PERIOD TO 0.20.

// Return phase scheduling. High-energy attitude changes are deliberately kept
// out of ascent and atmospheric coast.
SET COAST_TRACK_DESCENT_SPEED TO -25. // hold inertial attitude through the fast apex rotation
SET ENTRY_RETROGRADE_HEIGHT TO 50000. // nozzle points along velocity below this height
SET ENTRY_DECEL_HEIGHT TO 30000.      // one dedicated thermal/load reduction burn
SET ENTRY_HORIZONTAL_SPEED TO 1000.  // cut off as soon as surface-horizontal speed reaches this gate
SET ENTRY_ATTITUDE_SLEW_SECONDS TO 8.
SET TERMINAL_GUIDANCE_START_HEIGHT TO 30000.
SET TERMINAL_MAX_HORIZONTAL_ACCEL TO 55.
SET TERMINAL_ACCEL_FILTER TO 0.10.
// kOS needs roughly 0.3 s to execute the complete guidance loop on the loaded
// two-vessel scene. Use a faster filter above the waypoint so a maximum-energy
// stopping command does not spend several seconds ramping from 10% authority.
// The slower value remains active below 2 km to suppress final oscillation.
SET TERMINAL_HIGH_ENERGY_ACCEL_FILTER TO 0.80.
// Plan the main landing burn around 75% of the stock engine's maximum. The
// unused vector magnitude remains available for trajectory-error correction.
SET TERMINAL_NOMINAL_THRUST_FRACTION TO 0.75.
SET TERMINAL_TOTAL_THRUST_FRACTION TO 0.98.
SET TERMINAL_VELOCITY_CONE_DEGREES TO 30.
// Below 300 m/s the 30-degree retrograde load cone would rotate an efficient
// 80-degree lateral stop back toward vertical, over-braking descent to about
// 110 m/s at the 2 km gate. Dynamic pressure is already low enough here to
// hand over to direct vector control.
SET TERMINAL_VELOCITY_CONE_MIN_SPEED TO 300.
SET TERMINAL_WAYPOINT_HEIGHT TO 2000.
// The 2 km waypoint is a corridor rather than a one-sided maximum.  Aiming at
// its centre leaves margin for atmosphere/model error without permitting an
// early, fuel-wasting slowdown.
SET TERMINAL_WAYPOINT_VERTICAL_SPEED TO 175.
SET TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED TO 150.
SET TERMINAL_WAYPOINT_MAX_VERTICAL_SPEED TO 200.
// Main ignition is solved against the complete velocity change to the 2 km
// waypoint at the nominal 75% thrust magnitude.  Only a small state-estimation
// allowance is added; the former 1.5x stop-distance gate was intentionally
// conservative all the way to the net and ignited much too early.
SET TERMINAL_IGNITION_SAFETY TO 1.05.
SET TERMINAL_IGNITION_MARGIN TO 150.
// Measured time for the long empty stage to turn a new lateral command into
// acceleration.  Include it in both ignition height and powered footprint.
SET TERMINAL_GUIDANCE_RESPONSE_SECONDS TO 1.5.
SET TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS TO 1.5.
SET TERMINAL_HORIZONTAL_LEAD_SECONDS TO 0.45.
SET TERMINAL_WAYPOINT_POSITION_COEFFICIENT TO 9.0.
SET TERMINAL_WAYPOINT_VELOCITY_COEFFICIENT TO 4.6.
// Horizontal braking may add vertical thrust only while descent is faster than
// this smooth quarter-power floor.  This prevents lateral work from holding the
// stage near 110 m/s at 6-7 km while retaining full braking authority on entry.
SET TERMINAL_DESCENT_COUPLING_BAND TO 50.
// Finish the horizontal plan at the formal 2 km gate, just short of the
// ship centre.  The high-energy regression includes the measured 1.5 s stage
// response and requires both horizontal limits at this same crossing.
// This is inside the 30 m gate while preventing a high-altitude centre crossing;
// the existing low-energy controller removes the final offset below 2 km.
SET TERMINAL_HORIZONTAL_PLAN_END_HEIGHT TO 2000.
// The formal gate is a 30 m circle.  A 70 m stand-off made first entry depend
// on dynamic overshoot; target a point 15 m short so every nominal approach
// necessarily enters the one-way 30 m brake latch without aiming past the net.
SET TERMINAL_WAYPOINT_APPROACH_OFFSET TO 15.
SET TERMINAL_CAPTURE_ALIGN_ARM_HEIGHT TO 2000.
// After the final 16 km checkpoint, coast while already pointing along the
// solved main-burn vector.  Real telemetry measured a 2.5-3 s attitude lag;
// starting upright spent the first powered seconds over-braking vertically.
SET TERMINAL_MAIN_PREALIGN_HEIGHT TO 15000.
// A distinct one-way high-energy handoff removes the last few hundred metres
// of horizontal velocity without allowing a centre crossing to re-arm a fast
// pursuit in the opposite direction.
SET TERMINAL_HIGH_ENERGY_BRAKE_ARM_HEIGHT TO 6000.
SET TERMINAL_HIGH_ENERGY_BRAKE_RANGE TO 400.
SET TERMINAL_HIGH_ENERGY_BRAKE_POSITION_GAIN TO 0.20.
SET TERMINAL_HIGH_ENERGY_BRAKE_MAX_SPEED TO 30.
SET TERMINAL_HIGH_ENERGY_BRAKE_VELOCITY_GAIN TO 1.00.
SET TERMINAL_HIGH_ENERGY_BRAKE_MAX_ACCEL TO 55.
SET TERMINAL_HIGH_ENERGY_BRAKE_SETTLE_HEIGHT TO 2200.
// Main ignition and the powered horizontal loop use the measured attitude lag
// directly.  The 0.45 deceleration factor compensates the strong atmospheric
// braking still present between 11 km and the formal 2 km waypoint.
SET TERMINAL_MAIN_ATTITUDE_RESPONSE_SECONDS TO 2.8.
SET TERMINAL_MAIN_HORIZONTAL_STOP_SAFETY TO 1.0.
SET TERMINAL_MAIN_DIRECT_STOP_DECEL_GAIN TO 0.45.
SET TERMINAL_MAIN_DIRECT_STOP_CROSS_GAIN TO 0.02.
SET TERMINAL_MAIN_DIRECT_STOP_MIN_RANGE TO 100.
// Below the 300 m/s load-cone threshold, lead the lagging attitude slightly
// through the horizon once descent approaches the 200 m/s upper gate.  The
// real stage otherwise keeps an upward thrust component for another 3-5 s and
// reaches 2 km at only ~80 m/s despite a nearly horizontal command.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_SPEED TO 300.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_MAX_SURFACE_SPEED TO 500.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_DEGREES TO 30.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_HEIGHT TO 7000.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_MAX_SURFACE_SPEED TO 850.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_DEGREES TO 30.
// Release the vertical main burn only when a 270-285 m/s ballistic 2 km state
// is reachable.  The wider band covers one 0.2 s telemetry/control sample;
// the simultaneous horizontal gate still keeps total speed below the 300 m/s
// load-cone boundary.  Side-slip and the final pulse consume the extra speed.
SET TERMINAL_WAYPOINT_COAST_ERROR TO 30.
SET TERMINAL_WAYPOINT_COAST_HORIZONTAL_SPEED TO 4.5.
SET TERMINAL_WAYPOINT_COAST_MAX_VERTICAL_SPEED TO 285.
// This is the unbraked ballistic endpoint, not the required 2 km error.
// Real telemetry reaches the vertical-speed window with enough range to stop,
// but its coast-only endpoint has already crossed the ship by about 1.3 km.
// The trim controller below still has to finish inside the strict 30 m gate.
SET TERMINAL_WAYPOINT_COAST_ENTRY_ERROR TO 1800.
SET TERMINAL_WAYPOINT_COAST_ENTRY_HORIZONTAL_SPEED TO 230.
SET TERMINAL_WAYPOINT_COAST_ENTRY_MIN_VERTICAL_SPEED TO 270.
// Arm across the complete measured stopping footprint.  Waiting for both an
// 85-degree attitude and a 650 m radius let the unpowered stage cross the ship
// before its first pulse; at 65 degrees most thrust is already horizontal and
// the remaining vertical component is small enough for the 150 m/s lower gate.
SET TERMINAL_WAYPOINT_TRIM_ARM_RANGE TO 1800.
SET TERMINAL_WAYPOINT_TRIM_MIN_ACTUAL_TILT TO 65.
SET TERMINAL_WAYPOINT_TRIM_MAX_ACCEL TO 80.
// Once the first approach enters the formal 30 m circle, continuous position
// pursuit is finished. Apply one fixed-direction stop;
// this prevents a 180-degree command reversal after the centre crossing.
// The verified 157.429 / 2.117 / 21.514 m/s-m 2 km flight entered the frozen
// brake inside this first-approach neighbourhood.  The 15 m nominal endpoint
// makes that entry deterministic instead of relying on overshoot.  Do not add
// a speed gate here: telemetry showed the complete first pass stayed at
// 51-53 m/s and would otherwise miss this one-way event.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_ENTRY_ERROR TO 30.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_SAFETY TO 1.25.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_VELOCITY_GAIN TO 1.5.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_MIN_RANGE TO 5.
// One measured two-axis delta-v removes both the frozen primary speed and its
// perpendicular residual.  Do not reverse for a second endpoint correction:
// the long broadside stage retains too much angular momentum for a safe
// high-dynamic-pressure direction change.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_MAX_ACCEL TO 60.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_GAIN TO 3.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_MIN_ACCEL TO 5.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_TOLERANCE TO 0.5.
// The subsequent 90-to-0 degree coast slew consistently adds about 2.8 m/s
// outward and 35 m of displacement.  End this single impulse with a small
// opposite velocity so the measured slew drift, rather than another burn,
// brings the stage to the formal waypoint.
SET TERMINAL_WAYPOINT_ENDPOINT_VELOCITY_SCALE TO -0.75.
SET TERMINAL_WAYPOINT_ENDPOINT_MAX_PULSES TO 1.
// Keep the aligned post-upright experiment available for diagnostics, but the
// accepted path compensates the measured slew drift in the single endpoint
// impulse above.  A second high-altitude turn cost too much vertical speed.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ENABLED TO FALSE.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_HEIGHT TO 2300.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ARM_TILT TO 10.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_TILT TO 80.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ALIGNMENT_DEGREES TO 10.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_VELOCITY_SCALE TO 0.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MAX_ACCEL TO 20.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_GAIN TO 2.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_ACCEL TO 3.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_TOLERANCE TO 0.5.
// A final PWM pulse changes horizontal speed by several metres per second.
// Once the ballistic 2 km endpoint is inside the strict position gate and the
// residual speed is this small, latch coast instead of re-arming another pulse.
SET TERMINAL_WAYPOINT_FINAL_COAST_HORIZONTAL_SPEED TO 5.0.
// At main ignition, create one height-indexed Hermite path from the measured
// position and velocity to the recovery frame. Tracking a fixed spatial path
// avoids the aim-point jumps and alternating corrections caused by replans.
SET TERMINAL_PLAN_POSITION_GAIN TO 0.10.
SET TERMINAL_PLAN_VELOCITY_GAIN TO 3.00.
// Keep the safety envelope at the same physical authority as the controller.
// A 3 m/s^2 envelope clipped the Hermite reference to a few hundred m/s at
// 30 km and silently replaced height scheduling with premature spatial stop.
SET TERMINAL_PLAN_STOP_ACCEL TO 55.0.
// With little propellant left, keep moving through the frame rather than hover.
SET TERMINAL_LOW_FUEL_FRACTION TO 0.02. // begin the decisive crossing before the last two percent is spent hovering
SET TERMINAL_LOW_FUEL_DESCENT_SCALE TO 1.28.
// Six metres per second retains a 1.5 m/s margin inside the net's measured
// 7.5 m/s closing-speed envelope.  Once capture is committed, main.ks carries
// this same target continuously from 65 m to the net; allowing the height
// corridor to slow first caused a measured fuel-out at 7.59 m and a rejected
// 13.78 m/s free-fall crossing.
SET TERMINAL_LOW_FUEL_CAPTURE_SPEED TO 6.0.
SET TERMINAL_DESCENT_MAX_SPEED TO 700.
// After the 30 km thermal burn, coast at zero throttle.  At most one short
// footprint-correction pulse is allowed at each checkpoint; terminal braking
// supersedes a pulse whenever its solved ignition gate is reached.
SET MIDCOURSE_CHECKPOINT_1_HEIGHT TO 24000.
SET MIDCOURSE_CHECKPOINT_2_HEIGHT TO 20000.
SET MIDCOURSE_CHECKPOINT_3_HEIGHT TO 16000.
SET MIDCOURSE_PULSE_SECONDS TO 1.5.
SET MIDCOURSE_PREDICTED_ERROR TO 100.
SET MIDCOURSE_MAX_HORIZONTAL_ACCEL TO 8.
SET MIDCOURSE_MAX_THROTTLE TO 0.20.
SET MIDCOURSE_MAX_DELTA_V TO 15.
SET MIDCOURSE_MAX_HEIGHT_DROP TO 900.
SET MIDCOURSE_MIN_FUEL_FRACTION TO 0.12.
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
