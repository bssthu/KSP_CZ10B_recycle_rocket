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
SET TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN TO 3.5.
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
// Preserve the default cooked-steering response while the 2 km gate's outward
// drift is being arrested. Switch to the deliberately soft torque loop at 1 km,
// where the powered-flight angular-rate audit begins.
SET TERMINAL_STEERING_TUNE_HEIGHT TO 1000.
SET TERMINAL_STEERING_PITCH_YAW_TS TO 8.0.
SET TERMINAL_STEERING_ROLL_TS TO 5.0.
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
SET ENTRY_DECEL_HEIGHT TO 40000.      // dedicated thermal/load reduction burn
SET ENTRY_HORIZONTAL_SPEED TO 1500. // skip coupled retrograde braking; the 30 km plan removes lateral speed efficiently
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
// Below 300 m/s the 30-degree retrograde load cone would rotate an efficient
// 80-degree lateral stop back toward vertical, over-braking descent to about
// 110 m/s at the 2 km gate. Dynamic pressure is already low enough here to
// hand over to direct vector control.
SET TERMINAL_VELOCITY_CONE_MIN_SPEED TO 300.
SET TERMINAL_WAYPOINT_HEIGHT TO 2000.
// Aim close to, but below, the requested 200 m/s gate.  Horizontal braking is
// scheduled higher so this value controls the gate rather than tilt coupling.
SET TERMINAL_WAYPOINT_VERTICAL_SPEED TO 190.
// Horizontal braking may add vertical thrust only while descent is faster than
// this smooth quarter-power floor.  This prevents lateral work from holding the
// stage near 110 m/s at 6-7 km while retaining full braking authority on entry.
SET TERMINAL_DESCENT_COUPLING_BAND TO 50.
// Real-flight telemetry shows the stage still carrying 20 m/s laterally when
// it reaches 5 km.  Finish the reference there so the attitude-lagged stop is
// complete before the formal 2 km waypoint.  The former 3.5 km endpoint let
// the stage cross the ship at 4.8 km, then held it nearly sideways while it
// corrected a 70-80 m rebound; aerodynamic drag over-braked descent to 98 m/s.
SET TERMINAL_HORIZONTAL_PLAN_END_HEIGHT TO 6000.
// At 30 km, create one height-indexed Hermite path from the measured position
// and velocity to the recovery frame. Tracking a fixed spatial path avoids
// the aim-point jumps and alternating corrections caused by rolling replans.
SET TERMINAL_PLAN_POSITION_GAIN TO 0.10.
SET TERMINAL_PLAN_VELOCITY_GAIN TO 1.50.
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
