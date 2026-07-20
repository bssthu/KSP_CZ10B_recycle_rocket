// Mission geometry. Tune these first; gains live in controller.ks.
SET TARGET_VESSEL_NAME TO "Recovery Ship".
SET BOOSTER_ENGINE_PART_NAME TO "liquidEngineMainsail.v2".
// The stock Mainsail is limited to the former 1200 kN during ascent. After
// staging its full 1500 kN envelope becomes available to recovery guidance.
SET ASCENT_ENGINE_THRUST_LIMIT TO 80.
// The stock Mainsail represents the complete first-stage engine cluster during
// ascent.  Recovery entry keeps the equivalent 45% landing subset.  At the
// continuous main-burn handoff the limiter is reduced smoothly with mass.
// Without this normalization the same 75% command rose from roughly 29 to
// 47 m/s^2 as propellant depleted.  Runs 18 and 66 bound the useful actuator
// scale: too little acceleration makes the solver ignite prematurely, while
// 28 m/s^2 nearly stops the stage around 4 km.  The current value below is an
// attributable intermediate test, not a claim that the model is calibrated.
SET RETURN_ENGINE_THRUST_LIMIT TO 45.
// Run 66 showed that 28 m/s^2 makes the mandatory 75% command produce about
// 2.1 g before aerodynamic braking.  The stage then nearly stops near 4 km,
// making the live surface-retrograde axis ill-conditioned and causing the
// measured nozzle/velocity cone to rotate through the 30-degree limit.  Keep
// nominal thrust moderately above Kerbin gravity while preserving 25% vector
// authority for trajectory correction.
SET RETURN_ENGINE_MAX_ACCEL TO 21.5.
SET RETURN_ENGINE_MIN_THRUST_LIMIT TO 10.
// Burn-time prediction accounts for the large propellant fraction consumed by
// the continuous return burn.  KSP's stock Mainsail varies from 280 s at sea
// level to 310 s in vacuum; 300 s is the identified effective value over the
// 20-to-2 km main segment.  The dry mass is the 8.36 t value repeatedly
// measured after recovery propellant exhaustion.
SET RETURN_ENGINE_EFFECTIVE_ISP TO 300.
SET BOOSTER_RETURN_DRY_MASS TO 8.36.
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
// The integrated wheel represents only cold-gas/fine-control authority.  Four
// physically attached ModuleAeroSurface panels now provide grid-fin steering
// that scales with dynamic pressure.  Fixed 800--2400 kN-m wheel trials could
// not reach the cone edge at high speed and violently flipped the stage after
// it slowed, so keep the historical 20/20/10 kN-m response outside the bounded
// entry-cutoff-to-2-km terminal interval.
SET BOOSTER_FLIGHT_REACTION_WHEEL_AUTHORITY TO 0.625.
// Run 59 proved that 10% authority could not follow the late cone-edge stop:
// the requested 63--70 degree stage tilt remained at 50--55 degrees, leaving
// 132 m/s at the 2 km plane.  The former 16% trial exceeded the physical
// nozzle limit only with a 26.2 degree command.  Pair 16% with the deliberately
// narrower 20 degree command below; this retains substantially more tracking
// margin than that rejected combination while keeping the audited axis inside
// the hard 30 degree cone.
SET BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY TO 16.0.
SET BOOSTER_TERMINAL_REACTION_WHEEL_RESTORE_AUTHORITY TO 16.0.
SET BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_START_HEIGHT TO 4500.
SET BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_END_HEIGHT TO 3000.
// Retain physical grid-fin lift through the one 40 km entry burn, then remove
// it before checkpoint/main guidance.  Run 26 measured 7--21 m/s^2 of upward
// force and a kilometre-scale opposite side force even at coefficient 0.42.
// The bounded 5% terminal wheel above owns attitude after this handoff.
SET BOOSTER_TERMINAL_GRID_FIN_LIFT_AUTHORITY TO 0.0.
SET ASCENT_TURN_START TO 500.     // clear the pad before beginning the turn
// Reach the required near-horizontal state at 35 km.  The target pitch remains
// 12 degrees above the local horizon, inside the accepted 0--15 degree band.
SET ASCENT_TURN_END TO 35000.
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
SET TERMINAL_HORIZONTAL_DEADBAND TO 3.0.
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
// The part exposes extra terminal attitude authority above b0ef's physical
// wheel torque.  Enter its deliberately soft cooked-steering profile just below
// the formal 2 km measurement plane; waiting until 1 km produced a measured
// 61.4 deg/s spike.
SET TERMINAL_STEERING_TUNE_HEIGHT TO 1950.
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
SET ENTRY_DECEL_HEIGHT TO 40000.      // one dedicated continuous entry burn
// Keep the formal requirement at <=1000 m/s while absorbing the measured
// 14--15 m/s coast/reorientation gain before main ignition.
SET ENTRY_HORIZONTAL_SPEED TO 985.
SET ENTRY_ATTITUDE_SLEW_SECONDS TO 8.
SET TERMINAL_GUIDANCE_START_HEIGHT TO 40000.
SET TERMINAL_MAX_HORIZONTAL_ACCEL TO 55.
SET TERMINAL_ACCEL_FILTER TO 0.10.
// The run-17 0.80 filter moved the command faster than the long stage could
// physically slew and produced a 33.65 deg/s transient.  Keep enough response
// for the high-energy stop while filtering one-loop aerodynamic estimates.
// The slower value remains active below 2 km to suppress final oscillation.
SET TERMINAL_HIGH_ENERGY_ACCEL_FILTER TO 0.50.
// Plan the main landing burn around 75% of the stock engine's maximum. The
// unused vector magnitude remains available for trajectory-error correction.
SET TERMINAL_NOMINAL_THRUST_FRACTION TO 0.75.
SET TERMINAL_TOTAL_THRUST_FRACTION TO 1.0.
// The hybrid observer saturates the high-q vertical residual and therefore
// underestimates the vector magnitude needed to track the Run-52 path.  Real
// telemetry required 82--85% from 15 to 7 km while retaining a nearly exact
// surface-retrograde direction.  Treat this smooth 9-point addition as an
// atmospheric tracking correction, not a new nominal setting: ramp from 75%
// below 20 km, hold 84% from 15 to 7 km, then release it by 5 km so the solved
// terminal vector and 75--100% correction reserve own the endpoint.
SET TERMINAL_MAIN_TRACKING_THRUST_FRACTION TO 0.84.
SET TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT TO 20000.
SET TERMINAL_MAIN_TRACKING_FULL_HEIGHT TO 15000.
SET TERMINAL_MAIN_TRACKING_FADE_START_HEIGHT TO 7000.
SET TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT TO 5000.
// Above the nominal 75% floor, release correction magnitude only after the
// physical thrust axis has caught the requested direction.  Run 17 commanded
// almost full throttle while the stage was still about 30 degrees behind;
// that surplus then acted vertically and stopped the descent above 4 km.
SET MAIN_CORRECTION_FULL_ALIGNMENT_DEGREES TO 8.
SET MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES TO 20.
SET TERMINAL_VELOCITY_CONE_DEGREES TO 30.
// Run 59 measured 29.24 degrees at a 26.2-degree command.  Use 25.5 degrees
// with the current 16% authority to recover lateral allocation while retaining
// about one degree of physical tracking margin below the mandatory 30.0-degree
// nozzle limit.
// Below 5 m/s the cone axis changes from surface retrograde to local up
// (equivalently, nozzle axis toward local down).
SET TERMINAL_COMMAND_CONE_DEGREES TO 25.5.
SET TERMINAL_VELOCITY_CONE_MIN_SPEED TO 5.
// Main ignition is permitted only after the physical thrust axis has settled
// near surface retrograde.  Real unpowered telemetry showed an excellent
// 0.15--0.80 degree alignment but 0.2--6.6 deg/s single-frame controller noise;
// the former 2 deg/s for 1 s interlock therefore reset forever and caused a
// zero-thrust water impact.  Five deg/s for 0.15 s rejects a real slew while
// allowing the already aligned vehicle to light.  The mandatory physical
// nozzle cone remains independently audited at 30.0 degrees every powered
// physics frame.
SET MAIN_IGNITION_ALIGNMENT_DEGREES TO 3.
SET MAIN_IGNITION_MAX_ANGULAR_RATE_DEG TO 5.
SET MAIN_IGNITION_STABLE_DWELL_SECONDS TO 0.15.
// Do not turn a recoverable, hard-cone-safe state into a zero-thrust water
// impact merely because the preferred cone-edge attitude cannot be reached.
// After this small additional drop, ignition may use the physical retrograde-
// safe axis with a still-bounded angular rate while steering keeps pursuing the
// preferred lead.  The observer continues to audit the actual 30-degree cone.
SET MAIN_IGNITION_ALIGNMENT_FALLBACK_DROP TO 350.
SET MAIN_IGNITION_FALLBACK_MAX_ANGULAR_RATE_DEG TO 10.
// Complete this load-cone-safe lead while still unpowered.  Lighting only after
// the physical stage is on this axis avoids spending the main burn on a
// high-dynamic-pressure attitude slew.
SET MAIN_IGNITION_PRELEAD_DEGREES TO 22.
// Once the continuous main burn begins it may not return to zero before
// capture.  Below the 2 km gate this small floor only prevents an accidental
// off/on transition; the normal gravity-compensating command is higher.
SET TERMINAL_MIN_CONTINUOUS_THROTTLE TO 0.03.
SET TERMINAL_WAYPOINT_HEIGHT TO 2000.
// Aim at the centre of the mandatory 150--200 m/s band.  Runs 65--66 used a
// deliberately fast 225 m/s planning endpoint to compensate for excessive
// thrust; that compensation is invalid after correcting the actuator scale.
SET TERMINAL_WAYPOINT_VERTICAL_SPEED TO 175.
SET TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED TO 150.
SET TERMINAL_WAYPOINT_MAX_VERTICAL_SPEED TO 200.
SET TERMINAL_WAYPOINT_MAX_HORIZONTAL_SPEED TO 5.
// Run 20 showed that a vertical-only trigger at 0.82 still lit at 18.37 km
// with 930 m/s horizontal speed and only 15.1 km of range. Keep this vertical
// margin as one half of the two-dimensional gate; main.ks now also lights when
// the independently solved horizontal stopping footprint becomes critical.
// Run 35 identified the missing low-atmosphere footprint: below roughly
// 16 km, body/grid-fin drag supplies 8--16 m/s^2 of additional horizontal
// braking, while the mandatory velocity cone forbids thrusting forward to
// undo an early stop.  Move the solved gate about 2 km lower so the remaining
// powered trajectory is one-way braking all the way to the 2 km waypoint.
// Runs 41--43 showed a repeatable 0.41--0.84 km downrange undershoot while
// retaining ample throttle and propellant authority.  Lowering the vertical
// gate by about 0.30 km adds roughly 0.45 km of unpowered downrange travel;
// the continuous burn still has 25% correction headroom above its 75% design
// point to recover the required 150--200 m/s state at 2 km.
SET TERMINAL_IGNITION_SAFETY TO 0.729.
SET TERMINAL_IGNITION_MARGIN TO 50.
SET RETURN_ENGINE_ACCEL_UPDATE_SECONDS TO 0.5.
// During the powered return, infer aerodynamic acceleration from the measured
// surface-velocity derivative after subtracting gravity and live engine
// thrust.  Run 16 proved that treating this force as zero over-braked the
// vertical trajectory and let an unmodelled side force build kilometre-scale
// cross-track error.
SET POWERED_AERO_ACCEL_FILTER TO 0.12.
// Vertical drag really reached roughly 7--13 m/s^2 in run 17.  Side-force
// cancellation is capped more tightly because an attitude transient can make
// the live-thrust residual look like a large horizontal aerodynamic force.
SET POWERED_VERTICAL_AERO_MAX_ACCEL TO 20.
SET POWERED_HORIZONTAL_AERO_MAX_ACCEL TO 10.
// Measured time for the long empty stage to turn a new lateral command into
// acceleration.  Include it in both ignition height and powered footprint.
// The continuous main burn now waits for physical cone alignment before
// ignition.  Retain only control-loop latency here; the former 1.5 s attitude
// allowance double-counted a slew that had already completed during coast.
SET TERMINAL_GUIDANCE_RESPONSE_SECONDS TO 0.35.
SET TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS TO 0.35.
// The fixed Kerbin-surface endpoint law is retuned against the run-35
// identified-drag replay.  Real KSP remains authoritative.
SET TERMINAL_HORIZONTAL_LEAD_SECONDS TO 0.
// Run 90's joint-state audit showed that the retuned 10/5.7 law becomes
// prograde below roughly 8 km while the physical stopping boundary is already
// infeasible.  Restore the exact finite-time cubic endpoint coefficients from
// the equation documented in main.ks: a = 6*x/T^2 - 4*v/T.
SET TERMINAL_WAYPOINT_POSITION_COEFFICIENT TO 6.0.
SET TERMINAL_WAYPOINT_VELOCITY_COEFFICIENT TO 4.0.
// The long stage can follow the point-of-ignition downrange axis much faster
// than it can reverse a small cross-track cone-edge command.  Run 40 crossed
// the ship latitude near 7 km but continued accelerating cross-track for
// several seconds.  Keep the proven downrange coefficient and damp only the
// frozen cross-track component earlier.
// Run 56 showed that damping the cross-track velocity as aggressively as the
// downrange velocity stopped the north/south translation about 120 m before
// the ship.  Keep the finite-time position term, but retain cross-track motion
// longer so the vehicle reaches the centre before it is brought to rest.
SET TERMINAL_WAYPOINT_CROSS_VELOCITY_COEFFICIENT TO 6.0.
SET TERMINAL_WAYPOINT_CROSS_POSITION_COEFFICIENT TO 13.0.
// If unavoidable cone braking makes the measured downrange speed fall behind
// the zero-speed endpoint profile, rotate toward the cone edge that removes
// the least downrange speed.  Fade this allocation out before the final brake.
SET TERMINAL_ALONG_SPEED_DEFICIT_ARM TO 10.
SET TERMINAL_ALONG_SPEED_DEFICIT_BLEND TO 30.
SET TERMINAL_ALONG_COAST_FADE_START_HEIGHT TO 7000.
SET TERMINAL_ALONG_COAST_END_HEIGHT TO 5000.
SET TERMINAL_ALONG_COAST_MIN_REQUEST TO 5.
// Run 86 proved that surface-retrograde streamlining still removes roughly
// 17--24 m/s^2 along track after the stage falls below its synchronized speed;
// it reaches the 2 km neighbourhood short of range but still carrying about
// 145 m/s horizontally.  Run 89 proved that pointing toward the forward legal
// cone edge reduces engine braking but increases body aerodynamic braking by
// more, so surface-retrograde alignment is the true minimum-net-braking
// actuator when speed is deficient.
// The deficient/excess scalar reference is superseded by the joint cubic
// endpoint law.  Run 90 proved that it labels an unreachable stopping state as
// speed-deficient and therefore suppresses the braking the endpoint requires.
SET TERMINAL_ALONG_COAST_ENABLED TO FALSE.
// Run 98 reaches the 12/10/8/6 km planes with required downrange stopping
// acceleration of about 22.5/23.8/26.0/30.3 m/s^2, while the streamlined
// branch realises only 18.4--20.3.  Run 89 identified a second legal actuator:
// the forward/upright velocity-cone edge reduces engine braking but adds enough
// body drag to realise roughly 26--28 m/s^2 without Run 96's large upward-lift
// branch.  Close that measured reachability gap continuously; do not schedule
// it from the cubic reference's misleading speed-deficit label.  Run 99 also
// proved that blend zero must explicitly own surface retrograde: returning to
// the cubic command left the body trapped on the upright high-drag branch even
// after the scalar blend had released completely.
SET TERMINAL_ALONG_AERO_BRAKE_ENABLED TO TRUE.
SET TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT TO 14500.
SET TERMINAL_ALONG_AERO_BRAKE_FULL_HEIGHT TO 13500.
SET TERMINAL_ALONG_AERO_BRAKE_ACCEL_GAIN TO 8.
SET TERMINAL_ALONG_AERO_BRAKE_MARGIN TO 1.03.
SET TERMINAL_ALONG_AERO_BRAKE_ERROR_DEADBAND TO 0.5.
SET TERMINAL_ALONG_AERO_BRAKE_BUILD_RATE TO 0.18.
SET TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE TO 0.12.
SET TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND TO 0.85.
// Run 48 measured 16--63 m/s of along-track speed excess from 20 to 10 km,
// while the commanded velocity-cone angle remained only 2.8--15.5 degrees and
// the 25% correction reserve was not fully used.  Symmetrically allocate that
// reserve toward the maximum-braking cone edge only while the stage is faster
// than its finite-time reference.  The blend returns to the nominal trajectory
// before any forbidden forward/reversal request can appear.
// Run 95 leaves too much horizontal energy while becoming vertically slow.
// Re-enable only the legal cone-edge actuator; main.ks now drives it from a
// physical stopping-demand/authority ratio, not the rejected 2*x/T reference.
SET TERMINAL_ALONG_BRAKE_ENABLED TO FALSE.
// Runs 87-88 proved that withholding all authority until 12.5 or 18.5 km traps
// the controller in a self-sustaining maximum-brake branch.  Start before the
// measured 22.6-23.1 km main ignition, but use only half of the 25% correction
// reserve initially.  Continuously release the other half from 10 to 8 km,
// where the remaining range and speed require full terminal authority.
SET TERMINAL_ALONG_BRAKE_START_HEIGHT TO 25000.
SET TERMINAL_ALONG_BRAKE_FULL_HEIGHT TO 24000.
SET TERMINAL_ALONG_BRAKE_EARLY_MAX_BLEND TO 0.5.
SET TERMINAL_ALONG_BRAKE_LATE_RAMP_START_HEIGHT TO 10000.
SET TERMINAL_ALONG_BRAKE_LATE_FULL_HEIGHT TO 8000.
SET TERMINAL_ALONG_BRAKE_SPEED_EXCESS_ARM TO 10.
SET TERMINAL_ALONG_BRAKE_SPEED_EXCESS_BLEND TO 30.
SET TERMINAL_ALONG_BRAKE_MIN_REQUEST TO 5.
// The final limit remains one so the late height ramp can reach the full
// correction reserve without a throttle step or any engine on/off cycling.
SET TERMINAL_ALONG_BRAKE_MAX_BLEND TO 1.0.
// Reachability pressure is raw v^2/(2r) divided by the live full-throttle
// engine projection plus measured aerodynamic braking.  Blend continuously
// before the boundary becomes infeasible; height only arms the allocator.
SET TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT TO 20000.
SET TERMINAL_ALONG_BRAKE_REACHABILITY_FULL_HEIGHT TO 19000.
SET TERMINAL_ALONG_BRAKE_PRESSURE_ARM TO 0.75.
SET TERMINAL_ALONG_BRAKE_PRESSURE_FULL TO 1.0.
SET TERMINAL_ALONG_BRAKE_REACHABILITY_MIN_RANGE TO 100.
// Horizontal braking may add vertical thrust only while descent is faster than
// this smooth quarter-power floor.  This prevents lateral work from holding the
// stage near 110 m/s at 6-7 km while retaining full braking authority on entry.
SET TERMINAL_DESCENT_COUPLING_BAND TO 50.
// Finish the horizontal plan at the formal 2 km gate, just short of the
// ship centre.  A replay of the historical 6 km internal endpoint stopped the
// modern constrained trajectory about 4 km short, so it cannot be reused with
// the current continuous-burn and nozzle-cone requirements.
SET TERMINAL_HORIZONTAL_PLAN_END_HEIGHT TO 2000.
// Restore the fixed Run-52 footprint reference.  The later 20--160 m dynamic
// offset moved the internal endpoint about 115 m toward the stage and, through
// the finite-time feedback, put the nominal 75% vector on the load-cone edge
// as high as 15 km.  The physical hook centre is still audited independently
// to <=10 m at 2 km; this internal offset is not an acceptance radius.
// The Run-66 main-burn trace crossed the frozen downrange axis about 4.5 km
// ASL while still powered.  The identified 30 km point-mass sweep moves the
// Hermite approach allowance forward so the finite-time law does not plan an
// already-short stopping footprint.
SET TERMINAL_WAYPOINT_APPROACH_OFFSET TO 75.
SET TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED TO 644.
SET TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN TO 0.
SET TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET TO 75.
SET TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET TO 75.
SET TERMINAL_CAPTURE_ALIGN_ARM_HEIGHT TO 2000.
// Retained as a documented planning altitude.  main.ks now holds surface
// retrograde throughout every coast interval after entry cutoff, including the
// intervals between the three checkpoints, so no late attitude reversal is
// required before the continuous main burn.
// Start the cone-edge attitude during the checkpoint coast, before either
// component of the two-dimensional ignition gate can fire. Correction pulses
// temporarily own steering, but use almost the same load-cone-edge direction.
SET TERMINAL_MAIN_PREALIGN_HEIGHT TO 30000.
// A distinct one-way high-energy handoff removes the last few hundred metres
// of horizontal velocity without allowing a centre crossing to re-arm a fast
// pursuit in the opposite direction.
SET TERMINAL_HIGH_ENERGY_BRAKE_ARM_HEIGHT TO 6000.
SET TERMINAL_HIGH_ENERGY_BRAKE_RANGE TO 400.
// This does not enlarge the normal capture gate. It applies only after the
// frozen cross-track state has crossed the target and is moving farther away.
SET TERMINAL_CROSS_TRACK_BRAKE_ERROR TO 50.
SET TERMINAL_CROSS_TRACK_BRAKE_RANGE TO 2500.
SET TERMINAL_HIGH_ENERGY_BRAKE_POSITION_GAIN TO 0.50.
SET TERMINAL_HIGH_ENERGY_BRAKE_MAX_SPEED TO 30.
SET TERMINAL_HIGH_ENERGY_BRAKE_VELOCITY_GAIN TO 1.50.
SET TERMINAL_HIGH_ENERGY_BRAKE_MAX_ACCEL TO 55.
SET TERMINAL_HIGH_ENERGY_BRAKE_SETTLE_HEIGHT TO 2200.
// Main ignition and the powered horizontal loop use the measured attitude lag
// directly.  Pre-ignition alignment removes the former post-lighting slew.
// Run 29 crossed the target with about 1.6 km of along-track overshoot. At
// its 930 m/s ignition speed that is a 1.7 s unmodelled actuator footprint;
// use the independently observed roughly two-second long-stage response.
SET TERMINAL_MAIN_ATTITUDE_RESPONSE_SECONDS TO 2.0.
// The raw v^2/(2a) gate accounts for engine braking but not the 8--16 m/s^2
// body/grid-fin drag identified below 16 km.  Run 36 therefore fired at a
// 26.62 km horizontal range and reached 2 km 4.09 km short.  Its measured
// useful footprint is 22.3 / 26.62 = 0.84 of that trigger.  Runs 46--50 showed
// that 0.792--0.810 delayed ignition to 20.0--21.1 km and crossed the ship high.
// The best real-flight point in the same history used 0.863: it began the main
// burn near 21.9 km and reduced the 2 km position error to 218 m. Restore that
// measured gate while the independent vertical feasibility gate remains live.
SET TERMINAL_MAIN_HORIZONTAL_STOP_SAFETY TO 0.863.
// A positive predicted miss along the current velocity means the planned
// continuous burn stops short and could be repaired only with forbidden
// prograde thrust.  Run 67 predicted +4.42 km near ignition and measured
// +4.12 km at the 2 km plane.  Run 68 then showed that waiting for only 0.30 km
// left insufficient time to remove horizontal speed and overshot by 3.3 km.
// Run 91 proved that treating the 29 km horizontal boundary as an immediate
// continuous-burn command enters a minimum-thrust branch that stops about
// 4.7 km short.  Keep the vertical-ready boundary as the handoff into the
// non-coastable 75-100% segment; bounded checkpoints below shape vertical
// energy before that handoff.
SET TERMINAL_MAIN_MAX_PREDICTED_PROGRADE_MISS TO 3000.
// Run 93 proved the intermediate scalar handoff cannot make zero position and
// zero speed coincide under the nonzero aerodynamic-braking floor.  Return to
// the high-horizontal-energy late branch; explicit bounded checkpoints below
// now own the missing vertical-energy correction before continuous ignition.
SET TERMINAL_MAIN_VERTICAL_READY_MARGIN TO 400.
// Run 30 showed why the former instantaneous-velocity-axis stopping law cannot
// own a two-dimensional endpoint: its axis rotated near the ship, starving the
// fixed cross-track error and leaving 1.70 km at the formal 2 km gate.  Keep
// its parameters for A/B diagnostics, but use the fixed surface-frame law.
SET TERMINAL_MAIN_DIRECT_STOP_ENABLED TO FALSE.
SET TERMINAL_MAIN_DIRECT_STOP_DECEL_GAIN TO 1.00.
SET TERMINAL_MAIN_DIRECT_STOP_CROSS_TIME_COEFFICIENT TO 12.0.
SET TERMINAL_MAIN_DIRECT_STOP_CROSS_MAX_ACCEL TO 10.
SET TERMINAL_MAIN_DIRECT_STOP_MIN_RANGE TO 100.
// Run 52 followed the finite-time reference closely above 8 km, but that
// reference did not account for the measured loss of physical cone authority
// at peak dynamic pressure. It reached 3 km with 177 m/s along-track speed and
// crossed the ship before the formal waypoint. Add a one-way stop-distance
// constraint on the downrange axis frozen at main ignition. Unlike the rejected
// direct-stop A/B law, this axis cannot rotate after centre crossing and it
// leaves the independently damped cross-track command intact.
// Run 65 proved that a height-only brake can demand deceleration while the
// measured stage is already slower than the joint endpoint reference. Use the
// frozen-axis reachable stopping envelope instead: its demand is derived from
// remaining range, closing speed, and the measured long-stage response, and it
// is one-way so a centre crossing cannot re-arm it in the opposite direction.
// Runs 69--71 prove the smooth endpoint law bifurcates when its initial cubic
// acceleration changes sign: positive/prograde demand is unavailable inside
// the load cone, while the opposite branch brakes broadside far too early.
// Run 72 also proves that applying v^2/(2r) from 23 km is not a coast-then-brake
// switch: it immediately selected the broadside branch, crossed the ship near
// 5 km, and nearly hovered 2.5 km beyond it. Arm only in the lower-atmosphere
// window, then require the raw stopping demand to fit inside the live full-
// throttle/cone/aerodynamic authority estimate before latching it one-way.
// The late fixed-axis latch did not become feasible in Run 90 and cannot repair
// a vector-authority deficit first visible above 20 km.  Keep its diagnostics
// in source, but let the joint endpoint controller own downrange acceleration.
SET TERMINAL_MAIN_FIXED_AXIS_STOP_ENABLED TO FALSE.
SET TERMINAL_MAIN_FIXED_AXIS_STOP_GAIN TO 1.00.
SET TERMINAL_MAIN_FIXED_AXIS_STOP_MIN_RANGE TO 100.
SET TERMINAL_MAIN_FIXED_AXIS_RESPONSE_SECONDS TO 2.5.
SET TERMINAL_MAIN_FIXED_AXIS_ARM_HEIGHT TO 8000.
SET TERMINAL_MAIN_FIXED_AXIS_AUTHORITY_MARGIN TO 0.90.
// The centre-stop envelope is not the synchronized 2 km endpoint.  Run 85
// committed it while 153 m/s slower than the live reference, forcing an
// already-late trajectory to brake.  It may latch only from the fast side.
SET TERMINAL_MAIN_FIXED_AXIS_SYNC_EXCESS_ARM TO 10.
// Run 76 exposed an independent cross-track fly-through: at 12 km the stage
// was still being accelerated toward the cross-axis centre, then reached that
// centre near 6 km with 14 m/s of residual speed.  Freeze the initial cross
// direction and reserve only the finite stopping acceleration needed after a
// bounded attitude-response allowance.  The latch is one-way so a bad model
// remains observable instead of commanding a second pass after centre crossing.
SET TERMINAL_MAIN_CROSS_STOP_ENABLED TO TRUE.
SET TERMINAL_MAIN_CROSS_STOP_GAIN TO 1.00.
SET TERMINAL_MAIN_CROSS_STOP_MIN_RANGE TO 5.
SET TERMINAL_MAIN_CROSS_STOP_RESPONSE_SECONDS TO 2.5.
SET TERMINAL_MAIN_CROSS_STOP_ARM_HEIGHT TO 12500.
SET TERMINAL_MAIN_CROSS_STOP_MIN_SPEED TO 2.0.
SET TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL TO 15.
// Run 85 entered the 3 m frozen-axis deadband with 2.74 m/s still closing.
// Continue damping velocity only; never reacquire position after the first
// pass.  A 0.5/s pole is deliberately slower than the identified attitude
// response and limits the expected deadband crossing to several metres.
SET TERMINAL_MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN TO 0.5.
// Run 80 confirmed a high-q control reversal on the frozen cross axis.  Below
// about 7 km, outward engine steering produced a still larger inward body
// force.  Begin the opposite, inward attitude command at 8 km so the several-
// second physical slew is complete when the aerodynamic force becomes the
// dominant (and therefore outward-braking) actuator.
SET TERMINAL_MAIN_CROSS_AERO_BRAKE_ENABLED TO TRUE.
SET TERMINAL_MAIN_CROSS_AERO_BRAKE_START_HEIGHT TO 8000.
SET TERMINAL_MAIN_CROSS_AERO_BRAKE_MIN_ENGINE_ACCEL TO 5.
// Run 81 confirmed the reversed control sign, but a fixed 5 m/s^2 inward
// attitude kept producing roughly 2.9-3.3 m/s^2 of net outward braking after
// the frozen-axis requirement had fallen below 1 m/s^2.  Fade the cross
// attitude with the actual stopping demand so it reaches zero before closing
// speed reverses; the one-way latch remains the independent no-second-pass
// guard.  Five m/s^2 is the full-command demand identified in that run.
SET TERMINAL_MAIN_CROSS_AERO_BRAKE_FULL_DEMAND TO 5.
// The old one-way fixed-axis latch asked for the opposite maximum-engine-brake
// edge and would override the Run-99/100 two-endpoint aerodynamic allocator if
// its feasibility bit happened to latch.  The measured reachability allocator
// now owns the downrange attitude continuously; keep the old branch only as an
// inert diagnostic so Run 100 remains attributable to one actuator coordinate.
SET TERMINAL_MAIN_FIXED_AXIS_EDGE_STEERING_ENABLED TO FALSE.
// Keep the former height-indexed engine override only as an A/B diagnostic.
// It is applied after aerodynamic compensation, so its nominal -25 m/s^2
// engine request combined with roughly -10 m/s^2 of measured drag and stopped
// Run 65 about 0.88 km short of the ship.
SET TERMINAL_MAIN_ALONG_ENGINE_BRAKE_ENABLED TO FALSE.
SET TERMINAL_MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT TO 6000.
SET TERMINAL_MAIN_ALONG_ENGINE_BRAKE_FULL_HEIGHT TO 5000.
SET TERMINAL_MAIN_ALONG_ENGINE_BRAKE_ACCEL TO 25.

// The command cone alone cannot guarantee the physical cone when horizontal
// velocity approaches zero: the live retrograde axis can rotate faster than
// the long stage. Blend the steering target toward the live cone centre before
// the measured angle reaches the 30-degree hard boundary. Throttle is unchanged.
SET TERMINAL_ACTUAL_CONE_GUARD_START_DEGREES TO 20.
SET TERMINAL_ACTUAL_CONE_GUARD_FULL_DEGREES TO 24.

// Once the formal 2 km state has been reached, only small centring corrections
// remain.  Taper the commanded velocity-cone excursion before the horizontal
// velocity approaches zero; otherwise a changing retrograde azimuth can make
// the long empty stage physically overshoot the 30-degree nozzle limit even
// though each instantaneous command is inside it.
SET TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT TO 500.
SET TERMINAL_LOW_ALT_COMMAND_CONE_DEGREES TO 8.
// Run 79 removed the former cross-track offset and exposed a dense-air
// sideslip penalty: between 6 and 3 km, a 9--16 degree physical cone angle
// generated 14--17 m/s^2 of upward aerodynamic acceleration.  Begin reducing
// the commanded cone before that force builds, while keeping the same 8 degree
// near-field authority and the unchanged 30 degree physical hard limit.
SET TERMINAL_DENSE_AIR_CONE_START_HEIGHT TO 8000.
SET TERMINAL_DENSE_AIR_CONE_END_HEIGHT TO 4000.
SET TERMINAL_DENSE_AIR_COMMAND_CONE_DEGREES TO 8.
// Above 2 km the 75% nominal floor is a hard trajectory constraint.  Run 64
// proved that lowering throttle to preserve descent merely hides an infeasible
// vertical plan and violates G-03.  Vertical energy is now owned by the common
// finite-time target and the later horizontal-brake schedule; there is no
// pre-waypoint throttle-cap escape path.
// Below the 300 m/s load-cone threshold, lead the lagging attitude slightly
// through the horizon once descent approaches the 200 m/s upper gate.  The
// real stage otherwise keeps an upward thrust component for another 3-5 s and
// reaches 2 km at only ~80 m/s despite a nearly horizontal command.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_SPEED TO 300.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_MAX_SURFACE_SPEED TO 500.
SET TERMINAL_VERTICAL_RECOVERY_STEERING_DEGREES TO 22.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_HEIGHT TO 12000.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_MAX_SURFACE_SPEED TO 2000.
SET TERMINAL_VERTICAL_RECOVERY_PRELEAD_DEGREES TO 22.
// Release the vertical main burn only when a 270-285 m/s ballistic 2 km state
// is reachable.  The wider band covers one 0.2 s telemetry/control sample;
// the simultaneous horizontal gate still keeps total speed below the 300 m/s
// load-cone boundary.  Side-slip and the final pulse consume the extra speed.
SET TERMINAL_WAYPOINT_COAST_ERROR TO 10.
// The historical waypoint-coast state was designed for sparse 75% pulses.
// It is incompatible with the mandatory continuous main burn: its zero-force
// steering target was subsequently executed at the new 75% throttle floor.
// Keep the complete continuous trajectory controller in charge through 2 km.
SET TERMINAL_WAYPOINT_COAST_ENABLED TO FALSE.
SET TERMINAL_WAYPOINT_COAST_HORIZONTAL_SPEED TO 4.5.
SET TERMINAL_WAYPOINT_COAST_MAX_VERTICAL_SPEED TO 285.
// This is the unbraked ballistic endpoint, not the required 2 km error.
// Real telemetry reaches the vertical-speed window with enough range to stop,
// but its coast-only endpoint has already crossed the ship by about 1.3 km.
// The trim controller below still has to finish inside the strict 30 m gate.
// Do not hand an infeasible endpoint to the fixed-impulse trim.  The former
// 1.8 km gate armed with only 3.8 s remaining, then accelerated toward the ship
// before an unavoidable late reversal.  Keep the handoff inside 1 km.
SET TERMINAL_WAYPOINT_COAST_ENTRY_ERROR TO 1000.
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
// Arm one telemetry sample before the formal 30 m gate.  At 13-27 m/s a
// 0.2 s guidance pass can otherwise move from just outside 30 m to the far
// side before the one-way latch observes the entry.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_ENTRY_ERROR TO 10.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_SAFETY TO 1.25.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_VELOCITY_GAIN TO 1.5.
SET TERMINAL_WAYPOINT_CENTER_BRAKE_MIN_RANGE TO 5.
// Shape the one-way approach without stopping short of the 2 km plane.  A
// 1.25 multiplier reduced the measured 3.2 km closing speed from 96 to
// 16 m/s while 175 m still remained.  The 0.70 spatial braking law reaches
// the 30 m latch with a small velocity remainder, so the frozen endpoint law
// can finish the stop without a 180-degree attitude reversal.
SET TERMINAL_WAYPOINT_APPROACH_BRAKE_GAIN TO 0.70.
// A full guidance-cycle 75% pulse changes the nearly empty stage by more than
// the remaining horizontal speed.  Keep every non-zero command at exactly
// 75%, but shorten each endpoint quantum from the measured remaining delta-v.
SET TERMINAL_WAYPOINT_MICRO_PULSE_SPEED TO 30.
SET TERMINAL_WAYPOINT_MICRO_PULSE_SECONDS TO 0.07.
// Fire endpoint impulses only after the actual horizontal thrust direction is
// close to the measured residual-velocity stop direction.  This makes every
// pulse reduce horizontal kinetic energy despite azimuth steering lag.
SET TERMINAL_WAYPOINT_ENDPOINT_ALIGNMENT_DEGREES TO 20.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_MAX_ACCEL TO 60.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_GAIN TO 3.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_MIN_ACCEL TO 5.
SET TERMINAL_WAYPOINT_ENDPOINT_TRIM_TOLERANCE TO 0.5.
// Stop the measured velocity rather than baking a flight-specific slew offset
// into its target.  Up to three newly measured directions remove the small
// perpendicular residual left by a finite first impulse without position chase.
SET TERMINAL_WAYPOINT_ENDPOINT_VELOCITY_SCALE TO 0.
SET TERMINAL_WAYPOINT_ENDPOINT_MAX_PULSES TO 3.
// Usually the endpoint stop finishes high enough for the broadside-to-upright
// slew to settle ballistically.  If that slew starts late, however, its aero
// side force can rebuild more than 8 m/s before the 2 km plane.  Correct only
// that exceptional residual with one moderate-tilt pulse; normal 0-7 m/s slews
// remain engine-off and avoid unnecessary vertical impulse.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ENABLED TO TRUE.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_HEIGHT TO 2000.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ARM_TILT TO 45.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ARM_SPEED TO 8.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_TILT TO 60.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_ALIGNMENT_DEGREES TO 10.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_VELOCITY_SCALE TO 0.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MAX_ACCEL TO 20.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_GAIN TO 2.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_ACCEL TO 3.
SET TERMINAL_WAYPOINT_POST_UPRIGHT_TOLERANCE TO 0.5.
// Leave margin for the repeatable drift added while the broadside stage slews
// upright.  The formal 2 km limit remains 5 m/s after that attitude change.
SET TERMINAL_WAYPOINT_FINAL_COAST_HORIZONTAL_SPEED TO 1.5.
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
// After the 40 km thermal burn, coast at zero throttle.  At most one short
// footprint-correction pulse is allowed at each checkpoint; terminal braking
// supersedes a pulse whenever its solved ignition gate is reached.
// The 30 m/s^2 actuator and calibrated late main gate leave all three checks
// strictly between the ~36 km entry cutoff and main ignition.
// Run 33 measured the full correction and alignment drops after the 35.95 km
// entry cutoff.  These triggers retain three distinct bounded checks and move
// the final pulse early enough for main ignition near the simulator-validated
// 25.5--26.5 km window instead of the failed 23.36 km handoff.
SET MIDCOURSE_CHECKPOINT_1_HEIGHT TO 37500.
SET MIDCOURSE_CHECKPOINT_2_HEIGHT TO 35000.
SET MIDCOURSE_CHECKPOINT_3_HEIGHT TO 32500.
// Run 19 fired all three 2.2 s pulses, yet the predicted 2 km miss only fell
// from 3.24 km to 1.60 km.  Three 3.0 s one-shot burns remain short and
// discrete, fit between the 3 km-spaced gates, and give each check enough
// impulse to converge before the continuous main segment takes ownership.
SET MIDCOURSE_PULSE_SECONDS TO 3.0.
SET MIDCOURSE_PREDICTED_ERROR TO 100.
// Run 25 completed all three 3 s / 30% corrections but still entered the main
// burn with 1.93 km of predicted miss.  Permit each same-duration checkpoint
// to use the engine's load-cone-safe horizontal authority; the live 100 m miss
// gate still ends a pulse early, so this remains two or three bounded impulses
// rather than another continuous burn.
SET MIDCOURSE_MAX_HORIZONTAL_ACCEL TO 20.
SET MIDCOURSE_MAX_THROTTLE TO 0.67.
SET MIDCOURSE_MAX_DELTA_V TO 60.
SET MIDCOURSE_MAX_HEIGHT_DROP TO 1900.
// An unpowered checkpoint may not spend an unlimited amount of altitude
// acquiring attitude.  Run 34 lost 12.7 km at checkpoint 2 because steering
// targeted the correction vector while its ignition gate targeted the load-
// cone centre.  The corrected vector-relative gate should normally settle
// well inside this budget; this remains a fail-safe for high-q authority loss.
SET MIDCOURSE_MAX_ALIGNMENT_HEIGHT_DROP TO 1200.
SET MIDCOURSE_MIN_FUEL_FRACTION TO 0.12.
// Do not turn an infeasible forward/cross-range request into a retrograde burn
// merely because it was projected into the mandatory cone.  A positive cosine
// of 0.25 retains useful cross-track corrections; negative run-35 along-track
// projections are recorded and skipped without firing.
SET MIDCOURSE_MIN_REQUEST_EFFECTIVENESS TO 0.25.
// Apply only the delta-v that removes predicted endpoint miss.  Do not fold
// the planned main-burn velocity change into a checkpoint: that consumed the
// whole acceleration limit along-track in run 27 and starved cross-track.
// A pure cross-track request is projected into the surface-velocity cone, so
// only about sin(20 deg) of its magnitude is physically lateral.  Run 59's
// unit gain therefore let the feasible miss grow from 248 to 332 m across all
// three checks.  Compensate that known projection while retaining the same
// three bounded 3 s pulses and existing delta-v/fuel limits.
SET MIDCOURSE_MISS_GAIN TO 2.50.
// Run 96 showed that carrying angle of attack into dense air exchanges a small
// horizontal benefit for a much larger vertical-energy loss.  Runs 92/95 show
// the useful alternative: keep the same high-altitude horizontal checkpoint
// requests, but reduce their vertical component.  Across Run 95's measured
// 5.24 s of powered checkpoint time, lowering 0.75 g to 0.20 g removes roughly
// 25-30 m/s of premature upward impulse while retaining the horizontal request,
// three-second bounds, throttle cap, delta-v cap, and later engine-off coast.
SET MIDCOURSE_VERTICAL_THRUST_G TO 0.20.
SET MIDCOURSE_VERTICAL_TARGET_IGNITION_SPEED TO 600.
SET MIDCOURSE_VERTICAL_ERROR_DEADBAND TO 15.
// The endpoint predictor moves as vertical energy changes.  Runs 95/97 show
// 7-10 m/s of surface-horizontal recovery between checkpoint 3 and physical
// main alignment, so own a measured 960 m/s pulse-end state instead of letting
// a sign-changing predicted miss withdraw the high-altitude braking impulse.
SET MIDCOURSE_HORIZONTAL_TARGET_SPEED TO 960.
SET MIDCOURSE_HORIZONTAL_TARGET_DEADBAND TO 1.
// Once the explicit vertical-residual phase has run, its completion surface
// must not recede as the corrected vertical speed lowers the legacy ignition
// estimate.  Runs 90/92 place the repeatable late main commitment here.
SET MIDCOURSE_SHAPED_MAIN_HANDOFF_HEIGHT TO 23600.
// A checkpoint may light only when the physical stage is close to the actual
// requested correction vector and angular motion has settled.  The requested
// vector itself is already projected into the mandatory load cone.
SET MIDCOURSE_IGNITION_CONE_DEGREES TO 20.
SET MIDCOURSE_IGNITION_MAX_ANGULAR_RATE_DEG TO 10.

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
