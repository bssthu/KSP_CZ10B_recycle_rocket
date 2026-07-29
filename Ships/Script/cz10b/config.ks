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
// Run 107 reached the 2 km plane inside both horizontal-speed and nozzle-cone
// limits, but removed about 300 m/s of descent below 6 km.  Preserve the
// checkpoint and high-altitude main envelopes, then continuously reduce the
// normalized engine maximum in the dense low segment.  This changes the
// physical available-thrust envelope, not the mandatory 75% throttle floor;
// the mission observer continues to require available recovery TWR above one.
// Run 121's 16.6 m/s2 envelope passes descent by only 0.21 m/s.  The explicit
// low-speed centre settle below adds a more upright final attitude, so retain
// robust descent margin by moving the independent envelope another small step
// while available TWR remains well above one.
SET RETURN_ENGINE_LOW_ALT_MAX_ACCEL TO 16.3.
SET RETURN_ENGINE_LOW_ALT_RAMP_START_HEIGHT TO 6500.
SET RETURN_ENGINE_LOW_ALT_RAMP_FULL_HEIGHT TO 5500.
// Runs 126--129 all retained roughly 90--105 m/s of descent at the platform:
// the 16.3 m/s2 envelope is needed to preserve the formal 2 km descent band,
// but its 75% floor supplies only about 12.2 m/s2 vertically after crossing.
// Restore the original return authority continuously and strictly below the
// evaluated plane.  The blend is exactly zero at 2 km, so it cannot conceal
// or alter formal waypoint compliance.
SET RETURN_ENGINE_POST_WAYPOINT_MAX_ACCEL TO 21.5.
SET RETURN_ENGINE_POST_WAYPOINT_RAMP_FULL_HEIGHT TO 1500.
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
// Run 59 proved that 10% authority could not follow the late cone-edge stop.
// With the fins physically mounted, Run266 then measured a persistent
// 6--8-degree high-altitude tracking lag even after the command cone was
// widened, while the realised attitude and endpoint barely moved.  Run267's
// 18% high-altitude value cut formal position error by 175.10 m, and Run268's
// 20% value moved the signed residual another 104.18 m without approaching
// the physical cone.  Their local secant crosses zero at about 22.74%.
// Run269 sampled 22.7% only above 4.5 km, tapered it back to the established
// 16% by 3 km, and still restored the 0.625% flight setting below 1.95 km.
// Runs297--298 now leave the extreme-low entrance family about +63 m on the
// approach side of the formal plane. The clean Runs267--269 bracket moved
// signed formal position by about 52 m per authority percentage point.
// Run299 therefore backs the high-altitude value off by 1.2 points, the local
// inverse needed to remove that residual. The taper and independent
// 30-degree physical guard remain unchanged.
SET BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY TO 21.5.
SET BOOSTER_TERMINAL_REACTION_WHEEL_RESTORE_AUTHORITY TO 16.0.
SET BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_START_HEIGHT TO 4500.
SET BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_END_HEIGHT TO 3000.
// Run 104 proved that zero independent deployment still leaves 7--10 m/s^2 of
// upward ModuleAeroSurface/control force at 100% coefficient. Restore the
// historically measured post-entry zero-lift baseline; the bounded reaction
// wheels own attitude while Run 105 identifies the remaining body aerodynamics.
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
// Run 158 exposed a 0.265/s 5 km state that was already saturated. The 6 km
// family is much narrower at 0.169--0.191/s and 946--1093 m, so own it first.
SET FINAL_ALIGN_HEIGHT TO 6000.
SET FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT TO 65.
SET FINAL_ALIGN_RANGE TO 1200.
SET FINAL_ALIGN_HOLD_SECONDS TO 0.    // alignment continues during descent; do not spend the landing reserve hovering
SET FINAL_ALIGN_SPEED TO 200.0.
SET FINAL_ALIGN_POSITION_GAIN TO 0.18.
// Twice the 6 km median ratio supplies opposite, available corrections across
// the measured family. These values own entry shaping only until 5 km.
SET FINAL_ALIGN_ENTRY_COMPLEMENT_GAIN TO 0.36.
SET FINAL_ALIGN_MIN_POSITION_GAIN TO 0.16.
SET FINAL_ALIGN_MAX_POSITION_GAIN TO 0.20.
// Entry shaping owns 6--5 km only. Re-measure the transformed physical state
// at 5 km and select the terminal phase line from the Run-146/151 family.
SET FINAL_ALIGN_TERMINAL_PHASE_HEIGHT TO 5000.
// Run 172 brought physical error to 4.15 m at the formal plane, but the
// filtered long-stage response left 5.46 m/s of horizontal speed as the
// mathematical horizon collapsed. Project the same boundary state only 0.20 s
// forward; this preserves the endpoint and compensates measured response lag.
SET FINAL_ALIGN_TERMINAL_RESPONSE_LEAD_SECONDS TO 0.20.
// Run269 again reached 4.15 m position error from a 0.1801 entrance but
// retained 10.91 m/s as the finite-time 1/T tail rebuilt targetward demand.
// Run270 exposed that final align still owned a separate literal 4 while the
// similarly named main-guidance setting changed only the upstream entrance.
// Run271 verified the symbol but its high-energy branch added the historical
// +0.55 on top, producing 5.00 and stopping 26.21 m short. Treat the matched
// Run197 value as a minimum response floor: nominal/high/low family schedules
// that already exceed it retain their independently calibrated total.
SET FINAL_ALIGN_TERMINAL_MIN_VELOCITY_COEFFICIENT TO 4.45.
// Run 182 exercised a 0.248 s low-family lead but retained 8.02 m/s at the
// formal plane. Restore the common response horizon and use the remaining
// position margin only for late velocity damping. Fade the extra coefficient
// out before the repeatedly passing 0.177--0.178/s entrance family, and ramp
// it below 2.6 km so the earlier position travel and aero schedule do not move.
SET FINAL_ALIGN_TERMINAL_LOW_FAMILY_RATIO TO 0.170.
SET FINAL_ALIGN_TERMINAL_LOW_FAMILY_END_RATIO TO 0.172.
SET FINAL_ALIGN_TERMINAL_LOW_RATIO_EXTRA_VELOCITY_COEFFICIENT TO 0.50.
SET FINAL_ALIGN_TERMINAL_EXTRA_VELOCITY_START_HEIGHT TO 2600.
SET FINAL_ALIGN_TERMINAL_EXTRA_VELOCITY_FULL_HEIGHT TO 2000.
// Run 187's 0.1606/s entrance retained 13.46 m/s near the plane because the
// clamped cubic position term still saturated targetward with coefficient 4.5.
// Add a second, genuinely terminal stage only for that extreme-low family.
// Run 191 then exercised the partial blend at 0.1669/s: coefficient 5.326
// improved speed by 1.40 m/s while retaining 4.65 m position margin. Move the
// full endpoint upward to 0.165 so this boundary member receives about one more
// unit of terminal velocity weight; the zero endpoint and extreme maximum stay
// unchanged.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_RATIO TO 0.165.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_END_RATIO TO 0.168.
SET FINAL_ALIGN_TERMINAL_EXTREME_EXTRA_VELOCITY_COEFFICIENT TO 4.50.
// Runs 187/209 matched near 2.6 km. Raising the late coefficient from about
// 4.5 to 9 improved formal speed by 3.68 m/s, but the 2.5--2.0 km ramp was
// still too late to reach 5 m/s. The later 4--3 km experiment was first
// exercised after Run 216. Run 225 then closely matched Run 205: it spent
// 7.7 m more range and removed 3.7 m/s more speed by 3.5 km, yet regressed
// both formal gates. Restore the measured late ramp; terminal coefficient
// authority is unchanged, only its unsupported early timing is rejected.
SET FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_START_HEIGHT TO 3000.
SET FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_FULL_HEIGHT TO 2300.
// Runs 206/212 first identified 3 km physical range as the missing low-family
// phase coordinate. Runs 216/218/219 then showed that a large correction first
// applied there requests unrealizable acceleration inside the long-stage
// response tail. Move the same measurement one response window earlier.
// Near 3.5 km the representative physical ranges are 160 m (Run 206),
// 154 m (Run 212), 217 m (Run 216), 197 m (Run 218), and 148 m (Run 219).
// A 160 m reference and unit gain approximately preserve the required
// Run 216/218 negative corrections, protect Runs 206/212, and replace
// Run 219's late +50 m saturation with about +12 m of earlier shaping.
// Run 221 then sampled 3506 m just above an exact 3500 m comparison and did
// not latch until the next guidance update at 3342 m. Put the comparison at
// 3600 m so the first discrete eligible update lands near the calibrated
// 3.5 km state. Keep the separate high-start ownership and hard cap unchanged.
SET FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCH_HEIGHT TO 3600.
SET FINAL_ALIGN_TERMINAL_LOW_RANGE_REFERENCE TO 160.0.
SET FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS_GAIN TO 1.0.
SET FINAL_ALIGN_TERMINAL_LOW_RANGE_MAX_POSITION_BIAS TO 50.0.
// Runs 229/237/251/253 share the zero-authority 0.173--0.174 entrance
// family, yet span formal passes and opposite position misses. Their 3.6 km
// ranges all overlap at 145--150 m, so the earlier latch cannot separate
// phase. Projecting the first sub-3.2 km state to a common 3 km height does:
// passing Runs 214/229/237 measure 48.10/53.07/52.60 m, far-side Runs
// 230/251 measure 36.30/43.41 m, and near-side Run 253 measures 71.52 m.
// Apply one signed phase correction around the measured 52 m centre and fade
// it before the formal plane. Keep both the <=0.172 low family and >=0.175
// historical pass families mathematically untouched.
SET FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_START_RATIO TO 0.1720.
SET FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_FULL_START_RATIO TO 0.1730.
SET FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_FULL_END_RATIO TO 0.1745.
SET FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_END_RATIO TO 0.1750.
SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCH_HEIGHT TO 3200.
SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_PROJECTION_HEIGHT TO 3000.
SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_REFERENCE TO 52.0.
SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS_GAIN TO 1.5.
SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_MAX_POSITION_BIAS TO 20.0.
// Run 218 matched Run 205 near 3 km, but retaining its -33.8 m bias through
// the endpoint improved position by only 1.36 m while adding 3.92 m/s speed.
// Run 219 then saturated the opposite sign and requested -16.7 to -45.8 m/s2
// only after 2.9 km, while measured deceleration weakened. Treat both signs as
// early phase shaping rather than a permanent endpoint: full at the new latch
// and zero at the formal plane.
SET FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_START_HEIGHT TO 3500.
SET FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_END_HEIGHT TO 2000.
// Run 193's normal-high entrance selected coefficient 4.0, then crossed the
// centre at 2.82 km with 22.6 m/s and stopped 35.84 m on the far side. Gate a
// small earlier velocity term on the independently measured 5 km terminal
// ratio. Run 211 then reached 0.2229/s and missed the speed gate by only
// 0.35 m/s while retaining 3.96 m physical error. Move the zero point down to
// the measured Run 178 passing boundary; Runs 177/178/189 at or below 0.2206
// remain untouched, while Run 211-like states receive only a small partial
// coefficient. This avoids reopening the rejected broad entrance-ratio
// phase-start interpolation.
SET FINAL_ALIGN_TERMINAL_HIGH_ENERGY_RATIO TO 0.2206.
SET FINAL_ALIGN_TERMINAL_HIGH_ENERGY_FULL_RATIO TO 0.230.
// Run 243 sampled the same isolated normal-high branch at 0.2262 and reached
// 5.85 m formal position, but speed missed by only 0.18 m/s. Its physical
// axis remained above the protected settle tilt gate during the centre
// crossing, so do not weaken that gate. Raise only this bounded ceiling by
// 10%; the Run 243 blend receives about +0.030 coefficient, while every
// validated <=0.2206 family remains mathematically unchanged.
SET FINAL_ALIGN_TERMINAL_HIGH_ENERGY_EXTRA_VELOCITY_COEFFICIENT TO 0.55.
SET FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_START_HEIGHT TO 5000.
SET FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_FULL_HEIGHT TO 3000.
// Runs 190 and 200 isolated the extreme-high terminal family at ratios
// 0.2596/0.2577 and stopped 58.89/57.85 m on the same far side. Run 204 then
// used the same 6 km ownership and coefficient 4.5 at ratio about 0.245, yet
// the old zero endpoint left 49.88 m. Treat that continuous residual as an
// endpoint phase bias rather than earlier/stronger braking. Run 207 then
// entered the full 6 km ownership branch at only 0.2373 terminal ratio: an
// 11.45 m bias reduced the inferred no-bias endpoint from about 47.3 m to
// 35.9 m. Run 210 supplied the resulting 47 m full-start baseline. Against
// matched zero-bias Run 204, physical error fell from 49.88 to 24.74 m while
// both speeds remained near zero: only 0.535 m of physical endpoint moved per
// commanded metre. Run 244 later used the 90 m baseline but still reached
// -39.65 m / -1.10 m/s near the formal plane. Applying that measured response,
// another 60 m predicts about 32 m of target-side endpoint motion. Keep this
// correction inside the already isolated full high-start family. Run 234
// exposed the unowned
// boundary immediately above the protected Run 197 near-pass: terminal ratio
// 0.2313 received full velocity damping and stopped at -30.1 m / 0.21 m/s,
// because both range-family weights and the old 0.235 endpoint ramp were zero.
// Start at Run 197's 0.2295 zero-authority boundary and reach the already
// calibrated 55 m endpoint at Run 195's measured 0.2301 member.
SET FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_RATIO TO 0.2295.
SET FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_FULL_RATIO TO 0.2301.
SET FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS TO 55.0.
// Run264 added 400 m only after the 3.85 km latch, but the saturated actuator
// did not respond. Run265 moved a 50 m A/B here: its latch improved by 17 m,
// yet the formal residual worsened about 25 m and reversed velocity phase.
// Restore the protected baseline; stopping authority, not position error, is
// the active mounted-fin boundary.
SET FINAL_ALIGN_TERMINAL_EXTREME_HIGH_START_POSITION_BIAS TO 150.0.
// Runs 244/254/258 all stop at nearly zero speed on the far side, but their
// signed residual is ordered by the independently latched 4 km physical range.
// Normalize Run 244's old 90 m baseline to the current 150 m baseline with the
// measured 0.535 physical/commanded response, then fit all three samples:
// ranges 214.55/163.72/183.72 m require about 14/154/84 commanded metres.
// The existing 205 m reference with gain 4 gives 0/165/85 m after clipping.
// Keep this persistent term inside the full high-start isolation; the separate
// 3.6 km term remains transient phase shaping and still fades before 2 km.
SET FINAL_ALIGN_TERMINAL_HIGH_START_4KM_RANGE_REFERENCE TO 205.0.
SET FINAL_ALIGN_TERMINAL_HIGH_START_4KM_RANGE_BIAS_GAIN TO 4.0.
// Physically mounting the grid fins changed Runs261--263 into a new negative
// 3.85 km range family. Run264 raised this cap by 400 m, but horizontal
// acceleration and cone allocation were already saturated and the physical
// endpoint changed by less than ordinary run dispersion. Restore the isolated
// historical cap; further correction must act before the target crossing.
SET FINAL_ALIGN_TERMINAL_HIGH_START_4KM_MAX_POSITION_BIAS TO 165.0.
// Runs 173--176 bracketed the ownership lever. A 0.1702/s entrance starting
// at 5.36 km still reached the centre about 700 m below the formal plane,
// whereas Run 172's 0.1703/s entrance nearly passed from a 5 km start. Keep
// higher ratios interpolated toward 6 km, but return this measured low member
// to the terminal phase height. The endpoint law itself remains identical.
SET FINAL_ALIGN_FINITE_TIME_START_LOW_RATIO TO 0.170.
SET FINAL_ALIGN_FINITE_TIME_START_FULL_RATIO TO 0.188.
// Run 195 sampled almost the same terminal ratio as Run 192 (0.2301/0.2303).
// Its 5.54 km ownership plus coefficient 4.5 produced 30.25 m / 0.91 m/s,
// while Run 192's 5.81 km ownership plus coefficient 4.0 produced
// 8.00 m / 6.48 m/s. Run 197 then combined a 5.70 km start and coefficient
// 4.454 to reach 7.83 m / 5.02 m/s. Run 202 closely matched its 5 km state,
// but a 5.559 km start spent 23 m more range by 3 km and stopped 40.89 m far
// side. Reproduce the measured 5.70 km plateau by 0.1788 while preserving
// the passing <=0.1780 family and the separate extreme-high 6 km stage.
SET FINAL_ALIGN_FINITE_TIME_START_MEDIUM_RATIO TO 0.1785.
SET FINAL_ALIGN_FINITE_TIME_START_MEDIUM_FULL_RATIO TO 0.1788.
SET FINAL_ALIGN_FINITE_TIME_START_MEDIUM_TARGET_HEIGHT TO 5700.
// Run 190's 0.1831/s entrance did not acquire finite-time ownership until
// 5.73 km and crossed the centre at 3.2 km with 32 m/s. Run 192 then showed
// that extending the correction down to a normal-high 0.1814/s entrance
// regressed both speed and position when used without the new terminal-energy
// damping. Keep its independent final 5.8--6 km interpolation restricted to
// the extreme-high edge.
SET FINAL_ALIGN_FINITE_TIME_START_HIGH_RATIO TO 0.1828.
SET FINAL_ALIGN_FINITE_TIME_START_HIGH_FULL_RATIO TO 0.1831.
// The high-drag edge consumes acceleration magnitude, so a large signed phase
// correction increases braking in either direction. Run 161 then showed that
// increasing this gain with ratio amplifies the high-ratio crossing while the
// actuator is already at full drag. Keep the physical phase target constant;
// the independent hold-scale schedule below owns entrance dispersion.
SET FINAL_ALIGN_TERMINAL_MIN_POSITION_GAIN TO 0.20.
SET FINAL_ALIGN_TERMINAL_MAX_POSITION_GAIN TO 0.20.
// Early preload shifted the useful 5 km family downward. Run 169's 0.2067
// state still stopped 30 m short when the former 0.20--0.22 window assigned
// 0.668 hold. Keep it on the low endpoint and reserve full amplitude for 0.23;
// the separately latched duration schedule below still starts at 0.22.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_LOW_RATIO TO 0.21.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_FULL_RATIO TO 0.23.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_HOLD_SCALE TO 0.50.
// Run299 moved the frozen 6 km entrance to 0.1990, essentially matching
// Run272's 0.1978 joint-pass family. Its later terminal ratio rose to 0.2437,
// however, selecting scale 1.0 and stopping at +15 m about 500 m above the
// formal plane before reversing. Run272 used 0.8933. Cap only the already
// frozen <=0.200 entrance family near the Run272 actuator value; genuinely
// higher entrances retain the existing terminal-ratio schedule and full
// endpoint. Run303 was the first effective high-terminal observation of the
// 0.90 cap: it passed speed at 2.25 m/s but stopped 19.75 m short. Run272
// passed all formal gates at about 0.8933, so interpolate the cap to 0.89.
SET FINAL_ALIGN_TERMINAL_LOW_ENTRY_HOLD_CAP_RATIO TO 0.200.
SET FINAL_ALIGN_TERMINAL_LOW_ENTRY_HOLD_CAP TO 0.89.
// A fixed phase hold cannot normalize the full extreme-low family. Run304's
// 0.1727/0.1824 sample used 0.35 and stopped about 18 m short; Run305's
// 0.1825/0.2109 sample used 0.25, crossed near 2.55 km and was 37.61 m beyond
// centre formally. Freeze a monotone terminal-ratio schedule at the existing
// 5 km phase latch. Low-ratio states preserve travel with about 0.32, while
// high-ratio states spend more upper-band braking and avoid crossing before
// the positive-range gate can retain the late 1.00 endpoint. Run306 proved
// the monotone shape but selected 0.3468 and remained 22.37 m short. Shift
// both endpoints down by 0.03 to preserve about 12 m more upper-band travel.
// Run307 then sampled terminal ratio 0.1776, hit the clamped 0.29 endpoint
// and remained 19.26 m short. Extend the measured line to 0.25 at 0.175.
// Run309 sampled terminal ratio 0.2010 with active hold 0.3367; the new full
// late floor passed speed at 1.44 m/s but stopped 18.29 m short. The local
// Run307--308 slope calls for active hold about 0.309. Lower only the high
// endpoint to 0.332, which selects that value at 0.201 while moving the
// already-calibrated 0.179 family by less than 0.005.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_HOLD_SCALE TO 0.25.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_HIGH_HOLD_SCALE TO 0.332.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_HOLD_LOW_RATIO TO 0.175.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_HOLD_HIGH_RATIO TO 0.211.
// Run 165's rebuilt near-1.00 tail stopped 53 m short; Run 166's strict 0.50
// tail crossed the 2 km plane at 10.69 m/s. Retain the low upper-band release,
// then use the measured midpoint as the denser below-end-height ceiling.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_TAIL_CEILING TO 0.75.
// Run 167 proved that applying 0.75 immediately at 4 km spends the right
// braking too early. Expose that ceiling continuously only by 2.5 km.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING_FULL_HEIGHT TO 2500.
// Run304 validates a stronger time shift only in the frozen extreme-low
// family. Reach the unchanged 1.00 tail by 2.7 km: at about 3.5 km this still
// commands slightly less drag than Run304, while from about 3.0 km downward
// it commands more. Other ratio families retain the proven 2.5 km surface.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_TAIL_CEILING_FULL_HEIGHT TO 2700.
// Run306 retained the correct positive position sign, but realised only
// 0.64/0.81 drag blend near 2.95/2.77 km and missed formal speed by
// 0.79 m/s. Add a continuous lower ownership surface only after 3 km:
// begin at the measured 0.65 state and reach the unchanged physical maximum
// at 2.7 km. The positive physical range/speed gates remain mandatory.
// Run307's 0.65/3.0 km candidate remained below the naturally realised
// surface and had no authority while formal speed missed by only 0.20 m/s.
// Run308's 0.85/3.1 km floor restored position margin (6.94 m) but still
// crossed 1.42 m/s fast.  Raise only this already-gated late floor to full
// ownership; its endpoint, cone limits, and positive range/speed gates remain
// unchanged, so this isolates braking timing from the calibrated hold scale.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_LATE_FLOOR_START_HEIGHT TO 3100.
SET FINAL_ALIGN_TERMINAL_EXTREME_LOW_LATE_FLOOR_START_BLEND TO 1.00.
// Runs 162--163 reached the same 4 km speed but crossed at 3.08--2.83 km
// because the fixed release ramp ignored their remaining-range difference.
// High ratio means less range for the same speed, so retain the latched endpoint
// progressively lower. The positive-speed release gate remains authoritative.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_END_LOW_RATIO TO 0.22.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_END_FULL_RATIO TO 0.24.
SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_END_HEIGHT TO 2500.
// Run 164's unreachable 5 km entrance was already the high member of the
// physical closing-ratio family at 14 km. Preload only the existing 14--12 km
// high-drag ownership ramp from that one-time state; this does not expose a new
// attitude endpoint or modify the upstream checkpoint/main sequence.
SET TERMINAL_EARLY_AERO_BRAKE_LATCH_HEIGHT TO 14000.
SET TERMINAL_EARLY_AERO_BRAKE_HOLD_END_HEIGHT TO 12000.
SET TERMINAL_EARLY_AERO_BRAKE_LOW_RATIO TO 0.0714.
SET TERMINAL_EARLY_AERO_BRAKE_FULL_RATIO TO 0.0721.
SET TERMINAL_EARLY_AERO_BRAKE_MIN_FLOOR TO 0.12.
// Run 150 used 1.00 only to identify acquisition bandwidth. Restore the
// response that produced the Run-146 formal pass while moving the phase line.
SET FINAL_ALIGN_VELOCITY_GAIN TO 0.50.
SET FINAL_ALIGN_READY_ERROR TO 8.0.  // centroid plus 1.65 m hook radius remains inside the 10 m half-width
// Run 147's three narrow gates never overlapped, but its 2183 m state was
// 6.86 m / 3.81 m/s / 14.60 deg: already inside all physical capture limits
// with more than 2 km left for pure damping. Include that observed opportunity
// while retaining explicit margin inside 4 m/s and 15 degrees.
SET FINAL_ALIGN_READY_SPEED TO 3.9.
SET FINAL_ALIGN_READY_TILT TO 14.7.
// Run 189 passed the physical-hook waypoint but never found a simultaneous
// strict commitment window. At 2.47 km it was already inside a recoverable
// 15.78 m / 8.67 m/s / 13.99 deg state; retaining the collapsing cubic horizon
// then commanded about 20 degrees and rebuilt error. Run 222 repeated the
// boundary at 2.164 km with 20.30 m / 8.75 m/s / 7.98 deg, but the 16 m gate
// delayed bounded velocity settle until only 43 m above the formal plane.
// Run 226 restored the late extreme ramp and reached 14.62 m / 11.66 m/s /
// 10.38 deg at 2.133 km, followed by a fully acceptable 2.26 m position but
// 8.60 m/s at the formal plane. The collapsing law requested +31.68 m/s2
// targetward there, while the old 9 m/s gate never entered. Admit this single
// measured state by changing only the speed gate; the 21/24 m hysteresis,
// physical-axis gate, 1 m/s2 cap and strict legal commitment stay unchanged.
SET FINAL_ALIGN_PRECOMMIT_SETTLE_RANGE TO 21.0.
SET FINAL_ALIGN_PRECOMMIT_SETTLE_SPEED TO 12.0.
// Run 230 entered the weak 1 m/s2 settle law at only +1.17 m remaining range
// while still carrying 11.49 m/s. Run 232 then proved that an unsigned 8 m
// minimum admits an already-crossed -18.90 m state and releases only after
// error exceeds 24 m. Measure the footprint on the frozen signed approach
// axis instead. Runs 245/246 then repeated the discrete boundary: their first
// safe sub-12 m/s samples were +2.658 m and +4.412 m, just after the prior
// guidance sample missed the speed gate. A 2.5 m minimum admits both measured
// safe states while still rejecting Run 230's +1.17 m late transition and
// every already-crossed negative state.
SET FINAL_ALIGN_PRECOMMIT_SETTLE_MIN_RANGE TO 2.5.
// Run 196's new damping created an 11--14 m / 3.16 m/s settle window, but
// handing off while the physical thrust axis still lagged at 17.07 degrees
// rebuilt 11.79 m/s in reverse. Run 192 did the same from 18.36 degrees,
// whereas Run 191's useful handoff was 8.97 degrees and Run 194's near-pass
// edge was about 14.97 degrees. Require physical-axis convergence first.
SET FINAL_ALIGN_PRECOMMIT_SETTLE_TILT TO 15.5.
SET FINAL_ALIGN_PRECOMMIT_REACQUIRE_RANGE TO 24.0.
SET FINAL_ALIGN_PRECOMMIT_MAX_SPEED TO 1.0.
// Run 236 validated the signed entry at +8.90 m / 9.57 m/s / 8.56 deg and
// retained only 1.16 m physical error near the formal plane, but the shared
// 1 m/s2 cap left 6.02 m/s. The separate 2 m/s2 cap later enabled legal
// commits, yet Run 248 entered only 199 m above the plane at
// +17.24 m / 10.93 m/s / 8.76 deg and retained 7.63 m/s formally while
// position passed at 3.49 m. Add the measured 2.5 m/s2 one-second deficit
// only to this reversible precommit interval; post-commit recovery keeps its
// independently safer 1 m/s2 cap below.
SET FINAL_ALIGN_PRECOMMIT_MAX_ACCEL TO 4.5.
// Runs273/275/276 crossed the target with 12--20 m/s remaining because the
// rectangular 21 m / 12 m/s precommit gate was reached only after the
// actuator-limited stopping opportunity had passed. For the isolated mounted
// low-entry family, admit the existing bounded settle controller on a
// stopping-distance surface: x - v^2/(2*a_id) - margin <= 0. Run277 measured
// only 2.42--2.88 m/s2 realised deceleration while the command cap remained
// 4.5, so use a separate conservative plant identification in the surface.
// Run280 then showed that 90 m / 30 m/s withheld the bounded branch until
// 2.84 km; at 3.38 km it was already at +140.10 m / 31.58 m/s with admissible
// attitude. Admit that measured state while still excluding the energetic
// outer trajectory.
// Run272's 0.1978 joint-pass family remains outside this branch.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_MAX_ENTRY_RATIO TO 0.195.
// Runs293--295 independently exposed the same negative-surface state near
// 3.6 km at 190--218 m / 38--39 m/s. Their ordinary 150/35 latches remained
// 400--500 m late and left 8.8--10.9 m/s formally. Extend the existing
// <=0.195 family to the already bounded Run292 envelope. The Run272 joint
// pass at ratio 0.1978 remains outside this branch.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_MAX_RANGE TO 320.0.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_MAX_SPEED TO 42.0.
// Run292 sampled the distinct extreme-low family at entry ratio 0.1540.
// Its last negative stopping surface was +302.80 m / 39.73 m/s at 3694 m;
// by the time the generic 150/35 window admitted range, the surface was
// already 53 m positive. Runs301--302 then sampled stable retained-brake
// topologies at 0.1655 and 0.1814 without entering the former narrow family,
// so the intended 0.35/1.00 phase shaper had zero authority. Run302 already
// passed position at 7.10 m but retained 10.63 m/s: exactly the state that
// needs less upper hold and more late drag. Cover that measured topology
// through 0.185. The shared negative stopping surface remains mandatory,
// while Run272's protected joint-pass family at 0.1978 stays excluded.
SET FINAL_ALIGN_PRECOMMIT_EXTREME_LOW_MAX_ENTRY_RATIO TO 0.185.
SET FINAL_ALIGN_PRECOMMIT_EXTREME_LOW_MAX_RANGE TO 320.0.
SET FINAL_ALIGN_PRECOMMIT_EXTREME_LOW_MAX_SPEED TO 42.0.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_DECEL TO 2.5.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_MARGIN TO 10.0.
// Run296 proved that the 320 m entry window and the inherited 160 m release
// edge form an empty invariant set: reachability latched at 271.77 m and
// 243.61 m, then the following release test cleared it in the same update.
// Put the release surface outside the entry surface with 40 m of hysteresis.
// The controller can now retain ownership while braking, but still returns to
// final-align guidance if a disturbance carries the stage beyond 360 m.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_REACQUIRE_RANGE TO 360.0.
// Run278 latched at +77.59 m / 27.30 m/s. The ordinary 4.5 m/s2 cap then
// realised only about 3.0 m/s2 and crossed 33.74 m beyond the target. That
// state requires about 5.13 m/s2 to stop within 5 m, while the immediately
// preceding outer command had already demonstrated 5.75 m/s2. Expose more
// command only inside this isolated reachability latch; direction remains
// bounded by the unchanged software and physical cones.
// Run282's earlier 150 m / 35 m/s latch repaired position to 5.53 m, but its
// 3.93 m/s2 realised spatial deceleration left 9.37 m/s horizontally at the
// formal plane. About 4.07 m/s2 is sufficient over the same measured path.
// Add only 6.25% command authority for this latched family; every switching
// surface and cone barrier remains unchanged.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_MAX_ACCEL TO 8.5.
// Run279 reached -1.68 m / 6.68 m/s near the formal plane while the bounded
// controller still requested its ordinary 1 m/s targetward crawl. A
// reachability latch already owns a stopping branch, so remove only that
// residual reference; rectangular precommit retains the proven 1 m/s target.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_TARGET_SPEED TO 0.0.
// Run283 proved that the 8.5 m/s2 cap increases realised spatial deceleration,
// but the shared 0.60 velocity gain leaves saturation at 14.17 m/s. The last
// two pre-formal samples therefore requested only 7.30 and 5.71 m/s2 while
// attitude/aero lag retained 8.78 m/s formally. Hold the same tested maximum
// to 9.44 m/s only for the isolated reachability latch. Ordinary precommit and
// the safety-critical post-commit controller retain the shared 0.60 gain.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_VELOCITY_GAIN TO 0.90.
// Run284 showed that this upstream request is subsequently owned by the
// measured high-drag actuator: its tail ceiling reached exactly 0.75, the
// engine still pointed targetward, and body drag supplied the net braking.
// Raise only the isolated reachability tail through the existing 4.0--2.5 km
// smooth blend. A 0.85 endpoint corresponds to about a 17-degree command
// inside the unchanged 20-degree active and 30-degree physical limits, and
// the entire allocator is disabled at/below the formal waypoint.
SET FINAL_ALIGN_PRECOMMIT_REACHABILITY_AERO_BRAKE_TAIL_CEILING TO 0.85.
SET FINAL_ALIGN_PRECOMMIT_EXTREME_LOW_AERO_BRAKE_TAIL_CEILING TO 1.00.
// Run 177 committed cleanly above 2 km and held sub-metre error through 1 km,
// but residual speed then grew to about 2.8 m/s and carried the hook 19 m out
// at the net plane. Strengthen only pure velocity damping; the position target
// remains disabled and the 1 m/s2 cap still bounds attitude demand.
SET FINAL_CAPTURE_VELOCITY_GAIN TO 0.60.
SET FINAL_CAPTURE_MAX_ACCEL TO 1.0.
// Run 152 showed that immediate full-bandwidth drag cancellation above 2 km
// excites the long-body aerodynamic branch. Keep that pre-formal regime
// protected. Run 214, however, made a legal commitment and reached
// 6.10 m / 0.65 m/s at 1.50 km; the slow command-memory filter then rebuilt
// drift to 37.83 m / 7.20 m/s before the old 500 m handoff. Remove the second
// pole from that last safe committed state. FINAL_DESCENT_ARMED keeps this
// change strictly post-commit and the height remains below the formal plane.
SET FINAL_CAPTURE_DIRECT_CONTROL_HEIGHT TO 1500.
// Run 141's 0.75 m/s pursuit acted through the old slow command-memory filter
// and produced a delayed 50 m pass, so the speed cap was set to zero. Run 229
// is the first formal pass and strict commitment after moving direct control
// up to 1.5 km. Pure velocity damping then stopped the lateral speed but
// allowed position to drift from 3--5 m to about 60 m before the net. Restore
// the same bounded pursuit only in the already-committed mode: the new 1.5 km
// direct handoff removes the old second pole, while the 1 m/s2 acceleration
// cap and 2 m deadband remain unchanged.
SET FINAL_CAPTURE_POSITION_DEADBAND TO 2.0.
SET FINAL_CAPTURE_POSITION_GAIN TO 0.10.
SET FINAL_CAPTURE_MAX_SPEED TO 0.75.
// Run304 entered a useful local capture set near 309 m at about 5.15 m error
// and almost zero horizontal speed. Continuing position pursuit regenerated
// velocity after centre crossings and rotated the low-speed retrograde axis.
// Latch that controlled invariant set once: desired horizontal velocity then
// remains zero, while bounded velocity feedback and measured-aero cancellation
// stay active. Run308 showed that retaining -0.60*v after the latch alternated
// the translation state through the soft attitude plant until the valid
// 20-degree cone barrier fired.  Use a separate slow pole inside this local
// invariant set; pre-latch pursuit remains at the proven 0.60 gain.
SET FINAL_CAPTURE_NEAR_NET_DAMPING_HEIGHT TO 350.
SET FINAL_CAPTURE_NEAR_NET_DAMPING_RANGE TO 8.0.
SET FINAL_CAPTURE_NEAR_NET_DAMPING_SPEED TO 2.5.
SET FINAL_CAPTURE_NEAR_NET_DAMPING_VELOCITY_GAIN TO 0.25.
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
// Run 252 stayed controlled through roughly 300 m, then its transverse
// attitude rate grew through 25--54 deg/s below the net. Run 254's axis
// diagnostics independently measured 16.98 deg/s transverse rate against
// only 0.59 deg/s axial roll, with a well-conditioned roll reference. The
// visually observed Run 260 loss repeated below 300 m even though that flight
// had not passed the formal gate. This is a physical stability guard, not a
// reward for legal commitment: arm it for every descent at 500 m, before the
// measured growth begins. kOS defines MAXSTOPPINGTIME as the maximum target
// rate proportional to available torque/inertia; keep the already isolated
// 0.5 s value and the 8/5 s inner torque-loop settling times for the first
// authoritative active sample.
SET FINAL_CAPTURE_NEAR_NET_STEERING_HEIGHT TO 500.
SET FINAL_CAPTURE_NEAR_NET_MAX_STOPPING_TIME TO 0.5.
// A rate cap alone cannot distinguish command-following from a growing limit
// cycle. Below 600 m, pass the requested thrust direction through an exact
// angular reference governor and monitor a dimensionless attitude energy made
// from pointing error and the filtered finite-difference rate of the
// longitudinal body axis. That rate is invariant to roll. Physical elapsed
// time, rather than the controller's bounded tuning step, drives slew,
// filtering and dwell; energy growth must qualify on two consecutive samples.
// The hard body-axis-rate path remains immediate. Either path latches a
// fail-safe that removes horizontal pursuit and slews toward local up. The
// current safety cone is always projected last. These thresholds sit inside
// the independent 10 deg/s per-physics-frame T-02 audit.
SET FINAL_CAPTURE_ATTITUDE_MONITOR_HEIGHT TO 600.
SET FINAL_CAPTURE_ATTITUDE_TARGET_RATE_LIMIT TO 3.0.
SET FINAL_CAPTURE_ATTITUDE_RATE_FILTER_SECONDS TO 0.20.
SET FINAL_CAPTURE_ATTITUDE_WARNING_ERROR TO 3.0.
SET FINAL_CAPTURE_ATTITUDE_WARNING_RATE TO 5.0.
// Run303's one-way hard-rate latch fired at 8.027 deg/s near 209 m even
// though the independent Unity-frame transverse observer remained below
// 4.033 deg/s. The resulting permanent loss of horizontal pursuit rebuilt
// error after a 0.041 m centre crossing. Reject that aliased single-sensor
// edge at 9.5 deg/s while retaining margin inside the independent 10 deg/s
// T-02 boundary. Run307 still produced a false 9.933 deg/s kOS sample while
// the independent observer measured at most 4.457 deg/s and actual cone was
// only 1.297 degrees. Cross-qualify the computed hard rate with five degrees
// of measured cone; independent T-02 remains the immediate physical-rate stop.
SET FINAL_CAPTURE_ATTITUDE_HARD_RATE TO 9.5.
SET FINAL_CAPTURE_ATTITUDE_HARD_RATE_MIN_ACTUAL_CONE TO 5.0.
SET FINAL_CAPTURE_ATTITUDE_ENERGY_ERROR_SCALE TO 4.0.
SET FINAL_CAPTURE_ATTITUDE_ENERGY_RATE_SCALE TO 5.0.
SET FINAL_CAPTURE_ATTITUDE_ENERGY_GROWTH_PER_SECOND TO 0.5.
SET FINAL_CAPTURE_ATTITUDE_GROWTH_DWELL_SECONDS TO 0.20.
// Run306's computed energy path latched at 7.931 deg/s while the independent
// transverse observer measured at most 3.628 deg/s and actual cone was only
// 6.39 degrees. Require a second physical indicator before noisy energy alone
// can disable the verified near-net damper. Ten degrees leaves another ten
// degrees before the immediate cone barrier and twenty before T-01.
SET FINAL_CAPTURE_ATTITUDE_ENERGY_MIN_ACTUAL_CONE TO 10.0.
SET FINAL_CAPTURE_ATTITUDE_SAFE_MAX_STOPPING_TIME TO 0.20.
// Rate/energy detects a dynamic instability, but Run274 crossed the physical
// cone with only 1.82 deg/s filtered transverse rate because the low-speed
// surface-retrograde axis itself rotated. Treat 20 degrees as a control
// barrier: it leaves 10 degrees of plant/cadence margin and matches the
// existing actual-cone command guard's first intervention surface.
SET FINAL_CAPTURE_ATTITUDE_SAFE_CONE_BARRIER_DEGREES TO 20.0.
// Once the one-way safe latch has removed horizontal pursuit, keep local up
// inside a controlled invariant recovery set instead of projecting it back
// into the normal 8-degree cone around a rapidly rotating surface-retrograde
// axis. 18 degrees still leaves 12 degrees inside the independent physical
// 30-degree powered-flight boundary.
// Run294 proved that widening the normal 8-degree set to 18 degrees after a
// barrier crossing drives the moving low-speed cone in the wrong direction:
// command rose to 18 while actual cone rose from 20.78 to 39.37 degrees.
// Safe mode must be a subset of normal mode, leaving dynamic tracking margin
// inside the immutable 30-degree physical boundary.
SET FINAL_CAPTURE_ATTITUDE_SAFE_COMMAND_CONE_DEGREES TO 8.0.
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
// Aim near the centre of the mandatory 150--200 m/s band. Runs 65--66 used a
// deliberately fast 225 m/s planning endpoint to compensate for excessive
// thrust; that compensation is invalid after correcting the actuator scale.
// Run 217 put every horizontal gate inside but reached 200.33 m/s descent.
// Lowering this endpoint from 175 to 174 m/s retained broad lower-gate margin.
// Run 228 then genuinely exercised the 12 m/s reversible settle, passed both
// horizontal gates by wide margins, but reached 200.47 m/s descent. Moving the
// endpoint from 174 to 173 produced Run 229's 199.74 m/s pass. Run 237's newly
// validated 2 m/s2 horizontal settle again passed every horizontal gate but
// reached 200.72 m/s. Moving the endpoint to 172 produced broad improvements,
// but Run309 landed exactly on the upper edge at 200.01 m/s. Remove one more
// metre per second of independent vertical reference; about 21 m/s nominal
// margin remains above the lower gate.
SET TERMINAL_WAYPOINT_VERTICAL_SPEED TO 171.
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
// infeasible.  Preserve the exact upstream finite-time cubic coefficients.
// Final align owns its separately identified response coefficient above.
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
// Run 106's disabled branch raised measured upward body aerodynamics from 3.84
// to 14.07 m/s^2 near 6 km and lost 134 m/s of descent for only 19 m/s extra
// horizontal braking. Restore the identified forward/upright low-lift branch
// and let its continuous scalar reach the physical endpoint sooner.
SET TERMINAL_ALONG_AERO_BRAKE_ENABLED TO TRUE.
SET TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT TO 14500.
SET TERMINAL_ALONG_AERO_BRAKE_FULL_HEIGHT TO 13500.
SET TERMINAL_ALONG_AERO_BRAKE_ACCEL_GAIN TO 4.
SET TERMINAL_ALONG_AERO_BRAKE_MARGIN TO 1.03.
SET TERMINAL_ALONG_AERO_BRAKE_ERROR_DEADBAND TO 0.5.
SET TERMINAL_ALONG_AERO_BRAKE_BUILD_RATE TO 0.35.
SET TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE TO 0.12.
SET TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND TO 1.0.
// Run 118 proved that the surface-retrograde centre's reduced body drag costs
// more braking than its opposite engine component supplies.  Run 117 already
// reached the verified high-drag endpoint at 8 km, then released it and rebuilt
// too late.  Hold that physical endpoint below this boundary until the
// separately margined speed gate transfers ownership.
SET TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_HEIGHT TO 8000.
// Once the now-calibrated platform is close, Run 124 shows that an
// unconditional hold below 4 km integrates small 6 km speed dispersion into
// 60+ m of footprint dispersion.  Enter this boundary at full state, then let
// the measured range/time error release it continuously in the terminal tail.
SET TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_END_HEIGHT TO 4000.
// Runs 119--120 show the long-stage attitude/force tail carries signed speed
// through zero after a 4--5 m/s release.  Start unwinding at a positive 7 m/s
// so that physical lag, rather than a late reversal, supplies the final margin.
// The formal finite-time target remains 5 m/s and no reverse pursuit is added.
SET TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED TO 7.
// Run 112 reaches 6 km at 211/328 m/s horizontal/descent, then loses final
// net braking from about 11.6 to 6.0 m/s2 as dynamic pressure falls.  Build a
// backward-reachable staging boundary above that loss of authority instead of
// demanding an impossible correction below 3 km.
SET TERMINAL_ALONG_AERO_BRAKE_STAGE_HEIGHT TO 6000.
SET TERMINAL_ALONG_AERO_BRAKE_STAGE_HORIZONTAL_SPEED TO 160.
SET TERMINAL_ALONG_AERO_BRAKE_STAGE_DESCENT_SPEED TO 330.
SET TERMINAL_ALONG_AERO_BRAKE_STAGE_RESPONSE_SECONDS TO 2.
// Run 48 measured 16--63 m/s of along-track speed excess from 20 to 10 km,
// while the commanded velocity-cone angle remained only 2.8--15.5 degrees and
// the 25% correction reserve was not fully used.  Symmetrically allocate that
// reserve toward the maximum-braking cone edge only while the stage is faster
// than its finite-time reference.  The blend returns to the nominal trajectory
// before any forbidden forward/reversal request can appear.
// Run 95 leaves too much horizontal energy while becoming vertically slow.
// Re-enable only the legal cone-edge actuator; main.ks now drives it from a
// physical stopping-demand/authority ratio, not the rejected 2*x/T reference.
// Run 137 finally applied a real 0.35 opposite-edge share after all other
// arbitration. Near 4 km it reduced measured net braking from about 13.79 to
// 11.83 m/s2: the engine gain was smaller than the lost long-body drag. Keep
// this rejected Run-115 mechanism disabled; high drag owns the late endpoint.
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
// Retained only for diagnostic replays while the branch is disabled.
SET TERMINAL_ALONG_BRAKE_MAX_BLEND TO 0.35.
// Reachability pressure is raw v^2/(2r) divided by the live full-throttle
// engine projection plus measured aerodynamic braking.  Blend continuously
// before the boundary becomes infeasible; height only arms the allocator.
SET TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT TO 6000.
SET TERMINAL_ALONG_BRAKE_REACHABILITY_FULL_HEIGHT TO 5500.
SET TERMINAL_ALONG_BRAKE_PRESSURE_ARM TO 0.75.
SET TERMINAL_ALONG_BRAKE_PRESSURE_FULL TO 1.25.
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
// Retain the upstream Hermite/ignition allowance. Run 128's full fade to the
// physical centre repeated an approximately 31 m signed overshoot. Runs
// 129--130 then proved that moving the physical ship rewrites the whole
// guidance history. Keep the entity and the verified upstream 75 m plan fixed.
// Run 134 proved that enlarging the plan to 122 m also moved mainly the ruler,
// not the saturated physical trajectory. Retain the 31.5 / 75 = 0.42 late
// residue and let the bounded physical allocator above reject short entrances.
// Fade the residue below the plane so capture targets the physical hook.
SET TERMINAL_LIVE_APPROACH_OFFSET_FADE_START_HEIGHT TO 4000.
SET TERMINAL_LIVE_APPROACH_OFFSET_FADE_END_HEIGHT TO 3000.
// Runs 138--139 passed from 202--208 m latches with final blends near 0.8.
// Runs 136/140 then bracketed signed failure: 175 m / blend 1 moved negative,
// while 226 m / blend 0.56 stayed positive. The former positive range gain
// reinforced both disturbances. Centre the two passes and invert only this
// late scalar slope; the physical platform and upstream plant remain fixed.
SET TERMINAL_LIVE_APPROACH_OFFSET_FINAL_BLEND TO 0.81.
SET TERMINAL_LIVE_APPROACH_OFFSET_FADE_REFERENCE_RANGE TO 205.
SET TERMINAL_LIVE_APPROACH_OFFSET_FADE_RANGE_GAIN TO -0.50.
SET TERMINAL_LIVE_APPROACH_OFFSET_FINAL_MIN_BLEND TO 0.40.
SET TERMINAL_LIVE_APPROACH_OFFSET_FINAL_MAX_BLEND TO 1.0.
SET TERMINAL_LIVE_APPROACH_OFFSET_POST_FADE_END_HEIGHT TO 1500.
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
// Runs261--265 with physically mounted fins retain 16--18 m/s formal descent
// margin but cross the target before the late position controller can act.
// Run266 raised this 8--4 km floor to 12 degrees, moving the request by as much
// as three degrees while the body moved only about 0--0.5 degrees and the
// endpoint did not improve.  Restore the proven 8-degree floor; Run267 changes
// only high-altitude attitude authority.
SET TERMINAL_DENSE_AIR_COMMAND_CONE_DEGREES TO 8.
// Run 109 first enters the legal vertical-speed window, but the final along
// state becomes unreachable: at 4.0 km the plant realises only 9.8 m/s2 while
// 24.4 is required, with an actual cone near five degrees.  Preserve the
// low-lift dense-air branch above 4.5 km, then continuously expose part of the
// still-legal forward/upright aerodynamic-brake edge.  The independent
// observer continues to enforce the unchanged 30-degree physical cone.
SET TERMINAL_FINAL_AERO_BRAKE_CONE_START_HEIGHT TO 4500.
SET TERMINAL_FINAL_AERO_BRAKE_CONE_FULL_HEIGHT TO 4000.
SET TERMINAL_FINAL_AERO_BRAKE_CONE_DEGREES TO 20.
SET TERMINAL_FINAL_AERO_BRAKE_CONE_RELEASE_END_HEIGHT TO 1500.
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
// Step 106 showed that all three bounded impulses must occur after the measured
// ~37.35 km entry cutoff but before the 23.6 km main gate.  The former first
// surface was already above cutoff and therefore could not supply its budget.
SET MIDCOURSE_CHECKPOINT_1_HEIGHT TO 35000.
SET MIDCOURSE_CHECKPOINT_2_HEIGHT TO 31500.
SET MIDCOURSE_CHECKPOINT_3_HEIGHT TO 28000.
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
SET MIDCOURSE_MAX_HORIZONTAL_ACCEL TO 21.5.
SET MIDCOURSE_MAX_THROTTLE TO 1.0.
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
SET MIDCOURSE_VERTICAL_THRUST_G TO 0.
SET MIDCOURSE_VERTICAL_TARGET_IGNITION_SPEED TO 600.
SET MIDCOURSE_VERTICAL_ERROR_DEADBAND TO 15.
// The endpoint predictor moves as vertical energy changes.  Runs 95/97 show
// 7-10 m/s of surface-horizontal recovery between checkpoint 3 and physical
// main alignment, so own a measured 960 m/s pulse-end state instead of letting
// a sign-changing predicted miss withdraw the high-altitude braking impulse.
SET MIDCOURSE_HORIZONTAL_TARGET_SPEED TO 800.
SET MIDCOURSE_HORIZONTAL_TARGET_DEADBAND TO 1.
// Runs 135--137 show that a 172--179 m short entrance is already infeasible by
// the saturated 4 km high-drag boundary. Advance the repeatable continuous-main
// commitment by only 50 m, before saturation, while preserving descent margin.
SET MIDCOURSE_SHAPED_MAIN_HANDOFF_HEIGHT TO 23650.
// A checkpoint may light only when the physical stage is close to the actual
// requested correction vector and angular motion has settled.  The requested
// vector itself is already projected into the mandatory load cone.
SET MIDCOURSE_IGNITION_CONE_DEGREES TO 20.
SET MIDCOURSE_IGNITION_MAX_ANGULAR_RATE_DEG TO 10.

// Continuous ModuleAeroSurface corridor identified in Step 106.  Height gives
// the nominal one-shot open/stow envelope; live horizontal and downward speed
// residuals move the command inside that envelope.  Both kOS and the part
// module rate-limit the coordinate, so this cannot become action-group PWM.
SET GRID_FIN_AERO_OPEN_START_HEIGHT TO 20500.
SET GRID_FIN_AERO_FULL_HEIGHT TO 16500.
SET GRID_FIN_AERO_STOW_START_HEIGHT TO 10000.
SET GRID_FIN_AERO_STOW_END_HEIGHT TO 7500.
// Run 103 measured 13--19 m/s^2 of upward aerodynamic acceleration at
// 63--64 degrees, opposite the recovered Step-106 vertical-force sign.  Hold
// independent deployment at zero for the Run-104 powered-baseline flight.
SET GRID_FIN_AERO_MAX_DEPLOYMENT_PERCENT TO 0.
SET GRID_FIN_AERO_COMMAND_RATE_PERCENT_PER_SECOND TO 15.
SET GRID_FIN_AERO_HORIZONTAL_ERROR_GAIN TO 1.0.
SET GRID_FIN_AERO_HORIZONTAL_ERROR_LIMIT_PERCENT TO 25.
SET GRID_FIN_AERO_DOWN_ERROR_GAIN TO 0.5.
SET GRID_FIN_AERO_DOWN_ERROR_LIMIT_PERCENT TO 15.
SET HYBRID_CORRIDOR_END_HEIGHT TO 6000.
SET HYBRID_CORRIDOR_HORIZONTAL_GAIN TO 0.15.
SET HYBRID_CORRIDOR_DOWN_GAIN TO 0.12.
SET HYBRID_CORRIDOR_MAX_FEEDBACK_ACCEL TO 5.

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
