@LAZYGLOBAL OFF.
RUNPATH("0:/cz10b/config.ks").
RUNPATH("0:/cz10b/controller.ks").

CLEARSCREEN.
PRINT "CZ-10B cable-net recovery demonstrator".
PRINT "Searching for vessel: " + TARGET_VESSEL_NAME.

LOCAL RECOVERY_SHIP IS 0.
LOCAL COUNTDOWN_SECONDS IS 5.
LOCAL AUTOMATED_DEPLOYMENT IS FALSE.
IF SHIP:PARTSNAMED("CZ10B-RecoveryPlatform"):LENGTH > 0 {
    PRINT "Waiting for integrated mission rail release.".
    WAIT UNTIL SHIP:PARTSNAMED("CZ10B-RecoveryPlatform"):LENGTH = 0.
    WAIT UNTIL HASTARGET.
    SET RECOVERY_SHIP TO TARGET.
    SET AUTOMATED_DEPLOYMENT TO TRUE.
} ELSE {
    IF NOT HASTARGET {
        // After the mission rail separates, kOS can boot several seconds before
        // the observer switches back to the booster and binds the platform.
        // LIST VESSELS is not safe during that unpack/switch window, so give the
        // automated handoff time to finish before using the manual-run fallback.
        LOCAL TARGET_BIND_TIMEOUT IS TIME:SECONDS + 12.
        WAIT UNTIL HASTARGET OR TIME:SECONDS >= TARGET_BIND_TIMEOUT.
    }
    IF HASTARGET {
        SET RECOVERY_SHIP TO TARGET.
        SET AUTOMATED_DEPLOYMENT TO TRUE.
    } ELSE {
        SET RECOVERY_SHIP TO FIND_RECOVERY_SHIP(TARGET_VESSEL_NAME).
    }
}
IF AUTOMATED_DEPLOYMENT {
    // The observer separates and deploys the recovery ship first. For a sea
    // run, wait until its great-circle distance proves the move has completed.
    LOCAL DEPLOYMENT_TIMEOUT IS TIME:SECONDS + 6.
    WAIT UNTIL RECOVERY_SHIP:GEOPOSITION:DISTANCE > 1000
        OR TIME:SECONDS >= DEPLOYMENT_TIMEOUT.
    SET COUNTDOWN_SECONDS TO 10.
}
IF RECOVERY_SHIP = 0 {
    PRINT "ERROR: Recovery Ship not found.".
    PRINT "Launch and park the ship first; keep its exact vessel name.".
    WAIT 10.
    SHUTDOWN.
}

// The automated observer already binds the remote platform. Rebinding it in
// this same physics frame can race KSP's PatchedConicSolver at long range.
IF NOT AUTOMATED_DEPLOYMENT AND NOT HASTARGET {
    SET TARGET TO RECOVERY_SHIP.
}
LOCAL MISSION_ID IS ROUND(TIME:SECONDS,0).
LOG "mission,phase,ut,altitude,hook_height,v_vertical,v_horizontal,h_error,throttle,tilt,mass,max_thrust,actual_tilt"
    TO "0:/cz10b/telemetry.csv".
// Limit the 80 kN-m terminal wheel to the original 20 kN-m through the
// high-energy flight. Full authority is enabled only after main cutoff.
LOCAL BOOSTER_WHEEL_LIMITED IS
    SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
        BOOSTER_FLIGHT_REACTION_WHEEL_AUTHORITY).
LOG MISSION_ID + ",WHEEL_LIMIT," + ROUND(TIME:SECONDS,3)
    + ",limited=" + BOOSTER_WHEEL_LIMITED
    TO "0:/cz10b/telemetry.csv".

SAS OFF.
RCS ON.
BRAKES OFF.
// Bind each kOS fly-by-wire channel once and update ordinary command
// variables thereafter.  Reissuing LOCK inside 20--50 Hz loops made kOS log
// ToggleFlyByWire every physics frame, eventually consuming tens of GB and
// slowing one uninterrupted mission far below real time.
LOCAL FLIGHT_THROTTLE_CMD IS 0.
LOCAL FLIGHT_STEERING_CMD IS UP.
LOCK THROTTLE TO FLIGHT_THROTTLE_CMD.

PRINT "Target locked: " + RECOVERY_SHIP:NAME.
PRINT "Booster probe core must be the craft root.".
FROM { LOCAL COUNTDOWN IS COUNTDOWN_SECONDS. } UNTIL COUNTDOWN = 0 STEP { SET COUNTDOWN TO COUNTDOWN - 1. } DO {
    PRINT "T-" + COUNTDOWN AT(0,8).
    WAIT 1.
}

// Start the engine at zero throttle while the real TT18-A clamps still carry
// the vehicle. Increase the command linearly over five seconds and release only
// after measured thrust-to-weight strictly exceeds the configured threshold.
SET FLIGHT_STEERING_CMD TO UP.
LOCK STEERING TO FLIGHT_STEERING_CMD.
SET FLIGHT_THROTTLE_CMD TO 0.
LOCAL ENGINE_LOAD_DEADLINE IS TIME:SECONDS + 15.
WAIT UNTIL SHIP:ENGINES:LENGTH > 0
    OR TIME:SECONDS >= ENGINE_LOAD_DEADLINE.
LOCAL MISSION_ENGINES IS SHIP:ENGINES.
IF MISSION_ENGINES:LENGTH = 0 {
    PRINT "ABORT: engine modules did not load" AT(0,12).
    SET FLIGHT_THROTTLE_CMD TO 0.
    SHUTDOWN.
}
LOCAL BOOSTER_ENGINE_COUNT IS 0.
LOCAL IGNITION_STARTED_AT IS TIME:SECONDS.
FOR ENGINE IN MISSION_ENGINES {
    IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
        SET BOOSTER_ENGINE_COUNT TO BOOSTER_ENGINE_COUNT + 1.
        SET ENGINE:THRUSTLIMIT TO ASCENT_ENGINE_THRUST_LIMIT.
        ENGINE:ACTIVATE.
    }
}
IF BOOSTER_ENGINE_COUNT = 0 {
    PRINT "ABORT: booster engine not found" AT(0,12).
    SET FLIGHT_THROTTLE_CMD TO 0.
    SHUTDOWN.
}
PRINT "ENGINE START - TT18-A HOLD" AT(0,10).
LOCAL RELEASE_TWR IS 0.
LOCAL RELEASE_DEADLINE IS IGNITION_STARTED_AT
    + ASCENT_IGNITION_RAMP_SECONDS + 5.
UNTIL RELEASE_TWR > ASCENT_CLAMP_RELEASE_TWR
      OR TIME:SECONDS >= RELEASE_DEADLINE {
    LOCAL IGNITION_RAMP_FRACTION IS CLAMP((TIME:SECONDS
        - IGNITION_STARTED_AT) / ASCENT_IGNITION_RAMP_SECONDS, 0, 1).
    SET FLIGHT_THROTTLE_CMD TO IGNITION_RAMP_FRACTION.
    LOCAL CURRENT_THRUST IS 0.
    FOR ENGINE IN MISSION_ENGINES {
        IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
            SET CURRENT_THRUST TO CURRENT_THRUST + ENGINE:THRUST.
        }
    }
    LOCAL PAD_G IS SHIP:BODY:MU / ((SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2).
    SET RELEASE_TWR TO CURRENT_THRUST / MAX(SHIP:MASS * PAD_G, 0.001).
    PRINT "hold-down TWR " + ROUND(RELEASE_TWR,2) + "   " AT(0,11).
    WAIT 0.05.
}
IF RELEASE_TWR <= ASCENT_CLAMP_RELEASE_TWR {
    PRINT "ABORT: TWR did not exceed release threshold" AT(0,12).
    SET FLIGHT_THROTTLE_CMD TO 0.
    SHUTDOWN.
}
LOCAL RELEASED_CLAMPS IS RELEASE_LAUNCH_CLAMPS().
PRINT "LIFTOFF - released " + RELEASED_CLAMPS + " TT18-A" AT(0,10).
LOG MISSION_ID + ",CLAMP_RELEASE," + ROUND(TIME:SECONDS,3)
    + ",count=" + RELEASED_CLAMPS + ",twr=" + ROUND(RELEASE_TWR,3)
    TO "0:/cz10b/telemetry.csv".

LOCAL LAST_LOG IS TIME:SECONDS.
UNTIL BOOSTER_PROPELLANT_FRACTION() <= ASCENT_RESERVE_FRACTION
      AND SHIP:ALTITUDE >= ASCENT_MIN_SEPARATION_ALTITUDE {
    LOCAL BOOSTER_RESERVE IS BOOSTER_PROPELLANT_FRACTION().
    LOCAL TURN_FRACTION IS CLAMP((SHIP:ALTITUDE - ASCENT_TURN_START)
        / (ASCENT_TURN_END - ASCENT_TURN_START), 0, 1).
    // Smoothstep avoids a pitch-rate discontinuity at either end of the turn.
    LOCAL TURN_BLEND IS TURN_FRACTION^2 * (3 - 2 * TURN_FRACTION).
    LOCAL PITCH_CMD IS 90 - ASCENT_TURN_DEGREES * TURN_BLEND.
    LOCAL ASCENT_THROTTLE_COMMAND IS 1.
    IF SHIP:ALTITUDE < ASCENT_SPEED_LIMIT_END
       AND SHIP:GROUNDSPEED > ASCENT_MAX_SPEED {
        SET ASCENT_THROTTLE_COMMAND TO ASCENT_HIGH_SPEED_THROTTLE.
    }
    // Continue the same ramp after clamp release.  This avoids a discontinuous
    // jump to full throttle if TWR crosses 1.05 before the five seconds expire.
    LOCAL ASCENT_RAMP_LIMIT IS CLAMP((TIME:SECONDS - IGNITION_STARTED_AT)
        / ASCENT_IGNITION_RAMP_SECONDS, 0, 1).
    SET FLIGHT_THROTTLE_CMD TO MIN(ASCENT_THROTTLE_COMMAND,
        ASCENT_RAMP_LIMIT).
    SET FLIGHT_STEERING_CMD TO HEADING(ASCENT_HEADING, PITCH_CMD).

    IF TIME:SECONDS - LAST_LOG >= TELEMETRY_PERIOD {
        WRITE_TELEMETRY("ASCENT", MISSION_ID, SHIP:ALTITUDE, 0,
            SHIP:VERTICALSPEED, SHIP:GROUNDSPEED, RECOVERY_SHIP:GEOPOSITION:DISTANCE,
            THROTTLE, 90 - PITCH_CMD).
        SET LAST_LOG TO TIME:SECONDS.
    }
    PRINT "first-stage reserve " + ROUND(BOOSTER_RESERVE * 100,1)
        + "%   " AT(0,12).
    PRINT "pitch / apoapsis " + ROUND(PITCH_CMD,1) + " deg / "
        + ROUND(SHIP:APOAPSIS / 1000,1) + " km   " AT(0,13).
    WAIT 0.02.
}

SET FLIGHT_THROTTLE_CMD TO 0.
// Cooked throttle overrides the pilot setting while kOS is healthy.  Zero the
// persistent pilot throttle as well so an unexpected processor abort returns
// to cutoff instead of restoring the ascent command.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
// The cable net owns the vehicle after all four hooks latch.  Leaving the kOS
// steering manager active makes the reaction wheels fight the compliant
// tethers, producing a periodic captured-stage sway even with zero throttle.
UNLOCK STEERING.
SAS OFF.
LOCAL SEPARATION_RESERVE IS BOOSTER_PROPELLANT_FRACTION().
LOG MISSION_ID + ",STAGE_RESERVE," + ROUND(TIME:SECONDS,3)
    + ",fraction=" + ROUND(SEPARATION_RESERVE,5)
    + ",altitude=" + ROUND(SHIP:ALTITUDE,1)
    + ",speed=" + ROUND(SHIP:GROUNDSPEED,1)
    TO "0:/cz10b/telemetry.csv".
LOCAL SEPARATION_ATTITUDE IS SHIP:FACING.
SET FLIGHT_STEERING_CMD TO SEPARATION_ATTITUDE.
LOCK STEERING TO FLIGHT_STEERING_CMD.
// Remove first-stage thrust authority before opening the interstage. A throttle
// pulse used to restart the higher-TWR booster while both stages still shared
// one vessel, briefly driving it into the upper stage.
FOR ENGINE IN MISSION_ENGINES {
    IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
        SET ENGINE:THRUSTLIMIT TO 0.
    }
}
LOCAL THRUST_UNLOAD_DEADLINE IS TIME:SECONDS + 2.
LOCAL BOOSTER_LIVE_THRUST IS 1.
UNTIL BOOSTER_LIVE_THRUST < 0.5 OR TIME:SECONDS >= THRUST_UNLOAD_DEADLINE {
    SET BOOSTER_LIVE_THRUST TO 0.
    FOR ENGINE IN MISSION_ENGINES {
        IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
            SET BOOSTER_LIVE_THRUST TO BOOSTER_LIVE_THRUST + ENGINE:THRUST.
        }
    }
    WAIT 0.02.
}
WAIT 0.3.
// Direct part operations remain reliable after the integrated test rail has
// been programmatically released, while KSP's UI staging icon list may not.
LOCAL INTERSTAGES IS SHIP:PARTSNAMED("Decoupler.2").
IF INTERSTAGES:LENGTH = 0 {
    PRINT "ERROR: interstage not found".
    SHUTDOWN.
}
LOCAL DECOUPLE_MODULE IS INTERSTAGES[0]:GETMODULE("ModuleDecouple").
LOCAL DECOUPLE_EVENTS IS DECOUPLE_MODULE:ALLEVENTNAMES.
IF DECOUPLE_EVENTS:LENGTH = 0 {
    PRINT "ERROR: interstage event unavailable".
    SHUTDOWN.
}
DECOUPLE_MODULE:DOEVENT(DECOUPLE_EVENTS[0]).
LOCAL SEPARATION_DEADLINE IS TIME:SECONDS + 3.
WAIT UNTIL SHIP:PARTSNAMED("CZ10B-DemoUpperStage"):LENGTH = 0
    OR TIME:SECONDS >= SEPARATION_DEADLINE.
IF SHIP:PARTSNAMED("CZ10B-DemoUpperStage"):LENGTH > 0 {
    // One transition retry covers the rare unloaded-event race without
    // continuously replaying the decoupler or its sound.
    DECOUPLE_MODULE:DOEVENT(DECOUPLE_EVENTS[0]).
    SET SEPARATION_DEADLINE TO TIME:SECONDS + 2.
    WAIT UNTIL SHIP:PARTSNAMED("CZ10B-DemoUpperStage"):LENGTH = 0
        OR TIME:SECONDS >= SEPARATION_DEADLINE.
}
IF SHIP:PARTSNAMED("CZ10B-DemoUpperStage"):LENGTH > 0 {
    PRINT "ERROR: upper stage did not separate".
    SET FLIGHT_THROTTLE_CMD TO 0.
    SHUTDOWN.
}
WAIT 0.6.
FOR ENGINE IN MISSION_ENGINES {
    IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
        SET ENGINE:THRUSTLIMIT TO RETURN_ENGINE_THRUST_LIMIT.
    }
}
SET FLIGHT_THROTTLE_CMD TO 0.
PRINT "STAGE SEPARATION - BOOSTER GUIDANCE ACTIVE" AT(0,11).
// The four grid-fin analogues are low-drag ModuleControlSurface panels.  They
// remain neutral during the coast and respond only to steering demand.

// 1) Rate-damped ascent coast.  Holding the separation attitude made the
// reaction wheels chase an orientation through the rapidly changing surface
// frame, while a fully passive stage was aerodynamically unstable.  kOS's
// special "kill" target commands angular-rate damping without chasing a
// changing direction.
PRINT "RATE-DAMPED ASCENT COAST" AT(0,11).
SET FLIGHT_THROTTLE_CMD TO 0.
SAS OFF.
RCS ON.
SET FLIGHT_STEERING_CMD TO "kill".
SET LAST_LOG TO TIME:SECONDS.
UNTIL SHIP:VERTICALSPEED <= 0 {
    IF TIME:SECONDS - LAST_LOG >= TELEMETRY_PERIOD {
        WRITE_TELEMETRY("COAST", MISSION_ID, SHIP:ALTITUDE, 0,
            SHIP:VERTICALSPEED, SHIP:GROUNDSPEED,
            RECOVERY_SHIP:GEOPOSITION:DISTANCE, 0, 0).
        SET LAST_LOG TO TIME:SECONDS.
    }
    WAIT 0.02.
}

// Keep damping body rates until 50 km on the descending branch. Then turn the
// nose retrograde so the nozzle points along the surface-velocity vector.
UNTIL SHIP:ALTITUDE <= ENTRY_RETROGRADE_HEIGHT {
    IF TIME:SECONDS - LAST_LOG >= TELEMETRY_PERIOD {
        WRITE_TELEMETRY("COAST", MISSION_ID, SHIP:ALTITUDE, 0,
            SHIP:VERTICALSPEED, SHIP:GROUNDSPEED,
            RECOVERY_SHIP:GEOPOSITION:DISTANCE, 0, 0).
        SET LAST_LOG TO TIME:SECONDS.
    }
    WAIT 0.02.
}
PRINT "CONTROLLED RETROGRADE ENTRY SLEW" AT(0,11).
RCS ON.
LOCAL FLIP_START_TIME IS TIME:SECONDS.
LOCAL FLIP_START_VECTOR IS SHIP:FACING:FOREVECTOR:NORMALIZED.
UNTIL TIME:SECONDS - FLIP_START_TIME >= ENTRY_ATTITUDE_SLEW_SECONDS {
    LOCAL FLIP_BLEND_RAW IS CLAMP((TIME:SECONDS - FLIP_START_TIME)
        / ENTRY_ATTITUDE_SLEW_SECONDS, 0, 1).
    LOCAL FLIP_BLEND IS FLIP_BLEND_RAW^2 * (3 - 2 * FLIP_BLEND_RAW).
    LOCAL FLIP_TARGET IS -SHIP:VELOCITY:SURFACE:NORMALIZED.
    LOCAL FLIP_COMMAND IS (FLIP_START_VECTOR * (1 - FLIP_BLEND)
        + FLIP_TARGET * FLIP_BLEND):NORMALIZED.
    SET FLIGHT_STEERING_CMD TO LOOKDIRUP(FLIP_COMMAND,
        SHIP:UP:VECTOR).
    WAIT 0.05.
}
SET FLIGHT_THROTTLE_CMD TO 0.
SET FLIGHT_STEERING_CMD TO LOOKDIRUP(
    -SHIP:VELOCITY:SURFACE:NORMALIZED, SHIP:UP:VECTOR).
// The four aerodynamic steering panels now supply dynamic-pressure-scaled
// authority automatically; unlike the rejected run-11 airbrake configuration,
// they do not deploy into the airstream and erase the planned downrange travel.

// 2) At 40 km, perform one continuous thermal/load-reduction burn. Its only velocity
// objective is the surface-horizontal gate; reaching that gate always cuts the
// throttle instead of flowing directly into the landing burn.
WAIT UNTIL SHIP:ALTITUDE <= ENTRY_DECEL_HEIGHT.
PRINT "40 KM CONTINUOUS ENTRY BURN" AT(0,11).
LOCAL ENTRY_H_SPEED IS VXCL(SHIP:UP:VECTOR:NORMALIZED,
    SHIP:VELOCITY:SURFACE):MAG.
LOCAL ENTRY_BURN_STARTED IS FALSE.
UNTIL ENTRY_H_SPEED <= ENTRY_HORIZONTAL_SPEED {
    LOCAL ENTRY_THRUST_AXIS IS -SHIP:VELOCITY:SURFACE:NORMALIZED.
    SET FLIGHT_STEERING_CMD TO LOOKDIRUP(ENTRY_THRUST_AXIS,
        SHIP:UP:VECTOR).
    // Start only when the physical vehicle is already inside the hard cone.
    // Once started, remain continuously on until the horizontal-speed cutoff.
    IF NOT ENTRY_BURN_STARTED
        AND VANG(SHIP:FACING:VECTOR, ENTRY_THRUST_AXIS)
            <= TERMINAL_COMMAND_CONE_DEGREES {
        SET ENTRY_BURN_STARTED TO TRUE.
    }
    IF ENTRY_BURN_STARTED { SET FLIGHT_THROTTLE_CMD TO 1. }
    ELSE { SET FLIGHT_THROTTLE_CMD TO 0. }
    SET ENTRY_H_SPEED TO VXCL(SHIP:UP:VECTOR:NORMALIZED,
        SHIP:VELOCITY:SURFACE):MAG.
    IF TIME:SECONDS - LAST_LOG >= TELEMETRY_PERIOD {
        WRITE_TELEMETRY("ENTRY_BURN", MISSION_ID, SHIP:ALTITUDE, 0,
            SHIP:VERTICALSPEED, ENTRY_H_SPEED,
            RECOVERY_SHIP:GEOPOSITION:DISTANCE,
            CHOOSE 1 IF ENTRY_BURN_STARTED ELSE 0, 0).
        SET LAST_LOG TO TIME:SECONDS.
    }
    WAIT 0.02.
}
SET FLIGHT_THROTTLE_CMD TO 0.
// Audit the zero-throttle handoff against a ballistic 2 km footprint.  A large
// miss is not hidden by extending the entry burn; the discrete checkpoints
// below own any correction and make the source of the error visible in logs.
LOCAL ENTRY_UP IS SHIP:UP:VECTOR:NORMALIZED.
LOCAL ENTRY_REL_POS IS NET_POSITION(RECOVERY_SHIP) - SHIP:POSITION.
LOCAL ENTRY_HOOK_HEIGHT IS -VDOT(ENTRY_REL_POS, ENTRY_UP)
    + BOOSTER_HOOK_OFFSET_ALONG_UP(ENTRY_UP).
LOCAL ENTRY_VERTICAL_V IS VDOT(SHIP:VELOCITY:SURFACE, ENTRY_UP).
LOCAL ENTRY_H_VEL IS VXCL(ENTRY_UP, SHIP:VELOCITY:SURFACE).
LOCAL ENTRY_G IS SHIP:BODY:MU
    / ((SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2).
LOCAL ENTRY_NOMINAL_ACCEL IS SHIP:AVAILABLETHRUST
    / MAX(SHIP:MASS, 0.001) * TERMINAL_NOMINAL_THRUST_FRACTION.
LOCAL ENTRY_BURN_DV_VERTICAL IS -TERMINAL_WAYPOINT_VERTICAL_SPEED
    - ENTRY_VERTICAL_V.
LOCAL ENTRY_BURN_DV IS -ENTRY_H_VEL
    + ENTRY_UP * ENTRY_BURN_DV_VERTICAL.
LOCAL ENTRY_BURN_TIME IS DEPLETING_BURN_TIME(ENTRY_BURN_DV, ENTRY_UP,
    ENTRY_G, SHIP:MASS, SHIP:AVAILABLETHRUST
        * TERMINAL_NOMINAL_THRUST_FRACTION,
    RETURN_ENGINE_EFFECTIVE_ISP, BOOSTER_RETURN_DRY_MASS).
LOCAL ENTRY_PLANNED_BURN_TIME IS ENTRY_BURN_TIME
    + TERMINAL_GUIDANCE_RESPONSE_SECONDS.
LOCAL ENTRY_BURN_DROP IS MAX(-(ENTRY_VERTICAL_V
    - TERMINAL_WAYPOINT_VERTICAL_SPEED) * 0.5
    * ENTRY_PLANNED_BURN_TIME, 0).
LOCAL ENTRY_IGNITION_HEIGHT IS TERMINAL_WAYPOINT_HEIGHT
    + ENTRY_BURN_DROP * TERMINAL_IGNITION_SAFETY
    + TERMINAL_IGNITION_MARGIN.
LOCAL ENTRY_COAST_TGO IS 0.
IF ENTRY_HOOK_HEIGHT > ENTRY_IGNITION_HEIGHT {
    SET ENTRY_COAST_TGO TO COAST_TIME_TO_HEIGHT(ENTRY_HOOK_HEIGHT,
        ENTRY_VERTICAL_V, ENTRY_IGNITION_HEIGHT, ENTRY_G).
}
LOCAL ENTRY_IGNITION_VERTICAL_V IS ENTRY_VERTICAL_V
    - ENTRY_G * ENTRY_COAST_TGO.
LOCAL ENTRY_BURN_NET_VERTICAL_ACCEL IS
    (-TERMINAL_WAYPOINT_VERTICAL_SPEED - ENTRY_IGNITION_VERTICAL_V)
    / MAX(ENTRY_PLANNED_BURN_TIME, 0.1).
LOCAL ENTRY_HORIZONTAL_BURN_DROP IS MAX(ENTRY_IGNITION_HEIGHT
    - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 0).
LOCAL ENTRY_HORIZONTAL_BURN_TIME IS ENTRY_HORIZONTAL_BURN_DROP
    / MAX(-ENTRY_IGNITION_VERTICAL_V, 1).
IF ENTRY_BURN_NET_VERTICAL_ACCEL > 0.001 {
    LOCAL ENTRY_HORIZONTAL_TIME_DISC IS MAX(ENTRY_IGNITION_VERTICAL_V^2
        - 2 * ENTRY_BURN_NET_VERTICAL_ACCEL
            * ENTRY_HORIZONTAL_BURN_DROP, 0).
    SET ENTRY_HORIZONTAL_BURN_TIME TO
        (-ENTRY_IGNITION_VERTICAL_V
            - SQRT(ENTRY_HORIZONTAL_TIME_DISC))
        / ENTRY_BURN_NET_VERTICAL_ACCEL.
}
SET ENTRY_HORIZONTAL_BURN_TIME TO CLAMP(ENTRY_HORIZONTAL_BURN_TIME,
    0, ENTRY_PLANNED_BURN_TIME).
LOCAL ENTRY_APPROACH_OFFSET IS V(0,0,0).
IF ENTRY_H_VEL:MAG > 0.1 {
    SET ENTRY_APPROACH_OFFSET TO ENTRY_H_VEL:NORMALIZED
        * TERMINAL_WAYPOINT_APPROACH_OFFSET.
}
LOCAL ENTRY_PREDICTED_MISS IS VXCL(ENTRY_UP, ENTRY_REL_POS)
    - ENTRY_H_VEL * (ENTRY_COAST_TGO
        + 0.5 * (ENTRY_HORIZONTAL_BURN_TIME
            + TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS))
    - ENTRY_APPROACH_OFFSET.
WRITE_TELEMETRY("ENTRY_CUTOFF", MISSION_ID, SHIP:ALTITUDE,
    ENTRY_HOOK_HEIGHT, ENTRY_VERTICAL_V, ENTRY_H_VEL:MAG,
    ENTRY_PREDICTED_MISS:MAG, 0, 0).
PRINT "ENTRY CUTOFF / COAST" AT(0,11).
PRINT "h-speed / miss " + ROUND(ENTRY_H_VEL:MAG,0) + " / "
    + ROUND(ENTRY_PREDICTED_MISS:MAG,0) + " m" AT(0,12).

// Entry uses the fixed landing-engine subset.  From this zero-throttle handoff
// onward, schedule its limiter with mass so that a 75% main command represents
// one stable acceleration envelope instead of becoming almost twice as strong
// near propellant depletion.
LOCAL RETURN_ACCEL_NORMALIZED IS NORMALIZE_BOOSTER_ENGINE_ACCEL(
    BOOSTER_ENGINE_PART_NAME, RETURN_ENGINE_MAX_ACCEL,
    RETURN_ENGINE_MIN_THRUST_LIMIT, RETURN_ENGINE_THRUST_LIMIT).
LOCAL RETURN_ACCEL_LAST_UPDATE IS TIME:SECONDS.
WAIT 0.04.

// Coast after the thermal burn.  Three altitude checkpoints may each issue
// one short horizontal-only correction pulse.  Main ignition is calculated
// from the full velocity change to the 2 km waypoint at exactly 75% thrust.
LOCAL BALLISTIC_PREVIOUS_TIME IS TIME:SECONDS.
LOCAL BALLISTIC_PREVIOUS_V IS SHIP:VELOCITY:SURFACE.
LOCAL FILTERED_VERTICAL_DRAG IS 0.
LOCAL FILTERED_HORIZONTAL_DRAG IS V(0,0,0).
LOCAL TERMINAL_IGNITION IS FALSE.
LOCAL BALLISTIC_WAS_POWERED IS TRUE.
LOCAL MIDCOURSE_CHECKPOINT_INDEX IS 0.
LOCAL MIDCOURSE_PULSE_ACTIVE IS FALSE.
LOCAL MIDCOURSE_PULSE_END IS -1.
LOCAL MIDCOURSE_PULSE_START_HEIGHT IS -1.
LOCAL MIDCOURSE_PULSE_START_H_VEL IS V(0,0,0).
LOCAL MIDCOURSE_PULSE_BURN_STARTED IS FALSE.
LOCAL MIDCOURSE_LAST_STEERING IS V(0,0,0).
// High-q attitude authority is enabled only after the dedicated entry burn has
// ended. It stays active through checkpoint and main guidance; the existing
// near-field handoff restores the conservative flight authority below 2 km.
LOCAL BALLISTIC_TERMINAL_WHEEL_ENABLED IS
    SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
        BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY).
// Apply the configured post-entry coefficient independently of deployment.
// Run 104 measured a large upward force even at zero deployment and therefore
// requires the old zero-lift baseline before partial authority can be solved.
LOCAL BALLISTIC_GRID_FINS_ENABLED IS
    SET_BOOSTER_GRID_FIN_LIFT_AUTHORITY(
        BOOSTER_TERMINAL_GRID_FIN_LIFT_AUTHORITY).
LOCAL BALLISTIC_GRID_FINS_STOWED IS
    SET_BOOSTER_GRID_FIN_DEPLOYMENT(0).
LOG MISSION_ID + ",GRID_FIN_LIFT," + ROUND(TIME:SECONDS,3)
    + ",count=" + BALLISTIC_GRID_FINS_ENABLED
    + ",authority=" + BOOSTER_TERMINAL_GRID_FIN_LIFT_AUTHORITY
    + ",stowed=" + BALLISTIC_GRID_FINS_STOWED
    TO "0:/cz10b/telemetry.csv".
SET LAST_LOG TO TIME:SECONDS.
UNTIL TERMINAL_IGNITION {
    LOCAL BALLISTIC_NOW IS TIME:SECONDS.
    // This loop can span several KSP physics updates when the kOS instruction
    // budget is busy.  Acceleration is a physical velocity derivative, so its
    // denominator must be the full interval between the two stored samples.
    // Run 75 measured 0.32--0.34 s iterations; truncating them to 0.1 s
    // inflated identified drag by more than three times.
    LOCAL BALLISTIC_DT IS MAX(BALLISTIC_NOW - BALLISTIC_PREVIOUS_TIME,
        0.001).
    LOCAL BALLISTIC_UP IS SHIP:UP:VECTOR:NORMALIZED.
    LOCAL BALLISTIC_REL_POS IS NET_POSITION(RECOVERY_SHIP) - SHIP:POSITION.
    LOCAL BALLISTIC_H_POS IS VXCL(BALLISTIC_UP, BALLISTIC_REL_POS).
    // The DP2 platform is fixed in the rotating surface frame.  A remote
    // vessel's VELOCITY:SURFACE is expressed through its own local/floating
    // frame and cannot be subtracted as a common Cartesian vector.
    LOCAL BALLISTIC_REL_VEL IS SHIP:VELOCITY:SURFACE.
    LOCAL BALLISTIC_VERTICAL_V IS VDOT(BALLISTIC_REL_VEL, BALLISTIC_UP).
    LOCAL BALLISTIC_H_VEL IS VXCL(BALLISTIC_UP, BALLISTIC_REL_VEL).
    LOCAL BALLISTIC_HEIGHT IS -VDOT(BALLISTIC_REL_POS, BALLISTIC_UP)
        + BOOSTER_HOOK_OFFSET_ALONG_UP(BALLISTIC_UP).
    LOCAL BALLISTIC_G IS SHIP:BODY:MU
        / ((SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2).
    LOCAL BALLISTIC_ACCEL IS (SHIP:VELOCITY:SURFACE
        - BALLISTIC_PREVIOUS_V) / BALLISTIC_DT.
    LOCAL VERTICAL_DRAG_SAMPLE IS CLAMP(
        VDOT(BALLISTIC_ACCEL, BALLISTIC_UP) + BALLISTIC_G, 0, 40).
    LOCAL HORIZONTAL_DRAG_SAMPLE IS VXCL(BALLISTIC_UP, BALLISTIC_ACCEL).
    // Do not mistake a correction pulse for aerodynamic drag.
    IF NOT BALLISTIC_WAS_POWERED {
        SET FILTERED_VERTICAL_DRAG TO FILTERED_VERTICAL_DRAG
            * (1 - DRAG_ACCEL_FILTER)
            + VERTICAL_DRAG_SAMPLE * DRAG_ACCEL_FILTER.
        SET FILTERED_HORIZONTAL_DRAG TO FILTERED_HORIZONTAL_DRAG
            * (1 - DRAG_ACCEL_FILTER)
            + HORIZONTAL_DRAG_SAMPLE * DRAG_ACCEL_FILTER.
    }

    IF BALLISTIC_NOW - RETURN_ACCEL_LAST_UPDATE
            >= RETURN_ENGINE_ACCEL_UPDATE_SECONDS {
        SET RETURN_ACCEL_NORMALIZED TO NORMALIZE_BOOSTER_ENGINE_ACCEL(
            BOOSTER_ENGINE_PART_NAME, RETURN_ENGINE_MAX_ACCEL,
            RETURN_ENGINE_MIN_THRUST_LIMIT, RETURN_ENGINE_THRUST_LIMIT).
        SET RETURN_ACCEL_LAST_UPDATE TO BALLISTIC_NOW.
    }
    LOCAL BALLISTIC_AVAILABLE_ACCEL IS SHIP:AVAILABLETHRUST
        / MAX(SHIP:MASS, 0.001).
    // Solve |delta_v / t + g_up| = nominal_acceleration.  This includes both
    // the horizontal stop and the requested vertical 2 km state, unlike the
    // old vertical-only stop distance to the net plane.
    LOCAL EFFECTIVE_G IS MAX(BALLISTIC_G - FILTERED_VERTICAL_DRAG, 1).
    LOCAL NOMINAL_BURN_ACCEL IS BALLISTIC_AVAILABLE_ACCEL
        * TERMINAL_NOMINAL_THRUST_FRACTION.
    LOCAL BURN_DV_VERTICAL IS -TERMINAL_WAYPOINT_VERTICAL_SPEED
        - BALLISTIC_VERTICAL_V.
    LOCAL BURN_DV IS -BALLISTIC_H_VEL
        + BALLISTIC_UP * BURN_DV_VERTICAL.
    LOCAL NOMINAL_BURN_TIME IS CONSTANT_ACCEL_BURN_TIME(BURN_DV,
        BALLISTIC_UP, EFFECTIVE_G, NOMINAL_BURN_ACCEL).
    LOCAL PLANNED_BURN_TIME IS NOMINAL_BURN_TIME
        + TERMINAL_GUIDANCE_RESPONSE_SECONDS.
    LOCAL NOMINAL_BURN_DROP IS MAX(-(BALLISTIC_VERTICAL_V
        - TERMINAL_WAYPOINT_VERTICAL_SPEED) * 0.5
        * PLANNED_BURN_TIME, 0).
    LOCAL BALLISTIC_IGNITION_HEIGHT IS TERMINAL_WAYPOINT_HEIGHT
        + NOMINAL_BURN_DROP * TERMINAL_IGNITION_SAFETY
        + TERMINAL_IGNITION_MARGIN.
    // Predict the complete coast-plus-burn footprint, not the old ballistic
    // impact point.  At constant acceleration a burn ending at zero horizontal
    // speed covers half of its ignition horizontal velocity times burn time.
    LOCAL BALLISTIC_COAST_TGO IS 0.
    IF BALLISTIC_HEIGHT > BALLISTIC_IGNITION_HEIGHT {
        SET BALLISTIC_COAST_TGO TO COAST_TIME_TO_HEIGHT(BALLISTIC_HEIGHT,
            BALLISTIC_VERTICAL_V, BALLISTIC_IGNITION_HEIGHT, EFFECTIVE_G).
    }
    LOCAL PREDICTED_IGNITION_H_VEL IS BALLISTIC_H_VEL
        + FILTERED_HORIZONTAL_DRAG * BALLISTIC_COAST_TGO.
    LOCAL PREDICTED_IGNITION_VERTICAL_V IS BALLISTIC_VERTICAL_V
        - EFFECTIVE_G * BALLISTIC_COAST_TGO.
    // The continuous main burn cannot coast after its 75% floor begins.  Give
    // Every checkpoint owns the same measured horizontal-energy residual.
    // Vertical shaping is disabled for the Step-106 horizontal cone-edge
    // experiment so its impulse cannot silently replace the descent corridor.
    LOCAL MIDCOURSE_VERTICAL_SPEED_EXCESS IS MAX(
        -PREDICTED_IGNITION_VERTICAL_V
            - MIDCOURSE_VERTICAL_TARGET_IGNITION_SPEED, 0).
    LOCAL MIDCOURSE_VERTICAL_SHAPING_REQUESTED IS
        MIDCOURSE_CHECKPOINT_INDEX >= 0
        AND MIDCOURSE_VERTICAL_THRUST_G > 0
        AND MIDCOURSE_VERTICAL_SPEED_EXCESS
            > MIDCOURSE_VERTICAL_ERROR_DEADBAND.
    LOCAL MIDCOURSE_HORIZONTAL_SPEED_EXCESS IS MAX(
        BALLISTIC_H_VEL:MAG - MIDCOURSE_HORIZONTAL_TARGET_SPEED, 0).
    LOCAL MIDCOURSE_HORIZONTAL_SHAPING_REQUESTED IS
        MIDCOURSE_CHECKPOINT_INDEX >= 0
        AND MIDCOURSE_HORIZONTAL_TARGET_SPEED > 0
        AND MIDCOURSE_HORIZONTAL_SPEED_EXCESS
            > MIDCOURSE_HORIZONTAL_TARGET_DEADBAND.
    LOCAL PREDICTED_BURN_NET_VERTICAL_ACCEL IS
        (-TERMINAL_WAYPOINT_VERTICAL_SPEED
            - PREDICTED_IGNITION_VERTICAL_V)
        / MAX(PLANNED_BURN_TIME, 0.1).
    LOCAL PREDICTED_HORIZONTAL_BURN_DROP IS MAX(
        BALLISTIC_IGNITION_HEIGHT
            - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 0).
    LOCAL PREDICTED_HORIZONTAL_BURN_TIME IS
        PREDICTED_HORIZONTAL_BURN_DROP
        / MAX(-PREDICTED_IGNITION_VERTICAL_V, 1).
    IF PREDICTED_BURN_NET_VERTICAL_ACCEL > 0.001 {
        LOCAL PREDICTED_HORIZONTAL_TIME_DISC IS MAX(
            PREDICTED_IGNITION_VERTICAL_V^2
            - 2 * PREDICTED_BURN_NET_VERTICAL_ACCEL
                * PREDICTED_HORIZONTAL_BURN_DROP, 0).
        SET PREDICTED_HORIZONTAL_BURN_TIME TO
            (-PREDICTED_IGNITION_VERTICAL_V
                - SQRT(PREDICTED_HORIZONTAL_TIME_DISC))
            / PREDICTED_BURN_NET_VERTICAL_ACCEL.
    }
    SET PREDICTED_HORIZONTAL_BURN_TIME TO CLAMP(
        PREDICTED_HORIZONTAL_BURN_TIME, 0, PLANNED_BURN_TIME).
    LOCAL PREDICTED_APPROACH_DISTANCE IS CLAMP(
        TERMINAL_WAYPOINT_APPROACH_OFFSET
            + TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN
                * (-PREDICTED_IGNITION_VERTICAL_V
                    - TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED),
        TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET,
        TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET).
    LOCAL PREDICTED_APPROACH_OFFSET IS V(0,0,0).
    IF PREDICTED_IGNITION_H_VEL:MAG > 0.1 {
        SET PREDICTED_APPROACH_OFFSET TO
            PREDICTED_IGNITION_H_VEL:NORMALIZED
            * PREDICTED_APPROACH_DISTANCE.
    }
    LOCAL PREDICTED_DISPLACEMENT IS BALLISTIC_H_VEL
            * BALLISTIC_COAST_TGO
        + FILTERED_HORIZONTAL_DRAG
            * (0.5 * BALLISTIC_COAST_TGO^2)
        + PREDICTED_IGNITION_H_VEL
            * (0.5 * (PREDICTED_HORIZONTAL_BURN_TIME
                + TERMINAL_HORIZONTAL_FOOTPRINT_LAG_SECONDS)).
    LOCAL PREDICTED_MISS IS BALLISTIC_H_POS - PREDICTED_DISPLACEMENT
        - PREDICTED_APPROACH_OFFSET.
    // A positive component along the current horizontal velocity means the
    // ballistic footprint is short and would require prograde thrust.  That
    // component is unavailable inside the mandatory nozzle/velocity cone, but
    // it must not suppress an independently feasible cross-track correction.
    // Run 58 skipped checkpoint 1 because its large prograde residual made the
    // whole vector look infeasible, leaving a measured 246 m cross-track error
    // at main ignition and about 486 m at the 2 km plane.
    LOCAL PREDICTED_MISS_ALONG_DIRECTION IS V(0,0,0).
    IF BALLISTIC_H_VEL:MAG > 0.1 {
        SET PREDICTED_MISS_ALONG_DIRECTION TO
            BALLISTIC_H_VEL:NORMALIZED.
    }
    LOCAL PREDICTED_MISS_ALONG IS 0.
    IF PREDICTED_MISS_ALONG_DIRECTION:MAG > 0.1 {
        SET PREDICTED_MISS_ALONG TO VDOT(PREDICTED_MISS,
            PREDICTED_MISS_ALONG_DIRECTION).
    }
    LOCAL MIDCOURSE_FEASIBLE_MISS IS PREDICTED_MISS.
    IF PREDICTED_MISS_ALONG > 0 {
        SET MIDCOURSE_FEASIBLE_MISS TO PREDICTED_MISS
            - PREDICTED_MISS_ALONG_DIRECTION
                * PREDICTED_MISS_ALONG.
    }

    LOCAL NEXT_MIDCOURSE_HEIGHT IS -1.
    IF MIDCOURSE_CHECKPOINT_INDEX = 0 {
        SET NEXT_MIDCOURSE_HEIGHT TO MIDCOURSE_CHECKPOINT_1_HEIGHT.
    } ELSE IF MIDCOURSE_CHECKPOINT_INDEX = 1 {
        SET NEXT_MIDCOURSE_HEIGHT TO MIDCOURSE_CHECKPOINT_2_HEIGHT.
    } ELSE IF MIDCOURSE_CHECKPOINT_INDEX = 2 {
        SET NEXT_MIDCOURSE_HEIGHT TO MIDCOURSE_CHECKPOINT_3_HEIGHT.
    }
    IF NOT MIDCOURSE_PULSE_ACTIVE AND NEXT_MIDCOURSE_HEIGHT > 0
        AND BALLISTIC_HEIGHT <= NEXT_MIDCOURSE_HEIGHT {
        LOCAL CHECKPOINT_FEASIBLE_DIRECTION IS V(0,0,0).
        IF MIDCOURSE_FEASIBLE_MISS:MAG > 0.001 {
            SET CHECKPOINT_FEASIBLE_DIRECTION TO
                MIDCOURSE_FEASIBLE_MISS:NORMALIZED.
        }
        WRITE_TELEMETRY("TRAJECTORY_CHECK", MISSION_ID, SHIP:ALTITUDE,
            BALLISTIC_HEIGHT, BALLISTIC_VERTICAL_V,
            BALLISTIC_H_VEL:MAG, PREDICTED_MISS:MAG, 0, 0).
        LOG MISSION_ID + ",CHECKPOINT_VECTOR,"
            + ROUND(BALLISTIC_NOW,3)
            + ",total=" + ROUND(PREDICTED_MISS:MAG,2)
            + ",along=" + ROUND(PREDICTED_MISS_ALONG,2)
            + ",feasible=" + ROUND(MIDCOURSE_FEASIBLE_MISS:MAG,2)
            + ",dragMag=" + ROUND(FILTERED_HORIZONTAL_DRAG:MAG,3)
            + ",dragAlong=" + ROUND(VDOT(FILTERED_HORIZONTAL_DRAG,
                PREDICTED_MISS_ALONG_DIRECTION),3)
            + ",dragTowardFeasible=" + ROUND(VDOT(
                FILTERED_HORIZONTAL_DRAG,
                CHECKPOINT_FEASIBLE_DIRECTION),3)
            + ",tgo=" + ROUND(MAX(BALLISTIC_COAST_TGO
                + PLANNED_BURN_TIME,1),3)
            + ",desiredDv=" + ROUND(MIDCOURSE_FEASIBLE_MISS:MAG
                / MAX(BALLISTIC_COAST_TGO + PLANNED_BURN_TIME,1)
                * MIDCOURSE_MISS_GAIN,3)
            + ",verticalExcess="
                + ROUND(MIDCOURSE_VERTICAL_SPEED_EXCESS,3)
            + ",verticalShaping="
                + MIDCOURSE_VERTICAL_SHAPING_REQUESTED
            + ",horizontalSpeed=" + ROUND(BALLISTIC_H_VEL:MAG,3)
            + ",horizontalTarget="
                + ROUND(MIDCOURSE_HORIZONTAL_TARGET_SPEED,3)
            + ",horizontalExcess="
                + ROUND(MIDCOURSE_HORIZONTAL_SPEED_EXCESS,3)
            + ",horizontalShaping="
                + MIDCOURSE_HORIZONTAL_SHAPING_REQUESTED
            + ",targetPos=" + ROUND(VDOT(BALLISTIC_H_POS,
                CHECKPOINT_FEASIBLE_DIRECTION),2)
            + ",targetVel=" + ROUND(VDOT(BALLISTIC_H_VEL,
                CHECKPOINT_FEASIBLE_DIRECTION),3)
            TO "0:/cz10b/telemetry.csv".
        IF (MIDCOURSE_FEASIBLE_MISS:MAG > MIDCOURSE_PREDICTED_ERROR
                OR MIDCOURSE_VERTICAL_SHAPING_REQUESTED
                OR MIDCOURSE_HORIZONTAL_SHAPING_REQUESTED)
            AND BALLISTIC_HEIGHT > BALLISTIC_IGNITION_HEIGHT {
            SET MIDCOURSE_PULSE_ACTIVE TO TRUE.
            SET MIDCOURSE_PULSE_END TO -1.
            SET MIDCOURSE_PULSE_BURN_STARTED TO FALSE.
            SET MIDCOURSE_PULSE_START_HEIGHT TO BALLISTIC_HEIGHT.
            SET MIDCOURSE_PULSE_START_H_VEL TO BALLISTIC_H_VEL.
            PRINT "CHECKPOINT CORRECTION "
                + (MIDCOURSE_CHECKPOINT_INDEX + 1) AT(0,11).
        } ELSE {
            SET MIDCOURSE_CHECKPOINT_INDEX TO
                MIDCOURSE_CHECKPOINT_INDEX + 1.
        }
    }
    IF MIDCOURSE_PULSE_ACTIVE
        AND ((MIDCOURSE_PULSE_BURN_STARTED
                AND BALLISTIC_NOW >= MIDCOURSE_PULSE_END)
            OR (MIDCOURSE_FEASIBLE_MISS:MAG
                    <= MIDCOURSE_PREDICTED_ERROR
                AND NOT MIDCOURSE_VERTICAL_SHAPING_REQUESTED
                AND NOT MIDCOURSE_HORIZONTAL_SHAPING_REQUESTED)
            OR (MIDCOURSE_PULSE_BURN_STARTED
                AND (BALLISTIC_H_VEL
                    - MIDCOURSE_PULSE_START_H_VEL):MAG
                    >= MIDCOURSE_MAX_DELTA_V)
            OR (MIDCOURSE_PULSE_BURN_STARTED
                AND BALLISTIC_HEIGHT <= MIDCOURSE_PULSE_START_HEIGHT
                    - MIDCOURSE_MAX_HEIGHT_DROP)
            OR (NOT MIDCOURSE_PULSE_BURN_STARTED
                AND BALLISTIC_HEIGHT <= MIDCOURSE_PULSE_START_HEIGHT
                    - MIDCOURSE_MAX_ALIGNMENT_HEIGHT_DROP)
            OR BOOSTER_PROPELLANT_FRACTION()
                <= MIDCOURSE_MIN_FUEL_FRACTION) {
        SET MIDCOURSE_PULSE_ACTIVE TO FALSE.
        SET MIDCOURSE_PULSE_BURN_STARTED TO FALSE.
        SET MIDCOURSE_CHECKPOINT_INDEX TO MIDCOURSE_CHECKPOINT_INDEX + 1.
    }
    LOCAL MIDCOURSE_ACTIVE IS MIDCOURSE_PULSE_ACTIVE
        AND BALLISTIC_HEIGHT > BALLISTIC_IGNITION_HEIGHT.
    LOCAL MIDCOURSE_THROTTLE IS 0.
    LOCAL MIDCOURSE_TILT IS 0.
    LOCAL MIDCOURSE_TARGET_ALONG_ACCEL IS 0.
    IF MIDCOURSE_ACTIVE {
        // Include both the remaining coast and the powered footprint.  The old
        // checkpoint branch referenced the removed BALLISTIC_TGO variable and
        // aborted kOS on its first real correction pulse.
        LOCAL MIDCOURSE_TGO IS MAX(BALLISTIC_COAST_TGO
            + PLANNED_BURN_TIME, 1).
        // PREDICTED_MISS already includes the remaining coast and the planned
        // main-burn footprint.  Correct only that endpoint residual.  Solving
        // a fresh desired velocity from position also absorbed the main
        // burn's along-track braking into this three-second pulse; the vector
        // limit then left almost no authority for the measured cross-track
        // miss.
        LOCAL MIDCOURSE_DELTA_V IS MIDCOURSE_FEASIBLE_MISS
            / MIDCOURSE_TGO * MIDCOURSE_MISS_GAIN.
        LOCAL MIDCOURSE_H_ACCEL IS CLAMPV(MIDCOURSE_DELTA_V
                / MAX(MIDCOURSE_PULSE_SECONDS, 0.1),
            MIDCOURSE_MAX_HORIZONTAL_ACCEL).
        // The predicted footprint is still authoritative for cross-track
        // correction, but its along component changed sign during Run 97 and
        // withdrew braking at 979 m/s.  Checkpoints 2/3 therefore own one
        // measured pulse-end state.  Latch the needed acceleration from the
        // pulse-start speed; release it at the target and never reverse it.
        IF MIDCOURSE_CHECKPOINT_INDEX >= 0
            AND MIDCOURSE_PULSE_START_H_VEL:MAG
                > MIDCOURSE_HORIZONTAL_TARGET_SPEED
                    + MIDCOURSE_HORIZONTAL_TARGET_DEADBAND
            AND BALLISTIC_H_VEL:MAG
                > MIDCOURSE_HORIZONTAL_TARGET_SPEED
            AND BALLISTIC_H_VEL:MAG > 0.1 {
            SET MIDCOURSE_TARGET_ALONG_ACCEL TO
                (MIDCOURSE_PULSE_START_H_VEL:MAG
                    - MIDCOURSE_HORIZONTAL_TARGET_SPEED)
                / MAX(MIDCOURSE_PULSE_SECONDS, 0.1).
            SET MIDCOURSE_TARGET_ALONG_ACCEL TO MIN(
                MIDCOURSE_TARGET_ALONG_ACCEL,
                MIDCOURSE_MAX_HORIZONTAL_ACCEL).
            LOCAL MIDCOURSE_CURRENT_ALONG_ACCEL IS VDOT(
                MIDCOURSE_H_ACCEL, BALLISTIC_H_VEL:NORMALIZED).
            IF MIDCOURSE_CURRENT_ALONG_ACCEL
                    > -MIDCOURSE_TARGET_ALONG_ACCEL {
                SET MIDCOURSE_H_ACCEL TO MIDCOURSE_H_ACCEL
                    - BALLISTIC_H_VEL:NORMALIZED
                        * (MIDCOURSE_CURRENT_ALONG_ACCEL
                            + MIDCOURSE_TARGET_ALONG_ACCEL).
            }
            SET MIDCOURSE_H_ACCEL TO CLAMPV(MIDCOURSE_H_ACCEL,
                MIDCOURSE_MAX_HORIZONTAL_ACCEL).
        }
        LOCAL MIDCOURSE_VERTICAL_THRUST IS 0.
        IF MIDCOURSE_VERTICAL_SHAPING_REQUESTED {
            SET MIDCOURSE_VERTICAL_THRUST TO BALLISTIC_G
                * MIDCOURSE_VERTICAL_THRUST_G.
        }
        LOCAL MIDCOURSE_THRUST_REQUEST IS BALLISTIC_UP
            * MIDCOURSE_VERTICAL_THRUST + MIDCOURSE_H_ACCEL.
        LOCAL MIDCOURSE_SAFE_AXIS IS THRUST_SAFETY_AXIS(BALLISTIC_UP,
            TERMINAL_VELOCITY_CONE_MIN_SPEED).
        LOCAL MIDCOURSE_THRUST IS CONSTRAIN_THRUST_VECTOR(
            MIDCOURSE_THRUST_REQUEST, MIDCOURSE_SAFE_AXIS,
            TERMINAL_COMMAND_CONE_DEGREES).
        LOCAL MIDCOURSE_REQUEST_EFFECTIVENESS IS VDOT(
            MIDCOURSE_THRUST_REQUEST:NORMALIZED,
            MIDCOURSE_THRUST:NORMALIZED).
        IF MIDCOURSE_REQUEST_EFFECTIVENESS
                < MIDCOURSE_MIN_REQUEST_EFFECTIVENESS {
            // Run 35 needed more downrange travel, but projecting that forward
            // request into the velocity cone produced a retrograde impulse and
            // increased the final undershoot.  A fixed checkpoint remains an
            // observation; it is not permission to burn in the wrong direction.
            WRITE_TELEMETRY("CHECKPOINT_SKIPPED", MISSION_ID,
                SHIP:ALTITUDE, BALLISTIC_HEIGHT, BALLISTIC_VERTICAL_V,
                BALLISTIC_H_VEL:MAG, PREDICTED_MISS:MAG, 0,
                MIDCOURSE_REQUEST_EFFECTIVENESS).
            SET MIDCOURSE_PULSE_ACTIVE TO FALSE.
            SET MIDCOURSE_PULSE_BURN_STARTED TO FALSE.
            SET MIDCOURSE_CHECKPOINT_INDEX TO
                MIDCOURSE_CHECKPOINT_INDEX + 1.
            SET MIDCOURSE_ACTIVE TO FALSE.
            SET MIDCOURSE_THROTTLE TO 0.
            SET MIDCOURSE_TILT TO 0.
            SET FLIGHT_STEERING_CMD TO LOOKDIRUP(MIDCOURSE_SAFE_AXIS,
                BALLISTIC_UP).
            SET FLIGHT_THROTTLE_CMD TO 0.
        } ELSE {
            SET MIDCOURSE_THROTTLE TO CLAMP(SHIP:MASS
                * MIDCOURSE_THRUST:MAG / MAX(SHIP:AVAILABLETHRUST, 0.001),
                0, MIDCOURSE_MAX_THROTTLE).
            SET MIDCOURSE_TILT TO VANG(MIDCOURSE_THRUST, BALLISTIC_UP).
            SET FLIGHT_STEERING_CMD TO LOOKDIRUP(MIDCOURSE_THRUST,
                SHIP:FACING:TOPVECTOR).
            SET MIDCOURSE_LAST_STEERING TO MIDCOURSE_THRUST.
            // Start the timed impulse only after the physical stage is close
            // to the requested correction vector.  MIDCOURSE_THRUST is already
            // constrained, so it is the relevant safe alignment reference.
            IF NOT MIDCOURSE_PULSE_BURN_STARTED
                AND VANG(SHIP:FACING:VECTOR, MIDCOURSE_THRUST)
                    <= MIDCOURSE_IGNITION_CONE_DEGREES
                AND SHIP:ANGULARVEL:MAG * CONSTANT:RADTODEG
                    <= MIDCOURSE_IGNITION_MAX_ANGULAR_RATE_DEG {
                SET MIDCOURSE_PULSE_BURN_STARTED TO TRUE.
                // Delta-v and height-loss budgets apply to the powered impulse,
                // not to the preceding zero-throttle attitude acquisition.
                SET MIDCOURSE_PULSE_START_HEIGHT TO BALLISTIC_HEIGHT.
                SET MIDCOURSE_PULSE_START_H_VEL TO BALLISTIC_H_VEL.
                SET MIDCOURSE_PULSE_END TO BALLISTIC_NOW
                    + MIDCOURSE_PULSE_SECONDS.
            }
            IF MIDCOURSE_PULSE_BURN_STARTED {
                SET FLIGHT_THROTTLE_CMD TO MIDCOURSE_THROTTLE.
            } ELSE {
                SET FLIGHT_THROTTLE_CMD TO 0.
                SET MIDCOURSE_THROTTLE TO 0.
            }
        }
    } ELSE {
        // Below the pre-alignment height, establish the solved cone-relative
        // thrust axis while still unpowered.  Fifth/sixth-flight telemetry
        // showed that waiting until 7 km left the physical stage near the cone
        // centre for the whole burn and stopped it vertically near 3 km.
        LOCAL PREALIGN_SAFETY_AXIS IS THRUST_SAFETY_AXIS(BALLISTIC_UP,
            TERMINAL_VELOCITY_CONE_MIN_SPEED).
        // Preserve the last correction attitude between fixed checkpoints.
        // Their desired directions change only slightly, so this removes a
        // repeated turn back to retrograde without adding a powered segment.
        IF MIDCOURSE_CHECKPOINT_INDEX > 0
            AND MIDCOURSE_CHECKPOINT_INDEX < 3
            AND MIDCOURSE_LAST_STEERING:MAG > 0.1 {
            SET PREALIGN_SAFETY_AXIS TO CONSTRAIN_THRUST_VECTOR(
                MIDCOURSE_LAST_STEERING,
                THRUST_SAFETY_AXIS(BALLISTIC_UP,
                    TERMINAL_VELOCITY_CONE_MIN_SPEED),
                TERMINAL_COMMAND_CONE_DEGREES).
        } ELSE IF BALLISTIC_HEIGHT <= TERMINAL_MAIN_PREALIGN_HEIGHT
            AND BALLISTIC_H_VEL:MAG > 0.1 {
            IF NOT BALLISTIC_TERMINAL_WHEEL_ENABLED {
                SET BALLISTIC_TERMINAL_WHEEL_ENABLED TO
                    SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
                        BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY).
            }
            SET PREALIGN_SAFETY_AXIS TO PRELEAD_THRUST_AXIS(
                BALLISTIC_UP, -BALLISTIC_H_VEL,
                MAIN_IGNITION_PRELEAD_DEGREES).
        }
        SET FLIGHT_STEERING_CMD TO LOOKDIRUP(PREALIGN_SAFETY_AXIS,
            BALLISTIC_UP).
        SET FLIGHT_THROTTLE_CMD TO 0.
    }
    // A vertical-only ignition gate was 2-3 km late in real flight: at 8.8 km
    // the measured horizontal velocity already required more stopping distance
    // than remained to the ship.  Solve the horizontal component of the same
    // 75% vector and include the measured attitude response explicitly.
    LOCAL BALLISTIC_MAIN_VERTICAL_THRUST IS EFFECTIVE_G
        + BURN_DV_VERTICAL / MAX(PLANNED_BURN_TIME, 0.1).
    LOCAL BALLISTIC_MAIN_HORIZONTAL_ACCEL IS SQRT(MAX(
        NOMINAL_BURN_ACCEL^2 - BALLISTIC_MAIN_VERTICAL_THRUST^2, 1)).
    LOCAL BALLISTIC_HORIZONTAL_STOP_DISTANCE IS
        (BALLISTIC_H_VEL:MAG
                * TERMINAL_MAIN_ATTITUDE_RESPONSE_SECONDS
            + BALLISTIC_H_VEL:MAG^2
                / (2 * BALLISTIC_MAIN_HORIZONTAL_ACCEL))
            * TERMINAL_MAIN_HORIZONTAL_STOP_SAFETY.
    LOCAL BALLISTIC_HORIZONTAL_IGNITION IS BALLISTIC_H_POS:MAG
        <= BALLISTIC_HORIZONTAL_STOP_DISTANCE
            + CLAMP(TERMINAL_WAYPOINT_APPROACH_OFFSET
                    + TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN
                        * (-BALLISTIC_VERTICAL_V
                            - TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED),
                TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET,
                TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET).
    // The burn has one vector magnitude and must satisfy both endpoint axes.
    // A horizontal boundary may request a bounded checkpoint correction, but
    // Run 91 proved it cannot directly start the non-coastable 75% main burn:
    // minimum braking then stopped the vehicle about 4.7 km short.  Checkpoints
    // now shape vertical energy first; continuous ownership begins only inside
    // the complementary vertical-ready window.
    // A checkpoint is one indivisible, bounded correction.  Run 24 reached the
    // main feasibility boundary halfway through checkpoint 3 and truncated it
    // after roughly 1.4 of the planned 3 seconds, leaving about 2 km of the
    // predicted miss uncorrected.  Finish or explicitly budget-stop the active
    // pulse before the one continuous main segment takes ownership.
    // Complete (or explicitly skip on its miss gate) all three fixed checks
    // before main ignition.  Run 28's earlier feasibility boundary fired after
    // check 2 at 24 km, leaving a 1.46 km predicted residual and spending the
    // continuous 75% floor until horizontal speed vanished above 3 km.  The
    // third bounded check both removes that residual and places ignition on
    // the intended common position/velocity trajectory.
    LOCAL BALLISTIC_VERTICAL_IGNITION IS
        BALLISTIC_HEIGHT <= BALLISTIC_IGNITION_HEIGHT.
    LOCAL BALLISTIC_VERTICAL_READY IS BALLISTIC_HEIGHT
        <= BALLISTIC_IGNITION_HEIGHT
            + TERMINAL_MAIN_VERTICAL_READY_MARGIN.
    // The Step-106 corridor owns both velocity axes from a declared height,
    // including when its three checkpoints are horizontal-only.  Run 102 left
    // this fixed handoff conditional on nonzero checkpoint vertical thrust;
    // with that thrust deliberately disabled, the old vertical solver delayed
    // gate commit from 23.6 to 16.0 km and made the corridor unreachable.
    LOCAL BALLISTIC_SHAPED_HANDOFF_READY IS BALLISTIC_HEIGHT
        <= MIDCOURSE_SHAPED_MAIN_HANDOFF_HEIGHT.
    LOCAL BALLISTIC_ENERGY_IGNITION IS BALLISTIC_VERTICAL_IGNITION
        OR BALLISTIC_HORIZONTAL_IGNITION.
    // The load cone makes the powered path one-way in downrange velocity.  A
    // positive predicted along miss means this candidate burn stops short and
    // cannot be repaired after ignition.  Run 67's predictor exposed that
    // state accurately, but the former energy-only gate ignored it.
    LOCAL BALLISTIC_ONE_WAY_IGNITION IS PREDICTED_MISS_ALONG
        <= TERMINAL_MAIN_MAX_PREDICTED_PROGRADE_MISS.
    LOCAL BALLISTIC_MAIN_GATE_READY IS
        MIDCOURSE_CHECKPOINT_INDEX >= 3
        AND NOT MIDCOURSE_PULSE_ACTIVE
        AND BALLISTIC_VERTICAL_V < 0
        AND BALLISTIC_ENERGY_IGNITION
        AND BALLISTIC_SHAPED_HANDOFF_READY
        AND BALLISTIC_ONE_WAY_IGNITION.

    IF BALLISTIC_MAIN_GATE_READY {
        LOCAL BALLISTIC_IGNITION_REASON IS "HORIZONTAL".
        IF BALLISTIC_VERTICAL_IGNITION {
            SET BALLISTIC_IGNITION_REASON TO "VERTICAL".
        }
        IF BALLISTIC_VERTICAL_IGNITION AND BALLISTIC_HORIZONTAL_IGNITION {
            SET BALLISTIC_IGNITION_REASON TO "BOTH".
        }
        LOG MISSION_ID + ",MAIN_GATE_COMMIT,"
            + ROUND(BALLISTIC_NOW,3)
            + ",height=" + ROUND(BALLISTIC_HEIGHT,2)
            + ",reason=" + BALLISTIC_IGNITION_REASON
            + ",ignitionHeight=" + ROUND(BALLISTIC_IGNITION_HEIGHT,2)
            + ",range=" + ROUND(BALLISTIC_H_POS:MAG,2)
            + ",stopDistance="
                + ROUND(BALLISTIC_HORIZONTAL_STOP_DISTANCE,2)
            + ",alongMiss=" + ROUND(PREDICTED_MISS_ALONG,2)
            TO "0:/cz10b/telemetry.csv".
    }
    SET TERMINAL_IGNITION TO BALLISTIC_MAIN_GATE_READY.

    IF BALLISTIC_NOW - LAST_LOG >= TELEMETRY_PERIOD {
        LOCAL BALLISTIC_PHASE IS "BALLISTIC".
        IF MIDCOURSE_ACTIVE { SET BALLISTIC_PHASE TO "MIDCOURSE". }
        WRITE_TELEMETRY(BALLISTIC_PHASE, MISSION_ID, SHIP:ALTITUDE,
            BALLISTIC_HEIGHT, BALLISTIC_VERTICAL_V,
            BALLISTIC_H_VEL:MAG, PREDICTED_MISS:MAG,
            MIDCOURSE_THROTTLE, MIDCOURSE_TILT).
        LOG MISSION_ID + ",IGNITION_GATE,"
            + ROUND(BALLISTIC_NOW,3)
            + ",height=" + ROUND(BALLISTIC_HEIGHT,2)
            + ",verticalGate=" + BALLISTIC_VERTICAL_IGNITION
            + ",verticalReady=" + BALLISTIC_VERTICAL_READY
            + ",shapedHandoffReady="
                + BALLISTIC_SHAPED_HANDOFF_READY
            + ",shapedHandoffHeight="
                + ROUND(MIDCOURSE_SHAPED_MAIN_HANDOFF_HEIGHT,2)
            + ",horizontalGate=" + BALLISTIC_HORIZONTAL_IGNITION
            + ",oneWayGate=" + BALLISTIC_ONE_WAY_IGNITION
            + ",ignitionHeight=" + ROUND(BALLISTIC_IGNITION_HEIGHT,2)
            + ",range=" + ROUND(BALLISTIC_H_POS:MAG,2)
            + ",stopDistance="
                + ROUND(BALLISTIC_HORIZONTAL_STOP_DISTANCE,2)
            + ",alongMiss=" + ROUND(PREDICTED_MISS_ALONG,2)
            + ",burnTime=" + ROUND(PLANNED_BURN_TIME,2)
            + ",horizontalTarget="
                + ROUND(MIDCOURSE_HORIZONTAL_TARGET_SPEED,2)
            + ",horizontalExcess="
                + ROUND(MIDCOURSE_HORIZONTAL_SPEED_EXCESS,2)
            + ",horizontalShaping="
                + MIDCOURSE_HORIZONTAL_SHAPING_REQUESTED
            + ",horizontalHoldAccel="
                + ROUND(MIDCOURSE_TARGET_ALONG_ACCEL,3)
            TO "0:/cz10b/telemetry.csv".
        SET LAST_LOG TO BALLISTIC_NOW.
    }
    PRINT "ballistic h  " + ROUND(BALLISTIC_HEIGHT,0) + " m    " AT(0,12).
    PRINT "ignition gate" + ROUND(BALLISTIC_IGNITION_HEIGHT,0)
        + " m    " AT(0,13).
    PRINT "impact miss  " + ROUND(PREDICTED_MISS:MAG,0) + " m    " AT(0,14).
    SET BALLISTIC_PREVIOUS_TIME TO BALLISTIC_NOW.
    SET BALLISTIC_PREVIOUS_V TO SHIP:VELOCITY:SURFACE.
    SET BALLISTIC_WAS_POWERED TO MIDCOURSE_ACTIVE.
    WAIT 0.02.
}

// Step-106 joint corridor.  References are deliberately height indexed so the
// trajectory cannot extend its own deadline.  Their slopes convert the same
// path into physically dimensioned feed-forward acceleration.
FUNCTION HYBRID_CORRIDOR_LINEAR {
    PARAMETER HEIGHT_VALUE, HEIGHT_HIGH, VALUE_HIGH,
              HEIGHT_LOW, VALUE_LOW.
    LOCAL BLEND IS CLAMP((HEIGHT_HIGH - HEIGHT_VALUE)
        / MAX(HEIGHT_HIGH - HEIGHT_LOW, 1), 0, 1).
    RETURN VALUE_HIGH + (VALUE_LOW - VALUE_HIGH) * BLEND.
}

FUNCTION HYBRID_CORRIDOR_HORIZONTAL_REFERENCE {
    PARAMETER HEIGHT_VALUE.
    IF HEIGHT_VALUE >= 23572 { RETURN 821.3. }
    IF HEIGHT_VALUE >= 20572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 23572, 821.3, 20572, 753.5). }
    IF HEIGHT_VALUE >= 18572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 20572, 753.5, 18572, 691.5). }
    IF HEIGHT_VALUE >= 16572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 18572, 691.5, 16572, 614.5). }
    IF HEIGHT_VALUE >= 14572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 16572, 614.5, 14572, 530.0). }
    IF HEIGHT_VALUE >= 12572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 14572, 530.0, 12572, 425.0). }
    IF HEIGHT_VALUE >= 10572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 12572, 425.0, 10572, 315.0). }
    IF HEIGHT_VALUE >= 8572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 10572, 315.0, 8572, 230.0). }
    IF HEIGHT_VALUE >= 6572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 8572, 230.0, 6572, 160.0). }
    RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 6572, 160.0, 6000, 131.0).
}

FUNCTION HYBRID_CORRIDOR_DOWN_REFERENCE {
    PARAMETER HEIGHT_VALUE.
    IF HEIGHT_VALUE >= 23572 { RETURN 635.3. }
    IF HEIGHT_VALUE >= 20572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 23572, 635.3, 20572, 625.5). }
    IF HEIGHT_VALUE >= 18572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 20572, 625.5, 18572, 608.0). }
    IF HEIGHT_VALUE >= 16572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 18572, 608.0, 16572, 590.5). }
    IF HEIGHT_VALUE >= 14572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 16572, 590.5, 14572, 559.5). }
    IF HEIGHT_VALUE >= 12572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 14572, 559.5, 12572, 519.0). }
    IF HEIGHT_VALUE >= 10572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 12572, 519.0, 10572, 465.0). }
    IF HEIGHT_VALUE >= 8572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 10572, 465.0, 8572, 435.0). }
    IF HEIGHT_VALUE >= 6572 { RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 8572, 435.0, 6572, 400.0). }
    RETURN HYBRID_CORRIDOR_LINEAR(
        HEIGHT_VALUE, 6572, 400.0, 6000, 381.0).
}

FUNCTION HYBRID_CORRIDOR_HORIZONTAL_SLOPE {
    PARAMETER HEIGHT_VALUE.
    IF HEIGHT_VALUE >= 20572 { RETURN (821.3 - 753.5) / 3000. }
    IF HEIGHT_VALUE >= 18572 { RETURN (753.5 - 691.5) / 2000. }
    IF HEIGHT_VALUE >= 16572 { RETURN (691.5 - 614.5) / 2000. }
    IF HEIGHT_VALUE >= 14572 { RETURN (614.5 - 530.0) / 2000. }
    IF HEIGHT_VALUE >= 12572 { RETURN (530.0 - 425.0) / 2000. }
    IF HEIGHT_VALUE >= 10572 { RETURN (425.0 - 315.0) / 2000. }
    IF HEIGHT_VALUE >= 8572 { RETURN (315.0 - 230.0) / 2000. }
    IF HEIGHT_VALUE >= 6572 { RETURN (230.0 - 160.0) / 2000. }
    RETURN (160.0 - 131.0) / 572.
}

FUNCTION HYBRID_CORRIDOR_DOWN_SLOPE {
    PARAMETER HEIGHT_VALUE.
    IF HEIGHT_VALUE >= 20572 { RETURN (635.3 - 625.5) / 3000. }
    IF HEIGHT_VALUE >= 18572 { RETURN (625.5 - 608.0) / 2000. }
    IF HEIGHT_VALUE >= 16572 { RETURN (608.0 - 590.5) / 2000. }
    IF HEIGHT_VALUE >= 14572 { RETURN (590.5 - 559.5) / 2000. }
    IF HEIGHT_VALUE >= 12572 { RETURN (559.5 - 519.0) / 2000. }
    IF HEIGHT_VALUE >= 10572 { RETURN (519.0 - 465.0) / 2000. }
    IF HEIGHT_VALUE >= 8572 { RETURN (465.0 - 435.0) / 2000. }
    IF HEIGHT_VALUE >= 6572 { RETURN (435.0 - 400.0) / 2000. }
    RETURN (400.0 - 381.0) / 572.
}

// Prefer to light after reaching the planned cone-edge axis.  A bounded fallback
// still permits ignition from the physical retrograde-safe axis if high-q
// restoring torque prevents exact edge tracking; it can never waive the actual
// hard cone, which the flight observer audits on every powered frame.
SET FLIGHT_THROTTLE_CMD TO 0.
LOCAL MAIN_IGNITION_SAFETY_AXIS IS THRUST_SAFETY_AXIS(
    SHIP:UP:VECTOR:NORMALIZED, TERMINAL_VELOCITY_CONE_MIN_SPEED).
LOCAL MAIN_IGNITION_HARD_AXIS IS MAIN_IGNITION_SAFETY_AXIS.
LOCAL MAIN_IGNITION_H_BRAKE IS -VXCL(SHIP:UP:VECTOR:NORMALIZED,
    SHIP:VELOCITY:SURFACE).
IF MAIN_IGNITION_H_BRAKE:MAG > 0.1 {
    SET MAIN_IGNITION_SAFETY_AXIS TO PRELEAD_THRUST_AXIS(
        SHIP:UP:VECTOR:NORMALIZED, MAIN_IGNITION_H_BRAKE,
        MAIN_IGNITION_PRELEAD_DEGREES).
}
LOCAL MAIN_IGNITION_STABLE_SINCE IS -1.
LOCAL MAIN_IGNITION_ALIGNMENT_LAST_LOG IS TIME:SECONDS - TELEMETRY_PERIOD.
// BALLISTIC_IGNITION_HEIGHT is local to the planning loop in kOS.  Snapshot
// the actual gate crossing here so the bounded fallback never references a
// variable that has gone out of scope.
LOCAL MAIN_IGNITION_FALLBACK_HEIGHT IS SHIP:ALTITUDE
    - MAIN_IGNITION_ALIGNMENT_FALLBACK_DROP.
UNTIL MAIN_IGNITION_STABLE_SINCE >= 0
    AND TIME:SECONDS - MAIN_IGNITION_STABLE_SINCE
        >= MAIN_IGNITION_STABLE_DWELL_SECONDS {
    SET MAIN_IGNITION_H_BRAKE TO -VXCL(SHIP:UP:VECTOR:NORMALIZED,
        SHIP:VELOCITY:SURFACE).
    SET MAIN_IGNITION_SAFETY_AXIS TO THRUST_SAFETY_AXIS(
        SHIP:UP:VECTOR:NORMALIZED, TERMINAL_VELOCITY_CONE_MIN_SPEED).
    SET MAIN_IGNITION_HARD_AXIS TO MAIN_IGNITION_SAFETY_AXIS.
    IF MAIN_IGNITION_H_BRAKE:MAG > 0.1 {
        SET MAIN_IGNITION_SAFETY_AXIS TO PRELEAD_THRUST_AXIS(
            SHIP:UP:VECTOR:NORMALIZED, MAIN_IGNITION_H_BRAKE,
            MAIN_IGNITION_PRELEAD_DEGREES).
    }
    SET FLIGHT_STEERING_CMD TO LOOKDIRUP(MAIN_IGNITION_SAFETY_AXIS,
        SHIP:UP:VECTOR).
    LOCAL MAIN_IGNITION_ANGLE IS VANG(SHIP:FACING:VECTOR,
        MAIN_IGNITION_SAFETY_AXIS).
    LOCAL MAIN_IGNITION_HARD_ANGLE IS VANG(SHIP:FACING:VECTOR,
        MAIN_IGNITION_HARD_AXIS).
    LOCAL MAIN_IGNITION_ANGULAR_RATE_DEG IS SHIP:ANGULARVEL:MAG
        * CONSTANT:RADTODEG.
    LOCAL MAIN_IGNITION_EDGE_READY IS
        MAIN_IGNITION_ANGLE <= MAIN_IGNITION_ALIGNMENT_DEGREES
        AND MAIN_IGNITION_ANGULAR_RATE_DEG
            <= MAIN_IGNITION_MAX_ANGULAR_RATE_DEG.
    LOCAL MAIN_IGNITION_FALLBACK_READY IS
        SHIP:ALTITUDE <= MAIN_IGNITION_FALLBACK_HEIGHT
        AND MAIN_IGNITION_HARD_ANGLE <= TERMINAL_COMMAND_CONE_DEGREES
        AND MAIN_IGNITION_ANGULAR_RATE_DEG
            <= MAIN_IGNITION_FALLBACK_MAX_ANGULAR_RATE_DEG.
    IF MAIN_IGNITION_EDGE_READY OR MAIN_IGNITION_FALLBACK_READY {
        IF MAIN_IGNITION_STABLE_SINCE < 0 {
            SET MAIN_IGNITION_STABLE_SINCE TO TIME:SECONDS.
        }
    } ELSE {
        SET MAIN_IGNITION_STABLE_SINCE TO -1.
    }
    IF TIME:SECONDS - MAIN_IGNITION_ALIGNMENT_LAST_LOG >= TELEMETRY_PERIOD {
        LOCAL MAIN_IGNITION_HORIZONTAL_SPEED IS VXCL(
            SHIP:UP:VECTOR:NORMALIZED, SHIP:VELOCITY:SURFACE):MAG.
        WRITE_TELEMETRY("IGNITION_ALIGN", MISSION_ID, SHIP:ALTITUDE,
            SHIP:ALTITUDE, SHIP:VERTICALSPEED,
            MAIN_IGNITION_HORIZONTAL_SPEED,
            MAIN_IGNITION_ANGLE, 0, MAIN_IGNITION_ANGULAR_RATE_DEG).
        SET MAIN_IGNITION_ALIGNMENT_LAST_LOG TO TIME:SECONDS.
    }
    WAIT 0.02.
}

// Build one constrained, height-indexed trajectory from the solved ignition
// state.  The 75% main burn follows it to the 2 km waypoint; only the final
// frame entrance uses local damping.
PRINT "75% MAIN TRAJECTORY BURN" AT(0,11).
LOCAL VERTICAL_INTEGRAL IS 0.
LOCAL PREVIOUS_TIME IS TIME:SECONDS.
LOCAL PLAN_UP IS SHIP:UP:VECTOR:NORMALIZED.
LOCAL PLAN_REL_POS IS NET_POSITION(RECOVERY_SHIP) - SHIP:POSITION.
LOCAL PLAN_HOOK_HEIGHT IS -VDOT(PLAN_REL_POS, PLAN_UP)
    + BOOSTER_HOOK_OFFSET_ALONG_UP(PLAN_UP).
LOCAL PLAN_REL_VEL IS SHIP:VELOCITY:SURFACE.
LOCAL PLAN_VERTICAL_V IS VDOT(PLAN_REL_VEL, PLAN_UP).
LOCAL PLAN_H_POS IS VXCL(PLAN_UP, PLAN_REL_POS).
LOCAL PLAN_H_VEL IS VXCL(PLAN_UP, PLAN_REL_VEL).
LOCAL PLAN_APPROACH_DISTANCE IS CLAMP(
    TERMINAL_WAYPOINT_APPROACH_OFFSET
        + TERMINAL_WAYPOINT_APPROACH_VERTICAL_GAIN
            * (-PLAN_VERTICAL_V
                - TERMINAL_WAYPOINT_APPROACH_VERTICAL_REFERENCE_SPEED),
    TERMINAL_WAYPOINT_APPROACH_MIN_OFFSET,
    TERMINAL_WAYPOINT_APPROACH_MAX_OFFSET).
LOCAL PLAN_ALONG_DIRECTION IS V(0,0,0).
IF PLAN_H_VEL:MAG > 0.1 {
    SET PLAN_ALONG_DIRECTION TO PLAN_H_VEL:NORMALIZED.
} ELSE IF PLAN_H_POS:MAG > 0.1 {
    SET PLAN_ALONG_DIRECTION TO PLAN_H_POS:NORMALIZED.
}
LOCAL PLAN_APPROACH_OFFSET IS V(0,0,0).
IF PLAN_H_VEL:MAG > 0.1 {
    SET PLAN_APPROACH_OFFSET TO PLAN_H_VEL:NORMALIZED
        * PLAN_APPROACH_DISTANCE.
} ELSE IF PLAN_H_POS:MAG > 0.1 {
    SET PLAN_APPROACH_OFFSET TO PLAN_H_POS:NORMALIZED
        * PLAN_APPROACH_DISTANCE.
}
LOCAL PLAN_CONTROL_H_POS IS PLAN_H_POS - PLAN_APPROACH_OFFSET.
LOCAL PLAN_HORIZONTAL_END_HEIGHT IS MIN(
    TERMINAL_HORIZONTAL_PLAN_END_HEIGHT,
    MAX(TERMINAL_WAYPOINT_HEIGHT, PLAN_HOOK_HEIGHT - 500)).
LOCAL PLAN_PROGRESS_RATE0 IS MAX(-PLAN_VERTICAL_V,
    CAPTURE_FINAL_SPEED) / MAX(PLAN_HOOK_HEIGHT
        - PLAN_HORIZONTAL_END_HEIGHT, 1).
// Error derivative with respect to normalized height progress.  Matching this
// tangent makes the first guidance command continuous with the incoming flight.
LOCAL PLAN_H_ERROR_SLOPE0 IS -PLAN_H_VEL
    / MAX(PLAN_PROGRESS_RATE0, 0.0001).
LOCAL FUEL_URGENT IS FALSE.
LOCAL WAS_PID_MODE IS FALSE.
LOCAL CAPTURE_ALIGN_MODE IS FALSE.
LOCAL HIGH_ENERGY_BRAKE_MODE IS FALSE.
LOCAL WAYPOINT_COAST_MODE IS FALSE.
LOCAL WAYPOINT_CENTER_BRAKE_MODE IS FALSE.
LOCAL WAYPOINT_CENTER_BRAKE_DIRECTION IS V(0,0,0).
LOCAL WAYPOINT_ENDPOINT_TRIM_ACTIVE IS FALSE.
LOCAL WAYPOINT_ENDPOINT_TRIM_DIRECTION IS V(0,0,0).
LOCAL WAYPOINT_ENDPOINT_TRIM_TARGET_PROJECTION IS 0.
LOCAL WAYPOINT_ENDPOINT_TRIM_COUNT IS 0.
LOCAL WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE IS FALSE.
LOCAL WAYPOINT_POST_UPRIGHT_TRIM_DONE IS FALSE.
LOCAL WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION IS V(0,0,0).
LOCAL WAYPOINT_POST_UPRIGHT_TARGET_PROJECTION IS 0.
LOCAL WAYPOINT_FINAL_COAST_MODE IS FALSE.
    // Retain the bounded high-q terminal authority selected after entry cutoff.
    // The 2 km near-field handoff below restores 20/20/10 kN-m before capture.
    LOCAL TERMINAL_WHEEL_AUTHORITY_ENABLED IS
        SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
            BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY).
LOCAL CAPTURE_ALIGN_SPEED_LIMIT IS TERMINAL_ALIGN_SPEED.
LOCAL HORIZONTAL_SETTLE_MODE IS FALSE.
LOCAL WIRE_HOLD_STARTED_AT IS -1.
LOCAL FINAL_ALIGN_MODE IS FALSE.
LOCAL FINAL_ALIGN_STARTED_AT IS -1.
LOCAL FINAL_ALIGN_ENTRY_RATIO IS -1.
LOCAL FINAL_ALIGN_ACTIVE_POSITION_GAIN IS FINAL_ALIGN_POSITION_GAIN.
LOCAL TERMINAL_EARLY_AERO_BRAKE_LATCHED IS FALSE.
LOCAL TERMINAL_EARLY_AERO_BRAKE_ENTRY_RATIO IS -1.
LOCAL TERMINAL_EARLY_AERO_BRAKE_FLOOR IS 0.
LOCAL FINAL_ALIGN_TERMINAL_PHASE_LATCHED IS FALSE.
LOCAL FINAL_ALIGN_TERMINAL_ENTRY_RATIO IS -1.
LOCAL FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE IS 1.
LOCAL FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT IS
    TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_END_HEIGHT.
LOCAL FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCHED IS FALSE.
LOCAL FINAL_ALIGN_TERMINAL_LOW_RANGE_AT_LATCH IS -1.
LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCHED IS FALSE.
LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTED_RANGE_AT_LATCH IS -1.
LOCAL FINAL_DESCENT_ARMED IS FALSE.
LOCAL FINAL_ALIGN_PRECOMMIT_SETTLE IS FALSE.
LOCAL FINAL_CAPTURE_DIRECT_AERO_RESET IS FALSE.
LOCAL FINAL_CAPTURE_NEAR_NET_STEERING_TUNED IS FALSE.
LOCAL TERMINAL_STEERING_TUNED IS FALSE.
LOCAL FILTERED_H_ACCEL IS V(0,0,0).
LOCAL LIVE_APPROACH_OFFSET_FINAL_BLEND_STATE IS
    TERMINAL_LIVE_APPROACH_OFFSET_FINAL_BLEND.
LOCAL LIVE_APPROACH_OFFSET_RANGE_LATCHED IS FALSE.
LOCAL LIVE_APPROACH_OFFSET_RANGE_AT_LATCH IS -1.
// Persistent one-way latch for the reachable fixed-axis stopping boundary.
// A per-frame feasibility switch can chatter as drag and the velocity cone
// move; once a genuinely reachable stop is committed, retain it until the
// positive range/velocity gates naturally end the pass.
LOCAL FIXED_STOP_COMMITTED IS FALSE.
// Cross-track stopping uses its own frozen surface direction.  Keeping this
// separate from the downrange latch lets the cone-edge allocator share its
// azimuth between the two independently measured endpoint requirements.
LOCAL CROSS_STOP_COMMITTED IS FALSE.
LOCAL CROSS_STOP_DIRECTION IS V(0,0,0).
// Completion is a separate permanent state.  Recomputing suppression from the
// live velocity sign allowed ordinary cross control to make velocity positive
// again and silently reactivate the same stop in Run 84.
LOCAL CROSS_STOP_COMPLETED IS FALSE.
// The reversed-attitude aerodynamic actuator has a several-second force tail.
// Build its command monotonically until stopping demand peaks, then release it
// monotonically from the residual demand left after measured physical braking.
LOCAL CROSS_AERO_BRAKE_PREVIOUS_DEMAND IS 0.
LOCAL CROSS_AERO_BRAKE_BLEND_STATE IS 0.
LOCAL CROSS_AERO_BRAKE_RELEASE_STARTED IS FALSE.
// Continuous state for the measured downrange reachability allocator.  The
// legal cone-edge attitude has a multi-second aerodynamic response, so a raw
// per-frame pressure switch would merely turn force lag into direction PWM.
LOCAL ALONG_AERO_BRAKE_BLEND_STATE IS 0.
LOCAL ALONG_AERO_BRAKE_LOW_SPEED_SETTLE_ACTIVE IS FALSE.
LOCAL POWERED_PREVIOUS_V IS SHIP:VELOCITY:SURFACE.
// Carry the already identified coast aerodynamics across ignition. Starting
// these filters at zero made the first seconds of run 20 command excessive
// vertical thrust, placing the mandatory 75% floor mostly upward while
// horizontal stopping distance was already critical.
LOCAL POWERED_FILTERED_VERTICAL_AERO_ACCEL IS FILTERED_VERTICAL_DRAG.
LOCAL POWERED_FILTERED_HORIZONTAL_AERO_ACCEL IS FILTERED_HORIZONTAL_DRAG.
LOCAL TERMINAL_WHEEL_AUTHORITY_COMMAND IS
    BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY.
LOCAL GRID_FIN_DEPLOYMENT_COMMAND IS 0.
LOCAL GRID_FIN_LAST_SENT_COMMAND IS -1.
LOCAL GRID_FIN_APPLIED_DEGREES IS BOOSTER_GRID_FIN_APPLIED_DEPLOYMENT().
LOCAL GRID_FIN_INITIALIZED_COUNT IS
    SET_BOOSTER_GRID_FIN_DEPLOYMENT(0).
SET LAST_LOG TO TIME:SECONDS.

UNTIL HOOK_CAPTURED() {
    LOCAL NOW IS TIME:SECONDS.
    LOCAL POWERED_MEASUREMENT_DT IS MAX(NOW - PREVIOUS_TIME, 0.001).
    // Preserve the bounded integration step used by the historically tuned
    // near-field controllers, but do not use that artificial bound to assign
    // physical units to the acceleration observer below.
    LOCAL DT IS CLAMP(POWERED_MEASUREMENT_DT, 0.001, 0.1).
    SET PREVIOUS_TIME TO NOW.

    // Keep the checkpoint and upper main-burn actuator unchanged, then shed
    // only the excess low-altitude vertical authority identified in Run 107.
    // The height-indexed schedule is continuous and does not modulate throttle.
    LOCAL RETURN_ENGINE_LOW_ALT_BLEND IS CLAMP(
        (RETURN_ENGINE_LOW_ALT_RAMP_START_HEIGHT - SHIP:ALTITUDE)
        / MAX(RETURN_ENGINE_LOW_ALT_RAMP_START_HEIGHT
            - RETURN_ENGINE_LOW_ALT_RAMP_FULL_HEIGHT, 1), 0, 1).
    LOCAL RETURN_ENGINE_CURRENT_MAX_ACCEL IS RETURN_ENGINE_MAX_ACCEL
        + (RETURN_ENGINE_LOW_ALT_MAX_ACCEL - RETURN_ENGINE_MAX_ACCEL)
        * RETURN_ENGINE_LOW_ALT_BLEND.
    LOCAL RETURN_ENGINE_POST_WAYPOINT_BLEND IS CLAMP(
        (TERMINAL_WAYPOINT_HEIGHT - SHIP:ALTITUDE)
        / MAX(TERMINAL_WAYPOINT_HEIGHT
            - RETURN_ENGINE_POST_WAYPOINT_RAMP_FULL_HEIGHT, 1), 0, 1).
    SET RETURN_ENGINE_CURRENT_MAX_ACCEL TO
        RETURN_ENGINE_CURRENT_MAX_ACCEL
        + (RETURN_ENGINE_POST_WAYPOINT_MAX_ACCEL
            - RETURN_ENGINE_CURRENT_MAX_ACCEL)
            * RETURN_ENGINE_POST_WAYPOINT_BLEND.
    IF NOW - RETURN_ACCEL_LAST_UPDATE
            >= RETURN_ENGINE_ACCEL_UPDATE_SECONDS {
        SET RETURN_ACCEL_NORMALIZED TO NORMALIZE_BOOSTER_ENGINE_ACCEL(
            BOOSTER_ENGINE_PART_NAME, RETURN_ENGINE_CURRENT_MAX_ACCEL,
            RETURN_ENGINE_MIN_THRUST_LIMIT, RETURN_ENGINE_THRUST_LIMIT).
        SET RETURN_ACCEL_LAST_UPDATE TO NOW.
    }

    LOCAL UP_VEC IS SHIP:UP:VECTOR:NORMALIZED.
    LOCAL REL_POS IS NET_POSITION(RECOVERY_SHIP) - SHIP:POSITION.
    LOCAL HEIGHT IS -VDOT(REL_POS, UP_VEC).
    LOCAL HOOK_HEIGHT IS HEIGHT + BOOSTER_HOOK_OFFSET_ALONG_UP(UP_VEC).
    LOCAL HORIZONTAL_POS IS VXCL(UP_VEC, REL_POS).
    // Parallel-transport the frozen ignition heading into the live tangent
    // plane before using the 4 km signed-range calibration below.
    LOCAL CONTROL_ALONG_DIRECTION IS VXCL(UP_VEC, PLAN_ALONG_DIRECTION).
    IF CONTROL_ALONG_DIRECTION:MAG > 0.0001 {
        SET CONTROL_ALONG_DIRECTION TO CONTROL_ALONG_DIRECTION:NORMALIZED.
    } ELSE {
        SET CONTROL_ALONG_DIRECTION TO PLAN_ALONG_DIRECTION.
    }
    IF NOT LIVE_APPROACH_OFFSET_RANGE_LATCHED
        AND HOOK_HEIGHT
            <= TERMINAL_LIVE_APPROACH_OFFSET_FADE_START_HEIGHT {
        SET LIVE_APPROACH_OFFSET_RANGE_AT_LATCH TO VDOT(
            HORIZONTAL_POS, CONTROL_ALONG_DIRECTION).
        SET LIVE_APPROACH_OFFSET_FINAL_BLEND_STATE TO CLAMP(
            TERMINAL_LIVE_APPROACH_OFFSET_FINAL_BLEND
            + (TERMINAL_LIVE_APPROACH_OFFSET_FADE_REFERENCE_RANGE
                - LIVE_APPROACH_OFFSET_RANGE_AT_LATCH)
                * TERMINAL_LIVE_APPROACH_OFFSET_FADE_RANGE_GAIN
                / MAX(PLAN_APPROACH_DISTANCE, 1),
            TERMINAL_LIVE_APPROACH_OFFSET_FINAL_MIN_BLEND,
            TERMINAL_LIVE_APPROACH_OFFSET_FINAL_MAX_BLEND).
        SET LIVE_APPROACH_OFFSET_RANGE_LATCHED TO TRUE.
    }
    LOCAL LIVE_APPROACH_OFFSET_BASE_BLEND IS CLAMP(
        (HOOK_HEIGHT - TERMINAL_LIVE_APPROACH_OFFSET_FADE_END_HEIGHT)
        / MAX(TERMINAL_LIVE_APPROACH_OFFSET_FADE_START_HEIGHT
            - TERMINAL_LIVE_APPROACH_OFFSET_FADE_END_HEIGHT, 1), 0, 1).
    LOCAL LIVE_APPROACH_OFFSET_POST_BLEND IS CLAMP(
        (HOOK_HEIGHT
            - TERMINAL_LIVE_APPROACH_OFFSET_POST_FADE_END_HEIGHT)
        / MAX(TERMINAL_WAYPOINT_HEIGHT
            - TERMINAL_LIVE_APPROACH_OFFSET_POST_FADE_END_HEIGHT, 1), 0, 1).
    LOCAL LIVE_APPROACH_OFFSET_BLEND IS
        LIVE_APPROACH_OFFSET_BASE_BLEND
        + (1 - LIVE_APPROACH_OFFSET_BASE_BLEND)
            * LIVE_APPROACH_OFFSET_FINAL_BLEND_STATE
            * LIVE_APPROACH_OFFSET_POST_BLEND.
    LOCAL CONTROL_APPROACH_OFFSET IS VXCL(UP_VEC, PLAN_APPROACH_OFFSET)
        * LIVE_APPROACH_OFFSET_BLEND.
    LOCAL WAYPOINT_CONTROL_H_POS IS HORIZONTAL_POS
        - CONTROL_APPROACH_OFFSET.
    LOCAL TERMINAL_CONTROL_H_POS IS HORIZONTAL_POS.
    IF HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET TERMINAL_CONTROL_H_POS TO WAYPOINT_CONTROL_H_POS.
    }
    LOCAL REL_VEL IS SHIP:VELOCITY:SURFACE.
    LOCAL VERTICAL_V IS VDOT(REL_VEL, UP_VEC).
    LOCAL HORIZONTAL_VEL IS VXCL(UP_VEC, REL_VEL).
    LOCAL G_ACC IS SHIP:BODY:MU / ((SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2).
    LOCAL AVAILABLE_ACC IS SHIP:AVAILABLETHRUST / MAX(SHIP:MASS, 0.001).

    LOCAL HYBRID_CORRIDOR_ACTIVE IS HOOK_HEIGHT
        > HYBRID_CORRIDOR_END_HEIGHT.
    LOCAL HYBRID_HORIZONTAL_REFERENCE IS
        HYBRID_CORRIDOR_HORIZONTAL_REFERENCE(HOOK_HEIGHT).
    LOCAL HYBRID_DOWN_REFERENCE IS
        HYBRID_CORRIDOR_DOWN_REFERENCE(HOOK_HEIGHT).
    LOCAL HYBRID_HORIZONTAL_RESIDUAL IS HORIZONTAL_VEL:MAG
        - HYBRID_HORIZONTAL_REFERENCE.
    LOCAL HYBRID_DOWN_RESIDUAL IS -VERTICAL_V
        - HYBRID_DOWN_REFERENCE.
    LOCAL GRID_FIN_OPEN_BLEND IS CLAMP(
        (GRID_FIN_AERO_OPEN_START_HEIGHT - HOOK_HEIGHT)
        / MAX(GRID_FIN_AERO_OPEN_START_HEIGHT
            - GRID_FIN_AERO_FULL_HEIGHT, 1), 0, 1).
    LOCAL GRID_FIN_STOW_BLEND IS CLAMP(
        (HOOK_HEIGHT - GRID_FIN_AERO_STOW_END_HEIGHT)
        / MAX(GRID_FIN_AERO_STOW_START_HEIGHT
            - GRID_FIN_AERO_STOW_END_HEIGHT, 1), 0, 1).
    LOCAL GRID_FIN_NOMINAL_COMMAND IS 100
        * MIN(GRID_FIN_OPEN_BLEND, GRID_FIN_STOW_BLEND).
    LOCAL GRID_FIN_HORIZONTAL_CORRECTION IS CLAMP(
        HYBRID_HORIZONTAL_RESIDUAL
            * GRID_FIN_AERO_HORIZONTAL_ERROR_GAIN,
        -GRID_FIN_AERO_HORIZONTAL_ERROR_LIMIT_PERCENT,
        GRID_FIN_AERO_HORIZONTAL_ERROR_LIMIT_PERCENT).
    LOCAL GRID_FIN_DOWN_CORRECTION IS 0.
    IF HOOK_HEIGHT > 11000 {
        SET GRID_FIN_DOWN_CORRECTION TO CLAMP(
            -HYBRID_DOWN_RESIDUAL * GRID_FIN_AERO_DOWN_ERROR_GAIN,
            -GRID_FIN_AERO_DOWN_ERROR_LIMIT_PERCENT,
            GRID_FIN_AERO_DOWN_ERROR_LIMIT_PERCENT).
    }
    LOCAL GRID_FIN_DESIRED_COMMAND IS CLAMP(
        GRID_FIN_NOMINAL_COMMAND + GRID_FIN_HORIZONTAL_CORRECTION
            + GRID_FIN_DOWN_CORRECTION, 0,
        GRID_FIN_AERO_MAX_DEPLOYMENT_PERCENT).
    IF NOT HYBRID_CORRIDOR_ACTIVE
        OR HOOK_HEIGHT <= GRID_FIN_AERO_STOW_END_HEIGHT {
        SET GRID_FIN_DESIRED_COMMAND TO 0.
    }
    LOCAL GRID_FIN_COMMAND_STEP IS
        GRID_FIN_AERO_COMMAND_RATE_PERCENT_PER_SECOND
            * POWERED_MEASUREMENT_DT.
    IF GRID_FIN_DEPLOYMENT_COMMAND < GRID_FIN_DESIRED_COMMAND {
        SET GRID_FIN_DEPLOYMENT_COMMAND TO MIN(
            GRID_FIN_DEPLOYMENT_COMMAND + GRID_FIN_COMMAND_STEP,
            GRID_FIN_DESIRED_COMMAND).
    } ELSE {
        SET GRID_FIN_DEPLOYMENT_COMMAND TO MAX(
            GRID_FIN_DEPLOYMENT_COMMAND - GRID_FIN_COMMAND_STEP,
            GRID_FIN_DESIRED_COMMAND).
    }
    IF ABS(GRID_FIN_DEPLOYMENT_COMMAND
            - GRID_FIN_LAST_SENT_COMMAND) >= 0.25 {
        LOCAL GRID_FIN_DEPLOYMENT_COUNT IS
            SET_BOOSTER_GRID_FIN_DEPLOYMENT(
                GRID_FIN_DEPLOYMENT_COMMAND).
        SET GRID_FIN_LAST_SENT_COMMAND TO
            GRID_FIN_DEPLOYMENT_COMMAND.
    }

    // Use extra attitude authority only while dynamic pressure is high enough
    // to require it. Restore the proven 10% setting before the low-altitude
    // regime where run 51's stronger wheel overshot the physical load cone.
    IF HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL TERMINAL_WHEEL_TAPER IS CLAMP(
            (HOOK_HEIGHT
                - BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_END_HEIGHT)
            / MAX(BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_START_HEIGHT
                - BOOSTER_TERMINAL_REACTION_WHEEL_TAPER_END_HEIGHT, 1),
            0, 1).
        LOCAL TERMINAL_WHEEL_SCHEDULED_AUTHORITY IS
            BOOSTER_TERMINAL_REACTION_WHEEL_RESTORE_AUTHORITY
            + (BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY
                - BOOSTER_TERMINAL_REACTION_WHEEL_RESTORE_AUTHORITY)
                * TERMINAL_WHEEL_TAPER.
        IF ABS(TERMINAL_WHEEL_SCHEDULED_AUTHORITY
                - TERMINAL_WHEEL_AUTHORITY_COMMAND) >= 0.5 {
            LOCAL TERMINAL_WHEEL_SCHEDULE_RESULT IS
                SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
                    TERMINAL_WHEEL_SCHEDULED_AUTHORITY).
            SET TERMINAL_WHEEL_AUTHORITY_COMMAND TO
                TERMINAL_WHEEL_SCHEDULED_AUTHORITY.
            LOG MISSION_ID + ",WHEEL_SCHEDULE,"
                + ROUND(TIME:SECONDS,3)
                + ",height=" + ROUND(HOOK_HEIGHT,2)
                + ",authority="
                + ROUND(TERMINAL_WHEEL_AUTHORITY_COMMAND,2)
                TO "0:/cz10b/telemetry.csv".
        }
    }

    // Identify the force that the point-mass trajectory does not command.
    // Vessel:THRUST is the live engine thrust (not the full-throttle limit),
    // so measured acceleration + gravity - thrust leaves the aerodynamic
    // contribution in the current world-vector frame.  Filtering prevents
    // one physics-frame steering transient from becoming a guidance reversal.
    LOCAL POWERED_MEASURED_ACCEL IS
        (REL_VEL - POWERED_PREVIOUS_V) / POWERED_MEASUREMENT_DT.
    LOCAL POWERED_THRUST_ACCEL IS SHIP:FACING:VECTOR
        * (SHIP:THRUST / MAX(SHIP:MASS, 0.001)).
    LOCAL POWERED_AERO_SAMPLE IS POWERED_MEASURED_ACCEL
        + UP_VEC * G_ACC - POWERED_THRUST_ACCEL.
    LOCAL POWERED_VERTICAL_AERO_SAMPLE IS CLAMP(
        VDOT(POWERED_AERO_SAMPLE, UP_VEC), 0,
        POWERED_VERTICAL_AERO_MAX_ACCEL).
    LOCAL POWERED_HORIZONTAL_AERO_RAW_SAMPLE IS
        VXCL(UP_VEC, POWERED_AERO_SAMPLE).
    LOCAL POWERED_HORIZONTAL_AERO_SAMPLE IS CLAMPV(
        POWERED_HORIZONTAL_AERO_RAW_SAMPLE,
        POWERED_HORIZONTAL_AERO_MAX_ACCEL).
    SET POWERED_FILTERED_VERTICAL_AERO_ACCEL TO
        POWERED_FILTERED_VERTICAL_AERO_ACCEL
            * (1 - POWERED_AERO_ACCEL_FILTER)
        + POWERED_VERTICAL_AERO_SAMPLE * POWERED_AERO_ACCEL_FILTER.
    SET POWERED_FILTERED_HORIZONTAL_AERO_ACCEL TO
        POWERED_FILTERED_HORIZONTAL_AERO_ACCEL
            * (1 - POWERED_AERO_ACCEL_FILTER)
        + POWERED_HORIZONTAL_AERO_SAMPLE * POWERED_AERO_ACCEL_FILTER.
    SET POWERED_PREVIOUS_V TO REL_VEL.

    IF NOT TERMINAL_STEERING_TUNED
        AND HOOK_HEIGHT <= TERMINAL_STEERING_TUNE_HEIGHT {
        // kOS computes the torque PID gains from these settling times.  A
        // longer near-field response removes the reaction-wheel limit cycle
        // without changing the translational acceleration or tilt targets.
        SET STEERINGMANAGER:PITCHTS TO TERMINAL_STEERING_PITCH_YAW_TS.
        SET STEERINGMANAGER:YAWTS TO TERMINAL_STEERING_PITCH_YAW_TS.
        SET STEERINGMANAGER:ROLLTS TO TERMINAL_STEERING_ROLL_TS.
        SET STEERINGMANAGER:MAXSTOPPINGTIME TO
            TERMINAL_STEERING_MAX_STOPPING_TIME.
        SET STEERINGMANAGER:PITCHTORQUEFACTOR TO
            TERMINAL_STEERING_TORQUE_FACTOR.
        SET STEERINGMANAGER:YAWTORQUEFACTOR TO
            TERMINAL_STEERING_TORQUE_FACTOR.
        SET STEERINGMANAGER:ROLLTORQUEFACTOR TO
            TERMINAL_STEERING_TORQUE_FACTOR.
        // High-q grid-fin authority is no longer needed after the formal
        // waypoint.  Restore the 20/20/10 kN-m near-field response before the
        // upright handoff so the capture controller cannot snap or oscillate.
        LOCAL NEAR_FIELD_WHEEL_RESTORED IS
            SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
                BOOSTER_FLIGHT_REACTION_WHEEL_AUTHORITY).
        STEERINGMANAGER:RESETPIDS().
        SET TERMINAL_STEERING_TUNED TO TRUE.
    }

    LOCAL PID_MODE IS HOOK_HEIGHT <= PID_SWITCH_HEIGHT.
    LOCAL H_CORRIDOR_MODE IS HOOK_HEIGHT <= HORIZONTAL_CORRIDOR_HEIGHT
        AND HORIZONTAL_POS:MAG <= HORIZONTAL_CORRIDOR_RANGE.
    // This latch is intentionally one-way. Once the stage reaches the ship's
    // capture neighbourhood, a centre crossing must not re-arm the faster
    // stopping profile in the opposite direction.
    IF NOT CAPTURE_ALIGN_MODE AND H_CORRIDOR_MODE
        AND HOOK_HEIGHT <= TERMINAL_CAPTURE_ALIGN_ARM_HEIGHT
        AND TERMINAL_CONTROL_H_POS:MAG <= TERMINAL_ALIGN_RANGE {
        SET CAPTURE_ALIGN_MODE TO TRUE.
        LOCAL CAPTURE_ALIGN_RADIAL_SPEED IS 0.
        IF TERMINAL_CONTROL_H_POS:MAG > 0 {
            SET CAPTURE_ALIGN_RADIAL_SPEED TO VDOT(
                TERMINAL_CONTROL_H_POS:NORMALIZED, HORIZONTAL_VEL).
        }
        // This is a one-way *deceleration* handoff.  The old 30 m/s cap could
        // command a stage already approaching at 8-12 m/s to accelerate again
        // inside 300 m, feeding the attitude lag and causing another pass.
        // Keep enough floor for a near-tangential entry to converge; otherwise
        // never ask for more than the inward speed measured at the latch.
        SET CAPTURE_ALIGN_SPEED_LIMIT TO MAX(
            TERMINAL_ALIGN_MIN_SPEED, CAPTURE_ALIGN_RADIAL_SPEED).
    }
    // This high-energy latch is separate from the low-altitude capture field.
    // It can only fire while approaching the target and never releases, so a
    // centre crossing cannot command another full-speed pass.
    LOCAL BRAKE_ALONG_POS IS VDOT(TERMINAL_CONTROL_H_POS,
        CONTROL_ALONG_DIRECTION).
    LOCAL BRAKE_ALONG_VEL IS VDOT(HORIZONTAL_VEL,
        CONTROL_ALONG_DIRECTION).
    LOCAL BRAKE_CROSS_POS IS TERMINAL_CONTROL_H_POS
        - CONTROL_ALONG_DIRECTION * BRAKE_ALONG_POS.
    LOCAL BRAKE_CROSS_VEL IS HORIZONTAL_VEL
        - CONTROL_ALONG_DIRECTION * BRAKE_ALONG_VEL.
    LOCAL CROSS_TRACK_DIVERGING IS NOT CROSS_STOP_COMMITTED
        AND BRAKE_CROSS_POS:MAG
            >= TERMINAL_CROSS_TRACK_BRAKE_ERROR
        AND VDOT(BRAKE_CROSS_POS, BRAKE_CROSS_VEL) < 0
        AND TERMINAL_CONTROL_H_POS:MAG
            <= TERMINAL_CROSS_TRACK_BRAKE_RANGE.
    IF NOT HIGH_ENERGY_BRAKE_MODE AND H_CORRIDOR_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND HOOK_HEIGHT <= TERMINAL_HIGH_ENERGY_BRAKE_ARM_HEIGHT
        AND ((TERMINAL_CONTROL_H_POS:MAG
                <= TERMINAL_HIGH_ENERGY_BRAKE_RANGE
            AND VDOT(TERMINAL_CONTROL_H_POS, HORIZONTAL_VEL) > 0)
            OR CROSS_TRACK_DIVERGING) {
        SET HIGH_ENERGY_BRAKE_MODE TO TRUE.
    }
    // When the complete 2 km state is already reachable by a short coast,
    // latch zero throttle.  The previous controller reversed its thrust in
    // the last 40-80 m of height and turned a compliant centre crossing into
    // a 9-12 m/s rebound exactly at the formal measurement plane.
    IF TERMINAL_WAYPOINT_COAST_ENABLED
        AND NOT WAYPOINT_COAST_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL WAYPOINT_COAST_DROP IS MAX(HOOK_HEIGHT
            - TERMINAL_WAYPOINT_HEIGHT, 0).
        LOCAL WAYPOINT_COAST_VERTICAL_SPEED IS SQRT(MAX(
            VERTICAL_V^2 + 2 * G_ACC * WAYPOINT_COAST_DROP, 0)).
        LOCAL WAYPOINT_COAST_TGO IS (WAYPOINT_COAST_VERTICAL_SPEED
            + VERTICAL_V) / MAX(G_ACC, 0.1).
        LOCAL WAYPOINT_COAST_PREDICTED_H_POS IS HORIZONTAL_POS
            - HORIZONTAL_VEL * WAYPOINT_COAST_TGO.
        IF WAYPOINT_COAST_PREDICTED_H_POS:MAG
                <= TERMINAL_WAYPOINT_COAST_ENTRY_ERROR
            AND HORIZONTAL_VEL:MAG
                <= TERMINAL_WAYPOINT_COAST_ENTRY_HORIZONTAL_SPEED
            AND WAYPOINT_COAST_VERTICAL_SPEED
                >= TERMINAL_WAYPOINT_COAST_ENTRY_MIN_VERTICAL_SPEED
            AND WAYPOINT_COAST_VERTICAL_SPEED
                <= TERMINAL_WAYPOINT_COAST_MAX_VERTICAL_SPEED {
            SET WAYPOINT_COAST_MODE TO TRUE.
            IF NOT TERMINAL_WHEEL_AUTHORITY_ENABLED {
                SET TERMINAL_WHEEL_AUTHORITY_ENABLED TO
                    SET_BOOSTER_REACTION_WHEEL_AUTHORITY(
                        BOOSTER_TERMINAL_REACTION_WHEEL_AUTHORITY).
            }
            IF TERMINAL_WHEEL_AUTHORITY_ENABLED {
                SET STEERINGMANAGER:PITCHTS TO
                    TERMINAL_COAST_STEERING_PITCH_YAW_TS.
                SET STEERINGMANAGER:YAWTS TO
                    TERMINAL_COAST_STEERING_PITCH_YAW_TS.
                SET STEERINGMANAGER:ROLLTS TO
                    TERMINAL_COAST_STEERING_ROLL_TS.
                SET STEERINGMANAGER:MAXSTOPPINGTIME TO
                    TERMINAL_COAST_STEERING_MAX_STOPPING_TIME.
                SET STEERINGMANAGER:PITCHTORQUEFACTOR TO
                    TERMINAL_COAST_STEERING_TORQUE_FACTOR.
                SET STEERINGMANAGER:YAWTORQUEFACTOR TO
                    TERMINAL_COAST_STEERING_TORQUE_FACTOR.
                SET STEERINGMANAGER:ROLLTORQUEFACTOR TO
                    TERMINAL_COAST_STEERING_TORQUE_FACTOR.
                STEERINGMANAGER:RESETPIDS().
            }
        }
    }
    // The first entry into the formal waypoint circle is a one-way event.
    // Continue the existing braking direction instead of rebuilding a position
    // target on the opposite side of the ship after the centre crossing.
    IF WAYPOINT_COAST_MODE
        AND NOT WAYPOINT_CENTER_BRAKE_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND HORIZONTAL_POS:MAG
            <= TERMINAL_WAYPOINT_CENTER_BRAKE_ENTRY_ERROR
        AND VDOT(HORIZONTAL_POS, HORIZONTAL_VEL) > 0 {
        SET WAYPOINT_CENTER_BRAKE_MODE TO TRUE.
        SET WAYPOINT_CENTER_BRAKE_DIRECTION TO HORIZONTAL_VEL:NORMALIZED.
    }
    // Enter this state only once.  Holding the latch prevents a small position
    // excursion from switching the outer controller back on during settling.
    // The maximum-drag state becomes common near 12 km, after which an early
    // range deficit cannot be recovered. Latch the still-available 14 km
    // physical closing ratio and preload the existing ownership ramp only.
    IF NOT TERMINAL_EARLY_AERO_BRAKE_LATCHED
        AND HOOK_HEIGHT <= TERMINAL_EARLY_AERO_BRAKE_LATCH_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_EARLY_AERO_BRAKE_HOLD_END_HEIGHT {
        LOCAL TERMINAL_EARLY_AERO_BRAKE_INWARD_SPEED IS 0.
        IF HORIZONTAL_POS:MAG > 0.25 {
            SET TERMINAL_EARLY_AERO_BRAKE_INWARD_SPEED TO MAX(0, VDOT(
                HORIZONTAL_VEL, HORIZONTAL_POS:NORMALIZED)).
        }
        SET TERMINAL_EARLY_AERO_BRAKE_ENTRY_RATIO TO
            TERMINAL_EARLY_AERO_BRAKE_INWARD_SPEED
            / MAX(HORIZONTAL_POS:MAG, 1).
        SET TERMINAL_EARLY_AERO_BRAKE_FLOOR TO CLAMP(
            TERMINAL_EARLY_AERO_BRAKE_MIN_FLOOR
                + (TERMINAL_EARLY_AERO_BRAKE_ENTRY_RATIO
                    - TERMINAL_EARLY_AERO_BRAKE_LOW_RATIO)
                / MAX(
                    TERMINAL_EARLY_AERO_BRAKE_FULL_RATIO
                        - TERMINAL_EARLY_AERO_BRAKE_LOW_RATIO,
                    0.00001)
                * (1 - TERMINAL_EARLY_AERO_BRAKE_MIN_FLOOR),
            TERMINAL_EARLY_AERO_BRAKE_MIN_FLOOR,
            1).
        SET TERMINAL_EARLY_AERO_BRAKE_LATCHED TO TRUE.
    }
    IF NOT FINAL_ALIGN_MODE
        AND HOOK_HEIGHT <= FINAL_ALIGN_HEIGHT
        AND HORIZONTAL_POS:MAG <= FINAL_ALIGN_RANGE {
        LOCAL FINAL_ALIGN_ENTRY_INWARD_SPEED IS 0.
        IF HORIZONTAL_POS:MAG > 0.25 {
            SET FINAL_ALIGN_ENTRY_INWARD_SPEED TO MAX(0, VDOT(
                HORIZONTAL_VEL, HORIZONTAL_POS:NORMALIZED)).
        }
        SET FINAL_ALIGN_ENTRY_RATIO TO
            FINAL_ALIGN_ENTRY_INWARD_SPEED
            / MAX(HORIZONTAL_POS:MAG, 1).
        SET FINAL_ALIGN_ACTIVE_POSITION_GAIN TO CLAMP(
            FINAL_ALIGN_ENTRY_COMPLEMENT_GAIN
                - FINAL_ALIGN_ENTRY_RATIO,
            FINAL_ALIGN_MIN_POSITION_GAIN,
            FINAL_ALIGN_MAX_POSITION_GAIN).
        SET FINAL_ALIGN_MODE TO TRUE.
        SET FINAL_ALIGN_STARTED_AT TO NOW.
        SET VERTICAL_INTEGRAL TO 0.
        PRINT "FINAL ALIGN HOLD" AT(0,11).
    }
    // The 6--5 km phase only removes entrance dispersion before the dense-air
    // actuator saturates. Select the final phase line once at 5 km from that
    // transformed physical state, then retain it through the formal plane.
    IF FINAL_ALIGN_MODE
        AND NOT FINAL_ALIGN_TERMINAL_PHASE_LATCHED
        AND NOT FINAL_DESCENT_ARMED
        AND HOOK_HEIGHT <= FINAL_ALIGN_TERMINAL_PHASE_HEIGHT {
        LOCAL FINAL_ALIGN_TERMINAL_INWARD_SPEED IS 0.
        IF HORIZONTAL_POS:MAG > 0.25 {
            SET FINAL_ALIGN_TERMINAL_INWARD_SPEED TO MAX(0, VDOT(
                HORIZONTAL_VEL, HORIZONTAL_POS:NORMALIZED)).
        }
        SET FINAL_ALIGN_TERMINAL_ENTRY_RATIO TO
            FINAL_ALIGN_TERMINAL_INWARD_SPEED
            / MAX(HORIZONTAL_POS:MAG, 1).
        SET FINAL_ALIGN_ACTIVE_POSITION_GAIN TO CLAMP(
            FINAL_ALIGN_TERMINAL_ENTRY_RATIO,
            FINAL_ALIGN_TERMINAL_MIN_POSITION_GAIN,
            FINAL_ALIGN_TERMINAL_MAX_POSITION_GAIN).
        SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE TO CLAMP(
            FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_HOLD_SCALE
                + (FINAL_ALIGN_TERMINAL_ENTRY_RATIO
                    - FINAL_ALIGN_TERMINAL_AERO_BRAKE_LOW_RATIO)
                / MAX(
                    FINAL_ALIGN_TERMINAL_AERO_BRAKE_FULL_RATIO
                        - FINAL_ALIGN_TERMINAL_AERO_BRAKE_LOW_RATIO,
                    0.001)
                * (1 - FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_HOLD_SCALE),
            FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_HOLD_SCALE,
            1).
        SET FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT TO
            TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_END_HEIGHT
            + (FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_END_HEIGHT
                - TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_END_HEIGHT)
                * CLAMP(
                    (FINAL_ALIGN_TERMINAL_ENTRY_RATIO
                        - FINAL_ALIGN_TERMINAL_AERO_BRAKE_END_LOW_RATIO)
                    / MAX(
                        FINAL_ALIGN_TERMINAL_AERO_BRAKE_END_FULL_RATIO
                            - FINAL_ALIGN_TERMINAL_AERO_BRAKE_END_LOW_RATIO,
                        0.001),
                    0, 1).
        SET FINAL_ALIGN_TERMINAL_PHASE_LATCHED TO TRUE.
    }
    IF PID_MODE AND NOT WAS_PID_MODE {
        SET VERTICAL_INTEGRAL TO 0.
        PRINT "NEAR-FIELD PID HANDOVER" AT(0,11).
    }
    SET WAS_PID_MODE TO PID_MODE.

    LOCAL CENTERING_REQUIRED IS PID_MODE
        AND HOOK_HEIGHT < CENTERING_HOLD_ALTITUDE
        AND HORIZONTAL_POS:MAG > CENTERING_HOLD_ERROR.
    LOCAL WIRE_GEOMETRY_REQUIRED IS HOOK_HEIGHT < WIRE_HOLD_HEIGHT
        AND HORIZONTAL_POS:MAG <= WIRE_HOLD_HORIZONTAL_RANGE
        AND NOT NET_CLOSED(RECOVERY_SHIP).
    IF WIRE_GEOMETRY_REQUIRED AND WIRE_HOLD_STARTED_AT < 0 {
        SET WIRE_HOLD_STARTED_AT TO NOW.
    }
    // Module fields exposed through kOS can retain a stale boxed string even
    // while the plugin GUI already reports Closed. The hold is therefore a
    // one-shot timed state, not an unlimited dependency on that field.
    LOCAL WIRE_REQUIRED IS WIRE_GEOMETRY_REQUIRED
        AND NOW - WIRE_HOLD_STARTED_AT < WIRE_HOLD_MAX_SECONDS
        AND NOT FINAL_ALIGN_MODE.

    SET FUEL_URGENT TO BOOSTER_PROPELLANT_FRACTION()
        < TERMINAL_LOW_FUEL_FRACTION.
    // A height-indexed descent corridor guarantees continued downward motion:
    // high energy is accepted above the ship, while low speed is reserved for
    // the final tens of metres. This avoids both an expired-deadline dive and
    // a receding-horizon hover.
    LOCAL CORRIDOR_DOWN_SPEED IS MAX(CAPTURE_FINAL_SPEED,
        MIN(TERMINAL_DESCENT_MAX_SPEED, MAX(HOOK_HEIGHT, 0)
            * TERMINAL_WAYPOINT_VERTICAL_SPEED
            / TERMINAL_WAYPOINT_HEIGHT)).
    IF FUEL_URGENT {
        SET CORRIDOR_DOWN_SPEED TO CORRIDOR_DOWN_SPEED
            * TERMINAL_LOW_FUEL_DESCENT_SCALE.
    }
    LOCAL TARGET_VERTICAL_V IS -CORRIDOR_DOWN_SPEED.
    LOCAL NET_VERTICAL_ACCEL IS V_VEL_KP
        * (TARGET_VERTICAL_V - VERTICAL_V).
    // Do not pay gravity-compensation fuel while the stage is already falling
    // more slowly than the permitted descent corridor.  The previous gate
    // held roughly one-g thrust from 30 km and exhausted the recovery reserve
    // around 5 km.  Braking is required only when downward speed exceeds the
    // current height-indexed limit.
    IF VERTICAL_V < TARGET_VERTICAL_V {
        // Preserve the full uncertainty margin at high-energy ignition, then
        // taper it away as measurements improve. Keeping 1.5 all the way down
        // would deliberately stop the stage hundreds of metres too early.
        LOCAL BRAKE_SAFETY_BLEND IS CLAMP(HOOK_HEIGHT
            / TERMINAL_GUIDANCE_START_HEIGHT, 0, 1).
        LOCAL BRAKE_SAFETY IS 1 + (STOP_DISTANCE_SAFETY - 1)
            * BRAKE_SAFETY_BLEND.
        LOCAL SAFE_BRAKING_ACCEL IS (VERTICAL_V^2)
            / (2 * MAX(HOOK_HEIGHT, 1)) * BRAKE_SAFETY.
        SET NET_VERTICAL_ACCEL TO MAX(NET_VERTICAL_ACCEL,
            SAFE_BRAKING_ACCEL).
    }
    LOCAL VERTICAL_THRUST_CMD IS G_ACC + NET_VERTICAL_ACCEL.
    IF HOOK_HEIGHT > PLAN_HORIZONTAL_END_HEIGHT {
        // Solve the vertical component against the same finite 2 km state used
        // by the ignition calculation.  For constant net acceleration,
        // t = 2*distance/(initial_down_speed + final_down_speed).
        LOCAL WAYPOINT_VERTICAL_TGO IS 2
            * (HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT)
            / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
        LOCAL WAYPOINT_NET_VERTICAL_ACCEL IS
            (-TERMINAL_WAYPOINT_VERTICAL_SPEED - VERTICAL_V)
            / MAX(WAYPOINT_VERTICAL_TGO, 0.1).
        SET VERTICAL_THRUST_CMD TO G_ACC
            + WAYPOINT_NET_VERTICAL_ACCEL.
    }
    // The lower edge of the 2 km gate is as important as its upper edge.  If
    // model error makes the stage descend more slowly than 150 m/s before the
    // waypoint, remove deliberate vertical braking and let gravity restore the
    // corridor instead of spending fuel to arrive even slower.
    IF HOOK_HEIGHT >= TERMINAL_WAYPOINT_HEIGHT
        AND VERTICAL_V > -TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED {
        SET VERTICAL_THRUST_CMD TO 0.
    }
    // Cubic Hermite reference indexed by lost height.  Because progress comes
    // from altitude rather than a repeatedly extended deadline, the reference
    // reaches the frame entrance exactly once and cannot jump behind the stage.
    // Reach zero horizontal position and speed above the 2 km waypoint. The
    // extra settling height absorbs the long stage's attitude lag; below the
    // endpoint the reference remains at the ship centre, producing the
    // requested nearly vertical final leg instead of a late curved approach.
    LOCAL PLAN_HORIZONTAL_HEIGHT IS MAX(PLAN_HOOK_HEIGHT
        - PLAN_HORIZONTAL_END_HEIGHT, 1).
    LOCAL PLAN_PROGRESS IS CLAMP((PLAN_HOOK_HEIGHT - HOOK_HEIGHT)
        / PLAN_HORIZONTAL_HEIGHT, 0, 1).
    LOCAL PLAN_PROGRESS2 IS PLAN_PROGRESS^2.
    LOCAL PLAN_PROGRESS3 IS PLAN_PROGRESS^3.
    LOCAL PLAN_H00 IS 2 * PLAN_PROGRESS3 - 3 * PLAN_PROGRESS2 + 1.
    LOCAL PLAN_H10 IS PLAN_PROGRESS3 - 2 * PLAN_PROGRESS2 + PLAN_PROGRESS.
    LOCAL PLAN_REFERENCE_H_POS IS PLAN_CONTROL_H_POS * PLAN_H00
        + PLAN_H_ERROR_SLOPE0 * PLAN_H10.
    LOCAL PLAN_D_ERROR_DS IS PLAN_CONTROL_H_POS
        * (6 * PLAN_PROGRESS2 - 6 * PLAN_PROGRESS)
        + PLAN_H_ERROR_SLOPE0
        * (3 * PLAN_PROGRESS2 - 4 * PLAN_PROGRESS + 1).
    LOCAL PLAN_D2_ERROR_DS2 IS PLAN_CONTROL_H_POS
        * (12 * PLAN_PROGRESS - 6)
        + PLAN_H_ERROR_SLOPE0 * (6 * PLAN_PROGRESS - 4).
    LOCAL PLAN_PROGRESS_RATE IS MAX(-VERTICAL_V,
        CAPTURE_FINAL_SPEED) / PLAN_HORIZONTAL_HEIGHT.
    LOCAL PLAN_REFERENCE_H_VEL IS -PLAN_D_ERROR_DS
        * PLAN_PROGRESS_RATE.
    // The cubic supplies the route, while this stopping envelope guarantees
    // that its velocity cannot carry the long stage across the frame before
    // attitude lag has produced the requested braking acceleration.
    LOCAL PLAN_STOP_SPEED IS SQRT(2 * TERMINAL_PLAN_STOP_ACCEL
        * MAX(HORIZONTAL_POS:MAG - TERMINAL_HORIZONTAL_DEADBAND, 0)).
    SET PLAN_REFERENCE_H_VEL TO CLAMPV(PLAN_REFERENCE_H_VEL,
        PLAN_STOP_SPEED).
    LOCAL PLAN_FEEDFORWARD_H_ACCEL IS -PLAN_D2_ERROR_DS2
        * PLAN_PROGRESS_RATE^2.
    LOCAL H_ACCEL IS PLAN_FEEDFORWARD_H_ACCEL
        + (WAYPOINT_CONTROL_H_POS - PLAN_REFERENCE_H_POS)
            * TERMINAL_PLAN_POSITION_GAIN
        + (PLAN_REFERENCE_H_VEL - HORIZONTAL_VEL)
            * TERMINAL_PLAN_VELOCITY_GAIN.
    SET H_ACCEL TO CLAMPV(H_ACCEL, TERMINAL_MAX_HORIZONTAL_ACCEL).
    IF HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        // Finite-time cubic boundary controller for the formal 2 km state.
        // For remaining time T, a = 6*position/T^2 - 4*velocity/T reaches
        // zero position and zero horizontal velocity together.  This direct
        // endpoint condition removes the phase error left by merely tracking
        // a height-indexed reference with the long stage's attitude lag.
        LOCAL HORIZONTAL_VERTICAL_TGO IS 2
            * (HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT)
            / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
        LOCAL HORIZONTAL_VERTICAL_NET_ACCEL IS
            (-TERMINAL_WAYPOINT_VERTICAL_SPEED - VERTICAL_V)
            / MAX(HORIZONTAL_VERTICAL_TGO, 0.1).
        LOCAL HORIZONTAL_TARGET_DROP IS HOOK_HEIGHT
            - PLAN_HORIZONTAL_END_HEIGHT.
        LOCAL HORIZONTAL_TARGET_TGO IS HORIZONTAL_TARGET_DROP
            / MAX(-VERTICAL_V, 1).
        IF HORIZONTAL_VERTICAL_NET_ACCEL > 0.001 {
            LOCAL HORIZONTAL_TARGET_DISC IS MAX(VERTICAL_V^2
                - 2 * HORIZONTAL_VERTICAL_NET_ACCEL
                    * HORIZONTAL_TARGET_DROP, 0).
            SET HORIZONTAL_TARGET_TGO TO (-VERTICAL_V
                - SQRT(HORIZONTAL_TARGET_DISC))
                / HORIZONTAL_VERTICAL_NET_ACCEL.
        }
        LOCAL HORIZONTAL_CONTROL_TGO IS MAX(HORIZONTAL_TARGET_TGO
            - TERMINAL_HORIZONTAL_LEAD_SECONDS, 0.5).
        LOCAL HORIZONTAL_CONTROL_POS IS WAYPOINT_CONTROL_H_POS
            - HORIZONTAL_VEL * TERMINAL_HORIZONTAL_LEAD_SECONDS.
        // Freeze the downrange axis at main ignition.  A single vector gain
        // made the small cross-track correction reverse far too late in real
        // flight: run 40 crossed the target latitude near 7 km and reached the
        // 2 km plane 414 m beyond it.  Split the two horizontal components so
        // extra cross-track damping cannot disturb the proven downrange law.
        LOCAL HORIZONTAL_ALONG_CONTROL_POS IS CONTROL_ALONG_DIRECTION
            * VDOT(HORIZONTAL_CONTROL_POS, CONTROL_ALONG_DIRECTION).
        LOCAL HORIZONTAL_ALONG_VEL IS CONTROL_ALONG_DIRECTION
            * VDOT(HORIZONTAL_VEL, CONTROL_ALONG_DIRECTION).
        LOCAL HORIZONTAL_CROSS_CONTROL_POS IS HORIZONTAL_CONTROL_POS
            - HORIZONTAL_ALONG_CONTROL_POS.
        LOCAL HORIZONTAL_CROSS_VEL IS HORIZONTAL_VEL
            - HORIZONTAL_ALONG_VEL.
        SET H_ACCEL TO HORIZONTAL_ALONG_CONTROL_POS
                * TERMINAL_WAYPOINT_POSITION_COEFFICIENT
                / MAX(HORIZONTAL_CONTROL_TGO^2, 0.25)
            - HORIZONTAL_ALONG_VEL
                * TERMINAL_WAYPOINT_VELOCITY_COEFFICIENT
                / HORIZONTAL_CONTROL_TGO
            + HORIZONTAL_CROSS_CONTROL_POS
                * TERMINAL_WAYPOINT_CROSS_POSITION_COEFFICIENT
                / MAX(HORIZONTAL_CONTROL_TGO^2, 0.25)
            - HORIZONTAL_CROSS_VEL
                * TERMINAL_WAYPOINT_CROSS_VELOCITY_COEFFICIENT
                / HORIZONTAL_CONTROL_TGO.
        SET H_ACCEL TO CLAMPV(H_ACCEL, TERMINAL_MAX_HORIZONTAL_ACCEL).
    }
    // Retain the old velocity-axis stopping law only as an explicit A/B mode.
    // It is disabled for flight because its rotating basis left a 1.70 km
    // cross-track residual in run 30.  The fixed surface-frame finite-time law
    // above controls both horizontal axes to the same 2 km endpoint.
    IF TERMINAL_MAIN_DIRECT_STOP_ENABLED
        AND HOOK_HEIGHT > PLAN_HORIZONTAL_END_HEIGHT
        AND NOT HIGH_ENERGY_BRAKE_MODE
        AND HORIZONTAL_VEL:MAG > 0.1 {
        LOCAL MAIN_H_VEL_DIR IS HORIZONTAL_VEL:NORMALIZED.
        LOCAL MAIN_ALONG_RANGE IS MAX(VDOT(
            TERMINAL_CONTROL_H_POS, MAIN_H_VEL_DIR), 0).
        LOCAL MAIN_COMPENSATED_STOP_RANGE IS MAX(
            MAIN_ALONG_RANGE
                - HORIZONTAL_VEL:MAG
                    * TERMINAL_MAIN_ATTITUDE_RESPONSE_SECONDS
                - TERMINAL_HORIZONTAL_DEADBAND,
            TERMINAL_MAIN_DIRECT_STOP_MIN_RANGE).
        LOCAL MAIN_REQUIRED_H_DECEL IS HORIZONTAL_VEL:MAG^2
            / (2 * MAIN_COMPENSATED_STOP_RANGE)
            * TERMINAL_MAIN_DIRECT_STOP_DECEL_GAIN.
        LOCAL MAIN_CROSS_ERROR IS TERMINAL_CONTROL_H_POS
            - MAIN_H_VEL_DIR * MAIN_ALONG_RANGE.
        LOCAL MAIN_CROSS_TGO IS 2
            * MAX(HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT, 0)
            / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
        LOCAL MAIN_CROSS_ACCEL IS CLAMPV(MAIN_CROSS_ERROR
                * TERMINAL_MAIN_DIRECT_STOP_CROSS_TIME_COEFFICIENT
                / MAX(MAIN_CROSS_TGO^2, 1),
            TERMINAL_MAIN_DIRECT_STOP_CROSS_MAX_ACCEL).
        SET H_ACCEL TO -MAIN_H_VEL_DIR * MAIN_REQUIRED_H_DECEL
            + MAIN_CROSS_ACCEL.
        SET H_ACCEL TO CLAMPV(H_ACCEL,
            TERMINAL_MAX_HORIZONTAL_ACCEL).
    }
    // Keep the fixed downrange component inside a reachable stopping footprint.
    // Run 72 showed that an unrestricted v^2/(2r) law activates at main
    // ignition and simply recreates the early broadside branch. Below the late
    // arm height, compare the uncapped demand with a live upper bound: full
    // engine acceleration projected through the mandatory velocity cone plus
    // identified aerodynamic braking. A 10% margin covers filtering/model lag.
    // The persistent latch prevents feasibility noise from becoming guidance
    // chatter, while the positive range/velocity gates still prevent a return
    // pass after centre crossing.
    LOCAL FIXED_STOP_ACTIVE IS FALSE.
    LOCAL FIXED_STOP_COMMITTED_THIS_TICK IS FALSE.
    LOCAL FIXED_STOP_EDGE_ACTIVE IS FALSE.
    LOCAL FIXED_STOP_FEASIBLE_DIAG IS FALSE.
    LOCAL FIXED_STOP_USABLE_RANGE_DIAG IS -1.
    LOCAL FIXED_STOP_REQUIRED_DECEL_DIAG IS 0.
    LOCAL FIXED_STOP_RAW_REQUIRED_DECEL_DIAG IS 0.
    LOCAL FIXED_STOP_AUTHORITY_DIAG IS 0.
    LOCAL FIXED_STOP_SYNC_EXCESS_DIAG IS 0.
    LOCAL CROSS_STOP_ACTIVE IS FALSE.
    LOCAL CROSS_STOP_COMMITTED_THIS_TICK IS FALSE.
    LOCAL CROSS_STOP_SUPPRESS_ACTIVE IS FALSE.
    LOCAL CROSS_STOP_POS_DIAG IS 0.
    LOCAL CROSS_STOP_VEL_DIAG IS 0.
    LOCAL CROSS_STOP_USABLE_RANGE_DIAG IS -1.
    LOCAL CROSS_STOP_REQUIRED_DECEL_DIAG IS 0.
    LOCAL CROSS_STOP_FINISH_ACTIVE IS FALSE.
    LOCAL CROSS_STOP_FINISH_ACCEL_DIAG IS 0.
    LOCAL CROSS_AERO_BRAKE_STEERING_ACTIVE IS FALSE.
    LOCAL CROSS_AERO_BRAKE_STEERING_AXIS_DIAG IS 0.
    LOCAL CROSS_AERO_BRAKE_RELEASE_BLEND_DIAG IS 0.
    LOCAL CROSS_AERO_BRAKE_REQUEST_ACCEL_DIAG IS 0.
    LOCAL CROSS_AERO_BRAKE_REALIZED_DECEL_DIAG IS 0.
    LOCAL CROSS_AERO_BRAKE_RESIDUAL_DEMAND_DIAG IS 0.
    IF TERMINAL_MAIN_FIXED_AXIS_STOP_ENABLED
        AND HOOK_HEIGHT > PLAN_HORIZONTAL_END_HEIGHT
        AND HOOK_HEIGHT <= TERMINAL_MAIN_FIXED_AXIS_ARM_HEIGHT
        AND NOT HIGH_ENERGY_BRAKE_MODE {
        LOCAL FIXED_STOP_ALONG_POS IS VDOT(
            WAYPOINT_CONTROL_H_POS, CONTROL_ALONG_DIRECTION).
        LOCAL FIXED_STOP_ALONG_VEL IS VDOT(
            HORIZONTAL_VEL, CONTROL_ALONG_DIRECTION).
        IF FIXED_STOP_ALONG_POS > TERMINAL_HORIZONTAL_DEADBAND
            AND FIXED_STOP_ALONG_VEL > 0.1 {
            LOCAL FIXED_STOP_USABLE_RANGE IS MAX(
                FIXED_STOP_ALONG_POS
                    - FIXED_STOP_ALONG_VEL
                        * TERMINAL_MAIN_FIXED_AXIS_RESPONSE_SECONDS
                    - TERMINAL_HORIZONTAL_DEADBAND,
                TERMINAL_MAIN_FIXED_AXIS_STOP_MIN_RANGE).
            LOCAL FIXED_STOP_RAW_REQUIRED_DECEL IS
                FIXED_STOP_ALONG_VEL^2
                    / (2 * FIXED_STOP_USABLE_RANGE)
                    * TERMINAL_MAIN_FIXED_AXIS_STOP_GAIN.
            LOCAL FIXED_STOP_REQUIRED_DECEL IS MIN(
                TERMINAL_MAX_HORIZONTAL_ACCEL,
                FIXED_STOP_RAW_REQUIRED_DECEL).
            SET FIXED_STOP_USABLE_RANGE_DIAG TO
                FIXED_STOP_USABLE_RANGE.
            SET FIXED_STOP_REQUIRED_DECEL_DIAG TO
                FIXED_STOP_REQUIRED_DECEL.
            SET FIXED_STOP_RAW_REQUIRED_DECEL_DIAG TO
                FIXED_STOP_RAW_REQUIRED_DECEL.
            LOCAL FIXED_STOP_SAFETY_AXIS IS THRUST_SAFETY_AXIS(
                UP_VEC, TERMINAL_VELOCITY_CONE_MIN_SPEED).
            LOCAL FIXED_STOP_BRAKE_DIRECTION IS
                -CONTROL_ALONG_DIRECTION.
            LOCAL FIXED_STOP_AXIS_SEPARATION IS VANG(
                FIXED_STOP_SAFETY_AXIS, FIXED_STOP_BRAKE_DIRECTION).
            LOCAL FIXED_STOP_ENGINE_PROJECTION IS CLAMP(COS(MAX(
                FIXED_STOP_AXIS_SEPARATION
                    - TERMINAL_COMMAND_CONE_DEGREES, 0)), 0, 1).
            LOCAL FIXED_STOP_AERO_DECEL IS MAX(-VDOT(
                POWERED_FILTERED_HORIZONTAL_AERO_ACCEL,
                CONTROL_ALONG_DIRECTION), 0).
            LOCAL FIXED_STOP_AVAILABLE_DECEL IS
                AVAILABLE_ACC * FIXED_STOP_ENGINE_PROJECTION
                + FIXED_STOP_AERO_DECEL.
            SET FIXED_STOP_AUTHORITY_DIAG TO FIXED_STOP_AVAILABLE_DECEL.
            IF FIXED_STOP_RAW_REQUIRED_DECEL
                    <= FIXED_STOP_AVAILABLE_DECEL
                        * TERMINAL_MAIN_FIXED_AXIS_AUTHORITY_MARGIN {
                SET FIXED_STOP_FEASIBLE_DIAG TO TRUE.
            }
            LOCAL FIXED_STOP_CURRENT_ALONG_ACCEL IS VDOT(
                H_ACCEL, CONTROL_ALONG_DIRECTION).
            LOCAL FIXED_STOP_SYNC_TGO IS 2
                * MAX(HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT, 0)
                / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
            LOCAL FIXED_STOP_SYNC_REFERENCE_SPEED IS
                2 * FIXED_STOP_ALONG_POS
                    / MAX(FIXED_STOP_SYNC_TGO, 0.5).
            SET FIXED_STOP_SYNC_EXCESS_DIAG TO
                FIXED_STOP_ALONG_VEL
                    - FIXED_STOP_SYNC_REFERENCE_SPEED.
            IF NOT FIXED_STOP_COMMITTED
                AND FIXED_STOP_FEASIBLE_DIAG
                AND FIXED_STOP_SYNC_EXCESS_DIAG
                    > TERMINAL_MAIN_FIXED_AXIS_SYNC_EXCESS_ARM
                AND FIXED_STOP_CURRENT_ALONG_ACCEL
                    > -FIXED_STOP_REQUIRED_DECEL {
                SET FIXED_STOP_COMMITTED TO TRUE.
                SET FIXED_STOP_COMMITTED_THIS_TICK TO TRUE.
            }
            IF FIXED_STOP_COMMITTED
                AND FIXED_STOP_CURRENT_ALONG_ACCEL
                    > -FIXED_STOP_REQUIRED_DECEL {
                SET FIXED_STOP_ACTIVE TO TRUE.
                SET H_ACCEL TO H_ACCEL
                    - CONTROL_ALONG_DIRECTION
                        * (FIXED_STOP_CURRENT_ALONG_ACCEL
                            + FIXED_STOP_REQUIRED_DECEL).
                SET H_ACCEL TO CLAMPV(H_ACCEL,
                    TERMINAL_MAX_HORIZONTAL_ACCEL).
            }
        }
    }
    // Once the powered trajectory enters its final stopping neighbourhood,
    // damp the measured velocity through a bounded, low-speed inward field.
    // Below 2.2 km the desired velocity becomes zero; position pursuit cannot
    // reverse the long stage just before the waypoint.
    IF HIGH_ENERGY_BRAKE_MODE
        AND HOOK_HEIGHT > PLAN_HORIZONTAL_END_HEIGHT {
        LOCAL HIGH_ENERGY_DESIRED_H_VEL IS V(0,0,0).
        IF HOOK_HEIGHT > TERMINAL_HIGH_ENERGY_BRAKE_SETTLE_HEIGHT
            AND TERMINAL_CONTROL_H_POS:MAG
                > TERMINAL_HORIZONTAL_DEADBAND {
            LOCAL HIGH_ENERGY_DESIRED_SPEED IS MIN(
                TERMINAL_HIGH_ENERGY_BRAKE_MAX_SPEED,
                (TERMINAL_CONTROL_H_POS:MAG
                    - TERMINAL_HORIZONTAL_DEADBAND)
                    * TERMINAL_HIGH_ENERGY_BRAKE_POSITION_GAIN).
            SET HIGH_ENERGY_DESIRED_H_VEL TO
                TERMINAL_CONTROL_H_POS:NORMALIZED
                * HIGH_ENERGY_DESIRED_SPEED.
        }
        SET H_ACCEL TO CLAMPV((HIGH_ENERGY_DESIRED_H_VEL
                - HORIZONTAL_VEL)
                * TERMINAL_HIGH_ENERGY_BRAKE_VELOCITY_GAIN,
            TERMINAL_HIGH_ENERGY_BRAKE_MAX_ACCEL).
    }
    // The 30 km Hermite trajectory owns the high-energy translation.  Once its
    // one-way 300 m neighbourhood latch fires, use the range-indexed velocity
    // field immediately instead of continuing to chase the mathematical path
    // down to 2 km.  Real-flight telemetry showed the old delayed handoff
    // crossing the ship near 5-6 km, then holding a 60-80 m correction and a
    // near-horizontal attitude all the way to the waypoint.
    IF H_CORRIDOR_MODE AND (CAPTURE_ALIGN_MODE
        OR HOOK_HEIGHT <= TERMINAL_WAYPOINT_HEIGHT) {
        // A range-indexed stopping-speed corridor replaces the former direct
        // position gain. The old loop entered at 18 m with residual lateral
        // speed, crossed the ship, then repeatedly reversed. Enter early and
        // command only a velocity that can be stopped inside the remaining
        // range, including margin for the long stage's attitude response.
        LOCAL PID_RANGE IS TERMINAL_CONTROL_H_POS:MAG.
        LOCAL PID_STOP_RANGE IS MAX(PID_RANGE
            - TERMINAL_HORIZONTAL_DEADBAND, 0).
        LOCAL PID_STOP_SPEED IS SQRT(2 * TERMINAL_HORIZONTAL_STOP_ACCEL
            * PID_STOP_RANGE).
        // Do not taper horizontal speed merely because altitude is low while
        // the stage is still hundreds of metres from the frame.  That old
        // height cap stopped valid approaches about 350 m short.  Range and
        // available stopping acceleration are the actual constraints.
        LOCAL PID_H_SPEED IS MIN(PID_STOP_SPEED,
            TERMINAL_HORIZONTAL_CORRIDOR_SPEED).
        IF CAPTURE_ALIGN_MODE {
            // First damp residual speed, then follow a linear velocity field.
            // The lower inner-loop gain accounts for the long stage's several-
            // second attitude response. A centre
            // crossing can now produce only a small correction, not another
            // full-speed pursuit in the opposite direction.
            // Preserve the outer stopping envelope as a hard upper bound.
            // The former assignment replaced PID_STOP_SPEED with the linear
            // position field, so TERMINAL_HORIZONTAL_STOP_ACCEL had no effect
            // after capture-align latched—the exact phase where attitude-lag
            // braking was required.
            SET PID_H_SPEED TO MIN(PID_H_SPEED,
                MIN(TERMINAL_ALIGN_SPEED,
                    MIN(CAPTURE_ALIGN_SPEED_LIMIT,
                        PID_STOP_RANGE * TERMINAL_ALIGN_POSITION_GAIN))).
        }
        LOCAL DESIRED_H_VEL IS V(0,0,0).
        IF PID_RANGE > TERMINAL_HORIZONTAL_DEADBAND {
            SET DESIRED_H_VEL TO TERMINAL_CONTROL_H_POS:NORMALIZED
                * PID_H_SPEED.
        }
        LOCAL PID_VELOCITY_GAIN IS H_VEL_KP.
        LOCAL PID_ACCEL_LIMIT IS TERMINAL_MAX_HORIZONTAL_ACCEL.
        IF CAPTURE_ALIGN_MODE {
            SET PID_VELOCITY_GAIN TO TERMINAL_ALIGN_VELOCITY_GAIN.
            SET PID_ACCEL_LIMIT TO MAX(TERMINAL_HORIZONTAL_STOP_ACCEL,
                MIN(TERMINAL_MAX_HORIZONTAL_ACCEL,
                    MAX(PID_STOP_RANGE * TERMINAL_ALIGN_ACCEL_RANGE_GAIN,
                        HORIZONTAL_VEL:MAG
                            * TERMINAL_ALIGN_ACCEL_VELOCITY_GAIN))).
        }
        SET H_ACCEL TO CLAMPV((DESIRED_H_VEL - HORIZONTAL_VEL)
            * PID_VELOCITY_GAIN, PID_ACCEL_LIMIT).
    }

    // Once both formal horizontal limits are first satisfied, position chasing
    // is counterproductive above the waypoint: attitude lag can turn a tiny
    // correction into another centre crossing and keep the long stage broadside
    // to the airflow.  Latch a pure velocity-damping finish, with a shallow
    // acceleration cap.  At the 2 km waypoint hand control back to the bounded
    // range field: real-flight telemetry showed that aerodynamic drift can add
    // roughly 140 m after this latch, which is outside the 1 km final-align
    // acquisition radius if position feedback remains disabled forever.
    IF NOT HORIZONTAL_SETTLE_MODE AND CAPTURE_ALIGN_MODE
        AND TERMINAL_CONTROL_H_POS:MAG
            <= TERMINAL_ALIGN_SETTLE_ENTRY_RANGE
        AND HORIZONTAL_VEL:MAG <= TERMINAL_ALIGN_SETTLE_ENTRY_SPEED {
        SET HORIZONTAL_SETTLE_MODE TO TRUE.
        SET FILTERED_H_ACCEL TO V(0,0,0).
    }
    IF HORIZONTAL_SETTLE_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET H_ACCEL TO CLAMPV(TERMINAL_CONTROL_H_POS
            * TERMINAL_ALIGN_SETTLE_POSITION_GAIN
            - HORIZONTAL_VEL * TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN,
            TERMINAL_ALIGN_SETTLE_MAX_ACCEL).
        IF TERMINAL_CONTROL_H_POS:MAG > TERMINAL_ALIGN_REACQUIRE_RANGE {
            SET H_ACCEL TO CLAMPV(TERMINAL_CONTROL_H_POS
                * TERMINAL_ALIGN_REACQUIRE_POSITION_GAIN
                - HORIZONTAL_VEL * TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN,
                TERMINAL_ALIGN_REACQUIRE_MAX_ACCEL).
        }
    }
    LOCAL FINAL_ALIGN_TERMINAL_FINITE_TIME_ACTIVE IS FALSE.
    LOCAL FINAL_ALIGN_TERMINAL_TGO IS 0.
    LOCAL FINAL_ALIGN_TERMINAL_CONTROL_TGO IS 0.
    LOCAL FINAL_ALIGN_TERMINAL_CONTROL_POS IS V(0,0,0).
    LOCAL FINAL_ALIGN_TERMINAL_ACTIVE_RESPONSE_LEAD_SECONDS IS
        FINAL_ALIGN_TERMINAL_RESPONSE_LEAD_SECONDS.
    LOCAL FINAL_ALIGN_TERMINAL_LOW_FAMILY_BLEND IS 1 - CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_LOW_FAMILY_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_LOW_FAMILY_END_RATIO
            - FINAL_ALIGN_TERMINAL_LOW_FAMILY_RATIO, 0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_HIGH_START_RANGE_BLEND IS CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_HIGH_RATIO)
        / MAX(FINAL_ALIGN_FINITE_TIME_START_HIGH_FULL_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_HIGH_RATIO,
            0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_RANGE_FAMILY_BLEND IS MAX(
        FINAL_ALIGN_TERMINAL_LOW_FAMILY_BLEND,
        FINAL_ALIGN_TERMINAL_HIGH_START_RANGE_BLEND).
    IF FINAL_ALIGN_MODE
        AND FINAL_ALIGN_TERMINAL_PHASE_LATCHED
        AND NOT FINAL_DESCENT_ARMED
        AND NOT FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCHED
        AND HOOK_HEIGHT <= FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCH_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        // Preserve endpoint phase rather than folding an already-crossed
        // sample back onto the approach-side magnitude.
        SET FINAL_ALIGN_TERMINAL_LOW_RANGE_AT_LATCH TO VDOT(
            HORIZONTAL_POS, CONTROL_ALONG_DIRECTION).
        SET FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCHED TO TRUE.
    }
    LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_LOW_BLEND IS CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_START_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_FULL_START_RATIO
            - FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_START_RATIO,
            0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_HIGH_BLEND IS 1 - CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_FULL_END_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_END_RATIO
            - FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_FULL_END_RATIO,
            0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_BLEND IS MIN(
        FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_LOW_BLEND,
        FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_HIGH_BLEND).
    IF FINAL_ALIGN_MODE
        AND FINAL_ALIGN_TERMINAL_PHASE_LATCHED
        AND NOT FINAL_DESCENT_ARMED
        AND NOT FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCHED
        AND FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_BLEND > 0
        AND HOOK_HEIGHT
            <= FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCH_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTION_TGO IS MAX(
            (HOOK_HEIGHT
                - FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_PROJECTION_HEIGHT)
            / MAX(-VERTICAL_V, 1), 0).
        SET FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTED_RANGE_AT_LATCH TO
            VDOT(HORIZONTAL_POS, CONTROL_ALONG_DIRECTION)
            - VDOT(HORIZONTAL_VEL, CONTROL_ALONG_DIRECTION)
                * FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTION_TGO.
        SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCHED TO TRUE.
    }
    LOCAL FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS IS 0.
    LOCAL FINAL_ALIGN_TERMINAL_RANGE_BIAS_HEIGHT_BLEND IS CLAMP(
        (HOOK_HEIGHT
            - FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_END_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_START_HEIGHT
            - FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_END_HEIGHT,
            1), 0, 1).
    IF FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCHED {
        SET FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS TO CLAMP(
            (FINAL_ALIGN_TERMINAL_LOW_RANGE_REFERENCE
                - FINAL_ALIGN_TERMINAL_LOW_RANGE_AT_LATCH)
                * FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS_GAIN,
            -FINAL_ALIGN_TERMINAL_LOW_RANGE_MAX_POSITION_BIAS,
            FINAL_ALIGN_TERMINAL_LOW_RANGE_MAX_POSITION_BIAS)
            * FINAL_ALIGN_TERMINAL_RANGE_FAMILY_BLEND
            * FINAL_ALIGN_TERMINAL_RANGE_BIAS_HEIGHT_BLEND.
    }
    LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_HEIGHT_BLEND IS CLAMP(
        (HOOK_HEIGHT - FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_END_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCH_HEIGHT
            - FINAL_ALIGN_TERMINAL_RANGE_BIAS_FADE_END_HEIGHT, 1), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS IS 0.
    IF FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCHED {
        SET FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS TO CLAMP(
            (FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_REFERENCE
                - FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTED_RANGE_AT_LATCH)
                * FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS_GAIN,
            -FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_MAX_POSITION_BIAS,
            FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_MAX_POSITION_BIAS)
            * FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_BLEND
            * FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_HEIGHT_BLEND.
    }
    LOCAL FINAL_ALIGN_TERMINAL_LATE_VELOCITY_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_EXTRA_VELOCITY_START_HEIGHT - HOOK_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_EXTRA_VELOCITY_START_HEIGHT
            - FINAL_ALIGN_TERMINAL_EXTRA_VELOCITY_FULL_HEIGHT, 1), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_BLEND IS 1 - CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_END_RATIO
            - FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_RATIO, 0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_START_HEIGHT - HOOK_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_START_HEIGHT
            - FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_FULL_HEIGHT, 1), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_HIGH_ENERGY_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_HIGH_ENERGY_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_HIGH_ENERGY_FULL_RATIO
            - FINAL_ALIGN_TERMINAL_HIGH_ENERGY_RATIO, 0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_START_HEIGHT - HOOK_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_START_HEIGHT
            - FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_FULL_HEIGHT, 1), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_ENTRY_RATIO
            - FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_RATIO)
        / MAX(FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_FULL_RATIO
            - FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_RATIO,
            0.0001), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_ACTIVE_VELOCITY_COEFFICIENT IS 4
        + FINAL_ALIGN_TERMINAL_LOW_RATIO_EXTRA_VELOCITY_COEFFICIENT
            * FINAL_ALIGN_TERMINAL_LOW_FAMILY_BLEND
            * FINAL_ALIGN_TERMINAL_LATE_VELOCITY_BLEND
        + FINAL_ALIGN_TERMINAL_EXTREME_EXTRA_VELOCITY_COEFFICIENT
            * FINAL_ALIGN_TERMINAL_EXTREME_LOW_FAMILY_BLEND
            * FINAL_ALIGN_TERMINAL_EXTREME_VELOCITY_BLEND
        + FINAL_ALIGN_TERMINAL_HIGH_ENERGY_EXTRA_VELOCITY_COEFFICIENT
            * FINAL_ALIGN_TERMINAL_HIGH_ENERGY_BLEND
            * FINAL_ALIGN_TERMINAL_HIGH_ENERGY_VELOCITY_BLEND.
    LOCAL FINAL_ALIGN_FINITE_TIME_START_BLEND IS CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_LOW_RATIO)
        / MAX(FINAL_ALIGN_FINITE_TIME_START_FULL_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_LOW_RATIO, 0.0001), 0, 1).
    LOCAL FINAL_ALIGN_FINITE_TIME_START_HEIGHT IS
        FINAL_ALIGN_TERMINAL_PHASE_HEIGHT
        + (FINAL_ALIGN_HEIGHT - FINAL_ALIGN_TERMINAL_PHASE_HEIGHT)
            * FINAL_ALIGN_FINITE_TIME_START_BLEND.
    LOCAL FINAL_ALIGN_FINITE_TIME_MEDIUM_START_BLEND IS CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_MEDIUM_RATIO)
        / MAX(FINAL_ALIGN_FINITE_TIME_START_MEDIUM_FULL_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_MEDIUM_RATIO, 0.0001), 0, 1).
    SET FINAL_ALIGN_FINITE_TIME_START_HEIGHT TO
        FINAL_ALIGN_FINITE_TIME_START_HEIGHT
        + MAX(FINAL_ALIGN_FINITE_TIME_START_MEDIUM_TARGET_HEIGHT
                - FINAL_ALIGN_FINITE_TIME_START_HEIGHT, 0)
            * FINAL_ALIGN_FINITE_TIME_MEDIUM_START_BLEND.
    LOCAL FINAL_ALIGN_FINITE_TIME_HIGH_START_BLEND IS CLAMP(
        (FINAL_ALIGN_ENTRY_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_HIGH_RATIO)
        / MAX(FINAL_ALIGN_FINITE_TIME_START_HIGH_FULL_RATIO
            - FINAL_ALIGN_FINITE_TIME_START_HIGH_RATIO, 0.0001), 0, 1).
    SET FINAL_ALIGN_FINITE_TIME_START_HEIGHT TO
        FINAL_ALIGN_FINITE_TIME_START_HEIGHT
        + (FINAL_ALIGN_HEIGHT - FINAL_ALIGN_FINITE_TIME_START_HEIGHT)
            * FINAL_ALIGN_FINITE_TIME_HIGH_START_BLEND.
    LOCAL FINAL_ALIGN_TERMINAL_HIGH_START_4KM_POSITION_BIAS IS 0.
    IF LIVE_APPROACH_OFFSET_RANGE_LATCHED {
        SET FINAL_ALIGN_TERMINAL_HIGH_START_4KM_POSITION_BIAS TO CLAMP(
            (FINAL_ALIGN_TERMINAL_HIGH_START_4KM_RANGE_REFERENCE
                - LIVE_APPROACH_OFFSET_RANGE_AT_LATCH)
                * FINAL_ALIGN_TERMINAL_HIGH_START_4KM_RANGE_BIAS_GAIN,
            0,
            FINAL_ALIGN_TERMINAL_HIGH_START_4KM_MAX_POSITION_BIAS)
            * FINAL_ALIGN_TERMINAL_HIGH_START_RANGE_BLEND.
    }
    // The terminal-ratio residual and the full 6 km ownership residual are
    // independent observations of the same endpoint phase error. Select the
    // stronger calibrated estimate rather than adding them.
    LOCAL FINAL_ALIGN_TERMINAL_ACTIVE_POSITION_BIAS IS MAX(
        FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS
            * FINAL_ALIGN_TERMINAL_EXTREME_HIGH_POSITION_BIAS_BLEND,
        FINAL_ALIGN_TERMINAL_EXTREME_HIGH_START_POSITION_BIAS
            * FINAL_ALIGN_FINITE_TIME_HIGH_START_BLEND)
        + FINAL_ALIGN_TERMINAL_HIGH_START_4KM_POSITION_BIAS
        + FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS
        + FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS.
    IF FINAL_ALIGN_MODE AND NOT FINAL_DESCENT_ARMED {
        // The long stage has about a 1.5 s attitude response.  A deliberately
        // slower velocity loop settles that lag while the stage is held above
        // the frame, rather than commanding another full-speed centre pass.
        LOCAL FINAL_STOP_RANGE IS HORIZONTAL_POS:MAG.
        LOCAL FINAL_H_SPEED IS MIN(FINAL_ALIGN_SPEED,
            FINAL_STOP_RANGE * FINAL_ALIGN_ACTIVE_POSITION_GAIN).
        LOCAL FINAL_DESIRED_H_VEL IS V(0,0,0).
        IF HORIZONTAL_POS:MAG > 0.25 {
            SET FINAL_DESIRED_H_VEL TO HORIZONTAL_POS:NORMALIZED
                * FINAL_H_SPEED.
        }
        SET H_ACCEL TO CLAMPV((FINAL_DESIRED_H_VEL - HORIZONTAL_VEL)
            * FINAL_ALIGN_VELOCITY_GAIN,
            TERMINAL_MAX_HORIZONTAL_ACCEL).
        IF HOOK_HEIGHT <= FINAL_ALIGN_FINITE_TIME_START_HEIGHT
            AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
            // Once the physical target owns the existing 6 km final-align
            // mode, solve its position and velocity as one finite-time
            // boundary condition. Run 173 showed that retaining the static
            // phase until 5 km preserved a 33 m entrance-range dispersion
            // that the short terminal actuator horizon could not recover.
            // T follows the already controlled vertical 2 km endpoint, so the
            // stage brakes early enough to preserve stopping distance instead
            // of waiting for the static position-gain line to collapse.  At
            // and below 2 km the bounded settling law above resumes.
            SET FINAL_ALIGN_TERMINAL_TGO TO 2
                * (HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT)
                / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
            SET FINAL_ALIGN_TERMINAL_CONTROL_TGO TO MAX(
                FINAL_ALIGN_TERMINAL_TGO
                    - FINAL_ALIGN_TERMINAL_ACTIVE_RESPONSE_LEAD_SECONDS, 0.5).
            SET FINAL_ALIGN_TERMINAL_CONTROL_POS TO HORIZONTAL_POS
                - CONTROL_ALONG_DIRECTION
                    * FINAL_ALIGN_TERMINAL_ACTIVE_POSITION_BIAS
                - HORIZONTAL_VEL
                    * FINAL_ALIGN_TERMINAL_ACTIVE_RESPONSE_LEAD_SECONDS.
            SET H_ACCEL TO FINAL_ALIGN_TERMINAL_CONTROL_POS
                    * (6 / MAX(
                        FINAL_ALIGN_TERMINAL_CONTROL_TGO^2, 0.25))
                - HORIZONTAL_VEL
                    * (FINAL_ALIGN_TERMINAL_ACTIVE_VELOCITY_COEFFICIENT
                        / FINAL_ALIGN_TERMINAL_CONTROL_TGO).
            SET H_ACCEL TO CLAMPV(H_ACCEL,
                TERMINAL_MAX_HORIZONTAL_ACCEL).
            SET FINAL_ALIGN_TERMINAL_FINITE_TIME_ACTIVE TO TRUE.
        }
    }

    // Before the strict one-way gate, replace the collapsing finite-time
    // horizon with a bounded approach once the measured Run 189 neighbourhood
    // is reached. This remains reversible if disturbance carries the stage
    // outside 24 m, and it cannot expose post-commit vertical authority.
    // Use the same physical thrust axis as projection, telemetry, and the
    // strict commitment gate. Range/speed can converge before attitude lag
    // does; settling during that lag rebuilt the Run 196 reverse pass.
    LOCAL STAGE_TILT IS VANG(SHIP:FACING:VECTOR, UP_VEC).
    IF FINAL_ALIGN_MODE AND NOT FINAL_DESCENT_ARMED
        AND NOT FINAL_ALIGN_PRECOMMIT_SETTLE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND HORIZONTAL_POS:MAG <= FINAL_ALIGN_PRECOMMIT_SETTLE_RANGE
        AND VDOT(HORIZONTAL_POS, CONTROL_ALONG_DIRECTION)
            >= FINAL_ALIGN_PRECOMMIT_SETTLE_MIN_RANGE
        AND HORIZONTAL_VEL:MAG <= FINAL_ALIGN_PRECOMMIT_SETTLE_SPEED
        AND STAGE_TILT <= FINAL_ALIGN_PRECOMMIT_SETTLE_TILT {
        SET FINAL_ALIGN_PRECOMMIT_SETTLE TO TRUE.
        SET FILTERED_H_ACCEL TO V(0,0,0).
    }
    IF FINAL_ALIGN_PRECOMMIT_SETTLE AND NOT FINAL_DESCENT_ARMED
        AND HORIZONTAL_POS:MAG
            > FINAL_ALIGN_PRECOMMIT_REACQUIRE_RANGE {
        SET FINAL_ALIGN_PRECOMMIT_SETTLE TO FALSE.
        SET FILTERED_H_ACCEL TO V(0,0,0).
    }
    IF FINAL_ALIGN_PRECOMMIT_SETTLE AND NOT FINAL_DESCENT_ARMED {
        LOCAL PRECOMMIT_DESIRED_H_VEL IS V(0,0,0).
        IF HORIZONTAL_POS:MAG > FINAL_CAPTURE_POSITION_DEADBAND {
            SET PRECOMMIT_DESIRED_H_VEL TO HORIZONTAL_POS:NORMALIZED
                * MIN(FINAL_ALIGN_PRECOMMIT_MAX_SPEED,
                    (HORIZONTAL_POS:MAG - FINAL_CAPTURE_POSITION_DEADBAND)
                        * FINAL_CAPTURE_POSITION_GAIN).
        }
        SET H_ACCEL TO CLAMPV((PRECOMMIT_DESIRED_H_VEL - HORIZONTAL_VEL)
            * FINAL_CAPTURE_VELOCITY_GAIN,
            FINAL_ALIGN_PRECOMMIT_MAX_ACCEL).
    }

    // The strict one-way gate uses that same physical thrust axis. FOREVECTOR
    // can already appear upright while the lagging axis still carries lateral
    // impulse.
    IF FINAL_ALIGN_MODE AND NOT FINAL_DESCENT_ARMED
        AND HORIZONTAL_POS:MAG <= FINAL_ALIGN_READY_ERROR
        AND HORIZONTAL_VEL:MAG <= FINAL_ALIGN_READY_SPEED
        AND STAGE_TILT <= FINAL_ALIGN_READY_TILT {
        // From here to capture, position error is intentionally not chased.
        // Only residual velocity is damped, so the controller cannot reverse
        // into another centre-seeking pass after crossing the aim point.
        SET FINAL_DESCENT_ARMED TO TRUE.
        SET FILTERED_H_ACCEL TO V(0,0,0).
        PRINT "VERTICAL CAPTURE COMMITTED" AT(0,11).
    }

    // Run 252's fixed roll target did not prevent the physical thrust axis
    // from diverging below the net. Run 254 then measured about 17 deg/s
    // transverse rate versus only 0.6 deg/s axial roll. Reduce the cooked
    // steering angular-rate ceiling before that measured surface without
    // changing the torque-loop settling times or any translation command.
    IF FINAL_DESCENT_ARMED
        AND NOT FINAL_CAPTURE_NEAR_NET_STEERING_TUNED
        AND HOOK_HEIGHT <= FINAL_CAPTURE_NEAR_NET_STEERING_HEIGHT {
        SET STEERINGMANAGER:MAXSTOPPINGTIME TO
            FINAL_CAPTURE_NEAR_NET_MAX_STOPPING_TIME.
        STEERINGMANAGER:RESETPIDS().
        SET FINAL_CAPTURE_NEAR_NET_STEERING_TUNED TO TRUE.
        LOG MISSION_ID + ",FINAL_CAPTURE_NEAR_NET_STEERING,"
            + ROUND(NOW,3)
            + ",h=" + ROUND(HOOK_HEIGHT,2)
            + ",axialRate=" + ROUND(VDOT(
                SHIP:ANGULARVEL, SHIP:FACING:VECTOR)
                * CONSTANT:RADTODEG,3)
            + ",transverseRate=" + ROUND(VXCL(
                SHIP:FACING:VECTOR, SHIP:ANGULARVEL):MAG
                * CONSTANT:RADTODEG,3)
            TO "0:/cz10b/telemetry.csv".
    }

    // Run 240 made a legal commitment, but the 1.5 km direct-control handoff
    // retained a -4.75 m/s2 horizontal aero estimate while the live raw sample
    // was only -0.62 m/s2. Subtracting that obsolete state turned a bounded
    // targetward request into +3.77 m/s2 of target-away engine acceleration.
    // Clear only this horizontal estimator state on the first committed direct
    // frame. The vertical estimate and every pre-formal frame remain intact;
    // the normal aero filter resumes from zero on the following tick.
    IF FINAL_DESCENT_ARMED AND NOT FINAL_CAPTURE_DIRECT_AERO_RESET
        AND HOOK_HEIGHT <= FINAL_CAPTURE_DIRECT_CONTROL_HEIGHT {
        LOG MISSION_ID + ",FINAL_CAPTURE_AERO_RESET,"
            + ROUND(NOW,3)
            + ",h=" + ROUND(HOOK_HEIGHT,2)
            + ",filteredAlong=" + ROUND(VDOT(
                POWERED_FILTERED_HORIZONTAL_AERO_ACCEL,
                CONTROL_ALONG_DIRECTION),3)
            + ",rawAlong=" + ROUND(VDOT(
                POWERED_HORIZONTAL_AERO_RAW_SAMPLE,
                CONTROL_ALONG_DIRECTION),3)
            TO "0:/cz10b/telemetry.csv".
        SET POWERED_FILTERED_HORIZONTAL_AERO_ACCEL TO V(0,0,0).
        SET FINAL_CAPTURE_DIRECT_AERO_RESET TO TRUE.
    }

    IF PID_MODE {
        LOCAL PID_TARGET_V IS -MAX(CAPTURE_FINAL_SPEED,
            MIN(8, MAX(HOOK_HEIGHT, 0) * 0.04)).
        // Cable closure itself still gets a short stationary window so the stage
        // cannot cross open lines while fuel permits. The moving cradle tracks
        // lateral error; a miss must never turn into a fuel-exhausting hover.
        IF WIRE_REQUIRED AND NOT FUEL_URGENT { SET PID_TARGET_V TO 0. }
        IF WIRE_HOLD_STARTED_AT >= 0 AND NOT WIRE_REQUIRED {
            SET PID_TARGET_V TO -MAX(POST_WIRE_CROSSING_SPEED,
                -PID_TARGET_V).
        }
        // This is deliberately the final override: transient Closed/Tracking state
        // changes must not cancel the decisive low-fuel crossing of the net plane.
        IF FUEL_URGENT {
            SET PID_TARGET_V TO -MAX(TERMINAL_LOW_FUEL_CAPTURE_SPEED,
                -PID_TARGET_V).
        }
        SET VERTICAL_INTEGRAL TO CLAMP(VERTICAL_INTEGRAL
            + (PID_TARGET_V - VERTICAL_V) * DT, -10, 10).
        SET VERTICAL_THRUST_CMD TO G_ACC
            + V_VEL_KP * (PID_TARGET_V - VERTICAL_V)
            + V_VEL_KI * VERTICAL_INTEGRAL.
    }
    IF FINAL_ALIGN_MODE AND NOT FINAL_DESCENT_ARMED
        AND HOOK_HEIGHT <= FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT {
        // Horizontal settling begins higher, but vertical slowdown remains a
        // near-field action.  A real-flight trace showed that extending the
        // slow descent itself would consume the remaining propellant above the
        // cables. FINAL_ALIGN_HOLD_SECONDS remains configurable for test
        // articles, but the flight value is zero so the stage never trades its
        // landing reserve for a hover.
        LOCAL FINAL_TARGET_V IS 0.
        // Low fuel must never be traded for a stationary final two-metre
        // correction. Keep aligning during a capture-envelope-safe descent.
        IF FUEL_URGENT {
            SET FINAL_TARGET_V TO -TERMINAL_LOW_FUEL_CAPTURE_SPEED.
        } ELSE IF NOW - FINAL_ALIGN_STARTED_AT >= FINAL_ALIGN_HOLD_SECONDS {
            SET FINAL_TARGET_V TO -POST_WIRE_CROSSING_SPEED.
        }
        SET VERTICAL_INTEGRAL TO CLAMP(VERTICAL_INTEGRAL
            + (FINAL_TARGET_V - VERTICAL_V) * DT, -10, 10).
        SET VERTICAL_THRUST_CMD TO G_ACC
            + V_VEL_KP * (FINAL_TARGET_V - VERTICAL_V)
            + V_VEL_KI * VERTICAL_INTEGRAL.
    }
    IF FINAL_DESCENT_ARMED AND FUEL_URGENT AND NOT PID_MODE
        AND HOOK_HEIGHT <= FINAL_ALIGN_VERTICAL_CONTROL_HEIGHT {
        // Once the horizontal capture commitment is safe, do not let the
        // height corridor slow the stage below the decisive low-fuel crossing
        // speed.  A real flight decelerated from 3.85 to 2.30 m/s above the
        // 18 m PID handover, then spent its last propellant accelerating back
        // to 6 m/s and free-fell through the net at 13.78 m/s.  Carrying the
        // same envelope-safe target continuously removes that waste; PID_MODE
        // takes over with the identical target below its normal handover.
        LOCAL COMMITTED_FUEL_TARGET_V IS
            -TERMINAL_LOW_FUEL_CAPTURE_SPEED.
        SET VERTICAL_INTEGRAL TO CLAMP(VERTICAL_INTEGRAL
            + (COMMITTED_FUEL_TARGET_V - VERTICAL_V) * DT, -10, 10).
        SET VERTICAL_THRUST_CMD TO G_ACC
            + V_VEL_KP * (COMMITTED_FUEL_TARGET_V - VERTICAL_V)
            + V_VEL_KI * VERTICAL_INTEGRAL.
    }
    IF FINAL_DESCENT_ARMED {
        // Vertical capture is a one-way mode. The position-field structure is
        // retained for explicit A/B tests, but the flight speed cap is zero:
        // Run 141's 0.75 m/s pursuit produced a delayed 50 m pass. Run 177's
        // residual drift is handled by stronger bounded velocity damping.
        LOCAL FINAL_CAPTURE_DESIRED_H_VEL IS V(0,0,0).
        LOCAL FINAL_CAPTURE_POSITION_RANGE IS HORIZONTAL_POS:MAG.
        IF FINAL_CAPTURE_POSITION_RANGE > FINAL_CAPTURE_POSITION_DEADBAND {
            LOCAL FINAL_CAPTURE_SPEED IS MIN(FINAL_CAPTURE_MAX_SPEED,
                (FINAL_CAPTURE_POSITION_RANGE
                    - FINAL_CAPTURE_POSITION_DEADBAND)
                * FINAL_CAPTURE_POSITION_GAIN).
            SET FINAL_CAPTURE_DESIRED_H_VEL TO
                HORIZONTAL_POS:NORMALIZED * FINAL_CAPTURE_SPEED.
        }
        SET H_ACCEL TO CLAMPV((FINAL_CAPTURE_DESIRED_H_VEL
            - HORIZONTAL_VEL) * FINAL_CAPTURE_VELOCITY_GAIN,
            FINAL_CAPTURE_MAX_ACCEL).
    }
    // Final cross-axis arbitration before aerodynamic compensation.  The
    // endpoint controller used a rotating radial basis and did not request
    // braking until the physical stage could no longer stop before the centre.
    // Freeze the first deficient cross direction above 12 km, then enforce
    // v^2/(2r) only while both signed range and closing speed remain positive.
    // Once either gate ends, remove the rotating controller's cross component:
    // a failed stopping model must not be hidden by a low-altitude return pass.
    IF TERMINAL_MAIN_CROSS_STOP_ENABLED
        AND HOOK_HEIGHT > PLAN_HORIZONTAL_END_HEIGHT {
        LOCAL CROSS_STOP_CANDIDATE_POS IS WAYPOINT_CONTROL_H_POS
            - CONTROL_ALONG_DIRECTION * VDOT(
                WAYPOINT_CONTROL_H_POS, CONTROL_ALONG_DIRECTION).
        IF NOT CROSS_STOP_COMMITTED
            AND HOOK_HEIGHT <= TERMINAL_MAIN_CROSS_STOP_ARM_HEIGHT
            AND NOT HIGH_ENERGY_BRAKE_MODE
            AND CROSS_STOP_CANDIDATE_POS:MAG
                > TERMINAL_HORIZONTAL_DEADBAND {
            LOCAL CROSS_STOP_CANDIDATE_AXIS IS
                CROSS_STOP_CANDIDATE_POS:NORMALIZED.
            LOCAL CROSS_STOP_CANDIDATE_VEL IS VDOT(
                HORIZONTAL_VEL, CROSS_STOP_CANDIDATE_AXIS).
            IF CROSS_STOP_CANDIDATE_VEL
                    > TERMINAL_MAIN_CROSS_STOP_MIN_SPEED {
                LOCAL CROSS_STOP_CANDIDATE_USABLE IS MAX(
                    CROSS_STOP_CANDIDATE_POS:MAG
                        - CROSS_STOP_CANDIDATE_VEL
                            * TERMINAL_MAIN_CROSS_STOP_RESPONSE_SECONDS
                        - TERMINAL_HORIZONTAL_DEADBAND,
                    TERMINAL_MAIN_CROSS_STOP_MIN_RANGE).
                LOCAL CROSS_STOP_CANDIDATE_REQUIRED IS MIN(
                    TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL,
                    CROSS_STOP_CANDIDATE_VEL^2
                        / (2 * CROSS_STOP_CANDIDATE_USABLE)
                        * TERMINAL_MAIN_CROSS_STOP_GAIN).
                IF VDOT(H_ACCEL, CROSS_STOP_CANDIDATE_AXIS)
                        > -CROSS_STOP_CANDIDATE_REQUIRED {
                    SET CROSS_STOP_COMMITTED TO TRUE.
                    SET CROSS_STOP_COMMITTED_THIS_TICK TO TRUE.
                    SET CROSS_STOP_DIRECTION TO
                        CROSS_STOP_CANDIDATE_AXIS.
                    LOG MISSION_ID + ",CROSS_STOP_COMMIT,"
                        + ROUND(NOW,3)
                        + ",h=" + ROUND(HOOK_HEIGHT,2)
                        + ",pos="
                            + ROUND(CROSS_STOP_CANDIDATE_POS:MAG,2)
                        + ",vel="
                            + ROUND(CROSS_STOP_CANDIDATE_VEL,2)
                        + ",required="
                            + ROUND(CROSS_STOP_CANDIDATE_REQUIRED,2)
                        TO "0:/cz10b/telemetry.csv".
                }
            }
        }
        IF CROSS_STOP_COMMITTED {
            // Parallel-transport the frozen surface axis into the current
            // tangent plane; over this segment Kerbin curvature is small but
            // leaving a vertical residue would corrupt both range and thrust.
            LOCAL CROSS_STOP_AXIS IS VXCL(UP_VEC,
                CROSS_STOP_DIRECTION).
            IF CROSS_STOP_AXIS:MAG > 0.001 {
                SET CROSS_STOP_AXIS TO CROSS_STOP_AXIS:NORMALIZED.
                SET CROSS_STOP_POS_DIAG TO VDOT(
                    WAYPOINT_CONTROL_H_POS, CROSS_STOP_AXIS).
                SET CROSS_STOP_VEL_DIAG TO VDOT(
                    HORIZONTAL_VEL, CROSS_STOP_AXIS).
                LOCAL CROSS_STOP_CURRENT_ACCEL IS VDOT(
                    H_ACCEL, CROSS_STOP_AXIS).
                IF CROSS_STOP_COMPLETED {
                    SET CROSS_STOP_SUPPRESS_ACTIVE TO TRUE.
                    SET CROSS_STOP_FINISH_ACTIVE TO TRUE.
                    SET CROSS_STOP_FINISH_ACCEL_DIAG TO CLAMP(
                        -CROSS_STOP_VEL_DIAG
                            * TERMINAL_MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN,
                        -TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL,
                        TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL).
                    SET H_ACCEL TO H_ACCEL
                        - CROSS_STOP_AXIS * CROSS_STOP_CURRENT_ACCEL
                        + CROSS_STOP_AXIS
                            * CROSS_STOP_FINISH_ACCEL_DIAG.
                } ELSE IF CROSS_STOP_VEL_DIAG > 0.1 {
                    SET CROSS_STOP_ACTIVE TO TRUE.
                    IF CROSS_STOP_POS_DIAG
                            > TERMINAL_HORIZONTAL_DEADBAND {
                        SET CROSS_STOP_USABLE_RANGE_DIAG TO MAX(
                            CROSS_STOP_POS_DIAG
                                - CROSS_STOP_VEL_DIAG
                                    * TERMINAL_MAIN_CROSS_STOP_RESPONSE_SECONDS
                                - TERMINAL_HORIZONTAL_DEADBAND,
                            TERMINAL_MAIN_CROSS_STOP_MIN_RANGE).
                        SET CROSS_STOP_REQUIRED_DECEL_DIAG TO MIN(
                            TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL,
                            CROSS_STOP_VEL_DIAG^2
                                / (2 * CROSS_STOP_USABLE_RANGE_DIAG)
                                * TERMINAL_MAIN_CROSS_STOP_GAIN).
                    } ELSE {
                        // The first pass has entered the positional deadband,
                        // but it is not complete until frozen-axis momentum is
                        // removed.  This term never looks at position and thus
                        // cannot command a second centre-seeking pass.
                        SET CROSS_STOP_FINISH_ACTIVE TO TRUE.
                        SET CROSS_STOP_REQUIRED_DECEL_DIAG TO MIN(
                            TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL,
                            CROSS_STOP_VEL_DIAG
                                * TERMINAL_MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN).
                        SET CROSS_STOP_FINISH_ACCEL_DIAG TO
                            -CROSS_STOP_REQUIRED_DECEL_DIAG.
                    }
                    IF CROSS_STOP_CURRENT_ACCEL
                            > -CROSS_STOP_REQUIRED_DECEL_DIAG {
                        SET H_ACCEL TO H_ACCEL
                            - CROSS_STOP_AXIS
                                * (CROSS_STOP_CURRENT_ACCEL
                                    + CROSS_STOP_REQUIRED_DECEL_DIAG).
                        SET H_ACCEL TO CLAMPV(H_ACCEL,
                            TERMINAL_MAX_HORIZONTAL_ACCEL).
                    }
                } ELSE {
                    SET CROSS_STOP_COMPLETED TO TRUE.
                    SET CROSS_STOP_SUPPRESS_ACTIVE TO TRUE.
                    SET CROSS_STOP_FINISH_ACTIVE TO TRUE.
                    SET CROSS_STOP_FINISH_ACCEL_DIAG TO CLAMP(
                        -CROSS_STOP_VEL_DIAG
                            * TERMINAL_MAIN_CROSS_STOP_FINISH_VELOCITY_GAIN,
                        -TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL,
                        TERMINAL_MAIN_CROSS_STOP_MAX_ACCEL).
                    // Completion is sticky. Continue damping velocity in both
                    // signs, but never restore a position term or re-arm the
                    // aerodynamic first-pass stop.
                    SET H_ACCEL TO H_ACCEL
                        - CROSS_STOP_AXIS * CROSS_STOP_CURRENT_ACCEL
                        + CROSS_STOP_AXIS
                            * CROSS_STOP_FINISH_ACCEL_DIAG.
                }
            }
        }
    }
    // Above 6 km the corrected hybrid tube owns the two longitudinal state
    // coordinates independently of ship placement.  Position/cross-track
    // guidance remains intact, while height derivatives provide the nominal
    // net accelerations and bounded residual feedback absorbs force error.
    IF HYBRID_CORRIDOR_ACTIVE
        AND CONTROL_ALONG_DIRECTION:MAG > 0.001 {
        LOCAL HYBRID_CURRENT_ALONG_ACCEL IS VDOT(
            H_ACCEL, CONTROL_ALONG_DIRECTION).
        LOCAL HYBRID_CROSS_ACCEL IS H_ACCEL
            - CONTROL_ALONG_DIRECTION * HYBRID_CURRENT_ALONG_ACCEL.
        LOCAL HYBRID_HORIZONTAL_FEEDBACK IS CLAMP(
            -HYBRID_HORIZONTAL_RESIDUAL
                * HYBRID_CORRIDOR_HORIZONTAL_GAIN,
            -HYBRID_CORRIDOR_MAX_FEEDBACK_ACCEL,
            HYBRID_CORRIDOR_MAX_FEEDBACK_ACCEL).
        LOCAL HYBRID_TARGET_ALONG_ACCEL IS
            HYBRID_CORRIDOR_HORIZONTAL_SLOPE(HOOK_HEIGHT)
                * VERTICAL_V
            + HYBRID_HORIZONTAL_FEEDBACK.
        SET H_ACCEL TO HYBRID_CROSS_ACCEL
            + CONTROL_ALONG_DIRECTION * HYBRID_TARGET_ALONG_ACCEL.

        LOCAL HYBRID_DOWN_FEEDBACK IS CLAMP(
            HYBRID_DOWN_RESIDUAL * HYBRID_CORRIDOR_DOWN_GAIN,
            -HYBRID_CORRIDOR_MAX_FEEDBACK_ACCEL,
            HYBRID_CORRIDOR_MAX_FEEDBACK_ACCEL).
        LOCAL HYBRID_TARGET_VERTICAL_NET_ACCEL IS
            -HYBRID_CORRIDOR_DOWN_SLOPE(HOOK_HEIGHT) * VERTICAL_V
            + HYBRID_DOWN_FEEDBACK.
        SET VERTICAL_THRUST_CMD TO G_ACC
            + HYBRID_TARGET_VERTICAL_NET_ACCEL.
    }

    // All guidance laws above specify desired *net* acceleration.  Convert it
    // to an engine request by cancelling the identified aerodynamic force.
    // This is the powered half of the documented hybrid model: the ballistic
    // predictor supplies the nominal path and the measured residual closes the
    // KSP drag/side-force mismatch without replanning or PWM.
    SET H_ACCEL TO H_ACCEL - POWERED_FILTERED_HORIZONTAL_AERO_ACCEL.
    SET VERTICAL_THRUST_CMD TO VERTICAL_THRUST_CMD
        - POWERED_FILTERED_VERTICAL_AERO_ACCEL.
    LOCAL CURRENT_ACCEL_FILTER IS TERMINAL_ACCEL_FILTER.
    IF H_CORRIDOR_MODE AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET CURRENT_ACCEL_FILTER TO TERMINAL_HIGH_ENERGY_ACCEL_FILTER.
    }
    // Above 500 m the slow dense-air handoff is necessary: immediate direct
    // cancellation drove the stage to 23 degrees and failed formal speed in
    // Run 152. Below that measured surface, remove the second command-memory
    // pole so Run 151's stored request cannot survive another velocity sign
    // change. Run 240 showed that the aerodynamic estimator carried a second
    // obsolete state; its one-time committed transition is cleared above.
    IF FINAL_DESCENT_ARMED
        AND HOOK_HEIGHT <= FINAL_CAPTURE_DIRECT_CONTROL_HEIGHT {
        SET CURRENT_ACCEL_FILTER TO 1.
    }
    IF FIXED_STOP_COMMITTED_THIS_TICK {
        // Commitment is a one-way mode transition. Do not carry the preceding
        // unavailable-prograde command into the finite stopping footprint.
        // H_ACCEL is already drag-compensated here, so this resets the engine
        // request rather than mixing net and thrust accelerations.
        SET FILTERED_H_ACCEL TO H_ACCEL.
    } ELSE {
        SET FILTERED_H_ACCEL TO FILTERED_H_ACCEL
            * (1 - CURRENT_ACCEL_FILTER)
            + H_ACCEL * CURRENT_ACCEL_FILTER.
    }
    SET H_ACCEL TO FILTERED_H_ACCEL.

    // Run 52 retained the required vertical timeline but realised too little
    // downrange braking during its final cone-edge slew.  Reserve a bounded
    // engine component on the downrange axis frozen at main ignition only
    // below the configured late arm height.
    // Applying this after drag compensation and filtering makes the value an
    // engine request, while preserving both the vertical command and the
    // independently controlled cross-track component.  The positive range and
    // velocity gates make this strictly one-way: after centre crossing it
    // cannot turn into a pursuit command.
    LOCAL MAIN_ENGINE_BRAKE_ACTIVE IS FALSE.
    IF TERMINAL_MAIN_ALONG_ENGINE_BRAKE_ENABLED
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND HOOK_HEIGHT <= TERMINAL_MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT {
        LOCAL MAIN_ENGINE_BRAKE_ALONG_POS IS VDOT(
            WAYPOINT_CONTROL_H_POS, CONTROL_ALONG_DIRECTION).
        LOCAL MAIN_ENGINE_BRAKE_ALONG_VEL IS VDOT(
            HORIZONTAL_VEL, CONTROL_ALONG_DIRECTION).
        IF MAIN_ENGINE_BRAKE_ALONG_POS > TERMINAL_HORIZONTAL_DEADBAND
            AND MAIN_ENGINE_BRAKE_ALONG_VEL > 0.1 {
            SET MAIN_ENGINE_BRAKE_ACTIVE TO TRUE.
            LOCAL MAIN_ENGINE_BRAKE_BLEND IS CLAMP(
                (TERMINAL_MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT
                    - HOOK_HEIGHT)
                / MAX(TERMINAL_MAIN_ALONG_ENGINE_BRAKE_START_HEIGHT
                    - TERMINAL_MAIN_ALONG_ENGINE_BRAKE_FULL_HEIGHT, 1),
                0, 1).
            LOCAL MAIN_ENGINE_BRAKE_REQUIRED IS
                TERMINAL_MAIN_ALONG_ENGINE_BRAKE_ACCEL
                    * MAIN_ENGINE_BRAKE_BLEND.
            LOCAL MAIN_ENGINE_BRAKE_CURRENT IS VDOT(
                H_ACCEL, CONTROL_ALONG_DIRECTION).
            IF MAIN_ENGINE_BRAKE_CURRENT > -MAIN_ENGINE_BRAKE_REQUIRED {
                SET H_ACCEL TO H_ACCEL - CONTROL_ALONG_DIRECTION
                    * (MAIN_ENGINE_BRAKE_CURRENT
                        + MAIN_ENGINE_BRAKE_REQUIRED).
            }
        }
    }
    // The 75% main-burn ceiling is an upstream trajectory allocation, not a
    // landing limit.  Run 178 reached the 2 km state legally, but the same
    // ceiling then held vertical thrust at about 16.1 m/s2 all the way to the
    // water even though the energy corridor requested more braking.  After a
    // legal one-way capture commitment and below the formal ASL plane, expose
    // full continuous authority; no target, horizontal law, or waypoint state
    // changes here.
    LOCAL TERMINAL_VERTICAL_THRUST_FRACTION IS
        TERMINAL_NOMINAL_THRUST_FRACTION.
    IF FINAL_DESCENT_ARMED
        AND SHIP:ALTITUDE < TERMINAL_WAYPOINT_HEIGHT {
        SET TERMINAL_VERTICAL_THRUST_FRACTION TO 1.
    }
    SET VERTICAL_THRUST_CMD TO CLAMP(VERTICAL_THRUST_CMD, 0,
        AVAILABLE_ACC * TERMINAL_VERTICAL_THRUST_FRACTION).

    // Keep full entry authority through the 2 km waypoint, then taper it
    // smoothly to the 12-degree landing cone by 500 m.  The former 800 m
    // blend still allowed 29 degrees at 239 m and produced a measured
    // 16 deg/s attitude spike during a small centre correction.
    LOCAL ALT_BLEND IS CLAMP((HOOK_HEIGHT - 500) / 1500, 0, 1).
    LOCAL TILT_LIMIT IS ENTRY_MAX_TILT * ALT_BLEND
        + LANDING_MAX_TILT * (1 - ALT_BLEND).
    LOCAL ACTIVE_COMMAND_CONE_BLEND IS CLAMP(
        (HOOK_HEIGHT - TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT)
        / MAX(TERMINAL_WAYPOINT_HEIGHT
            - TERMINAL_LOW_ALT_COMMAND_CONE_END_HEIGHT, 1), 0, 1).
    LOCAL ACTIVE_COMMAND_CONE_DEGREES IS
        TERMINAL_LOW_ALT_COMMAND_CONE_DEGREES
        + (TERMINAL_COMMAND_CONE_DEGREES
            - TERMINAL_LOW_ALT_COMMAND_CONE_DEGREES)
            * ACTIVE_COMMAND_CONE_BLEND.
    // The formal load limit is unchanged.  This earlier, smooth schedule is a
    // trajectory allocation change: reduce angle-of-attack lift in dense air
    // before the long stage accumulates an unrecoverable upward impulse.
    LOCAL DENSE_AIR_COMMAND_CONE_BLEND IS CLAMP(
        (HOOK_HEIGHT - TERMINAL_DENSE_AIR_CONE_END_HEIGHT)
        / MAX(TERMINAL_DENSE_AIR_CONE_START_HEIGHT
            - TERMINAL_DENSE_AIR_CONE_END_HEIGHT, 1), 0, 1).
    LOCAL DENSE_AIR_COMMAND_CONE_DEGREES IS
        TERMINAL_DENSE_AIR_COMMAND_CONE_DEGREES
        + (TERMINAL_COMMAND_CONE_DEGREES
            - TERMINAL_DENSE_AIR_COMMAND_CONE_DEGREES)
            * DENSE_AIR_COMMAND_CONE_BLEND.
    SET ACTIVE_COMMAND_CONE_DEGREES TO MIN(
        ACTIVE_COMMAND_CONE_DEGREES,
        DENSE_AIR_COMMAND_CONE_DEGREES).
    // Preserve Run 107's identified low-lift branch through the upper dense
    // segment, then expose additional continuous aerodynamic-brake authority
    // only where Run 109's measured stopping demand became infeasible.  This
    // is an actuator envelope; the force-feedback allocator still owns the
    // actual direction, and the formal physical cone remains 30 degrees.
    LOCAL FINAL_AERO_BRAKE_CONE_BLEND IS CLAMP(
        (TERMINAL_FINAL_AERO_BRAKE_CONE_START_HEIGHT - HOOK_HEIGHT)
        / MAX(TERMINAL_FINAL_AERO_BRAKE_CONE_START_HEIGHT
            - TERMINAL_FINAL_AERO_BRAKE_CONE_FULL_HEIGHT, 1), 0, 1).
    LOCAL FINAL_AERO_BRAKE_CONE_RELEASE_BLEND IS CLAMP(
        (HOOK_HEIGHT - TERMINAL_FINAL_AERO_BRAKE_CONE_RELEASE_END_HEIGHT)
        / MAX(TERMINAL_WAYPOINT_HEIGHT
            - TERMINAL_FINAL_AERO_BRAKE_CONE_RELEASE_END_HEIGHT, 1), 0, 1).
    SET FINAL_AERO_BRAKE_CONE_BLEND TO MIN(
        FINAL_AERO_BRAKE_CONE_BLEND,
        FINAL_AERO_BRAKE_CONE_RELEASE_BLEND).
    SET ACTIVE_COMMAND_CONE_DEGREES TO ACTIVE_COMMAND_CONE_DEGREES
        + (TERMINAL_FINAL_AERO_BRAKE_CONE_DEGREES
            - ACTIVE_COMMAND_CONE_DEGREES)
            * FINAL_AERO_BRAKE_CONE_BLEND.
    // Horizontal braking and tilt are coupled.  When the vertical descent is
    // already riding its speed corridor, its standalone controller can ask
    // for only a few m/s^2 upward.  Using that small number directly in
    // a_h <= a_v*tan(tilt) reduced a requested 55 m/s^2 stop to about 5.4.
    // Add the vertical component required to realise the allowed thrust angle
    // only while substantial translation remains.  Inside the last 500 m of
    // height, raising vertical thrust to satisfy a lateral request can turn a
    // small correction into an upward hop; there we cap lateral acceleration
    // against the existing vertical command instead.  The 75% nominal cap
    // above remains the hard main-braking limit.
    IF H_CORRIDOR_MODE AND H_ACCEL:MAG > 0.01
        AND HOOK_HEIGHT > 500
        AND (HOOK_HEIGHT < TERMINAL_WAYPOINT_HEIGHT
            OR VERTICAL_V <= -TERMINAL_WAYPOINT_MIN_VERTICAL_SPEED) {
        LOCAL REQUIRED_VERTICAL_FOR_TILT IS H_ACCEL:MAG
            / MAX(TAN(TILT_LIMIT), 0.01).
        LOCAL COUPLED_VERTICAL_THRUST IS VERTICAL_THRUST_CMD.
        IF HOOK_HEIGHT >= TERMINAL_WAYPOINT_HEIGHT {
            // At the 89-degree direct-control limit, realising the full
            // 55 m/s^2 lateral command needs only about 0.96 m/s^2 vertically.
            // This decouples the stop from vertical braking below the 300 m/s
            // load-cone handoff; the old 80-degree vector cancelled nearly one
            // gravity and slowed the stage to roughly 110 m/s at the waypoint.
            SET COUPLED_VERTICAL_THRUST TO MAX(VERTICAL_THRUST_CMD,
                REQUIRED_VERTICAL_FOR_TILT).
        } ELSE {
            // Below the waypoint, fade the extra component away as descent
            // approaches 190 m/s. This prevents a small final correction from
            // producing an upward hop.
            LOCAL COUPLING_MIN_DOWN_SPEED IS
                TERMINAL_WAYPOINT_VERTICAL_SPEED.
            LOCAL COUPLING_BLEND IS CLAMP((-VERTICAL_V
                - COUPLING_MIN_DOWN_SPEED)
                / TERMINAL_DESCENT_COUPLING_BAND, 0, 1).
            SET COUPLED_VERTICAL_THRUST TO VERTICAL_THRUST_CMD
                + (MAX(VERTICAL_THRUST_CMD, REQUIRED_VERTICAL_FOR_TILT)
                    - VERTICAL_THRUST_CMD) * COUPLING_BLEND.
        }
        SET VERTICAL_THRUST_CMD TO MIN(COUPLED_VERTICAL_THRUST,
            AVAILABLE_ACC * TERMINAL_VERTICAL_THRUST_FRACTION).
    }
    LOCAL MAX_H_ACCEL IS MAX(VERTICAL_THRUST_CMD, G_ACC * 0.2)
        * TAN(TILT_LIMIT).
    SET H_ACCEL TO CLAMPV(H_ACCEL, MAX_H_ACCEL).
    LOCAL DESIRED_THRUST IS UP_VEC * MAX(VERTICAL_THRUST_CMD, G_ACC * 0.2)
        + H_ACCEL.
    LOCAL TERMINAL_THRUST_LIMIT_FRACTION IS
        TERMINAL_TOTAL_THRUST_FRACTION.
    SET DESIRED_THRUST TO CLAMPV(DESIRED_THRUST,
        AVAILABLE_ACC * TERMINAL_THRUST_LIMIT_FRACTION).
    LOCAL STEERING_THRUST IS DESIRED_THRUST.
    LOCAL STEERING_ROLL_REFERENCE IS SHIP:FACING:TOPVECTOR.
    IF WAYPOINT_FINAL_COAST_MODE OR FINAL_DESCENT_ARMED {
        // A fixed horizontal reference lets the steering manager remove roll.
        // Reusing the stage's instantaneous top vector made the target rotate
        // with the vehicle and preserved a 40-50 deg/s spin below 1 km.
        // Run 249 reproduced that exact failure after strict commitment:
        // commanded cone was zero, but the omitted fixed reference let angular
        // rate reach 41 deg/s and violated the physical nozzle cone near the
        // net. Share this already validated roll reference with both terminal
        // upright modes; desired thrust and every translation limit are
        // unchanged.
        SET STEERING_ROLL_REFERENCE TO
            RECOVERY_SHIP:FACING:STARVECTOR.
    }
    LOCAL STEERING_SURFACE_SPEED IS SHIP:VELOCITY:SURFACE:MAG.
    // Keep thrust magnitude and direction coupled to the same trajectory
    // command.  The rejected fixed 22-degree powered prelead replaced this
    // direction after magnitude had already been solved; run 13 therefore
    // applied near-horizontal thrust to a command that requested 28.9 degrees
    // from local up, stopped 5 km short, and rebounded.  Prelead remains an
    // unpowered ignition-alignment aid.  The final safety projection below
    // still enforces the narrowed command cone on this live trajectory vector.
    LOCAL ACTUAL_THRUST_TILT IS VANG(SHIP:FACING:VECTOR, UP_VEC).
    LOCAL WAYPOINT_TRIM_ACTIVE IS FALSE.
    IF WAYPOINT_COAST_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL WAYPOINT_FINAL_DROP IS MAX(HOOK_HEIGHT
            - TERMINAL_WAYPOINT_HEIGHT, 0).
        LOCAL WAYPOINT_FINAL_VERTICAL_SPEED IS SQRT(MAX(
            VERTICAL_V^2 + 2 * G_ACC * WAYPOINT_FINAL_DROP, 0)).
        LOCAL WAYPOINT_FINAL_TGO IS (WAYPOINT_FINAL_VERTICAL_SPEED
            + VERTICAL_V) / MAX(G_ACC, 0.1).
        LOCAL WAYPOINT_FINAL_PREDICTED_H_POS IS HORIZONTAL_POS
            - HORIZONTAL_VEL * WAYPOINT_FINAL_TGO.
        LOCAL WAYPOINT_CENTER_ALONG_SPEED IS HORIZONTAL_VEL:MAG.
        IF WAYPOINT_CENTER_BRAKE_MODE
            AND WAYPOINT_CENTER_BRAKE_DIRECTION:MAG > 0.1 {
            SET WAYPOINT_CENTER_ALONG_SPEED TO MAX(VDOT(HORIZONTAL_VEL,
                WAYPOINT_CENTER_BRAKE_DIRECTION), 0).
        }
        IF NOT WAYPOINT_FINAL_COAST_MODE
            AND HORIZONTAL_VEL:MAG
                <= TERMINAL_WAYPOINT_FINAL_COAST_HORIZONTAL_SPEED {
            IF WAYPOINT_CENTER_BRAKE_MODE
                AND HORIZONTAL_POS:MAG <= TERMINAL_WAYPOINT_COAST_ERROR
                AND WAYPOINT_FINAL_PREDICTED_H_POS:MAG
                    <= TERMINAL_WAYPOINT_COAST_ERROR {
                SET WAYPOINT_FINAL_COAST_MODE TO TRUE.
            } ELSE IF NOT WAYPOINT_CENTER_BRAKE_MODE
                AND WAYPOINT_FINAL_PREDICTED_H_POS:MAG
                    <= TERMINAL_WAYPOINT_COAST_ERROR {
                SET WAYPOINT_FINAL_COAST_MODE TO TRUE.
            }
        }
        // The primary frozen-axis brake can leave a small perpendicular
        // velocity.  Correct it with measured fixed-direction delta-v impulses.
        // Each impulse finishes before a fresh residual direction is selected,
        // so guidance damps velocity without restoring position pursuit.
        IF WAYPOINT_CENTER_BRAKE_MODE
            AND NOT WAYPOINT_FINAL_COAST_MODE {
            IF WAYPOINT_ENDPOINT_TRIM_ACTIVE {
                LOCAL WAYPOINT_ENDPOINT_PROJECTION_REMAINING IS
                    WAYPOINT_ENDPOINT_TRIM_TARGET_PROJECTION
                    - VDOT(HORIZONTAL_VEL,
                        WAYPOINT_ENDPOINT_TRIM_DIRECTION).
                IF WAYPOINT_ENDPOINT_PROJECTION_REMAINING
                    <= TERMINAL_WAYPOINT_ENDPOINT_TRIM_TOLERANCE {
                    SET WAYPOINT_ENDPOINT_TRIM_ACTIVE TO FALSE.
                }
            } ELSE IF WAYPOINT_ENDPOINT_TRIM_COUNT
                < TERMINAL_WAYPOINT_ENDPOINT_MAX_PULSES {
                LOCAL WAYPOINT_ENDPOINT_DESIRED_H_VEL IS HORIZONTAL_POS
                    / MAX(WAYPOINT_FINAL_TGO, 1)
                    * TERMINAL_WAYPOINT_ENDPOINT_VELOCITY_SCALE.
                LOCAL WAYPOINT_ENDPOINT_DELTA_V IS
                    WAYPOINT_ENDPOINT_DESIRED_H_VEL - HORIZONTAL_VEL.
                IF WAYPOINT_ENDPOINT_DELTA_V:MAG
                    > TERMINAL_WAYPOINT_ENDPOINT_TRIM_TOLERANCE {
                    SET WAYPOINT_ENDPOINT_TRIM_DIRECTION TO
                        WAYPOINT_ENDPOINT_DELTA_V:NORMALIZED.
                    SET WAYPOINT_ENDPOINT_TRIM_TARGET_PROJECTION TO
                        VDOT(HORIZONTAL_VEL,
                            WAYPOINT_ENDPOINT_TRIM_DIRECTION)
                        + WAYPOINT_ENDPOINT_DELTA_V:MAG.
                    SET WAYPOINT_ENDPOINT_TRIM_ACTIVE TO TRUE.
                    SET WAYPOINT_ENDPOINT_TRIM_COUNT TO
                        WAYPOINT_ENDPOINT_TRIM_COUNT + 1.
                }
            }
        }
        // Coast first so the low-speed stage can rotate without spending
        // vertical impulse.  Once aligned and close to the ship, sparse 75%
        // pulses remove only the residual horizontal velocity.
        // Receding-horizon cubic rendezvous law.  In both horizontal axes it
        // drives position and velocity to zero at the ballistic 2 km crossing:
        // a = 6 * position / T^2 - 4 * velocity / T.  Unlike rebuilding an
        // along/cross frame from instantaneous velocity, this cannot turn a
        // nearly stopped approach into a circular chase around the ship.
        LOCAL WAYPOINT_TRIM_CONTROL_TGO IS MAX(WAYPOINT_FINAL_TGO, 0.5).
        LOCAL WAYPOINT_TRIM_ACCEL IS HORIZONTAL_POS
                * (6 / WAYPOINT_TRIM_CONTROL_TGO^2)
            - HORIZONTAL_VEL * (4 / WAYPOINT_TRIM_CONTROL_TGO).
        // The receding-horizon solution can ask for one last acceleration
        // toward the ship before a mathematically sharper stop.  The long
        // stage cannot reverse its attitude quickly enough to execute that
        // final switch: the measured approach slowed to 69 m/s at 149 m,
        // accelerated back to 137 m/s at 39 m, and then crossed the waypoint
        // while still turning around.  Once coast/trim owns the approach,
        // preserve its cross-axis correction but make the closing axis
        // one-way: it may only brake until the frozen centre-brake latch fires.
        IF NOT WAYPOINT_CENTER_BRAKE_MODE
            AND HORIZONTAL_POS:MAG
                > TERMINAL_WAYPOINT_CENTER_BRAKE_ENTRY_ERROR {
            LOCAL WAYPOINT_APPROACH_DIRECTION IS
                HORIZONTAL_POS:NORMALIZED.
            LOCAL WAYPOINT_APPROACH_SPEED IS VDOT(HORIZONTAL_VEL,
                WAYPOINT_APPROACH_DIRECTION).
            IF WAYPOINT_APPROACH_SPEED > 0 {
                // Aim halfway inside the latch radius so finite sampling cannot
                // stop just outside it and re-arm inward pursuit.
                LOCAL WAYPOINT_APPROACH_BRAKE_RANGE IS MAX(
                    HORIZONTAL_POS:MAG
                        - TERMINAL_WAYPOINT_CENTER_BRAKE_ENTRY_ERROR / 2,
                    TERMINAL_WAYPOINT_CENTER_BRAKE_MIN_RANGE).
                LOCAL WAYPOINT_APPROACH_BRAKE_ACCEL IS
                    WAYPOINT_APPROACH_SPEED^2
                    / (2 * WAYPOINT_APPROACH_BRAKE_RANGE)
                    * TERMINAL_WAYPOINT_APPROACH_BRAKE_GAIN.
                LOCAL WAYPOINT_APPROACH_ACCEL IS VDOT(
                    WAYPOINT_TRIM_ACCEL,
                    WAYPOINT_APPROACH_DIRECTION).
                IF WAYPOINT_APPROACH_ACCEL
                    > -WAYPOINT_APPROACH_BRAKE_ACCEL {
                    SET WAYPOINT_TRIM_ACCEL TO WAYPOINT_TRIM_ACCEL
                        - WAYPOINT_APPROACH_DIRECTION
                            * (WAYPOINT_APPROACH_ACCEL
                                + WAYPOINT_APPROACH_BRAKE_ACCEL).
                }
            }
        }
        IF WAYPOINT_CENTER_BRAKE_MODE {
            IF WAYPOINT_ENDPOINT_TRIM_ACTIVE {
                LOCAL WAYPOINT_ENDPOINT_REMAINING_DELTA_V IS MAX(
                    WAYPOINT_ENDPOINT_TRIM_TARGET_PROJECTION
                        - VDOT(HORIZONTAL_VEL,
                            WAYPOINT_ENDPOINT_TRIM_DIRECTION), 0).
                LOCAL WAYPOINT_ENDPOINT_TRIM_ACCEL IS MIN(
                    TERMINAL_WAYPOINT_ENDPOINT_TRIM_MAX_ACCEL,
                    MAX(WAYPOINT_ENDPOINT_REMAINING_DELTA_V
                            * TERMINAL_WAYPOINT_ENDPOINT_TRIM_GAIN,
                        TERMINAL_WAYPOINT_ENDPOINT_TRIM_MIN_ACCEL)).
                SET WAYPOINT_TRIM_ACCEL TO
                    WAYPOINT_ENDPOINT_TRIM_DIRECTION
                    * WAYPOINT_ENDPOINT_TRIM_ACCEL.
            } ELSE {
                SET WAYPOINT_TRIM_ACCEL TO V(0,0,0).
                SET STEERING_THRUST TO UP_VEC.
            }
        }
        SET WAYPOINT_TRIM_ACCEL TO CLAMPV(WAYPOINT_TRIM_ACCEL,
            TERMINAL_WAYPOINT_TRIM_MAX_ACCEL).
        SET DESIRED_THRUST TO WAYPOINT_TRIM_ACCEL.
        IF WAYPOINT_TRIM_ACCEL:MAG > 0.1 {
            SET STEERING_THRUST TO WAYPOINT_TRIM_ACCEL.
            // Once the total speed is below the audited 300 m/s load-cone
            // boundary, lead the horizontal trim slightly below the horizon if
            // descent has already become too slow.  Holding a 90-degree target
            // broadside reduced the measured 2 km descent speed to 79 m/s.
            IF STEERING_SURFACE_SPEED
                    < TERMINAL_VELOCITY_CONE_MIN_SPEED
                AND VERTICAL_V
                    > -TERMINAL_WAYPOINT_VERTICAL_SPEED {
                SET STEERING_THRUST TO
                    WAYPOINT_TRIM_ACCEL:NORMALIZED
                        * COS(TERMINAL_VERTICAL_RECOVERY_STEERING_DEGREES)
                    - UP_VEC
                        * SIN(TERMINAL_VERTICAL_RECOVERY_STEERING_DEGREES).
            }
        }
        SET WAYPOINT_TRIM_ACTIVE TO
            NOT WAYPOINT_FINAL_COAST_MODE
            AND HORIZONTAL_POS:MAG <= TERMINAL_WAYPOINT_TRIM_ARM_RANGE
            AND ACTUAL_THRUST_TILT
                >= TERMINAL_WAYPOINT_TRIM_MIN_ACTUAL_TILT
            AND (WAYPOINT_CENTER_BRAKE_MODE
                OR HORIZONTAL_VEL:MAG
                    > TERMINAL_WAYPOINT_COAST_HORIZONTAL_SPEED
                OR HORIZONTAL_POS:MAG
                    > TERMINAL_WAYPOINT_COAST_ERROR).
        // Once the strict horizontal corridor is reachable without another
        // pulse, turn the long empty stage upright while continuing to coast.
        // Remaining broadside at 90 degrees removed over 100 m/s of descent
        // through drag between this handoff and the formal 2 km plane.
        IF WAYPOINT_FINAL_COAST_MODE {
            IF WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE {
                LOCAL WAYPOINT_POST_UPRIGHT_REMAINING IS
                    WAYPOINT_POST_UPRIGHT_TARGET_PROJECTION
                    - VDOT(HORIZONTAL_VEL,
                        WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION).
                IF WAYPOINT_POST_UPRIGHT_REMAINING
                    <= TERMINAL_WAYPOINT_POST_UPRIGHT_TOLERANCE {
                    SET WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE TO FALSE.
                    SET WAYPOINT_POST_UPRIGHT_TRIM_DONE TO TRUE.
                }
            } ELSE IF TERMINAL_WAYPOINT_POST_UPRIGHT_ENABLED
                AND NOT WAYPOINT_POST_UPRIGHT_TRIM_DONE
                AND HOOK_HEIGHT
                    >= TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_HEIGHT
                AND HORIZONTAL_VEL:MAG
                    >= TERMINAL_WAYPOINT_POST_UPRIGHT_ARM_SPEED
                AND ACTUAL_THRUST_TILT
                    <= TERMINAL_WAYPOINT_POST_UPRIGHT_ARM_TILT {
                LOCAL WAYPOINT_POST_UPRIGHT_DESIRED_H_VEL IS
                    HORIZONTAL_POS / MAX(WAYPOINT_FINAL_TGO, 1)
                    * TERMINAL_WAYPOINT_POST_UPRIGHT_VELOCITY_SCALE.
                LOCAL WAYPOINT_POST_UPRIGHT_DELTA_V IS
                    WAYPOINT_POST_UPRIGHT_DESIRED_H_VEL - HORIZONTAL_VEL.
                IF WAYPOINT_POST_UPRIGHT_DELTA_V:MAG
                    > TERMINAL_WAYPOINT_POST_UPRIGHT_TOLERANCE {
                    SET WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION TO
                        WAYPOINT_POST_UPRIGHT_DELTA_V:NORMALIZED.
                    SET WAYPOINT_POST_UPRIGHT_TARGET_PROJECTION TO
                        VDOT(HORIZONTAL_VEL,
                            WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION)
                        + WAYPOINT_POST_UPRIGHT_DELTA_V:MAG.
                    SET WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE TO TRUE.
                } ELSE {
                    SET WAYPOINT_POST_UPRIGHT_TRIM_DONE TO TRUE.
                }
            }
            IF WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE {
                LOCAL WAYPOINT_POST_UPRIGHT_DELTA_REMAINING IS MAX(
                    WAYPOINT_POST_UPRIGHT_TARGET_PROJECTION
                        - VDOT(HORIZONTAL_VEL,
                            WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION), 0).
                LOCAL WAYPOINT_POST_UPRIGHT_H_ACCEL IS MIN(
                    TERMINAL_WAYPOINT_POST_UPRIGHT_MAX_ACCEL,
                    MAX(WAYPOINT_POST_UPRIGHT_DELTA_REMAINING
                            * TERMINAL_WAYPOINT_POST_UPRIGHT_GAIN,
                        TERMINAL_WAYPOINT_POST_UPRIGHT_MIN_ACCEL)).
                LOCAL WAYPOINT_POST_UPRIGHT_STEERING IS
                    UP_VEC * COS(TERMINAL_WAYPOINT_POST_UPRIGHT_TILT)
                    + WAYPOINT_POST_UPRIGHT_TRIM_DIRECTION
                        * SIN(TERMINAL_WAYPOINT_POST_UPRIGHT_TILT).
                SET STEERING_THRUST TO WAYPOINT_POST_UPRIGHT_STEERING.
                // Planning the correction and being ready to fire are separate
                // states.  The old path applied 75% while the physical stage
                // was still upright, turning a lateral trim into a large
                // vertical impulse.  Slew first, then start the correction.
                IF VANG(SHIP:FACING:VECTOR,
                    WAYPOINT_POST_UPRIGHT_STEERING)
                    <= TERMINAL_WAYPOINT_POST_UPRIGHT_ALIGNMENT_DEGREES {
                    SET DESIRED_THRUST TO WAYPOINT_POST_UPRIGHT_STEERING
                        * (WAYPOINT_POST_UPRIGHT_H_ACCEL
                            / SIN(TERMINAL_WAYPOINT_POST_UPRIGHT_TILT)).
                } ELSE {
                    SET DESIRED_THRUST TO V(0,0,0).
                }
            } ELSE {
                SET DESIRED_THRUST TO V(0,0,0).
                SET STEERING_THRUST TO
                    -SHIP:VELOCITY:SURFACE:NORMALIZED.
            }
        }
    }
    // Above the formal waypoint the mandatory continuous command cannot fall
    // below 75%.  Its direction must still follow the solved trajectory.  Run
    // 60 exposed that replacing every sub-floor vector with pure retrograde
    // applied -11.7 m/s^2 away from the cross-track target and erased the
    // closing velocity established by the fixed checkpoints.  Project the
    // complete solved vector into the safety cone, then add magnitude only;
    // this preserves both axes without weakening the physical nozzle audit.
    IF HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL MAIN_TRACKING_HIGH_BLEND IS CLAMP(
            (TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT - HOOK_HEIGHT)
            / MAX(TERMINAL_MAIN_TRACKING_RAMP_START_HEIGHT
                - TERMINAL_MAIN_TRACKING_FULL_HEIGHT, 1), 0, 1).
        LOCAL MAIN_TRACKING_LOW_BLEND IS CLAMP(
            (HOOK_HEIGHT - TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT)
            / MAX(TERMINAL_MAIN_TRACKING_FADE_START_HEIGHT
                - TERMINAL_MAIN_TRACKING_FADE_END_HEIGHT, 1), 0, 1).
        LOCAL MAIN_TRACKING_BLEND IS MIN(MAIN_TRACKING_HIGH_BLEND,
            MAIN_TRACKING_LOW_BLEND).
        LOCAL MAIN_FLOOR_FRACTION IS
            TERMINAL_NOMINAL_THRUST_FRACTION
            + (TERMINAL_MAIN_TRACKING_THRUST_FRACTION
                - TERMINAL_NOMINAL_THRUST_FRACTION)
                * MAIN_TRACKING_BLEND.
        LOCAL MAIN_FLOOR_ACCEL IS AVAILABLE_ACC * MAIN_FLOOR_FRACTION.
        IF DESIRED_THRUST:MAG < MAIN_FLOOR_ACCEL {
            LOCAL MAIN_FLOOR_AXIS IS THRUST_SAFETY_AXIS(UP_VEC,
                TERMINAL_VELOCITY_CONE_MIN_SPEED).
            IF DESIRED_THRUST:MAG > 0.01 {
                SET MAIN_FLOOR_AXIS TO CONSTRAIN_THRUST_VECTOR(
                    DESIRED_THRUST, MAIN_FLOOR_AXIS,
                    ACTIVE_COMMAND_CONE_DEGREES):NORMALIZED.
            }
            SET DESIRED_THRUST TO MAIN_FLOOR_AXIS * MAIN_FLOOR_ACCEL.
            SET STEERING_THRUST TO DESIRED_THRUST.
        }
    }
    // Apply the safety cone last.  Several legacy handoff branches modify the
    // steering vector after translational guidance; final projection prevents
    // any of them from overriding the actual powered-flight constraint.
    LOCAL FINAL_THRUST_SAFETY_AXIS IS THRUST_SAFETY_AXIS(UP_VEC,
        TERMINAL_VELOCITY_CONE_MIN_SPEED).
    SET STEERING_THRUST TO CONSTRAIN_THRUST_VECTOR(STEERING_THRUST,
        FINAL_THRUST_SAFETY_AXIS, ACTIVE_COMMAND_CONE_DEGREES).
    // A position controller may request forward acceleration after the load
    // cone has already removed too much downrange speed.  Forward thrust is
    // unavailable, but the azimuth of the safe cone edge can minimise further
    // braking while retaining the independently damped cross-track request.
    // Use the constant-deceleration endpoint speed as a feasibility monitor;
    // fade this allocation away above 4 km so the final segment owns velocity
    // removal and the 2 km state cannot become a high-speed fly-through.
    LOCAL MAIN_ALONG_POS IS MAX(VDOT(WAYPOINT_CONTROL_H_POS,
        CONTROL_ALONG_DIRECTION), 0).
    LOCAL MAIN_ALONG_COAST_THROTTLE_SCALE IS 1.
    LOCAL MAIN_ALONG_COAST_STEERING_AXIS_DIAG IS 0.
    LOCAL MAIN_ALONG_VEL IS VDOT(HORIZONTAL_VEL,
        CONTROL_ALONG_DIRECTION).
    LOCAL MAIN_ALONG_TGO IS 2
        * MAX(HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT, 0)
        / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
    LOCAL MAIN_ALONG_REFERENCE_SPEED IS 2 * MAIN_ALONG_POS
        / MAX(MAIN_ALONG_TGO, 0.5).
    LOCAL MAIN_ALONG_SPEED_DEFICIT IS MAIN_ALONG_REFERENCE_SPEED
        - MAIN_ALONG_VEL.
    LOCAL MAIN_ALONG_SPEED_EXCESS IS MAIN_ALONG_VEL
        - MAIN_ALONG_REFERENCE_SPEED.
    LOCAL MAIN_ALONG_BRAKE_USABLE_RANGE IS MAX(
        MAIN_ALONG_POS - TERMINAL_HORIZONTAL_DEADBAND,
        TERMINAL_ALONG_BRAKE_REACHABILITY_MIN_RANGE).
    LOCAL MAIN_ALONG_BRAKE_REQUIRED_DECEL IS 0.
    LOCAL MAIN_ALONG_BRAKE_ENGINE_PROJECTION IS 0.
    LOCAL MAIN_ALONG_BRAKE_AERO_DECEL IS 0.
    LOCAL MAIN_ALONG_BRAKE_AVAILABLE_DECEL IS 0.
    LOCAL MAIN_ALONG_BRAKE_PRESSURE IS 0.
    IF MAIN_ALONG_POS > TERMINAL_HORIZONTAL_DEADBAND
        AND MAIN_ALONG_VEL > 0.1 {
        SET MAIN_ALONG_BRAKE_REQUIRED_DECEL TO MAIN_ALONG_VEL^2
            / (2 * MAIN_ALONG_BRAKE_USABLE_RANGE).
        LOCAL MAIN_ALONG_BRAKE_DIRECTION IS -CONTROL_ALONG_DIRECTION.
        LOCAL MAIN_ALONG_BRAKE_SEPARATION IS VANG(
            FINAL_THRUST_SAFETY_AXIS, MAIN_ALONG_BRAKE_DIRECTION).
        SET MAIN_ALONG_BRAKE_ENGINE_PROJECTION TO CLAMP(COS(MAX(
            MAIN_ALONG_BRAKE_SEPARATION
                - ACTIVE_COMMAND_CONE_DEGREES, 0)), 0, 1).
        SET MAIN_ALONG_BRAKE_AERO_DECEL TO MAX(-VDOT(
            POWERED_FILTERED_HORIZONTAL_AERO_ACCEL,
            CONTROL_ALONG_DIRECTION), 0).
        SET MAIN_ALONG_BRAKE_AVAILABLE_DECEL TO
            AVAILABLE_ACC * MAIN_ALONG_BRAKE_ENGINE_PROJECTION
            + MAIN_ALONG_BRAKE_AERO_DECEL.
        SET MAIN_ALONG_BRAKE_PRESSURE TO
            MAIN_ALONG_BRAKE_REQUIRED_DECEL
            / MAX(MAIN_ALONG_BRAKE_AVAILABLE_DECEL, 1).
    }
    LOCAL MAIN_ALONG_BRAKE_AUTHORITY_LIMIT IS
        TERMINAL_ALONG_BRAKE_MAX_BLEND.
    LOCAL MAIN_ALONG_BRAKE_LATE_BLEND IS 0.
    LOCAL MAIN_ALONG_BRAKE_PRESSURE_BLEND IS CLAMP(
        (MAIN_ALONG_BRAKE_PRESSURE
            - TERMINAL_ALONG_BRAKE_PRESSURE_ARM)
        / MAX(TERMINAL_ALONG_BRAKE_PRESSURE_FULL
            - TERMINAL_ALONG_BRAKE_PRESSURE_ARM, 0.01), 0, 1).
    LOCAL MAIN_ALONG_BRAKE_HEIGHT_BLEND IS CLAMP(
        (TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT - HOOK_HEIGHT)
        / MAX(TERMINAL_ALONG_BRAKE_REACHABILITY_START_HEIGHT
            - TERMINAL_ALONG_BRAKE_REACHABILITY_FULL_HEIGHT, 1), 0, 1).
    LOCAL MAIN_ALONG_BRAKE_BLEND IS MAIN_ALONG_BRAKE_PRESSURE_BLEND
        * MAIN_ALONG_BRAKE_HEIGHT_BLEND.
    SET MAIN_ALONG_BRAKE_BLEND TO MIN(MAIN_ALONG_BRAKE_BLEND,
        MAIN_ALONG_BRAKE_AUTHORITY_LIMIT).
    // A disabled diagnostic allocator must be completely inert.  Run 49
    // exposed that the old non-zero diagnostic blend still raised the main
    // throttle floor from 75.0% to 75.75%, even though its steering branch was
    // correctly skipped.
    IF NOT TERMINAL_ALONG_BRAKE_ENABLED {
        SET MAIN_ALONG_BRAKE_BLEND TO 0.
    }
    LOCAL MAIN_ALONG_H_ACCEL IS VDOT(H_ACCEL,
        CONTROL_ALONG_DIRECTION).
    LOCAL MAIN_CROSS_H_ACCEL IS H_ACCEL
        - CONTROL_ALONG_DIRECTION * MAIN_ALONG_H_ACCEL.
    LOCAL MAIN_ALONG_COAST_SPEED_BLEND IS CLAMP(
        (MAIN_ALONG_SPEED_DEFICIT
            - TERMINAL_ALONG_SPEED_DEFICIT_ARM)
        / MAX(TERMINAL_ALONG_SPEED_DEFICIT_BLEND, 0.1), 0, 1).
    LOCAL MAIN_ALONG_COAST_HEIGHT_BLEND IS CLAMP(
        (HOOK_HEIGHT - TERMINAL_ALONG_COAST_END_HEIGHT)
        / MAX(TERMINAL_ALONG_COAST_FADE_START_HEIGHT
            - TERMINAL_ALONG_COAST_END_HEIGHT, 1), 0, 1).
    LOCAL MAIN_ALONG_COAST_BLEND IS MAIN_ALONG_COAST_SPEED_BLEND
        * MAIN_ALONG_COAST_HEIGHT_BLEND.
    IF TERMINAL_ALONG_COAST_ENABLED AND MAIN_ALONG_COAST_BLEND > 0 {
        // Run 89 measured the forward legal cone edge directly: its engine
        // along-track braking was slightly smaller, but its added body drag was
        // much larger.  Blend toward surface retrograde instead; this is the
        // identified minimum-net-braking powered attitude for the current body.
        LOCAL MAIN_ALONG_COAST_PREVIOUS_AXIS IS
            STEERING_THRUST:NORMALIZED.
        LOCAL MAIN_ALONG_COAST_PREVIOUS_UP IS MAX(VDOT(
            MAIN_ALONG_COAST_PREVIOUS_AXIS, UP_VEC), 0.05).
        LOCAL MAIN_ALONG_COAST_AXIS IS
            (MAIN_ALONG_COAST_PREVIOUS_AXIS
                * (1 - MAIN_ALONG_COAST_BLEND)
            + FINAL_THRUST_SAFETY_AXIS:NORMALIZED
                * MAIN_ALONG_COAST_BLEND):NORMALIZED.
        LOCAL MAIN_ALONG_COAST_NEW_UP IS MAX(VDOT(
            MAIN_ALONG_COAST_AXIS, UP_VEC), 0.05).
        SET MAIN_ALONG_COAST_THROTTLE_SCALE TO MAX(1,
            MAIN_ALONG_COAST_PREVIOUS_UP
                / MAIN_ALONG_COAST_NEW_UP).
        SET STEERING_THRUST TO MAIN_ALONG_COAST_AXIS
            * MAX(STEERING_THRUST:MAG, 0.001).
        SET MAIN_ALONG_COAST_STEERING_AXIS_DIAG TO VDOT(
            MAIN_ALONG_COAST_AXIS, CONTROL_ALONG_DIRECTION).
    }
    // Run 98 proved that the cubic endpoint request can remain smooth while
    // the physical state has already crossed outside its stopping footprint.
    // Runs 109-110 then proved that v^2/(2r) controls the eventual ship
    // crossing, not the required horizontal speed at the formal 2 km height:
    // moving the ship rewrote the complete 14-6 km force history, and an early
    // crossing disabled useful one-way braking.  Compare a finite time-to-plane
    // requirement with measured net force instead.
    // When braking is short, blend toward the *forward/upright* legal cone edge
    // identified in Run 89.  Its engine component is smaller, but the long
    // body's added downrange drag makes it the useful maximum-net-brake
    // actuator without the large upward aerodynamic force of Run 96's opposite
    // edge.  State/rate limits absorb attitude lag and prevent direction PWM.
    LOCAL MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND IS CLAMP(
        (TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT - HOOK_HEIGHT)
        / MAX(TERMINAL_ALONG_AERO_BRAKE_START_HEIGHT
            - TERMINAL_ALONG_AERO_BRAKE_FULL_HEIGHT, 1), 0, 1).
    LOCAL MAIN_ALONG_AERO_BRAKE_TGO IS
        2 * MAX(HOOK_HEIGHT - TERMINAL_WAYPOINT_HEIGHT, 0)
        / MAX(-VERTICAL_V + TERMINAL_WAYPOINT_VERTICAL_SPEED, 1).
    LOCAL MAIN_ALONG_AERO_BRAKE_TIME_REQUIRED IS MAX(
        MAIN_ALONG_VEL - TERMINAL_WAYPOINT_MAX_HORIZONTAL_SPEED, 0)
        / MAX(MAIN_ALONG_AERO_BRAKE_TGO, 0.25)
        * TERMINAL_ALONG_AERO_BRAKE_MARGIN.
    LOCAL MAIN_ALONG_AERO_BRAKE_RANGE_REQUIRED IS
        MAIN_ALONG_BRAKE_REQUIRED_DECEL
            * TERMINAL_ALONG_AERO_BRAKE_MARGIN.
    LOCAL MAIN_ALONG_AERO_BRAKE_STAGE_TGO IS 0.
    LOCAL MAIN_ALONG_AERO_BRAKE_STAGE_REQUIRED IS 0.
    IF HOOK_HEIGHT > TERMINAL_ALONG_AERO_BRAKE_STAGE_HEIGHT {
        SET MAIN_ALONG_AERO_BRAKE_STAGE_TGO TO
            2 * (HOOK_HEIGHT - TERMINAL_ALONG_AERO_BRAKE_STAGE_HEIGHT)
            / MAX(-VERTICAL_V
                + TERMINAL_ALONG_AERO_BRAKE_STAGE_DESCENT_SPEED, 1).
        SET MAIN_ALONG_AERO_BRAKE_STAGE_REQUIRED TO MAX(
            MAIN_ALONG_VEL
                - TERMINAL_ALONG_AERO_BRAKE_STAGE_HORIZONTAL_SPEED, 0)
            / MAX(MAIN_ALONG_AERO_BRAKE_STAGE_TGO,
                TERMINAL_ALONG_AERO_BRAKE_STAGE_RESPONSE_SECONDS)
            * TERMINAL_ALONG_AERO_BRAKE_MARGIN.
    }
    LOCAL MAIN_ALONG_AERO_BRAKE_REQUIRED IS MAX(
        MAIN_ALONG_AERO_BRAKE_STAGE_REQUIRED,
        MAX(MAIN_ALONG_AERO_BRAKE_TIME_REQUIRED,
            MAIN_ALONG_AERO_BRAKE_RANGE_REQUIRED)).
    LOCAL MAIN_ALONG_AERO_BRAKE_REALIZED IS MAX(-VDOT(
        POWERED_MEASURED_ACCEL, CONTROL_ALONG_DIRECTION), 0).
    LOCAL MAIN_ALONG_AERO_BRAKE_ERROR IS
        MAIN_ALONG_AERO_BRAKE_REQUIRED
            - MAIN_ALONG_AERO_BRAKE_REALIZED.
    IF ABS(MAIN_ALONG_AERO_BRAKE_ERROR)
            <= TERMINAL_ALONG_AERO_BRAKE_ERROR_DEADBAND {
        SET MAIN_ALONG_AERO_BRAKE_ERROR TO 0.
    }
    LOCAL MAIN_ALONG_AERO_BRAKE_RATE IS
        TERMINAL_ALONG_AERO_BRAKE_BUILD_RATE.
    IF MAIN_ALONG_AERO_BRAKE_ERROR < 0 {
        SET MAIN_ALONG_AERO_BRAKE_RATE TO
            TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE.
    }
    // These configuration values are physical scalar-per-second rates.  The
    // historical controller DT is deliberately capped at 0.1 s, while real
    // KSP physics samples in Runs 111-113 are about 0.4-0.46 s; using DT made
    // the actuator respond roughly four times slower than declared.  Retain a
    // 0.5 s discontinuity cap without assigning fake units to the rate.
    LOCAL MAIN_ALONG_AERO_BRAKE_RATE_DT IS CLAMP(
        POWERED_MEASUREMENT_DT, 0.001, 0.5).
    LOCAL MAIN_ALONG_AERO_BRAKE_STEP IS CLAMP(
        MAIN_ALONG_AERO_BRAKE_ERROR
            / MAX(TERMINAL_ALONG_AERO_BRAKE_ACCEL_GAIN, 0.1),
        -MAIN_ALONG_AERO_BRAKE_RATE * MAIN_ALONG_AERO_BRAKE_RATE_DT,
        MAIN_ALONG_AERO_BRAKE_RATE * MAIN_ALONG_AERO_BRAKE_RATE_DT).
    IF NOT TERMINAL_ALONG_AERO_BRAKE_ENABLED
        OR HOOK_HEIGHT <= TERMINAL_WAYPOINT_HEIGHT
        OR MAIN_ALONG_VEL <= TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED {
        SET MAIN_ALONG_AERO_BRAKE_STEP TO
            -TERMINAL_ALONG_AERO_BRAKE_RELEASE_RATE
                * MAIN_ALONG_AERO_BRAKE_RATE_DT.
    } ELSE {
        SET MAIN_ALONG_AERO_BRAKE_STEP TO
            MAIN_ALONG_AERO_BRAKE_STEP
                * MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND.
    }
    SET ALONG_AERO_BRAKE_BLEND_STATE TO CLAMP(
        ALONG_AERO_BRAKE_BLEND_STATE + MAIN_ALONG_AERO_BRAKE_STEP,
        0, TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND).
    IF TERMINAL_EARLY_AERO_BRAKE_LATCHED
        AND HOOK_HEIGHT <= TERMINAL_EARLY_AERO_BRAKE_LATCH_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_EARLY_AERO_BRAKE_HOLD_END_HEIGHT
        AND MAIN_ALONG_VEL > TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED {
        SET ALONG_AERO_BRAKE_BLEND_STATE TO MAX(
            ALONG_AERO_BRAKE_BLEND_STATE,
            TERMINAL_EARLY_AERO_BRAKE_FLOOR).
    }
    // The high-drag endpoint is the measured maximum-net-braking actuator in
    // both the high-q and terminal samples.  Once Run 117 has reached it near
    // 8 km, do not let an intermediate force surplus unwind the actuator only
    // to demand it again below 3.5 km.  The margined velocity gate still
    // removes this ownership before a reversal can be requested.
    IF HOOK_HEIGHT <= TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_HEIGHT
        AND HOOK_HEIGHT
            > FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT
        AND MAIN_ALONG_VEL > TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED {
        SET ALONG_AERO_BRAKE_BLEND_STATE TO
            TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND
                * FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE.
    }
    // The frozen low-ratio branch also bounds tail authority. Run 166's 0.50
    // crossed fast; Run 167's immediate 0.75 stopped 30 m short. Ramp between
    // them with height so travel is retained first and braking restored late.
    // A scale-1 branch remains mathematically unchanged.
    LOCAL FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING_BLEND IS CLAMP(
        (FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT - HOOK_HEIGHT)
        / MAX(FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT
            - FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING_FULL_HEIGHT,
            1), 0, 1).
    LOCAL FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING IS
        FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE
        + (MAX(FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE,
                FINAL_ALIGN_TERMINAL_AERO_BRAKE_MIN_TAIL_CEILING)
            - FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE)
            * FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING_BLEND.
    IF FINAL_ALIGN_TERMINAL_PHASE_LATCHED
        AND HOOK_HEIGHT
            <= FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND MAIN_ALONG_VEL > TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED {
        SET ALONG_AERO_BRAKE_BLEND_STATE TO MIN(
            ALONG_AERO_BRAKE_BLEND_STATE,
            TERMINAL_ALONG_AERO_BRAKE_MAX_BLEND
                * FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING).
    }
    // Runs 119--121 proved that merely releasing allocator ownership leaves
    // the fallback trajectory on a 10--15 degree cone while the long stage is
    // still near 21 degrees, carrying signed speed through zero. Once the
    // verified high-drag branch has reduced speed to the lead threshold *and*
    // the physical hook is inside its readiness ring, latch a live
    // surface-retrograde centre target that damps either velocity sign. Runs
    // 167--170 proved that speed alone can occur 19--40 m off target.
    IF NOT ALONG_AERO_BRAKE_LOW_SPEED_SETTLE_ACTIVE
        AND HOOK_HEIGHT <= TERMINAL_ALONG_AERO_BRAKE_FULL_HOLD_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND MAIN_ALONG_VEL <= TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED
        AND MAIN_ALONG_VEL > 0
        AND HORIZONTAL_POS:MAG <= FINAL_ALIGN_READY_ERROR {
        SET ALONG_AERO_BRAKE_LOW_SPEED_SETTLE_ACTIVE TO TRUE.
    }
    LOCAL MAIN_ALONG_AERO_BRAKE_AXIS_DIAG IS 0.
    IF TERMINAL_ALONG_AERO_BRAKE_ENABLED
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND MAIN_ALONG_VEL > TERMINAL_ALONG_AERO_BRAKE_RELEASE_SPEED
        AND MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND > 0 {
        LOCAL MAIN_ALONG_AERO_BRAKE_REQUEST IS
                CONTROL_ALONG_DIRECTION
                    * MAX(ABS(MAIN_ALONG_H_ACCEL),
                        TERMINAL_ALONG_BRAKE_MIN_REQUEST)
            + MAIN_CROSS_H_ACCEL.
        LOCAL MAIN_ALONG_AERO_BRAKE_EDGE IS CONSTRAIN_THRUST_VECTOR(
            MAIN_ALONG_AERO_BRAKE_REQUEST,
            FINAL_THRUST_SAFETY_AXIS,
            ACTIVE_COMMAND_CONE_DEGREES):NORMALIZED.
        // The actuator coordinate must have physical endpoints.  Run 99
        // released the scalar to zero, but zero meant "cubic command" and the
        // cubic retained the same upright/high-drag attitude.  Interpolate
        // explicitly from measured minimum-net braking (surface retrograde) to
        // the measured maximum-net aerodynamic edge, then use the height ramp
        // only to transfer ownership smoothly from the preceding trajectory.
        LOCAL MAIN_ALONG_AERO_BRAKE_ACTUATOR_AXIS IS
            (FINAL_THRUST_SAFETY_AXIS:NORMALIZED
                * (1 - ALONG_AERO_BRAKE_BLEND_STATE)
            + MAIN_ALONG_AERO_BRAKE_EDGE
                * ALONG_AERO_BRAKE_BLEND_STATE):NORMALIZED.
        LOCAL MAIN_ALONG_AERO_BRAKE_AXIS IS
            (STEERING_THRUST:NORMALIZED
                * (1 - MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND)
            + MAIN_ALONG_AERO_BRAKE_ACTUATOR_AXIS
                * MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND):NORMALIZED.
        SET STEERING_THRUST TO CONSTRAIN_THRUST_VECTOR(
            MAIN_ALONG_AERO_BRAKE_AXIS,
            FINAL_THRUST_SAFETY_AXIS,
            ACTIVE_COMMAND_CONE_DEGREES)
            * MAX(STEERING_THRUST:MAG, 0.001).
        SET MAIN_ALONG_AERO_BRAKE_AXIS_DIAG TO VDOT(
            STEERING_THRUST:NORMALIZED,
            CONTROL_ALONG_DIRECTION).
    }
    // Short constant-acceleration projections are diagnostics, not another
    // controller. They expose whether the measured KSP plant would reach the
    // formal plane under its current force, so a miss can be separated into
    // bad timing, bad force magnitude, or a stale attitude response.
    LOCAL MAIN_MEASURED_ALONG_ACCEL IS VDOT(POWERED_MEASURED_ACCEL,
        CONTROL_ALONG_DIRECTION).
    LOCAL MAIN_MEASURED_VERTICAL_ACCEL IS VDOT(POWERED_MEASURED_ACCEL,
        UP_VEC).
    LOCAL MAIN_PREDICTED_ALONG_POS IS MAIN_ALONG_POS
        - MAIN_ALONG_VEL * MAIN_ALONG_TGO
        - 0.5 * MAIN_MEASURED_ALONG_ACCEL * MAIN_ALONG_TGO^2.
    LOCAL MAIN_PREDICTED_ALONG_VEL IS MAIN_ALONG_VEL
        + MAIN_MEASURED_ALONG_ACCEL * MAIN_ALONG_TGO.
    LOCAL MAIN_PREDICTED_VERTICAL_VEL IS VERTICAL_V
        + MAIN_MEASURED_VERTICAL_ACCEL * MAIN_ALONG_TGO.
    // The stopping latch is stronger than the smooth endpoint reference: once
    // it commits, the stage has no remaining prograde branch.  Put the final
    // attitude target on the braking side of the legal velocity cone while
    // preserving the independently solved cross-track azimuth and thrust
    // magnitude.  CONSTRAIN_THRUST_VECTOR and the measured physical barrier
    // below still enforce the mandatory 30-degree limit.
    IF TERMINAL_MAIN_FIXED_AXIS_EDGE_STEERING_ENABLED
        AND FIXED_STOP_COMMITTED
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL FIXED_EDGE_ALONG_POS IS VDOT(WAYPOINT_CONTROL_H_POS,
            CONTROL_ALONG_DIRECTION).
        LOCAL FIXED_EDGE_ALONG_VEL IS VDOT(HORIZONTAL_VEL,
            CONTROL_ALONG_DIRECTION).
        IF FIXED_EDGE_ALONG_POS > TERMINAL_HORIZONTAL_DEADBAND
            AND FIXED_EDGE_ALONG_VEL > 0.1 {
            SET FIXED_STOP_EDGE_ACTIVE TO TRUE.
            LOCAL FIXED_EDGE_ALONG_REQUEST IS MAX(
                ABS(MAIN_ALONG_H_ACCEL), 5).
            LOCAL FIXED_EDGE_REQUEST IS -CONTROL_ALONG_DIRECTION
                    * FIXED_EDGE_ALONG_REQUEST
                + MAIN_CROSS_H_ACCEL.
            LOCAL FIXED_EDGE_AXIS IS CONSTRAIN_THRUST_VECTOR(
                FIXED_EDGE_REQUEST, FINAL_THRUST_SAFETY_AXIS,
                ACTIVE_COMMAND_CONE_DEGREES):NORMALIZED.
            SET STEERING_THRUST TO FIXED_EDGE_AXIS
                * MAX(STEERING_THRUST:MAG, 0.001).
        }
    }
    // The long body reverses the cross-axis control sign in dense air.  Runs
    // 77/80 commanded the engine outward, but near 6 km the resulting inward
    // body force exceeded the engine by roughly 1.8:1.  Preserve the frozen
    // *inward* cross attitude instead; after the 8 km slew lead, weathercock
    // force supplies the requested outward braking without a second pass.
    // Only the cross sign changes.  The already-solved vertical/downrange
    // components, thrust magnitude, active cone and physical guard remain.
    IF TERMINAL_MAIN_CROSS_AERO_BRAKE_ENABLED
        AND CROSS_STOP_ACTIVE
        AND NOT CROSS_STOP_SUPPRESS_ACTIVE
        AND HOOK_HEIGHT <= TERMINAL_MAIN_CROSS_AERO_BRAKE_START_HEIGHT
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL CROSS_AERO_BRAKE_AXIS IS VXCL(UP_VEC,
            CROSS_STOP_DIRECTION).
        IF CROSS_AERO_BRAKE_AXIS:MAG > 0.001 {
            SET CROSS_AERO_BRAKE_AXIS TO
                CROSS_AERO_BRAKE_AXIS:NORMALIZED.
            LOCAL CROSS_AERO_BRAKE_SIGNED_COMPONENT IS VDOT(
                STEERING_THRUST, CROSS_AERO_BRAKE_AXIS).
            LOCAL CROSS_AERO_BRAKE_BASE IS STEERING_THRUST
                - CROSS_AERO_BRAKE_AXIS
                    * CROSS_AERO_BRAKE_SIGNED_COMPONENT.
            LOCAL CROSS_AERO_BRAKE_FULL_COMPONENT IS MAX(
                ABS(CROSS_AERO_BRAKE_SIGNED_COMPONENT),
                TERMINAL_MAIN_CROSS_AERO_BRAKE_MIN_ENGINE_ACCEL).
            // Direction reversal is a high-q actuator, not a permanent trim.
            // Run 84 reached a near-zero command while the physical stage was
            // still producing 1.5-3.6 m/s^2 of braking.  Schedule on the
            // remaining demand after that measured force, and make the build
            // / release phases monotonic so force noise cannot recreate a
            // direction-PWM loop.
            LOCAL CROSS_AERO_BRAKE_REALIZED_DECEL IS MAX(-VDOT(
                POWERED_MEASURED_ACCEL, CROSS_AERO_BRAKE_AXIS), 0).
            LOCAL CROSS_AERO_BRAKE_RESIDUAL_DEMAND IS MAX(
                CROSS_STOP_REQUIRED_DECEL_DIAG
                    - CROSS_AERO_BRAKE_REALIZED_DECEL, 0).
            LOCAL CROSS_AERO_BRAKE_CANDIDATE_BLEND IS MIN(1, MAX(0,
                CROSS_AERO_BRAKE_RESIDUAL_DEMAND
                    / TERMINAL_MAIN_CROSS_AERO_BRAKE_FULL_DEMAND)).
            IF NOT CROSS_AERO_BRAKE_RELEASE_STARTED
                AND CROSS_AERO_BRAKE_PREVIOUS_DEMAND
                    >= TERMINAL_MAIN_CROSS_AERO_BRAKE_FULL_DEMAND
                AND CROSS_STOP_REQUIRED_DECEL_DIAG
                    < CROSS_AERO_BRAKE_PREVIOUS_DEMAND {
                SET CROSS_AERO_BRAKE_RELEASE_STARTED TO TRUE.
            }
            IF CROSS_AERO_BRAKE_RELEASE_STARTED {
                SET CROSS_AERO_BRAKE_BLEND_STATE TO MIN(
                    CROSS_AERO_BRAKE_BLEND_STATE,
                    CROSS_AERO_BRAKE_CANDIDATE_BLEND).
            } ELSE {
                SET CROSS_AERO_BRAKE_BLEND_STATE TO MAX(
                    CROSS_AERO_BRAKE_BLEND_STATE,
                    CROSS_AERO_BRAKE_CANDIDATE_BLEND).
            }
            LOCAL CROSS_AERO_BRAKE_RELEASE_BLEND IS
                CROSS_AERO_BRAKE_BLEND_STATE.
            LOCAL CROSS_AERO_BRAKE_INWARD_COMPONENT IS
                CROSS_AERO_BRAKE_FULL_COMPONENT
                    * CROSS_AERO_BRAKE_RELEASE_BLEND.
            LOCAL CROSS_AERO_BRAKE_REQUEST IS
                CROSS_AERO_BRAKE_BASE
                + CROSS_AERO_BRAKE_AXIS
                    * CROSS_AERO_BRAKE_INWARD_COMPONENT.
            LOCAL CROSS_AERO_BRAKE_LIMITED IS CONSTRAIN_THRUST_VECTOR(
                CROSS_AERO_BRAKE_REQUEST,
                FINAL_THRUST_SAFETY_AXIS,
                ACTIVE_COMMAND_CONE_DEGREES):NORMALIZED.
            SET STEERING_THRUST TO CROSS_AERO_BRAKE_LIMITED
                * MAX(STEERING_THRUST:MAG, 0.001).
            SET CROSS_AERO_BRAKE_STEERING_ACTIVE TO TRUE.
            SET CROSS_AERO_BRAKE_STEERING_AXIS_DIAG TO VDOT(
                CROSS_AERO_BRAKE_AXIS,
                STEERING_THRUST:NORMALIZED).
            SET CROSS_AERO_BRAKE_RELEASE_BLEND_DIAG TO
                CROSS_AERO_BRAKE_RELEASE_BLEND.
            SET CROSS_AERO_BRAKE_REQUEST_ACCEL_DIAG TO
                CROSS_AERO_BRAKE_INWARD_COMPONENT.
            SET CROSS_AERO_BRAKE_REALIZED_DECEL_DIAG TO
                CROSS_AERO_BRAKE_REALIZED_DECEL.
            SET CROSS_AERO_BRAKE_RESIDUAL_DEMAND_DIAG TO
                CROSS_AERO_BRAKE_RESIDUAL_DEMAND.
        }
    }
    IF CROSS_STOP_ACTIVE {
        SET CROSS_AERO_BRAKE_PREVIOUS_DEMAND TO
            CROSS_STOP_REQUIRED_DECEL_DIAG.
    }
    // Apply reachability braking only after the high-drag and cross-axis
    // allocators have produced their actual command. Runs 135--136 exposed
    // that applying this blend earlier merely let the high-drag branch
    // overwrite it later in the same update. The 0.35 cap therefore retains
    // at least 65% of that measured aerodynamic endpoint while finally making
    // the requested opposite-edge share physical.
    LOCAL MAIN_ALONG_BRAKE_APPLIED_DIAG IS FALSE.
    LOCAL MAIN_ALONG_BRAKE_APPLIED_AXIS_DIAG IS 0.
    IF TERMINAL_ALONG_BRAKE_ENABLED
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT
        AND MAIN_ALONG_BRAKE_BLEND > 0 {
        LOCAL FINAL_ALONG_BRAKE_REQUEST IS -CONTROL_ALONG_DIRECTION
                * MAX(ABS(MAIN_ALONG_H_ACCEL),
                    TERMINAL_ALONG_BRAKE_MIN_REQUEST)
            + MAIN_CROSS_H_ACCEL.
        LOCAL FINAL_ALONG_BRAKE_EDGE IS CONSTRAIN_THRUST_VECTOR(
            FINAL_ALONG_BRAKE_REQUEST, FINAL_THRUST_SAFETY_AXIS,
            ACTIVE_COMMAND_CONE_DEGREES):NORMALIZED.
        LOCAL FINAL_ALONG_BRAKE_AXIS IS
            (STEERING_THRUST:NORMALIZED
                * (1 - MAIN_ALONG_BRAKE_BLEND)
            + FINAL_ALONG_BRAKE_EDGE
                * MAIN_ALONG_BRAKE_BLEND):NORMALIZED.
        SET STEERING_THRUST TO CONSTRAIN_THRUST_VECTOR(
            FINAL_ALONG_BRAKE_AXIS, FINAL_THRUST_SAFETY_AXIS,
            ACTIVE_COMMAND_CONE_DEGREES)
            * MAX(STEERING_THRUST:MAG, 0.001).
        SET MAIN_ALONG_BRAKE_APPLIED_DIAG TO TRUE.
        SET MAIN_ALONG_BRAKE_APPLIED_AXIS_DIAG TO VDOT(
            STEERING_THRUST:NORMALIZED, CONTROL_ALONG_DIRECTION).
    }
    // Apply the low-speed settle after the along/cross arbitration so the
    // fallback position controller cannot recreate the high-drag cone.  The
    // centre itself is the mandatory live velocity-cone axis, so it remains
    // legal and provides sign-symmetric horizontal damping without pursuit.
    IF ALONG_AERO_BRAKE_LOW_SPEED_SETTLE_ACTIVE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET STEERING_THRUST TO FINAL_THRUST_SAFETY_AXIS:NORMALIZED
            * MAX(STEERING_THRUST:MAG, 0.001).
    }
    // A command projected inside the load cone is insufficient when its centre
    // (live surface retrograde) rotates quickly near zero horizontal velocity.
    // Use the measured physical angle as a barrier and smoothly pull the target
    // toward the cone centre before the 30-degree observer limit is reached.
    // This changes direction only; the mandatory continuous throttle floor is
    // applied below and remains invariant.
    LOCAL ACTUAL_CONE_GUARD_ANGLE IS VANG(SHIP:FACING:VECTOR,
        FINAL_THRUST_SAFETY_AXIS).
    LOCAL ACTUAL_CONE_GUARD_BLEND IS CLAMP(
        (ACTUAL_CONE_GUARD_ANGLE
            - TERMINAL_ACTUAL_CONE_GUARD_START_DEGREES)
        / MAX(TERMINAL_ACTUAL_CONE_GUARD_FULL_DEGREES
            - TERMINAL_ACTUAL_CONE_GUARD_START_DEGREES, 0.1), 0, 1).
    IF ACTUAL_CONE_GUARD_BLEND > 0 {
        LOCAL GUARDED_STEERING_AXIS IS
            (STEERING_THRUST:NORMALIZED
                * (1 - ACTUAL_CONE_GUARD_BLEND)
            + FINAL_THRUST_SAFETY_AXIS:NORMALIZED
                * ACTUAL_CONE_GUARD_BLEND):NORMALIZED.
        SET STEERING_THRUST TO GUARDED_STEERING_AXIS
            * MAX(STEERING_THRUST:MAG, 0.001).
        SET STEERING_THRUST TO CONSTRAIN_THRUST_VECTOR(STEERING_THRUST,
            FINAL_THRUST_SAFETY_AXIS, ACTIVE_COMMAND_CONE_DEGREES).
    }
    LOCAL TILT_CMD IS VANG(STEERING_THRUST, UP_VEC).
    LOCAL THROTTLE_CMD IS CLAMP(SHIP:MASS * DESIRED_THRUST:MAG /
        MAX(SHIP:AVAILABLETHRUST, 0.001), 0, 1).
    // The solved main segment is a single continuous burn.  Seventy-five
    // percent is the nominal floor above 2 km; guidance may increase smoothly
    // to 100 percent for correction.  Below 2 km the continuous controller may
    // reduce thrust, but cannot return to zero before the one final capture
    // cutoff after the loop exits. The mandatory 2 km boundary is Kerbin
    // sea-level altitude, not the lower hook point. Run 70 crossed hook height
    // 2 km while the vessel was still at 2011.7 m ASL and illegally released
    // this floor one sample early.
    IF SHIP:ALTITUDE > TERMINAL_WAYPOINT_HEIGHT {
        LOCAL MAIN_BRAKE_THROTTLE_FLOOR IS
            TERMINAL_NOMINAL_THRUST_FRACTION
            + (1 - TERMINAL_NOMINAL_THRUST_FRACTION)
                * MAIN_ALONG_BRAKE_BLEND.
        LOCAL MAIN_STREAMLINE_REQUESTED_THROTTLE IS
            THROTTLE_CMD * MAIN_ALONG_COAST_THROTTLE_SCALE.
        LOCAL MAIN_REQUESTED_THROTTLE IS CLAMP(MAX(
            MAIN_STREAMLINE_REQUESTED_THROTTLE,
            MAIN_BRAKE_THROTTLE_FLOOR),
            TERMINAL_NOMINAL_THRUST_FRACTION, 1).
        LOCAL MAIN_CORRECTION_ALIGNMENT IS VANG(
            SHIP:FACING:VECTOR, STEERING_THRUST).
        LOCAL MAIN_CORRECTION_ALIGNMENT_BLEND IS CLAMP(
            (MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES
                - MAIN_CORRECTION_ALIGNMENT)
            / MAX(MAIN_CORRECTION_ZERO_ALIGNMENT_DEGREES
                - MAIN_CORRECTION_FULL_ALIGNMENT_DEGREES, 0.1), 0, 1).
        SET THROTTLE_CMD TO TERMINAL_NOMINAL_THRUST_FRACTION
            + (MAIN_REQUESTED_THROTTLE
                - TERMINAL_NOMINAL_THRUST_FRACTION)
                * MAIN_CORRECTION_ALIGNMENT_BLEND.
        // Run 64 proved that a smooth sub-75% "descent recovery" is still a
        // hard G-03 failure and removes horizontal authority exactly where the
        // endpoint needs it.  Keep the solved continuous floor invariant here.
        // If the vertical state becomes infeasible, the observer must reject
        // the run; guidance may not conceal it with a low-throttle coast.
    } ELSE {
        SET THROTTLE_CMD TO CLAMP(MAX(THROTTLE_CMD,
            TERMINAL_MIN_CONTINUOUS_THROTTLE),
            TERMINAL_MIN_CONTINUOUS_THROTTLE, 1).
    }
    SET FLIGHT_STEERING_CMD TO LOOKDIRUP(STEERING_THRUST,
        STEERING_ROLL_REFERENCE).
    SET FLIGHT_THROTTLE_CMD TO THROTTLE_CMD.

    LOCAL GUIDANCE_PHASE IS "TRAJECTORY".
    IF H_CORRIDOR_MODE
        AND HOOK_HEIGHT <= TERMINAL_WAYPOINT_HEIGHT {
        SET GUIDANCE_PHASE TO "H_STOPPING".
    }
    IF CAPTURE_ALIGN_MODE { SET GUIDANCE_PHASE TO "H_ALIGN". }
    IF HIGH_ENERGY_BRAKE_MODE { SET GUIDANCE_PHASE TO "H_BRAKE". }
    IF WAYPOINT_COAST_MODE { SET GUIDANCE_PHASE TO "WAYPOINT_COAST". }
    IF WAYPOINT_TRIM_ACTIVE { SET GUIDANCE_PHASE TO "WAYPOINT_TRIM". }
    IF WAYPOINT_CENTER_BRAKE_MODE AND WAYPOINT_TRIM_ACTIVE {
        SET GUIDANCE_PHASE TO "WAYPOINT_CENTER_BRAKE".
    }
    IF WAYPOINT_FINAL_COAST_MODE {
        SET GUIDANCE_PHASE TO "WAYPOINT_FINAL_COAST".
    }
    IF WAYPOINT_POST_UPRIGHT_TRIM_ACTIVE {
        SET GUIDANCE_PHASE TO "WAYPOINT_POST_UPRIGHT_TRIM".
    }
    IF HORIZONTAL_SETTLE_MODE { SET GUIDANCE_PHASE TO "H_SETTLE". }
    IF PID_MODE { SET GUIDANCE_PHASE TO "PID_TERMINAL". }
    IF FINAL_ALIGN_MODE { SET GUIDANCE_PHASE TO "FINAL_ALIGN". }
    IF FINAL_DESCENT_ARMED { SET GUIDANCE_PHASE TO "VERTICAL_CAPTURE". }
    IF NOW - LAST_LOG >= TELEMETRY_PERIOD {
        SET GRID_FIN_APPLIED_DEGREES TO
            BOOSTER_GRID_FIN_APPLIED_DEPLOYMENT().
        LOCAL DIAG_ALONG_POS IS VDOT(WAYPOINT_CONTROL_H_POS,
            CONTROL_ALONG_DIRECTION).
        LOCAL DIAG_ALONG_VEL IS VDOT(HORIZONTAL_VEL,
            CONTROL_ALONG_DIRECTION).
        LOCAL DIAG_CROSS_POS IS WAYPOINT_CONTROL_H_POS
            - CONTROL_ALONG_DIRECTION * DIAG_ALONG_POS.
        LOCAL DIAG_CROSS_VEL IS HORIZONTAL_VEL
            - CONTROL_ALONG_DIRECTION * DIAG_ALONG_VEL.
        LOCAL DIAG_CROSS_RADIAL_VEL IS 0.
        LOCAL DIAG_CROSS_RADIAL_ACCEL IS 0.
        LOCAL DIAG_CROSS_ACTUAL_THRUST IS 0.
        LOCAL DIAG_CROSS_AERO_ACCEL IS 0.
        LOCAL DIAG_CROSS_RAW_AERO_ACCEL IS 0.
        LOCAL DIAG_CROSS_MEASURED_ACCEL IS 0.
        LOCAL DIAG_CROSS_STEERING_AXIS IS 0.
        // Run 252 proved that a fixed roll reference alone does not prevent
        // the low-altitude attitude divergence. Split body-axis roll from
        // transverse pitch/yaw rate before changing the cooked-steering
        // profile, and record whether either roll projection becomes
        // geometrically ill-conditioned.
        LOCAL DIAG_BODY_AXIS IS SHIP:FACING:VECTOR:NORMALIZED.
        LOCAL DIAG_ANGULAR_VELOCITY IS SHIP:ANGULARVEL.
        LOCAL DIAG_AXIAL_ANGULAR_RATE_DEG IS VDOT(
            DIAG_ANGULAR_VELOCITY, DIAG_BODY_AXIS)
            * CONSTANT:RADTODEG.
        LOCAL DIAG_TRANSVERSE_ANGULAR_RATE_DEG IS VXCL(
            DIAG_BODY_AXIS, DIAG_ANGULAR_VELOCITY):MAG
            * CONSTANT:RADTODEG.
        LOCAL DIAG_BODY_ROLL_REFERENCE IS VXCL(
            DIAG_BODY_AXIS, STEERING_ROLL_REFERENCE).
        LOCAL DIAG_COMMAND_ROLL_REFERENCE IS VXCL(
            STEERING_THRUST:NORMALIZED, STEERING_ROLL_REFERENCE).
        LOCAL DIAG_ROLL_ERROR_DEG IS -1.
        IF DIAG_BODY_ROLL_REFERENCE:MAG > 0.001 {
            SET DIAG_ROLL_ERROR_DEG TO VANG(
                SHIP:FACING:TOPVECTOR,
                DIAG_BODY_ROLL_REFERENCE:NORMALIZED).
        }
        LOCAL DIAG_FIXED_ROLL_REFERENCE_ACTIVE IS
            WAYPOINT_FINAL_COAST_MODE OR FINAL_DESCENT_ARMED.
        IF DIAG_CROSS_POS:MAG > 0.1 {
            SET DIAG_CROSS_RADIAL_VEL TO VDOT(
                DIAG_CROSS_POS:NORMALIZED, DIAG_CROSS_VEL).
            SET DIAG_CROSS_RADIAL_ACCEL TO VDOT(
                DIAG_CROSS_POS:NORMALIZED, H_ACCEL).
            SET DIAG_CROSS_ACTUAL_THRUST TO VDOT(
                DIAG_CROSS_POS:NORMALIZED, SHIP:FACING:VECTOR)
                * SHIP:THRUST / MAX(SHIP:MASS,0.001).
            SET DIAG_CROSS_AERO_ACCEL TO VDOT(
                DIAG_CROSS_POS:NORMALIZED,
                POWERED_FILTERED_HORIZONTAL_AERO_ACCEL).
            SET DIAG_CROSS_RAW_AERO_ACCEL TO VDOT(
                DIAG_CROSS_POS:NORMALIZED,
                POWERED_HORIZONTAL_AERO_RAW_SAMPLE).
            SET DIAG_CROSS_MEASURED_ACCEL TO VDOT(
                DIAG_CROSS_POS:NORMALIZED, POWERED_MEASURED_ACCEL).
            SET DIAG_CROSS_STEERING_AXIS TO VDOT(
                DIAG_CROSS_POS:NORMALIZED,
                STEERING_THRUST:NORMALIZED).
        }
        LOG MISSION_ID + ",GUIDANCE_VECTOR," + ROUND(NOW,3)
            + ",h=" + ROUND(HOOK_HEIGHT,2)
            + ",alongPos=" + ROUND(DIAG_ALONG_POS,2)
            + ",approachOffsetBlend="
                + ROUND(LIVE_APPROACH_OFFSET_BLEND,3)
            + ",approachOffsetFinalBlend="
                + ROUND(LIVE_APPROACH_OFFSET_FINAL_BLEND_STATE,3)
            + ",approachOffsetRangeAtLatch="
                + ROUND(LIVE_APPROACH_OFFSET_RANGE_AT_LATCH,2)
            + ",approachOffsetPostBlend="
                + ROUND(LIVE_APPROACH_OFFSET_POST_BLEND,3)
            + ",finalAlignEntryRatio="
                + ROUND(FINAL_ALIGN_ENTRY_RATIO,4)
            + ",earlyAeroBrakeLatched="
                + TERMINAL_EARLY_AERO_BRAKE_LATCHED
            + ",earlyAeroBrakeEntryRatio="
                + ROUND(TERMINAL_EARLY_AERO_BRAKE_ENTRY_RATIO,5)
            + ",earlyAeroBrakeFloor="
                + ROUND(TERMINAL_EARLY_AERO_BRAKE_FLOOR,3)
            + ",finalAlignActivePositionGain="
                + ROUND(FINAL_ALIGN_ACTIVE_POSITION_GAIN,4)
            + ",finalAlignTerminalLatched="
                + FINAL_ALIGN_TERMINAL_PHASE_LATCHED
            + ",finalAlignTerminalEntryRatio="
                + ROUND(FINAL_ALIGN_TERMINAL_ENTRY_RATIO,4)
            + ",finalAlignTerminalAeroBrakeHoldScale="
                + ROUND(FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_SCALE,4)
            + ",finalAlignTerminalAeroBrakeHoldEndHeight="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_AERO_BRAKE_HOLD_END_HEIGHT,1)
            + ",finalAlignTerminalAeroBrakeTailCeiling="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_AERO_BRAKE_TAIL_CEILING,3)
            + ",finalAlignTerminalFiniteTime="
                + FINAL_ALIGN_TERMINAL_FINITE_TIME_ACTIVE
            + ",finalAlignTerminalTgo="
                + ROUND(FINAL_ALIGN_TERMINAL_TGO,3)
            + ",finalAlignTerminalControlTgo="
                + ROUND(FINAL_ALIGN_TERMINAL_CONTROL_TGO,3)
            + ",finalAlignTerminalControlPos="
                + ROUND(FINAL_ALIGN_TERMINAL_CONTROL_POS:MAG,3)
            + ",finalAlignTerminalSignedControlPos="
                + ROUND(VDOT(FINAL_ALIGN_TERMINAL_CONTROL_POS,
                    CONTROL_ALONG_DIRECTION),3)
            + ",finalAlignTerminalResponseLead="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_ACTIVE_RESPONSE_LEAD_SECONDS,3)
            + ",finalAlignTerminalPositionBias="
                + ROUND(FINAL_ALIGN_TERMINAL_ACTIVE_POSITION_BIAS,3)
            + ",finalAlignHighStart4kmPositionBias="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_HIGH_START_4KM_POSITION_BIAS,3)
            + ",finalAlignLowRangeLatched="
                + FINAL_ALIGN_TERMINAL_LOW_RANGE_LATCHED
            + ",finalAlignLowRangeAtLatch="
                + ROUND(FINAL_ALIGN_TERMINAL_LOW_RANGE_AT_LATCH,3)
            + ",finalAlignLowRangePositionBias="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_LOW_RANGE_POSITION_BIAS,3)
            + ",finalAlignMiddleRangeLatched="
                + FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_LATCHED
            + ",finalAlignMiddleProjectedRange="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_MIDDLE_PROJECTED_RANGE_AT_LATCH,
                    3)
            + ",finalAlignMiddleRangePositionBias="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_POSITION_BIAS,3)
            + ",finalAlignMiddleRangeFamilyBlend="
                + ROUND(FINAL_ALIGN_TERMINAL_MIDDLE_FAMILY_BLEND,3)
            + ",finalAlignMiddleRangeHeightBlend="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_MIDDLE_RANGE_HEIGHT_BLEND,3)
            + ",finalAlignRangeBiasHeightBlend="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_RANGE_BIAS_HEIGHT_BLEND,
                    3)
            + ",finalAlignRangeFamilyBlend="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_RANGE_FAMILY_BLEND,3)
            + ",physicalAlongPos="
                + ROUND(VDOT(
                    HORIZONTAL_POS, CONTROL_ALONG_DIRECTION),3)
            + ",physicalCrossPos="
                + ROUND((HORIZONTAL_POS
                    - CONTROL_ALONG_DIRECTION * VDOT(
                        HORIZONTAL_POS,
                        CONTROL_ALONG_DIRECTION)):MAG,3)
            + ",finalAlignTerminalVelocityCoefficient="
                + ROUND(
                    FINAL_ALIGN_TERMINAL_ACTIVE_VELOCITY_COEFFICIENT,3)
            + ",finalAlignPrecommitSettle="
                + FINAL_ALIGN_PRECOMMIT_SETTLE
            + ",finalAlignFiniteTimeStartBlend="
                + ROUND(FINAL_ALIGN_FINITE_TIME_START_BLEND,3)
            + ",finalAlignFiniteTimeMediumStartBlend="
                + ROUND(FINAL_ALIGN_FINITE_TIME_MEDIUM_START_BLEND,3)
            + ",finalAlignFiniteTimeHighStartBlend="
                + ROUND(FINAL_ALIGN_FINITE_TIME_HIGH_START_BLEND,3)
            + ",finalAlignFiniteTimeStartHeight="
                + ROUND(FINAL_ALIGN_FINITE_TIME_START_HEIGHT,1)
            + ",postWaypointEngineBlend="
                + ROUND(RETURN_ENGINE_POST_WAYPOINT_BLEND,3)
            + ",finalCaptureDirectAeroReset="
                + FINAL_CAPTURE_DIRECT_AERO_RESET
            + ",engineMaxAccel="
                + ROUND(RETURN_ENGINE_CURRENT_MAX_ACCEL,2)
            + ",alongVel=" + ROUND(DIAG_ALONG_VEL,2)
            + ",alongRef=" + ROUND(MAIN_ALONG_REFERENCE_SPEED,2)
            + ",alongCoast=" + ROUND(MAIN_ALONG_COAST_BLEND,3)
            + ",alongStreamlineScale="
                + ROUND(MAIN_ALONG_COAST_THROTTLE_SCALE,3)
            + ",alongCoastAxis="
                + ROUND(MAIN_ALONG_COAST_STEERING_AXIS_DIAG,4)
            + ",alongAeroBrake="
                + ROUND(ALONG_AERO_BRAKE_BLEND_STATE,4)
            + ",alongAeroBrakeSettle="
                + ALONG_AERO_BRAKE_LOW_SPEED_SETTLE_ACTIVE
            + ",alongAeroBrakeHeight="
                + ROUND(MAIN_ALONG_AERO_BRAKE_HEIGHT_BLEND,3)
            + ",alongAeroBrakeTgo="
                + ROUND(MAIN_ALONG_AERO_BRAKE_TGO,2)
            + ",alongAeroBrakeTimeRequired="
                + ROUND(MAIN_ALONG_AERO_BRAKE_TIME_REQUIRED,2)
            + ",alongAeroBrakeRangeRequired="
                + ROUND(MAIN_ALONG_AERO_BRAKE_RANGE_REQUIRED,2)
            + ",alongAeroBrakeStageTgo="
                + ROUND(MAIN_ALONG_AERO_BRAKE_STAGE_TGO,2)
            + ",alongAeroBrakeStageRequired="
                + ROUND(MAIN_ALONG_AERO_BRAKE_STAGE_REQUIRED,2)
            + ",alongAeroBrakeRequired="
                + ROUND(MAIN_ALONG_AERO_BRAKE_REQUIRED,2)
            + ",alongAeroBrakeRealized="
                + ROUND(MAIN_ALONG_AERO_BRAKE_REALIZED,2)
            + ",alongAeroBrakeError="
                + ROUND(MAIN_ALONG_AERO_BRAKE_ERROR,2)
            + ",alongAeroBrakeAxis="
                + ROUND(MAIN_ALONG_AERO_BRAKE_AXIS_DIAG,4)
            + ",alongBrake=" + ROUND(MAIN_ALONG_BRAKE_BLEND,3)
            + ",alongBrakeApplied="
                + MAIN_ALONG_BRAKE_APPLIED_DIAG
            + ",alongBrakeAppliedAxis="
                + ROUND(MAIN_ALONG_BRAKE_APPLIED_AXIS_DIAG,4)
            + ",alongBrakeHeight="
                + ROUND(MAIN_ALONG_BRAKE_HEIGHT_BLEND,3)
            + ",alongBrakeAuthority="
                + ROUND(MAIN_ALONG_BRAKE_AUTHORITY_LIMIT,3)
            + ",alongBrakeLate="
                + ROUND(MAIN_ALONG_BRAKE_LATE_BLEND,3)
            + ",alongBrakeRequired="
                + ROUND(MAIN_ALONG_BRAKE_REQUIRED_DECEL,2)
            + ",alongBrakeAvailable="
                + ROUND(MAIN_ALONG_BRAKE_AVAILABLE_DECEL,2)
            + ",alongBrakePressure="
                + ROUND(MAIN_ALONG_BRAKE_PRESSURE,3)
            + ",alongBrakeEngineProjection="
                + ROUND(MAIN_ALONG_BRAKE_ENGINE_PROJECTION,3)
            + ",alongBrakeAero="
                + ROUND(MAIN_ALONG_BRAKE_AERO_DECEL,2)
            + ",measuredAlongAccel="
                + ROUND(MAIN_MEASURED_ALONG_ACCEL,2)
            + ",predictedAlongPos="
                + ROUND(MAIN_PREDICTED_ALONG_POS,2)
            + ",predictedAlongVel="
                + ROUND(MAIN_PREDICTED_ALONG_VEL,2)
            + ",predictedVerticalVel="
                + ROUND(MAIN_PREDICTED_VERTICAL_VEL,2)
            + ",alongAccel=" + ROUND(VDOT(H_ACCEL,
                CONTROL_ALONG_DIRECTION),2)
            + ",fixedStop=" + FIXED_STOP_ACTIVE
            + ",fixedCommitted=" + FIXED_STOP_COMMITTED
            + ",fixedCommitEdge=" + FIXED_STOP_COMMITTED_THIS_TICK
            + ",fixedEdge=" + FIXED_STOP_EDGE_ACTIVE
            + ",fixedFeasible=" + FIXED_STOP_FEASIBLE_DIAG
            + ",fixedUsable="
                + ROUND(FIXED_STOP_USABLE_RANGE_DIAG,2)
            + ",fixedRequired="
                + ROUND(FIXED_STOP_REQUIRED_DECEL_DIAG,2)
            + ",fixedRawRequired="
                + ROUND(FIXED_STOP_RAW_REQUIRED_DECEL_DIAG,2)
            + ",fixedAuthority="
                + ROUND(FIXED_STOP_AUTHORITY_DIAG,2)
            + ",fixedSyncExcess="
                + ROUND(FIXED_STOP_SYNC_EXCESS_DIAG,2)
            + ",crossStop=" + CROSS_STOP_ACTIVE
            + ",crossStopCommitted=" + CROSS_STOP_COMMITTED
            + ",crossStopCompleted=" + CROSS_STOP_COMPLETED
            + ",crossStopCommitEdge="
                + CROSS_STOP_COMMITTED_THIS_TICK
            + ",crossStopSuppress=" + CROSS_STOP_SUPPRESS_ACTIVE
            + ",crossStopPos=" + ROUND(CROSS_STOP_POS_DIAG,2)
            + ",crossStopVel=" + ROUND(CROSS_STOP_VEL_DIAG,2)
            + ",crossStopUsable="
                + ROUND(CROSS_STOP_USABLE_RANGE_DIAG,2)
            + ",crossStopRequired="
                + ROUND(CROSS_STOP_REQUIRED_DECEL_DIAG,2)
            + ",crossStopFinish=" + CROSS_STOP_FINISH_ACTIVE
            + ",crossStopFinishAccel="
                + ROUND(CROSS_STOP_FINISH_ACCEL_DIAG,3)
            + ",crossAeroBrake="
                + CROSS_AERO_BRAKE_STEERING_ACTIVE
            + ",crossAeroBrakeAxis="
                + ROUND(CROSS_AERO_BRAKE_STEERING_AXIS_DIAG,4)
            + ",crossAeroBrakeBlend="
                + ROUND(CROSS_AERO_BRAKE_RELEASE_BLEND_DIAG,4)
            + ",crossAeroBrakeRequest="
                + ROUND(CROSS_AERO_BRAKE_REQUEST_ACCEL_DIAG,3)
            + ",crossAeroBrakeRealized="
                + ROUND(CROSS_AERO_BRAKE_REALIZED_DECEL_DIAG,3)
            + ",crossAeroBrakeResidual="
                + ROUND(CROSS_AERO_BRAKE_RESIDUAL_DEMAND_DIAG,3)
            + ",crossAeroBrakeRelease="
                + CROSS_AERO_BRAKE_RELEASE_STARTED
            + ",commandCone=" + ROUND(VANG(STEERING_THRUST,
                FINAL_THRUST_SAFETY_AXIS),2)
            + ",commandAlongAxis=" + ROUND(VDOT(
                STEERING_THRUST:NORMALIZED,
                CONTROL_ALONG_DIRECTION),4)
            + ",activeConeLimit="
                + ROUND(ACTIVE_COMMAND_CONE_DEGREES,2)
            + ",actualCone=" + ROUND(ACTUAL_CONE_GUARD_ANGLE,2)
            + ",coneGuard=" + ROUND(ACTUAL_CONE_GUARD_BLEND,3)
            + ",actualTilt=" + ROUND(VANG(SHIP:FACING:VECTOR,
                UP_VEC),2)
            + ",axialAngularRate="
                + ROUND(DIAG_AXIAL_ANGULAR_RATE_DEG,3)
            + ",transverseAngularRate="
                + ROUND(DIAG_TRANSVERSE_ANGULAR_RATE_DEG,3)
            + ",rollError=" + ROUND(DIAG_ROLL_ERROR_DEG,2)
            + ",bodyRollReferenceProjection="
                + ROUND(DIAG_BODY_ROLL_REFERENCE:MAG,4)
            + ",commandRollReferenceProjection="
                + ROUND(DIAG_COMMAND_ROLL_REFERENCE:MAG,4)
            + ",fixedRollReference="
                + DIAG_FIXED_ROLL_REFERENCE_ACTIVE
            + ",nearNetSteeringTuned="
                + FINAL_CAPTURE_NEAR_NET_STEERING_TUNED
            + ",actualAlongAxis=" + ROUND(VDOT(
                SHIP:FACING:VECTOR, CONTROL_ALONG_DIRECTION),4)
            + ",actualAlongThrust=" + ROUND(VDOT(
                SHIP:FACING:VECTOR, CONTROL_ALONG_DIRECTION)
                * SHIP:THRUST / MAX(SHIP:MASS,0.001),2)
            + ",actualVerticalThrust=" + ROUND(VDOT(
                SHIP:FACING:VECTOR, UP_VEC)
                * SHIP:THRUST / MAX(SHIP:MASS,0.001),2)
            + ",aeroAlong=" + ROUND(VDOT(
                POWERED_FILTERED_HORIZONTAL_AERO_ACCEL,
                CONTROL_ALONG_DIRECTION),2)
            + ",aeroDt=" + ROUND(POWERED_MEASUREMENT_DT,3)
            + ",aeroRawMag="
                + ROUND(POWERED_HORIZONTAL_AERO_RAW_SAMPLE:MAG,2)
            + ",aeroSampleMag="
                + ROUND(POWERED_HORIZONTAL_AERO_SAMPLE:MAG,2)
            + ",aeroRawAlong=" + ROUND(VDOT(
                POWERED_HORIZONTAL_AERO_RAW_SAMPLE,
                CONTROL_ALONG_DIRECTION),2)
            + ",aeroUp=" + ROUND(
                POWERED_FILTERED_VERTICAL_AERO_ACCEL,2)
            + ",hybridActive=" + HYBRID_CORRIDOR_ACTIVE
            + ",hybridHRef="
                + ROUND(HYBRID_HORIZONTAL_REFERENCE,2)
            + ",hybridHResidual="
                + ROUND(HYBRID_HORIZONTAL_RESIDUAL,2)
            + ",hybridDownRef="
                + ROUND(HYBRID_DOWN_REFERENCE,2)
            + ",hybridDownResidual="
                + ROUND(HYBRID_DOWN_RESIDUAL,2)
            + ",gridFinNominal="
                + ROUND(GRID_FIN_NOMINAL_COMMAND,2)
            + ",gridFinDesired="
                + ROUND(GRID_FIN_DESIRED_COMMAND,2)
            + ",gridFinCommand="
                + ROUND(GRID_FIN_DEPLOYMENT_COMMAND,2)
            + ",gridFinApplied="
                + ROUND(GRID_FIN_APPLIED_DEGREES,2)
            + ",crossPos=" + ROUND(DIAG_CROSS_POS:MAG,2)
            + ",crossRadialVel=" + ROUND(DIAG_CROSS_RADIAL_VEL,2)
            + ",crossRadialAccel=" + ROUND(DIAG_CROSS_RADIAL_ACCEL,2)
            + ",crossActualThrust=" + ROUND(DIAG_CROSS_ACTUAL_THRUST,2)
            + ",crossAero=" + ROUND(DIAG_CROSS_AERO_ACCEL,2)
            + ",crossAeroRaw=" + ROUND(DIAG_CROSS_RAW_AERO_ACCEL,2)
            + ",crossMeasured=" + ROUND(DIAG_CROSS_MEASURED_ACCEL,2)
            + ",crossSteeringAxis=" + ROUND(DIAG_CROSS_STEERING_AXIS,4)
            TO "0:/cz10b/telemetry.csv".
        WRITE_TELEMETRY(GUIDANCE_PHASE, MISSION_ID, SHIP:ALTITUDE, HOOK_HEIGHT,
            VERTICAL_V, HORIZONTAL_VEL:MAG, HORIZONTAL_POS:MAG,
            THROTTLE_CMD, TILT_CMD).
        SET LAST_LOG TO NOW.
    }
    PRINT "phase       " + GUIDANCE_PHASE + " / "
        + NET_STATE(RECOVERY_SHIP) + "       " AT(0,12).
    PRINT "hook height " + ROUND(HOOK_HEIGHT,1) + " m    " AT(0,13).
    PRINT "vertical    " + ROUND(VERTICAL_V,2) + " m/s  " AT(0,14).
    PRINT "cross error " + ROUND(HORIZONTAL_POS:MAG,1) + " m    " AT(0,15).
    PRINT "throttle    " + ROUND(THROTTLE_CMD,3) + "      " AT(0,16).

    IF AG10 {
        PRINT "MANUAL ABORT (AG10)".
        BREAK.
    }
    WAIT 0.02.
}

SET FLIGHT_THROTTLE_CMD TO 0.
LOCAL GRID_FIN_CAPTURE_STOWED IS
    SET_BOOSTER_GRID_FIN_DEPLOYMENT(0).
SET FLIGHT_STEERING_CMD TO UP.
RCS OFF.
PRINT "CAPTURE COMPLETE" AT(0,18).
LOG MISSION_ID + ",CAPTURE," + ROUND(TIME:SECONDS,3) TO "0:/cz10b/telemetry.csv".
WAIT UNTIL FALSE.
