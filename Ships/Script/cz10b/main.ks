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
    IF HASTARGET {
        // On a slow first physics frame the observer may have already separated
        // the rail and bound the platform before this boot program starts. Reuse
        // that target instead of enumerating vessels while one is unpacking.
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
LOG "mission,phase,ut,altitude,hook_height,v_vertical,v_horizontal,h_error,throttle,tilt,mass,max_thrust"
    TO "0:/cz10b/telemetry.csv".

SAS OFF.
RCS ON.
BRAKES OFF.
LOCK THROTTLE TO 0.

PRINT "Target locked: " + RECOVERY_SHIP:NAME.
PRINT "Booster probe core must be the craft root.".
FROM { LOCAL COUNTDOWN IS COUNTDOWN_SECONDS. } UNTIL COUNTDOWN = 0 STEP { SET COUNTDOWN TO COUNTDOWN - 1. } DO {
    PRINT "T-" + COUNTDOWN AT(0,8).
    WAIT 1.
}

// Start the engine at zero throttle while the real TT18-A clamps still carry
// the vehicle. Increase the command linearly over five seconds and release only
// after measured thrust-to-weight strictly exceeds the configured threshold.
LOCK STEERING TO UP.
LOCK THROTTLE TO 0.
LOCAL ENGINE_LOAD_DEADLINE IS TIME:SECONDS + 15.
WAIT UNTIL SHIP:ENGINES:LENGTH > 0
    OR TIME:SECONDS >= ENGINE_LOAD_DEADLINE.
LOCAL MISSION_ENGINES IS SHIP:ENGINES.
IF MISSION_ENGINES:LENGTH = 0 {
    PRINT "ABORT: engine modules did not load" AT(0,12).
    LOCK THROTTLE TO 0.
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
    LOCK THROTTLE TO 0.
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
    LOCK THROTTLE TO IGNITION_RAMP_FRACTION.
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
    LOCK THROTTLE TO 0.
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
    LOCK THROTTLE TO MIN(ASCENT_THROTTLE_COMMAND, ASCENT_RAMP_LIMIT).
    LOCK STEERING TO HEADING(ASCENT_HEADING, PITCH_CMD).

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

LOCK THROTTLE TO 0.
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
LOCK STEERING TO SEPARATION_ATTITUDE.
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
    LOCK THROTTLE TO 0.
    SHUTDOWN.
}
WAIT 0.6.
FOR ENGINE IN MISSION_ENGINES {
    IF ENGINE:NAME = BOOSTER_ENGINE_PART_NAME {
        SET ENGINE:THRUSTLIMIT TO 100.
    }
}
LOCK THROTTLE TO 0.
PRINT "STAGE SEPARATION - BOOSTER GUIDANCE ACTIVE" AT(0,11).
BRAKES ON. // four stock airbrakes serve as the grid-fin analogue

// 1) Rate-damped ascent coast.  Holding the separation attitude made the
// reaction wheels chase an orientation through the rapidly changing surface
// frame, while a fully passive stage was aerodynamically unstable.  kOS's
// special "kill" target commands angular-rate damping without chasing a
// changing direction.
PRINT "RATE-DAMPED ASCENT COAST" AT(0,11).
LOCK THROTTLE TO 0.
SAS OFF.
RCS ON.
LOCK STEERING TO "kill".
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
    LOCK STEERING TO LOOKDIRUP(FLIP_COMMAND, SHIP:UP:VECTOR).
    WAIT 0.05.
}
LOCK THROTTLE TO 0.
LOCK STEERING TO LOOKDIRUP(-SHIP:VELOCITY:SURFACE:NORMALIZED,
    SHIP:UP:VECTOR).

// 2) Below 40 km, use a retrograde safety burn only if horizontal speed exceeds
// the configured entry ceiling.  The flight value is 1500 m/s, so the nominal
// trajectory preserves vertical energy and propellant for the efficient,
// height-indexed lateral stop beginning at 30 km.
WAIT UNTIL SHIP:ALTITUDE <= ENTRY_DECEL_HEIGHT.
PRINT "40 KM RETROGRADE ENTRY BURN" AT(0,11).
LOCAL ENTRY_H_SPEED IS VXCL(SHIP:UP:VECTOR:NORMALIZED,
    SHIP:VELOCITY:SURFACE):MAG.
UNTIL ENTRY_H_SPEED <= ENTRY_HORIZONTAL_SPEED
      OR SHIP:ALTITUDE <= TERMINAL_GUIDANCE_START_HEIGHT {
    LOCK STEERING TO LOOKDIRUP(-SHIP:VELOCITY:SURFACE:NORMALIZED,
        SHIP:UP:VECTOR).
    LOCK THROTTLE TO 1.
    SET ENTRY_H_SPEED TO VXCL(SHIP:UP:VECTOR:NORMALIZED,
        SHIP:VELOCITY:SURFACE):MAG.
    IF TIME:SECONDS - LAST_LOG >= TELEMETRY_PERIOD {
        WRITE_TELEMETRY("ENTRY_BURN", MISSION_ID, SHIP:ALTITUDE, 0,
            SHIP:VERTICALSPEED, ENTRY_H_SPEED,
            RECOVERY_SHIP:GEOPOSITION:DISTANCE, 1, 0).
        SET LAST_LOG TO TIME:SECONDS.
    }
    WAIT 0.02.
}
LOCK THROTTLE TO 0.
UNTIL SHIP:ALTITUDE <= TERMINAL_GUIDANCE_START_HEIGHT {
    LOCK STEERING TO LOOKDIRUP(-SHIP:VELOCITY:SURFACE:NORMALIZED,
        SHIP:UP:VECTOR).
    WAIT 0.02.
}
LOCK STEERING TO UP.
PRINT "30 KM CONSTRAINED TRAJECTORY" AT(0,11).
// GFOLD's minimum-time phase is approximated here by a live suicide-burn gate.
// Measured unpowered acceleration includes KSP's current atmospheric drag, so
// the gate moves lower as the airframe decelerates instead of assuming vacuum.
LOCAL BALLISTIC_PREVIOUS_TIME IS TIME:SECONDS.
LOCAL BALLISTIC_PREVIOUS_V IS SHIP:VELOCITY:SURFACE.
LOCAL FILTERED_VERTICAL_DRAG IS 0.
LOCAL FILTERED_HORIZONTAL_DRAG IS V(0,0,0).
// Start trajectory shaping at 30 km. Throttle remains demand-driven: the
// vertical braking component is capped at 75%, while only measured trajectory
// error can use the remaining vector authority up to 98%.
LOCAL TERMINAL_IGNITION IS TRUE.
LOCAL BALLISTIC_WAS_POWERED IS FALSE.
SET LAST_LOG TO TIME:SECONDS.
UNTIL TERMINAL_IGNITION {
    LOCAL BALLISTIC_NOW IS TIME:SECONDS.
    LOCAL BALLISTIC_DT IS CLAMP(BALLISTIC_NOW - BALLISTIC_PREVIOUS_TIME,
        0.001, 0.1).
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

    LOCAL BALLISTIC_AVAILABLE_ACCEL IS SHIP:AVAILABLETHRUST
        / MAX(SHIP:MASS, 0.001).
    LOCAL BALLISTIC_NET_DECEL IS MAX(BALLISTIC_AVAILABLE_ACCEL
        * TERMINAL_NOMINAL_THRUST_FRACTION
        - BALLISTIC_G + FILTERED_VERTICAL_DRAG, 0.1).
    LOCAL BALLISTIC_STOP_DISTANCE IS MAX(BALLISTIC_VERTICAL_V^2
        - CAPTURE_FINAL_SPEED^2, 0) / (2 * BALLISTIC_NET_DECEL)
        * STOP_DISTANCE_SAFETY.
    LOCAL EFFECTIVE_G IS MAX(BALLISTIC_G - FILTERED_VERTICAL_DRAG, 1).
    LOCAL BALLISTIC_TGO IS COAST_TIME_TO_HEIGHT(BALLISTIC_HEIGHT,
        BALLISTIC_VERTICAL_V, 0, EFFECTIVE_G).
    LOCAL PREDICTED_DISPLACEMENT IS BALLISTIC_H_VEL * BALLISTIC_TGO
        + FILTERED_HORIZONTAL_DRAG * (0.5 * BALLISTIC_TGO^2).
    LOCAL PREDICTED_MISS IS BALLISTIC_H_POS - PREDICTED_DISPLACEMENT.
    // Below 15 km, correct only the predicted footprint error.  The desired
    // velocity remains range/time-to-go rather than zero, so the stage keeps
    // moving downrange until the late powered descent.
    LOCAL MIDCOURSE_ACTIVE IS BALLISTIC_HEIGHT <= MIDCOURSE_START_HEIGHT
        AND BALLISTIC_HEIGHT > BALLISTIC_STOP_DISTANCE + MIDCOURSE_END_MARGIN
        AND PREDICTED_MISS:MAG > MIDCOURSE_PREDICTED_ERROR.
    LOCAL MIDCOURSE_THROTTLE IS 0.
    LOCAL MIDCOURSE_TILT IS 0.
    IF MIDCOURSE_ACTIVE {
        LOCAL MIDCOURSE_DESIRED_H_VEL IS (BALLISTIC_H_POS
            - FILTERED_HORIZONTAL_DRAG * (0.5 * BALLISTIC_TGO^2))
            / MAX(BALLISTIC_TGO, 1).
        LOCAL MIDCOURSE_H_ACCEL IS CLAMPV((MIDCOURSE_DESIRED_H_VEL
            - BALLISTIC_H_VEL) * MIDCOURSE_VELOCITY_GAIN,
            MIDCOURSE_MAX_HORIZONTAL_ACCEL).
        LOCAL MIDCOURSE_VERTICAL_THRUST IS BALLISTIC_G
            * MIDCOURSE_VERTICAL_THRUST_G.
        LOCAL MIDCOURSE_THRUST IS BALLISTIC_UP
            * MIDCOURSE_VERTICAL_THRUST + MIDCOURSE_H_ACCEL.
        SET MIDCOURSE_THROTTLE TO CLAMP(SHIP:MASS
            * MIDCOURSE_THRUST:MAG / MAX(SHIP:AVAILABLETHRUST, 0.001),
            0, 1).
        SET MIDCOURSE_TILT TO VANG(MIDCOURSE_THRUST, BALLISTIC_UP).
        LOCK STEERING TO LOOKDIRUP(MIDCOURSE_THRUST,
            SHIP:FACING:TOPVECTOR).
        LOCK THROTTLE TO MIDCOURSE_THROTTLE.
    } ELSE {
        LOCK STEERING TO UP.
        LOCK THROTTLE TO 0.
    }
    // Thirty kilometres is the planning handover, not an instruction to hold
    // thrust all the way down.  Coast along the predicted footprint and begin
    // the fixed powered trajectory only at the measured suicide-burn gate.
    SET TERMINAL_IGNITION TO BALLISTIC_VERTICAL_V < 0
        AND BALLISTIC_HEIGHT <= BALLISTIC_STOP_DISTANCE
            + BURN_ALTITUDE_MARGIN.

    IF BALLISTIC_NOW - LAST_LOG >= TELEMETRY_PERIOD {
        LOCAL BALLISTIC_PHASE IS "BALLISTIC".
        IF MIDCOURSE_ACTIVE { SET BALLISTIC_PHASE TO "MIDCOURSE". }
        WRITE_TELEMETRY(BALLISTIC_PHASE, MISSION_ID, SHIP:ALTITUDE,
            BALLISTIC_HEIGHT, BALLISTIC_VERTICAL_V,
            BALLISTIC_H_VEL:MAG, PREDICTED_MISS:MAG,
            MIDCOURSE_THROTTLE, MIDCOURSE_TILT).
        SET LAST_LOG TO BALLISTIC_NOW.
    }
    PRINT "ballistic h  " + ROUND(BALLISTIC_HEIGHT,0) + " m    " AT(0,12).
    PRINT "stop gate    " + ROUND(BALLISTIC_STOP_DISTANCE
        + BURN_ALTITUDE_MARGIN,0) + " m    " AT(0,13).
    PRINT "impact miss  " + ROUND(PREDICTED_MISS:MAG,0) + " m    " AT(0,14).
    SET BALLISTIC_PREVIOUS_TIME TO BALLISTIC_NOW.
    SET BALLISTIC_PREVIOUS_V TO SHIP:VELOCITY:SURFACE.
    SET BALLISTIC_WAS_POWERED TO MIDCOURSE_ACTIVE.
    WAIT 0.02.
}

// Build one constrained, height-indexed trajectory from the measured 30 km
// state.  The stage follows this path directly; it does not move the aim point
// or restart a time-to-go solution after each small tracking error.  Only the
// final frame entrance uses local damping.
PRINT "FIXED-TRAJECTORY BURN" AT(0,11).
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
LOCAL PLAN_PROGRESS_RATE0 IS MAX(-PLAN_VERTICAL_V,
    CAPTURE_FINAL_SPEED) / MAX(PLAN_HOOK_HEIGHT
        - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 1).
// Error derivative with respect to normalized height progress.  Matching this
// tangent makes the first guidance command continuous with the incoming flight.
LOCAL PLAN_H_ERROR_SLOPE0 IS -PLAN_H_VEL
    / MAX(PLAN_PROGRESS_RATE0, 0.0001).
LOCAL FUEL_URGENT IS FALSE.
LOCAL WAS_PID_MODE IS FALSE.
LOCAL CAPTURE_ALIGN_MODE IS FALSE.
LOCAL CAPTURE_ALIGN_SPEED_LIMIT IS TERMINAL_ALIGN_SPEED.
LOCAL HORIZONTAL_SETTLE_MODE IS FALSE.
LOCAL WIRE_HOLD_STARTED_AT IS -1.
LOCAL FINAL_ALIGN_MODE IS FALSE.
LOCAL FINAL_ALIGN_STARTED_AT IS -1.
LOCAL FINAL_DESCENT_ARMED IS FALSE.
LOCAL TERMINAL_STEERING_TUNED IS FALSE.
LOCAL FILTERED_H_ACCEL IS V(0,0,0).
SET LAST_LOG TO TIME:SECONDS.

UNTIL HOOK_CAPTURED() {
    LOCAL NOW IS TIME:SECONDS.
    LOCAL DT IS CLAMP(NOW - PREVIOUS_TIME, 0.001, 0.1).
    SET PREVIOUS_TIME TO NOW.

    LOCAL UP_VEC IS SHIP:UP:VECTOR:NORMALIZED.
    LOCAL REL_POS IS NET_POSITION(RECOVERY_SHIP) - SHIP:POSITION.
    LOCAL HEIGHT IS -VDOT(REL_POS, UP_VEC).
    LOCAL HOOK_HEIGHT IS HEIGHT + BOOSTER_HOOK_OFFSET_ALONG_UP(UP_VEC).
    LOCAL HORIZONTAL_POS IS VXCL(UP_VEC, REL_POS).
    LOCAL REL_VEL IS SHIP:VELOCITY:SURFACE.
    LOCAL VERTICAL_V IS VDOT(REL_VEL, UP_VEC).
    LOCAL HORIZONTAL_VEL IS VXCL(UP_VEC, REL_VEL).
    LOCAL G_ACC IS SHIP:BODY:MU / ((SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2).
    LOCAL AVAILABLE_ACC IS SHIP:AVAILABLETHRUST / MAX(SHIP:MASS, 0.001).

    IF NOT TERMINAL_STEERING_TUNED
        AND HOOK_HEIGHT <= TERMINAL_STEERING_TUNE_HEIGHT {
        // kOS computes the torque PID gains from these settling times.  A
        // longer near-field response removes the reaction-wheel limit cycle
        // without changing the translational acceleration or tilt targets.
        SET STEERINGMANAGER:PITCHTS TO TERMINAL_STEERING_PITCH_YAW_TS.
        SET STEERINGMANAGER:YAWTS TO TERMINAL_STEERING_PITCH_YAW_TS.
        SET STEERINGMANAGER:ROLLTS TO TERMINAL_STEERING_ROLL_TS.
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
        AND HORIZONTAL_POS:MAG <= TERMINAL_ALIGN_RANGE {
        SET CAPTURE_ALIGN_MODE TO TRUE.
        LOCAL CAPTURE_ALIGN_RADIAL_SPEED IS 0.
        IF HORIZONTAL_POS:MAG > 0 {
            SET CAPTURE_ALIGN_RADIAL_SPEED TO VDOT(
                HORIZONTAL_POS:NORMALIZED, HORIZONTAL_VEL).
        }
        // This is a one-way *deceleration* handoff.  The old 30 m/s cap could
        // command a stage already approaching at 8-12 m/s to accelerate again
        // inside 300 m, feeding the attitude lag and causing another pass.
        // Keep enough floor for a near-tangential entry to converge; otherwise
        // never ask for more than the inward speed measured at the latch.
        SET CAPTURE_ALIGN_SPEED_LIMIT TO MAX(
            TERMINAL_ALIGN_MIN_SPEED, CAPTURE_ALIGN_RADIAL_SPEED).
    }
    // Enter this state only once.  Holding the latch prevents a small position
    // excursion from switching the outer controller back on during settling.
    IF NOT FINAL_ALIGN_MODE
        AND HOOK_HEIGHT <= FINAL_ALIGN_HEIGHT
        AND HORIZONTAL_POS:MAG <= FINAL_ALIGN_RANGE {
        SET FINAL_ALIGN_MODE TO TRUE.
        SET FINAL_ALIGN_STARTED_AT TO NOW.
        SET VERTICAL_INTEGRAL TO 0.
        PRINT "FINAL ALIGN HOLD" AT(0,11).
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
    // Cubic Hermite reference indexed by lost height.  Because progress comes
    // from altitude rather than a repeatedly extended deadline, the reference
    // reaches the frame entrance exactly once and cannot jump behind the stage.
    // Reach zero horizontal position and speed above the 2 km waypoint. The
    // extra settling height absorbs the long stage's attitude lag; below the
    // endpoint the reference remains at the ship centre, producing the
    // requested nearly vertical final leg instead of a late curved approach.
    LOCAL PLAN_HORIZONTAL_HEIGHT IS MAX(PLAN_HOOK_HEIGHT
        - TERMINAL_HORIZONTAL_PLAN_END_HEIGHT, 1).
    LOCAL PLAN_PROGRESS IS CLAMP((PLAN_HOOK_HEIGHT - HOOK_HEIGHT)
        / PLAN_HORIZONTAL_HEIGHT, 0, 1).
    LOCAL PLAN_PROGRESS2 IS PLAN_PROGRESS^2.
    LOCAL PLAN_PROGRESS3 IS PLAN_PROGRESS^3.
    LOCAL PLAN_H00 IS 2 * PLAN_PROGRESS3 - 3 * PLAN_PROGRESS2 + 1.
    LOCAL PLAN_H10 IS PLAN_PROGRESS3 - 2 * PLAN_PROGRESS2 + PLAN_PROGRESS.
    LOCAL PLAN_REFERENCE_H_POS IS PLAN_H_POS * PLAN_H00
        + PLAN_H_ERROR_SLOPE0 * PLAN_H10.
    LOCAL PLAN_D_ERROR_DS IS PLAN_H_POS
        * (6 * PLAN_PROGRESS2 - 6 * PLAN_PROGRESS)
        + PLAN_H_ERROR_SLOPE0
        * (3 * PLAN_PROGRESS2 - 4 * PLAN_PROGRESS + 1).
    LOCAL PLAN_D2_ERROR_DS2 IS PLAN_H_POS * (12 * PLAN_PROGRESS - 6)
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
        + (HORIZONTAL_POS - PLAN_REFERENCE_H_POS)
            * TERMINAL_PLAN_POSITION_GAIN
        + (PLAN_REFERENCE_H_VEL - HORIZONTAL_VEL)
            * TERMINAL_PLAN_VELOCITY_GAIN.
    SET H_ACCEL TO CLAMPV(H_ACCEL, TERMINAL_MAX_HORIZONTAL_ACCEL).

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
        LOCAL PID_RANGE IS HORIZONTAL_POS:MAG.
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
            SET DESIRED_H_VEL TO HORIZONTAL_POS:NORMALIZED * PID_H_SPEED.
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
        AND HORIZONTAL_POS:MAG <= TERMINAL_ALIGN_SETTLE_ENTRY_RANGE
        AND HORIZONTAL_VEL:MAG <= TERMINAL_ALIGN_SETTLE_ENTRY_SPEED {
        SET HORIZONTAL_SETTLE_MODE TO TRUE.
        SET FILTERED_H_ACCEL TO V(0,0,0).
    }
    IF HORIZONTAL_SETTLE_MODE
        AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET H_ACCEL TO CLAMPV(HORIZONTAL_POS
            * TERMINAL_ALIGN_SETTLE_POSITION_GAIN
            - HORIZONTAL_VEL * TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN,
            TERMINAL_ALIGN_SETTLE_MAX_ACCEL).
        IF HORIZONTAL_POS:MAG > TERMINAL_ALIGN_REACQUIRE_RANGE {
            SET H_ACCEL TO CLAMPV(HORIZONTAL_POS
                * TERMINAL_ALIGN_REACQUIRE_POSITION_GAIN
                - HORIZONTAL_VEL * TERMINAL_ALIGN_SETTLE_VELOCITY_GAIN,
                TERMINAL_ALIGN_REACQUIRE_MAX_ACCEL).
        }
    }

    IF FINAL_ALIGN_MODE AND NOT FINAL_DESCENT_ARMED {
        // The long stage has about a 1.5 s attitude response.  A deliberately
        // slower velocity loop settles that lag while the stage is held above
        // the frame, rather than commanding another full-speed centre pass.
        LOCAL FINAL_STOP_RANGE IS HORIZONTAL_POS:MAG.
        LOCAL FINAL_H_SPEED IS MIN(FINAL_ALIGN_SPEED,
            FINAL_STOP_RANGE * FINAL_ALIGN_POSITION_GAIN).
        LOCAL FINAL_DESIRED_H_VEL IS V(0,0,0).
        IF HORIZONTAL_POS:MAG > 0.25 {
            SET FINAL_DESIRED_H_VEL TO HORIZONTAL_POS:NORMALIZED
                * FINAL_H_SPEED.
        }
        SET H_ACCEL TO CLAMPV((FINAL_DESIRED_H_VEL - HORIZONTAL_VEL)
            * FINAL_ALIGN_VELOCITY_GAIN,
            TERMINAL_MAX_HORIZONTAL_ACCEL).
    }

    LOCAL STAGE_TILT IS VANG(SHIP:FACING:FOREVECTOR, UP_VEC).
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
        // Vertical capture is a one-way mode, but it still owns a deliberately
        // slow centring field. Pure velocity damping allowed a 6-7 m committed
        // alignment to drift beyond the cable cradle's short-axis travel during
        // the final low-fuel crossing. The 0.75 m/s cap cannot recreate the old
        // centre-chasing pass or demand a large low-altitude tilt.
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
    LOCAL CURRENT_ACCEL_FILTER IS TERMINAL_ACCEL_FILTER.
    IF H_CORRIDOR_MODE AND HOOK_HEIGHT > TERMINAL_WAYPOINT_HEIGHT {
        SET CURRENT_ACCEL_FILTER TO TERMINAL_HIGH_ENERGY_ACCEL_FILTER.
    }
    SET FILTERED_H_ACCEL TO FILTERED_H_ACCEL * (1 - CURRENT_ACCEL_FILTER)
        + H_ACCEL * CURRENT_ACCEL_FILTER.
    SET H_ACCEL TO FILTERED_H_ACCEL.
    SET VERTICAL_THRUST_CMD TO CLAMP(VERTICAL_THRUST_CMD, 0,
        AVAILABLE_ACC * TERMINAL_NOMINAL_THRUST_FRACTION).

    // Keep full entry authority through the 2 km waypoint, then taper it
    // smoothly to the 12-degree landing cone by 500 m.  The former 800 m
    // blend still allowed 29 degrees at 239 m and produced a measured
    // 16 deg/s attitude spike during a small centre correction.
    LOCAL ALT_BLEND IS CLAMP((HOOK_HEIGHT - 500) / 1500, 0, 1).
    LOCAL TILT_LIMIT IS ENTRY_MAX_TILT * ALT_BLEND
        + LANDING_MAX_TILT * (1 - ALT_BLEND).
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
        AND HOOK_HEIGHT > 500 {
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
            AVAILABLE_ACC * TERMINAL_NOMINAL_THRUST_FRACTION).
    }
    LOCAL MAX_H_ACCEL IS MAX(VERTICAL_THRUST_CMD, G_ACC * 0.2)
        * TAN(TILT_LIMIT).
    SET H_ACCEL TO CLAMPV(H_ACCEL, MAX_H_ACCEL).
    LOCAL DESIRED_THRUST IS UP_VEC * MAX(VERTICAL_THRUST_CMD, G_ACC * 0.2)
        + H_ACCEL.
    // Aerodynamic/load constraint: the nozzle axis follows surface velocity.
    // Since DESIRED_THRUST points through the nose, constrain it to a 30-degree
    // cone around surface retrograde (the opposite of the nozzle direction).
    IF SHIP:VELOCITY:SURFACE:MAG >= TERMINAL_VELOCITY_CONE_MIN_SPEED {
        LOCAL RETROGRADE_THRUST_AXIS IS
            -SHIP:VELOCITY:SURFACE:NORMALIZED.
        LOCAL CONE_AXIAL IS VDOT(DESIRED_THRUST,
            RETROGRADE_THRUST_AXIS).
        IF CONE_AXIAL <= 0 {
            SET DESIRED_THRUST TO RETROGRADE_THRUST_AXIS
                * DESIRED_THRUST:MAG.
        } ELSE IF VANG(DESIRED_THRUST, RETROGRADE_THRUST_AXIS)
            > TERMINAL_VELOCITY_CONE_DEGREES {
            LOCAL CONE_LATERAL IS VXCL(RETROGRADE_THRUST_AXIS,
                DESIRED_THRUST).
            SET DESIRED_THRUST TO RETROGRADE_THRUST_AXIS * CONE_AXIAL
                + CLAMPV(CONE_LATERAL, CONE_AXIAL
                    * TAN(TERMINAL_VELOCITY_CONE_DEGREES)).
        }
    }
    SET DESIRED_THRUST TO CLAMPV(DESIRED_THRUST,
        AVAILABLE_ACC * TERMINAL_TOTAL_THRUST_FRACTION).
    LOCAL TILT_CMD IS VANG(DESIRED_THRUST, UP_VEC).
    LOCAL THROTTLE_CMD IS CLAMP(SHIP:MASS * DESIRED_THRUST:MAG /
        MAX(SHIP:AVAILABLETHRUST, 0.001), 0, 1).
    LOCK STEERING TO LOOKDIRUP(DESIRED_THRUST, SHIP:FACING:TOPVECTOR).
    LOCK THROTTLE TO THROTTLE_CMD.

    LOCAL GUIDANCE_PHASE IS "TRAJECTORY".
    IF H_CORRIDOR_MODE
        AND HOOK_HEIGHT <= TERMINAL_WAYPOINT_HEIGHT {
        SET GUIDANCE_PHASE TO "H_STOPPING".
    }
    IF CAPTURE_ALIGN_MODE { SET GUIDANCE_PHASE TO "H_ALIGN". }
    IF HORIZONTAL_SETTLE_MODE { SET GUIDANCE_PHASE TO "H_SETTLE". }
    IF PID_MODE { SET GUIDANCE_PHASE TO "PID_TERMINAL". }
    IF FINAL_ALIGN_MODE { SET GUIDANCE_PHASE TO "FINAL_ALIGN". }
    IF FINAL_DESCENT_ARMED { SET GUIDANCE_PHASE TO "VERTICAL_CAPTURE". }
    IF NOW - LAST_LOG >= TELEMETRY_PERIOD {
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

LOCK THROTTLE TO 0.
LOCK STEERING TO UP.
RCS OFF.
PRINT "CAPTURE COMPLETE" AT(0,18).
LOG MISSION_ID + ",CAPTURE," + ROUND(TIME:SECONDS,3) TO "0:/cz10b/telemetry.csv".
WAIT UNTIL FALSE.
