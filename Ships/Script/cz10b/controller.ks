// Reusable vector and guidance helpers for kOS 1.4+.

FUNCTION CLAMP {
    PARAMETER X, LO, HI.
    RETURN MAX(LO, MIN(HI, X)).
}

FUNCTION CLAMPV {
    PARAMETER VEC, LIMIT.
    IF VEC:MAG > LIMIT {
        RETURN VEC:NORMALIZED * LIMIT.
    }
    RETURN VEC.
}

FUNCTION COAST_TIME_TO_HEIGHT {
    PARAMETER CURRENT_HEIGHT, VERTICAL_V, TARGET_HEIGHT, GRAVITY.
    // Positive vertical velocity is upward. Solve
    // dh + v*t - 0.5*g*t^2 = 0 for the future descending crossing.
    LOCAL DH IS MAX(CURRENT_HEIGHT - TARGET_HEIGHT, 0).
    RETURN MAX((VERTICAL_V + SQRT(MAX(VERTICAL_V^2
        + 2 * GRAVITY * DH, 0))) / MAX(GRAVITY, 0.001), 1).
}

FUNCTION BOOSTER_PROPELLANT_FRACTION {
    // Read only the integrated first-stage tanks. SHIP:LIQUIDFUEL would also
    // include the upper stage and cannot enforce a recovery reserve.
    LOCAL BOOSTERS IS SHIP:PARTSNAMED("CZ10B-DemoBooster").
    IF BOOSTERS:LENGTH = 0 { RETURN 0. }
    LOCAL AMOUNT IS 0.
    LOCAL CAPACITY IS 0.
    FOR RESOURCE IN BOOSTERS[0]:RESOURCES {
        IF RESOURCE:NAME = "LiquidFuel" OR RESOURCE:NAME = "Oxidizer" {
            SET AMOUNT TO AMOUNT + RESOURCE:AMOUNT.
            SET CAPACITY TO CAPACITY + RESOURCE:CAPACITY.
        }
    }
    IF CAPACITY <= 0 { RETURN 0. }
    RETURN AMOUNT / CAPACITY.
}

FUNCTION BOOSTER_HOOK_OFFSET_ALONG_UP {
    PARAMETER UP_VEC.
    LOCAL BOOSTERS IS SHIP:PARTSNAMED("CZ10B-DemoBooster").
    IF BOOSTERS:LENGTH = 0 { RETURN HOOK_ABOVE_COM. }
    // The full mission now carries a physically separate 6 t stock engine, so
    // the vessel CoM moves as propellant is consumed. Measure the hook relative
    // to the live CoM instead of assuming the tank origin remains the CoM.
    LOCAL HOOK_FROM_COM IS BOOSTERS[0]:POSITION - SHIP:POSITION
        + SHIP:FACING:FOREVECTOR * BOOSTER_HOOK_LOCAL_Y.
    RETURN VDOT(HOOK_FROM_COM, UP_VEC) - NET_PLANE_OFFSET.
}

FUNCTION FIND_RECOVERY_SHIP {
    PARAMETER WANTED_NAME.
    LOCAL FOUND IS 0.
    LOCAL ALL_VESSELS IS LIST().
    LIST VESSELS IN ALL_VESSELS.
    FOR CANDIDATE IN ALL_VESSELS {
        IF CANDIDATE:NAME = WANTED_NAME {
            SET FOUND TO CANDIDATE.
        }
    }
    RETURN FOUND.
}

FUNCTION NET_POSITION {
    PARAMETER PLATFORM.
    // A landed target outside KSP's physics bubble has an orbit-level position
    // but no live Part collection. Use its vessel origin until it reloads.
    IF NOT PLATFORM:LOADED { RETURN PLATFORM:POSITION. }
    LOCAL NET_PARTS IS PLATFORM:PARTSNAMED("CZ10B-CatchNet").
    IF NET_PARTS:LENGTH = 0 {
        SET NET_PARTS TO PLATFORM:PARTSNAMED("CZ10B-RecoveryPlatform").
    }
    IF NET_PARTS:LENGTH > 0 {
        RETURN NET_PARTS[0]:POSITION.
    }
    // Fallback allows early tests against a ship that has no custom net part.
    RETURN PLATFORM:POSITION.
}

FUNCTION NET_STATE {
    PARAMETER PLATFORM.
    IF NOT PLATFORM:LOADED { RETURN "Unloaded". }
    LOCAL NET_PARTS IS PLATFORM:PARTSNAMED("CZ10B-CatchNet").
    IF NET_PARTS:LENGTH = 0 {
        SET NET_PARTS TO PLATFORM:PARTSNAMED("CZ10B-RecoveryPlatform").
    }
    IF NET_PARTS:LENGTH = 0 { RETURN "Missing". }
    LOCAL NET_MODULE IS NET_PARTS[0]:GETMODULE("ModuleCatchNet").
    IF NOT NET_MODULE:HASFIELD("Net state") { RETURN "Unknown". }
    RETURN NET_MODULE:GETFIELD("Net state").
}

FUNCTION NET_CLOSED {
    PARAMETER PLATFORM.
    LOCAL STATE IS NET_STATE(PLATFORM).
    RETURN STATE = "Closed".
}

FUNCTION RELEASE_LAUNCH_CLAMPS {
    LOCAL CLAMPS IS SHIP:PARTSNAMED("launchClamp1").
    FOR CLAMP_PART IN CLAMPS {
        LOCAL CLAMP_MODULE IS CLAMP_PART:GETMODULE("LaunchClamp").
        LOCAL CLAMP_EVENTS IS CLAMP_MODULE:ALLEVENTNAMES.
        IF CLAMP_EVENTS:LENGTH > 0 {
            CLAMP_MODULE:DOEVENT(CLAMP_EVENTS[0]).
        }
    }
    RETURN CLAMPS:LENGTH.
}

FUNCTION HOOK_CAPTURED {
    LOCAL HOOKS IS SHIP:PARTSNAMED("CZ10B-CatchHook").
    IF HOOKS:LENGTH = 0 {
        SET HOOKS TO SHIP:PARTSNAMED("CZ10B-DemoBooster").
    }
    IF HOOKS:LENGTH = 0 {
        SET HOOKS TO SHIP:PARTSNAMED("CZ10B-HoverTestBooster").
    }
    IF HOOKS:LENGTH = 0 { RETURN FALSE. }
    LOCAL HOOK_MODULE IS HOOKS[0]:GETMODULE("ModuleCatchHook").
    IF NOT HOOK_MODULE:HASFIELD("Hook state") { RETURN FALSE. }
    RETURN HOOK_MODULE:GETFIELD("Hook state") = "Captured".
}

FUNCTION WRITE_TELEMETRY {
    PARAMETER PHASE_NAME, MISSION_ID, HEIGHT, HOOK_HEIGHT, VERTICAL_V,
              HORIZONTAL_V, HORIZONTAL_ERROR, THROTTLE_CMD, TILT_CMD.
    LOCAL ROW IS MISSION_ID + "," + PHASE_NAME + "," + ROUND(TIME:SECONDS,3)
        + "," + ROUND(HEIGHT,2) + "," + ROUND(HOOK_HEIGHT,2)
        + "," + ROUND(VERTICAL_V,3) + "," + ROUND(HORIZONTAL_V,3)
        + "," + ROUND(HORIZONTAL_ERROR,3) + "," + ROUND(THROTTLE_CMD,4)
        + "," + ROUND(TILT_CMD,3) + "," + ROUND(SHIP:MASS,3)
        + "," + ROUND(SHIP:AVAILABLETHRUST,2).
    LOG ROW TO "0:/cz10b/telemetry.csv".
}
