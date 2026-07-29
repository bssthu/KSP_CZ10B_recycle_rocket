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

FUNCTION THRUST_SAFETY_AXIS {
    PARAMETER UP_VEC, MIN_SURFACE_SPEED.
    IF SHIP:VELOCITY:SURFACE:MAG >= MIN_SURFACE_SPEED {
        // The vehicle's facing/thrust axis is opposite the nozzle axis.  A
        // nozzle aligned with surface velocity therefore means thrust points
        // along surface retrograde.
        RETURN -SHIP:VELOCITY:SURFACE:NORMALIZED.
    }
    // At very low speed the velocity direction is ill-conditioned.  Requiring
    // thrust toward local up is equivalent to requiring the nozzle toward down.
    RETURN UP_VEC:NORMALIZED.
}

FUNCTION CONSTRAIN_THRUST_VECTOR {
    PARAMETER REQUESTED, AXIS, HALF_ANGLE_DEGREES.
    IF REQUESTED:MAG <= 0.0001 { RETURN AXIS:NORMALIZED * 0.0001. }
    LOCAL SAFE_AXIS IS AXIS:NORMALIZED.
    LOCAL REQUEST_MAG IS REQUESTED:MAG.
    LOCAL AXIAL IS VDOT(REQUESTED, SAFE_AXIS).
    IF AXIAL > 0
        AND VANG(REQUESTED, SAFE_AXIS) <= HALF_ANGLE_DEGREES {
        RETURN REQUESTED.
    }
    // Project to the closest cone boundary even when the request lies behind
    // the cone plane. Returning the centre axis for AXIAL <= 0 discarded the
    // direction of a valid "travel farther" correction and kept braking a
    // stage that was already short of the ship.
    LOCAL LATERAL IS VXCL(SAFE_AXIS, REQUESTED).
    IF LATERAL:MAG <= 0.0001 { RETURN SAFE_AXIS * REQUEST_MAG. }
    LOCAL LIMITED IS SAFE_AXIS * COS(HALF_ANGLE_DEGREES)
        + LATERAL:NORMALIZED * SIN(HALF_ANGLE_DEGREES).
    RETURN LIMITED * REQUEST_MAG.
}

FUNCTION LIMIT_VECTOR_SLEW {
    PARAMETER PREVIOUS, REQUESTED, MAX_ANGLE_DEGREES.
    LOCAL FROM_AXIS IS PREVIOUS:NORMALIZED.
    LOCAL TO_AXIS IS REQUESTED:NORMALIZED.
    LOCAL TARGET_ANGLE IS VANG(FROM_AXIS, TO_AXIS).
    LOCAL STEP_ANGLE IS MAX(MAX_ANGLE_DEGREES, 0).
    IF TARGET_ANGLE <= STEP_ANGLE { RETURN TO_AXIS. }
    // Normalized linear interpolation does not produce a constant angular
    // rate for a large reference change. Move on the great-circle arc so the
    // configured degrees/second limit remains physical at every target angle.
    LOCAL TURN_COMPONENT IS VXCL(FROM_AXIS, TO_AXIS).
    IF TURN_COMPONENT:MAG <= 0.0001 {
        // An exactly antiparallel request has no unique shortest turn plane.
        // Hold one sample rather than selecting an arbitrary discontinuity;
        // the final safety-cone projection still has absolute authority.
        RETURN FROM_AXIS.
    }
    RETURN FROM_AXIS * COS(STEP_ANGLE)
        + TURN_COMPONENT:NORMALIZED * SIN(STEP_ANGLE).
}

FUNCTION PRELEAD_THRUST_AXIS {
    PARAMETER UP_VEC, HORIZONTAL_BRAKE, LEAD_DEGREES.
    LOCAL RETROGRADE_AXIS IS THRUST_SAFETY_AXIS(UP_VEC,
        TERMINAL_VELOCITY_CONE_MIN_SPEED).
    IF HORIZONTAL_BRAKE:MAG <= 0.0001 OR LEAD_DEGREES <= 0 {
        RETURN RETROGRADE_AXIS.
    }
    // Rotate from surface retrograde toward horizontal braking.  Because this
    // is rebuilt from live velocity, the requested nozzle/velocity separation
    // is exactly LEAD_DEGREES on every update.
    LOCAL TURN_AXIS IS VXCL(RETROGRADE_AXIS,
        HORIZONTAL_BRAKE:NORMALIZED).
    IF TURN_AXIS:MAG <= 0.0001 { RETURN RETROGRADE_AXIS. }
    RETURN RETROGRADE_AXIS * COS(LEAD_DEGREES)
        + TURN_AXIS:NORMALIZED * SIN(LEAD_DEGREES).
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

FUNCTION DEPLETING_BURN_TIME {
    PARAMETER DELTA_V_VECTOR, UP_AXIS, GRAVITY_ACCEL, CURRENT_MASS,
        THRUST_KN, EFFECTIVE_ISP, MINIMUM_MASS.
    // Solve |delta-v + g*t| = ve*ln(m0/m1).  Unlike the former constant-mass
    // quadratic, this includes the nearly twofold acceleration increase while
    // the landing burn consumes most of the return propellant.
    IF THRUST_KN <= 0.001 OR EFFECTIVE_ISP <= 0.001
        OR CURRENT_MASS <= MINIMUM_MASS { RETURN 999. }
    LOCAL EXHAUST_VELOCITY IS EFFECTIVE_ISP * 9.80665.
    // With kN and tonnes, T/(Isp*g0) is directly tonnes per second.
    LOCAL MASS_FLOW IS THRUST_KN / EXHAUST_VELOCITY.
    LOCAL MAXIMUM_TIME IS MIN(180, (CURRENT_MASS - MINIMUM_MASS)
        / MAX(MASS_FLOW, 0.000001) * 0.995).
    IF MAXIMUM_TIME <= 0 { RETURN 999. }

    LOCAL HIGH_MASS IS CURRENT_MASS - MASS_FLOW * MAXIMUM_TIME.
    LOCAL HIGH_AVAILABLE_DV IS EXHAUST_VELOCITY
        * LN(CURRENT_MASS / MAX(HIGH_MASS, MINIMUM_MASS)).
    LOCAL HIGH_REQUIRED_DV IS (DELTA_V_VECTOR
        + UP_AXIS * GRAVITY_ACCEL * MAXIMUM_TIME):MAG.
    IF HIGH_AVAILABLE_DV < HIGH_REQUIRED_DV { RETURN 999. }

    LOCAL LOW_TIME IS 0.
    LOCAL HIGH_TIME IS MAXIMUM_TIME.
    LOCAL ITERATION IS 0.
    UNTIL ITERATION >= 24 {
        LOCAL MID_TIME IS (LOW_TIME + HIGH_TIME) * 0.5.
        LOCAL MID_MASS IS CURRENT_MASS - MASS_FLOW * MID_TIME.
        LOCAL MID_AVAILABLE_DV IS EXHAUST_VELOCITY
            * LN(CURRENT_MASS / MAX(MID_MASS, MINIMUM_MASS)).
        LOCAL MID_REQUIRED_DV IS (DELTA_V_VECTOR
            + UP_AXIS * GRAVITY_ACCEL * MID_TIME):MAG.
        IF MID_AVAILABLE_DV >= MID_REQUIRED_DV {
            SET HIGH_TIME TO MID_TIME.
        } ELSE {
            SET LOW_TIME TO MID_TIME.
        }
        SET ITERATION TO ITERATION + 1.
    }
    RETURN HIGH_TIME.
}

FUNCTION CONSTANT_ACCEL_BURN_TIME {
    PARAMETER DELTA_V_VECTOR, UP_AXIS, GRAVITY_ACCEL, THRUST_ACCEL.
    // Solve |delta_v + g_up*t| = a*t.  This is the matching ignition model for
    // the main-burn actuator, whose thrust limiter is scheduled with mass so
    // that full-command acceleration remains approximately constant.
    LOCAL DV_UP IS VDOT(DELTA_V_VECTOR, UP_AXIS:NORMALIZED).
    LOCAL A IS GRAVITY_ACCEL^2 - THRUST_ACCEL^2.
    LOCAL B IS 2 * GRAVITY_ACCEL * DV_UP.
    LOCAL C IS DELTA_V_VECTOR:MAG^2.
    IF A >= -0.001 { RETURN 999. }
    LOCAL DISC IS MAX(B^2 - 4 * A * C, 0).
    RETURN MAX((-B - SQRT(DISC)) / (2 * A), 0).
}

FUNCTION NORMALIZE_BOOSTER_ENGINE_ACCEL {
    PARAMETER ENGINE_NAME, TARGET_ACCEL, MIN_LIMIT, MAX_LIMIT.
    // Scale every matching active engine by the ratio between requested and
    // currently available vessel acceleration.  The update is continuous and
    // monotonic with mass; it does not switch the engine or introduce PWM.
    LOCAL CURRENT_AVAILABLE IS SHIP:AVAILABLETHRUST.
    IF CURRENT_AVAILABLE <= 0 OR SHIP:MASS <= 0 { RETURN FALSE. }
    LOCAL CURRENT_ACCEL IS CURRENT_AVAILABLE / SHIP:MASS.
    IF CURRENT_ACCEL <= 0 { RETURN FALSE. }
    LOCAL LIMIT_SCALE IS TARGET_ACCEL / CURRENT_ACCEL.
    LOCAL CHANGED IS FALSE.
    FOR ENGINE IN SHIP:ENGINES {
        IF ENGINE:NAME = ENGINE_NAME {
            LOCAL NEW_LIMIT IS CLAMP(ENGINE:THRUSTLIMIT
                * LIMIT_SCALE, MIN_LIMIT, MAX_LIMIT).
            // Avoid rebuilding KSP's engine/part state for a numerically
            // identical limit.  Per-frame writes in run 15 reduced physics to
            // roughly one tenth real time despite producing the right thrust.
            IF ABS(NEW_LIMIT - ENGINE:THRUSTLIMIT) >= 0.01 {
                SET ENGINE:THRUSTLIMIT TO NEW_LIMIT.
                SET CHANGED TO TRUE.
            }
        }
    }
    RETURN CHANGED.
}

FUNCTION BOOSTER_HOOK_OFFSET_FROM_COM {
    LOCAL BOOSTERS IS SHIP:PARTSNAMED("CZ10B-DemoBooster").
    IF BOOSTERS:LENGTH = 0 {
        RETURN SHIP:FACING:FOREVECTOR * HOOK_ABOVE_COM.
    }
    // The full mission now carries a physically separate 6 t stock engine, so
    // the vessel CoM moves as propellant is consumed. Measure the hook relative
    // to the live CoM instead of assuming the tank origin remains the CoM.
    // This vector is also the exact centre of the four virtual hook points:
    // their symmetric 1.65 m radius averages to the configured local-Y point.
    // ModuleCatchHook uses TransformPoint(0, hookOffsetY, 0). Part:FACING is
    // the local-to-world rotation in kOS, so rotate that same local-Y vector;
    // SHIP:FACING:FOREVECTOR would incorrectly use vessel-local Z.
    RETURN BOOSTERS[0]:POSITION - SHIP:POSITION
        + BOOSTERS[0]:FACING * V(0, BOOSTER_HOOK_LOCAL_Y, 0).
}

FUNCTION BOOSTER_HOOK_OFFSET_ALONG_UP {
    PARAMETER UP_VEC.
    RETURN VDOT(BOOSTER_HOOK_OFFSET_FROM_COM(), UP_VEC)
        - NET_PLANE_OFFSET.
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

FUNCTION SET_BOOSTER_REACTION_WHEEL_AUTHORITY {
    PARAMETER AUTHORITY_PERCENT.
    LOCAL BOOSTER_PARTS IS SHIP:PARTSNAMED("CZ10B-DemoBooster").
    IF BOOSTER_PARTS:LENGTH = 0 { RETURN FALSE. }
    LOCAL WHEEL_MODULE IS BOOSTER_PARTS[0]:GETMODULE("ModuleReactionWheel").
    LOCAL AUTHORITY_FIELD IS "".
    IF WHEEL_MODULE:HASFIELD("Authority Limiter") {
        SET AUTHORITY_FIELD TO "Authority Limiter".
    } ELSE IF WHEEL_MODULE:HASFIELD("动量轮扭力限制") {
        SET AUTHORITY_FIELD TO "动量轮扭力限制".
    } ELSE IF WHEEL_MODULE:HASFIELD("authorityLimiter") {
        SET AUTHORITY_FIELD TO "authorityLimiter".
    }
    IF AUTHORITY_FIELD = "" { RETURN FALSE. }
    WHEEL_MODULE:SETFIELD(AUTHORITY_FIELD,
        CLAMP(AUTHORITY_PERCENT, 0, 100)).
    RETURN TRUE.
}

FUNCTION SET_BOOSTER_GRID_FIN_LIFT_AUTHORITY {
    PARAMETER AUTHORITY_PERCENT.
    LOCAL CHANGED_COUNT IS 0.
    LOCAL GRID_FINS IS SHIP:PARTSNAMED("CZ10B-GridFin").
    FOR GRID_FIN IN GRID_FINS {
        LOCAL FIN_MODULE IS
            GRID_FIN:GETMODULE("ModuleGridFinAuthority").
        LOCAL AUTHORITY_FIELD IS "".
        IF FIN_MODULE:HASFIELD("Lift authority") {
            SET AUTHORITY_FIELD TO "Lift authority".
        } ELSE IF FIN_MODULE:HASFIELD("liftAuthorityPercent") {
            SET AUTHORITY_FIELD TO "liftAuthorityPercent".
        }
        IF AUTHORITY_FIELD <> "" {
            FIN_MODULE:SETFIELD(AUTHORITY_FIELD,
                CLAMP(AUTHORITY_PERCENT, 0, 100)).
            SET CHANGED_COUNT TO CHANGED_COUNT + 1.
        }
    }
    RETURN CHANGED_COUNT.
}

FUNCTION SET_BOOSTER_GRID_FIN_DEPLOYMENT {
    PARAMETER DEPLOYMENT_PERCENT.
    LOCAL CHANGED_COUNT IS 0.
    LOCAL GRID_FINS IS SHIP:PARTSNAMED("CZ10B-GridFin").
    FOR GRID_FIN IN GRID_FINS {
        LOCAL FIN_MODULE IS
            GRID_FIN:GETMODULE("ModuleGridFinAuthority").
        LOCAL DEPLOYMENT_FIELD IS "".
        IF FIN_MODULE:HASFIELD("Deployment command") {
            SET DEPLOYMENT_FIELD TO "Deployment command".
        } ELSE IF FIN_MODULE:HASFIELD("deploymentCommandPercent") {
            SET DEPLOYMENT_FIELD TO "deploymentCommandPercent".
        }
        IF DEPLOYMENT_FIELD <> "" {
            FIN_MODULE:SETFIELD(DEPLOYMENT_FIELD,
                CLAMP(DEPLOYMENT_PERCENT, 0, 100)).
            SET CHANGED_COUNT TO CHANGED_COUNT + 1.
        }
    }
    RETURN CHANGED_COUNT.
}

FUNCTION BOOSTER_GRID_FIN_APPLIED_DEPLOYMENT {
    LOCAL APPLIED_TOTAL IS 0.
    LOCAL APPLIED_COUNT IS 0.
    LOCAL GRID_FINS IS SHIP:PARTSNAMED("CZ10B-GridFin").
    FOR GRID_FIN IN GRID_FINS {
        LOCAL FIN_MODULE IS
            GRID_FIN:GETMODULE("ModuleGridFinAuthority").
        LOCAL APPLIED_FIELD IS "".
        IF FIN_MODULE:HASFIELD("Applied deployment") {
            SET APPLIED_FIELD TO "Applied deployment".
        } ELSE IF FIN_MODULE:HASFIELD("appliedDeploymentDegrees") {
            SET APPLIED_FIELD TO "appliedDeploymentDegrees".
        }
        IF APPLIED_FIELD <> "" {
            SET APPLIED_TOTAL TO APPLIED_TOTAL
                + FIN_MODULE:GETFIELD(APPLIED_FIELD).
            SET APPLIED_COUNT TO APPLIED_COUNT + 1.
        }
    }
    IF APPLIED_COUNT = 0 { RETURN -1. }
    RETURN APPLIED_TOTAL / APPLIED_COUNT.
}

FUNCTION WRITE_TELEMETRY {
    PARAMETER PHASE_NAME, MISSION_ID, HEIGHT, HOOK_HEIGHT, VERTICAL_V,
              HORIZONTAL_V, HORIZONTAL_ERROR, THROTTLE_CMD, TILT_CMD.
    LOCAL ROW IS MISSION_ID + "," + PHASE_NAME + "," + ROUND(TIME:SECONDS,3)
        + "," + ROUND(HEIGHT,2) + "," + ROUND(HOOK_HEIGHT,2)
        + "," + ROUND(VERTICAL_V,3) + "," + ROUND(HORIZONTAL_V,3)
        + "," + ROUND(HORIZONTAL_ERROR,3) + "," + ROUND(THROTTLE_CMD,4)
        + "," + ROUND(TILT_CMD,3) + "," + ROUND(SHIP:MASS,3)
        + "," + ROUND(SHIP:AVAILABLETHRUST,2)
        + "," + ROUND(VANG(SHIP:FACING:VECTOR,
            SHIP:UP:VECTOR:NORMALIZED),3).
    LOG ROW TO "0:/cz10b/telemetry.csv".
}
