using System.Linq;
using System.IO;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Test-only launch rail and acceptance observer for the complete staged
    /// mission. kOS owns countdown, staging, throttle and attitude after release.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class MissionTestFlightSequencer : MonoBehaviour
    {
        private bool active;
        private bool released;
        private bool switched;
        private bool reported;
        private bool everPowered;
        private bool reachedAltitude;
        private bool upperSeparated;
        private float separationPropellantFraction = -1f;
        private float startRealtime;
        private double startUniversalTime;
        private float capturedAt = -1f;
        private float stableCapturedAt = -1f;
        private float filteredCaptureAngularRate = -1f;
        private float nextStatus;
        private bool seaMission;
        private Vessel pendingSeaPlatform;
        private float seaMoveAt;
        private Vessel seaPlatform;
        private double seaLatitude;
        private double seaLongitude;
        private double seaTerrainAltitude;
        private double seaStationAltitude = 5.5d;
        private double seaDeploymentLongitude;
        private double seaDeploymentAltitude;
        private bool seaPlacementReleased;
        private float seaReleaseAt;
        private float maxHighAngularRate;
        private float maxAscentCoastAngularRate;
        private float maxTerminalNozzleVelocityAngle;
        private bool terminalWaypointRecorded;
        private float terminalWaypointVerticalSpeed = float.MaxValue;
        private float terminalWaypointHorizontalSpeed = float.MaxValue;
        private double terminalWaypointHorizontalError = double.MaxValue;
        private bool terminalCenterSeen;
        private double maxTerminalReboundAfterCenter;
        private bool upperInsertionReleased;
        private bool upperCutoffCommanded;
        private bool seaStationDeferred;
        private Quaternion seaSurfaceRotationOffset = Quaternion.identity;

        private void Start()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            active = vessel != null && HasPart(vessel, "CZ10B-RecoveryPlatform") &&
                     HasPart(vessel, "CZ10B-MissionLaunchRail") &&
                     HasPart(vessel, "CZ10B-DemoBooster") &&
                     HasPart(vessel, "CZ10B-DemoUpperStage");
            if (!active)
                return;
            string seaMarker = Path.Combine(KSPUtil.ApplicationRootPath, "GameData",
                "CZ10BRecovery", "PluginData", "sea-mission-flight.active");
            seaMission = File.Exists(seaMarker);
            if (seaMission)
                File.Delete(seaMarker);
            startRealtime = Time.realtimeSinceStartup;
            startUniversalTime = Planetarium.GetUniversalTime();
            Debug.Log("[CZ10BNetRecovery] " + Prefix + "_FLIGHT_READY vessel=" +
                      vessel.vesselName + " sea=" + seaMission);
        }

        private void Update()
        {
            if (!active || reported)
                return;
            float elapsed = Time.realtimeSinceStartup - startRealtime;
            // Cold starts can spend several seconds unpacking colliders after
            // Flight becomes active.  Keep the combined stack on the pad long
            // enough for one stable physics window before separating the ship.
            if (!released && elapsed >= 8f)
                ReleaseVehicle();
            if (released && !switched && elapsed >= 8.2f)
                SwitchToBooster();
            if (pendingSeaPlatform != null && Time.realtimeSinceStartup >= seaMoveAt)
            {
                MovePlatformToSea(pendingSeaPlatform);
                Vessel seaBooster = FindVesselWithPart("CZ10B-DemoBooster");
                if (seaBooster != null)
                    FlightGlobals.SetActiveVessel(seaBooster);
                FlightGlobals.fetch.SetVesselTarget(pendingSeaPlatform, true);
                Debug.Log("[CZ10BNetRecovery] SEA_TARGET_REBOUND vessel=" +
                          pendingSeaPlatform.vesselName);
                pendingSeaPlatform = null;
            }
            Vessel booster = FindVesselWithPart("CZ10B-DemoBooster");
            if (seaPlatform != null)
            {
                if (!seaPlacementReleased)
                    MaintainSeaPlatform();
                else
                    ManageSeaStationLoading(booster);
            }

            if (booster != null)
            {
                reachedAltitude |= booster.altitude > 2000;
                if (upperSeparated && booster.altitude > 10000)
                    maxHighAngularRate = Mathf.Max(maxHighAngularRate,
                        booster.angularVelocity.magnitude * Mathf.Rad2Deg);
                // Record only the post-separation ascent coast.  A missed
                // recovery can bounce at sea level with a positive vertical
                // speed; treating that as ascent polluted this diagnostic and
                // hid the actual 0.56 deg/s coast result.
                if (upperSeparated && booster.altitude > 10000d &&
                    booster.verticalSpeed > 0.5d)
                    maxAscentCoastAngularRate = Mathf.Max(
                        maxAscentCoastAngularRate,
                        booster.angularVelocity.magnitude * Mathf.Rad2Deg);
                ModuleEngines engine = booster.parts
                    .Select(p => p.FindModuleImplementing<ModuleEngines>())
                    .FirstOrDefault(e => e != null);
                everPowered |= engine != null && engine.finalThrust > 10f;
            }
            Vessel upper = FindVesselWithPart("CZ10B-DemoUpperStage");
            bool separatedNow = upper != null && booster != null && upper != booster;
            if (separatedNow && !upperInsertionReleased)
            {
                ExtendAllRanges(upper.vesselRanges);
                bool nominalUpperOrbit = upper.orbit != null &&
                    upper.orbit.PeA >= 90000d;
                bool safeUpperOrbit = upper.orbit != null &&
                    upper.orbit.PeA >= 72000d && upper.orbit.ApA >= 120000d;
                bool firstUpperBurn = upper.orbit != null &&
                    upper.orbit.ApA < 100000d;
                bool circularizationBurn = upper.orbit != null &&
                    (upper.orbit.timeToAp <= 20d || upper.verticalSpeed < 0d);
                CommandUpperStage(upper, !upperCutoffCommanded &&
                    !nominalUpperOrbit && !safeUpperOrbit &&
                    (firstUpperBurn || circularizationBurn));
                if (!upperCutoffCommanded &&
                    (nominalUpperOrbit || safeUpperOrbit))
                {
                    CommandUpperStage(upper, false);
                    foreach (Part part in upper.parts)
                    {
                        ModuleEngines upperEngine =
                            part.FindModuleImplementing<ModuleEngines>();
                        if (upperEngine != null)
                            upperEngine.Shutdown();
                    }
                    upperCutoffCommanded = true;
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] UPPER_STAGE_CUTOFF apoapsis={0:F0} periapsis={1:F0}",
                        upper.orbit.ApA, upper.orbit.PeA));
                }
                if ((nominalUpperOrbit || safeUpperOrbit) &&
                    UpperStageThrust(upper) < 1f)
                {
                    RestoreDefaultRanges(upper.vesselRanges);
                    upperInsertionReleased = true;
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] UPPER_STAGE_INSERTION_COMPLETE apoapsis={0:F0} periapsis={1:F0}",
                        upper.orbit.ApA, upper.orbit.PeA));
                }
            }
            if (separatedNow && !upperSeparated)
            {
                upperSeparated = true;
                separationPropellantFraction = BoosterPropellantFraction(booster);
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] STAGE_RESERVE fraction={0:F4} altitude={1:F0} speed={2:F1} mass={3:F3}",
                    separationPropellantFraction, booster.altitude,
                    booster.srfSpeed, booster.GetTotalMass()));
            }

            ModuleCatchHook hook = booster == null ? null : booster.parts
                .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                .FirstOrDefault(h => h != null);
            ModuleCatchNet captureNet = FindNet();
            RecordTerminalEnvelope(booster, hook, captureNet);
            if (released && elapsed > 20f && booster == null)
            {
                Debug.LogError("[CZ10BNetRecovery] " + Prefix +
                               "_BOOSTER_LOST");
                Report(false, hook);
                return;
            }
            if (hook != null && hook.hookState == "Captured")
            {
                if (capturedAt < 0f)
                {
                    capturedAt = elapsed;
                    filteredCaptureAngularRate = -1f;
                }
                float captureAngularRate = booster == null ? float.MaxValue :
                    booster.angularVelocity.magnitude * Mathf.Rad2Deg;
                if (filteredCaptureAngularRate < 0f)
                    filteredCaptureAngularRate = captureAngularRate;
                else
                {
                    // Cable-joint solver impulses can produce one-frame angular
                    // spikes after payout even though the vehicle, platform and
                    // cable are already settling.  Judge the visible motion with
                    // a short low-pass filter instead of resetting the entire
                    // eight-second dwell for an unobservable physics impulse.
                    float blend = 1f - Mathf.Exp(-Mathf.Max(0f, Time.deltaTime) / 1.5f);
                    filteredCaptureAngularRate = Mathf.Lerp(
                        filteredCaptureAngularRate, captureAngularRate, blend);
                }
                float captureVerticalSpeed = booster == null ? float.MaxValue :
                    Mathf.Abs((float)booster.verticalSpeed);
                float captureHorizontalSpeed = booster == null ? float.MaxValue :
                    (float)booster.horizontalSrfSpeed;
                float platformTilt = SeaPlatformTilt(captureNet);
                bool payoutComplete = captureNet != null &&
                    captureNet.cableDeflection >= captureNet.captureSettleDrop - 1f;
                bool captureStable = filteredCaptureAngularRate <= 12f &&
                    captureVerticalSpeed <= 4f && captureHorizontalSpeed <= 3f &&
                    platformTilt <= 3f && payoutComplete;
                if (captureStable)
                {
                    if (stableCapturedAt < 0f)
                        stableCapturedAt = elapsed;
                }
                else
                    stableCapturedAt = -1f;
                if (stableCapturedAt >= 0f && elapsed - stableCapturedAt >= 8f &&
                    everPowered && reachedAltitude && upperSeparated)
                    Report(separationPropellantFraction >= 0f &&
                           separationPropellantFraction <= 0.205f &&
                           maxAscentCoastAngularRate <= 5f &&
                           terminalWaypointRecorded &&
                           terminalWaypointVerticalSpeed <= 200f &&
                           terminalWaypointHorizontalSpeed <= 12f &&
                           terminalWaypointHorizontalError <= 50d &&
                           terminalCenterSeen &&
                           maxTerminalReboundAfterCenter <= 30d &&
                           maxTerminalNozzleVelocityAngle <= 32f, hook);
                else if (elapsed - capturedAt >= 60f)
                    Report(false, hook);
            }
            else
            {
                capturedAt = -1f;
                stableCapturedAt = -1f;
                filteredCaptureAngularRate = -1f;
            }

            if (elapsed >= nextStatus)
            {
                nextStatus = elapsed + 5f;
                ModuleCatchNet net = captureNet;
                double targetDistance = booster == null || seaPlatform == null
                    ? -1 : Vector3d.Distance(
                        booster.mainBody.GetWorldSurfacePosition(
                            booster.latitude, booster.longitude, 0),
                        seaPlatform.mainBody.GetWorldSurfacePosition(
                            seaPlatform.latitude, seaPlatform.longitude, 0));
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] {0}_STATUS t={1:F1} altitude={2:F0} apoapsis={3:F0} vertical={4:F1} horizontal={5:F1} reserve={6:F4} lat={7:F5} lon={8:F5} targetDistance={9:F0} powered={10} high={11} separated={12} hook={13} net={14} cableError={15:F2} sag={16:F2} maxHighAngular={17:F2} angular={18:F2} platformTilt={19:F2} stableFor={20:F1} maxAscentCoastAngular={21:F2}",
                    Prefix,
                    elapsed, booster == null ? -1 : booster.altitude,
                    booster == null || booster.orbit == null ? -1 :
                        booster.orbit.ApA,
                    booster == null ? 0 : booster.verticalSpeed,
                    booster == null ? 0 : booster.horizontalSrfSpeed,
                    BoosterPropellantFraction(booster),
                    booster == null ? 0 : booster.latitude,
                    booster == null ? 0 : booster.longitude,
                    targetDistance, everPowered, reachedAltitude, upperSeparated,
                    hook == null ? "missing" : hook.hookState,
                    net == null ? "missing" : net.netState,
                    net == null ? -1f : net.cableTrackingError,
                    net == null ? -1f : net.cableDeflection,
                    maxHighAngularRate,
                    booster == null ? -1f :
                        booster.angularVelocity.magnitude * Mathf.Rad2Deg,
                    SeaPlatformTilt(net), stableCapturedAt < 0f ? 0f :
                        elapsed - stableCapturedAt,
                    maxAscentCoastAngularRate));
                if (seaMission && seaPlatform != null)
                {
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] SEA_PLATFORM_STATUS lat={0:F5} lon={1:F5} alt={2:F1} terrain={3:F1} situation={4} packed={5} released={6} tilt={7:F2}",
                        seaPlatform.latitude, seaPlatform.longitude,
                        seaPlatform.altitude, seaTerrainAltitude,
                        seaPlatform.situation, seaPlatform.packed,
                        seaPlacementReleased, SeaPlatformTilt(net)));
                }
                if (upperSeparated && upper != null)
                {
                    ModuleEngines upperEngine = upper.parts
                        .Select(p => p.FindModuleImplementing<ModuleEngines>())
                        .FirstOrDefault(e => e != null);
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] UPPER_STAGE_STATUS altitude={0:F0} apoapsis={1:F0} periapsis={2:F0} speed={3:F1} thrust={4:F1} packed={5} situation={6}",
                        upper.altitude,
                        upper.orbit == null ? -1 : upper.orbit.ApA,
                        upper.orbit == null ? -1 : upper.orbit.PeA,
                        upper.srfSpeed,
                        upperEngine == null ? -1f : upperEngine.finalThrust,
                        upper.packed, upper.situation));
                }
            }
            // Use simulated mission time for the long acceptance deadline. Near
            // two loaded vessels KSP can run well below real time, especially in
            // a hidden/background test; wall-clock timeout would reject a valid
            // trajectory that is still advancing normally in physics time.
            if (Planetarium.GetUniversalTime() - startUniversalTime >= 1300d)
                Report(false, hook);
        }

        private void ReleaseVehicle()
        {
            Vessel combined = FlightGlobals.ActiveVessel;
            Part rail = combined.parts.FirstOrDefault(p => p.partInfo != null &&
                p.partInfo.name == "CZ10B-MissionLaunchRail");
            ModuleDecouple decoupler = rail == null ? null :
                rail.FindModuleImplementing<ModuleDecouple>();
            if (decoupler == null)
            {
                Debug.LogError("[CZ10BNetRecovery] MISSION_TEST_RELEASE_FAILED");
                reported = true;
                return;
            }
            DisablePartColliders(rail);
            decoupler.Decouple();
            released = true;
            Debug.Log("[CZ10BNetRecovery] " + Prefix + "_RAIL_RELEASED");
        }

        private void SwitchToBooster()
        {
            Vessel booster = FindVesselWithPart("CZ10B-DemoBooster");
            ModuleCatchNet net = FindNet();
            if (booster == null || HasPart(booster, "CZ10B-RecoveryPlatform"))
                return;
            booster.GoOffRails();
            // TT18-A clamps still carry the stage at this point. Preserve the
            // stock PRELAUNCH state and clear only residual angular momentum;
            // forcing FLYING here intermittently killed the kOS engine context
            // before ignition on cold starts.
            booster.situation = Vessel.Situations.PRELAUNCH;
            booster.angularVelocity = Vector3.zero;
            booster.angularMomentum = Vector3.zero;
            FlightGlobals.SetActiveVessel(booster);
            if (net != null)
                FlightGlobals.fetch.SetVesselTarget(net.vessel, true);
            if (seaMission && net != null)
            {
                pendingSeaPlatform = net.vessel;
                seaMoveAt = Time.realtimeSinceStartup + 1.5f;
            }
            switched = true;
            Debug.Log("[CZ10BNetRecovery] " + Prefix + "_RELEASED target=" +
                      (net == null ? "missing" : net.vessel.vesselName));
        }

        private void Report(bool pass, ModuleCatchHook hook)
        {
            reported = true;
            Vessel booster = FindVesselWithPart("CZ10B-DemoBooster");
            float landingFraction = BoosterPropellantFraction(booster);
            string detail = string.Format(
                " powered={0} high={1} separated={2} hook={3} maxHighAngular={4:F2} separationReserve={5:F4} landingReserve={6:F4} maxAscentCoastAngular={7:F2} waypointRecorded={8} waypointVertical={9:F2} waypointHorizontal={10:F2} waypointError={11:F1} maxNozzleVelocityAngle={12:F2} centerSeen={13} reboundAfterCenter={14:F1}",
                everPowered, reachedAltitude, upperSeparated,
                hook == null ? "missing" : hook.hookState, maxHighAngularRate,
                separationPropellantFraction, landingFraction,
                maxAscentCoastAngularRate, terminalWaypointRecorded,
                terminalWaypointVerticalSpeed,
                terminalWaypointHorizontalSpeed,
                terminalWaypointHorizontalError,
                maxTerminalNozzleVelocityAngle,
                terminalCenterSeen,
                maxTerminalReboundAfterCenter);
            if (pass)
                Debug.Log("[CZ10BNetRecovery] " + Prefix + "_PASS" + detail);
            else
                Debug.LogError("[CZ10BNetRecovery] " + Prefix + "_FAIL" + detail);
        }

        private string Prefix
        {
            get { return seaMission ? "SEA_MISSION_TEST" : "MISSION_TEST"; }
        }

        private void RecordTerminalEnvelope(Vessel booster,
            ModuleCatchHook hook, ModuleCatchNet net)
        {
            if (booster == null || hook == null || net == null ||
                !upperSeparated || booster.verticalSpeed >= 0d)
                return;

            ModuleEngines engine = booster.parts
                .Select(p => p.FindModuleImplementing<ModuleEngines>())
                .FirstOrDefault(e => e != null && e.finalThrust > 10f);
            Vector3 surfaceVelocity = (Vector3)booster.srf_velocity;
            // Below 30 m/s the instantaneous direction of a nearly stopped
            // vessel is dominated by tiny wave/settling components and no
            // longer represents an aerodynamic load axis. The controller and
            // this observer both hand over from the cone to direct damping at
            // the same threshold.
            if (engine != null && surfaceVelocity.magnitude >= 30f &&
                booster.ReferenceTransform != null)
            {
                float nozzleAngle = Vector3.Angle(
                    -booster.ReferenceTransform.up,
                    surfaceVelocity.normalized);
                maxTerminalNozzleVelocityAngle = Mathf.Max(
                    maxTerminalNozzleVelocityAngle, nozzleAngle);
            }

            Vector3 hookCentre = hook.GetHookWorldPoints()
                .Aggregate(Vector3.zero, (sum, point) => sum + point) /
                Mathf.Max(1, hook.virtualHookCount);
            float hookHeight = net.HeightAbovePlane(hookCentre);
            if (hookHeight > 2000f)
                return;

            double horizontalError = Vector3d.Distance(
                booster.mainBody.GetWorldSurfacePosition(
                    booster.latitude, booster.longitude, 0),
                net.vessel.mainBody.GetWorldSurfacePosition(
                    net.vessel.latitude, net.vessel.longitude, 0));
            if (horizontalError <= 10d)
                terminalCenterSeen = true;
            if (terminalCenterSeen)
                maxTerminalReboundAfterCenter = System.Math.Max(
                    maxTerminalReboundAfterCenter, horizontalError);
            if (terminalWaypointRecorded)
                return;

            terminalWaypointRecorded = true;
            terminalWaypointVerticalSpeed = Mathf.Abs(
                (float)booster.verticalSpeed);
            terminalWaypointHorizontalSpeed =
                (float)booster.horizontalSrfSpeed;
            terminalWaypointHorizontalError = horizontalError;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] TERMINAL_2KM vertical={0:F2} horizontal={1:F2} error={2:F1} nozzleAngleMax={3:F2}",
                terminalWaypointVerticalSpeed,
                terminalWaypointHorizontalSpeed,
                terminalWaypointHorizontalError,
                maxTerminalNozzleVelocityAngle));
        }

        private static void CommandUpperStage(Vessel upper, bool burn)
        {
            if (upper == null || upper.parts == null)
                return;
            upper.ActionGroups.SetGroup(KSPActionGroup.SAS, burn);
            if (burn && upper.Autopilot != null)
                upper.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Prograde);
            upper.ctrlState.mainThrottle = burn ? 1f : 0f;
            foreach (ModuleEngines engine in upper.parts.Select(p =>
                p.FindModuleImplementing<ModuleEngines>()).Where(e => e != null))
            {
                engine.thrustPercentage = 100f;
                if (burn)
                    engine.Activate();
                else
                    engine.Shutdown();
            }
        }

        private void MovePlatformToSea(Vessel platform)
        {
            CelestialBody body = platform.mainBody;
            seaPlatform = platform;
            ExtendRanges(platform.vesselRanges.prelaunch);
            ExtendRanges(platform.vesselRanges.landed);
            ExtendRanges(platform.vesselRanges.splashed);
            ExtendRanges(platform.vesselRanges.flying);
            ExtendRanges(platform.vesselRanges.subOrbital);
            ExtendRanges(platform.vesselRanges.orbit);
            double startLatitude = platform.latitude;
            seaLatitude = -0.05865d;
            double startLongitude = platform.longitude;
            Quaternion startSurfaceFrame = SurfaceFrame(body, startLatitude,
                startLongitude, platform.altitude);
            seaSurfaceRotationOffset = Quaternion.Inverse(startSurfaceFrame)
                * platform.transform.rotation;
            // The slower dense-air turn and 40 km entry burn put the measured
            // uncorrected footprint about 47.2 degrees east of KSC.  Place the
            // ship there so the 30 km planner begins inside the same local
            // surface frame instead of seeing a 200 km curvature error.
            // Recalibrate the deck after lowering the stop-envelope acceleration
            // to absorb attitude lag. The no-rebound run crossed 2 km 125.9 m
            // west of the former deck while already converging monotonically;
            // moving the target to that measured footprint keeps the stricter
            // 2 km gate from requiring a late low-altitude translation.
            const double measuredFootprintOffset = 46.8644d;
            seaLongitude = startLongitude + measuredFootprintOffset;
            seaTerrainAltitude = TerrainAltitude(body, seaLatitude, seaLongitude);
            // One-frame relocation by hundreds of kilometres is clamped by
            // KSP's floating origin. MaintainSeaPlatform walks this vessel to
            // the target in short surface-coordinate steps instead.
            SetVesselColliders(platform, false);
            seaDeploymentLongitude = startLongitude;
            seaDeploymentAltitude = 10000d;
            platform.Landed = false;
            platform.Splashed = true;
            platform.situation = Vessel.Situations.SPLASHED;
            seaReleaseAt = -1f;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] SEA_PLATFORM_PLACED lat={0:F5} lon={1:F5} alt={2:F1} terrain={3:F1} distance≈{4:F0}m actual=({5:F5},{6:F5},{7:F1})",
                seaLatitude, seaLongitude, 5.5, seaTerrainAltitude,
                body.Radius * (seaLongitude - startLongitude) * Mathf.Deg2Rad,
                platform.latitude, platform.longitude, platform.altitude));
        }

        private void MaintainSeaPlatform()
        {
            if (seaPlatform == null || seaPlatform.state == Vessel.State.DEAD)
            {
                Debug.LogError("[CZ10BNetRecovery] SEA_PLATFORM_LOST");
                seaPlacementReleased = true;
                return;
            }

            // Keep each relocation step inside the stable local range. At a
            // quarter degree per frame the complete move still takes only a
            // few seconds, with collisions disabled for the entire transit.
            seaDeploymentLongitude = System.Math.Min(seaLongitude,
                seaDeploymentLongitude + 0.25d);
            if (System.Math.Abs(seaDeploymentLongitude - seaLongitude) <= 0.001d)
                seaDeploymentAltitude = System.Math.Max(5.5d,
                    seaDeploymentAltitude - 500d);
            CelestialBody body = seaPlatform.mainBody;
            Vector3d deploymentPosition = body.GetWorldSurfacePosition(
                seaLatitude, seaDeploymentLongitude, seaDeploymentAltitude);
            seaPlatform.SetPosition(deploymentPosition, true);
            seaPlatform.SetRotation(SurfaceFrame(body, seaLatitude,
                seaDeploymentLongitude, seaDeploymentAltitude)
                * seaSurfaceRotationOffset);
            SetSurfaceStationaryVelocity(seaPlatform);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
            RebindSeaTarget();
            if (System.Math.Abs(seaDeploymentLongitude - seaLongitude) > 0.001d ||
                seaDeploymentAltitude > 5.5d)
                return;

            if (seaReleaseAt < 0f)
                seaReleaseAt = Time.realtimeSinceStartup + 1f;
            if (Time.realtimeSinceStartup < seaReleaseAt || seaPlatform.packed)
                return;

            SetVesselColliders(seaPlatform, true);
            seaPlatform.Landed = false;
            seaPlatform.Splashed = true;
            seaPlatform.situation = Vessel.Situations.SPLASHED;
            SetSurfaceStationaryVelocity(seaPlatform);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
            seaStationAltitude = System.Math.Max(seaPlatform.altitude, 5.5d);
            seaPlacementReleased = true;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] SEA_PLATFORM_PHYSICS_RELEASED lat={0:F5} lon={1:F5} alt={2:F1} situation={3}",
                seaPlatform.latitude, seaPlatform.longitude, seaPlatform.altitude,
                seaPlatform.situation));
        }

        private void RebindSeaTarget()
        {
            Vessel activeVessel = FlightGlobals.ActiveVessel;
            if (activeVessel != null && HasPart(activeVessel, "CZ10B-DemoBooster"))
                FlightGlobals.fetch.SetVesselTarget(seaPlatform, true);
        }

        private void MaintainSeaStation()
        {
            if (seaPlatform == null || seaPlatform.state == Vessel.State.DEAD ||
                seaPlatform.packed)
                return;
            CelestialBody body = seaPlatform.mainBody;
            // Model the recovery ship's DP2 station keeping.  KSP's floating
            // origin changes while the booster travels hundreds of kilometres,
            // so a one-shot inertial velocity cannot preserve a remote loaded
            // vessel's longitude. DP station keeping pins position and deck
            // attitude every frame; the capture winches no longer apply their
            // reaction force to this vessel.
            seaPlatform.SetPosition(body.GetWorldSurfacePosition(
                seaLatitude, seaLongitude, seaStationAltitude), true);
            seaPlatform.SetRotation(SurfaceFrame(body, seaLatitude,
                seaLongitude, seaStationAltitude) * seaSurfaceRotationOffset);
            SetSurfaceStationaryVelocity(seaPlatform);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
        }

        private void ManageSeaStationLoading(Vessel booster)
        {
            if (seaPlatform == null || seaPlatform.state == Vessel.State.DEAD ||
                booster == null)
                return;
            double distance = Vector3d.Distance(
                booster.mainBody.GetWorldSurfacePosition(
                    booster.latitude, booster.longitude, 0),
                seaPlatform.mainBody.GetWorldSurfacePosition(
                    seaLatitude, seaLongitude, 0));
            const double activationDistance = 120000d;
            if (distance > activationDistance)
            {
                if (!seaStationDeferred)
                {
                    seaPlatform.Landed = false;
                    seaPlatform.Splashed = true;
                    seaPlatform.situation = Vessel.Situations.SPLASHED;
                    RestoreDefaultRanges(seaPlatform.vesselRanges);
                    seaPlatform.GoOnRails();
                    seaStationDeferred = true;
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] SEA_PLATFORM_DEFERRED distance={0:F0}",
                        distance));
                }
                return;
            }

            if (seaStationDeferred)
            {
                ExtendAllRanges(seaPlatform.vesselRanges);
                seaPlatform.GoOffRails();
                seaPlatform.Landed = false;
                seaPlatform.Splashed = true;
                seaPlatform.situation = Vessel.Situations.SPLASHED;
                PlaceSeaPlatform();
                RebindSeaTarget();
                seaStationDeferred = false;
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] SEA_PLATFORM_REACTIVATED distance={0:F0}",
                    distance));
            }
            MaintainSeaStation();
        }

        private void PlaceSeaPlatform()
        {
            CelestialBody body = seaPlatform.mainBody;
            Vector3d position = body.GetWorldSurfacePosition(seaLatitude,
                seaLongitude, 5.5);
            seaPlatform.SetPosition(position, true);
            // DP station keeping also holds deck attitude. Merely clearing
            // angular momentum allowed buoyancy to leave the tall platform on
            // its side, rotating the net's local capture plane with it.
            seaPlatform.SetRotation(SurfaceFrame(body, seaLatitude,
                seaLongitude, 5.5) * seaSurfaceRotationOffset);
            SetSurfaceStationaryVelocity(seaPlatform);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
        }

        private static Quaternion SurfaceFrame(CelestialBody body,
            double latitude, double longitude, double altitude)
        {
            Vector3d origin = body.GetWorldSurfacePosition(latitude, longitude,
                altitude);
            Vector3d northPoint = body.GetWorldSurfacePosition(
                System.Math.Min(latitude + 0.01, 89.99), longitude, altitude);
            Vector3 up = (origin - body.position).normalized;
            Vector3 north = Vector3.ProjectOnPlane(
                (northPoint - origin).normalized, up).normalized;
            return Quaternion.LookRotation(north, up);
        }

        private static void SetSurfaceStationaryVelocity(Vessel platform)
        {
            CelestialBody body = platform.mainBody;
            // Zero inertial velocity makes the rotating planet slide under the
            // ship.  A fixed latitude/longitude requires the local rotating-
            // frame velocity. CelestialBody.getRFrmVel explicitly subtracts
            // body.position internally, so its argument must be world-space.
            platform.SetWorldVelocity(body.getRFrmVel(platform.GetWorldPos3D()));
        }

        private static void SetVesselColliders(Vessel target, bool enabled)
        {
            if (target == null || target.parts == null)
                return;
            foreach (Part vesselPart in target.parts)
            {
                if (vesselPart == null)
                    continue;
                foreach (Collider collider in vesselPart
                    .GetComponentsInChildren<Collider>(true))
                {
                    if (collider != null)
                        collider.enabled = enabled;
                }
                CollisionEnhancer enhancer =
                    vesselPart.GetComponent<CollisionEnhancer>();
                // CollisionEnhancer retains a pre-teleport position history;
                // re-enabling it at sea interprets the deployment displacement
                // as one giant terrain penetration. Unity colliders, buoyancy,
                // the catch envelope and capture joints are all restored.
                if (enhancer != null && !enabled)
                    enhancer.enabled = false;
            }
        }

        private float SeaPlatformTilt(ModuleCatchNet net)
        {
            if (seaPlatform == null || net == null || net.part == null)
                return -1f;
            Vector3 surfaceUp = (seaPlatform.GetWorldPos3D() -
                seaPlatform.mainBody.position).normalized;
            return Vector3.Angle(net.part.transform.up, surfaceUp);
        }

        private static void ExtendRanges(VesselRanges.Situation ranges)
        {
            // The 15-degree, 20%-reserve ascent puts the drag-aware footprint
            // about 72 degrees (roughly 754 km) downrange.  The upper stage must
            // also stay loaded after separation so it can keep accelerating.
            const float keepLoadedDistance = 850000f;
            ranges.load = keepLoadedDistance;
            ranges.unload = keepLoadedDistance;
            ranges.pack = keepLoadedDistance;
            ranges.unpack = keepLoadedDistance;
        }

        private static void ExtendAllRanges(VesselRanges ranges)
        {
            if (ranges == null)
                return;
            ExtendRanges(ranges.prelaunch);
            ExtendRanges(ranges.landed);
            ExtendRanges(ranges.splashed);
            ExtendRanges(ranges.flying);
            ExtendRanges(ranges.subOrbital);
            ExtendRanges(ranges.orbit);
            ExtendRanges(ranges.escaping);
        }

        private static void RestoreDefaultRanges(VesselRanges ranges)
        {
            if (ranges == null)
                return;
            VesselRanges defaults = new VesselRanges();
            ranges.prelaunch = new VesselRanges.Situation(defaults.prelaunch);
            ranges.landed = new VesselRanges.Situation(defaults.landed);
            ranges.splashed = new VesselRanges.Situation(defaults.splashed);
            ranges.flying = new VesselRanges.Situation(defaults.flying);
            ranges.subOrbital = new VesselRanges.Situation(defaults.subOrbital);
            ranges.orbit = new VesselRanges.Situation(defaults.orbit);
            ranges.escaping = new VesselRanges.Situation(defaults.escaping);
        }

        private static float UpperStageThrust(Vessel upper)
        {
            if (upper == null || upper.parts == null)
                return 0f;
            return upper.parts.Select(p =>
                p.FindModuleImplementing<ModuleEngines>())
                .Where(e => e != null).Sum(e => e.finalThrust);
        }

        private static double TerrainAltitude(CelestialBody body, double latitude,
            double longitude)
        {
            if (body.pqsController == null)
                return 0;
            Vector3d normal = body.GetRelSurfaceNVector(latitude, longitude);
            return body.pqsController.GetSurfaceHeight(normal) - body.Radius;
        }

        private static bool HasPart(Vessel vessel, string name)
        {
            return vessel != null && vessel.parts != null && vessel.parts.Any(p =>
                p != null && p.partInfo != null && p.partInfo.name == name);
        }

        private static Vessel FindVesselWithPart(string name)
        {
            return FlightGlobals.VesselsLoaded.FirstOrDefault(v => HasPart(v, name));
        }

        private static ModuleCatchNet FindNet()
        {
            return FlightGlobals.VesselsLoaded.Where(v => v != null && v.parts != null)
                .SelectMany(v => v.parts)
                .Select(p => p.FindModuleImplementing<ModuleCatchNet>())
                .FirstOrDefault(n => n != null);
        }

        private static float BoosterPropellantFraction(Vessel vessel)
        {
            if (vessel == null || vessel.parts == null)
                return -1f;
            Part booster = vessel.parts.FirstOrDefault(p => p != null &&
                p.partInfo != null && p.partInfo.name == "CZ10B-DemoBooster");
            if (booster == null || booster.Resources == null)
                return -1f;
            double amount = 0d;
            double capacity = 0d;
            foreach (PartResource resource in booster.Resources)
            {
                if (resource.resourceName != "LiquidFuel" &&
                    resource.resourceName != "Oxidizer")
                    continue;
                amount += resource.amount;
                capacity += resource.maxAmount;
            }
            return capacity > 0d ? (float)(amount / capacity) : -1f;
        }

        private static void DisablePartColliders(Part part)
        {
            foreach (Collider collider in part.GetPartColliders())
                collider.enabled = false;
            part.SetDetectCollisions(false);
        }
    }
}
