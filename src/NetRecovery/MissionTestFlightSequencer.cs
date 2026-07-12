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
        private float startRealtime;
        private double startUniversalTime;
        private float capturedAt = -1f;
        private float nextStatus;
        private bool seaMission;
        private Vessel pendingSeaPlatform;
        private float seaMoveAt;
        private Vessel seaPlatform;
        private double seaLatitude;
        private double seaLongitude;
        private double seaTerrainAltitude;
        private bool seaPlacementReleased;
        private float seaReleaseAt;

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
            if (!released && elapsed >= 3f)
                ReleaseVehicle();
            if (released && !switched && elapsed >= 3.1f)
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
            if (seaPlatform != null && !seaPlacementReleased)
                MaintainSeaPlatform();

            Vessel booster = FindVesselWithPart("CZ10B-DemoBooster");
            if (booster != null)
            {
                reachedAltitude |= booster.altitude > 2000;
                ModuleEngines engine = booster.parts
                    .Select(p => p.FindModuleImplementing<ModuleEngines>())
                    .FirstOrDefault(e => e != null);
                everPowered |= engine != null && engine.finalThrust > 10f;
            }
            Vessel upper = FindVesselWithPart("CZ10B-DemoUpperStage");
            upperSeparated |= upper != null && booster != null && upper != booster;

            ModuleCatchHook hook = booster == null ? null : booster.parts
                .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                .FirstOrDefault(h => h != null);
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
                    capturedAt = elapsed;
                if (elapsed - capturedAt >= 8f && everPowered && reachedAltitude && upperSeparated)
                    Report(true, hook);
            }
            else
                capturedAt = -1f;

            if (elapsed >= nextStatus)
            {
                nextStatus = elapsed + 5f;
                ModuleCatchNet net = FindNet();
                double targetDistance = booster == null || seaPlatform == null
                    ? -1 : Vector3d.Distance(
                        booster.mainBody.GetWorldSurfacePosition(
                            booster.latitude, booster.longitude, 0),
                        seaPlatform.mainBody.GetWorldSurfacePosition(
                            seaPlatform.latitude, seaPlatform.longitude, 0));
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] {0}_STATUS t={1:F1} altitude={2:F0} lat={3:F5} lon={4:F5} horizontal={5:F1} targetDistance={6:F0} powered={7} high={8} separated={9} hook={10} net={11}",
                    Prefix,
                    elapsed, booster == null ? -1 : booster.altitude,
                    booster == null ? 0 : booster.latitude,
                    booster == null ? 0 : booster.longitude,
                    booster == null ? 0 : booster.horizontalSrfSpeed,
                    targetDistance, everPowered, reachedAltitude, upperSeparated,
                    hook == null ? "missing" : hook.hookState,
                    net == null ? "missing" : net.netState));
                if (seaMission && seaPlatform != null)
                {
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] SEA_PLATFORM_STATUS lat={0:F5} lon={1:F5} alt={2:F1} terrain={3:F1} situation={4} packed={5} released={6}",
                        seaPlatform.latitude, seaPlatform.longitude,
                        seaPlatform.altitude, seaTerrainAltitude,
                        seaPlatform.situation, seaPlatform.packed,
                        seaPlacementReleased));
                }
            }
            // Use simulated mission time for the long acceptance deadline. Near
            // two loaded vessels KSP can run well below real time, especially in
            // a hidden/background test; wall-clock timeout would reject a valid
            // trajectory that is still advancing normally in physics time.
            if (Planetarium.GetUniversalTime() - startUniversalTime >= 600d)
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
            booster.situation = Vessel.Situations.FLYING;
            FlightGlobals.SetActiveVessel(booster);
            if (net != null)
                FlightGlobals.fetch.SetVesselTarget(net.vessel, true);
            if (seaMission && net != null)
            {
                pendingSeaPlatform = net.vessel;
                seaMoveAt = Time.realtimeSinceStartup + 0.75f;
            }
            switched = true;
            Debug.Log("[CZ10BNetRecovery] " + Prefix + "_RELEASED target=" +
                      (net == null ? "missing" : net.vessel.vesselName));
        }

        private void Report(bool pass, ModuleCatchHook hook)
        {
            reported = true;
            string detail = string.Format(
                " powered={0} high={1} separated={2} hook={3}", everPowered,
                reachedAltitude, upperSeparated, hook == null ? "missing" : hook.hookState);
            if (pass)
                Debug.Log("[CZ10BNetRecovery] " + Prefix + "_PASS" + detail);
            else
                Debug.LogError("[CZ10BNetRecovery] " + Prefix + "_FAIL" + detail);
        }

        private string Prefix
        {
            get { return seaMission ? "SEA_MISSION_TEST" : "MISSION_TEST"; }
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
            seaLatitude = platform.latitude;
            double startLongitude = platform.longitude;
            // Put the ship under the measured down-range arc. The old 6 km test
            // position forced an unrealistic continuous boost-back immediately
            // after separation; about 26 km preserves the apogee coast while
            // leaving enough range to brake the horizontal velocity at terminal.
            seaLongitude = startLongitude + 2.50;
            seaTerrainAltitude = TerrainAltitude(body, seaLatitude, seaLongitude);
            for (double offset = 2.50; offset <= 5.00; offset += 0.20)
            {
                double candidateLongitude = startLongitude + offset;
                double candidateTerrain = TerrainAltitude(body, seaLatitude,
                    candidateLongitude);
                if (candidateTerrain <= 0.5)
                {
                    seaLongitude = candidateLongitude;
                    seaTerrainAltitude = candidateTerrain;
                    break;
                }
            }
            platform.GoOffRails();
            platform.Landed = false;
            platform.Splashed = true;
            platform.situation = Vessel.Situations.SPLASHED;
            seaReleaseAt = Time.realtimeSinceStartup + 3f;
            PlaceSeaPlatform();
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

            if (Time.realtimeSinceStartup < seaReleaseAt || seaPlatform.packed)
            {
                PlaceSeaPlatform();
                RebindSeaTarget();
                return;
            }

            seaPlatform.Landed = false;
            seaPlatform.Splashed = true;
            seaPlatform.situation = Vessel.Situations.SPLASHED;
            seaPlatform.SetWorldVelocity(Vector3d.zero);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
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

        private void PlaceSeaPlatform()
        {
            CelestialBody body = seaPlatform.mainBody;
            Vector3d position = body.GetWorldSurfacePosition(seaLatitude,
                seaLongitude, 5.5);
            seaPlatform.SetPosition(position, true);
            seaPlatform.SetWorldVelocity(Vector3d.zero);
            seaPlatform.angularVelocity = Vector3.zero;
            seaPlatform.angularMomentum = Vector3.zero;
        }

        private static void ExtendRanges(VesselRanges.Situation ranges)
        {
            const float keepLoadedDistance = 100000f;
            ranges.load = keepLoadedDistance;
            ranges.unload = keepLoadedDistance;
            ranges.pack = keepLoadedDistance;
            ranges.unpack = keepLoadedDistance;
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

        private static void DisablePartColliders(Part part)
        {
            foreach (Collider collider in part.GetPartColliders())
                collider.enabled = false;
            part.SetDetectCollisions(false);
        }
    }
}
