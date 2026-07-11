using System.Linq;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// One-shot acceptance sequence for the dedicated launch-pad rig. It only
    /// activates when all three identifying parts are present on the active vessel.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class DropTestFlightSequencer : MonoBehaviour
    {
        private bool active;
        private bool released;
        private bool reported;
        private float startRealtime;
        private float nextDiagnostic;
        private bool nudgeApplied;

        private void Start()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || vessel.parts == null)
                return;

            active = HasPart(vessel, "CZ10B-RecoveryPlatform") &&
                     HasPart(vessel, "CZ10B-TestSuspension") &&
                     HasPart(vessel, "CZ10B-DropTestMass");
            if (!active)
                return;

            startRealtime = Time.realtimeSinceStartup;
            Debug.Log("[CZ10BNetRecovery] DROP_TEST_FLIGHT_READY vessel=" + vessel.vesselName);
        }

        private void Update()
        {
            if (!active || reported)
                return;

            float elapsed = Time.realtimeSinceStartup - startRealtime;
            if (!released && elapsed >= 3f)
            {
                released = true;
                ModuleDecouple decoupler = FlightGlobals.ActiveVessel.parts
                    .Select(p => p.FindModuleImplementing<ModuleDecouple>())
                    .FirstOrDefault(module => module != null);
                if (decoupler == null)
                {
                    Debug.LogError("[CZ10BNetRecovery] DROP_TEST_RELEASE_FAILED no ModuleDecouple");
                    reported = true;
                    return;
                }
                DisablePartColliders(decoupler.part);
                Part suspension = FlightGlobals.ActiveVessel.parts.FirstOrDefault(p =>
                    p.partInfo != null && p.partInfo.name == "CZ10B-TestSuspension");
                if (suspension != null)
                    DisablePartColliders(suspension);
                decoupler.Decouple();
                nextDiagnostic = elapsed;
                Debug.Log("[CZ10BNetRecovery] DROP_TEST_RELEASED_DIRECT part=" +
                          decoupler.part.partInfo.name);
            }

            if (released && !nudgeApplied && elapsed >= 3.15f)
            {
                ModuleCatchHook hook = FlightGlobals.VesselsLoaded
                    .Where(v => v != null && v.parts != null)
                    .SelectMany(v => v.parts)
                    .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                    .FirstOrDefault(h => h != null && h.part.partInfo.name == "CZ10B-DropTestMass");
                ModuleCatchNet net = FlightGlobals.VesselsLoaded
                    .Where(v => v != null && v.parts != null)
                    .SelectMany(v => v.parts)
                    .Select(p => p.FindModuleImplementing<ModuleCatchNet>())
                    .FirstOrDefault(n => n != null);
                if (hook != null && net != null && hook.part.Rigidbody != null)
                {
                    if (hook.hookState != "Captured")
                    {
                        hook.vessel.GoOffRails();
                        hook.vessel.situation = Vessel.Situations.FLYING;
                        hook.part.Rigidbody.AddForce(-net.part.transform.up * 0.5f,
                            ForceMode.VelocityChange);
                    }
                    nudgeApplied = true;
                    Debug.Log("[CZ10BNetRecovery] DROP_TEST_NUDGE_STATE captured=" +
                              (hook.hookState == "Captured"));
                }
            }

            if (released && !reported && elapsed >= nextDiagnostic && elapsed < 11f)
            {
                LogGeometry(elapsed);
                nextDiagnostic += 0.5f;
            }

            if (released && elapsed >= 12f)
                ReportResult();
        }

        private void ReportResult()
        {
            reported = true;
            ModuleCatchHook capturedHook = FlightGlobals.VesselsLoaded
                .Where(v => v != null && v.parts != null)
                .SelectMany(v => v.parts)
                .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                .FirstOrDefault(h => h != null && h.hookState == "Captured");

            ModuleCatchNet net = FlightGlobals.VesselsLoaded
                .Where(v => v != null && v.parts != null)
                .SelectMany(v => v.parts)
                .Select(p => p.FindModuleImplementing<ModuleCatchNet>())
                .FirstOrDefault(n => n != null);

            if (capturedHook != null)
            {
                Debug.Log("[CZ10BNetRecovery] DROP_TEST_PASS hook=" +
                          capturedHook.part.partInfo.name + " netState=" +
                          (net == null ? "missing" : net.netState));
                ScreenMessages.PostScreenMessage("CZ-10B DROP TEST: PASS", 10f,
                    ScreenMessageStyle.UPPER_CENTER);
            }
            else
            {
                Debug.LogError("[CZ10BNetRecovery] DROP_TEST_FAIL netState=" +
                               (net == null ? "missing" : net.netState));
                ScreenMessages.PostScreenMessage("CZ-10B DROP TEST: FAIL - inspect KSP.log", 10f,
                    ScreenMessageStyle.UPPER_CENTER);
            }
        }

        private static bool HasPart(Vessel vessel, string name)
        {
            return vessel.parts.Any(part => part != null && part.partInfo != null &&
                                             part.partInfo.name == name);
        }

        private static void LogGeometry(float elapsed)
        {
            ModuleCatchNet net = FlightGlobals.VesselsLoaded
                .Where(v => v != null && v.parts != null)
                .SelectMany(v => v.parts)
                .Select(p => p.FindModuleImplementing<ModuleCatchNet>())
                .FirstOrDefault(n => n != null);
            ModuleCatchHook hook = FlightGlobals.VesselsLoaded
                .Where(v => v != null && v.parts != null)
                .SelectMany(v => v.parts)
                .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                .FirstOrDefault(h => h != null && h.part.partInfo.name == "CZ10B-DropTestMass");

            if (net == null || hook == null)
            {
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] DROP_TEST_GEOMETRY t={0:F1} vessels={1} net={2} hook={3}",
                    elapsed, FlightGlobals.VesselsLoaded.Count, net != null, hook != null));
                return;
            }

            Vector3 point = hook.GetHookWorldPoints().First();
            Vector3 local = net.part.transform.InverseTransformPoint(point);
            Vector3 relativeVelocity = (Vector3)(hook.vessel.srf_velocity - net.vessel.srf_velocity);
            float normalSpeed = Vector3.Dot(relativeVelocity, net.part.transform.up);
            Vector3 lateral = relativeVelocity - normalSpeed * net.part.transform.up;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] DROP_TEST_GEOMETRY t={0:F1} vessels={1} same={2} local=({3:F2},{4:F2},{5:F2}) normalV={6:F2} lateralV={7:F2} hookState={8} situation={9}",
                elapsed, FlightGlobals.VesselsLoaded.Count, hook.vessel == net.vessel,
                local.x, local.y, local.z, normalSpeed, lateral.magnitude,
                hook.hookState, hook.vessel.situation));
        }

        private static void DisablePartColliders(Part part)
        {
            foreach (Collider collider in part.GetPartColliders())
                collider.enabled = false;
            part.SetDetectCollisions(false);
        }
    }
}
