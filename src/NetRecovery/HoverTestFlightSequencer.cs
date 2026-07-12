using System.Linq;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Releases the dedicated low-altitude rig and observes the result. This
    /// harness deliberately issues no attitude, throttle, engine or force commands;
    /// those are the responsibility of cz10b/hover_test.ks running on the booster.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class HoverTestFlightSequencer : MonoBehaviour
    {
        private bool active;
        private bool released;
        private bool switched;
        private bool reported;
        private float startRealtime;
        private float capturedAt = -1f;
        private float stableCapturedAt = -1f;
        private float nextDiagnostic;
        private bool everPowered;

        private void Start()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || vessel.parts == null)
                return;

            active = HasPart(vessel, "CZ10B-RecoveryPlatform") &&
                     HasPart(vessel, "CZ10B-HoverSuspension") &&
                     HasPart(vessel, "CZ10B-HoverTestBooster");
            if (!active)
                return;

            startRealtime = Time.realtimeSinceStartup;
            Debug.Log("[CZ10BNetRecovery] HOVER_TEST_FLIGHT_READY vessel=" +
                      vessel.vesselName);
        }

        private void Update()
        {
            if (!active || reported)
                return;

            float elapsed = Time.realtimeSinceStartup - startRealtime;
            if (!released && elapsed >= 3f)
                ReleaseBooster(elapsed);

            if (released && !switched && elapsed >= 3.1f)
            {
                Vessel booster = FindBoosterVessel();
                if (booster != null)
                {
                    booster.GoOffRails();
                    booster.situation = Vessel.Situations.FLYING;
                    FlightGlobals.SetActiveVessel(booster);
                    ModuleCatchNet net = FindNet();
                    if (net != null)
                        FlightGlobals.fetch.SetVesselTarget(net.vessel, true);
                    switched = true;
                    nextDiagnostic = elapsed;
                    Debug.Log("[CZ10BNetRecovery] HOVER_TEST_ACTIVE_BOOSTER vessel=" +
                              booster.vesselName + " target=" +
                              (net == null ? "missing" : net.vessel.vesselName));
                }
            }

            ModuleCatchHook hook = FindHoverHook();
            Vessel boosterVessel = FindBoosterVessel();
            ModuleEngines engine = FindEngine(boosterVessel);
            if (engine != null && engine.finalThrust > 10f)
                everPowered = true;
            if (switched && elapsed >= nextDiagnostic)
            {
                LogDiagnostic(elapsed, hook);
                nextDiagnostic = elapsed + 1f;
            }

            if (hook != null && hook.hookState == "Captured")
            {
                if (capturedAt < 0f)
                    capturedAt = elapsed;
                ModuleCatchNet net = FindNet();
                float angularRate = boosterVessel == null ? float.MaxValue :
                    boosterVessel.angularVelocity.magnitude * Mathf.Rad2Deg;
                bool payoutComplete = net != null &&
                    net.cableDeflection >= net.captureSettleDrop - 1f;
                bool stable = boosterVessel != null &&
                    Mathf.Abs((float)boosterVessel.verticalSpeed) <= 4f &&
                    boosterVessel.horizontalSrfSpeed <= 3f && angularRate <= 12f &&
                    payoutComplete;
                if (stable)
                {
                    if (stableCapturedAt < 0f)
                        stableCapturedAt = elapsed;
                }
                else
                    stableCapturedAt = -1f;
                if (stableCapturedAt >= 0f && elapsed - stableCapturedAt >= 8f &&
                    everPowered)
                    ReportPass(hook);
            }
            else
            {
                capturedAt = -1f;
                stableCapturedAt = -1f;
            }

            if (elapsed >= 82f)
                ReportFailure(hook);
        }

        private void ReleaseBooster(float elapsed)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            ModuleDecouple decoupler = vessel.parts
                .Select(p => p.FindModuleImplementing<ModuleDecouple>())
                .FirstOrDefault(module => module != null);
            if (decoupler == null)
            {
                Debug.LogError("[CZ10BNetRecovery] HOVER_TEST_RELEASE_FAILED no ModuleDecouple");
                reported = true;
                return;
            }

            DisablePartColliders(decoupler.part);
            Part suspension = vessel.parts.FirstOrDefault(p =>
                p.partInfo != null && p.partInfo.name == "CZ10B-HoverSuspension");
            if (suspension != null)
                DisablePartColliders(suspension);
            decoupler.Decouple();
            released = true;
            nextDiagnostic = elapsed;
            Debug.Log("[CZ10BNetRecovery] HOVER_TEST_RELEASED_DIRECT part=" +
                      decoupler.part.partInfo.name);
        }

        private void ReportPass(ModuleCatchHook hook)
        {
            reported = true;
            ModuleCatchNet net = FindNet();
            Debug.Log("[CZ10BNetRecovery] HOVER_TEST_PASS hook=" +
                      hook.part.partInfo.name + " netState=" +
                      (net == null ? "missing" : net.netState) +
                      " powered=" + everPowered);
            ScreenMessages.PostScreenMessage("CZ-10B kOS HOVER TEST: PASS", 10f,
                ScreenMessageStyle.UPPER_CENTER);
        }

        private void ReportFailure(ModuleCatchHook hook)
        {
            reported = true;
            ModuleCatchNet net = FindNet();
            Debug.LogError("[CZ10BNetRecovery] HOVER_TEST_FAIL hookState=" +
                           (hook == null ? "missing" : hook.hookState) +
                           " netState=" + (net == null ? "missing" : net.netState) +
                           " powered=" + everPowered);
            ScreenMessages.PostScreenMessage(
                "CZ-10B kOS HOVER TEST: FAIL - inspect KSP.log", 10f,
                ScreenMessageStyle.UPPER_CENTER);
        }

        private static void LogDiagnostic(float elapsed, ModuleCatchHook hook)
        {
            ModuleCatchNet net = FindNet();
            Vessel booster = FindBoosterVessel();
            if (booster == null || net == null)
            {
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] HOVER_TEST_STATUS t={0:F1} booster={1} net={2} hook={3}",
                    elapsed, booster != null, net != null, hook != null));
                return;
            }

            Vector3 relativeVelocity = (Vector3)(booster.srf_velocity - net.vessel.srf_velocity);
            Vector3 up = net.part.transform.up;
            float vertical = Vector3.Dot(relativeVelocity, up);
            Vector3 lateral = relativeVelocity - vertical * up;
            Vector3 local = net.part.transform.InverseTransformPoint(
                hook == null ? booster.transform.position : hook.GetHookWorldPoints().First());
            ModuleEngines engine = FindEngine(booster);
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] HOVER_TEST_STATUS t={0:F1} local=({1:F2},{2:F2},{3:F2}) vertical={4:F2} lateral={5:F2} throttle={6:F3} thrust={7:F1} hookState={8} angular={9:F2} sag={10:F2}",
                elapsed, local.x, local.y, local.z, vertical, lateral.magnitude,
                engine == null ? 0f : engine.currentThrottle,
                engine == null ? 0f : engine.finalThrust,
                hook == null ? "missing" : hook.hookState,
                booster.angularVelocity.magnitude * Mathf.Rad2Deg,
                net.cableDeflection));
        }

        private static bool HasPart(Vessel vessel, string name)
        {
            return vessel.parts.Any(part => part != null && part.partInfo != null &&
                                             part.partInfo.name == name);
        }

        private static Vessel FindBoosterVessel()
        {
            return FlightGlobals.VesselsLoaded.FirstOrDefault(v => v != null &&
                v.parts != null && HasPart(v, "CZ10B-HoverTestBooster"));
        }

        private static ModuleEngines FindEngine(Vessel vessel)
        {
            return vessel == null ? null : vessel.parts
                .Select(p => p.FindModuleImplementing<ModuleEngines>())
                .FirstOrDefault(e => e != null);
        }

        private static ModuleCatchHook FindHoverHook()
        {
            Vessel booster = FindBoosterVessel();
            return booster == null ? null : booster.parts
                .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                .FirstOrDefault(h => h != null && h.part.partInfo.name ==
                                     "CZ10B-HoverTestBooster");
        }

        private static ModuleCatchNet FindNet()
        {
            return FlightGlobals.VesselsLoaded
                .Where(v => v != null && v.parts != null)
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
