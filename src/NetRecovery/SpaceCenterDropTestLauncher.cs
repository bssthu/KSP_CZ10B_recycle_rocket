using System;
using System.IO;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Launches only after the Space Center scene has created facility spawn
    /// transforms. Direct MainMenu-to-Flight launch leaves LaunchPad_spawn null.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
    public sealed class SpaceCenterDropTestLauncher : MonoBehaviour
    {
        private string marker;
        private string saveName;
        private string craftFileName;
        private string logPrefix;
        private bool seaMission;
        private float readyAt;
        private bool pending;

        private void Awake()
        {
            string pluginData = Path.Combine(KSPUtil.ApplicationRootPath, "GameData",
                "CZ10BRecovery", "PluginData");
            string hoverMarker = Path.Combine(pluginData,
                "launch-hover-test-from-spacecenter.once");
            string dropMarker = Path.Combine(pluginData,
                "launch-drop-test-from-spacecenter.once");
            string missionMarker = Path.Combine(pluginData,
                "launch-mission-test-from-spacecenter.once");
            string seaMissionMarker = Path.Combine(pluginData,
                "launch-sea-mission-test-from-spacecenter.once");
            if (File.Exists(seaMissionMarker))
            {
                marker = seaMissionMarker;
                craftFileName = "CZ10B Full Mission Recovery Test.craft";
                logPrefix = "SEA_MISSION_TEST";
                seaMission = true;
            }
            else if (File.Exists(missionMarker))
            {
                marker = missionMarker;
                craftFileName = "CZ10B Full Mission Recovery Test.craft";
                logPrefix = "MISSION_TEST";
            }
            else if (File.Exists(hoverMarker))
            {
                marker = hoverMarker;
                craftFileName = "CZ10B kOS Hover Recovery Test.craft";
                logPrefix = "HOVER_TEST";
            }
            else if (File.Exists(dropMarker))
            {
                marker = dropMarker;
                craftFileName = "CZ10B Cable Capture Drop Test.craft";
                logPrefix = "DROP_TEST";
            }
            else
                return;

            saveName = File.ReadAllText(marker).Trim();
            SetNextKscLocalSixAm();
            readyAt = Time.realtimeSinceStartup + 3f;
            pending = true;
            Debug.Log("[CZ10BNetRecovery] " + logPrefix +
                      "_SPACECENTER_READY save=" + saveName);
        }

        private static void SetNextKscLocalSixAm()
        {
            CelestialBody kerbin = FlightGlobals.GetBodyByName("Kerbin");
            CelestialBody sun = FlightGlobals.GetBodyByName("Sun");
            if (kerbin == null || sun == null || kerbin.rotationPeriod <= 0)
                return;

            const double kscLatitude = -0.0972;
            const double kscLongitude = -74.5577;
            Vector3d surface = kerbin.GetWorldSurfacePosition(kscLatitude,
                kscLongitude, 0);
            Vector3d axis = kerbin.transform.up.normalized;
            Vector3d localUp = (surface - kerbin.position).normalized;
            Vector3d sunDirection = (sun.position - kerbin.position).normalized;
            Vector3d upPlane = Vector3d.Exclude(axis, localUp).normalized;
            Vector3d sunPlane = Vector3d.Exclude(axis, sunDirection).normalized;
            double angle = Math.Atan2(Vector3d.Dot(axis,
                Vector3d.Cross(upPlane, sunPlane)),
                Vector3d.Dot(upPlane, sunPlane));
            if (angle < 0)
                angle += Math.PI * 2;
            double delta = angle / (Math.PI * 2) * kerbin.rotationPeriod;
            // Solar 06:00 is one quarter rotation before local noon. Launching
            // at sunrise makes the downrange recovery site approach midday
            // instead of sunset after the roughly twelve-minute mission.
            delta -= kerbin.rotationPeriod * 0.25d;
            if (delta < 60)
                delta += kerbin.rotationPeriod;
            Planetarium.SetUniversalTime(Planetarium.GetUniversalTime() + delta);
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] KSC_LOCAL_0600_SET delta={0:F0}s ut={1:F0}",
                delta, Planetarium.GetUniversalTime()));
        }

        private void Update()
        {
            if (!pending || Time.realtimeSinceStartup < readyAt)
                return;

            pending = false;
            try
            {
                File.Delete(marker);
                string craftPath = Path.Combine(KSPUtil.ApplicationRootPath, "saves",
                    saveName, "Ships", "VAB", craftFileName);
                Debug.Log("[CZ10BNetRecovery] " + logPrefix +
                          "_LAUNCH craft=" + craftPath);
                if (seaMission)
                {
                    string flightMarker = Path.Combine(KSPUtil.ApplicationRootPath,
                        "GameData", "CZ10BRecovery", "PluginData",
                        "sea-mission-flight.active");
                    File.WriteAllText(flightMarker, saveName);
                }
                FlightDriver.StartWithNewLaunch(craftPath, "Squad/Flags/default",
                    "LaunchPad", new VesselCrewManifest());
            }
            catch (Exception error)
            {
                Debug.LogError("[CZ10BNetRecovery] " + logPrefix +
                               "_LAUNCH_FAILED " + error);
            }
        }
    }
}
