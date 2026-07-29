using System;
using System.IO;
using System.Linq;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Opt-in runtime validator for generated craft templates. It is dormant unless
    /// GameData/CZ10BRecovery/PluginData/validate-crafts.once exists. This lets the
    /// real KSP loader, rather than a hand-written parser, be the acceptance test.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public sealed class CraftTemplateValidator : MonoBehaviour
    {
        private void Awake()
        {
            string root = Path.Combine(KSPUtil.ApplicationRootPath,
                "GameData", "CZ10BRecovery");
            string marker = Path.Combine(root, "PluginData", "validate-crafts.once");
            string dropMarker = Path.Combine(root, "PluginData", "launch-drop-test.once");
            string hoverMarker = Path.Combine(root, "PluginData", "launch-hover-test.once");
            string missionMarker = Path.Combine(root, "PluginData", "launch-mission-test.once");
            string seaMissionMarker = Path.Combine(root, "PluginData", "launch-sea-mission-test.once");
            if (!File.Exists(marker) && !File.Exists(dropMarker) &&
                !File.Exists(hoverMarker) && !File.Exists(missionMarker) &&
                !File.Exists(seaMissionMarker))
                return;

            if (File.Exists(marker))
            {
                Debug.Log("[CZ10BNetRecovery] CRAFT_VALIDATION_START marker=" + marker);
                string directory = Path.Combine(root, "CraftTemplates", "VAB");
                foreach (string path in Directory.GetFiles(directory, "*.craft"))
                {
                    ShipConstruct construct = null;
                    try
                    {
                        construct = ShipConstruction.LoadShip(path);
                        if (construct == null || construct.parts == null || construct.parts.Count == 0)
                            throw new InvalidDataException("KSP returned an empty ShipConstruct");
                        ValidateGridFinMounting(construct, path);

                        Debug.Log(string.Format(
                            "[CZ10BNetRecovery] CRAFT_VALID name={0} parts={1} root={2}",
                            Path.GetFileName(path), construct.parts.Count,
                            construct.parts[0].partInfo == null
                                ? construct.parts[0].name
                                : construct.parts[0].partInfo.name));
                    }
                    catch (Exception error)
                    {
                        Debug.LogError(string.Format(
                            "[CZ10BNetRecovery] CRAFT_INVALID name={0} error={1}",
                            Path.GetFileName(path), error));
                    }
                    finally
                    {
                        if (construct != null && construct.parts != null)
                        {
                            foreach (Part loadedPart in construct.parts)
                            {
                                if (loadedPart != null)
                                    UnityEngine.Object.Destroy(loadedPart.gameObject);
                            }
                        }
                    }
                }
                File.Delete(marker);
            }

            if (File.Exists(dropMarker))
                PrepareTest(dropMarker, "CZ10BRecoveryTest",
                    "CZ10B Cable Capture Drop Test.craft",
                    "launch-drop-test-from-spacecenter.once", "DROP_TEST");
            else if (File.Exists(hoverMarker))
                PrepareTest(hoverMarker, "CZ10BHoverTest",
                    "CZ10B kOS Hover Recovery Test.craft",
                    "launch-hover-test-from-spacecenter.once", "HOVER_TEST");
            else if (File.Exists(missionMarker))
                PrepareTest(missionMarker, "CZ10BMissionTest",
                    "CZ10B Full Mission Recovery Test.craft",
                    "launch-mission-test-from-spacecenter.once", "MISSION_TEST");
            else if (File.Exists(seaMissionMarker))
                PrepareTest(seaMissionMarker, "CZ10BSeaMissionTest",
                    "CZ10B Full Mission Recovery Test.craft",
                    "launch-sea-mission-test-from-spacecenter.once", "SEA_MISSION_TEST");
        }

        private static void PrepareTest(string marker, string defaultSaveName,
            string craftFileName, string spaceCenterMarkerName, string logPrefix)
        {
            try
            {
                string saveName = File.ReadAllText(marker).Trim();
                if (string.IsNullOrEmpty(saveName))
                    saveName = defaultSaveName;
                File.Delete(marker);

                Game game = GamePersistence.CreateNewGame(saveName, Game.Modes.SANDBOX,
                    new GameParameters(), "Squad/Flags/default", GameScenes.SPACECENTER,
                    EditorFacility.VAB);
                if (game == null)
                    throw new InvalidDataException("GamePersistence.CreateNewGame returned null");

                HighLogic.CurrentGame = game;
                HighLogic.SaveFolder = saveName;

                string craftPath = Path.Combine(KSPUtil.ApplicationRootPath, "saves",
                    saveName, "Ships", "VAB", craftFileName);
                Directory.CreateDirectory(Path.GetDirectoryName(craftPath));
                string modRoot = Directory.GetParent(Path.GetDirectoryName(marker)).FullName;
                string sourceCraft = Path.Combine(modRoot, "CraftTemplates", "VAB",
                    craftFileName);
                File.Copy(sourceCraft, craftPath, true);
                GamePersistence.SaveGame(game, "persistent", saveName, SaveMode.OVERWRITE);

                string spaceCenterMarker = Path.Combine(modRoot, "PluginData",
                    spaceCenterMarkerName);
                File.WriteAllText(spaceCenterMarker, saveName);
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] {0}_PREPARED save={1} craft={2}",
                    logPrefix, saveName, craftPath));
                HighLogic.LoadScene(GameScenes.SPACECENTER);
            }
            catch (Exception error)
            {
                Debug.LogError("[CZ10BNetRecovery] " + logPrefix +
                               "_LAUNCH_FAILED " + error);
            }
        }

        private static void ValidateGridFinMounting(ShipConstruct construct,
            string path)
        {
            Part booster = construct.parts.FirstOrDefault(part => part != null &&
                part.partInfo != null &&
                part.partInfo.name == "CZ10B-DemoBooster");
            Part[] gridFins = construct.parts.Where(part => part != null &&
                part.partInfo != null &&
                part.partInfo.name == "CZ10B-GridFin").ToArray();
            if (gridFins.Length == 0)
                return;
            if (booster == null)
                throw new InvalidDataException(
                    "grid fins exist without CZ10B-DemoBooster");
            if (gridFins.Length != 4)
                throw new InvalidDataException(
                    "expected four grid fins, found " + gridFins.Length);

            const float expectedOriginRadius = 1.275f;
            const float mountingTolerance = 0.03f;
            float maximumMountingError = 0f;
            foreach (Part gridFin in gridFins)
            {
                if (gridFin.parent != booster)
                    throw new InvalidDataException(
                        "grid fin is not surface-attached to the booster");
                Vector3 localPosition = booster.transform.InverseTransformPoint(
                    gridFin.transform.position);
                float originRadius = new Vector2(
                    localPosition.x, localPosition.z).magnitude;
                float mountingError = Mathf.Abs(
                    originRadius - expectedOriginRadius);
                maximumMountingError = Mathf.Max(
                    maximumMountingError, mountingError);
                if (mountingError > mountingTolerance)
                    throw new InvalidDataException(string.Format(
                        "grid fin origin radius {0:F3} m leaves it off the 1.25 m booster skin",
                        originRadius));
            }
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] CRAFT_GRID_FIN_ATTACHMENT_VALID name={0} count={1} maxOriginRadiusError={2:F4}",
                Path.GetFileName(path), gridFins.Length,
                maximumMountingError));
        }
    }
}
