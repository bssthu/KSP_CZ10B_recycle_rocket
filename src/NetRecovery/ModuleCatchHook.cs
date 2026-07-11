using System.Collections.Generic;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Marker module for the four lightweight hook mechanisms carried by the booster.
    /// The net owns detection and joints so several hooks can be captured atomically.
    /// </summary>
    public sealed class ModuleCatchHook : PartModule
    {
        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true,
            guiName = "Catch hook armed")]
        [UI_Toggle(enabledText = "ARMED", disabledText = "SAFE")]
        public bool armed = true;

        [KSPField(guiActive = true, guiName = "Hook state")]
        public string hookState = "Ready";

        // A normal attachable hook uses one point at the part origin. Integrated
        // demo stages can expose four physical anchor points without four extra
        // parts, keeping the first playable craft deliberately simple.
        [KSPField] public int virtualHookCount = 1;
        [KSPField] public float hookRadius = 0f;
        [KSPField] public float hookOffsetY = 0f;

        private readonly List<LineRenderer> hookRenderers = new List<LineRenderer>();

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            if ((HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor) &&
                virtualHookCount > 1 && hookRadius > 0f)
                CreateHookVisuals();
        }

        internal IEnumerable<Vector3> GetHookWorldPoints()
        {
            if (virtualHookCount <= 1 || hookRadius <= 0f)
            {
                yield return part.transform.TransformPoint(new Vector3(0f, hookOffsetY, 0f));
                yield break;
            }

            int count = Mathf.Max(1, virtualHookCount);
            for (int index = 0; index < count; ++index)
            {
                float angle = index * Mathf.PI * 2f / count;
                Vector3 local = new Vector3(Mathf.Cos(angle) * hookRadius,
                    hookOffsetY, Mathf.Sin(angle) * hookRadius);
                yield return part.transform.TransformPoint(local);
            }
        }

        internal void SetCaptured(bool captured)
        {
            hookState = captured ? "Captured" : (armed ? "Ready" : "Safe");
        }

        public void OnDestroy()
        {
            foreach (LineRenderer renderer in hookRenderers)
            {
                if (renderer != null)
                    UnityEngine.Object.Destroy(renderer.gameObject);
            }
            hookRenderers.Clear();
        }

        private void CreateHookVisuals()
        {
            Color color = new Color(1f, 0.45f, 0.08f, 1f);
            foreach (Vector3 worldPoint in GetHookWorldPoints())
            {
                Vector3 localPoint = part.transform.InverseTransformPoint(worldPoint);
                GameObject arm = new GameObject("CZ10B_VirtualCatchHook");
                arm.transform.SetParent(part.transform, false);
                LineRenderer renderer = arm.AddComponent<LineRenderer>();
                renderer.useWorldSpace = false;
                renderer.positionCount = 3;
                renderer.SetPosition(0, new Vector3(0f, hookOffsetY, 0f));
                renderer.SetPosition(1, localPoint);
                renderer.SetPosition(2, localPoint + new Vector3(0f, -0.45f, 0f));
                renderer.startWidth = 0.10f;
                renderer.endWidth = 0.16f;
                renderer.startColor = color;
                renderer.endColor = color;
                Shader shader = Shader.Find("KSP/Unlit") ?? Shader.Find("Unlit/Color");
                if (shader != null)
                    renderer.material = new Material(shader) { color = color };
                hookRenderers.Add(renderer);
            }
        }
    }
}
