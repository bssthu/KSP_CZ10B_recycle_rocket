using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Defines a rectangular horizontal cable-net and performs envelope-gated soft capture.
    /// Local X/Z span the net and local +Y is the upward normal.
    /// </summary>
    public sealed class ModuleCatchNet : PartModule
    {
        [KSPField] public float halfWidth = 10f;
        [KSPField] public float halfLength = 10f;
        [KSPField] public float planeOffset = 0.4f;
        [KSPField] public float detectionDepth = 1.5f;
        [KSPField] public float maxClosingSpeed = 7.5f;
        [KSPField] public float maxLateralSpeed = 4f;
        [KSPField] public float maxTilt = 15f;
        [KSPField] public float jointTravel = 1.5f;
        [KSPField] public float spring = 180000f;
        [KSPField] public float damper = 45000f;
        [KSPField] public float maximumForce = 2000000f;
        [KSPField] public bool debugLogging = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true,
            guiName = "Capture system")]
        [UI_Toggle(enabledText = "ARMED", disabledText = "SAFE")]
        public bool armed = true;

        [KSPField(guiActive = true, guiName = "Net state")]
        public string netState = "Ready";

        private readonly Dictionary<Guid, List<ConfigurableJoint>> captures =
            new Dictionary<Guid, List<ConfigurableJoint>>();
        private readonly List<LineRenderer> cableRenderers = new List<LineRenderer>();
        private double lastScan;
        private double lastDebug;

        [KSPEvent(guiActive = true, guiName = "Release captured stage", active = true)]
        public void ReleaseCapturedStage()
        {
            foreach (List<ConfigurableJoint> joints in captures.Values)
            {
                foreach (ConfigurableJoint joint in joints)
                {
                    if (joint != null)
                        UnityEngine.Object.Destroy(joint);
                }
            }

            captures.Clear();
            netState = armed ? "Ready" : "Safe";
            ScreenMessages.PostScreenMessage("CZ-10B net: captured stage released", 4f,
                ScreenMessageStyle.UPPER_CENTER);
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            if (HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor)
                CreateCableVisuals();
        }

        public override void OnInactive()
        {
            ReleaseCapturedStage();
            DestroyCableVisuals();
            base.OnInactive();
        }

        public void OnDestroy()
        {
            DestroyCableVisuals();
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();

            double universalTime = Planetarium.GetUniversalTime();
            if (debugLogging && universalTime - lastDebug >= 0.5)
            {
                lastDebug = universalTime;
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] NET_HEARTBEAT armed={0} part={1} vessel={2} packed={3} rb={4} loadedVessels={5}",
                    armed, part != null, vessel != null,
                    vessel != null && vessel.packed,
                    part != null && part.Rigidbody != null,
                    FlightGlobals.VesselsLoaded == null ? -1 : FlightGlobals.VesselsLoaded.Count));
            }

            if (!HighLogic.LoadedSceneIsFlight || !armed || part == null || vessel == null ||
                vessel.packed || part.Rigidbody == null)
                return;

            // 20 Hz is enough for a 1.5 m capture slab at the permitted closing speed.
            if (universalTime - lastScan < 0.05)
                return;
            lastScan = universalTime;

            PruneBrokenJoints();

            foreach (Vessel candidate in FlightGlobals.VesselsLoaded)
            {
                if (candidate == null || candidate == vessel || candidate.packed ||
                    captures.ContainsKey(candidate.id))
                    continue;

                List<ModuleCatchHook> hooks = candidate.parts
                    .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                    .Where(h => h != null && h.armed && h.part != null && h.part.Rigidbody != null)
                    .ToList();

                if (debugLogging && universalTime - lastDebug < 0.03)
                {
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] NET_CANDIDATE name={0} packed={1} parts={2} hooks={3}",
                        candidate.vesselName, candidate.packed, candidate.parts.Count, hooks.Count));
                }

                if (hooks.Count == 0)
                    continue;

                ModuleCatchHook triggerHook = hooks.FirstOrDefault(h =>
                    h.GetHookWorldPoints().Any(IsPointInsideCaptureSlab));
                if (triggerHook == null)
                    continue;

                Vector3 relativeVelocity = (Vector3)(candidate.srf_velocity - vessel.srf_velocity);
                float normalSpeed = Vector3.Dot(relativeVelocity, part.transform.up);
                Vector3 lateralVelocity = relativeVelocity - normalSpeed * part.transform.up;
                float tilt = candidate.ReferenceTransform == null
                    ? 0f
                    : Vector3.Angle(candidate.ReferenceTransform.up, part.transform.up);

                bool descending = normalSpeed < -0.05f;
                bool withinEnvelope = descending && -normalSpeed <= maxClosingSpeed &&
                                      lateralVelocity.magnitude <= maxLateralSpeed &&
                                      tilt <= maxTilt;

                if (!withinEnvelope)
                {
                    netState = "Rejected: envelope";
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] REJECT vessel={0} vertical={1:F2} lateral={2:F2} tilt={3:F1}",
                        candidate.vesselName, normalSpeed, lateralVelocity.magnitude, tilt));
                    continue;
                }

                Capture(candidate, hooks, normalSpeed, lateralVelocity.magnitude, tilt);
            }
        }

        // KSP does not dispatch PartModule.OnFixedUpdate for every inactive/landed
        // command part. Unity's physics callback is reliable for a stationary net.
        private void FixedUpdate()
        {
            OnFixedUpdate();
        }

        private bool IsPointInsideCaptureSlab(Vector3 worldPoint)
        {
            Vector3 local = part.transform.InverseTransformPoint(worldPoint);
            return Math.Abs(local.x) <= halfWidth &&
                   Math.Abs(local.z) <= halfLength &&
                   Math.Abs(local.y - planeOffset) <= detectionDepth;
        }

        private void Capture(Vessel candidate, List<ModuleCatchHook> hooks,
            float normalSpeed, float lateralSpeed, float tilt)
        {
            List<ConfigurableJoint> joints = new List<ConfigurableJoint>();
            int expectedHookPoints = hooks.Sum(h => Mathf.Max(1, h.virtualHookCount));
            int availableHookPoints = hooks.Sum(h => h.GetHookWorldPoints().Count(IsPointOverNet));
            if (availableHookPoints < expectedHookPoints)
            {
                netState = "Rejected: partial hooks";
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] REJECT_PARTIAL vessel={0} hooks={1}/{2}",
                    candidate.vesselName, availableHookPoints, expectedHookPoints));
                return;
            }

            // Attach every armed hook over the net in one physics frame. Four spaced joints
            // reproduce the stabilising moment of the real four-hook arrangement.
            foreach (ModuleCatchHook hook in hooks)
            {
                foreach (Vector3 hookWorld in hook.GetHookWorldPoints().Where(IsPointOverNet))
                {
                    ConfigurableJoint joint = hook.part.gameObject.AddComponent<ConfigurableJoint>();
                    joint.connectedBody = part.Rigidbody;
                    joint.autoConfigureConnectedAnchor = false;
                    joint.anchor = hook.part.transform.InverseTransformPoint(hookWorld);
                    joint.connectedAnchor = part.transform.InverseTransformPoint(hookWorld);

                    joint.xMotion = ConfigurableJointMotion.Limited;
                    joint.yMotion = ConfigurableJointMotion.Limited;
                    joint.zMotion = ConfigurableJointMotion.Limited;
                    joint.angularXMotion = ConfigurableJointMotion.Limited;
                    joint.angularYMotion = ConfigurableJointMotion.Limited;
                    joint.angularZMotion = ConfigurableJointMotion.Limited;

                    SoftJointLimit linearLimit = joint.linearLimit;
                    linearLimit.limit = jointTravel;
                    joint.linearLimit = linearLimit;

                    SoftJointLimit angularLimit = joint.lowAngularXLimit;
                    angularLimit.limit = -maxTilt;
                    joint.lowAngularXLimit = angularLimit;
                    angularLimit.limit = maxTilt;
                    joint.highAngularXLimit = angularLimit;
                    joint.angularYLimit = angularLimit;
                    joint.angularZLimit = angularLimit;

                    JointDrive drive = new JointDrive
                    {
                        positionSpring = spring,
                        positionDamper = damper,
                        maximumForce = maximumForce
                    };
                    joint.xDrive = drive;
                    joint.yDrive = drive;
                    joint.zDrive = drive;
                    joint.targetPosition = Vector3.zero;
                    joint.breakForce = maximumForce * 1.5f;
                    joint.breakTorque = maximumForce;
                    joint.enableCollision = false;

                    joints.Add(joint);
                }
                hook.SetCaptured(true);
            }

            if (joints.Count == 0)
                return;

            captures[candidate.id] = joints;
            netState = "Captured: " + candidate.vesselName;
            ScreenMessages.PostScreenMessage(
                string.Format("NET CAPTURE: {0} ({1} hooks)", candidate.vesselName, joints.Count),
                8f, ScreenMessageStyle.UPPER_CENTER);
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] CAPTURE vessel={0} hooks={1} vertical={2:F2} lateral={3:F2} tilt={4:F1}",
                candidate.vesselName, joints.Count, normalSpeed, lateralSpeed, tilt));
        }

        private bool IsPointOverNet(Vector3 worldPoint)
        {
            Vector3 local = part.transform.InverseTransformPoint(worldPoint);
            return Math.Abs(local.x) <= halfWidth && Math.Abs(local.z) <= halfLength &&
                   Math.Abs(local.y - planeOffset) <= detectionDepth * 3f;
        }

        private void PruneBrokenJoints()
        {
            foreach (Guid vesselId in captures.Keys.ToList())
            {
                captures[vesselId].RemoveAll(j => j == null);
                if (captures[vesselId].Count == 0)
                    captures.Remove(vesselId);
            }
            if (captures.Count == 0 && netState.StartsWith("Captured", StringComparison.Ordinal))
                netState = "Ready";
        }

        private void CreateCableVisuals()
        {
            if (cableRenderers.Count > 0)
                return;

            Color cableColor = new Color(0.95f, 0.75f, 0.12f, 1f);
            float xInset = halfWidth * 0.32f;
            float zInset = halfLength * 0.32f;
            AddCable(new Vector3(-xInset, planeOffset, -halfLength),
                new Vector3(-xInset, planeOffset, halfLength), cableColor);
            AddCable(new Vector3(xInset, planeOffset, -halfLength),
                new Vector3(xInset, planeOffset, halfLength), cableColor);
            AddCable(new Vector3(-halfWidth, planeOffset, -zInset),
                new Vector3(halfWidth, planeOffset, -zInset), cableColor);
            AddCable(new Vector3(-halfWidth, planeOffset, zInset),
                new Vector3(halfWidth, planeOffset, zInset), cableColor);
        }

        private void AddCable(Vector3 start, Vector3 end, Color color)
        {
            GameObject cable = new GameObject("CZ10B_CatchCable");
            cable.transform.SetParent(part.transform, false);
            LineRenderer renderer = cable.AddComponent<LineRenderer>();
            renderer.useWorldSpace = false;
            renderer.positionCount = 2;
            renderer.SetPosition(0, start);
            renderer.SetPosition(1, end);
            renderer.startWidth = 0.16f;
            renderer.endWidth = 0.16f;
            renderer.startColor = color;
            renderer.endColor = color;
            Shader shader = Shader.Find("KSP/Unlit");
            if (shader == null)
                shader = Shader.Find("Unlit/Color");
            if (shader != null)
                renderer.material = new Material(shader) { color = color };
            cableRenderers.Add(renderer);
        }

        private void DestroyCableVisuals()
        {
            foreach (LineRenderer renderer in cableRenderers)
            {
                if (renderer != null)
                    UnityEngine.Object.Destroy(renderer.gameObject);
            }
            cableRenderers.Clear();
        }
    }
}
