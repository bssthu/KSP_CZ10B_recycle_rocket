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
        [KSPField] public float captureSettleDrop = 0.75f;
        [KSPField] public float openCableInset = 8f;
        [KSPField] public float closedCableInset = 1.65f;
        [KSPField] public float cableClosureSpeed = 2.5f;
        [KSPField] public float cableTrackingSpeed = 5f;
        [KSPField] public float closureHeight = 16f;
        [KSPField] public float closureHalfWidth = 8f;
        [KSPField] public bool debugLogging = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true,
            guiName = "Capture system")]
        [UI_Toggle(enabledText = "ARMED", disabledText = "SAFE")]
        public bool armed = true;

        [KSPField(guiActive = true, guiName = "Net state")]
        public string netState = "Ready";

        [KSPField(guiActive = true, guiName = "Cable tracking error",
            guiFormat = "F2", guiUnits = " m")]
        public float cableTrackingError;

        [KSPField(guiActive = true, guiName = "Cable deflection",
            guiFormat = "F2", guiUnits = " m")]
        public float cableDeflection;

        private readonly Dictionary<Guid, List<ConfigurableJoint>> captures =
            new Dictionary<Guid, List<ConfigurableJoint>>();
        private readonly List<LineRenderer> cableRenderers = new List<LineRenderer>();
        private double lastScan;
        private double lastDebug;
        private float currentCableInset;
        private Vector2 currentCableCentre;

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
            currentCableInset = openCableInset;
            currentCableCentre = Vector2.zero;
            cableTrackingError = 0f;
            cableDeflection = 0f;
            netState = armed ? "Open" : "Safe";
            UpdateCableVisuals();
            ScreenMessages.PostScreenMessage("CZ-10B net: captured stage released", 4f,
                ScreenMessageStyle.UPPER_CENTER);
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            currentCableInset = openCableInset;
            currentCableCentre = Vector2.zero;
            if (HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor)
            {
                CreateCableVisuals();
                UpdateCableVisuals();
            }
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
            UpdateCableClosure();

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

                // The four winch-driven lines must finish moving under the hook
                // points before the physical capture slab is armed.
                if (Math.Abs(currentCableInset - closedCableInset) > 0.15f)
                    continue;

                // Do not capture through a visually displaced cable cradle.
                // The winches continuously follow the hook centroid and must be
                // settled under it before the physical capture slab is armed.
                if (!HooksInsideTrackedCradle(hooks))
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
            float height = local.y - planeOffset;
            return Math.Abs(local.x) <= halfWidth &&
                   Math.Abs(local.z) <= halfLength &&
                   height <= 0.15f && height >= -detectionDepth;
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
                    // Establish the loaded rope equilibrium below the contact
                    // plane. This models the short elastic drop visible after
                    // the real frame takes the stage weight, rather than
                    // pinning it at the first collision sample.
                    Vector3 settledWorld = hookWorld - part.transform.up * captureSettleDrop;
                    joint.connectedAnchor = part.transform.InverseTransformPoint(settledWorld);

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
                netState = armed ? "Open" : "Safe";
        }

        private void UpdateCableClosure()
        {
            if (captures.Count > 0)
            {
                currentCableInset = closedCableInset;
                Vector2 capturedCentre;
                if (TryGetCableTarget(true, out capturedCentre))
                    MoveCableCentre(capturedCentre);
                UpdateCableVisuals();
                return;
            }

            Vector2 requestedCentre = Vector2.zero;
            bool requested = armed && TryGetCableTarget(false, out requestedCentre);
            float targetInset = requested ? closedCableInset : openCableInset;
            currentCableInset = Mathf.MoveTowards(currentCableInset, targetInset,
                cableClosureSpeed * 0.05f);
            MoveCableCentre(requested ? requestedCentre : Vector2.zero);

            if (!armed)
                netState = "Safe";
            else if (!requested)
                netState = "Open";
            else if (Math.Abs(currentCableInset - closedCableInset) <= 0.05f &&
                     Vector2.Distance(currentCableCentre, requestedCentre) <= 0.5f)
                netState = "Closed";
            else if (Math.Abs(currentCableInset - closedCableInset) <= 0.05f)
                netState = "Tracking";
            else
                netState = "Closing";
            UpdateCableVisuals();
        }

        private bool TryGetCableTarget(bool capturedOnly, out Vector2 target)
        {
            target = Vector2.zero;
            float bestDistance = float.MaxValue;
            bool found = false;
            foreach (Vessel candidate in FlightGlobals.VesselsLoaded)
            {
                if (candidate == null || candidate == vessel || candidate.packed ||
                    candidate.parts == null || (capturedOnly && !captures.ContainsKey(candidate.id)))
                    continue;

                List<Vector3> points = candidate.parts
                    .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                    .Where(h => h != null && h.armed)
                    .SelectMany(h => h.GetHookWorldPoints())
                    .Select(p => part.transform.InverseTransformPoint(p))
                    .ToList();
                if (points.Count == 0)
                    continue;

                Vector3 centre = points.Aggregate(Vector3.zero, (sum, p) => sum + p) /
                                 points.Count;
                float height = centre.y - planeOffset;
                if (Math.Abs(centre.x) <= closureHalfWidth &&
                    Math.Abs(centre.z) <= closureHalfWidth &&
                    height >= -jointTravel - detectionDepth && height <= closureHeight)
                {
                    float distance = Mathf.Abs(height);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        target = new Vector2(centre.x, centre.z);
                        found = true;
                    }
                }
            }
            return found;
        }

        private void MoveCableCentre(Vector2 requestedCentre)
        {
            // While wide open the lines can move only a little without leaving
            // the frame. Their available tracking range grows continuously as
            // they close around the stage.
            float limitX = Mathf.Max(0f, halfWidth - currentCableInset);
            float limitZ = Mathf.Max(0f, halfLength - currentCableInset);
            Vector2 bounded = new Vector2(
                Mathf.Clamp(requestedCentre.x, -limitX, limitX),
                Mathf.Clamp(requestedCentre.y, -limitZ, limitZ));
            currentCableCentre = Vector2.MoveTowards(currentCableCentre, bounded,
                cableTrackingSpeed * 0.05f);
            cableTrackingError = Vector2.Distance(currentCableCentre, bounded);
        }

        private bool HooksInsideTrackedCradle(List<ModuleCatchHook> hooks)
        {
            List<Vector3> points = hooks.SelectMany(h => h.GetHookWorldPoints())
                .Select(p => part.transform.InverseTransformPoint(p)).ToList();
            if (points.Count == 0)
                return false;
            Vector3 centre = points.Aggregate(Vector3.zero, (sum, p) => sum + p) /
                             points.Count;
            return Vector2.Distance(new Vector2(centre.x, centre.z), currentCableCentre) <= 0.75f;
        }

        private void CreateCableVisuals()
        {
            if (cableRenderers.Count > 0)
                return;

            Color cableColor = new Color(0.95f, 0.75f, 0.12f, 1f);
            for (int index = 0; index < 4; ++index)
                AddCable(cableColor);
        }

        private void AddCable(Color color)
        {
            GameObject cable = new GameObject("CZ10B_CatchCable");
            cable.transform.SetParent(part.transform, false);
            LineRenderer renderer = cable.AddComponent<LineRenderer>();
            renderer.useWorldSpace = false;
            renderer.positionCount = 3;
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

        private void UpdateCableVisuals()
        {
            if (cableRenderers.Count != 4)
                return;

            float sagY = planeOffset;
            if (captures.Count > 0)
            {
                List<Vector3> capturedPoints = FlightGlobals.VesselsLoaded
                    .Where(v => v != null && captures.ContainsKey(v.id) && v.parts != null)
                    .SelectMany(v => v.parts)
                    .Select(p => p.FindModuleImplementing<ModuleCatchHook>())
                    .Where(h => h != null && h.hookState == "Captured")
                    .SelectMany(h => h.GetHookWorldPoints())
                    .Select(p => part.transform.InverseTransformPoint(p))
                    .ToList();
                if (capturedPoints.Count > 0)
                {
                    float averageY = capturedPoints.Average(p => p.y);
                    sagY = Mathf.Clamp(averageY, planeOffset - jointTravel, planeOffset);
                }
            }
            cableDeflection = Mathf.Max(0f, planeOffset - sagY);

            float inset = Mathf.Clamp(currentCableInset, 0f,
                Mathf.Min(halfWidth, halfLength));
            float centreX = currentCableCentre.x;
            float centreZ = currentCableCentre.y;
            SetCable(cableRenderers[0],
                new Vector3(centreX - inset, planeOffset, -halfLength),
                new Vector3(centreX - inset, sagY, centreZ),
                new Vector3(centreX - inset, planeOffset, halfLength));
            SetCable(cableRenderers[1],
                new Vector3(centreX + inset, planeOffset, -halfLength),
                new Vector3(centreX + inset, sagY, centreZ),
                new Vector3(centreX + inset, planeOffset, halfLength));
            SetCable(cableRenderers[2],
                new Vector3(-halfWidth, planeOffset, centreZ - inset),
                new Vector3(centreX, sagY, centreZ - inset),
                new Vector3(halfWidth, planeOffset, centreZ - inset));
            SetCable(cableRenderers[3],
                new Vector3(-halfWidth, planeOffset, centreZ + inset),
                new Vector3(centreX, sagY, centreZ + inset),
                new Vector3(halfWidth, planeOffset, centreZ + inset));
        }

        private static void SetCable(LineRenderer renderer, Vector3 start,
            Vector3 middle, Vector3 end)
        {
            if (renderer == null)
                return;
            renderer.SetPosition(0, start);
            renderer.SetPosition(1, middle);
            renderer.SetPosition(2, end);
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
