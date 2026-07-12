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
        [KSPField] public float jointSlack = 0.75f;
        [KSPField] public float spring = 180000f;
        [KSPField] public float damper = 45000f;
        [KSPField] public float maximumForce = 2000000f;
        [KSPField] public float captureSettleDrop = 0.75f;
        [KSPField] public float captureSettleSpeed = 2.5f;
        [KSPField] public float captureLinearDamping = 3f;
        [KSPField] public float captureAngularDamping = 5f;
        [KSPField] public float openCableInset = 8f;
        [KSPField] public float closedCableInset = 1.65f;
        [KSPField] public float cableClosureSpeed = 2.5f;
        [KSPField] public float cableTrackingSpeed = 5f;
        [KSPField] public float cableAnchorHalfWidth = -1f;
        [KSPField] public float cableAnchorHalfLength = -1f;
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
        private readonly Dictionary<Guid, Vessel> capturedStages =
            new Dictionary<Guid, Vessel>();
        private sealed class CaptureTether
        {
            public ConfigurableJoint joint;
            public Rigidbody winchBody;
            public Guid vesselId;
            public Vector3 currentWorld;
            public Vector3 settledWorld;
            public Vector3 commandedVelocity;
        }

        private readonly Dictionary<ConfigurableJoint, CaptureTether> captureTethers =
            new Dictionary<ConfigurableJoint, CaptureTether>();
        private readonly List<LineRenderer> cableRenderers = new List<LineRenderer>();
        private double lastScan;
        private double lastTetherUpdate;
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
            capturedStages.Clear();
            DestroyCaptureTethers();
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
            DestroyCaptureTethers();
            DestroyCableVisuals();
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();

            double universalTime = Planetarium.GetUniversalTime();
            float tetherDeltaTime = UpdateCaptureSettling(universalTime);
            DampCapturedStages(tetherDeltaTime);
            if (debugLogging && universalTime - lastDebug >= 0.5)
            {
                lastDebug = universalTime;
                Debug.Log(string.Format(
                    "[CZ10BNetRecovery] NET_HEARTBEAT armed={0} part={1} vessel={2} packed={3} rb={4} loadedVessels={5} state={6} inset={7:F2} centre=({8:F2},{9:F2})",
                    armed, part != null, vessel != null,
                    vessel != null && vessel.packed,
                    part != null && part.Rigidbody != null,
                    FlightGlobals.VesselsLoaded == null ? -1 : FlightGlobals.VesselsLoaded.Count,
                    netState, currentCableInset, currentCableCentre.x,
                    currentCableCentre.y));
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

                // The recovery ship is held fixed in Kerbin's rotating surface
                // frame. candidate.srf_velocity is therefore already the stage
                // velocity relative to the platform. Subtracting the inactive
                // ship's velocity adds the planet's ~175 m/s rotation a second
                // time (the same rule is used by the kOS terminal controller).
                Vector3 relativeVelocity = (Vector3)candidate.srf_velocity;
                Vector3 netUp = NetUp();
                float normalSpeed = Vector3.Dot(relativeVelocity, netUp);
                Vector3 lateralVelocity = relativeVelocity - normalSpeed * netUp;
                float tilt = candidate.ReferenceTransform == null
                    ? 0f
                    : Vector3.Angle(candidate.ReferenceTransform.up, netUp);

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
            Vector3 local = WorldToNet(worldPoint);
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
                    // Do not feed the cable reaction into the dynamically
                    // positioned recovery ship. A physical joint attached
                    // directly to the ship fought the DP station keeper and
                    // injected enough energy to roll the platform and spin the
                    // captured stage. Each line therefore terminates at a
                    // kinematic winch point which follows the ship's local net
                    // frame but absorbs the equal-and-opposite joint force.
                    GameObject winchObject = new GameObject(
                        "CZ10B kinematic winch anchor");
                    winchObject.transform.position = hookWorld;
                    Rigidbody winchBody = winchObject.AddComponent<Rigidbody>();
                    winchBody.isKinematic = true;
                    winchBody.useGravity = false;
                    winchBody.detectCollisions = false;
                    joint.connectedBody = winchBody;
                    joint.autoConfigureConnectedAnchor = false;
                    joint.anchor = hook.part.transform.InverseTransformPoint(hookWorld);
                    // Establish one loaded equilibrium relative to the cable
                    // plane, independent of where inside the compliant sweep
                    // volume contact happened. A later edge capture therefore
                    // cannot add its detection depth to the intended rope sag.
                    Vector3 settledLocal = WorldToNet(hookWorld);
                    settledLocal.y = planeOffset - captureSettleDrop;
                    Vector3 settledWorld = NetToWorld(settledLocal);
                    // Start with zero geometric error. The winch then pays the
                    // line out gradually, avoiding a destructive 17 m impulse
                    // on the full-size booster in the first physics frame.
                    joint.connectedAnchor = Vector3.zero;

                    joint.xMotion = ConfigurableJointMotion.Limited;
                    joint.yMotion = ConfigurableJointMotion.Limited;
                    joint.zMotion = ConfigurableJointMotion.Limited;
                    // Four separated linear hook points already provide the
                    // stabilising moment. Repeating angular constraints on all
                    // four joints over-constrains a tilted full-size stage.
                    joint.angularXMotion = ConfigurableJointMotion.Free;
                    joint.angularYMotion = ConfigurableJointMotion.Free;
                    joint.angularZMotion = ConfigurableJointMotion.Free;

                    SoftJointLimit linearLimit = joint.linearLimit;
                    // jointTravel is the total cable sweep/sag envelope, not
                    // extra rope slack. Keep each hook close to the moving
                    // winch anchor so it cannot free-fall another 18.5 m and
                    // snap into the limit at destructive speed.
                    linearLimit.limit = jointSlack;
                    joint.linearLimit = linearLimit;

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
                    captureTethers[joint] = new CaptureTether
                    {
                        joint = joint,
                        winchBody = winchBody,
                        vesselId = candidate.id,
                        currentWorld = hookWorld,
                        settledWorld = settledWorld
                    };
                }
                hook.SetCaptured(true);
            }

            if (joints.Count == 0)
                return;

            captures[candidate.id] = joints;
            capturedStages[candidate.id] = candidate;
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
            Vector3 local = WorldToNet(worldPoint);
            return Math.Abs(local.x) <= halfWidth && Math.Abs(local.z) <= halfLength &&
                   Math.Abs(local.y - planeOffset) <= detectionDepth * 3f;
        }

        private void PruneBrokenJoints()
        {
            foreach (Guid vesselId in captures.Keys.ToList())
            {
                captures[vesselId].RemoveAll(j => j == null);
                if (captures[vesselId].Count == 0)
                {
                    captures.Remove(vesselId);
                    capturedStages.Remove(vesselId);
                }
            }
            if (captures.Count == 0 && netState.StartsWith("Captured", StringComparison.Ordinal))
                netState = armed ? "Open" : "Safe";
            foreach (KeyValuePair<ConfigurableJoint, CaptureTether> entry in
                     captureTethers.ToList())
            {
                if (entry.Key != null)
                    continue;
                if (entry.Value.winchBody != null)
                    UnityEngine.Object.Destroy(entry.Value.winchBody.gameObject);
                captureTethers.Remove(entry.Key);
            }
        }

        private float UpdateCaptureSettling(double universalTime)
        {
            float deltaTime = lastTetherUpdate <= 0d ? 0f :
                Mathf.Clamp((float)(universalTime - lastTetherUpdate), 0f, 0.05f);
            lastTetherUpdate = universalTime;
            if (deltaTime <= 0f)
                return 0f;
            foreach (KeyValuePair<ConfigurableJoint, CaptureTether> entry in
                     captureTethers.ToList())
            {
                ConfigurableJoint joint = entry.Key;
                CaptureTether tether = entry.Value;
                if (joint == null || tether.winchBody == null)
                {
                    if (tether.winchBody != null)
                        UnityEngine.Object.Destroy(tether.winchBody.gameObject);
                    captureTethers.Remove(joint);
                    continue;
                }
                // Freeze the payout path in the capture-time world frame.
                // Recomputing it from the DP-positioned platform every frame
                // converted its sub-metre station corrections into periodic
                // 100+ m/s kinematic-anchor impulses.
                Vector3 commandedPosition = Vector3.MoveTowards(
                    tether.currentWorld, tether.settledWorld,
                    captureSettleSpeed * deltaTime);
                tether.commandedVelocity =
                    (commandedPosition - tether.currentWorld) / deltaTime;
                tether.currentWorld = commandedPosition;
                tether.winchBody.MovePosition(commandedPosition);
            }
            return deltaTime;
        }

        private void DampCapturedStages(float deltaTime)
        {
            if (deltaTime <= 0f)
                return;
            foreach (KeyValuePair<Guid, Vessel> entry in capturedStages.ToList())
            {
                Vessel stage = entry.Value;
                if (stage == null || stage.state == Vessel.State.DEAD ||
                    stage.packed || stage.rootPart == null ||
                    stage.rootPart.Rigidbody == null)
                {
                    capturedStages.Remove(entry.Key);
                    continue;
                }

                List<CaptureTether> stageTethers = captureTethers.Values
                    .Where(t => t.vesselId == entry.Key && t.winchBody != null)
                    .ToList();
                if (stageTethers.Count == 0)
                    continue;
                Vector3 winchVelocity = stageTethers.Aggregate(Vector3.zero,
                    (sum, tether) => sum + tether.commandedVelocity) /
                    stageTethers.Count;
                Vector3 stageVelocity = stage.rootPart.Rigidbody.velocity;
                float linearBlend = 1f - Mathf.Exp(-captureLinearDamping * deltaTime);
                Vector3 velocityCorrection =
                    (winchVelocity - stageVelocity) * linearBlend;
                float angularScale = Mathf.Exp(-captureAngularDamping * deltaTime);
                foreach (Part stagePart in stage.parts)
                {
                    if (stagePart == null || stagePart.Rigidbody == null)
                        continue;
                    stagePart.Rigidbody.velocity += velocityCorrection;
                    stagePart.Rigidbody.angularVelocity *= angularScale;
                }
            }
        }

        private void DestroyCaptureTethers()
        {
            foreach (CaptureTether tether in captureTethers.Values)
            {
                if (tether.winchBody != null)
                    UnityEngine.Object.Destroy(tether.winchBody.gameObject);
            }
            captureTethers.Clear();
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
                    .Select(WorldToNet)
                    .ToList();
                if (points.Count == 0)
                    continue;

                Vector3 centre = points.Aggregate(Vector3.zero, (sum, p) => sum + p) /
                                 points.Count;
                float height = centre.y - planeOffset;
                if (debugLogging && Planetarium.GetUniversalTime() - lastDebug < 0.03)
                {
                    Debug.Log(string.Format(
                        "[CZ10BNetRecovery] NET_TARGET name={0} net=({1:F2},{2:F2},{3:F2}) height={4:F2} bounds=({5:F1},{6:F1})",
                        candidate.vesselName, centre.x, centre.y, centre.z,
                        height, closureHalfWidth, closureHeight));
                }
                // Once the lines are closed, retain the target through a modest
                // transient sway instead of reopening the cradle at the worst
                // possible moment. Initial acquisition keeps the tighter bounds.
                float activeHalfWidth = closureHalfWidth;
                if (currentCableInset <= closedCableInset + 0.15f)
                    activeHalfWidth += 5f;
                if (Math.Abs(centre.x) <= activeHalfWidth &&
                    Math.Abs(centre.z) <= activeHalfWidth &&
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
                .Select(WorldToNet).ToList();
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
            // Express the cable in an explicit local-surface frame. Part-local
            // Y is not a reliable deck normal after sea deployment or buoyancy.
            renderer.useWorldSpace = true;
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
                    .Select(WorldToNet)
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
            float anchorWidth = cableAnchorHalfWidth > 0f
                ? cableAnchorHalfWidth : halfWidth;
            float anchorLength = cableAnchorHalfLength > 0f
                ? cableAnchorHalfLength : halfLength;
            SetCable(cableRenderers[0],
                new Vector3(centreX - inset, planeOffset, -anchorLength),
                new Vector3(centreX - inset, sagY, centreZ),
                new Vector3(centreX - inset, planeOffset, anchorLength));
            SetCable(cableRenderers[1],
                new Vector3(centreX + inset, planeOffset, -anchorLength),
                new Vector3(centreX + inset, sagY, centreZ),
                new Vector3(centreX + inset, planeOffset, anchorLength));
            SetCable(cableRenderers[2],
                new Vector3(-anchorWidth, planeOffset, centreZ - inset),
                new Vector3(centreX, sagY, centreZ - inset),
                new Vector3(anchorWidth, planeOffset, centreZ - inset));
            SetCable(cableRenderers[3],
                new Vector3(-anchorWidth, planeOffset, centreZ + inset),
                new Vector3(centreX, sagY, centreZ + inset),
                new Vector3(anchorWidth, planeOffset, centreZ + inset));
        }

        private void SetCable(LineRenderer renderer, Vector3 start,
            Vector3 middle, Vector3 end)
        {
            if (renderer == null)
                return;
            renderer.SetPosition(0, NetToWorld(start));
            renderer.SetPosition(1, NetToWorld(middle));
            renderer.SetPosition(2, NetToWorld(end));
        }

        private Vector3 NetUp()
        {
            if (part == null || vessel == null || vessel.mainBody == null)
                return part == null ? Vector3.up : part.transform.up;
            Vector3 up = (part.transform.position -
                (Vector3)vessel.mainBody.position).normalized;
            return up.sqrMagnitude > 0.5f ? up : part.transform.up;
        }

        private void NetAxes(out Vector3 right, out Vector3 up,
            out Vector3 forward)
        {
            up = NetUp();
            right = Vector3.ProjectOnPlane(part.transform.right, up).normalized;
            if (right.sqrMagnitude < 0.5f)
            {
                forward = Vector3.ProjectOnPlane(part.transform.forward, up).normalized;
                right = Vector3.Cross(up, forward).normalized;
            }
            forward = Vector3.Cross(right, up).normalized;
        }

        private Vector3 WorldToNet(Vector3 worldPoint)
        {
            Vector3 right;
            Vector3 up;
            Vector3 forward;
            NetAxes(out right, out up, out forward);
            Vector3 delta = worldPoint - part.transform.position;
            return new Vector3(Vector3.Dot(delta, right),
                Vector3.Dot(delta, up), Vector3.Dot(delta, forward));
        }

        private Vector3 NetToWorld(Vector3 netPoint)
        {
            Vector3 right;
            Vector3 up;
            Vector3 forward;
            NetAxes(out right, out up, out forward);
            return part.transform.position + right * netPoint.x
                + up * netPoint.y + forward * netPoint.z;
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
