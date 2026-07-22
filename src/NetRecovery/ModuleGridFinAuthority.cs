using System;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Gives kOS a stable, continuous deployment coordinate for the four
    /// ModuleAeroSurface grid-fin analogues.  Deployment is rate limited here
    /// rather than toggled through the Brakes action group, while the stock
    /// module retains its ordinary pitch/yaw steering deflection.
    /// </summary>
    public sealed class ModuleGridFinAuthority : PartModule
    {
        [KSPField] public float fullLiftCoefficient = -1f;

        [KSPField(isPersistant = true, guiActive = true,
            guiActiveEditor = true, guiName = "Lift authority",
            guiFormat = "F0", guiUnits = "%")]
        [UI_FloatRange(minValue = 0f, maxValue = 100f, stepIncrement = 1f)]
        public float liftAuthorityPercent = 100f;

        [KSPField] public float maxDeploymentDegrees = 70f;
        [KSPField] public float deploymentRateDegreesPerSecond = 10f;
        [KSPField] public float steeringAuthorityPercent = 35.714f;

        [KSPField(isPersistant = true, guiActive = true,
            guiActiveEditor = true, guiName = "Deployment command",
            guiFormat = "F0", guiUnits = "%")]
        [UI_FloatRange(minValue = 0f, maxValue = 100f, stepIncrement = 1f)]
        public float deploymentCommandPercent;

        [KSPField(isPersistant = true, guiActive = true,
            guiName = "Applied deployment", guiFormat = "F1",
            guiUnits = " deg")]
        public float appliedDeploymentDegrees;

        private ModuleAeroSurface aeroSurface;
        private float appliedAuthority = float.NaN;
        private float loggedDeployment = float.NaN;
        private float lastFixedTime = float.NaN;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            aeroSurface = part == null
                ? null : part.FindModuleImplementing<ModuleAeroSurface>();
            if (aeroSurface != null && fullLiftCoefficient < 0f)
                fullLiftCoefficient = aeroSurface.deflectionLiftCoeff;
            appliedDeploymentDegrees = Mathf.Clamp(appliedDeploymentDegrees,
                0f, maxDeploymentDegrees);
            ApplyState(true);
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();
            ApplyState(false);
        }

        // KSP does not reliably dispatch PartModule.OnFixedUpdate for every
        // custom module state.  Unity still invokes this MonoBehaviour message;
        // ModuleCatchNet uses the same bridge for its flight physics.
        private void FixedUpdate()
        {
            ApplyState(false);
        }

        private void ApplyState(bool force)
        {
            if (aeroSurface == null || fullLiftCoefficient < 0f)
                return;

            float fixedTime = Time.fixedTime;
            if (!force && !float.IsNaN(lastFixedTime) &&
                Math.Abs(fixedTime - lastFixedTime) < 0.0001f)
                return;
            lastFixedTime = fixedTime;

            float authority = Mathf.Clamp(liftAuthorityPercent, 0f, 100f);
            aeroSurface.deflectionLiftCoeff =
                fullLiftCoefficient * authority / 100f;
            aeroSurface.aeroAuthorityLimiter = Mathf.Clamp(
                steeringAuthorityPercent, 0f, 100f);
            aeroSurface.aeroAuthorityLimiterUI = aeroSurface.ctrlSurfaceRange *
                aeroSurface.aeroAuthorityLimiter / 100f;

            float targetDegrees = maxDeploymentDegrees * Mathf.Clamp(
                deploymentCommandPercent, 0f, 100f) / 100f;
            if (force)
                appliedDeploymentDegrees = Mathf.Clamp(
                    appliedDeploymentDegrees, 0f, maxDeploymentDegrees);
            else
                appliedDeploymentDegrees = Mathf.MoveTowards(
                    appliedDeploymentDegrees, targetDegrees,
                    Math.Max(deploymentRateDegreesPerSecond, 0f) *
                    TimeWarp.fixedDeltaTime);
            aeroSurface.aeroDeployAngle = appliedDeploymentDegrees;
            aeroSurface.aeroDeployAngleLimits = new Vector2(
                0f, maxDeploymentDegrees);
            if (HighLogic.LoadedSceneIsFlight)
                aeroSurface.deploy = true;

            appliedAuthority = authority;
            if (!force && !float.IsNaN(loggedDeployment) &&
                Math.Abs(appliedDeploymentDegrees - loggedDeployment) < 5f &&
                Math.Abs(authority - appliedAuthority) < 0.01f)
                return;
            loggedDeployment = appliedDeploymentDegrees;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] GRID_FIN_STATE part={0} liftPercent={1:F1} coefficient={2:F3} deploymentCommand={3:F1} appliedDegrees={4:F1}",
                part == null ? "missing" : part.partInfo.name,
                authority, aeroSurface.deflectionLiftCoeff,
                deploymentCommandPercent, appliedDeploymentDegrees));
        }
    }
}
