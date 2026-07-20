using System;
using UnityEngine;

namespace CZ10BNetRecovery
{
    /// <summary>
    /// Lets kOS remove grid-fin lift after the dedicated atmospheric entry burn.
    /// The visual control surface remains attached; only its aerodynamic lift
    /// coefficient is scheduled.  This prevents a high-angle terminal thrust
    /// command from also creating an unplanned 15--23 m/s^2 lifting force.
    /// </summary>
    public sealed class ModuleGridFinAuthority : PartModule
    {
        [KSPField] public float fullLiftCoefficient = -1f;

        [KSPField(isPersistant = true, guiActive = true,
            guiActiveEditor = true, guiName = "Lift authority",
            guiFormat = "F0", guiUnits = "%")]
        [UI_FloatRange(minValue = 0f, maxValue = 100f, stepIncrement = 1f)]
        public float liftAuthorityPercent = 100f;

        private ModuleControlSurface controlSurface;
        private float appliedAuthority = float.NaN;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            controlSurface = part == null
                ? null : part.FindModuleImplementing<ModuleControlSurface>();
            if (controlSurface != null && fullLiftCoefficient < 0f)
                fullLiftCoefficient = controlSurface.deflectionLiftCoeff;
            ApplyAuthority(true);
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();
            ApplyAuthority(false);
        }

        // KSP does not reliably dispatch PartModule.OnFixedUpdate for every
        // custom module state.  Unity still invokes this MonoBehaviour message;
        // ModuleCatchNet uses the same bridge for its flight physics.
        private void FixedUpdate()
        {
            ApplyAuthority(false);
        }

        private void ApplyAuthority(bool force)
        {
            if (controlSurface == null || fullLiftCoefficient < 0f)
                return;
            float authority = Mathf.Clamp(liftAuthorityPercent, 0f, 100f);
            if (!force && Math.Abs(authority - appliedAuthority) < 0.01f)
                return;
            controlSurface.deflectionLiftCoeff =
                fullLiftCoefficient * authority / 100f;
            appliedAuthority = authority;
            Debug.Log(string.Format(
                "[CZ10BNetRecovery] GRID_FIN_LIFT_AUTHORITY part={0} percent={1:F1} coefficient={2:F3}",
                part == null ? "missing" : part.partInfo.name,
                authority, controlSurface.deflectionLiftCoeff));
        }
    }
}
