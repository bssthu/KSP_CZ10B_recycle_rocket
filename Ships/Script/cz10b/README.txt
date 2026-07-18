Copy this directory to <KSP>/Ships/Script/cz10b and cz10b-boot.ks to
<KSP>/Ships/Script/boot. In the VAB, open the booster probe core's kOS
settings, select cz10b-boot.ks, and ensure that probe core is the craft root.

The complete sea mission deploys a vessel named exactly "Recovery Ship" and
expects it to carry one CZ-10B Active Cable Capture Net part. AG10 aborts the
controller. Telemetry appends to Ships/Script/cz10b/telemetry.csv.

The current controller is NOT accepted: the latest observed run splashed the
booster and showed low-altitude PWM plus a nozzle/ground-velocity angle above
30 degrees. A PASS string alone is not success. Use these repository documents
as the authoritative requirements and physics design:

  docs/mandatory-mission-constraints.md
  docs/ksp-physics-and-hybrid-model.md
  docs/automatic-flight-control-scheme.md

Do not claim full success until one uninterrupted KSP mission satisfies every
mandatory constraint, including the 2 km gate, no PWM, no water contact, and
60 seconds of stable four-cable capture.
