@LAZYGLOBAL OFF.

// Autonomous upper-stage insertion.  The first burn establishes the 100 km
// apoapsis; a short prograde burn around apoapsis then raises periapsis above
// the atmosphere without interfering with the booster's recovery computer.
LOCAL TARGET_APOAPSIS IS 100000.
LOCAL TARGET_PERIAPSIS IS 90000.
LOCAL SAFE_APOAPSIS IS 120000.

SAS OFF.
RCS ON.
FOR ENGINE IN SHIP:ENGINES {
    ENGINE:ACTIVATE.
}
LOCK STEERING TO PROGRADE.
LOCK THROTTLE TO 1.
WAIT UNTIL SHIP:APOAPSIS >= TARGET_APOAPSIS.
LOCK THROTTLE TO 0.

// The enlarged payload needs a finite burn centred on apoapsis.  Begin early
// and use full thrust; the former 10--20% burn started too late, continued far
// down the descending branch and raised apoapsis much more than periapsis.
WAIT UNTIL ETA:APOAPSIS <= 20 OR SHIP:VERTICALSPEED < 0.
LOCK STEERING TO PROGRADE.
LOCK THROTTLE TO 1.
UNTIL SHIP:PERIAPSIS >= TARGET_PERIAPSIS
      OR (SHIP:PERIAPSIS >= 72000
          AND SHIP:APOAPSIS >= SAFE_APOAPSIS) {
    WAIT 0.02.
}
LOCK THROTTLE TO 0.
PRINT "UPPER STAGE ORBIT ACHIEVED".
PRINT "AP " + ROUND(SHIP:APOAPSIS / 1000,1) + " km".
PRINT "PE " + ROUND(SHIP:PERIAPSIS / 1000,1) + " km".
