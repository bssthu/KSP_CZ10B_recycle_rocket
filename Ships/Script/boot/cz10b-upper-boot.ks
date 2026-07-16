@LAZYGLOBAL OFF.
WAIT UNTIL SHIP:PARTSNAMED("CZ10B-DemoBooster"):LENGTH = 0.
// Let the decoupler impulse open a physical gap before commanding thrust.
WAIT 0.8.
RUNPATH("0:/cz10b/upper.ks").
