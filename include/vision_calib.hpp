#pragma once

// ─── Vision calibration ─────────────────────────────────────────────────
// HOME -> VISION. A diagnostic/tuning screen for the V5 Vision Sensor on
// port 20: pick a color to track -- YEL / RED / BLUE buttons mapped to the
// sensor's signature slots 1 / 2 / 3 (captured beforehand with the VEX
// Vision Utility on a PC -- the sensor itself has no on-robot "learn"
// command in the documented PROS API; SIG -/+ reaches any of the 7 slots),
// see all three colors' live object counts at once, tweak exposure, and
// run FOLLOW mode, where the robot pivots to keep the largest object of
// the selected color centered in view while the element is moved around
// by hand.
void visionCalibControl();
