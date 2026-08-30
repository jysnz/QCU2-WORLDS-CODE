#pragma once

// ─── PID Tuner ───────────────────────────────────────────────────────────────
// On-robot tuning mode for the LemLib angular/lateral controllers.
// Enter by HOLDING DPAD-LEFT when driver control starts.
//
// Controls inside the tuner:
//   Y            toggle ANGULAR / LATERAL controller
//   LEFT/RIGHT   select gain (kP -> kI -> kD)
//   UP/DOWN      adjust selected gain (hold R2 for fine step, x0.1)
//   A            run small test  (90 deg turn / 24 in drive)
//   X            run big test    (180 deg turn / 48 in drive)
//   B            run return test (back to heading 0 / drive back)
//
// The brain screen shows the gain panel plus a live error-vs-time graph of
// each run; the controller screen shows gains and the last run's overshoot /
// settle time / final error. Full CSV telemetry streams over `pros terminal`.
void pidTunerControl();

// True while the tuner owns the brain screen; the HUD task in main.cpp must
// not draw while this is set.
extern bool pidTunerActive;
