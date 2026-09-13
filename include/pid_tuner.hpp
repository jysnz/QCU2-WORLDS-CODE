#pragma once

// ─── PID Tuner ───────────────────────────────────────────────────────────────
// On-robot tuning mode for the LemLib angular/lateral controllers.
// Enter by HOLDING DPAD-LEFT when driver control starts. See the full
// controls reference at the top of src/pid_tuner.cpp -- short version:
//
//   Y            toggle which controller LEFT/RIGHT/UP/DOWN edit (ANGULAR
//                or LATERAL); doesn't affect R1/R2 below
//   LEFT/RIGHT   select gain (kP -> kI -> kD)
//   L1/L2        move the digit cursor (coarser/finer)
//   UP/DOWN      adjust the selected gain by the current digit's step
//   A/X/B        run a single test (small / big / return) in whichever
//                mode Y currently has selected
//   R1           run the ANGULAR sweep (chained turns + return-to-start),
//                regardless of Y mode
//   R2           run the LATERAL sweep (chained drives + return-to-start),
//                regardless of Y mode
//
// The brain screen shows the gain panel plus a live target-vs-actual graph
// of each run; the controller screen shows gains and the last run's
// overshoot / settle time / final error. Full CSV telemetry streams over
// `pros terminal`.
void pidTunerControl();

// True while the tuner owns the brain screen; the HUD task in main.cpp must
// not draw while this is set.
extern bool pidTunerActive;
