#pragma once

// ─── Automatic PID Tuner ─────────────────────────────────────────────────────
// Finds kP / kD / kI for one LemLib controller (ANGULAR or LATERAL) on its
// own. Enter by tapping "AUTO PID TUNE" on the driver menu; tap BACK
// (top-right) to leave. Pick the controller (ANGULAR / LATERAL buttons, or
// Y), tap START (or A), and clear the robot's surroundings: it will drive
// the manual tuner's 4-leg sweep (45 / 90 / 180 deg turns then back to the
// start, or 12 / 24 / 48 in then back) twenty-odd times, each run ending
// where it began. STOP (or X, or the laptop's STOP / Esc) aborts at any
// point.
//
// The recipe it follows (see src/auto_tuner.cpp for the details):
//   1. kP  zero everything, then raise kP run by run until any leg of the
//          sweep oscillates around its target (crosses it at least twice
//          with a clear overshoot). That kP is recorded and kept.
//   2. kD  raise kD run by run until the response is smooth: no bounce
//          past the target. Stops early (and keeps the best kD so far) if
//          more kD stops helping or starts chattering.
//   3. kI  only if an overshoot or a steady-state error is left: raise kI
//          until it fades, again keeping the best value if it gets worse.
//   4.     one more sweep with the result, as verification.
// The result is written into the same in-RAM gain sets the manual PID
// tuner edits (and applied to the chassis), so PID TUNING shows them
// afterwards; write them into main.cpp from the laptop window.
//
// tools/remote_touch.py opens tools/auto_pid_tuner.py when the brain enters
// this screen: START / STOP from there, the live graph of every run, a run
// history table, and "Write to main.cpp". Wire format, robot -> laptop:
//   RUI|AUTO_TUNER|...|AUTO:<mode>,<phase>,<run>,<aP>,<aI>,<aD>,<lP>,<lI>,<lD>,
//                              <Pu>,<bestD>,<bestI>,<running>|MSG:<status text>
//   AT|RUN|run|angular|phase|P|I|D|label|nLegs|label,level,timeoutMs;...
//                                                  a run (sweep) starts
//   AT|LEG|i|label|target|start|timeoutMs           one leg starts
//   AT|S|ms|target|actual|error                     one sample
//   AT|END|i|overshoot|settleMs|final|durMs|crossings|reversals|verdict
//                                                  that leg's numbers
//   AT|VERDICT|run|verdict|overshoot|crossings|settleMs|final|reversals|durMs
//                                                  the whole run's
//   AT|DONE|phase|P|I|D                             tune ended
// laptop -> robot: PID AUTOSTART <mode>, PID AUTOMODE <mode>, PID STOP.
void autoTunerControl();

// True while the auto tuner owns the brain screen; the HUD task in main.cpp
// must not draw while this is set.
extern bool autoTunerActive;
