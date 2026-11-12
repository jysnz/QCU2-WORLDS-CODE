#pragma once
// Run recorder: a "flight recorder" for diagnosing a run afterwards.
//
// While a run is being recorded, a background task samples the robot every 20 ms (pose, the IR
// pin sensor, both claws, whether a motion is running, which plan step it is on) and the code adds
// named events at the moments that matter (a step beginning, the pin being seen, the claws
// closing). The same task checks the sensors against each other, per plan step: the IMU's heading
// change against the one the drive motors give, the tracking wheels against the drive motors, how
// far the robot tipped, and whether the heading moved while the robot stood still.
//
// When the run ends, everything is sent to the laptop as numbered "LOG|#run.line|..." lines;
// tools/path_planner.py turns them into a report (tools/run_reports/). The recording stays in
// memory until the next run, so the laptop asks again for any line the link dropped
// ("LOG NEED ...", see resend()).
//
//   runlog::begin("plan");  ...  runlog::end("DONE");
//
// Recording costs two small tasks and ~70 KB of RAM. Nothing here ever changes how the robot
// drives; with no laptop connected the lines simply go nowhere.

namespace runlog {

void begin(const char *name);   // start recording (drops the previous recording)
void end(const char *status);   // stop; send the recording to the laptop in the background
void setStep(int idx);          // the plan step the samples belong to from now on (-1 = none)
bool recording();
bool sending();                 // lines of the last recording are still on their way

// The laptop's "LOG NEED <run> <lines>": send those lines of the last recording again.
// <run> 0 = whichever is the last; <lines> e.g. "3-9 12 40-41", or "ALL".
void resend(const char *spec);

// A named moment, stamped with the time and the pose. No '|' or ',' (they are replaced).
void event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

} // namespace runlog
