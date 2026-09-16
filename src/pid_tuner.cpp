// ─────────────────────────────────────────────────────────────────────────────
// PID TUNER — CONTROLS REFERENCE
// ─────────────────────────────────────────────────────────────────────────────
// ENTERING / EXITING THE TUNER
//   Tap "PID TUNING" on the driver menu shown at the start of driver
//   control (see driver_menu.hpp/.cpp). Tap BACK (top-right) to exit back
//   to that menu.
//
// SELECTING WHAT TO TUNE
//   Y            toggle ANGULAR (turns, cyan header) / LATERAL (drive,
//                orange header)
//   DPAD-LEFT    select previous gain (kP <- kI <- kD)
//   DPAD-RIGHT   select next gain     (kP -> kI -> kD)
//
// ADJUSTING THE SELECTED GAIN
//   L1           move the digit cursor left  (coarser: x1 -> x10 -> x100)
//   L2           move the digit cursor right (finer:   x1 -> x0.1 -> x0.01
//                -> x0.001) -- lets you dial in kI's thousandths directly
//   DPAD-UP      increase the selected gain by the current digit's step
//   DPAD-DOWN    decrease the selected gain by the current digit's step
//   The current step size is shown next to the gain panel (e.g. "step
//   0.010"), and on controller line 1 (e.g. "+/-0.010").
//
// RUNNING TESTS (make sure the robot has clear space!)
//   A            small test:  90 deg turn   / 24 in drive forward
//   X            big test:    180 deg turn  / 48 in drive forward
//   B            return test: turn -90 deg   / 24 in drive backward
//   Every run starts by zeroing the pose (chassis.setPose(0, 0, 0)): x, y
//   and heading all read 0 before the first motion, so each test is the
//   same motion from the same starting numbers no matter what ran before.
//   R1           ANGULAR sweep test (runs regardless of Y mode): chains
//                three turns -- 45, then 90, then 180 deg (each size
//                relative to wherever the last one stopped, not an
//                absolute checkpoint) -- then turns straight back to the
//                EXACT starting heading, into one continuous motion with
//                no stop or pose reset in between.
//   R2           LATERAL sweep test (runs regardless of Y mode): chains
//                12, then 24, then 48 in of driving, then a return-to-
//                start drive. The first 3 legs exit early (once within
//                kLateralSettleBand of target) instead of fully stopping,
//                so the chassis carries speed straight into the next leg
//                -- a real chained drive, not just back-to-back stops.
//   Both sweeps let you see how gains hold up back-to-back across sizes.
//   All 4 legs of whichever sweep you ran are plotted together on one
//   shared graph, divided by gray vertical lines and labeled by leg size.
//
// READING THE RESULTS
//   Brain graph: classic PID step-response view. Pink = target/setpoint
//   (a flat line, or a step schedule for the sweep). Green = actual value
//   chasing it. Grid lines are labeled in degrees/inches. Footer and
//   controller line 3 show OVERSHOOT / SETTLE ms / FINAL error (for the
//   sweep, these update after each leg finishes).
//   Full CSV telemetry streams over USB: run `pros terminal`.
//
// TUNING RECIPE
//   1. Raise kP until the green trace slightly overshoots the pink target
//      line (a small bounce past it before settling back).
//   2. Add kD until that bounce disappears and settle time drops.
//   3. Only add kI if the green trace flattens just short of the pink line
//      instead of reaching it.
//   4. Verify with X (big test) and B (-90 test), then copy the final
//      values into lateral_controller / angular_controller in main.cpp.
//
// FROM A LAPTOP (tools/pid_tuner.py, opened by tools/remote_touch.py)
//   Everything above can also be done from a laptop window over the same
//   USB cable `pros terminal` uses: type P / I / D values straight in, run
//   the tests and sweeps, stop a run, and watch the same target-vs-actual
//   graph, bigger. The bridge is the driver menu's stdin listener (see the
//   "Remote touch bridge" section in driver_menu.cpp):
//     laptop -> robot   PID SET/MODE/SEL/DIGIT/TEST/SWEEP/CAL/STOP, KEY, TOUCH
//     robot -> laptop   RUI|PID_TUNER|...   gains / selection / last result
//                                           (a few times a second)
//                       PID|RUN / LEG / END / DONE   one run's schedule,
//                                           each leg's start and result
//                       CSV,ms,target,actual,error   the live trace
//   Remote controller keys (KEY ...) act like the physical ones for
//   editing (LEFT/RIGHT/UP/DOWN/L1/L2/Y) and the sweeps (R1/R2). The
//   mirror's "Run (A)" / "Reset pose (B)" buttons are ignored here rather
//   than setting the robot off -- the tuner window has its own test
//   buttons (PID TEST) -- and its "Cancel (X)" stops the run.
// ─────────────────────────────────────────────────────────────────────────────

#include "pid_tuner.hpp"
#include "driver_menu.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

bool pidTunerActive = false;

// ─── Gain state ──────────────────────────────────────────────────────────────
// gains = {kP, kI, kD}. Starting values are copied from lateral_controller /
// angular_controller in main.cpp the first time the tuner opens, so the
// tuner always starts from whatever the robot actually drives with. Edits
// live only in RAM -- once a set feels good, copy it back into main.cpp.
struct GainSet {
  float gains[3];
  float windup;
};

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

static GainSet angularGains{{0.0f, 0.0f, 0.0f}, 0.0f};
static GainSet lateralGains{{0.0f, 0.0f, 0.0f}, 0.0f};

static void loadGainsFromChassisConfig() {
  static bool loaded = false;
  if (loaded)
    return;
  loaded = true;
  angularGains = {{angular_controller.kP, angular_controller.kI, angular_controller.kD},
                  angular_controller.windupRange};
  lateralGains = {{lateral_controller.kP, lateral_controller.kI, lateral_controller.kD},
                  lateral_controller.windupRange};
}

static bool tuningAngular = true;
static int selectedGain = 0; // index into GainSet::gains: 0=kP, 1=kI, 2=kD

// Which decimal place UP/DOWN edits, as a power of ten (2 -> hundreds,
// 0 -> ones, -3 -> thousandths). L1/L2 move this cursor left/right so any
// digit of any gain -- including kI's thousandths -- can be dialed in
// directly instead of only ever nudging by one fixed step.
static int digitExp = -2;
static const int kDigitExpMax = 2;
static const int kDigitExpMin = -3;

static float digitStep() { return std::pow(10.0f, (float)digitExp); }

static GainSet &activeGains() {
  return tuningAngular ? angularGains : lateralGains;
}

// LemLib exposes chassis.angularPID / lateralPID but their gains are const,
// so the only way to swap gains at runtime is to reconstruct the PID in
// place. lemlib::PID holds no resources, so this is safe.
//
// IMPORTANT: this takes `angular` explicitly -- which PID to rebuild is
// which motion is about to run, NOT which mode the Y toggle currently has
// selected for editing (tuningAngular). Those two can disagree: R1/R2 run
// their sweep regardless of Y mode, and even A/X/B only happen to line up
// with tuningAngular because they use it to pick both the test set and
// (formerly) the gains. Using tuningAngular here silently reapplied the
// WRONG gain set into the WRONG PID (or reapplied gains that were already
// current, a no-op), leaving the actual PID driving the test unchanged no
// matter what you edited -- exactly why the graph looked identical
// regardless of gain changes.
static void applyGains(bool angular) {
  const GainSet &g = angular ? angularGains : lateralGains;
  lemlib::PID fresh(g.gains[0], g.gains[1], g.gains[2], g.windup);
  if (angular)
    new (&chassis.angularPID) lemlib::PID(fresh);
  else
    new (&chassis.lateralPID) lemlib::PID(fresh);
}

static float wrap180(float deg) {
  while (deg > 180.0f)
    deg -= 360.0f;
  while (deg < -180.0f)
    deg += 360.0f;
  return deg;
}

// ─── Test specs & results ────────────────────────────────────────────────────
struct TestSpec {
  const char *label;
  float target;
  int timeoutMs;
  // If true, `target` is an absolute heading/position to drive straight to
  // (used by the sweep's final return-to-start leg). If false (default),
  // `target` is a size applied relative to wherever the chassis currently
  // is (used by every other leg, so they chain instead of resetting).
  bool absolute = false;
};

// Index matches the A / X / B buttons: small / big / return. Heading is
// zeroed before every run, so these are all turns from 0.
static const TestSpec angularTests[3] = {
    {"turn90", 90, 2500}, {"turn180", 180, 3000}, {"turn-90", -90, 2500}};
static const TestSpec lateralTests[3] = {
    {"drive24", 24, 3500}, {"drive48", 48, 5000}, {"driveBack", -24, 3500}};

// R1 sweep: runs every size in order, chained into one continuous motion
// (no stop/reset between legs). Each of the first 3 legs' `target` is its
// OWN turn/drive size (45, then 90, then 180 -- not a checkpoint), applied
// relative to wherever the previous leg stopped, so every leg is a
// genuinely different-sized motion instead of identical hops. The 4th and
// final leg drives straight back to the exact starting pose
// (absolute = true) instead of by a relative size, so the robot ends up
// exactly where it began regardless of any small drift accumulated over
// the first 3 legs.
static const int kSweepLegs = 4;
static const TestSpec angularSweep[kSweepLegs] = {{"turn45", 45, 1200},
                                                  {"turn90", 90, 1500},
                                                  {"turn180", 180, 2000},
                                                  {"return0", 0, 1800, true}};
static const TestSpec lateralSweep[kSweepLegs] = {{"drive12", 12, 1500},
                                                  {"drive24", 24, 2000},
                                                  {"drive48", 48, 3000},
                                                  {"return0", 0, 3500, true}};

struct TestResult {
  float overshoot = 0;
  int settleMs = -1; // first time error stayed in-band for kSettleHoldMs
  float finalError = 0;
  int durationMs = 0;
};

static TestResult lastResult;
static const char *lastLabel = "none";

// True from the moment a test/sweep starts until it finishes. Only ever
// read by the laptop (it's what greys its run buttons out), since the
// tuner loop itself is blocked inside the run meanwhile.
static bool runBusy = false;
// Likewise while calibrateForTuning() is blocking.
static bool calibrating = false;

// ─── Laptop mirror (tools/pid_tuner.py via tools/remote_touch.py) ────────────
// The tuner's twin of driver_menu.cpp's sendRemoteUiState(): one line
// describing the screen, sent whenever something changed (throttled to the
// rate the laptop asked for) and as a 1 s heartbeat otherwise. The common
// segments keep the same shape as the driver menu's so the laptop parses
// both with one function; BTN carries the BACK button so the mirror can
// tap it, and PID / RES carry everything the tuner window shows:
//   PID:<mode>,<sel>,<digitExp>,<aP>,<aI>,<aD>,<lP>,<lI>,<lD>,<busy>,<calibrating>
//   RES:<label>,<overshoot>,<settleMs>,<finalError>,<durationMs>
static void sendTunerState(bool force = false) {
  static uint32_t lastSend = 0, lastActualSend = 0;
  static char lastLine[400] = "";
  const uint32_t kHeartbeatMs = 1000;
  uint32_t interval = remoteSendIntervalMs();
  if (!force && pros::millis() - lastSend < interval)
    return;
  lastSend = pros::millis();

  char line[400];
  snprintf(line, sizeof(line),
           "RUI|PID_TUNER|%s|BTN:410,3,476,27,BACK|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:"
           "|STEP:%.3f|FOOTC:0|PLOT:|TRAIL:|SEL:%d|ACTIVE:-1,-1|PLAN:|PATH:"
           "|PID:%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d|RES:%s,%.2f,%d,%.2f,%d",
           tuningAngular ? "ANGULAR" : "LATERAL", digitStep(), selectedGain,
           tuningAngular ? 0 : 1, selectedGain, digitExp, angularGains.gains[0],
           angularGains.gains[1], angularGains.gains[2], lateralGains.gains[0],
           lateralGains.gains[1], lateralGains.gains[2], runBusy ? 1 : 0, calibrating ? 1 : 0,
           lastLabel,
           lastResult.overshoot, lastResult.settleMs, lastResult.finalError,
           lastResult.durationMs);
  bool changed = strcmp(line, lastLine) != 0;
  if (!force && !changed && pros::millis() - lastActualSend < kHeartbeatMs)
    return;
  strcpy(lastLine, line);
  lastActualSend = pros::millis();
  printf("%s\n", line);
}

// How often to print a CSV sample while a run is in progress. Every sample
// on a cable (the default 150 ms state rate), but only 10/s when the
// laptop said it's on the controller's radio (300 ms) -- that link can't
// carry 50 lines/s, and a backed-up radio delays the STOP coming back in.
static int csvPeriodMs() { return remoteSendIntervalMs() > 150 ? 100 : 20; }

// Cancel request from the laptop (PID STOP, or the mirror's Cancel (X)
// button) -- checked inside every sampling loop.
static bool remoteStopRequested() {
  bool a = takeRemotePidStop();
  bool b = takeRemoteKey(RK_X);
  return a || b;
}

// One run's setpoint schedule, sent up front so the laptop can draw the
// whole pink step line (and lay out the legs' time slots) before the
// first sample arrives -- exactly what the brain graph does.
//   PID|RUN|<angular>|<nLegs>|<label>,<idealLevel>,<timeoutMs>;...
static void sendRunSchedule(bool angular, const TestSpec *tests, const float *levels,
                            int n) {
  char buf[200];
  int len = snprintf(buf, sizeof(buf), "PID|RUN|%d|%d|", angular ? 1 : 0, n);
  for (int i = 0; i < n && len < (int)sizeof(buf); i++)
    len += snprintf(buf + len, sizeof(buf) - len, "%s%s,%.2f,%d", i ? ";" : "",
                    tests[i].label, levels[i], tests[i].timeoutMs);
  printf("%s\n", buf);
}

// ─── Brain screen UI (matches the cyber-HUD palette in main.cpp) ─────────────
namespace ui {
static const int BG = 0x0A0A0F;
static const int CARD = 0x161B22;
static const int CYAN = 0x00F0FF;
static const int ORANGE = 0xFF8C00;
static const int RED = 0xFF3131;
static const int GRAY = 0x777777;
static const int GRID = 0x121217;
static const int SETTLE_BAND_BG = 0x1c2838;
static const int FOOTER_BG = 0x18181F;
static const int TARGET_LINE = 0xFF6B81; // pink/salmon step (setpoint)
static const int ACTUAL_LINE = 0x2EFF8C; // green trace (measured value)

// Graph rectangle (value-vs-time plot)
static const int GX = 160, GY = 40, GW = 310, GH = 160;
} // namespace ui

static const char *kGainNames[3] = {"kP", "kI", "kD"};

// ─── Exit button ──────────────────────────────────────────────────────────
// Tapping this (top-right of the header) exits back to the driver menu --
// otherwise there'd be no way out of the tuner short of restarting the
// program.
static const int kBackX0 = 410, kBackY0 = 3, kBackX1 = 476, kBackY1 = 27;
static void drawBackButton() {
  pros::screen::set_pen(0x000000);
  pros::screen::fill_rect(kBackX0, kBackY0, kBackX1, kBackY1);
  pros::screen::set_eraser(0x000000);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, kBackX0 + 6, kBackY0 + 6, "BACK");
}

// ─── Value-scale graph ────────────────────────────────────────────────────
// Classic PID step-response view: a flat/step target line (pink) and the
// actual value chasing it (green), both against one absolute value axis
// (degrees or inches -- not normalized error), with grid lines at "nice"
// round numbers, e.g. 50 / 30 / 10 / -10.

// Rounds a rough grid spacing up to a "nice" 1/2/5 x 10^n step.
static float niceStep(float roughStep) {
  if (roughStep <= 0.0f)
    return 1.0f;
  float mag = std::pow(10.0f, std::floor(std::log10(roughStep)));
  float norm = roughStep / mag;
  float niceNorm = norm < 1.5f ? 1.0f : norm < 3.0f ? 2.0f : norm < 7.0f ? 5.0f : 10.0f;
  return niceNorm * mag;
}

// Value <-> pixel mapping for the graph, plus the grid step used to label
// it. minVal/maxVal are always "nice" multiples of step.
struct GraphScale {
  float minVal, maxVal, step;
  int toY(float v) const {
    float t = (v - minVal) / (maxVal - minVal);
    return ui::GY + ui::GH - (int)(std::clamp(t, 0.0f, 1.0f) * ui::GH);
  }
};

// Builds a scale covering [loVal, hiVal] with headroom, snapped to nice
// grid lines (so the labeled values look like 50/30/10/-10, not 47.3).
static GraphScale computeScale(float loVal, float hiVal) {
  if (hiVal < loVal)
    std::swap(loVal, hiVal);
  float span = std::max(1.0f, hiVal - loVal);
  float pad = span * 0.15f;
  float step = niceStep((span + 2 * pad) / 4.0f);
  float minVal = std::floor((loVal - pad) / step) * step;
  float maxVal = std::ceil((hiVal + pad) / step) * step;
  if (maxVal <= minVal)
    maxVal = minVal + step;
  return {minVal, maxVal, step};
}

// Clears the graph area and draws its frame + labeled horizontal grid
// lines for the given value scale.
static void drawValueGraphFrame(const GraphScale &sc) {
  using namespace ui;
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(GX, GY, GX + GW, GY + GH);
  pros::screen::set_pen(GRID);
  for (int x = GX; x < GX + GW; x += 40)
    pros::screen::draw_line(x, GY, x, GY + GH);

  pros::screen::set_pen(GRID);
  for (float v = sc.minVal; v <= sc.maxVal + sc.step * 0.5f; v += sc.step) {
    int y = sc.toY(v);
    pros::screen::draw_line(GX, y, GX + GW, y);
  }
  pros::screen::set_pen(GRAY);
  for (float v = sc.minVal; v <= sc.maxVal + sc.step * 0.5f; v += sc.step) {
    int y = sc.toY(v);
    pros::screen::print(pros::E_TEXT_SMALL, GX + 2, y - 11, "%.0f", v);
  }
}

// Draws the full target step schedule in one pass: n (level, slot) pairs,
// each level held flat across its slot with a vertical jump connecting it
// to the previous slot's level. Call once, before sampling starts, since
// the setpoint schedule is known up front.
static void drawTargetSteps(const GraphScale &sc, const int *slotX0,
                            const int *slotW, const float *levels, int n) {
  using namespace ui;
  pros::screen::set_pen(TARGET_LINE);
  int prevY = sc.toY(levels[0]);
  for (int i = 0; i < n; i++) {
    int y = sc.toY(levels[i]);
    int x0 = slotX0[i], x1 = slotX0[i] + slotW[i];
    if (i > 0)
      pros::screen::draw_line(x0, prevY, x0, y);
    pros::screen::draw_line(x0, y, x1, y);
    prevY = y;
  }
}

// Redraws the footer with the given result (used both after a full redraw
// and right after a test completes, without touching the graph trace).
static void drawFooter(const char *label, const TestResult &r) {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(r.settleMs >= 0 ? CYAN : RED);
  pros::screen::print(
      pros::E_TEXT_SMALL, 10, 222,
      "%s // OVERSHOOT %.2f // SETTLE %dms // FINAL %.2f // %dms", label,
      r.overshoot, r.settleMs, r.finalError, r.durationMs);
}

// Full redraw: header, gain panel, empty graph frame, footer.
static void drawTunerUI() {
  using namespace ui;
  const GainSet &g = activeGains();

  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);

  pros::screen::set_pen(tuningAngular ? CYAN : ORANGE);
  pros::screen::fill_rect(0, 0, 480, 30);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 7, "PID TUNER // %s",
                      tuningAngular ? "ANGULAR (turns)" : "LATERAL (drive)");
  drawBackButton();

  for (int i = 0; i < 3; i++) {
    int y = 40 + i * 45;
    bool sel = (i == selectedGain);

    pros::screen::set_pen(sel ? SETTLE_BAND_BG : CARD);
    pros::screen::fill_rect(10, y, 150, y + 38);
    pros::screen::set_pen(sel ? ORANGE : 0x2d2d38);
    pros::screen::draw_rect(10, y, 150, y + 38);
    if (sel) {
      pros::screen::set_pen(ORANGE);
      pros::screen::fill_rect(10, y, 14, y + 38);
    }

    pros::screen::set_pen(sel ? 0xFFFFFF : GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 22, y + 4, "%s", kGainNames[i]);
    pros::screen::set_pen(sel ? CYAN : GRAY);
    pros::screen::print(pros::E_TEXT_MEDIUM, 22, y + 17, "%.3f", g.gains[i]);
  }

  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 178, "Y:mode </>:gain");
  pros::screen::print(pros::E_TEXT_SMALL, 10, 192, "^/v:adj L1/L2:digit");

  {
    float previewTarget = tuningAngular ? 90.0f : 24.0f;
    GraphScale sc = computeScale(0.0f, previewTarget);
    drawValueGraphFrame(sc);
    int x0[1] = {GX}, w[1] = {GW};
    float lvl[1] = {previewTarget};
    drawTargetSteps(sc, x0, w, lvl, 1);
  }
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 160, GY + 2,
                      "A:90 X:180 B:back");
  pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 160, GY + 16,
                      "R1:sweepANG R2:sweepLAT");
  pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 160, GY + 30,
                      "digit step: %.3f", digitStep());

  drawFooter(lastLabel, lastResult);
}

// ─── Controller screen ───────────────────────────────────────────────────────
// The controller LCD only accepts ~1 line per 50ms, so lines rotate.
static void drawControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120)
    return;
  lastDraw = pros::millis();

  const GainSet &g = activeGains();
  switch (line) {
  case 0:
    // Shows which gain is selected and the exact digit step L1/L2 has
    // dialed in (e.g. "ANG kI +/-0.010"), so it's clear what one UP/DOWN
    // press will change before you press it.
    controller.print(0, 0, "%s %s +/-%.3f  ", tuningAngular ? "ANG" : "LAT",
                     kGainNames[selectedGain], digitStep());
    break;
  case 1:
    // kI can now be dialed down to the thousandths place (L2), so it
    // needs 3 decimals here to actually show that. P and D are still
    // legible at fewer decimals, so they're left as-is.
    controller.print(1, 0, "P%.2f I%.3f D%.1f ", g.gains[0], g.gains[1],
                     g.gains[2]);
    break;
  default:
    controller.print(2, 0, "OS%.1f St%d E%.1f  ", lastResult.overshoot,
                     lastResult.settleMs, lastResult.finalError);
    break;
  }
  line = (line + 1) % 3;
}

// ─── Test runner ─────────────────────────────────────────────────────────────
// Runs one motion, samples error every 20ms, plots it live on the brain
// graph, streams CSV to the terminal and records overshoot / settle / final.
static const float kAngularSettleBand = 1.5f; // degrees
static const float kLateralSettleBand = 1.0f; // inches
static const int kSettleHoldMs = 250;
static const int kSampleDelayMs = 20;

// Zeroes the pose before a run: x, y and heading all start at 0. This is
// the sensor reset lemlib supports -- taring the rotation sensors / IMU
// directly would make odometry (which tracks each sensor's previous
// reading) see one giant bogus step.
static void zeroPoseForRun() {
  chassis.setPose(0, 0, 0);
  pros::delay(20); // let the odometry task pick the new pose up
  printf("POSE ZEROED\n");
}

static TestResult runTest(bool angular, const TestSpec &spec) {
  const float settleBand = angular ? kAngularSettleBand : kLateralSettleBand;
  const float target = spec.target;
  const int timeoutMs = spec.timeoutMs;

  runBusy = true;
  zeroPoseForRun();
  remoteStopRequested(); // drop a stale STOP so it can't cancel this run
  sendTunerState(true);
  {
    float level[1] = {target};
    sendRunSchedule(angular, &spec, level, 1);
  }

  applyGains(angular);
  float startVal;
  if (angular) {
    startVal = chassis.getPose().theta;
    chassis.turnToHeading(target, timeoutMs, {}, true);
  } else {
    startVal = 0;
    // A negative target (the "driveBack" test) is behind the chassis, not
    // in front of it -- drive there backwards instead of turning 180 deg
    // to face it first, which is what moveToPoint's default (forwards =
    // true) would otherwise do.
    chassis.moveToPoint(0, target, timeoutMs, {.forwards = target >= 0},
                        true);
  }
  // isInMotion() can briefly read false right after an async motion is
  // issued, before its task has flagged itself running. Give it a moment to
  // spin up so the sampling loop below doesn't fall through immediately
  // (which would let the next queued test reconstruct the PID mid-motion).
  pros::delay(10);

  // Value-scale graph: a flat target line at `target` for the whole run,
  // and the actual value chasing it, both on one absolute axis covering
  // wherever we started through the target (with headroom).
  GraphScale sc = computeScale(startVal, target);
  drawValueGraphFrame(sc);
  int stepX0[1] = {ui::GX}, stepW[1] = {ui::GW};
  float stepLevel[1] = {target};
  drawTargetSteps(sc, stepX0, stepW, stepLevel, 1);

  TestResult res;
  uint32_t start = pros::millis();
  float initialError = 0;
  bool crossedTarget = false;
  bool first = true;
  bool cancelled = false;
  int inBandMs = 0;
  int prevX = ui::GX, prevY = sc.toY(startVal);
  const int csvEvery = csvPeriodMs();
  uint32_t lastCsv = 0;
  bool firstSample = true;

  printf("PID|LEG|0|%s|%.2f|%.2f|%d\n", spec.label, target, startVal, timeoutMs);
  printf("CSV,ms,target,actual,error\n");
  while (chassis.isInMotion()) {
    if (remoteStopRequested()) {
      chassis.cancelMotion();
      cancelled = true;
    }
    lemlib::Pose pose = chassis.getPose();
    float actual = angular ? pose.theta : pose.y;
    float error = angular ? wrap180(target - actual) : (target - actual);
    uint32_t t = pros::millis() - start;

    if (first) {
      initialError = error;
      first = false;
    }
    // Overshoot: worst error after the trace first crosses the target.
    if (!crossedTarget && initialError != 0 &&
        (error == 0 || (error > 0) != (initialError > 0)))
      crossedTarget = true;
    if (crossedTarget)
      res.overshoot = std::max(res.overshoot, std::abs(error));

    if (std::abs(error) < settleBand) {
      inBandMs += kSampleDelayMs;
      if (inBandMs >= kSettleHoldMs && res.settleMs < 0)
        res.settleMs = (int)t - kSettleHoldMs;
    } else {
      inBandMs = 0;
      res.settleMs = -1; // must stay settled: re-arm if it leaves the band
    }
    res.finalError = error;

    // Live plot: x = time across the run window, y = actual value (using
    // target - wrap-corrected error rather than the raw pose so an angular
    // trace doesn't glitch across the +-180 wrap boundary).
    int x = std::clamp(ui::GX + (int)((float)t / timeoutMs * ui::GW), ui::GX,
                       ui::GX + ui::GW - 1);
    int y = sc.toY(target - error);
    pros::screen::set_pen(ui::ACTUAL_LINE);
    pros::screen::draw_line(prevX, prevY, x, y);
    prevX = x;
    prevY = y;

    if (firstSample || t - lastCsv >= (uint32_t)csvEvery) {
      firstSample = false;
      lastCsv = t;
      printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, target, actual,
             error);
    }
    sendTunerState(); // heartbeat for the laptop while the loop is busy
    pros::delay(kSampleDelayMs);
  }
  // Belt-and-suspenders: make sure the motion task is fully finished (and
  // its hold on the chassis released) before returning, so a chained test
  // (sweep) can't reconstruct the PID object while this one is still
  // wrapping up.
  chassis.waitUntilDone();
  res.durationMs = (int)(pros::millis() - start);

  lastLabel = spec.label;
  lastResult = res;
  printf("RESULT %s: overshoot=%.2f settle=%dms final=%.2f dur=%dms\n",
         spec.label, res.overshoot, res.settleMs, res.finalError,
         res.durationMs);
  printf("PID|END|0|%s|%.2f|%d|%.2f|%d\n", spec.label, res.overshoot, res.settleMs,
         res.finalError, res.durationMs);
  printf("PID|DONE|%d\n", cancelled ? 1 : 0);
  controller.rumble(".");
  drawFooter(spec.label, res);
  runBusy = false;
  sendTunerState(true);
  return res;
}

// Runs the 4-leg sequence (see angularSweep / lateralSweep) as ONE
// continuous chain: no stop, no pose reset between legs -- each leg turns/
// drives by its OWN size (45, then 90, then 135, then 180 deg -- or
// 12/24/36/48 in) relative to wherever the previous leg ended, so the legs
// are genuinely different sizes rather than four identical 45 deg hops to
// evenly-spaced absolute checkpoints. Gains are only reapplied (applyGains)
// once the previous leg's motion task has fully finished
// (chassis.waitUntilDone() inside the sampling loop below), so nothing gets
// reconstructed mid-motion.
//
// All 4 legs are drawn on ONE graph instead of each clearing the last: the
// frame is split into 4 time slots (sized by each leg's timeout), with a
// gray divider + target label at the start of each slot after the first.
static void runSweep(bool angular) {
  const TestSpec *tests = angular ? angularSweep : lateralSweep;
  const float settleBand = angular ? kAngularSettleBand : kLateralSettleBand;

  runBusy = true;
  remoteStopRequested(); // drop a stale STOP so it can't cancel this run
  sendTunerState(true);

  // Known, repeatable starting point so "45/90/135/180" (or
  // "12/24/36/48") land where labeled instead of drifting from whatever
  // heading/position preceded the sweep.
  zeroPoseForRun();

  // Precompute each leg's time slot (x0, width) and its IDEAL cumulative
  // target level (running sum of leg sizes, or the absolute value itself
  // for the return leg), so the whole step schedule can be drawn up front --
  // the setpoint sequence is known before any motion runs, exactly like the
  // pink line in a real PID step-response plot.
  int totalTimeout = 0;
  for (int i = 0; i < kSweepLegs; i++)
    totalTimeout += tests[i].timeoutMs;
  int slotX0[kSweepLegs], slotW[kSweepLegs];
  float idealLevel[kSweepLegs];
  int xBase = ui::GX;
  float cum = 0;
  for (int i = 0; i < kSweepLegs; i++) {
    slotW[i] =
        std::max(1, (int)((float)tests[i].timeoutMs / totalTimeout * ui::GW));
    slotX0[i] = xBase;
    xBase += slotW[i];
    cum = tests[i].absolute ? tests[i].target : cum + tests[i].target;
    idealLevel[i] = cum;
  }

  sendRunSchedule(angular, tests, idealLevel, kSweepLegs);

  float hiVal = *std::max_element(idealLevel, idealLevel + kSweepLegs);
  GraphScale sc = computeScale(0.0f, hiVal);
  drawValueGraphFrame(sc);
  drawTargetSteps(sc, slotX0, slotW, idealLevel, kSweepLegs);
  pros::screen::set_pen(ui::GRAY);
  for (int i = 0; i < kSweepLegs; i++) {
    if (i > 0)
      pros::screen::draw_line(slotX0[i], ui::GY, slotX0[i], ui::GY + ui::GH);
    pros::screen::print(pros::E_TEXT_SMALL, slotX0[i] + 2, ui::GY + 2, "%.0f",
                        tests[i].target);
  }

  printf("SWEEP START (%s)\n", angular ? "angular" : "lateral");

  // Actual-value trace carries continuously across legs (the chassis
  // physically doesn't jump between them, even though the target does), so
  // prevX/prevY live outside the leg loop instead of resetting per leg.
  int prevX = ui::GX, prevY = sc.toY(0.0f);
  bool cancelled = false;
  const int csvEvery = csvPeriodMs();

  for (int i = 0; i < kSweepLegs && !cancelled; i++) {
    const TestSpec &spec = tests[i];
    int x0 = slotX0[i], slotW_i = slotW[i];

    applyGains(angular);
    // spec.target is this leg's OWN turn/drive size, applied relative to
    // wherever the chassis is right now (so leg i actually moves
    // spec.target worth of degrees/inches, not just closes the gap to the
    // next evenly-spaced checkpoint) -- except the final leg, which drives
    // straight to the exact starting pose (spec.absolute) so any drift
    // accumulated over the previous legs doesn't carry into where "0" ends
    // up.
    float startVal = angular ? chassis.getPose().theta : chassis.getPose().y;
    float target = spec.absolute ? spec.target : startVal + spec.target;
    if (angular) {
      chassis.turnToHeading(target, spec.timeoutMs, {}, true);
    } else {
      // Chain lateral legs into one continuous drive instead of fully
      // stopping at each waypoint: every leg except the final
      // return-to-start exits early (minSpeed + earlyExitRange) once
      // within kLateralSettleBand of its target, so the chassis carries
      // speed straight into the next leg's moveToPoint call rather than
      // decelerating to a stop first. The return leg still fully settles,
      // since it needs to land exactly back at the start -- and it drives
      // there backwards (forwards = false) instead of turning 180 deg to
      // face the point first, since the point is directly behind it.
      lemlib::MoveToPointParams params;
      if (i < kSweepLegs - 1) {
        params.minSpeed = 60;
        params.earlyExitRange = kLateralSettleBand;
      } else {
        params.forwards = false;
      }
      chassis.moveToPoint(0, target, spec.timeoutMs, params, true);
    }
    // See runTest: isInMotion() can briefly read false right after an
    // async motion starts, before its task flags itself running.
    pros::delay(10);

    TestResult res;
    uint32_t start = pros::millis();
    float initialError =
        angular ? wrap180(target - startVal) : (target - startVal);
    bool crossedTarget = false;
    int inBandMs = 0;
    uint32_t lastCsv = 0;
    bool firstSample = true;

    printf("PID|LEG|%d|%s|%.2f|%.2f|%d\n", i, spec.label, target, startVal, spec.timeoutMs);
    printf("CSV,ms,target,actual,error\n");
    while (chassis.isInMotion()) {
      if (remoteStopRequested()) {
        chassis.cancelMotion();
        cancelled = true;
      }
      lemlib::Pose pose = chassis.getPose();
      float actual = angular ? pose.theta : pose.y;
      float error = angular ? wrap180(target - actual) : (target - actual);
      uint32_t t = pros::millis() - start;

      if (!crossedTarget && initialError != 0 &&
          (error == 0 || (error > 0) != (initialError > 0)))
        crossedTarget = true;
      if (crossedTarget)
        res.overshoot = std::max(res.overshoot, std::abs(error));

      if (std::abs(error) < settleBand) {
        inBandMs += kSampleDelayMs;
        if (inBandMs >= kSettleHoldMs && res.settleMs < 0)
          res.settleMs = (int)t - kSettleHoldMs;
      } else {
        inBandMs = 0;
        res.settleMs = -1;
      }
      res.finalError = error;

      // x = time within this leg's slot. y = actual value, computed as
      // target - wrap-corrected error (not the raw pose) so the trace
      // doesn't glitch across the +-180 wrap boundary on big angular
      // sweeps -- see runTest.
      int x = std::clamp(x0 + (int)((float)t / spec.timeoutMs * slotW_i), x0,
                         x0 + slotW_i - 1);
      int y = sc.toY(target - error);
      pros::screen::set_pen(ui::ACTUAL_LINE);
      pros::screen::draw_line(prevX, prevY, x, y);
      prevX = x;
      prevY = y;

      if (firstSample || t - lastCsv >= (uint32_t)csvEvery) {
        firstSample = false;
        lastCsv = t;
        printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, target, actual,
               error);
      }
      sendTunerState(); // heartbeat for the laptop while the loop is busy
      pros::delay(kSampleDelayMs);
    }
    // Make sure this leg's motion task is fully done (and its hold on the
    // chassis released) before the next leg reconstructs the PID via
    // applyGains() -- otherwise the next leg can corrupt this one mid-flight.
    chassis.waitUntilDone();
    res.durationMs = (int)(pros::millis() - start);

    lastLabel = spec.label;
    lastResult = res;
    printf("RESULT %s: overshoot=%.2f settle=%dms final=%.2f dur=%dms\n",
           spec.label, res.overshoot, res.settleMs, res.finalError,
           res.durationMs);
    printf("PID|END|%d|%s|%.2f|%d|%.2f|%d\n", i, spec.label, res.overshoot, res.settleMs,
           res.finalError, res.durationMs);
    controller.rumble(".");
    drawFooter(spec.label, res);
  }

  printf("SWEEP %s\n", cancelled ? "CANCELLED" : "DONE");
  printf("PID|DONE|%d\n", cancelled ? 1 : 0);
  controller.rumble("---");
  runBusy = false;
  sendTunerState(true);
}

// ─── Calibration ─────────────────────────────────────────────────────────────
// Re-runs chassis.calibrate() (IMU reset, ~3 s, the robot must sit still)
// and zeroes the pose, so every test starts from a heading and position
// that are actually trustworthy -- not whatever the IMU drifted to since
// initialize(), or wherever the last test left the odometry. Runs once
// when the tuner opens and on PID CAL from the laptop.
static void calibrateForTuning() {
  calibrating = true;
  sendTunerState(true);
  pros::screen::set_pen(ui::ORANGE);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_eraser(ui::ORANGE);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                      "CALIBRATING -- keep the robot still (~3 s)");
  pros::screen::set_eraser(ui::BG);
  controller.print(0, 0, "CALIBRATING...  ");
  printf("PID TUNER CALIBRATING\n");

  chassis.calibrate();
  chassis.setPose(0, 0, 0);

  calibrating = false;
  printf("PID TUNER CALIBRATED\n");
  controller.rumble("-");
  drawTunerUI();
  sendTunerState(true);
}

// ─── Button edge detection ───────────────────────────────────────────────────
// Wraps a digital button so callers can ask "was this just pressed?" without
// hand-rolling a `wasX` bool per button.
struct EdgeButton {
  pros::controller_digital_e_t id;
  bool prevHeld = false;

  bool held() const { return controller.get_digital(id); }

  bool pressed() {
    bool now = held();
    bool edge = now && !prevHeld;
    prevHeld = now;
    return edge;
  }
};

// ─── Main tuner loop ─────────────────────────────────────────────────────────
// Applies one "PID ..." line from the laptop. Returns true if the screen
// needs redrawing. TEST / SWEEP are run right here (they block, like the
// button-driven ones) -- except when `allowRuns` is false, which is how
// commands that piled up while a run was already in progress are drained:
// their gain edits still apply, but a queued-up test doesn't suddenly set
// the robot off again the moment the previous one ends.
static bool applyRemotePidCommand(const RemotePidCommand &c, bool allowRuns) {
  bool angular = c.mode == 0;
  switch (c.kind) {
  case RemotePidCommand::SET: {
    if (c.index < 0 || c.index > 2)
      return false;
    GainSet &g = angular ? angularGains : lateralGains;
    g.gains[c.index] = std::max(0.0f, c.value);
    printf("PID SET %s %s = %.4f\n", angular ? "angular" : "lateral", kGainNames[c.index],
           g.gains[c.index]);
    return true;
  }
  case RemotePidCommand::MODE:
    tuningAngular = angular;
    return true;
  case RemotePidCommand::SEL:
    if (c.index < 0 || c.index > 2)
      return false;
    selectedGain = c.index;
    return true;
  case RemotePidCommand::DIGIT:
    digitExp = std::clamp(c.index, kDigitExpMin, kDigitExpMax);
    return true;
  case RemotePidCommand::TEST:
    if (!allowRuns || c.index < 0 || c.index > 2)
      return false;
    runTest(angular, (angular ? angularTests : lateralTests)[c.index]);
    return false;
  case RemotePidCommand::SWEEP:
    if (!allowRuns)
      return false;
    runSweep(angular);
    return false;
  case RemotePidCommand::CAL:
    if (!allowRuns)
      return false;
    calibrateForTuning();
    return false;
  }
  return false;
}

// Anything the laptop sent while a run was blocking the loop: keep the
// gain edits, drop the runs (see applyRemotePidCommand).
static bool drainQueuedRemotePidCommands() {
  bool dirty = false;
  RemotePidCommand c;
  while (takeRemotePidCommand(c))
    dirty |= applyRemotePidCommand(c, false);
  return dirty;
}

// ─── Main tuner loop ─────────────────────────────────────────────────────────
void pidTunerControl() {
  pidTunerActive = true;
  loadGainsFromChassisConfig();
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("PID TUNER ACTIVE\n");
  drawTunerUI();
  sendTunerState(true);
  calibrateForTuning(); // fresh heading / zeroed pose before any test runs

  EdgeButton up{pros::E_CONTROLLER_DIGITAL_UP};
  EdgeButton down{pros::E_CONTROLLER_DIGITAL_DOWN};
  EdgeButton left{pros::E_CONTROLLER_DIGITAL_LEFT};
  EdgeButton right{pros::E_CONTROLLER_DIGITAL_RIGHT};
  EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
  EdgeButton btnB{pros::E_CONTROLLER_DIGITAL_B};
  EdgeButton btnX{pros::E_CONTROLLER_DIGITAL_X};
  EdgeButton btnY{pros::E_CONTROLLER_DIGITAL_Y};
  EdgeButton btnR1{pros::E_CONTROLLER_DIGITAL_R1};
  EdgeButton btnR2{pros::E_CONTROLLER_DIGITAL_R2};
  EdgeButton btnL1{pros::E_CONTROLLER_DIGITAL_L1};
  EdgeButton btnL2{pros::E_CONTROLLER_DIGITAL_L2};

  bool wasTouched = false;
  while (true) {
    sendTunerState(); // throttled internally; keeps the laptop window live

    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    bool touchPress = status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int touchX = status.x, touchY = status.y;
    if (status.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (status.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    // A tap from the laptop mirror counts like a real one (see
    // driver_menu.cpp) -- BACK is the only thing here to tap.
    if (!touchPress && takeRemoteTouch(touchX, touchY))
      touchPress = true;
    if (touchPress && touchX >= kBackX0 && touchX <= kBackX1 && touchY >= kBackY0 &&
        touchY <= kBackY1) {
      pidTunerActive = false;
      controller.rumble(".");
      printf("PID TUNER EXIT\n");
      return; // back to the driver menu
    }

    // Physical buttons or the laptop's KEY lines -- same edges either way,
    // except A / B / X: the laptop starts tests with PID TEST instead, and
    // its X is a stop (see the header comment), so those three are only
    // ever physical here.
    bool upEdge = up.pressed() || takeRemoteKey(RK_UP);
    bool downEdge = down.pressed() || takeRemoteKey(RK_DOWN);
    bool leftEdge = left.pressed() || takeRemoteKey(RK_LEFT);
    bool rightEdge = right.pressed() || takeRemoteKey(RK_RIGHT);
    bool aEdge = btnA.pressed();
    bool bEdge = btnB.pressed();
    bool xEdge = btnX.pressed();
    bool yEdge = btnY.pressed() || takeRemoteKey(RK_Y);
    bool r1Edge = btnR1.pressed() || takeRemoteKey(RK_R1);
    bool r2Edge = btnR2.pressed() || takeRemoteKey(RK_R2);
    bool l1Edge = btnL1.pressed() || takeRemoteKey(RK_L1);
    bool l2Edge = btnL2.pressed() || takeRemoteKey(RK_L2);
    takeRemoteKey(RK_A); // ignored (see above); drop so they don't pile up
    takeRemoteKey(RK_B);
    takeRemoteKey(RK_X); // only meaningful mid-run; drop stale ones
    takeRemotePidStop();

    bool dirty = yEdge || leftEdge || rightEdge || l1Edge || l2Edge;
    if (yEdge)
      tuningAngular = !tuningAngular;
    if (rightEdge)
      selectedGain = (selectedGain + 1) % 3;
    if (leftEdge)
      selectedGain = (selectedGain + 2) % 3;
    if (l1Edge)
      digitExp = std::min(digitExp + 1, kDigitExpMax);
    if (l2Edge)
      digitExp = std::max(digitExp - 1, kDigitExpMin);

    if (upEdge || downEdge) {
      GainSet &g = activeGains();
      float step = downEdge ? -digitStep() : digitStep();
      g.gains[selectedGain] = std::max(0.0f, g.gains[selectedGain] + step);
      dirty = true;
    }

    const TestSpec *tests = tuningAngular ? angularTests : lateralTests;
    bool ran = true;
    if (aEdge) {
      runTest(tuningAngular, tests[0]);
    } else if (xEdge) {
      runTest(tuningAngular, tests[1]);
    } else if (bEdge) {
      runTest(tuningAngular, tests[2]);
    } else if (r1Edge) {
      runSweep(true); // R1 always runs the ANGULAR sweep, regardless of Y mode
    } else if (r2Edge) {
      runSweep(false); // R2 always runs the LATERAL sweep, regardless of Y mode
    } else {
      ran = false;
      // One laptop command per tick, so a SET that arrives just before a
      // TEST is applied before the test starts.
      RemotePidCommand cmd;
      if (takeRemotePidCommand(cmd)) {
        bool isRun = cmd.kind == RemotePidCommand::TEST || cmd.kind == RemotePidCommand::SWEEP ||
                     cmd.kind == RemotePidCommand::CAL;
        dirty |= applyRemotePidCommand(cmd, true);
        ran = isRun;
      }
    }
    if (ran) {
      // The run blocked this loop; whatever queued up meanwhile gets its
      // gain edits applied but its runs dropped. Only redraw if one of
      // those edits landed -- a redraw wipes the run's graph.
      if (drainQueuedRemotePidCommands())
        drawTunerUI();
      sendTunerState(true);
    } else if (dirty) {
      drawTunerUI();
      sendTunerState(true);
    }

    drawControllerUI();
    pros::delay(20);
  }
}
