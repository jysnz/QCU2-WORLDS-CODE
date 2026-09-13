// ─────────────────────────────────────────────────────────────────────────────
// PID TUNER — CONTROLS REFERENCE
// ─────────────────────────────────────────────────────────────────────────────
// ENTERING THE TUNER
//   Hold DPAD-LEFT when driver control starts (opcontrol). Normal driving
//   runs if the button is not held.
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
//   B            return test: turn back to 0 / 24 in drive backward
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
//   4. Verify with X (big test) and B (return test), then copy the final
//      values into lateral_controller / angular_controller in main.cpp.
// ─────────────────────────────────────────────────────────────────────────────

#include "pid_tuner.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <new>

bool pidTunerActive = false;

// ─── Gain state ──────────────────────────────────────────────────────────────
// gains = {kP, kI, kD}. Starting values mirror lateral_controller /
// angular_controller in main.cpp. Edits live only in RAM -- once a set
// feels good, copy it back into main.cpp.
struct GainSet {
  float gains[3];
  float windup;
};

static GainSet angularGains{{1.5f, 0.0f, 10.0f}, 0.0f};
static GainSet lateralGains{{10.0f, 0.0f, 28.0f}, 3.0f};

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

// Index matches the A / X / B buttons: small / big / return.
static const TestSpec angularTests[3] = {
    {"turn90", 90, 2500}, {"turn180", 180, 3000}, {"turn0", 0, 2500}};
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

static TestResult runTest(bool angular, const TestSpec &spec) {
  const float settleBand = angular ? kAngularSettleBand : kLateralSettleBand;
  const float target = spec.target;
  const int timeoutMs = spec.timeoutMs;

  applyGains(angular);
  float startVal;
  if (angular) {
    startVal = chassis.getPose().theta;
    chassis.turnToHeading(target, timeoutMs, {}, true);
  } else {
    chassis.setPose(0, 0, 0);
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
  int inBandMs = 0;
  int prevX = ui::GX, prevY = sc.toY(startVal);

  printf("CSV,ms,target,actual,error\n");
  while (chassis.isInMotion()) {
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

    printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, target, actual,
           error);
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
  controller.rumble(".");
  drawFooter(spec.label, res);
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

  // Known, repeatable starting point so "45/90/135/180" (or
  // "12/24/36/48") land where labeled instead of drifting from whatever
  // heading/position preceded the sweep. Angular only zeroes heading;
  // lateral zeroes the whole pose (same as a standalone lateral test).
  if (angular) {
    lemlib::Pose p = chassis.getPose();
    chassis.setPose(p.x, p.y, 0);
  } else {
    chassis.setPose(0, 0, 0);
  }

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

  for (int i = 0; i < kSweepLegs; i++) {
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

    printf("CSV,ms,target,actual,error\n");
    while (chassis.isInMotion()) {
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

      printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, target, actual,
             error);
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
    controller.rumble(".");
    drawFooter(spec.label, res);
  }

  printf("SWEEP DONE\n");
  controller.rumble("---");
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
void pidTunerControl() {
  pidTunerActive = true;
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("PID TUNER ACTIVE\n");
  drawTunerUI();

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

  while (true) {
    bool upEdge = up.pressed(), downEdge = down.pressed();
    bool leftEdge = left.pressed(), rightEdge = right.pressed();
    bool aEdge = btnA.pressed(), bEdge = btnB.pressed(), xEdge = btnX.pressed();
    bool yEdge = btnY.pressed();
    bool r1Edge = btnR1.pressed(), r2Edge = btnR2.pressed();
    bool l1Edge = btnL1.pressed(), l2Edge = btnL2.pressed();

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
    } else if (dirty) {
      drawTunerUI();
    }

    drawControllerUI();
    pros::delay(20);
  }
}
