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
//   DPAD-UP      increase gain by one step
//   DPAD-DOWN    decrease gain by one step
//   hold R2      fine step (x0.1) while pressing UP/DOWN
//   Steps: ANGULAR kP 0.1 / kI 0.01 / kD 0.5
//          LATERAL kP 0.5 / kI 0.01 / kD 1.0   (see kAngular / kLateral)
//
// RUNNING TESTS (make sure the robot has clear space!)
//   A            small test:  90 deg turn   / 24 in drive forward
//   X            big test:    180 deg turn  / 48 in drive forward
//   B            return test: turn back to 0 / 24 in drive backward
//
// READING THE RESULTS
//   Brain graph: error vs time. Cyan = approaching target, red = after
//   crossing the target (overshoot). Shaded band = settle zone. Footer and
//   controller line 3 show OVERSHOOT / SETTLE ms / FINAL error.
//   Full CSV telemetry streams over USB: run `pros terminal`.
//
// TUNING RECIPE
//   1. Raise kP until the trace slightly overshoots (small red bounce).
//   2. Add kD until the red bounce disappears and settle time drops.
//   3. Only add kI if the trace flattens just OUTSIDE the settle band.
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
// gains = {kP, kI, kD}; steps = coarse adjustment per gain (hold R2 for x0.1).
// Starting values mirror lateral_controller / angular_controller in main.cpp.
// Edits live only in RAM — once a set feels good, copy it back into main.cpp.
struct GainSet {
  float gains[3];
  float windup;
  const float steps[3];
};

static GainSet angularGains{{1.5f, 0.0f, 10.0f}, 0.0f, {0.1f, 0.01f, 0.5f}};
static GainSet lateralGains{{10.0f, 0.0f, 28.0f}, 3.0f, {0.5f, 0.01f, 1.0f}};

static bool tuningAngular = true;
static int selectedGain = 0; // index into GainSet::gains: 0=kP, 1=kI, 2=kD

static GainSet &activeGains() {
  return tuningAngular ? angularGains : lateralGains;
}

// LemLib exposes chassis.angularPID / lateralPID but their gains are const,
// so the only way to swap gains at runtime is to reconstruct the PID in
// place. lemlib::PID holds no resources, so this is safe.
static void applyGains() {
  const GainSet &g = activeGains();
  lemlib::PID fresh(g.gains[0], g.gains[1], g.gains[2], g.windup);
  if (tuningAngular)
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
};

// Index matches the A / X / B buttons: small / big / return.
static const TestSpec angularTests[3] = {
    {"turn90", 90, 2500}, {"turn180", 180, 3000}, {"turn0", 0, 2500}};
static const TestSpec lateralTests[3] = {
    {"drive24", 24, 3500}, {"drive48", 48, 5000}, {"driveBack", -24, 3500}};

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

// Graph rectangle (error-vs-time plot)
static const int GX = 160, GY = 40, GW = 310, GH = 160;
} // namespace ui

static const char *kGainNames[3] = {"kP", "kI", "kD"};

// Clears the graph area and draws its frame, zero line and settle band.
// errScale = error magnitude mapped to the top edge of the plot.
static void drawGraphFrame(float errScale, float settleBand) {
  using namespace ui;
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(GX, GY, GX + GW, GY + GH);
  pros::screen::set_pen(GRID);
  for (int x = GX; x < GX + GW; x += 40)
    pros::screen::draw_line(x, GY, x, GY + GH);

  int midY = GY + GH / 2;
  int bandPx = (int)(settleBand / errScale * (GH / 2));
  pros::screen::set_pen(SETTLE_BAND_BG);
  pros::screen::fill_rect(GX, midY - bandPx, GX + GW, midY + bandPx);
  pros::screen::set_pen(ORANGE);
  pros::screen::draw_line(GX, midY, GX + GW, midY);

  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, GX + 4, GY + 2, "+%.0f", errScale);
  pros::screen::print(pros::E_TEXT_SMALL, GX + 4, GY + GH - 14, "-%.0f",
                      errScale);
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
  pros::screen::print(pros::E_TEXT_SMALL, 10, 192, "^/v:adj R2:fine");

  drawGraphFrame(tuningAngular ? 90.0f : 24.0f, tuningAngular ? 1.5f : 1.0f);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 130, GY + 2,
                      "A:90 X:180 B:back");

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
    controller.print(0, 0, "%s edit:%s       ", tuningAngular ? "ANG" : "LAT",
                     kGainNames[selectedGain]);
    break;
  case 1:
    controller.print(1, 0, "P%.2f I%.2f D%.1f  ", g.gains[0], g.gains[1],
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

  applyGains();
  float startVal;
  if (angular) {
    startVal = chassis.getPose().theta;
    chassis.turnToHeading(target, timeoutMs, {}, true);
  } else {
    chassis.setPose(0, 0, 0);
    startVal = 0;
    chassis.moveToPoint(0, target, timeoutMs, {}, true);
  }

  // Graph scale: initial error magnitude fills half the plot height, with
  // headroom for overshoot.
  float initialMag =
      std::max(1.0f, angular ? std::abs(wrap180(target - startVal))
                              : std::abs(target));
  float errScale = initialMag * 1.2f;
  drawGraphFrame(errScale, settleBand);

  TestResult res;
  uint32_t start = pros::millis();
  float initialError = 0;
  bool crossedTarget = false;
  bool first = true;
  int inBandMs = 0;
  int prevX = ui::GX, prevY = ui::GY + ui::GH / 2;

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

    // Live plot: x = time across the run window, y = error around midline.
    int x = std::clamp(ui::GX + (int)((float)t / timeoutMs * ui::GW), ui::GX,
                       ui::GX + ui::GW - 1);
    float norm = std::clamp(error / errScale, -1.0f, 1.0f);
    int y = ui::GY + ui::GH / 2 - (int)(norm * (ui::GH / 2 - 2));
    pros::screen::set_pen(crossedTarget ? ui::RED : ui::CYAN);
    pros::screen::draw_line(prevX, prevY, x, y);
    prevX = x;
    prevY = y;

    printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, target, actual,
           error);
    pros::delay(kSampleDelayMs);
  }
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

  while (true) {
    bool upEdge = up.pressed(), downEdge = down.pressed();
    bool leftEdge = left.pressed(), rightEdge = right.pressed();
    bool aEdge = btnA.pressed(), bEdge = btnB.pressed(), xEdge = btnX.pressed();
    bool yEdge = btnY.pressed();
    bool fine = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool dirty = yEdge || leftEdge || rightEdge;
    if (yEdge)
      tuningAngular = !tuningAngular;
    if (rightEdge)
      selectedGain = (selectedGain + 1) % 3;
    if (leftEdge)
      selectedGain = (selectedGain + 2) % 3;

    if (upEdge || downEdge) {
      GainSet &g = activeGains();
      float step = g.steps[selectedGain] * (fine ? 0.1f : 1.0f);
      if (downEdge)
        step = -step;
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
    } else if (dirty) {
      drawTunerUI();
    }

    drawControllerUI();
    pros::delay(20);
  }
}
