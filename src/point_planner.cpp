// ─────────────────────────────────────────────────────────────────────────────
// POINT PLANNER — see include/point_planner.hpp for the full controls
// reference. Short version: dial in (x, y) and a speed with the controller,
// press A, and the chassis runs a single chassis.moveToPoint() to that field
// position. R1 swaps the tool over to chassis.turnToHeading() mode, where
// you dial in a heading and speed instead. Nothing more -- no waypoint
// queue, no PID editing.
// ─────────────────────────────────────────────────────────────────────────────

#include "point_planner.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

bool pointPlannerActive = false;

// ─── Mode ────────────────────────────────────────────────────────────────────
// MOVE_TO_POINT drives to (targetX, targetY); TURN_TO_HEADING turns in place
// to targetHeading. R1 toggles between them.
enum class PlannerMode { MOVE_TO_POINT, TURN_TO_HEADING };
static PlannerMode mode = PlannerMode::MOVE_TO_POINT;

// ─── Target state ────────────────────────────────────────────────────────────
static float targetX = 24.0f;
static float targetY = 24.0f;
static float targetHeading = 0.0f;
static float targetSpeed = 127.0f; // 0-127, passed as maxSpeed
static int selectedField = 1;      // index into the current mode's field list
static bool driveForwards = true;  // moveToPoint only

// Number of editable fields for the current mode: {X, Y, SPEED} or
// {HEADING, SPEED}.
static int fieldCount() {
  return mode == PlannerMode::MOVE_TO_POINT ? 3 : 2;
}

// Pointer to whichever field is currently selected, so UP/DOWN and the
// digit cursor can edit it without a mode-specific switch at every call
// site.
static float *selectedFieldPtr() {
  if (mode == PlannerMode::MOVE_TO_POINT) {
    switch (selectedField) {
    case 0:
      return &targetX;
    case 1:
      return &targetY;
    default:
      return &targetSpeed;
    }
  }
  return selectedField == 0 ? &targetHeading : &targetSpeed;
}

// Digit cursor, same idea as the PID tuner: UP/DOWN adjusts the selected
// field by 10^digitExp, and L1/L2 move which digit that is.
static int digitExp = 0;
static const int kDigitExpMax = 2;  // hundreds
static const int kDigitExpMin = -2; // hundredths
static float digitStep() { return std::pow(10.0f, (float)digitExp); }

// ─── Last run result ─────────────────────────────────────────────────────────
struct PlanResult {
  bool ran = false;
  bool cancelled = false; // true if X aborted the run early
  bool isTurn = false;    // true if this was a turnToHeading run
  float finalX = 0, finalY = 0, finalHeading = 0;
  float error = 0; // straight-line distance (in) or heading error (deg)
  int durationMs = 0;
};
static PlanResult lastResult;

// ─── Brain screen UI (matches the palette used by the PID tuner) ────────────
namespace ui {
static const int BG = 0x0A0A0F;
static const int CARD = 0x161B22;
static const int CYAN = 0x00F0FF;
static const int ORANGE = 0xFF8C00;
static const int GRAY = 0x777777;
static const int GRID = 0x121217;
static const int AXIS = 0x2d2d38;
static const int SEL_BG = 0x1c2838;
static const int FOOTER_BG = 0x18181F;
static const int TARGET_LINE = 0xFF6B81; // pink crosshair (target)
static const int PATH_LINE = 0x2EFF8C;   // green trail (actual path)

// Field-plot rectangle -- same footprint as the PID tuner's graph, so the
// two tools share a layout.
static const int FX = 160, FY = 40, FW = 310, FH = 160;
static const float FIELD_HALF = 72.0f; // inches shown each side of center
} // namespace ui

// Inches -> pixel, uniform on both axes (no stretching), centered in the
// field-plot rectangle. Anything past FIELD_HALF is clamped to the edge of
// the plot so an out-of-range target still shows (at the border) instead of
// vanishing off-screen.
static void toPixel(float x, float y, int &px, int &py) {
  using namespace ui;
  float ppi = (FH / 2.0f) / FIELD_HALF; // pixels per inch
  float dx = std::clamp(x * ppi, -(float)(FW / 2 - 4), (float)(FW / 2 - 4));
  float dy = std::clamp(y * ppi, -(float)(FH / 2 - 4), (float)(FH / 2 - 4));
  px = FX + FW / 2 + (int)dx;
  py = FY + FH / 2 - (int)dy; // screen y grows downward; field y grows "forward"
}

// Clears the field-plot area and draws a grid every 24 in, with the two
// center axes picked out in a brighter color.
static void drawFieldFrame() {
  using namespace ui;
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(FX, FY, FX + FW, FY + FH);

  for (float v = -FIELD_HALF; v <= FIELD_HALF + 0.5f; v += 24.0f) {
    int px, py;
    toPixel(v, 0, px, py);
    pros::screen::set_pen(std::abs(v) < 0.5f ? AXIS : GRID);
    pros::screen::draw_line(px, FY, px, FY + FH);
    toPixel(0, v, px, py);
    pros::screen::set_pen(std::abs(v) < 0.5f ? AXIS : GRID);
    pros::screen::draw_line(FX, py, FX + FW, py);
  }
}

// Draws a small pink crosshair at the given field position (the target, or
// wherever it's about to be).
static void drawCrosshair(float x, float y) {
  int px, py;
  toPixel(x, y, px, py);
  pros::screen::set_pen(ui::TARGET_LINE);
  pros::screen::draw_line(px - 6, py, px + 6, py);
  pros::screen::draw_line(px, py - 6, px, py + 6);
  pros::screen::draw_circle(px, py, 3);
}

// Draws a pink arrow from the robot's current position out to a fixed
// length in the target heading direction -- the turnToHeading equivalent
// of the crosshair. Heading is compass-style: 0 = +y, increasing clockwise.
static void drawHeadingIndicator(float originX, float originY,
                                 float headingDeg) {
  static const float kLen = 24.0f; // inches, just for visualization
  float rad = lemlib::degToRad(headingDeg);
  float tipX = originX + std::sin(rad) * kLen;
  float tipY = originY + std::cos(rad) * kLen;

  int ox, oy, tx, ty;
  toPixel(originX, originY, ox, oy);
  toPixel(tipX, tipY, tx, ty);

  pros::screen::set_pen(ui::TARGET_LINE);
  pros::screen::draw_line(ox, oy, tx, ty);
  pros::screen::draw_circle(ox, oy, 3);
  pros::screen::draw_circle(tx, ty, 3);
}

static void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(lastResult.cancelled ? TARGET_LINE : GRAY);
  if (lastResult.ran) {
    if (lastResult.isTurn) {
      pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                          "%sfinal heading %.2f // error %.2f deg // %dms",
                          lastResult.cancelled ? "CANCELLED // " : "",
                          lastResult.finalHeading, lastResult.error,
                          lastResult.durationMs);
    } else {
      pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                          "%sfinal (%.2f, %.2f) // error %.2f in // %dms",
                          lastResult.cancelled ? "CANCELLED // " : "",
                          lastResult.finalX, lastResult.finalY,
                          lastResult.error, lastResult.durationMs);
    }
  } else {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                        "no run yet -- press A to go, X to cancel one");
  }
}

// Full redraw: header, target panel, field plot (with target crosshair),
// footer. Doesn't touch any in-progress path trail.
static void drawPlannerUI() {
  using namespace ui;
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);

  pros::screen::set_pen(CYAN);
  pros::screen::fill_rect(0, 0, 480, 30);
  pros::screen::set_pen(0x000000);
  bool isMove = mode == PlannerMode::MOVE_TO_POINT;
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 7, "%s%s",
                      isMove ? "MOVE TO POINT" : "TURN TO HEADING",
                      isMove ? (driveForwards ? " // FWD" : " // BWD") : "");

  const char *namesMove[3] = {"X", "Y", "SPD"};
  const char *namesTurn[2] = {"HDG", "SPD"};
  float valuesMove[3] = {targetX, targetY, targetSpeed};
  float valuesTurn[2] = {targetHeading, targetSpeed};
  const char **names = isMove ? namesMove : namesTurn;
  float *values = isMove ? valuesMove : valuesTurn;
  int fc = fieldCount();
  for (int i = 0; i < fc; i++) {
    int y = 40 + i * 42;
    bool sel = (i == selectedField);

    pros::screen::set_pen(sel ? SEL_BG : CARD);
    pros::screen::fill_rect(10, y, 150, y + 34);
    pros::screen::set_pen(sel ? ORANGE : AXIS);
    pros::screen::draw_rect(10, y, 150, y + 34);
    if (sel) {
      pros::screen::set_pen(ORANGE);
      pros::screen::fill_rect(10, y, 14, y + 34);
    }

    pros::screen::set_pen(sel ? 0xFFFFFF : GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 22, y + 3, "%s", names[i]);
    pros::screen::set_pen(sel ? CYAN : GRAY);
    pros::screen::print(pros::E_TEXT_MEDIUM, 22, y + 15, "%.2f", values[i]);
  }

  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 160, "</>:field  ^/v:adj");
  pros::screen::print(pros::E_TEXT_SMALL, 10, 172, "L1/L2:digit  step %.2f",
                      digitStep());
  pros::screen::print(pros::E_TEXT_SMALL, 10, 184, "Y:dir  R1:move/turn");
  pros::screen::print(pros::E_TEXT_SMALL, 10, 196, "B:reset pose to 0,0,0");
  pros::screen::print(pros::E_TEXT_SMALL, 10, 208, "A:go  X:cancel run");

  drawFieldFrame();
  if (isMove) {
    drawCrosshair(targetX, targetY);
  } else {
    lemlib::Pose pose = chassis.getPose();
    drawHeadingIndicator(pose.x, pose.y, targetHeading);
  }

  drawFooter();
}

// Watches for X (abort) while a motion is running, common to both modes.
// Returns true if the motion was cancelled.
static bool waitForCancel() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    chassis.cancelMotion();
    return true;
  }
  return false;
}

// ─── Run: moveToPoint ────────────────────────────────────────────────────────
// Issues ONE chassis.moveToPoint() to (targetX, targetY) at targetSpeed,
// tracing the actual path (green) on top of the target crosshair (pink) as
// it drives, and streaming CSV telemetry the same way the PID tuner does.
// Holding X at any point aborts the motion immediately
// (chassis.cancelMotion()).
static void runMoveToPoint() {
  lemlib::Pose start = chassis.getPose();
  float dist = std::hypot(targetX - start.x, targetY - start.y);
  // Longer moves get more time; clamped to a sane floor/ceiling so a tiny
  // target doesn't get an unreasonably short window and a huge one doesn't
  // wait forever if something's wrong.
  int timeoutMs = (int)std::clamp(dist * 60.0f + 800.0f, 800.0f, 6000.0f);

  drawFieldFrame();
  drawCrosshair(targetX, targetY);

  chassis.moveToPoint(targetX, targetY, timeoutMs,
                      {.forwards = driveForwards, .maxSpeed = targetSpeed},
                      true);
  // isInMotion() can briefly read false right after an async motion is
  // issued, before its task has flagged itself running (see the PID
  // tuner's runTest for the same guard).
  pros::delay(10);

  uint32_t startMs = pros::millis();
  int prevX, prevY;
  toPixel(start.x, start.y, prevX, prevY);

  bool cancelled = false;
  printf("CSV,ms,x,y,target_x,target_y,error\n");
  while (chassis.isInMotion()) {
    // X aborts the run immediately -- checked every sample (20ms) so it
    // reacts as fast as the loop itself, not just once at the top.
    if (waitForCancel()) {
      cancelled = true;
      break;
    }

    lemlib::Pose pose = chassis.getPose();
    float error = std::hypot(targetX - pose.x, targetY - pose.y);
    uint32_t t = pros::millis() - startMs;

    int px, py;
    toPixel(pose.x, pose.y, px, py);
    pros::screen::set_pen(ui::PATH_LINE);
    pros::screen::draw_line(prevX, prevY, px, py);
    prevX = px;
    prevY = py;

    printf("CSV,%lu,%.2f,%.2f,%.2f,%.2f,%.2f\n", (unsigned long)t, pose.x,
           pose.y, targetX, targetY, error);
    pros::delay(20);
  }
  chassis.waitUntilDone();

  lemlib::Pose end = chassis.getPose();
  lastResult.ran = true;
  lastResult.cancelled = cancelled;
  lastResult.isTurn = false;
  lastResult.finalX = end.x;
  lastResult.finalY = end.y;
  lastResult.error = std::hypot(targetX - end.x, targetY - end.y);
  lastResult.durationMs = (int)(pros::millis() - startMs);

  printf("RESULT %sfinal=(%.2f, %.2f) error=%.2f dur=%dms\n",
         cancelled ? "CANCELLED " : "", end.x, end.y, lastResult.error,
         lastResult.durationMs);
  controller.rumble(cancelled ? "-" : ".");
  drawFooter();
}

// ─── Run: turnToHeading ──────────────────────────────────────────────────────
// Issues ONE chassis.turnToHeading() to targetHeading at targetSpeed,
// redrawing the pink heading indicator each sample so it tracks the robot
// as it turns, and streaming CSV telemetry. Holding X aborts immediately.
static float headingError(float target, float actual) {
  float diff = std::fmod(target - actual + 180.0f, 360.0f);
  if (diff < 0)
    diff += 360.0f;
  return std::abs(diff - 180.0f);
}

static void runTurnToHeading() {
  lemlib::Pose start = chassis.getPose();
  float err0 = headingError(targetHeading, start.theta);
  // Longer turns get more time; same floor/ceiling idea as moveToPoint.
  int timeoutMs = (int)std::clamp(err0 * 20.0f + 800.0f, 800.0f, 4000.0f);

  drawFieldFrame();
  drawHeadingIndicator(start.x, start.y, targetHeading);

  chassis.turnToHeading(targetHeading, timeoutMs, {.maxSpeed = targetSpeed},
                        true);
  pros::delay(10);

  uint32_t startMs = pros::millis();
  bool cancelled = false;
  printf("CSV,ms,heading,target_heading,error\n");
  while (chassis.isInMotion()) {
    if (waitForCancel()) {
      cancelled = true;
      break;
    }

    lemlib::Pose pose = chassis.getPose();
    float error = headingError(targetHeading, pose.theta);
    uint32_t t = pros::millis() - startMs;

    drawFieldFrame();
    drawHeadingIndicator(pose.x, pose.y, pose.theta);

    printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, pose.theta,
           targetHeading, error);
    pros::delay(20);
  }
  chassis.waitUntilDone();

  lemlib::Pose end = chassis.getPose();
  lastResult.ran = true;
  lastResult.cancelled = cancelled;
  lastResult.isTurn = true;
  lastResult.finalHeading = end.theta;
  lastResult.error = headingError(targetHeading, end.theta);
  lastResult.durationMs = (int)(pros::millis() - startMs);

  printf("RESULT %sfinal_heading=%.2f error=%.2f dur=%dms\n",
         cancelled ? "CANCELLED " : "", end.theta, lastResult.error,
         lastResult.durationMs);
  controller.rumble(cancelled ? "-" : ".");
  drawFieldFrame();
  drawHeadingIndicator(end.x, end.y, targetHeading);
  drawFooter();
}

static void runPlan() {
  if (mode == PlannerMode::MOVE_TO_POINT) {
    runMoveToPoint();
  } else {
    runTurnToHeading();
  }
}

// ─── Controller screen ───────────────────────────────────────────────────────
static void drawControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120)
    return;
  lastDraw = pros::millis();

  bool isMove = mode == PlannerMode::MOVE_TO_POINT;
  const char *namesMove[3] = {"X", "Y", "SPD"};
  const char *namesTurn[2] = {"HDG", "SPD"};
  const char *fieldName =
      isMove ? namesMove[selectedField] : namesTurn[selectedField];

  switch (line) {
  case 0:
    if (isMove)
      controller.print(0, 0, "X%.2f Y%.2f S%.0f %s", targetX, targetY,
                       targetSpeed, driveForwards ? "FWD" : "BWD");
    else
      controller.print(0, 0, "HDG%.2f S%.0f       ", targetHeading,
                       targetSpeed);
    break;
  case 1:
    controller.print(1, 0, "edit:%s +/-%.2f   ", fieldName, digitStep());
    break;
  default:
    if (lastResult.cancelled)
      controller.print(2, 0, "CANCELLED err%.2f   ", lastResult.error);
    else if (lastResult.ran)
      controller.print(2, 0, "err%.2f %dms       ", lastResult.error,
                       lastResult.durationMs);
    else
      controller.print(2, 0, "A:run R1:mode       ");
    break;
  }
  line = (line + 1) % 3;
}

// ─── Button edge detection (same small helper as the PID tuner) ─────────────
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

// ─── Main planner loop ───────────────────────────────────────────────────────
void pointPlannerControl() {
  pointPlannerActive = true;
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("POINT PLANNER ACTIVE\n");
  drawPlannerUI();

  EdgeButton left{pros::E_CONTROLLER_DIGITAL_LEFT};
  EdgeButton right{pros::E_CONTROLLER_DIGITAL_RIGHT};
  EdgeButton up{pros::E_CONTROLLER_DIGITAL_UP};
  EdgeButton down{pros::E_CONTROLLER_DIGITAL_DOWN};
  EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
  EdgeButton btnB{pros::E_CONTROLLER_DIGITAL_B};
  EdgeButton btnY{pros::E_CONTROLLER_DIGITAL_Y};
  EdgeButton btnL1{pros::E_CONTROLLER_DIGITAL_L1};
  EdgeButton btnL2{pros::E_CONTROLLER_DIGITAL_L2};
  EdgeButton btnR1{pros::E_CONTROLLER_DIGITAL_R1};

  while (true) {
    bool leftEdge = left.pressed(), rightEdge = right.pressed();
    bool upEdge = up.pressed(), downEdge = down.pressed();
    bool aEdge = btnA.pressed(), bEdge = btnB.pressed(), yEdge = btnY.pressed();
    bool l1Edge = btnL1.pressed(), l2Edge = btnL2.pressed();
    bool r1Edge = btnR1.pressed();

    bool dirty = leftEdge || rightEdge || yEdge || l1Edge || l2Edge || r1Edge;

    if (r1Edge) {
      mode = mode == PlannerMode::MOVE_TO_POINT ? PlannerMode::TURN_TO_HEADING
                                                 : PlannerMode::MOVE_TO_POINT;
      selectedField = std::min(selectedField, fieldCount() - 1);
      lastResult = PlanResult{}; // units differ between modes -- drop it
    }
    // LEFT and RIGHT both just cycle through the current mode's fields.
    if (leftEdge || rightEdge)
      selectedField = (selectedField + 1) % fieldCount();
    if (yEdge)
      driveForwards = !driveForwards;
    if (l1Edge)
      digitExp = std::min(digitExp + 1, kDigitExpMax);
    if (l2Edge)
      digitExp = std::max(digitExp - 1, kDigitExpMin);

    if (upEdge || downEdge) {
      float step = downEdge ? -digitStep() : digitStep();
      *selectedFieldPtr() += step;
      targetSpeed = std::clamp(targetSpeed, 0.0f, 127.0f);
      if (mode == PlannerMode::TURN_TO_HEADING) {
        targetHeading = std::fmod(targetHeading, 360.0f);
        if (targetHeading < 0)
          targetHeading += 360.0f;
      }
      dirty = true;
    }

    if (bEdge) {
      chassis.setPose(0, 0, 0);
      dirty = true;
    }

    if (aEdge) {
      runPlan();
    } else if (dirty) {
      drawPlannerUI();
    }

    drawControllerUI();
    pros::delay(20);
  }
}
