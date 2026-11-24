// ─────────────────────────────────────────────────────────────────────────────
// PATH PLANNER -- see include/path_planner.hpp for the controls. This is the on-brain movement editor
// from the `debi` branch's driver menu (ANGULAR / LATERAL -> pick a movement -> EDIT its parameters
// -> run it), trimmed to the LemLib movements this branch has: turnToHeading, turnToPoint,
// swingToHeading, swingToPoint, moveToPoint and moveToPose. debi's laptop planner window, its plan
// lists and its robot-function steps (pins, IR sensors, wall relocalisation) are not here.
// ─────────────────────────────────────────────────────────────────────────────

#include "path_planner.hpp"
#include "driver_menu.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

bool pathPlannerActive = false;

namespace {

// ─── Palette (same as the debi menu) ─────────────────────────────────────────
namespace ui {
const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int GRAY = 0x777777;
const int GRID = 0x121217;
const int AXIS = 0x2d2d38;
const int SEL_BG = 0x1c2838;
const int FOOTER_BG = 0x18181F;
const int TARGET_LINE = 0xFF6B81; // pink crosshair / heading arrow
const int PATH_LINE = 0x2EFF8C;   // green trail (actual path)

// Field-plot rectangle, under the rounded header every screen uses.
const int FX = 160, FY = 58, FW = 310, FH = 110;
const float FIELD_HALF = 72.0f; // inches shown each side of centre

// Bottom shared by the field plot and the field list, so they can never drift apart and overlap the
// code strip below.
const int LIST_Y1 = FY + FH;

// Code-preview strip: full width, 3 lines of small text, between the plot/list and the footer.
const int CODE_Y0 = LIST_Y1 + 4, CODE_Y1 = CODE_Y0 + 40;
} // namespace ui

// ─── Small rect + drawing helpers ────────────────────────────────────────────
struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

void clearScreen() {
  pros::screen::set_pen(ui::BG);
  pros::screen::fill_rect(0, 0, 480, 240);
}

// Rounded header bar: title on the left, optional breadcrumb under it.
void drawHeader(const char *title, const char *breadcrumb, uint32_t color) {
  fillRoundedRect(5, 5, 475, 50, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, breadcrumb ? 12 : 18, "%s", title);
  if (breadcrumb) {
    pros::screen::set_pen(0x1a1a1a);
    pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "%s", breadcrumb);
  }
}

const Rect kBack = {396, 14, 468, 40};
void drawBackButton() {
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, ui::BG);
  pros::screen::set_eraser(ui::BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
}

// ─── Field-plot drawing ──────────────────────────────────────────────────────
// Inches -> pixel, uniform on both axes, centred in the field-plot rect.
void toPixel(float x, float y, int &px, int &py) {
  using namespace ui;
  float ppi = (FH / 2.0f) / FIELD_HALF;
  float dx = std::clamp(x * ppi, -(float)(FW / 2 - 4), (float)(FW / 2 - 4));
  float dy = std::clamp(y * ppi, -(float)(FH / 2 - 4), (float)(FH / 2 - 4));
  px = FX + FW / 2 + (int)dx;
  py = FY + FH / 2 - (int)dy;
}

void drawFieldFrame() {
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

void drawCrosshair(float x, float y) {
  int px, py;
  toPixel(x, y, px, py);
  pros::screen::set_pen(ui::TARGET_LINE);
  pros::screen::draw_line(px - 6, py, px + 6, py);
  pros::screen::draw_line(px, py - 6, px, py + 6);
  pros::screen::draw_circle(px, py, 3);
}

// Heading is compass-style: 0 = +y, increasing clockwise.
void drawHeadingIndicator(float originX, float originY, float headingDeg) {
  const float kLen = 24.0f;
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

// ─── Motion catalog ──────────────────────────────────────────────────────────
enum class Motion { TURN_TO_HEADING, TURN_TO_POINT, SWING_TO_HEADING, SWING_TO_POINT, MOVE_TO_POINT, MOVE_TO_POSE };
enum class ErrorKind { HEADING, FACE_POINT, POSITION, POSITION_AND_HEADING };

struct MotionInfo {
  const char *name;      // shown on the EDIT header
  const char *shortName; // shown on the controller screen (<= 16 chars)
  bool angular;          // which category it lives under
  bool hasPointTarget;   // does it have editable X/Y fields?
  bool hasHeadingTarget; // does it have an editable heading field?
  ErrorKind errorKind;
};

const MotionInfo kMotionInfo[6] = {
    {"TURN TO HEADING", "TURN HDG", true, false, true, ErrorKind::HEADING},
    {"TURN TO POINT", "TURN PT", true, true, false, ErrorKind::FACE_POINT},
    {"SWING TO HEADING", "SWING HDG", true, false, true, ErrorKind::HEADING},
    {"SWING TO POINT", "SWING PT", true, true, false, ErrorKind::FACE_POINT},
    {"MOVE TO POINT", "MOVE PT", false, true, false, ErrorKind::POSITION},
    {"MOVE TO POSE", "MOVE POSE", false, true, true, ErrorKind::POSITION_AND_HEADING},
};

// ─── Shared parameter state ──────────────────────────────────────────────────
// Persists across motions and visits -- dial in an (x, y) with moveToPoint, switch to swingToPoint and
// it is already there.
float pX = 24.0f, pY = 24.0f, pTheta = 0.0f;
float pMaxSpeed = 127.0f, pMinSpeed = 0.0f, pEarlyExitRange = 0.0f;
float pHorizontalDrift = 0.0f, pLead = 0.6f;
float pTimeoutMs = 2000.0f;
float pForwardsIdx = 0.0f;  // 0 = forwards, 1 = backwards
float pDirectionIdx = 0.0f; // 0 = AUTO, 1 = CW, 2 = CCW
float pSideIdx = 0.0f;      // 0 = LEFT, 1 = RIGHT

const char *kForwardsLabels[2] = {"FWD", "BWD"};
const char *kDirectionLabels[3] = {"AUTO", "CW", "CCW"};
const char *kSideLabels[2] = {"LEFT", "RIGHT"};

lemlib::AngularDirection directionFromIdx(int idx) {
  switch (idx) {
  case 1:
    return lemlib::AngularDirection::CW_CLOCKWISE;
  case 2:
    return lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
  default:
    return lemlib::AngularDirection::AUTO;
  }
}
lemlib::DriveSide sideFromIdx(int idx) { return idx == 1 ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT; }

// ─── One movement with every parameter it needs ──────────────────────────────
// The EDIT screen builds one of these from its p* globals; startStep() and formatStepCode() both read
// it, so the code preview and what the robot actually does cannot drift apart.
struct PlanStep {
  Motion kind = Motion::MOVE_TO_POINT;
  float x = 0, y = 0, theta = 0;
  int timeoutMs = 2000;
  bool forwards = true;
  int dirIdx = 0, sideIdx = 0;
  float maxSpeed = 127, minSpeed = 0, earlyExit = 0, lead = 0.6f, drift = 0;
};

// Issues the movement's chassis call asynchronously.
void startStep(const PlanStep &st) {
  lemlib::AngularDirection dir = directionFromIdx(st.dirIdx);
  lemlib::DriveSide side = sideFromIdx(st.sideIdx);
  switch (st.kind) {
  case Motion::TURN_TO_HEADING:
    chassis.turnToHeading(st.theta, st.timeoutMs,
                          {.direction = dir, .maxSpeed = (int)st.maxSpeed, .minSpeed = (int)st.minSpeed,
                           .earlyExitRange = st.earlyExit},
                          true);
    break;
  case Motion::TURN_TO_POINT:
    chassis.turnToPoint(st.x, st.y, st.timeoutMs,
                        {.forwards = st.forwards, .direction = dir, .maxSpeed = (int)st.maxSpeed,
                         .minSpeed = (int)st.minSpeed, .earlyExitRange = st.earlyExit},
                        true);
    break;
  case Motion::SWING_TO_HEADING:
    chassis.swingToHeading(st.theta, side, st.timeoutMs,
                           {.direction = dir, .maxSpeed = st.maxSpeed, .minSpeed = st.minSpeed,
                            .earlyExitRange = st.earlyExit},
                           true);
    break;
  case Motion::SWING_TO_POINT:
    chassis.swingToPoint(st.x, st.y, side, st.timeoutMs,
                         {.forwards = st.forwards, .direction = dir, .maxSpeed = st.maxSpeed,
                          .minSpeed = st.minSpeed, .earlyExitRange = st.earlyExit},
                         true);
    break;
  case Motion::MOVE_TO_POINT:
    chassis.moveToPoint(st.x, st.y, st.timeoutMs,
                        {.forwards = st.forwards, .maxSpeed = st.maxSpeed, .minSpeed = st.minSpeed,
                         .earlyExitRange = st.earlyExit},
                        true);
    break;
  case Motion::MOVE_TO_POSE:
    chassis.moveToPose(st.x, st.y, st.theta, st.timeoutMs,
                       {.forwards = st.forwards, .horizontalDrift = st.drift, .lead = st.lead,
                        .maxSpeed = st.maxSpeed, .minSpeed = st.minSpeed, .earlyExitRange = st.earlyExit},
                       true);
    break;
  }
}

// The exact C++ startStep() issues, as up to 3 lines of small text.
void formatStepCode(const PlanStep &st, char lines[3][80]) {
  lines[0][0] = lines[1][0] = lines[2][0] = '\0';
  const char *fwdLbl = st.forwards ? "true" : "false";
  const char *dirLbl = kDirectionLabels[std::clamp(st.dirIdx, 0, 2)];
  const char *sideLbl = kSideLabels[std::clamp(st.sideIdx, 0, 1)];

  switch (st.kind) {
  case Motion::TURN_TO_HEADING:
    snprintf(lines[0], 80, "chassis.turnToHeading(%.2f, %d,", st.theta, st.timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl, st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case Motion::TURN_TO_POINT:
    snprintf(lines[0], 80, "chassis.turnToPoint(%.2f, %.2f, %d,", st.x, st.y, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl, st.maxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", st.minSpeed, st.earlyExit);
    break;
  case Motion::SWING_TO_HEADING:
    snprintf(lines[0], 80, "chassis.swingToHeading(%.2f, %s, %d,", st.theta, sideLbl, st.timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl, st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case Motion::SWING_TO_POINT:
    snprintf(lines[0], 80, "chassis.swingToPoint(%.2f, %.2f, %s, %d,", st.x, st.y, sideLbl, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl, st.maxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", st.minSpeed, st.earlyExit);
    break;
  case Motion::MOVE_TO_POINT:
    snprintf(lines[0], 80, "chassis.moveToPoint(%.2f, %.2f, %d,", st.x, st.y, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", fwdLbl, st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case Motion::MOVE_TO_POSE:
    snprintf(lines[0], 80, "chassis.moveToPose(%.2f, %.2f, %.2f, %d,", st.x, st.y, st.theta, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .horizontalDrift=%.2f, .lead=%.2f,", fwdLbl, st.drift, st.lead);
    snprintf(lines[2], 80, "   .maxSpeed=%.0f, .minSpeed=%.0f, .earlyExitRange=%.2f});", st.maxSpeed, st.minSpeed,
             st.earlyExit);
    break;
  }
}

// ─── Editable field descriptors ──────────────────────────────────────────────
enum class FieldKind { NUMERIC, CHOICE };
struct FieldDef {
  const char *label;
  FieldKind kind;
  float *value;
  float minV, maxV;           // NUMERIC clamp range
  const char *const *choices; // CHOICE labels
  int numChoices;
};

std::vector<FieldDef> fields;
int selectedField = 0;
Motion currentMotion = Motion::MOVE_TO_POINT;

void buildFields(Motion m) {
  fields.clear();
  auto numeric = [](const char *label, float *v, float lo, float hi) {
    fields.push_back({label, FieldKind::NUMERIC, v, lo, hi, nullptr, 0});
  };
  auto choice = [](const char *label, float *v, const char *const *labels, int n) {
    fields.push_back({label, FieldKind::CHOICE, v, 0, (float)(n - 1), labels, n});
  };

  switch (m) {
  case Motion::TURN_TO_HEADING:
    numeric("HDG", &pTheta, -1e6f, 1e6f);
    choice("DIR", &pDirectionIdx, kDirectionLabels, 3);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 180);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  case Motion::TURN_TO_POINT:
    numeric("X", &pX, -1e6f, 1e6f);
    numeric("Y", &pY, -1e6f, 1e6f);
    choice("FWD", &pForwardsIdx, kForwardsLabels, 2);
    choice("DIR", &pDirectionIdx, kDirectionLabels, 3);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 72);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  case Motion::SWING_TO_HEADING:
    numeric("HDG", &pTheta, -1e6f, 1e6f);
    choice("SIDE", &pSideIdx, kSideLabels, 2);
    choice("DIR", &pDirectionIdx, kDirectionLabels, 3);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 180);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  case Motion::SWING_TO_POINT:
    numeric("X", &pX, -1e6f, 1e6f);
    numeric("Y", &pY, -1e6f, 1e6f);
    choice("SIDE", &pSideIdx, kSideLabels, 2);
    choice("FWD", &pForwardsIdx, kForwardsLabels, 2);
    choice("DIR", &pDirectionIdx, kDirectionLabels, 3);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 72);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  case Motion::MOVE_TO_POINT:
    numeric("X", &pX, -1e6f, 1e6f);
    numeric("Y", &pY, -1e6f, 1e6f);
    choice("FWD", &pForwardsIdx, kForwardsLabels, 2);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 72);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  case Motion::MOVE_TO_POSE:
    numeric("X", &pX, -1e6f, 1e6f);
    numeric("Y", &pY, -1e6f, 1e6f);
    numeric("HDG", &pTheta, -1e6f, 1e6f);
    choice("FWD", &pForwardsIdx, kForwardsLabels, 2);
    numeric("DRIFT", &pHorizontalDrift, 0, 30);
    numeric("LEAD", &pLead, 0, 1);
    numeric("MAXSPD", &pMaxSpeed, 0, 127);
    numeric("MINSPD", &pMinSpeed, 0, 127);
    numeric("EXIT", &pEarlyExitRange, 0, 72);
    numeric("TIME", &pTimeoutMs, 100, 15000);
    break;
  }
  selectedField = std::clamp(selectedField, 0, (int)fields.size() - 1);
}

void formatFieldValue(const FieldDef &f, char *buf, size_t n) {
  if (f.kind == FieldKind::CHOICE) {
    int idx = std::clamp((int)std::lround(*f.value), 0, f.numChoices - 1);
    snprintf(buf, n, "%s", f.choices[idx]);
  } else {
    snprintf(buf, n, "%.2f", *f.value);
  }
}

// Digit cursor, same idea as the PID tuner: UP/DOWN adjusts a NUMERIC field by 10^digitExp, and L1/L2
// move which digit that is.
int digitExp = 0;
const int kDigitExpMax = 2;
const int kDigitExpMin = -2;
float digitStep() { return std::pow(10.0f, (float)digitExp); }

// ─── Last run result ─────────────────────────────────────────────────────────
struct RunResult {
  bool ran = false, cancelled = false;
  ErrorKind kind = ErrorKind::POSITION;
  float finalX = 0, finalY = 0, finalHeading = 0;
  float posError = 0, headingError = 0;
  int durationMs = 0;
};
RunResult lastResult;

float headingError(float target, float actual) {
  float diff = std::fmod(target - actual + 180.0f, 360.0f);
  if (diff < 0) diff += 360.0f;
  return std::abs(diff - 180.0f);
}

// ─── Menu grid: 3 cards per row ──────────────────────────────────────────────
struct GridItem {
  const char *label;
};
const int GRID_X0 = 15, GRID_X1 = 465, GRID_Y0 = 58, GRID_ROW_H = 70, GRID_GAP = 8, GRID_COLS = 3;

Rect gridItemRect(int i) {
  int colW = (GRID_X1 - GRID_X0 - (GRID_COLS - 1) * GRID_GAP) / GRID_COLS;
  int x0 = GRID_X0 + (i % GRID_COLS) * (colW + GRID_GAP);
  int y0 = GRID_Y0 + (i / GRID_COLS) * (GRID_ROW_H + GRID_GAP);
  return {x0, y0, x0 + colW, y0 + GRID_ROW_H};
}

// Card with a cyan accent bar, the label, and a ">" hinting that it opens something.
void drawGridButton(const Rect &r, const GridItem &item) {
  using namespace ui;
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  fillRoundedRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, CYAN);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 12, r.y0 + (r.y1 - r.y0) / 2 - 6, "%s", item.label);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, r.x1 - 16, r.y0 + (r.y1 - r.y0) / 2 - 6, ">");
}

void drawGrid(const GridItem *items, int count) {
  for (int i = 0; i < count; i++) drawGridButton(gridItemRect(i), items[i]);
}

int gridHitTest(int count, int x, int y) {
  for (int i = 0; i < count; i++)
    if (inRect(gridItemRect(i), x, y)) return i;
  return -1;
}

const GridItem kPathItems[2] = {{"ANGULAR"}, {"LATERAL"}};
const GridItem kAngularItems[4] = {{"TURN HDG"}, {"TURN PT"}, {"SWING HDG"}, {"SWING PT"}};
const GridItem kLateralItems[2] = {{"MOVE PT"}, {"MOVE POSE"}};
const Motion kAngularMotions[4] = {Motion::TURN_TO_HEADING, Motion::TURN_TO_POINT, Motion::SWING_TO_HEADING,
                                   Motion::SWING_TO_POINT};
const Motion kLateralMotions[2] = {Motion::MOVE_TO_POINT, Motion::MOVE_TO_POSE};

void drawPathType() {
  clearScreen();
  drawHeader("PATH PLANNER", nullptr, ui::ORANGE);
  drawBackButton();
  drawGrid(kPathItems, 2);
}

void drawAngularList() {
  clearScreen();
  drawHeader("PATH PLANNER", "ANGULAR", ui::CYAN);
  drawBackButton();
  drawGrid(kAngularItems, 4);
}

void drawLateralList() {
  clearScreen();
  drawHeader("PATH PLANNER", "LATERAL", ui::ORANGE);
  drawBackButton();
  drawGrid(kLateralItems, 2);
}

// ─── The EDIT screen ─────────────────────────────────────────────────────────
// What this motion's target looks like on the field plot: a crosshair at (X, Y) for point motions, a
// heading arrow from the robot's current position for heading-only motions, and for moveToPose (which
// has both) a short heading tick out of the target point too.
void drawTargetIndicator() {
  const MotionInfo &info = kMotionInfo[(int)currentMotion];
  if (info.hasPointTarget) {
    drawCrosshair(pX, pY);
    if (info.hasHeadingTarget) drawHeadingIndicator(pX, pY, pTheta);
  } else if (info.hasHeadingTarget) {
    lemlib::Pose pose = chassis.getPose();
    drawHeadingIndicator(pose.x, pose.y, pTheta);
  }
}

void buildFooterText(char *buf, size_t cap) {
  if (!lastResult.ran) {
    snprintf(buf, cap, "no run yet -- press A to go, X to cancel one");
    return;
  }
  int n = 0;
  if (lastResult.cancelled) n += snprintf(buf + n, cap - n, "CANCELLED // ");
  switch (lastResult.kind) {
  case ErrorKind::HEADING:
  case ErrorKind::FACE_POINT:
    n += snprintf(buf + n, cap - n, "final hdg %.2f // err %.2f deg ", lastResult.finalHeading, lastResult.headingError);
    break;
  case ErrorKind::POSITION:
    n += snprintf(buf + n, cap - n, "final (%.2f, %.2f) // err %.2f in ", lastResult.finalX, lastResult.finalY,
                  lastResult.posError);
    break;
  case ErrorKind::POSITION_AND_HEADING:
    n += snprintf(buf + n, cap - n, "final (%.2f, %.2f) hdg %.2f // %.2fin %.2fdeg ", lastResult.finalX,
                  lastResult.finalY, lastResult.finalHeading, lastResult.posError, lastResult.headingError);
    break;
  }
  snprintf(buf + n, cap - n, "// %dms", lastResult.durationMs);
}

void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(lastResult.cancelled ? TARGET_LINE : GRAY);
  char buf[100];
  buildFooterText(buf, sizeof(buf));
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", buf);
}

// The EDIT screen's current p* parameters as one PlanStep.
PlanStep currentParamsAsStep() {
  PlanStep st;
  st.kind = currentMotion;
  st.x = pX;
  st.y = pY;
  st.theta = pTheta;
  st.timeoutMs = (int)pTimeoutMs;
  st.forwards = pForwardsIdx < 0.5f;
  st.dirIdx = std::clamp((int)std::lround(pDirectionIdx), 0, 2);
  st.sideIdx = std::clamp((int)std::lround(pSideIdx), 0, 1);
  st.maxSpeed = pMaxSpeed;
  st.minSpeed = pMinSpeed;
  st.earlyExit = pEarlyExitRange;
  st.lead = pLead;
  st.drift = pHorizontalDrift;
  return st;
}

// The real chassis.* call the current values produce, so editing a parameter shows the C++ that results.
void drawCodePreview() {
  using namespace ui;
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(10, CODE_Y0, 470, CODE_Y1);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(10, CODE_Y0, 470, CODE_Y1);

  char lines[3][80];
  formatStepCode(currentParamsAsStep(), lines);

  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(GREEN);
  for (int i = 0; i < 3; i++)
    if (lines[i][0]) pros::screen::print(pros::E_TEXT_SMALL, 16, CODE_Y0 + 3 + i * 12, "%s", lines[i]);
}

// Full redraw of the EDIT screen: header, scrollable field list, field plot with target, code, footer.
void drawEditUI() {
  using namespace ui;
  const MotionInfo &info = kMotionInfo[(int)currentMotion];

  clearScreen();
  drawHeader(info.name, info.angular ? "ANGULAR" : "LATERAL", info.angular ? CYAN : ORANGE);
  drawBackButton();

  // Scrollable field list, left column: it keeps the selected field visible since some motions have up
  // to 10 fields.
  const int listX0 = 10, listX1 = 150, listY0 = FY, listY1 = LIST_Y1;
  const int rowH = 22;
  int visibleRows = (listY1 - listY0) / rowH;
  int maxScroll = std::max(0, (int)fields.size() - visibleRows);
  int scrollIdx = std::clamp(selectedField - visibleRows / 2, 0, maxScroll);

  for (int row = 0; row < visibleRows && scrollIdx + row < (int)fields.size(); row++) {
    int i = scrollIdx + row;
    int y = listY0 + row * rowH;
    bool sel = (i == selectedField);

    pros::screen::set_pen(sel ? SEL_BG : CARD);
    pros::screen::fill_rect(listX0, y, listX1, y + rowH - 3);
    if (sel) {
      pros::screen::set_pen(ORANGE);
      pros::screen::fill_rect(listX0, y, listX0 + 4, y + rowH - 3);
    }

    pros::screen::set_eraser(sel ? SEL_BG : CARD);
    pros::screen::set_pen(sel ? 0xFFFFFF : GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, listX0 + 8, y + 3, "%s", fields[i].label);
    pros::screen::set_pen(sel ? CYAN : GRAY);
    char valBuf[16];
    formatFieldValue(fields[i], valBuf, sizeof(valBuf));
    pros::screen::print(pros::E_TEXT_SMALL, listX0 + 62, y + 3, "%s", valBuf);
  }

  drawFieldFrame();
  drawTargetIndicator();

  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, FX + FW - 160, FY + 2, "</>:field ^/v:adj");
  pros::screen::print(pros::E_TEXT_SMALL, FX + FW - 160, FY + 14, "L1/L2:digit  %.2f", digitStep());
  pros::screen::print(pros::E_TEXT_SMALL, FX + FW - 160, FY + 26, "A:go X:cancel B:reset");

  drawCodePreview();
  drawFooter();
}

// ─── Controller screen ───────────────────────────────────────────────────────
void drawEditControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120) return;
  lastDraw = pros::millis();

  const MotionInfo &info = kMotionInfo[(int)currentMotion];
  switch (line) {
  case 0:
    controller.print(0, 0, "%s          ", info.shortName);
    break;
  case 1: {
    const FieldDef &f = fields[selectedField];
    char valBuf[16];
    formatFieldValue(f, valBuf, sizeof(valBuf));
    controller.print(1, 0, "%s=%s +/-%.2f   ", f.label, valBuf, digitStep());
    break;
  }
  default:
    if (lastResult.cancelled) controller.print(2, 0, "CANCELLED           ");
    else if (lastResult.ran) controller.print(2, 0, "done %dms           ", lastResult.durationMs);
    else controller.print(2, 0, "A:run X:cancel       ");
    break;
  }
  line = (line + 1) % 3;
}

// ─── Running a motion ────────────────────────────────────────────────────────
bool waitForCancel() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    chassis.cancelMotion();
    return true;
  }
  return false;
}

void runMotion() {
  const MotionInfo &info = kMotionInfo[(int)currentMotion];

  lemlib::Pose start = chassis.getPose();
  drawFieldFrame();
  drawTargetIndicator();

  startStep(currentParamsAsStep());

  // isInMotion() can briefly read false right after an async motion is issued, before its task has
  // flagged itself running.
  pros::delay(10);

  uint32_t startMs = pros::millis();
  int prevX, prevY;
  toPixel(start.x, start.y, prevX, prevY);

  bool cancelled = false;
  printf("CSV,ms,x,y,theta\n");
  while (chassis.isInMotion()) {
    if (waitForCancel()) {
      cancelled = true;
      break;
    }

    lemlib::Pose pose = chassis.getPose();
    uint32_t t = pros::millis() - startMs;

    if (info.hasPointTarget) {
      int px, py;
      toPixel(pose.x, pose.y, px, py);
      pros::screen::set_pen(ui::PATH_LINE);
      pros::screen::draw_line(prevX, prevY, px, py);
      prevX = px;
      prevY = py;
    } else {
      drawFieldFrame();
      drawHeadingIndicator(pose.x, pose.y, pose.theta);
    }

    printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, pose.x, pose.y, pose.theta);
    pros::delay(20);
  }
  // no chassis.waitUntilDone(): the isInMotion() loop is the wait, and waitUntilDone() hangs if the
  // motion never started

  lemlib::Pose end = chassis.getPose();
  lastResult.ran = true;
  lastResult.cancelled = cancelled;
  lastResult.kind = info.errorKind;
  switch (info.errorKind) {
  case ErrorKind::HEADING:
    lastResult.finalHeading = end.theta;
    lastResult.headingError = headingError(pTheta, end.theta);
    break;
  case ErrorKind::FACE_POINT: {
    float ideal = lemlib::radToDeg(std::atan2(pX - end.x, pY - end.y));
    lastResult.finalHeading = end.theta;
    lastResult.headingError = headingError(ideal, end.theta);
    break;
  }
  case ErrorKind::POSITION:
    lastResult.finalX = end.x;
    lastResult.finalY = end.y;
    lastResult.posError = std::hypot(pX - end.x, pY - end.y);
    break;
  case ErrorKind::POSITION_AND_HEADING:
    lastResult.finalX = end.x;
    lastResult.finalY = end.y;
    lastResult.finalHeading = end.theta;
    lastResult.posError = std::hypot(pX - end.x, pY - end.y);
    lastResult.headingError = headingError(pTheta, end.theta);
    break;
  }
  lastResult.durationMs = (int)(pros::millis() - startMs);

  printf("RESULT %sfinal=(%.2f, %.2f, %.2f) dur=%dms\n", cancelled ? "CANCELLED " : "", end.x, end.y, end.theta,
         lastResult.durationMs);
  controller.rumble(cancelled ? "-" : ".");
  drawFooter();
}

// ─── Button edge detection ───────────────────────────────────────────────────
struct EdgeButton {
  pros::controller_digital_e_t id;
  bool prevHeld = false;
  bool pressed() {
    bool now = controller.get_digital(id);
    bool edge = now && !prevHeld;
    prevHeld = now;
    return edge;
  }
};

enum class Screen { PATH_TYPE, ANGULAR_LIST, LATERAL_LIST, EDIT };

} // namespace

// ─── Main loop ───────────────────────────────────────────────────────────────
void pathPlannerControl() {
  pathPlannerActive = true;
  pros::delay(100); // let the dashboard task finish its current frame
  controller.rumble("--");
  printf("PATH PLANNER ACTIVE\n");

  Screen screen = Screen::PATH_TYPE;
  drawPathType();

  auto goAngular = [&] { screen = Screen::ANGULAR_LIST; drawAngularList(); };
  auto goLateral = [&] { screen = Screen::LATERAL_LIST; drawLateralList(); };
  auto goPathType = [&] { screen = Screen::PATH_TYPE; drawPathType(); };
  auto enterEdit = [&](Motion m) {
    currentMotion = m;
    buildFields(m);
    selectedField = 0;
    lastResult = RunResult{}; // fields and units differ between motions
    screen = Screen::EDIT;
    drawEditUI();
  };

  EdgeButton left{pros::E_CONTROLLER_DIGITAL_LEFT};
  EdgeButton right{pros::E_CONTROLLER_DIGITAL_RIGHT};
  EdgeButton up{pros::E_CONTROLLER_DIGITAL_UP};
  EdgeButton down{pros::E_CONTROLLER_DIGITAL_DOWN};
  EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
  EdgeButton btnB{pros::E_CONTROLLER_DIGITAL_B};
  EdgeButton btnL1{pros::E_CONTROLLER_DIGITAL_L1};
  EdgeButton btnL2{pros::E_CONTROLLER_DIGITAL_L2};

  bool wasTouched = true; // a touch already down when this opens is not a tap
  while (true) {
    if (driverMenuLeaveRequested()) { // top-right corner held, or LEAVE from the laptop
      pathPlannerActive = false;
      controller.rumble(".");
      return;
    }

    pros::screen_touch_status_s_t t = pros::screen::touch_status();
    bool down_ = t.touch_status != pros::E_TOUCH_RELEASED;
    bool tap = down_ && !wasTouched;
    wasTouched = down_;
    int tx = t.x, ty = t.y;
    int rx, ry;
    if (takeRemoteTouch(rx, ry)) { // a click in the laptop window counts like a tap
      tap = true;
      tx = rx;
      ty = ry;
    }

    if (tap) {
      switch (screen) {
      case Screen::PATH_TYPE:
        if (inRect(kBack, tx, ty)) {
          pathPlannerActive = false; // back to the driver menu
          controller.rumble(".");
          return;
        }
        switch (gridHitTest(2, tx, ty)) {
        case 0: goAngular(); break;
        case 1: goLateral(); break;
        }
        break;
      case Screen::ANGULAR_LIST: {
        if (inRect(kBack, tx, ty)) {
          goPathType();
          break;
        }
        int hit = gridHitTest(4, tx, ty);
        if (hit >= 0) enterEdit(kAngularMotions[hit]);
        break;
      }
      case Screen::LATERAL_LIST: {
        if (inRect(kBack, tx, ty)) {
          goPathType();
          break;
        }
        int hit = gridHitTest(2, tx, ty);
        if (hit >= 0) enterEdit(kLateralMotions[hit]);
        break;
      }
      case Screen::EDIT:
        if (inRect(kBack, tx, ty)) {
          if (kMotionInfo[(int)currentMotion].angular) goAngular();
          else goLateral();
        }
        break;
      }
    }

    bool leftEdge = left.pressed(), rightEdge = right.pressed();
    bool upEdge = up.pressed(), downEdge = down.pressed();
    bool aEdge = btnA.pressed(), bEdge = btnB.pressed();
    bool l1Edge = btnL1.pressed(), l2Edge = btnL2.pressed();

    if (screen == Screen::EDIT && !fields.empty()) {
      bool dirty = leftEdge || rightEdge || l1Edge || l2Edge;
      if (leftEdge) selectedField = (selectedField + (int)fields.size() - 1) % (int)fields.size();
      if (rightEdge) selectedField = (selectedField + 1) % (int)fields.size();
      if (l1Edge) digitExp = std::min(digitExp + 1, kDigitExpMax);
      if (l2Edge) digitExp = std::max(digitExp - 1, kDigitExpMin);

      if (upEdge || downEdge) {
        FieldDef &f = fields[selectedField];
        if (f.kind == FieldKind::CHOICE) {
          int idx = std::clamp((int)std::lround(*f.value), 0, f.numChoices - 1);
          idx = (idx + (upEdge ? 1 : f.numChoices - 1)) % f.numChoices;
          *f.value = (float)idx;
        } else {
          float step = downEdge ? -digitStep() : digitStep();
          *f.value = std::clamp(*f.value + step, f.minV, f.maxV);
        }
        dirty = true;
      }

      if (bEdge) {
        chassis.setPose(0, 0, 0);
        dirty = true;
      }

      if (aEdge) runMotion();
      else if (dirty) drawEditUI();

      drawEditControllerUI();
    }

    pros::delay(20);
  }
}
