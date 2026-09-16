// ─────────────────────────────────────────────────────────────────────────────
// DRIVER MENU — see include/driver_menu.hpp for the full controls reference.
// Touchscreen home screen shown at the start of opcontrol():
//   HOME -> PID TUNING | PATH PLANNER | TEST MOTORS | DRIVE | LAPTOP DRIVE
//           | MOTOR TEMPS
//   PATH PLANNER -> ANGULAR | LATERAL
//   ANGULAR -> turnToHeading | turnToPoint | swingToHeading | swingToPoint
//   LATERAL -> moveToPoint | moveToPose
// Picking a motion opens an EDIT screen listing every parameter that
// motion's chassis.* call takes, edited with the controller (LEFT/RIGHT
// select field, UP/DOWN adjust, L1/L2 digit cursor, A run, X cancel, B
// reset pose) -- the same idea as the PID tuner / old point planner.
// TEST MOTORS spins every drivetrain motor one at a time and reports,
// per port, whether it physically spun forward (Positive) or backward
// (Negative / inverted).
// ─────────────────────────────────────────────────────────────────────────────

#include "driver_menu.hpp"
#include "pid_tuner.hpp"
#include "driver_menu.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <vector>

bool driverMenuActive = false;

// ─── Palette (matches the PID tuner / old point planner) ────────────────────
namespace ui {
static const int BG = 0x0A0A0F;
static const int CARD = 0x161B22;
static const int CYAN = 0x00F0FF;
static const int ORANGE = 0xFF8C00;
static const int GREEN = 0x2ECC71;
static const int GRAY = 0x777777;
static const int GRID = 0x121217;
static const int AXIS = 0x2d2d38;
static const int SEL_BG = 0x1c2838;
static const int FOOTER_BG = 0x18181F;
static const int TARGET_LINE = 0xFF6B81; // pink crosshair / heading arrow
static const int PATH_LINE = 0x2EFF8C;   // green trail (actual path)

// Field-plot rectangle -- same footprint as the PID tuner's graph, shifted
// down to sit under the taller rounded header used by every screen.
// Shortened from the original 152px to leave room for the code-preview
// strip (see kCodeY0/kCodeY1 below) between it and the footer.
static const int FX = 160, FY = 58, FW = 310, FH = 110;
static const float FIELD_HALF = 72.0f; // inches shown each side of center

// Bottom shared by the field-plot and the field-list column (they sit at
// the same y-range, just different x-ranges) -- kept as one constant so
// the two can never drift apart and start overlapping the strip below.
static const int LIST_Y1 = FY + FH;

// Code-preview strip: sits in the gap between the field plot/list and the
// footer, full width, tall enough for 3 lines of E_TEXT_SMALL text.
static const int CODE_Y0 = LIST_Y1 + 4, CODE_Y1 = CODE_Y0 + 40;
} // namespace ui

// ─── Small rect + button helpers ─────────────────────────────────────────────
struct Rect {
  int x0, y0, x1, y1;
};
static bool inRect(const Rect &r, int x, int y) {
  return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

static void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

static void clearScreen() {
  pros::screen::set_pen(ui::BG);
  pros::screen::fill_rect(0, 0, 480, 240);
}

// Rounded header bar shared by every screen -- title on the left, an
// optional breadcrumb line underneath, and a thin dark seam along the
// bottom edge for a bit of depth instead of a flat color block.
static void drawHeader(const char *title, const char *breadcrumb, uint32_t color) {
  using namespace ui;
  fillRoundedRect(5, 5, 475, 50, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, breadcrumb ? 12 : 18, "%s", title);
  if (breadcrumb) {
    pros::screen::set_pen(0x1a1a1a);
    pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "%s", breadcrumb);
  }
}

// BACK button, top-right corner of the header -- present on every screen
// except HOME.
static const Rect kBack = {396, 14, 468, 40};
static void drawBackButton() {
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, ui::BG);
  pros::screen::set_eraser(ui::BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
}

// ─── Field-plot drawing (shared by every EDIT screen) ────────────────────────
// Inches -> pixel, uniform on both axes, centered in the field-plot rect.
static void toPixel(float x, float y, int &px, int &py) {
  using namespace ui;
  float ppi = (FH / 2.0f) / FIELD_HALF;
  float dx = std::clamp(x * ppi, -(float)(FW / 2 - 4), (float)(FW / 2 - 4));
  float dy = std::clamp(y * ppi, -(float)(FH / 2 - 4), (float)(FH / 2 - 4));
  px = FX + FW / 2 + (int)dx;
  py = FY + FH / 2 - (int)dy;
}

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

static void drawCrosshair(float x, float y) {
  int px, py;
  toPixel(x, y, px, py);
  pros::screen::set_pen(ui::TARGET_LINE);
  pros::screen::draw_line(px - 6, py, px + 6, py);
  pros::screen::draw_line(px, py - 6, px, py + 6);
  pros::screen::draw_circle(px, py, 3);
}

// What the field plot is currently showing, tracked purely so the remote
// mirror (see "Remote touch bridge" below) can redraw the same thing:
//   0  target indicator only (drawEditUI)
//   1  target indicator + the green trail runMotion() draws as it goes
//   2  live heading arrow at the robot's pose (heading-only motions while
//      running -- drawFieldFrame() wipes the target indicator there)
// The trail is kept in brain-screen pixels, same as the lines drawn.
static int remotePlotMode = 0;
static std::vector<std::pair<int16_t, int16_t>> remoteTrail;
static lemlib::Pose remoteLivePose{0, 0, 0};

// Heading is compass-style: 0 = +y, increasing clockwise.
static void drawHeadingIndicator(float originX, float originY, float headingDeg) {
  static const float kLen = 24.0f;
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
  const char *name;      // shown on the EDIT header / menu buttons
  const char *shortName; // shown on the controller LCD (<=16 chars)
  bool angular;           // which category it lives under
  bool hasPointTarget;    // does it have editable X/Y fields?
  bool hasHeadingTarget;  // does it have an editable heading field?
  ErrorKind errorKind;
};

static const MotionInfo kMotionInfo[6] = {
    {"TURN TO HEADING", "TURN HDG", true, false, true, ErrorKind::HEADING},
    {"TURN TO POINT", "TURN PT", true, true, false, ErrorKind::FACE_POINT},
    {"SWING TO HEADING", "SWING HDG", true, false, true, ErrorKind::HEADING},
    {"SWING TO POINT", "SWING PT", true, true, false, ErrorKind::FACE_POINT},
    {"MOVE TO POINT", "MOVE PT", false, true, false, ErrorKind::POSITION},
    {"MOVE TO POSE", "MOVE POSE", false, true, true, ErrorKind::POSITION_AND_HEADING},
};

// ─── Shared parameter state ──────────────────────────────────────────────────
// Persists across motions/tool visits -- e.g. dial in an (x, y) with
// moveToPoint, then switch to swingToPoint and it's already there.
static float pX = 24.0f, pY = 24.0f, pTheta = 0.0f;
static float pMaxSpeed = 127.0f, pMinSpeed = 0.0f, pEarlyExitRange = 0.0f;
static float pHorizontalDrift = 0.0f, pLead = 0.6f;
static float pTimeoutMs = 2000.0f;
static float pForwardsIdx = 0.0f; // 0 = forwards, 1 = backwards
static float pDirectionIdx = 0.0f; // 0 = AUTO, 1 = CW, 2 = CCW
static float pSideIdx = 0.0f;      // 0 = LEFT, 1 = RIGHT

static const char *kForwardsLabels[2] = {"FWD", "BWD"};
static const char *kDirectionLabels[3] = {"AUTO", "CW", "CCW"};
static const char *kSideLabels[2] = {"LEFT", "RIGHT"};

static lemlib::AngularDirection directionFromIdx(int idx) {
  switch (idx) {
  case 1:
    return lemlib::AngularDirection::CW_CLOCKWISE;
  case 2:
    return lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
  default:
    return lemlib::AngularDirection::AUTO;
  }
}
static lemlib::DriveSide sideFromIdx(int idx) {
  return idx == 1 ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT;
}

// ─── Plan steps ──────────────────────────────────────────────────────────────
// One movement with every parameter it needs, as a plain value. The
// on-brain EDIT screen builds one of these from its p* globals; the laptop
// planner sends whole sequences of them (see "Laptop path planner" below).
// Both run through startStep()/formatStepCode() so the code preview, the
// exported code and what the robot actually does can't drift apart.
enum class StepKind {
  TURN_TO_HEADING, // == Motion::* for the first six, so the two cast freely
  TURN_TO_POINT,
  SWING_TO_HEADING,
  SWING_TO_POINT,
  MOVE_TO_POINT,
  MOVE_TO_POSE,
  WAIT,            // pros::delay(timeoutMs)
  SET_POSE,        // chassis.setPose(x, y, theta)
  WAIT_UNTIL_DONE, // chassis.waitUntilDone() -- lemlib motions are async by
                   // default, so exported code needs one after each move
};
struct PlanStep {
  StepKind kind = StepKind::MOVE_TO_POINT;
  float x = 0, y = 0, theta = 0;
  int timeoutMs = 2000; // for WAIT this is the delay itself
  bool forwards = true;
  int dirIdx = 0, sideIdx = 0;
  float maxSpeed = 127, minSpeed = 0, earlyExit = 0, lead = 0.6f, drift = 0;
};

static bool stepHasPointTarget(StepKind k) {
  return k == StepKind::TURN_TO_POINT || k == StepKind::SWING_TO_POINT ||
         k == StepKind::MOVE_TO_POINT || k == StepKind::MOVE_TO_POSE;
}

// Issues the step's chassis call asynchronously (WAIT/SET_POSE are not
// motions; callers handle those). Mirrors runMotion()'s original switch.
static void startStep(const PlanStep &st) {
  lemlib::AngularDirection dir = directionFromIdx(st.dirIdx);
  lemlib::DriveSide side = sideFromIdx(st.sideIdx);
  switch (st.kind) {
  case StepKind::TURN_TO_HEADING:
    chassis.turnToHeading(st.theta, st.timeoutMs,
                          {.direction = dir,
                           .maxSpeed = (int)st.maxSpeed,
                           .minSpeed = (int)st.minSpeed,
                           .earlyExitRange = st.earlyExit},
                          true);
    break;
  case StepKind::TURN_TO_POINT:
    chassis.turnToPoint(st.x, st.y, st.timeoutMs,
                        {.forwards = st.forwards,
                         .direction = dir,
                         .maxSpeed = (int)st.maxSpeed,
                         .minSpeed = (int)st.minSpeed,
                         .earlyExitRange = st.earlyExit},
                        true);
    break;
  case StepKind::SWING_TO_HEADING:
    chassis.swingToHeading(st.theta, side, st.timeoutMs,
                           {.direction = dir,
                            .maxSpeed = st.maxSpeed,
                            .minSpeed = st.minSpeed,
                            .earlyExitRange = st.earlyExit},
                           true);
    break;
  case StepKind::SWING_TO_POINT:
    chassis.swingToPoint(st.x, st.y, side, st.timeoutMs,
                         {.forwards = st.forwards,
                          .direction = dir,
                          .maxSpeed = st.maxSpeed,
                          .minSpeed = st.minSpeed,
                          .earlyExitRange = st.earlyExit},
                         true);
    break;
  case StepKind::MOVE_TO_POINT:
    chassis.moveToPoint(st.x, st.y, st.timeoutMs,
                        {.forwards = st.forwards,
                         .maxSpeed = st.maxSpeed,
                         .minSpeed = st.minSpeed,
                         .earlyExitRange = st.earlyExit},
                        true);
    break;
  case StepKind::MOVE_TO_POSE:
    chassis.moveToPose(st.x, st.y, st.theta, st.timeoutMs,
                       {.forwards = st.forwards,
                        .horizontalDrift = st.drift,
                        .lead = st.lead,
                        .maxSpeed = st.maxSpeed,
                        .minSpeed = st.minSpeed,
                        .earlyExitRange = st.earlyExit},
                       true);
    break;
  case StepKind::WAIT:
  case StepKind::SET_POSE:
  case StepKind::WAIT_UNTIL_DONE:
    break;
  }
}

// The exact C++ startStep() issues, as up to 3 lines of E_TEXT_SMALL.
static void formatStepCode(const PlanStep &st, char lines[3][80]) {
  lines[0][0] = lines[1][0] = lines[2][0] = '\0';
  const char *fwdLbl = st.forwards ? "true" : "false";
  const char *dirLbl = kDirectionLabels[std::clamp(st.dirIdx, 0, 2)];
  const char *sideLbl = kSideLabels[std::clamp(st.sideIdx, 0, 1)];

  switch (st.kind) {
  case StepKind::TURN_TO_HEADING:
    snprintf(lines[0], 80, "chassis.turnToHeading(%.2f, %d,", st.theta, st.timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl,
             st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case StepKind::TURN_TO_POINT:
    snprintf(lines[0], 80, "chassis.turnToPoint(%.2f, %.2f, %d,", st.x, st.y, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl,
             st.maxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", st.minSpeed,
             st.earlyExit);
    break;
  case StepKind::SWING_TO_HEADING:
    snprintf(lines[0], 80, "chassis.swingToHeading(%.2f, %s, %d,", st.theta, sideLbl,
             st.timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl,
             st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case StepKind::SWING_TO_POINT:
    snprintf(lines[0], 80, "chassis.swingToPoint(%.2f, %.2f, %s, %d,", st.x, st.y, sideLbl,
             st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl,
             st.maxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", st.minSpeed,
             st.earlyExit);
    break;
  case StepKind::MOVE_TO_POINT:
    snprintf(lines[0], 80, "chassis.moveToPoint(%.2f, %.2f, %d,", st.x, st.y, st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", fwdLbl,
             st.maxSpeed, st.minSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", st.earlyExit);
    break;
  case StepKind::MOVE_TO_POSE:
    snprintf(lines[0], 80, "chassis.moveToPose(%.2f, %.2f, %.2f, %d,", st.x, st.y, st.theta,
             st.timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .horizontalDrift=%.2f, .lead=%.2f,", fwdLbl,
             st.drift, st.lead);
    snprintf(lines[2], 80, "   .maxSpeed=%.0f, .minSpeed=%.0f, .earlyExitRange=%.2f});",
             st.maxSpeed, st.minSpeed, st.earlyExit);
    break;
  case StepKind::WAIT:
    snprintf(lines[0], 80, "pros::delay(%d);", st.timeoutMs);
    break;
  case StepKind::SET_POSE:
    snprintf(lines[0], 80, "chassis.setPose(%.2f, %.2f, %.2f);", st.x, st.y, st.theta);
    break;
  case StepKind::WAIT_UNTIL_DONE:
    snprintf(lines[0], 80, "chassis.waitUntilDone();");
    break;
  }
}

// ─── Editable field descriptors ──────────────────────────────────────────────
enum class FieldKind { NUMERIC, CHOICE };
struct FieldDef {
  const char *label;
  FieldKind kind;
  float *value;
  float minV, maxV;                // NUMERIC clamp range
  const char *const *choices; // CHOICE labels
  int numChoices;
};

static std::vector<FieldDef> fields;
static int selectedField = 0;
static Motion currentMotion = Motion::MOVE_TO_POINT;

static void buildFields(Motion m) {
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

static void formatFieldValue(const FieldDef &f, char *buf, size_t n) {
  if (f.kind == FieldKind::CHOICE) {
    int idx = std::clamp((int)std::lround(*f.value), 0, f.numChoices - 1);
    snprintf(buf, n, "%s", f.choices[idx]);
  } else {
    snprintf(buf, n, "%.2f", *f.value);
  }
}

// Digit cursor, same idea as the PID tuner: UP/DOWN adjusts a NUMERIC field
// by 10^digitExp, and L1/L2 move which digit that is.
static int digitExp = 0;
static const int kDigitExpMax = 2;
static const int kDigitExpMin = -2;
static float digitStep() { return std::pow(10.0f, (float)digitExp); }

// ─── Last run result ─────────────────────────────────────────────────────────
struct RunResult {
  bool ran = false, cancelled = false;
  ErrorKind kind = ErrorKind::POSITION;
  float finalX = 0, finalY = 0, finalHeading = 0;
  float posError = 0, headingError = 0;
  int durationMs = 0;
};
static RunResult lastResult;

static float headingError(float target, float actual) {
  float diff = std::fmod(target - actual + 180.0f, 360.0f);
  if (diff < 0)
    diff += 360.0f;
  return std::abs(diff - 180.0f);
}

// ─── Screens ──────────────────────────────────────────────────────────────────
enum class Screen {
  HOME,
  PATH_TYPE,
  ANGULAR_LIST,
  LATERAL_LIST,
  EDIT,
  MOTOR_TEST,
  PLANNER,
  REMOTE_DRIVE,
  TEMPS
};
static Screen screen = Screen::HOME;

// ─── Button grid: 3 per row, with an up/down scroll sidebar on the right
// for whenever a screen ever grows past what fits (today nothing does, but
// the mechanism is fully live, not just decorative). Every menu screen
// (HOME, PATH_TYPE, ANGULAR_LIST, LATERAL_LIST) is built from this one grid
// instead of a bespoke button layout each.
struct GridItem {
  const char *label;
};

static const int GRID_X0 = 15, GRID_X1 = 428;
static const int GRID_Y0 = 58, GRID_Y1 = 210;
static const int GRID_COLS = 3;
static const int GRID_ROW_H = 70;
static const int GRID_GAP = 8;
static const int GRID_VISIBLE_ROWS = (GRID_Y1 - GRID_Y0 + GRID_GAP) / (GRID_ROW_H + GRID_GAP);

static const Rect kScrollUp = {435, 60, 475, 130};
static const Rect kScrollDown = {435, 140, 475, 210};

static int scrollOffset = 0; // in rows; reset to 0 whenever a new screen is entered

static int gridRowCount(int count) { return (count + GRID_COLS - 1) / GRID_COLS; }
static int gridMaxScroll(int count) { return std::max(0, gridRowCount(count) - GRID_VISIBLE_ROWS); }

static Rect gridItemRect(int i) {
  int row = i / GRID_COLS - scrollOffset;
  int col = i % GRID_COLS;
  int colW = (GRID_X1 - GRID_X0 - (GRID_COLS - 1) * GRID_GAP) / GRID_COLS;
  int x0 = GRID_X0 + col * (colW + GRID_GAP);
  int y0 = GRID_Y0 + row * (GRID_ROW_H + GRID_GAP);
  return {x0, y0, x0 + colW, y0 + GRID_ROW_H};
}

// Card with a blue left accent bar, a centered title, and a ">" chevron
// hinting that it opens something -- every button uses the same blue, no
// per-category color-coding.
static void drawGridButton(const Rect &r, const GridItem &item) {
  using namespace ui;
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  fillRoundedRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, CYAN);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);

  // Small font (not MEDIUM) so long labels like "PATH PLANNER" stay clear
  // of the ">" chevron reserved in the last ~18px of the card instead of
  // running into it.
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 12, r.y0 + (r.y1 - r.y0) / 2 - 6, "%s",
                      item.label);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, r.x1 - 16, r.y0 + (r.y1 - r.y0) / 2 - 6, ">");
}

// Draws every visible row of `items` plus the scroll sidebar (dimmed when
// there's nothing that way to scroll to).
static void drawGrid(const GridItem *items, int count) {
  using namespace ui;
  scrollOffset = std::clamp(scrollOffset, 0, gridMaxScroll(count));

  for (int i = 0; i < count; i++) {
    int row = i / GRID_COLS - scrollOffset;
    if (row < 0 || row >= GRID_VISIBLE_ROWS)
      continue;
    drawGridButton(gridItemRect(i), items[i]);
  }

  bool canUp = scrollOffset > 0;
  bool canDown = scrollOffset < gridMaxScroll(count);
  fillRoundedRect(kScrollUp.x0, kScrollUp.y0, kScrollUp.x1, kScrollUp.y1, 8, CARD);
  fillRoundedRect(kScrollDown.x0, kScrollDown.y0, kScrollDown.x1, kScrollDown.y1, 8, CARD);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(canUp ? CYAN : AXIS);
  pros::screen::print(pros::E_TEXT_MEDIUM, kScrollUp.x0 + 28, kScrollUp.y0 + 28, "^");
  pros::screen::set_pen(canDown ? CYAN : AXIS);
  pros::screen::print(pros::E_TEXT_MEDIUM, kScrollDown.x0 + 28, kScrollDown.y0 + 28, "v");
}

// Hit-tests only the currently-visible buttons (off-screen, scrolled-away
// rows can't be tapped). Returns -1 if nothing was hit.
static int gridHitTest(int count, int x, int y) {
  for (int i = 0; i < count; i++) {
    int row = i / GRID_COLS - scrollOffset;
    if (row < 0 || row >= GRID_VISIBLE_ROWS)
      continue;
    if (inRect(gridItemRect(i), x, y))
      return i;
  }
  return -1;
}

// Handles a tap on either scroll arrow; returns true if it consumed the
// touch (so callers don't also hit-test the grid itself for that tap).
static bool gridHandleScrollTouch(int count, int x, int y) {
  if (inRect(kScrollUp, x, y)) {
    scrollOffset = std::max(0, scrollOffset - 1);
    return true;
  }
  if (inRect(kScrollDown, x, y)) {
    scrollOffset = std::min(gridMaxScroll(count), scrollOffset + 1);
    return true;
  }
  return false;
}

static const GridItem kHomeItems[6] = {
    {"PID TUNING"},
    {"PATH PLANNER"},
    {"TEST MOTORS"},
    {"DRIVE"},
    {"LAPTOP DRIVE"},
    {"MOTOR TEMPS"},
};
static const int kHomeItemCount = 6;

static const GridItem kPathItems[2] = {
    {"ANGULAR"},
    {"LATERAL"},
};

static const GridItem kAngularItems[4] = {
    {"TURN HDG"},
    {"TURN PT"},
    {"SWING HDG"},
    {"SWING PT"},
};

static const GridItem kLateralItems[2] = {
    {"MOVE PT"},
    {"MOVE POSE"},
};

static void drawHome() {
  clearScreen();
  drawHeader("DRIVER MENU", nullptr, ui::CYAN);
  drawGrid(kHomeItems, kHomeItemCount);
}

static void drawPathType() {
  clearScreen();
  drawHeader("PATH PLANNER", nullptr, ui::ORANGE);
  drawBackButton();
  drawGrid(kPathItems, 2);
}

static void drawAngularList() {
  clearScreen();
  drawHeader("PATH PLANNER", "ANGULAR", ui::CYAN);
  drawBackButton();
  drawGrid(kAngularItems, 4);
}

static void drawLateralList() {
  clearScreen();
  drawHeader("PATH PLANNER", "LATERAL", ui::ORANGE);
  drawBackButton();
  drawGrid(kLateralItems, 2);
}

// Draws whatever this motion's target looks like on the field plot: a
// crosshair at (X, Y) for point-based motions, a heading arrow from the
// robot's current position for heading-only motions, and (for moveToPose,
// which has both) a short heading tick out of the target point too.
static void drawTargetIndicator() {
  const MotionInfo &info = kMotionInfo[(int)currentMotion];
  if (info.hasPointTarget) {
    drawCrosshair(pX, pY);
    if (info.hasHeadingTarget)
      drawHeadingIndicator(pX, pY, pTheta);
  } else if (info.hasHeadingTarget) {
    lemlib::Pose pose = chassis.getPose();
    drawHeadingIndicator(pose.x, pose.y, pTheta);
  }
}

// Builds the footer's status line into `buf` -- pulled out of drawFooter()
// so the exact same text can also go out over the remote-touch bridge
// (see sendRemoteUiState()) instead of being re-derived by hand there.
static void buildFooterText(char *buf, size_t cap) {
  if (!lastResult.ran) {
    snprintf(buf, cap, "no run yet -- press A to go, X to cancel one");
    return;
  }
  int n = 0;
  if (lastResult.cancelled)
    n += snprintf(buf + n, cap - n, "CANCELLED // ");
  switch (lastResult.kind) {
  case ErrorKind::HEADING:
  case ErrorKind::FACE_POINT:
    n += snprintf(buf + n, cap - n, "final hdg %.2f // err %.2f deg ",
                  lastResult.finalHeading, lastResult.headingError);
    break;
  case ErrorKind::POSITION:
    n += snprintf(buf + n, cap - n, "final (%.2f, %.2f) // err %.2f in ",
                  lastResult.finalX, lastResult.finalY, lastResult.posError);
    break;
  case ErrorKind::POSITION_AND_HEADING:
    n += snprintf(buf + n, cap - n,
                  "final (%.2f, %.2f) hdg %.2f // %.2fin %.2fdeg ",
                  lastResult.finalX, lastResult.finalY, lastResult.finalHeading,
                  lastResult.posError, lastResult.headingError);
    break;
  }
  snprintf(buf + n, cap - n, "// %dms", lastResult.durationMs);
}

static void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(lastResult.cancelled ? TARGET_LINE : GRAY);
  char buf[100];
  buildFooterText(buf, sizeof(buf));
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", buf);
}

// ─── Movement code preview ────────────────────────────────────────────────
// Builds the exact chassis.* call runMotion() below will issue for the
// current motion, using the *current* field values -- so editing a
// parameter shows the real C++ that results, not just the raw number.
// Kept in its own function (rather than duplicated by hand) and mirrored
// against runMotion()'s switch so the two can't drift apart.
// The EDIT screen's current p* parameters as one PlanStep.
static PlanStep currentParamsAsStep() {
  PlanStep st;
  st.kind = (StepKind)(int)currentMotion;
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

static void formatMotionCode(char lines[3][80]) { formatStepCode(currentParamsAsStep(), lines); }

// Full-width strip between the field plot/list and the footer -- never
// overlaps either since it lives in the dedicated ui::CODE_Y0..CODE_Y1 gap.
static void drawCodePreview() {
  using namespace ui;
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(10, CODE_Y0, 470, CODE_Y1);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(10, CODE_Y0, 470, CODE_Y1);

  char lines[3][80];
  formatMotionCode(lines);

  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(GREEN);
  for (int i = 0; i < 3; i++) {
    if (lines[i][0])
      pros::screen::print(pros::E_TEXT_SMALL, 16, CODE_Y0 + 3 + i * 12, "%s", lines[i]);
  }
}

// Full redraw of the EDIT screen: header, scrollable field list, field
// plot (with target indicator), footer.
static void drawEditUI() {
  using namespace ui;
  const MotionInfo &info = kMotionInfo[(int)currentMotion];

  clearScreen();
  drawHeader(info.name, info.angular ? "ANGULAR" : "LATERAL", info.angular ? CYAN : ORANGE);
  drawBackButton();
  remotePlotMode = 0;
  remoteTrail.clear();

  // Scrollable field list, left column -- auto-scrolls to keep the
  // selected field visible since some motions have up to 9 fields.
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
  pros::screen::print(pros::E_TEXT_SMALL, FX + FW - 160, FY + 14, "L1/L2:digit  %.2f",
                      digitStep());
  pros::screen::print(pros::E_TEXT_SMALL, FX + FW - 160, FY + 26, "A:go X:cancel B:reset");

  drawCodePreview();
  drawFooter();
}

// ─── Controller screen ───────────────────────────────────────────────────────
static void drawEditControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120)
    return;
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
    if (lastResult.cancelled)
      controller.print(2, 0, "CANCELLED           ");
    else if (lastResult.ran)
      controller.print(2, 0, "done %dms           ", lastResult.durationMs);
    else
      controller.print(2, 0, "A:run X:cancel       ");
    break;
  }
  line = (line + 1) % 3;
}

// Defined in the "Remote touch bridge" section below; `force` skips the
// send-rate throttle for one-off pushes right after the screen changed.
static void sendRemoteUiState(bool force = false);

// ─── Running a motion ────────────────────────────────────────────────────────
static bool waitForCancel() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) || takeRemoteKey(RK_X)) {
    chassis.cancelMotion();
    return true;
  }
  return false;
}

static void runMotion() {
  const MotionInfo &info = kMotionInfo[(int)currentMotion];

  lemlib::Pose start = chassis.getPose();
  drawFieldFrame();
  drawTargetIndicator();
  remotePlotMode = 0;
  remoteTrail.clear();

  startStep(currentParamsAsStep());

  // isInMotion() can briefly read false right after an async motion is
  // issued, before its task has flagged itself running.
  pros::delay(10);

  uint32_t startMs = pros::millis();
  int prevX, prevY;
  toPixel(start.x, start.y, prevX, prevY);
  if (info.hasPointTarget) {
    remotePlotMode = 1;
    remoteTrail.push_back({(int16_t)prevX, (int16_t)prevY});
  }

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
      remoteTrail.push_back({(int16_t)px, (int16_t)py});
    } else {
      drawFieldFrame();
      drawHeadingIndicator(pose.x, pose.y, pose.theta);
      remotePlotMode = 2;
      remoteLivePose = pose;
    }

    printf("CSV,%lu,%.2f,%.2f,%.2f\n", (unsigned long)t, pose.x, pose.y, pose.theta);
    sendRemoteUiState(); // keep the laptop's plot moving with the brain's
    pros::delay(20);
  }
  // no chassis.waitUntilDone(): the isInMotion() loop is the wait, and
  // waitUntilDone() hangs if the motion never started (see runPlanStep)

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

  printf("RESULT %sfinal=(%.2f, %.2f, %.2f) dur=%dms\n",
         cancelled ? "CANCELLED " : "", end.x, end.y, end.theta,
         lastResult.durationMs);
  controller.rumble(cancelled ? "-" : ".");
  drawFooter();
  sendRemoteUiState(true); // footer changed; push it right away
}

// ─── Motor direction test ──────────────────────────────────────────────────
// Spins every drivetrain motor, one at a time, and reports whether it's
// currently configured as reversed ("Negative"/inverted) or not
// ("Positive") in code, alongside whether it actually moved at all.
//
// Note this deliberately reads the *configured* reversed flag (the sign on
// the port in main.cpp's MotorGroup, i.e. what is_reversed() reports) --
// not a measured spin direction. A V5 smart motor's "reversed" flag also
// flips its own telemetry to stay self-consistent (see pros/motors.hpp),
// so commanding +voltage through a motor's own handle -- reversed or not
// -- always reads back a positive get_actual_velocity(); measuring the
// spin sign that way can never actually distinguish the two and would
// always report "Positive". Reading the configured flag directly is both
// correct and simpler; the spin itself is just so the port can be
// physically/visually confirmed as it's tested.
struct MotorTestEntry {
  std::uint8_t rawPort = 0;
  bool configuredReversed = false; // true = coded as a negative port ("Negative"/inverted)
  bool tested = false;
  bool noMove = false;
};

static std::vector<MotorTestEntry> leftMotorTest, rightMotorTest;
static bool motorTestDone = false;

static std::vector<MotorTestEntry> makeMotorTestList(pros::MotorGroup &group) {
  std::vector<MotorTestEntry> list;
  for (std::int8_t port : group.get_port_all())
    list.push_back({(std::uint8_t)std::abs(port), port < 0});
  return list;
}

// One panel ("LEFT DRIVETRAIN" / "RIGHT DRIVETRAIN") listing every port in
// `results`, each on its own row so labels never collide with each other --
// `activeIdx` (-1 when this side isn't the one currently spinning)
// highlights whichever port is mid-test.
static void drawMotorTestPanel(const Rect &r, const char *title, uint32_t accent,
                               const std::vector<MotorTestEntry> &results, int activeIdx) {
  using namespace ui;
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  fillRoundedRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, accent);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);

  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, r.x0 + 14, r.y0 + 8, "%s", title);

  int rowY = r.y0 + 34;
  for (size_t i = 0; i < results.size(); i++) {
    const MotorTestEntry &e = results[i];
    char line[32];
    if (e.tested) {
      const char *state =
          e.noMove ? "No spin" : (e.configuredReversed ? "Negative" : "Positive");
      snprintf(line, sizeof(line), "Port %d - %s", (int)e.rawPort, state);
      pros::screen::set_pen(e.noMove ? ORANGE : (e.configuredReversed ? TARGET_LINE : GREEN));
    } else if ((int)i == activeIdx) {
      snprintf(line, sizeof(line), "Port %d - testing...", (int)e.rawPort);
      pros::screen::set_pen(CYAN);
    } else {
      snprintf(line, sizeof(line), "Port %d - pending", (int)e.rawPort);
      pros::screen::set_pen(GRAY);
    }
    pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 14, rowY, "%s", line);
    rowY += 22;
  }
}

static const Rect kMotorTestLeftPanel = {10, 58, 233, 210};
static const Rect kMotorTestRightPanel = {247, 58, 470, 210};

// Full redraw -- called after every single motor's result comes in so the
// screen always reflects the latest state without needing a timer/poll.
static int motorTestActiveSide = -1, motorTestActiveIdx = -1;
static void drawMotorTestScreen(int activeSide, int activeIdx) {
  using namespace ui;
  motorTestActiveSide = activeSide;
  motorTestActiveIdx = activeIdx;
  clearScreen();
  drawHeader("TEST MOTORS", "DRIVETRAIN", CYAN);
  drawBackButton();

  drawMotorTestPanel(kMotorTestLeftPanel, "LEFT DRIVETRAIN", CYAN, leftMotorTest,
                     activeSide == 0 ? activeIdx : -1);
  drawMotorTestPanel(kMotorTestRightPanel, "RIGHT DRIVETRAIN", ORANGE, rightMotorTest,
                     activeSide == 1 ? activeIdx : -1);

  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                      motorTestDone ? "done -- tap BACK to return"
                                    : "testing -- keep the drivetrain clear");
  sendRemoteUiState(true); // this screen only redraws on change, so push each one
}

// Spins one motor -- through its actual configured (possibly reversed)
// handle, so what you see it do on the robot matches the report -- just
// long enough to confirm it actually moved, then lets it coast to a stop
// before the next one starts. Positive/Negative in the report comes from
// entry.configuredReversed (set from the port's sign in code), not from
// this measurement -- see the comment above MotorTestEntry.
static void spinAndClassify(MotorTestEntry &entry) {
  std::int8_t configuredPort = entry.configuredReversed ? -(std::int8_t)entry.rawPort
                                                         : (std::int8_t)entry.rawPort;
  pros::Motor testMotor(configuredPort);
  testMotor.move_voltage(6000);
  pros::delay(300);
  double velocity = testMotor.get_actual_velocity();
  testMotor.move_voltage(0);
  pros::delay(200); // coast to a stop before the next motor spins
  entry.tested = true;
  entry.noMove = std::abs(velocity) <= 1;
}

// Runs every drivetrain motor (both groups) one by one, redrawing the
// screen between each so the result appears live instead of all at once
// at the end.
static void runMotorDirectionTest() {
  leftMotorTest = makeMotorTestList(left_motor_group);
  rightMotorTest = makeMotorTestList(right_motor_group);
  motorTestDone = false;
  drawMotorTestScreen(-1, -1);
  controller.print(0, 0, "TESTING MOTORS      ");

  for (size_t i = 0; i < leftMotorTest.size(); i++) {
    drawMotorTestScreen(0, (int)i);
    controller.print(1, 0, "L port %-3d           ", (int)leftMotorTest[i].rawPort);
    spinAndClassify(leftMotorTest[i]);
    drawMotorTestScreen(0, -1);
  }
  for (size_t i = 0; i < rightMotorTest.size(); i++) {
    drawMotorTestScreen(1, (int)i);
    controller.print(1, 0, "R port %-3d           ", (int)rightMotorTest[i].rawPort);
    spinAndClassify(rightMotorTest[i]);
    drawMotorTestScreen(1, -1);
  }

  motorTestDone = true;
  drawMotorTestScreen(-1, -1);
  controller.print(1, 0, "done                 ");
  controller.rumble(".");
}

// ─── Laptop path planner ─────────────────────────────────────────────────
// HOME -> PATH PLANNER lands here. The actual planning UI lives on the
// laptop (tools/path_planner.py, opened by tools/remote_touch.py): a big
// field map, stackable movement blocks, parameters, and code export. The
// brain just holds the block list the laptop sends it, runs it on request
// exactly the way an autonomous routine would, and shows the live plot
// here so the two screens agree. The old on-brain editor is still one tap
// away (BRAIN EDITOR) for when there's no laptop around.
static std::vector<PlanStep> plan;
static const size_t kMaxPlanSteps = 48;
enum class PlanStatus { IDLE, RUNNING, DONE, CANCELLED };
static PlanStatus planStatus = PlanStatus::IDLE;
static int planStepIdx = -1;                      // step being run / last run
static std::vector<std::pair<float, float>> planPath; // robot path in inches, for the laptop map
static const Rect kBrainEditor = {10, 140, 150, 166};

static const char *planStatusText() {
  switch (planStatus) {
  case PlanStatus::RUNNING:
    return "RUNNING";
  case PlanStatus::DONE:
    return "DONE";
  case PlanStatus::CANCELLED:
    return "CANCELLED";
  default:
    return "IDLE";
  }
}

static void drawPlannerScreen() {
  using namespace ui;
  clearScreen();
  drawHeader("PATH PLANNER", "LAPTOP", ORANGE);
  drawBackButton();

  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 2, "Plan on the laptop:");
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 14, "tools/remote_touch.py");
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 34, "%d step%s", (int)plan.size(),
                      plan.size() == 1 ? "" : "s");
  pros::screen::set_pen(planStatus == PlanStatus::CANCELLED ? TARGET_LINE
                        : planStatus == PlanStatus::RUNNING ? CYAN
                                                            : GRAY);
  if (planStepIdx >= 0)
    pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 46, "%s %d/%d", planStatusText(),
                        planStepIdx + 1, (int)plan.size());
  else
    pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 46, "%s", planStatusText());

  fillRoundedRect(kBrainEditor.x0, kBrainEditor.y0, kBrainEditor.x1, kBrainEditor.y1, 6, CARD);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, kBrainEditor.x0 + 10, kBrainEditor.y0 + 7,
                      "BRAIN EDITOR >");

  drawFieldFrame();
  // Trail so far, then the robot's live pose on top.
  pros::screen::set_pen(PATH_LINE);
  for (size_t i = 1; i < remoteTrail.size(); i++)
    pros::screen::draw_line(remoteTrail[i - 1].first, remoteTrail[i - 1].second,
                            remoteTrail[i].first, remoteTrail[i].second);
  lemlib::Pose pose = chassis.getPose();
  drawHeadingIndicator(pose.x, pose.y, pose.theta);
  remotePlotMode = 3;
  remoteLivePose = pose;

  // Current / last step's code in the strip, footer with the pose.
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(10, CODE_Y0, 470, CODE_Y1);
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(10, CODE_Y0, 470, CODE_Y1);
  if (planStepIdx >= 0 && planStepIdx < (int)plan.size()) {
    char lines[3][80];
    formatStepCode(plan[planStepIdx], lines);
    pros::screen::set_eraser(CARD);
    pros::screen::set_pen(GREEN);
    for (int i = 0; i < 3; i++)
      if (lines[i][0])
        pros::screen::print(pros::E_TEXT_SMALL, 16, CODE_Y0 + 3 + i * 12, "%s", lines[i]);
  }
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "pose (%.1f, %.1f) hdg %.1f   A:run all  X:stop",
                      pose.x, pose.y, pose.theta);
}

// Runs one step to completion, drawing the plot live. Returns false if
// the run was cancelled (controller X or the laptop's stop).
static bool runPlanStep(const PlanStep &st) {
  if (st.kind == StepKind::WAIT_UNTIL_DONE) {
    // The runner already waits for each motion, so normally this returns
    // at once; it exists so the plan mirrors the exported code one-to-one.
    // Deliberately NOT chassis.waitUntilDone(): that spins until
    // distTraveled == -1, which lemlib only sets when a motion *ends* --
    // before any motion has run it stays at its initial 0 and the call
    // never returns, which left the plan stuck in RUNNING forever.
    while (chassis.isInMotion()) {
      if (waitForCancel())
        return false;
      sendRemoteUiState();
      pros::delay(10);
    }
    return true;
  }
  if (st.kind == StepKind::SET_POSE) {
    chassis.setPose(st.x, st.y, st.theta);
    remoteTrail.clear();
    planPath.clear();
    drawPlannerScreen();
    return true;
  }
  if (st.kind == StepKind::WAIT) {
    uint32_t until = pros::millis() + st.timeoutMs;
    while (pros::millis() < until) {
      if (waitForCancel())
        return false;
      sendRemoteUiState();
      pros::delay(20);
    }
    return true;
  }

  lemlib::Pose start = chassis.getPose();
  int prevX, prevY;
  toPixel(start.x, start.y, prevX, prevY);
  if (remoteTrail.empty())
    remoteTrail.push_back({(int16_t)prevX, (int16_t)prevY});
  if (planPath.empty())
    planPath.push_back({start.x, start.y});

  startStep(st);
  pros::delay(10);
  while (chassis.isInMotion()) {
    if (waitForCancel())
      return false;
    lemlib::Pose pose = chassis.getPose();
    int px, py;
    toPixel(pose.x, pose.y, px, py);
    pros::screen::set_pen(ui::PATH_LINE);
    pros::screen::draw_line(prevX, prevY, px, py);
    prevX = px;
    prevY = py;
    remoteTrail.push_back({(int16_t)px, (int16_t)py});
    planPath.push_back({pose.x, pose.y});
    remoteLivePose = pose;
    sendRemoteUiState();
    pros::delay(20);
  }
  // (no chassis.waitUntilDone() here -- see WAIT_UNTIL_DONE above; the
  // isInMotion() loop above is the wait)
  return true;
}

// Runs the whole plan (only < 0) or just step `only`.
static void runPlan(int only) {
  if (plan.empty())
    return;
  int from = only < 0 ? 0 : std::clamp(only, 0, (int)plan.size() - 1);
  int to = only < 0 ? (int)plan.size() - 1 : from;
  if (only < 0) {
    remoteTrail.clear();
    planPath.clear();
  }
  planStatus = PlanStatus::RUNNING;
  takeRemoteKey(RK_X); // don't let a stale stop cancel the first step
  printf("PLAN START %d..%d\n", from, to);
  for (int i = from; i <= to; i++) {
    planStepIdx = i;
    drawPlannerScreen();
    sendRemoteUiState(true);
    if (!runPlanStep(plan[i])) {
      planStatus = PlanStatus::CANCELLED;
      controller.rumble("-");
      drawPlannerScreen();
      sendRemoteUiState(true);
      printf("PLAN CANCELLED at %d\n", i);
      return;
    }
  }
  planStatus = PlanStatus::DONE;
  controller.rumble(".");
  drawPlannerScreen();
  sendRemoteUiState(true);
  printf("PLAN DONE\n");
}

// ─── Motor temperatures ──────────────────────────────────────────────────────
// HOME -> MOTOR TEMPS. The same tiles the driving HUD in main.cpp shows
// (card, status dot, temperature, heat gauge, coloured by how hot), for
// every motor on the robot, laid out in the same sections. The HUD only
// draws while the driver menu is closed, so this is how you see the
// temperatures without leaving the menu.
static const int kTempRowH = 66;   // one section: its label plus a row of tiles
static const int kTempTileH = 46;
static const int kTempSections = 5;
static int tempsScroll = 0;        // pixels, <= 0 (content scrolled up)

struct TempSection {
  const char *label;
  uint32_t color;
  const char *prefix;
  int tilesPerRow;
};

// The five groups, in the order the HUD lists them.
static void collectTemps(std::vector<double> temps[kTempSections],
                         std::vector<std::int8_t> ports[kTempSections]) {
  temps[0] = left_motor_group.get_temperature_all();
  ports[0] = left_motor_group.get_port_all();
  temps[1] = right_motor_group.get_temperature_all();
  ports[1] = right_motor_group.get_port_all();
  temps[2] = {intake1.get_temperature(), intake2.get_temperature()};
  ports[2] = {intake1.get_port(), intake2.get_port()};
  temps[3] = {lift1.get_temperature(), lift2.get_temperature()};
  ports[3] = {lift1.get_port(), lift2.get_port()};
  temps[4] = {bunchy.get_temperature(), bunchArm.get_temperature()};
  ports[4] = {bunchy.get_port(), bunchArm.get_port()};
}

static const TempSection kTempSectionInfo[kTempSections] = {
    {"LEFT DRIVETRAIN", ui::CYAN, "L", 5},
    {"RIGHT DRIVETRAIN", ui::ORANGE, "R", 5},
    {"INTAKE", ui::GREEN, "Intake", 2},
    {"LIFT", ui::CYAN, "Lift", 2},
    {"BUNCH", ui::ORANGE, "Bunch", 2},
};

// Cyan up to 45C, orange to 55C, red past it -- the HUD's thresholds.
static uint32_t tempColor(double t) {
  return t < 45 ? ui::CYAN : (t < 55 ? ui::ORANGE : 0xFF4D4D);
}

static void drawTempTile(int x, int y, int tileW, const char *label, double temp) {
  using namespace ui;
  if (y + kTempTileH < 55 || y > 215) // outside the scrolling content area
    return;
  uint32_t col = tempColor(temp);
  fillRoundedRect(x, y, x + tileW, y + kTempTileH, 5, CARD);
  pros::screen::set_pen(col);
  pros::screen::fill_circle(x + tileW - 9, y + 9, 4);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, x + 8, y + 5, "%s", label);
  pros::screen::set_pen(col);
  pros::screen::print(pros::E_TEXT_SMALL, x + 8, y + 20, "%.1fC", temp);

  int barX = x + 8, barY = y + kTempTileH - 9, barW = tileW - 16, barH = 4;
  double frac = (temp - 20.0) / 50.0; // 20C..70C
  frac = std::clamp(frac, 0.0, 1.0);
  fillRoundedRect(barX, barY, barX + barW, barY + barH, 2, AXIS);
  int fillW = (int)(barW * frac);
  if (fillW >= 4)
    fillRoundedRect(barX, barY, barX + fillW, barY + barH, 2, col);
}

static int tempsMinScroll() {
  int content = kTempSections * kTempRowH;
  return content > 160 ? -(content - 160) : 0;
}

static void drawTempsScreen() {
  using namespace ui;
  clearScreen();
  drawHeader("MOTOR TEMPS", nullptr, CYAN);
  drawBackButton();

  std::vector<double> temps[kTempSections];
  std::vector<std::int8_t> ports[kTempSections];
  collectTemps(temps, ports);

  const int rowLeft = 15, rowRight = 428, labelH = 14, gap = 6;
  double hottest = 0;
  for (int sIdx = 0; sIdx < kTempSections; sIdx++) {
    const TempSection &info = kTempSectionInfo[sIdx];
    int labelY = 55 + tempsScroll + sIdx * kTempRowH;
    int tileY = labelY + labelH;
    for (double t : temps[sIdx])
      hottest = std::max(hottest, t);
    if (tileY + kTempTileH < 55 || labelY > 215)
      continue;
    if (labelY >= 50 && labelY + labelH <= 215) {
      pros::screen::set_eraser(BG);
      pros::screen::set_pen(info.color);
      pros::screen::print(pros::E_TEXT_SMALL, rowLeft, labelY, "%s", info.label);
    }
    int tileW = (rowRight - rowLeft - (info.tilesPerRow - 1) * gap) / info.tilesPerRow;
    for (int i = 0; i < (int)temps[sIdx].size() && i < info.tilesPerRow; i++) {
      char label[24];
      snprintf(label, sizeof(label), "%s %d P%d", info.prefix, i + 1, (int)std::abs(ports[sIdx][i]));
      drawTempTile(rowLeft + i * (tileW + gap), tileY, tileW, label, temps[sIdx][i]);
    }
  }

  // Scroll arrows in the same rects (and the same look) the grid screens use.
  if (tempsMinScroll() < 0) {
    bool canUp = tempsScroll < 0, canDown = tempsScroll > tempsMinScroll();
    fillRoundedRect(kScrollUp.x0, kScrollUp.y0, kScrollUp.x1, kScrollUp.y1, 8, CARD);
    fillRoundedRect(kScrollDown.x0, kScrollDown.y0, kScrollDown.x1, kScrollDown.y1, 8, CARD);
    pros::screen::set_eraser(CARD);
    pros::screen::set_pen(canUp ? CYAN : AXIS);
    pros::screen::print(pros::E_TEXT_MEDIUM, kScrollUp.x0 + 28, kScrollUp.y0 + 28, "^");
    pros::screen::set_pen(canDown ? CYAN : AXIS);
    pros::screen::print(pros::E_TEXT_MEDIUM, kScrollDown.x0 + 28, kScrollDown.y0 + 28, "v");
  }

  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(tempColor(hottest));
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "hottest %.1fC", hottest);
}

// ─── Remote touch bridge ──────────────────────────────────────────────────
// Lets a laptop mirror this menu over the same USB cable already used by
// `pros terminal` and "tap" it remotely -- see tools/remote_touch.py for
// the companion app. Two directions, both riding on stdout/stdin:
//   robot -> laptop   a `pros terminal`-safe line describing every
//                      touchable button on the current screen (see
//                      sendRemoteUiState()), sent a few times a second.
//   laptop -> robot    a "TOUCH <x> <y>" line, read here and turned into
//                      a synthetic touch-down the menu loop handles
//                      exactly like a real one.
// Only touchscreen taps are bridged -- EDIT/MOTOR_TEST field editing still
// needs the physical controller, same as it always has.

// Reading stdin blocks (there's no documented non-blocking mode for it),
// so it lives in its own low-priority task rather than the menu loop --
// blocking there can't stall drawing or button handling.
// Everything the laptop can send, all guarded by one mutex:
//   TOUCH <x> <y>        synthetic screen tap
//   KEY <name>           one press of a controller button, by name
//                        (LEFT RIGHT UP DOWN A B X L1 L2) -- handled on
//                        the next loop tick exactly like a real edge
//   SEL <index>          select EDIT field #index outright
//   SET <index> <value>  type a value straight into EDIT field #index
//                        (numeric, clamped to the field's range; for a
//                        CHOICE field either the choice index or its label)
//   RATE <ms>            how often to send the RUI state line. The laptop
//                        asks for a slower rate when it's talking through
//                        the controller's radio link instead of a cable.
//   PLAN CLEAR           drop the block list
//   PLAN ADD k x y th t fwd dir side max min exit lead drift
//                        append one step (k = StepKind index)
//   PLAN RUN [i]         run the whole plan, or just step i
//   PLAN STOP            cancel a running plan (same as controller X)
//   POSE x y th          chassis.setPose(x, y, th) right now
//   PID SET m g v        PID tuner: set gain g (0 kP, 1 kI, 2 kD) of
//                        controller m (0 angular, 1 lateral) to v
//   PID MODE m           PID tuner: which controller UP/DOWN edit
//   PID SEL g            PID tuner: which gain UP/DOWN edit
//   PID DIGIT e          PID tuner: digit cursor, as a power of ten (-3..2)
//   PID TEST m i         PID tuner: run single test i (0 small, 1 big,
//                        2 return) on controller m
//   PID SWEEP m          PID tuner: run the 4-leg sweep on controller m
//   PID STOP             PID tuner: cancel the motion in progress
//   The PID ones are consumed by pidTunerControl() (see pid_tuner.cpp)
//   through the take*() hooks declared in driver_menu.hpp.
struct RemoteTouch {
  bool pending = false;
  int x = 0, y = 0;
};
static const char *const kRemoteKeyNames[RK_COUNT] = {"LEFT", "RIGHT", "UP", "DOWN", "A",  "B",
                                                       "X",    "L1",    "L2", "Y",    "R1", "R2"};
struct RemoteInput {
  RemoteTouch touch;
  bool key[RK_COUNT] = {};
  int selIndex = -1;          // -1 = none pending
  int setIndex = -1;          // -1 = none pending
  char setValue[24] = "";
  int sendIntervalMs = 150;   // RUI line period (see sendRemoteUiState)
  bool planClear = false;
  std::vector<PlanStep> planAdds;
  int planRun = -2;           // -2 = none pending, -1 = all, >= 0 = that step
  bool poseSet = false;
  float poseX = 0, poseY = 0, poseTheta = 0;
  int driveL = 0, driveR = 0;     // laptop WASD driving, see applyRemoteDrive()
  uint32_t driveAt = 0;           // when that command arrived (pros::millis)
  bool driveActive = false;
  std::vector<RemotePidCommand> pidCmds;   // PID tuner commands, oldest first
  bool pidStop = false;
};
static RemoteInput remoteInput;
static pros::Mutex remoteTouchMutex;

static void remoteTouchListenerTask(void *) {
  char line[160];
  while (true) {
    if (!fgets(line, sizeof(line), stdin))
      continue;
    int x, y, idx, dl, dr;
    char name[16], value[24];
    float fx, fy, ft;
    if (strncmp(line, "PLAN ", 5) == 0) {
      const char *rest = line + 5;
      PlanStep st;
      int k, fwd;
      if (strncmp(rest, "CLEAR", 5) == 0) {
        remoteTouchMutex.take();
        remoteInput.planClear = true;
        remoteInput.planAdds.clear();
        remoteTouchMutex.give();
      } else if (sscanf(rest, "ADD %d %f %f %f %d %d %d %d %f %f %f %f %f", &k, &st.x, &st.y,
                        &st.theta, &st.timeoutMs, &fwd, &st.dirIdx, &st.sideIdx, &st.maxSpeed,
                        &st.minSpeed, &st.earlyExit, &st.lead, &st.drift) == 13) {
        st.kind = (StepKind)std::clamp(k, 0, (int)StepKind::WAIT_UNTIL_DONE);
        st.forwards = fwd != 0;
        remoteTouchMutex.take();
        if (remoteInput.planAdds.size() < kMaxPlanSteps)
          remoteInput.planAdds.push_back(st);
        remoteTouchMutex.give();
      } else if (strncmp(rest, "RUN", 3) == 0) {
        int only = -1;
        sscanf(rest, "RUN %d", &only);
        remoteTouchMutex.take();
        remoteInput.planRun = only;
        remoteTouchMutex.give();
      } else if (strncmp(rest, "STOP", 4) == 0) {
        remoteTouchMutex.take();
        remoteInput.key[RK_X] = true;
        remoteTouchMutex.give();
      }
    } else if (strncmp(line, "PID ", 4) == 0) {
      const char *rest = line + 4;
      RemotePidCommand c;
      bool ok = true;
      if (strncmp(rest, "STOP", 4) == 0) {
        remoteTouchMutex.take();
        remoteInput.pidStop = true;
        remoteTouchMutex.give();
        continue;
      } else if (sscanf(rest, "SET %d %d %f", &c.mode, &c.index, &c.value) == 3) {
        c.kind = RemotePidCommand::SET;
      } else if (sscanf(rest, "MODE %d", &c.mode) == 1) {
        c.kind = RemotePidCommand::MODE;
      } else if (sscanf(rest, "SEL %d", &c.index) == 1) {
        c.kind = RemotePidCommand::SEL;
      } else if (sscanf(rest, "DIGIT %d", &c.index) == 1) {
        c.kind = RemotePidCommand::DIGIT;
      } else if (sscanf(rest, "TEST %d %d", &c.mode, &c.index) == 2) {
        c.kind = RemotePidCommand::TEST;
      } else if (sscanf(rest, "SWEEP %d", &c.mode) == 1) {
        c.kind = RemotePidCommand::SWEEP;
      } else {
        ok = false;
      }
      if (ok) {
        remoteTouchMutex.take();
        if (remoteInput.pidCmds.size() < 32)
          remoteInput.pidCmds.push_back(c);
        remoteTouchMutex.give();
      }
    } else if (sscanf(line, "POSE %f %f %f", &fx, &fy, &ft) == 3) {
      remoteTouchMutex.take();
      remoteInput.poseSet = true;
      remoteInput.poseX = fx;
      remoteInput.poseY = fy;
      remoteInput.poseTheta = ft;
      remoteTouchMutex.give();
    } else if (sscanf(line, "DRIVE %d %d", &dl, &dr) == 2) {
      remoteTouchMutex.take();
      remoteInput.driveL = std::clamp(dl, -127, 127);
      remoteInput.driveR = std::clamp(dr, -127, 127);
      remoteInput.driveAt = pros::millis();
      remoteInput.driveActive = true;
      remoteTouchMutex.give();
    } else if (sscanf(line, "TOUCH %d %d", &x, &y) == 2) {
      remoteTouchMutex.take();
      remoteInput.touch = {true, x, y};
      remoteTouchMutex.give();
    } else if (sscanf(line, "KEY %15s", name) == 1) {
      for (int k = 0; k < RK_COUNT; k++) {
        if (strcmp(name, kRemoteKeyNames[k]) == 0) {
          remoteTouchMutex.take();
          remoteInput.key[k] = true;
          remoteTouchMutex.give();
        }
      }
    } else if (sscanf(line, "SET %d %23s", &idx, value) == 2) {
      remoteTouchMutex.take();
      remoteInput.setIndex = idx;
      snprintf(remoteInput.setValue, sizeof(remoteInput.setValue), "%s", value);
      remoteTouchMutex.give();
    } else if (sscanf(line, "SEL %d", &idx) == 1) {
      remoteTouchMutex.take();
      remoteInput.selIndex = idx;
      remoteTouchMutex.give();
    } else if (sscanf(line, "RATE %d", &idx) == 1) {
      remoteTouchMutex.take();
      remoteInput.sendIntervalMs = std::clamp(idx, 50, 2000);
      remoteTouchMutex.give();
    }
  }
}

// Starts the listener the first time the menu is opened -- it then just
// keeps running for the rest of the program, same as the HUD task.
static void ensureRemoteTouchListener() {
  static bool started = false;
  if (!started) {
    started = true;
    pros::Task(remoteTouchListenerTask, nullptr, "remote touch listener");
  }
}

// Pulls the latest pending remote touch (if any), clearing it -- shaped
// like a physical touch-down edge so the caller can treat the two
// identically.
bool takeRemoteTouch(int &x, int &y) {
  bool got = false;
  remoteTouchMutex.take();
  if (remoteInput.touch.pending) {
    got = true;
    x = remoteInput.touch.x;
    y = remoteInput.touch.y;
    remoteInput.touch.pending = false;
  }
  remoteTouchMutex.give();
  return got;
}

// One-shot read of a remote key press -- true once per KEY line received,
// so it composes with EdgeButton::pressed() as `edge || takeRemoteKey(k)`.
bool takeRemoteKey(RemoteKey k) {
  remoteTouchMutex.take();
  bool got = remoteInput.key[k];
  remoteInput.key[k] = false;
  remoteTouchMutex.give();
  return got;
}

int remoteSendIntervalMs() {
  remoteTouchMutex.take();
  int ms = remoteInput.sendIntervalMs;
  remoteTouchMutex.give();
  return ms;
}

// Oldest queued "PID ..." command, if any (see pid_tuner.cpp).
bool takeRemotePidCommand(RemotePidCommand &out) {
  bool got = false;
  remoteTouchMutex.take();
  if (!remoteInput.pidCmds.empty()) {
    out = remoteInput.pidCmds.front();
    remoteInput.pidCmds.erase(remoteInput.pidCmds.begin());
    got = true;
  }
  remoteTouchMutex.give();
  return got;
}

bool takeRemotePidStop() {
  remoteTouchMutex.take();
  bool got = remoteInput.pidStop;
  remoteInput.pidStop = false;
  remoteTouchMutex.give();
  return got;
}

// Applies any pending SEL/SET to the EDIT fields. Returns true if the
// screen needs a redraw. Out-of-range indexes are ignored; values are
// clamped exactly as UP/DOWN nudging clamps them.
static bool applyRemoteFieldEdits() {
  int selIdx, setIdx;
  char value[24];
  remoteTouchMutex.take();
  selIdx = remoteInput.selIndex;
  setIdx = remoteInput.setIndex;
  snprintf(value, sizeof(value), "%s", remoteInput.setValue);
  remoteInput.selIndex = remoteInput.setIndex = -1;
  remoteTouchMutex.give();

  bool dirty = false;
  if (selIdx >= 0 && selIdx < (int)fields.size()) {
    selectedField = selIdx;
    dirty = true;
  }
  if (setIdx >= 0 && setIdx < (int)fields.size()) {
    FieldDef &f = fields[setIdx];
    if (f.kind == FieldKind::CHOICE) {
      int idx = -1;
      for (int i = 0; i < f.numChoices; i++)
        if (strcasecmp(value, f.choices[i]) == 0)
          idx = i;
      if (idx < 0 && sscanf(value, "%d", &idx) != 1)
        idx = -1;
      if (idx >= 0 && idx < f.numChoices) {
        *f.value = (float)idx;
        dirty = true;
      }
    } else {
      float v;
      if (sscanf(value, "%f", &v) == 1) {
        *f.value = std::clamp(v, f.minV, f.maxV);
        dirty = true;
      }
    }
    selectedField = setIdx;
  }
  return dirty;
}

// Laptop driving: tools/path_planner.py's "Drive (WASD)" sends
// "DRIVE <left> <right>" (-127..127) while keys are held, and repeats the
// command as a keepalive.
//
// The command deliberately EXPIRES. A radio link drops lines, the laptop
// window can lose focus with a key still down, the Python side can be
// killed outright -- and none of those must leave the drivetrain powered.
// So a command is only obeyed while it's fresh, and the moment it goes
// stale the motors are stopped once, here, on the robot.
static constexpr uint32_t kRemoteDriveTimeoutMs = 300;
static bool remoteDriving = false;

static void applyRemoteDrive() {
  int l, r;
  uint32_t at;
  bool active;
  remoteTouchMutex.take();
  l = remoteInput.driveL;
  r = remoteInput.driveR;
  at = remoteInput.driveAt;
  active = remoteInput.driveActive;
  remoteTouchMutex.give();

  // Only the two screens that mean it: LAPTOP DRIVE, and the planner
  // (whose own Drive button sends the same command). Anywhere else a
  // stray DRIVE line is ignored rather than surprising anyone.
  bool armed = screen == Screen::REMOTE_DRIVE || screen == Screen::PLANNER;
  bool fresh = armed && active && (pros::millis() - at) <= kRemoteDriveTimeoutMs;
  if (fresh) {
    left_motor_group.move(l);
    right_motor_group.move(r);
    remoteDriving = true;
  } else if (remoteDriving) {
    left_motor_group.move(0);
    right_motor_group.move(0);
    remoteDriving = false;
    remoteTouchMutex.take();
    remoteInput.driveActive = false;
    remoteTouchMutex.give();
  }
}

// Stops a laptop-driven robot and forgets the command, for the paths that
// leave this menu (a motion is about to run, or we're handing control back
// to the driver).
static void stopRemoteDrive() {
  remoteTouchMutex.take();
  remoteInput.driveActive = false;
  remoteInput.driveL = remoteInput.driveR = 0;
  remoteTouchMutex.give();
  if (remoteDriving) {
    left_motor_group.move(0);
    right_motor_group.move(0);
    remoteDriving = false;
  }
}

// HOME -> LAPTOP DRIVE. Driving the robot from the laptop's keyboard:
// W / A / S / D in tools/remote_touch.py's window (or the path planner's)
// become "DRIVE left right" lines, which applyRemoteDrive() puts on the
// motors for as long as they keep arriving.
static int drawnDriveL = 0, drawnDriveR = 0;
static bool drawnDriveLive = false;

// Just the numbers, redrawn as they change -- repainting the whole screen
// at the loop's rate would flicker.
static void drawRemoteDriveStatus() {
  using namespace ui;
  int l, r;
  remoteTouchMutex.take();
  l = remoteInput.driveL;
  r = remoteInput.driveR;
  remoteTouchMutex.give();
  if (!remoteDriving)
    l = r = 0;

  pros::screen::set_pen(BG);
  pros::screen::fill_rect(10, FY + 60, 470, FY + 104);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(remoteDriving ? CYAN : GRAY);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, FY + 62, "L %4d   R %4d", l, r);
  pros::screen::set_pen(remoteDriving ? GREEN : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 90,
                      remoteDriving ? "driving" : "waiting for the laptop");

  lemlib::Pose pose = chassis.getPose();
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "pose (%.1f, %.1f) hdg %.1f", pose.x, pose.y,
                      pose.theta);
  drawnDriveL = l;
  drawnDriveR = r;
  drawnDriveLive = remoteDriving;
}

static void drawRemoteDriveScreen() {
  using namespace ui;
  clearScreen();
  drawHeader("LAPTOP DRIVE", "W A S D", CYAN);
  drawBackButton();

  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 2, "Drive from the laptop's keyboard:");
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 16, "W forward   S back   A left   D right");
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 30, "in tools/remote_touch.py's window");
  pros::screen::print(pros::E_TEXT_SMALL, 10, FY + 44, "The robot stops the moment you let go.");

  drawnDriveL = drawnDriveR = -1; // force the status to paint
  drawRemoteDriveStatus();
}

// Applies pending PLAN */POSE commands. Returns true if a run was
// requested (the caller decides whether/where to run it).
static bool applyRemotePlanCommands(int &runWhich) {
  bool clear, poseSet;
  float px, py, pt;
  std::vector<PlanStep> adds;
  remoteTouchMutex.take();
  clear = remoteInput.planClear;
  adds.swap(remoteInput.planAdds);
  runWhich = remoteInput.planRun;
  poseSet = remoteInput.poseSet;
  px = remoteInput.poseX;
  py = remoteInput.poseY;
  pt = remoteInput.poseTheta;
  remoteInput.planClear = false;
  remoteInput.planRun = -2;
  remoteInput.poseSet = false;
  remoteTouchMutex.give();

  bool changed = false;
  if (clear) {
    plan.clear();
    planStepIdx = -1;
    planStatus = PlanStatus::IDLE;
    changed = true;
  }
  for (const PlanStep &st : adds) {
    if (plan.size() < kMaxPlanSteps)
      plan.push_back(st);
    changed = true;
  }
  if (poseSet) {
    chassis.setPose(px, py, pt);
    remoteTrail.clear();
    planPath.clear();
    changed = true;
  }
  if (changed && screen == Screen::PLANNER)
    drawPlannerScreen();
  return runWhich != -2;
}

// Appends one "x0,y0,x1,y1,Label;" element to `buf` (bounds-checked via
// snprintf's return value the same way the rest of this file builds
// strings), for every visible button of a GridItem screen.
static void appendGridElements(char *buf, int &n, size_t cap, const GridItem *items, int count) {
  for (int i = 0; i < count; i++) {
    int row = i / GRID_COLS - scrollOffset;
    if (row < 0 || row >= GRID_VISIBLE_ROWS)
      continue;
    Rect r = gridItemRect(i);
    n += snprintf(buf + n, cap - n, "%d,%d,%d,%d,%s;", r.x0, r.y0, r.x1, r.y1, items[i].label);
  }
  if (gridMaxScroll(count) > 0) {
    n += snprintf(buf + n, cap - n, "%d,%d,%d,%d,^;", kScrollUp.x0, kScrollUp.y0, kScrollUp.x1,
                  kScrollUp.y1);
    n += snprintf(buf + n, cap - n, "%d,%d,%d,%d,v;", kScrollDown.x0, kScrollDown.y0,
                  kScrollDown.x1, kScrollDown.y1);
  }
}

// "canUp,canDown" for the grid's scroll sidebar -- mirrors drawGrid()'s
// dimming logic so the laptop can dim the same arrows.
static void formatScrollState(char *buf, size_t cap, int count) {
  snprintf(buf, cap, "%d,%d", scrollOffset > 0 ? 1 : 0,
           scrollOffset < gridMaxScroll(count) ? 1 : 0);
}

static void appendBackElement(char *buf, int &n, size_t cap) {
  n += snprintf(buf + n, cap - n, "%d,%d,%d,%d,BACK;", kBack.x0, kBack.y0, kBack.x1, kBack.y1);
}

// One line per update:
//   RUI|<screen>|<breadcrumb>|BTN:...|FIELDS:...|FOOTER:...|CODE:...|MOTORS:...
//      |SCROLL:up,down|STEP:d|FOOTC:c|PLOT:mode,hasPt,hasHdg,x,y,th,px,py,pth
//      |TRAIL:x,y;x,y;...|SEL:i|ACTIVE:side,idx
// Tagged segments so screens that don't have some kind of content (e.g.
// HOME has no field list) just send it empty rather than needing a
// different line shape per screen. Sent a few times a second (not every
// loop tick -- no need to spam the link) so a laptop app can redraw its
// mirror without polling. This carries *every* input the brain's own
// draw*() functions read, so the laptop can run a port of the same
// drawing code and come out pixel-for-pixel the same layout.
static void sendRemoteUiState(bool force) {
  static uint32_t lastSend = 0;
  remoteTouchMutex.take();
  uint32_t interval = remoteInput.sendIntervalMs;
  remoteTouchMutex.give();
  if (!force && pros::millis() - lastSend < interval)
    return;
  lastSend = pros::millis();

  char btnBuf[512];
  btnBuf[0] = '\0';
  int btnN = 0;
  char fieldsBuf[256];
  fieldsBuf[0] = '\0';
  int fieldsN = 0;
  char footerBuf[100] = "";
  char codeBuf[256] = "";
  char motorsBuf[256];
  motorsBuf[0] = '\0';
  int motorsN = 0;
  char scrollBuf[8] = "";
  char planBuf[32] = "";
  char pathBuf[1024];
  pathBuf[0] = '\0';
  char plotBuf[128] = "";
  char trailBuf[1400];
  trailBuf[0] = '\0';
  int trailN = 0;
  float step = digitStep();
  int footerCancelled = 0;
  int activeSide = -1, activeIdx = -1;

  const char *screenTag = "HOME", *breadcrumb = "-";

  switch (screen) {
  case Screen::HOME:
    screenTag = "HOME";
    appendGridElements(btnBuf, btnN, sizeof(btnBuf), kHomeItems, kHomeItemCount);
    formatScrollState(scrollBuf, sizeof(scrollBuf), 4);
    break;
  case Screen::PATH_TYPE:
    screenTag = "PATH_TYPE";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    appendGridElements(btnBuf, btnN, sizeof(btnBuf), kPathItems, 2);
    formatScrollState(scrollBuf, sizeof(scrollBuf), 2);
    break;
  case Screen::ANGULAR_LIST:
    screenTag = "ANGULAR_LIST";
    breadcrumb = "ANGULAR";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    appendGridElements(btnBuf, btnN, sizeof(btnBuf), kAngularItems, 4);
    formatScrollState(scrollBuf, sizeof(scrollBuf), 4);
    break;
  case Screen::LATERAL_LIST:
    screenTag = "LATERAL_LIST";
    breadcrumb = "LATERAL";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    appendGridElements(btnBuf, btnN, sizeof(btnBuf), kLateralItems, 2);
    formatScrollState(scrollBuf, sizeof(scrollBuf), 2);
    break;
  case Screen::EDIT: {
    screenTag = "EDIT";
    breadcrumb = kMotionInfo[(int)currentMotion].name;
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));

    for (size_t i = 0; i < fields.size(); i++) {
      char valBuf[16];
      formatFieldValue(fields[i], valBuf, sizeof(valBuf));
      fieldsN += snprintf(fieldsBuf + fieldsN, sizeof(fieldsBuf) - fieldsN, "%s,%s,%d;",
                          fields[i].label, valBuf, (int)i == selectedField ? 1 : 0);
    }

    buildFooterText(footerBuf, sizeof(footerBuf));
    footerCancelled = lastResult.cancelled ? 1 : 0;

    char codeLines[3][80];
    formatMotionCode(codeLines);
    snprintf(codeBuf, sizeof(codeBuf), "%s~%s~%s", codeLines[0], codeLines[1], codeLines[2]);

    // Everything drawTargetIndicator()/runMotion() feed the field plot.
    const MotionInfo &info = kMotionInfo[(int)currentMotion];
    lemlib::Pose pose = remotePlotMode == 2 ? remoteLivePose : chassis.getPose();
    snprintf(plotBuf, sizeof(plotBuf), "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", remotePlotMode,
             info.hasPointTarget ? 1 : 0, info.hasHeadingTarget ? 1 : 0, pX, pY, pTheta, pose.x,
             pose.y, pose.theta);

    // Trail points, thinned to a fixed budget so the line never outgrows
    // its buffer on a long run (the plot is only 310px wide anyway).
    const int kMaxTrailPts = 160;
    int total = (int)remoteTrail.size();
    int stride = std::max(1, (total + kMaxTrailPts - 1) / kMaxTrailPts);
    for (int i = 0; i < total; i += stride) {
      int w = snprintf(trailBuf + trailN, sizeof(trailBuf) - trailN, "%d,%d;",
                       (int)remoteTrail[i].first, (int)remoteTrail[i].second);
      if (w < 0 || trailN + w >= (int)sizeof(trailBuf))
        break;
      trailN += w;
    }
    break;
  }
  case Screen::TEMPS: {
    screenTag = "TEMPS";
    breadcrumb = "";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    // Reuses the FIELDS segment ("label,value,selected;"), which is
    // already a name/value list -- no new segment for the laptop to learn.
    // A section header rides along as an entry whose label starts with '#'.
    std::vector<double> temps[kTempSections];
    std::vector<std::int8_t> ports[kTempSections];
    collectTemps(temps, ports);
    // How far the list is scrolled, so the mirror lands on the same rows.
    fieldsN += snprintf(fieldsBuf + fieldsN, sizeof(fieldsBuf) - fieldsN, "$scroll,%d,0;",
                        tempsScroll);
    for (int sIdx = 0; sIdx < kTempSections; sIdx++) {
      const TempSection &info = kTempSectionInfo[sIdx];
      // "#<label>,<colour>:<tiles per row>,0;" -- the segment's third
      // value is a flag to everything else, so the layout rides in the
      // value instead.
      fieldsN += snprintf(fieldsBuf + fieldsN, sizeof(fieldsBuf) - fieldsN, "#%s,%06X:%d,0;",
                          info.label, (unsigned)info.color, info.tilesPerRow);
      for (int i = 0; i < (int)temps[sIdx].size() && i < info.tilesPerRow; i++)
        fieldsN += snprintf(fieldsBuf + fieldsN, sizeof(fieldsBuf) - fieldsN, "%s %d P%d,%.1f,0;",
                            info.prefix, i + 1, (int)std::abs(ports[sIdx][i]), temps[sIdx][i]);
    }
    if (tempsMinScroll() < 0) {
      btnN += snprintf(btnBuf + btnN, sizeof(btnBuf) - btnN, "%d,%d,%d,%d,^;", kScrollUp.x0,
                       kScrollUp.y0, kScrollUp.x1, kScrollUp.y1);
      btnN += snprintf(btnBuf + btnN, sizeof(btnBuf) - btnN, "%d,%d,%d,%d,v;", kScrollDown.x0,
                       kScrollDown.y0, kScrollDown.x1, kScrollDown.y1);
    }
    snprintf(scrollBuf, sizeof(scrollBuf), "%d,%d", tempsScroll < 0 ? 1 : 0,
             tempsScroll > tempsMinScroll() ? 1 : 0);
    break;
  }
  case Screen::REMOTE_DRIVE: {
    screenTag = "REMOTE_DRIVE";
    breadcrumb = "W A S D";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    lemlib::Pose pose = chassis.getPose();
    snprintf(plotBuf, sizeof(plotBuf), "3,0,0,0,0,0,%.2f,%.2f,%.2f", pose.x, pose.y, pose.theta);
    snprintf(footerBuf, sizeof(footerBuf), "W/A/S/D drives   L %d  R %d",
             remoteDriving ? remoteInput.driveL : 0, remoteDriving ? remoteInput.driveR : 0);
    break;
  }
  case Screen::PLANNER: {
    screenTag = "PLANNER";
    breadcrumb = "LAPTOP";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));
    btnN += snprintf(btnBuf + btnN, sizeof(btnBuf) - btnN, "%d,%d,%d,%d,BRAIN EDITOR;",
                     kBrainEditor.x0, kBrainEditor.y0, kBrainEditor.x1, kBrainEditor.y1);
    lemlib::Pose pose = planStatus == PlanStatus::RUNNING ? remoteLivePose : chassis.getPose();
    snprintf(plotBuf, sizeof(plotBuf), "3,0,0,0,0,0,%.2f,%.2f,%.2f", pose.x, pose.y, pose.theta);
    snprintf(planBuf, sizeof(planBuf), "%d,%d,%d", (int)planStatus, planStepIdx, (int)plan.size());
    snprintf(footerBuf, sizeof(footerBuf), "pose (%.1f, %.1f) hdg %.1f   A:run all  X:stop",
             pose.x, pose.y, pose.theta);
    if (planStepIdx >= 0 && planStepIdx < (int)plan.size()) {
      char codeLines[3][80];
      formatStepCode(plan[planStepIdx], codeLines);
      snprintf(codeBuf, sizeof(codeBuf), "%s~%s~%s", codeLines[0], codeLines[1], codeLines[2]);
    }
    // Pixel trail for the brain-screen mirror, inch path for the map.
    const int kMaxPts = 80;
    int total = (int)remoteTrail.size();
    int stride = std::max(1, (total + kMaxPts - 1) / kMaxPts);
    for (int i = 0; i < total; i += stride) {
      int w = snprintf(trailBuf + trailN, sizeof(trailBuf) - trailN, "%d,%d;",
                       (int)remoteTrail[i].first, (int)remoteTrail[i].second);
      if (w < 0 || trailN + w >= (int)sizeof(trailBuf))
        break;
      trailN += w;
    }
    int pathN = 0;
    total = (int)planPath.size();
    stride = std::max(1, (total + kMaxPts - 1) / kMaxPts);
    for (int i = 0; i < total; i += stride) {
      int w = snprintf(pathBuf + pathN, sizeof(pathBuf) - pathN, "%.1f,%.1f;",
                       planPath[i].first, planPath[i].second);
      if (w < 0 || pathN + w >= (int)sizeof(pathBuf))
        break;
      pathN += w;
    }
    break;
  }
  case Screen::MOTOR_TEST: {
    screenTag = "MOTOR_TEST";
    appendBackElement(btnBuf, btnN, sizeof(btnBuf));

    auto appendSide = [&](char side, const std::vector<MotorTestEntry> &list) {
      for (const MotorTestEntry &e : list) {
        const char *state = !e.tested                ? "pending"
                            : e.noMove                ? "No spin"
                            : e.configuredReversed    ? "Negative"
                                                       : "Positive";
        motorsN += snprintf(motorsBuf + motorsN, sizeof(motorsBuf) - motorsN, "%c,%d,%s;", side,
                            (int)e.rawPort, state);
      }
    };
    appendSide('L', leftMotorTest);
    appendSide('R', rightMotorTest);
    activeSide = motorTestActiveSide;
    activeIdx = motorTestActiveIdx;
    snprintf(footerBuf, sizeof(footerBuf), "%s",
             motorTestDone ? "done -- tap BACK to return" : "testing -- keep the drivetrain clear");
    break;
  }
  }

  // Build the line, then only send it if it differs from the last one
  // sent (or a heartbeat is due). Idle screens then cost the link
  // nothing, which matters over the controller's radio: every byte we
  // push out delays the taps coming back in.
  static char line[4200];
  static char lastLine[4200] = "";
  static uint32_t lastActualSend = 0;
  const uint32_t kHeartbeatMs = 1000;
  snprintf(line, sizeof(line),
           "RUI|%s|%s|BTN:%s|FIELDS:%s|FOOTER:%s|CODE:%s|MOTORS:%s|SCROLL:%s|STEP:%.2f|FOOTC:%d"
           "|PLOT:%s|TRAIL:%s|SEL:%d|ACTIVE:%d,%d|PLAN:%s|PATH:%s",
           screenTag, breadcrumb, btnBuf, fieldsBuf, footerBuf, codeBuf, motorsBuf, scrollBuf, step,
           footerCancelled, plotBuf, trailBuf, selectedField, activeSide, activeIdx, planBuf,
           pathBuf);
  bool changed = strcmp(line, lastLine) != 0;
  if (!force && !changed && pros::millis() - lastActualSend < kHeartbeatMs)
    return;
  strcpy(lastLine, line);
  lastActualSend = pros::millis();
  printf("%s\n", line);
}

// Where the menu hands the screen to something the laptop doesn't mirror
// (normal driving) -- tell it so it can say so instead of showing a stale
// HOME. (The PID tuner mirrors itself: see sendTunerState in pid_tuner.cpp.)
static void sendRemoteHandoff(const char *what) {
  printf("RUI|HANDOFF|%s|BTN:|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0|FOOTC:0|PLOT:|TRAIL:"
         "|SEL:0|ACTIVE:-1,-1|PLAN:|PATH:\n", what);
}

// ─── Navigation ──────────────────────────────────────────────────────────────
static void goHome() {
  screen = Screen::HOME;
  scrollOffset = 0;
  drawHome();
}
static void goPathType() {
  screen = Screen::PATH_TYPE;
  scrollOffset = 0;
  drawPathType();
}
static void goAngularList() {
  screen = Screen::ANGULAR_LIST;
  scrollOffset = 0;
  drawAngularList();
}
static void goLateralList() {
  screen = Screen::LATERAL_LIST;
  scrollOffset = 0;
  drawLateralList();
}
static void goPlanner() {
  screen = Screen::PLANNER;
  scrollOffset = 0;
  drawPlannerScreen();
}
static void goTemps() {
  screen = Screen::TEMPS;
  scrollOffset = 0;
  tempsScroll = 0;
  drawTempsScreen();
}
static void goRemoteDrive() {
  screen = Screen::REMOTE_DRIVE;
  scrollOffset = 0;
  drawRemoteDriveScreen();
}
static void goMotorTest() {
  screen = Screen::MOTOR_TEST;
  scrollOffset = 0;
  stopRemoteDrive(); // this test drives the motors itself
  runMotorDirectionTest(); // blocks; draws its own screen live as it goes
}

static void enterEdit(Motion m) {
  currentMotion = m;
  buildFields(m);
  selectedField = 0;
  lastResult = RunResult{}; // fields/units differ between motions
  screen = Screen::EDIT;
  drawEditUI();
}

static void backFromEdit() {
  bool angular = kMotionInfo[(int)currentMotion].angular;
  angular ? goAngularList() : goLateralList();
}

// ─── Button edge detection (same small helper used by the PID tuner) ────────
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

// ─── Main menu loop ───────────────────────────────────────────────────────────
void driverMenuControl() {
  driverMenuActive = true;
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("DRIVER MENU ACTIVE\n");
  ensureRemoteTouchListener(); // see tools/remote_touch.py
  goHome();

  EdgeButton left{pros::E_CONTROLLER_DIGITAL_LEFT};
  EdgeButton right{pros::E_CONTROLLER_DIGITAL_RIGHT};
  EdgeButton up{pros::E_CONTROLLER_DIGITAL_UP};
  EdgeButton down{pros::E_CONTROLLER_DIGITAL_DOWN};
  EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
  EdgeButton btnB{pros::E_CONTROLLER_DIGITAL_B};
  EdgeButton btnL1{pros::E_CONTROLLER_DIGITAL_L1};
  EdgeButton btnL2{pros::E_CONTROLLER_DIGITAL_L2};

  bool wasTouched = false;

  while (true) {
    sendRemoteUiState(); // throttled internally; keeps the laptop mirror live

    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    bool pressEdge = status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int touchX = status.x, touchY = status.y;
    if (status.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (status.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;

    // A remote tap counts exactly like a physical one -- but never
    // overrides a real touch already in progress this tick.
    if (!pressEdge && takeRemoteTouch(touchX, touchY))
      pressEdge = true;

    if (pressEdge) {
      int x = touchX, y = touchY;
      static const Motion kAngularMotions[4] = {Motion::TURN_TO_HEADING, Motion::TURN_TO_POINT,
                                                Motion::SWING_TO_HEADING, Motion::SWING_TO_POINT};
      static const Motion kLateralMotions[2] = {Motion::MOVE_TO_POINT, Motion::MOVE_TO_POSE};

      switch (screen) {
      case Screen::HOME: {
        int hit = gridHitTest(kHomeItemCount, x, y);
        if (hit == 0) {
          pidTunerControl(); // returns once its own BACK button is tapped;
                             // sends its own RUI|PID_TUNER lines meanwhile
          goHome();
        } else if (hit == 1) {
          goPlanner();
        } else if (hit == 2) {
          goMotorTest();
        } else if (hit == 3) {
          sendRemoteHandoff("DRIVING");
          stopRemoteDrive();
          driverMenuActive = false;
          return; // caller falls through to normal driving
        } else if (hit == 4) {
          goRemoteDrive();
        } else if (hit == 5) {
          goTemps();
        }
        break;
      }
      case Screen::PATH_TYPE: {
        if (inRect(kBack, x, y)) {
          goHome();
          break;
        }
        if (gridHandleScrollTouch(2, x, y)) {
          drawPathType();
          break;
        }
        int hit = gridHitTest(2, x, y);
        if (hit == 0)
          goAngularList();
        else if (hit == 1)
          goLateralList();
        break;
      }
      case Screen::ANGULAR_LIST: {
        if (inRect(kBack, x, y)) {
          goPathType();
          break;
        }
        if (gridHandleScrollTouch(4, x, y)) {
          drawAngularList();
          break;
        }
        int hit = gridHitTest(4, x, y);
        if (hit >= 0)
          enterEdit(kAngularMotions[hit]);
        break;
      }
      case Screen::LATERAL_LIST: {
        if (inRect(kBack, x, y)) {
          goPathType();
          break;
        }
        if (gridHandleScrollTouch(2, x, y)) {
          drawLateralList();
          break;
        }
        int hit = gridHitTest(2, x, y);
        if (hit >= 0)
          enterEdit(kLateralMotions[hit]);
        break;
      }
      case Screen::EDIT:
        if (inRect(kBack, x, y)) {
          backFromEdit();
        }
        break;
      case Screen::MOTOR_TEST:
        if (inRect(kBack, x, y)) {
          goHome();
        }
        break;
      case Screen::PLANNER:
        if (inRect(kBack, x, y))
          goHome();
        else if (inRect(kBrainEditor, x, y))
          goPathType();
        break;
      case Screen::REMOTE_DRIVE:
        if (inRect(kBack, x, y)) {
          stopRemoteDrive(); // leaving the screen gives the drivetrain up
          goHome();
        }
        break;
      case Screen::TEMPS:
        if (inRect(kBack, x, y)) {
          goHome();
        } else if (inRect(kScrollUp, x, y)) {
          tempsScroll = std::min(0, tempsScroll + kTempRowH);
          drawTempsScreen();
        } else if (inRect(kScrollDown, x, y)) {
          tempsScroll = std::max(tempsMinScroll(), tempsScroll - kTempRowH);
          drawTempsScreen();
        }
        break;
      }
    }

    // Laptop planner: block list edits apply anywhere; a run request pulls
    // the brain onto the planner screen first so its plot shows the run.
    {
      int runWhich;
      if (applyRemotePlanCommands(runWhich)) {
        stopRemoteDrive(); // a planned motion takes the drivetrain over
        if (screen != Screen::PLANNER)
          goPlanner();
        runPlan(runWhich);
      }
    }

    // Temperatures creep rather than jump, so this screen is repainted a
    // few times a second instead of every tick.
    if (screen == Screen::TEMPS) {
      static uint32_t lastTempDraw = 0;
      if (pros::millis() - lastTempDraw >= 500) {
        lastTempDraw = pros::millis();
        drawTempsScreen();
      }
    }

    applyRemoteDrive();
    if (screen == Screen::REMOTE_DRIVE) {
      int l, r;
      remoteTouchMutex.take();
      l = remoteDriving ? remoteInput.driveL : 0;
      r = remoteDriving ? remoteInput.driveR : 0;
      remoteTouchMutex.give();
      if (l != drawnDriveL || r != drawnDriveR || remoteDriving != drawnDriveLive)
        drawRemoteDriveStatus();
    }

    // Controller edges OR'd with the laptop's KEY presses -- from here on
    // the two are indistinguishable.
    bool leftEdge = left.pressed() || takeRemoteKey(RK_LEFT);
    bool rightEdge = right.pressed() || takeRemoteKey(RK_RIGHT);
    bool upEdge = up.pressed() || takeRemoteKey(RK_UP);
    bool downEdge = down.pressed() || takeRemoteKey(RK_DOWN);
    bool aEdge = btnA.pressed() || takeRemoteKey(RK_A);
    bool bEdge = btnB.pressed() || takeRemoteKey(RK_B);
    bool l1Edge = btnL1.pressed() || takeRemoteKey(RK_L1);
    bool l2Edge = btnL2.pressed() || takeRemoteKey(RK_L2);
    takeRemoteKey(RK_X); // only meaningful mid-run (waitForCancel); drop stale ones

    if (screen == Screen::PLANNER && aEdge) {
      stopRemoteDrive();
      runPlan(-1);
    }

    if (screen == Screen::EDIT && !fields.empty()) {
      bool dirty = leftEdge || rightEdge || l1Edge || l2Edge;
      dirty = applyRemoteFieldEdits() || dirty;
      if (leftEdge)
        selectedField = (selectedField + (int)fields.size() - 1) % (int)fields.size();
      if (rightEdge)
        selectedField = (selectedField + 1) % (int)fields.size();
      if (l1Edge)
        digitExp = std::min(digitExp + 1, kDigitExpMax);
      if (l2Edge)
        digitExp = std::max(digitExp - 1, kDigitExpMin);

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

      if (aEdge) {
        runMotion();
      } else if (dirty) {
        drawEditUI();
        sendRemoteUiState(true);
      }

      drawEditControllerUI();
    }

    pros::delay(20);
  }
}
