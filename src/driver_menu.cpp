// ─────────────────────────────────────────────────────────────────────────────
// DRIVER MENU — see include/driver_menu.hpp for the full controls reference.
// Touchscreen home screen shown at the start of opcontrol():
//   HOME -> PID TUNING | PATH PLANNER | TEST MOTORS | DRIVE
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
#include "autonomous_builder.hpp"
#include "pid_tuner.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
enum class Screen { HOME, PATH_TYPE, ANGULAR_LIST, LATERAL_LIST, EDIT, MOTOR_TEST };
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

static const GridItem kHomeItems[5] = {
    {"PID TUNING"},
  {"AUTON BUILDER"},
    {"PATH PLANNER"},
    {"TEST MOTORS"},
    {"DRIVE"},
};

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
  drawGrid(kHomeItems, 5);
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

static void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_pen(lastResult.cancelled ? TARGET_LINE : GRAY);
  if (!lastResult.ran) {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 222,
                        "no run yet -- press A to go, X to cancel one");
    return;
  }
  char buf[100];
  int n = 0;
  if (lastResult.cancelled)
    n += snprintf(buf + n, sizeof(buf) - n, "CANCELLED // ");
  switch (lastResult.kind) {
  case ErrorKind::HEADING:
  case ErrorKind::FACE_POINT:
    n += snprintf(buf + n, sizeof(buf) - n, "final hdg %.2f // err %.2f deg ",
                  lastResult.finalHeading, lastResult.headingError);
    break;
  case ErrorKind::POSITION:
    n += snprintf(buf + n, sizeof(buf) - n, "final (%.2f, %.2f) // err %.2f in ",
                  lastResult.finalX, lastResult.finalY, lastResult.posError);
    break;
  case ErrorKind::POSITION_AND_HEADING:
    n += snprintf(buf + n, sizeof(buf) - n,
                  "final (%.2f, %.2f) hdg %.2f // %.2fin %.2fdeg ",
                  lastResult.finalX, lastResult.finalY, lastResult.finalHeading,
                  lastResult.posError, lastResult.headingError);
    break;
  }
  snprintf(buf + n, sizeof(buf) - n, "// %dms", lastResult.durationMs);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", buf);
}

// ─── Movement code preview ────────────────────────────────────────────────
// Builds the exact chassis.* call runMotion() below will issue for the
// current motion, using the *current* field values -- so editing a
// parameter shows the real C++ that results, not just the raw number.
// Kept in its own function (rather than duplicated by hand) and mirrored
// against runMotion()'s switch so the two can't drift apart.
static void formatMotionCode(char lines[3][80]) {
  lines[0][0] = lines[1][0] = lines[2][0] = '\0';
  bool forwards = pForwardsIdx < 0.5f;
  const char *fwdLbl = forwards ? "true" : "false";
  const char *dirLbl = kDirectionLabels[std::clamp((int)std::lround(pDirectionIdx), 0, 2)];
  const char *sideLbl = kSideLabels[std::clamp((int)std::lround(pSideIdx), 0, 1)];
  int timeoutMs = (int)pTimeoutMs;

  switch (currentMotion) {
  case Motion::TURN_TO_HEADING:
    snprintf(lines[0], 80, "chassis.turnToHeading(%.2f, %d,", pTheta, timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl,
             pMaxSpeed, pMinSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", pEarlyExitRange);
    break;
  case Motion::TURN_TO_POINT:
    snprintf(lines[0], 80, "chassis.turnToPoint(%.2f, %.2f, %d,", pX, pY, timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl,
             pMaxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", pMinSpeed,
             pEarlyExitRange);
    break;
  case Motion::SWING_TO_HEADING:
    snprintf(lines[0], 80, "chassis.swingToHeading(%.2f, %s, %d,", pTheta, sideLbl, timeoutMs);
    snprintf(lines[1], 80, "  {.direction=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", dirLbl,
             pMaxSpeed, pMinSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", pEarlyExitRange);
    break;
  case Motion::SWING_TO_POINT:
    snprintf(lines[0], 80, "chassis.swingToPoint(%.2f, %.2f, %s, %d,", pX, pY, sideLbl,
             timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .direction=%s, .maxSpeed=%.0f,", fwdLbl, dirLbl,
             pMaxSpeed);
    snprintf(lines[2], 80, "   .minSpeed=%.0f, .earlyExitRange=%.2f});", pMinSpeed,
             pEarlyExitRange);
    break;
  case Motion::MOVE_TO_POINT:
    snprintf(lines[0], 80, "chassis.moveToPoint(%.2f, %.2f, %d,", pX, pY, timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .maxSpeed=%.0f, .minSpeed=%.0f,", fwdLbl,
             pMaxSpeed, pMinSpeed);
    snprintf(lines[2], 80, "   .earlyExitRange=%.2f});", pEarlyExitRange);
    break;
  case Motion::MOVE_TO_POSE:
    snprintf(lines[0], 80, "chassis.moveToPose(%.2f, %.2f, %.2f, %d,", pX, pY, pTheta,
             timeoutMs);
    snprintf(lines[1], 80, "  {.forwards=%s, .horizontalDrift=%.2f, .lead=%.2f,", fwdLbl,
             pHorizontalDrift, pLead);
    snprintf(lines[2], 80, "   .maxSpeed=%.0f, .minSpeed=%.0f, .earlyExitRange=%.2f});",
             pMaxSpeed, pMinSpeed, pEarlyExitRange);
    break;
  }
}

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

// ─── Running a motion ────────────────────────────────────────────────────────
static bool waitForCancel() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    chassis.cancelMotion();
    return true;
  }
  return false;
}

static void runMotion() {
  const MotionInfo &info = kMotionInfo[(int)currentMotion];
  int timeoutMs = (int)pTimeoutMs;
  bool forwards = pForwardsIdx < 0.5f;
  lemlib::AngularDirection dir = directionFromIdx((int)std::lround(pDirectionIdx));
  lemlib::DriveSide side = sideFromIdx((int)std::lround(pSideIdx));

  lemlib::Pose start = chassis.getPose();
  drawFieldFrame();
  drawTargetIndicator();

  switch (currentMotion) {
  case Motion::TURN_TO_HEADING:
    chassis.turnToHeading(pTheta, timeoutMs,
                          {.direction = dir,
                           .maxSpeed = (int)pMaxSpeed,
                           .minSpeed = (int)pMinSpeed,
                           .earlyExitRange = pEarlyExitRange},
                          true);
    break;
  case Motion::TURN_TO_POINT:
    chassis.turnToPoint(pX, pY, timeoutMs,
                        {.forwards = forwards,
                         .direction = dir,
                         .maxSpeed = (int)pMaxSpeed,
                         .minSpeed = (int)pMinSpeed,
                         .earlyExitRange = pEarlyExitRange},
                        true);
    break;
  case Motion::SWING_TO_HEADING:
    chassis.swingToHeading(pTheta, side, timeoutMs,
                           {.direction = dir,
                            .maxSpeed = pMaxSpeed,
                            .minSpeed = pMinSpeed,
                            .earlyExitRange = pEarlyExitRange},
                           true);
    break;
  case Motion::SWING_TO_POINT:
    chassis.swingToPoint(pX, pY, side, timeoutMs,
                         {.forwards = forwards,
                          .direction = dir,
                          .maxSpeed = pMaxSpeed,
                          .minSpeed = pMinSpeed,
                          .earlyExitRange = pEarlyExitRange},
                         true);
    break;
  case Motion::MOVE_TO_POINT:
    chassis.moveToPoint(pX, pY, timeoutMs,
                        {.forwards = forwards,
                         .maxSpeed = pMaxSpeed,
                         .minSpeed = pMinSpeed,
                         .earlyExitRange = pEarlyExitRange},
                        true);
    break;
  case Motion::MOVE_TO_POSE:
    chassis.moveToPose(pX, pY, pTheta, timeoutMs,
                       {.forwards = forwards,
                        .horizontalDrift = pHorizontalDrift,
                        .lead = pLead,
                        .maxSpeed = pMaxSpeed,
                        .minSpeed = pMinSpeed,
                        .earlyExitRange = pEarlyExitRange},
                       true);
    break;
  }

  // isInMotion() can briefly read false right after an async motion is
  // issued, before its task has flagged itself running.
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
  chassis.waitUntilDone();

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
static void drawMotorTestScreen(int activeSide, int activeIdx) {
  using namespace ui;
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
static void goMotorTest() {
  screen = Screen::MOTOR_TEST;
  scrollOffset = 0;
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
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    bool pressEdge = status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    if (status.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (status.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;

    if (pressEdge) {
      int x = status.x, y = status.y;
      static const Motion kAngularMotions[4] = {Motion::TURN_TO_HEADING, Motion::TURN_TO_POINT,
                                                Motion::SWING_TO_HEADING, Motion::SWING_TO_POINT};
      static const Motion kLateralMotions[2] = {Motion::MOVE_TO_POINT, Motion::MOVE_TO_POSE};

      switch (screen) {
      case Screen::HOME: {
        int hit = gridHitTest(5, x, y);
        if (hit == 0) {
          pidTunerControl(); // returns once its own BACK button is tapped
          goHome();
        } else if (hit == 1) {
          autonomousBuilderControl();
          goHome();
        } else if (hit == 2) {
          goPathType();
        } else if (hit == 3) {
          goMotorTest();
        } else if (hit == 4) {
          driverMenuActive = false;
          return; // caller falls through to normal driving
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
      }
    }

    bool leftEdge = left.pressed(), rightEdge = right.pressed();
    bool upEdge = up.pressed(), downEdge = down.pressed();
    bool aEdge = btnA.pressed(), bEdge = btnB.pressed();
    bool l1Edge = btnL1.pressed(), l2Edge = btnL2.pressed();

    if (screen == Screen::EDIT && !fields.empty()) {
      bool dirty = leftEdge || rightEdge || l1Edge || l2Edge;
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
      }

      drawEditControllerUI();
    }

    pros::delay(20);
  }
}
