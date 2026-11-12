#include "lift_calibration.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "pid_tuner_shared.hpp"
#include "run_log.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

// ─── The level table ─────────────────────────────────────────────────────────
// Motor degrees from the limit switch home position. The laptop window's WRITE TO
// SOURCE rewrites the lines between the two markers, so keep them as they are.
// (The first level, LIFT_GROUND_LEVEL, has no drop angle: its entry is unused.)
// LIFT-LEVELS-BEGIN
static const char *const kLevelNames[LIFT_LEVEL_COUNT] = {"GROUND", "LOW", "MID", "HIGH", "MAX", "PIN"};
static const double kDefaultAngles[LIFT_LEVEL_COUNT] = {171.4, 255.6, 336.0, 410.4, 500.0, 29.6};
static const double kDefaultDropAngles[LIFT_LEVEL_COUNT] = {0.0, 109.8, 248.0, 314.6, 431.8, 0.0};
// LIFT-LEVELS-END

double liftLevelAngles[LIFT_LEVEL_COUNT];
double liftDropAngles[LIFT_LEVEL_COUNT];
static const bool kTablesInit = [] {
  for (int i = 0; i < LIFT_LEVEL_COUNT; i++) {
    liftLevelAngles[i] = kDefaultAngles[i];
    liftDropAngles[i] = kDefaultDropAngles[i];
  }
  return true;
}();

const char *liftLevelName(int level) {
  return level >= 0 && level < LIFT_LEVEL_COUNT ? kLevelNames[level] : "?";
}

static double clampAngle(double deg) { return std::clamp(deg, 0.0, LIFT_UPPER_LIMIT); }

static constexpr double kArriveTolDeg = 3.0;

static double levelAngle(int level, bool drop) {
  return clampAngle(drop ? liftDropAngles[level] : liftLevelAngles[level]);
}

bool liftAtLevel(int level, bool drop) {
  if (level < 0 || level >= LIFT_LEVEL_COUNT)
    return false;
  return std::fabs(lift.get_position() - levelAngle(level, drop)) <= kArriveTolDeg;
}

bool liftLimitPressed() {
  static uint32_t pressedSince = 0; // 0 = not pressed
  if (!liftLimitSwitch.get_value()) {
    pressedSince = 0;
    return false;
  }
  const uint32_t now = pros::millis();
  if (pressedSince == 0)
    pressedSince = now ? now : 1;
  return now - pressedSince >= 30;
}

static volatile bool liftRelaxed = false;

void liftRelax() {
  liftRelaxed = true;
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  lift.brake();
}

void liftHold() {
  if (!liftRelaxed)
    return;
  liftRelaxed = false;
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

bool liftControlledStep(double targetDeg, double &vel, int dtMs) {
  liftHold();
  double err = targetDeg - lift.get_position();
  if (std::fabs(err) <= kArriveTolDeg || (err < 0 && liftLimitPressed())) {
    vel = 0;
    lift.move_velocity(0); // brake mode HOLD keeps it there
    return true;
  }
  // Ease off in proportion to the distance left, but never crawl.
  double want = std::clamp(std::fabs(err) * LIFT_DROP_KP, LIFT_DROP_MIN_VEL, LIFT_DROP_MAX_VEL);
  if (err < 0)
    want = -want;
  // Never change speed faster than the acceleration limit: that is what keeps the
  // start and the stop smooth instead of jerky.
  double maxStep = LIFT_DROP_ACCEL * std::max(dtMs, 1);
  vel += std::clamp(want - vel, -maxStep, maxStep);
  lift.move_velocity((int)std::lround(vel));
  return false;
}

void liftGoToLevel(int level, bool wait, int timeoutMs, bool drop) {
  if (level < 0 || level >= LIFT_LEVEL_COUNT)
    return;
  liftHold();
  if (drop) { // drops are always controlled (see liftControlledStep)
    const double target = levelAngle(level, true);
    if (!wait) {
      lift.move_absolute(target, (int)LIFT_DROP_MAX_VEL); // no loop to ease it: just capped
      return;
    }
    double vel = 0;
    uint32_t until = pros::millis() + (uint32_t)std::max(0, timeoutMs);
    while (pros::millis() < until) {
      if (liftControlledStep(target, vel, 10))
        return;
      pros::delay(10);
    }
    lift.move_velocity(0); // out of time: hold where it got to
    return;
  }
  lift.move_absolute(levelAngle(level, drop), 100);
  if (!wait)
    return;
  uint32_t until = pros::millis() + (uint32_t)std::max(0, timeoutMs);
  while (!liftAtLevel(level, drop) && pros::millis() < until)
    pros::delay(10);
}

void liftOpenClaws() {
  if (LIFT_CLAW_OPEN_RETRACTED) {
    claw.retract();
  } else {
    claw.extend();
  }
}

void clawsClose(bool useClaw) {
  if (!useClaw)
    return;
  if (LIFT_CLAW_OPEN_RETRACTED)
    claw.extend();
  else
    claw.retract();
}

// ─── IR pin sensor ───────────────────────────────────────────────────────────
bool pinSensorActiveHigh = true;

bool pinDetected() {
  const int32_t v = pinSensor.get_value();
  return v == (pinSensorActiveHigh ? 1 : 0); // PROS_ERR (unplugged) matches neither
}

bool liftAtStart() {
  return liftLimitSwitch.get_value() || lift.get_position() <= kArriveTolDeg;
}

static volatile bool pinAutoOn = false;
static volatile bool pinGrabMoving = false;
static volatile int pinGrabs = 0;
static volatile bool pinGrabFast = false; // still on full power, not yet handed to the position hold
static uint32_t pinGrabAtMs = 0;
// An auto-grab drives the lift up at full power (a position move ramps up and is slow over this
// short rise) and switches to the position hold this many degrees before the PIN angle.
static constexpr double kPinHandoffDeg = 15.0;

// Called every 10 ms by the task: finishes the fast rise with the position hold.
static void pinGrabUpdate() {
  if (!pinGrabMoving || !pinGrabFast)
    return;
  const double target = levelAngle(LIFT_PIN_LEVEL, false);
  if (lift.get_position() >= target - kPinHandoffDeg) {
    pinGrabFast = false;
    lift.move_absolute(target, 100);
  }
}

bool pinAutoGrabEnabled() { return pinAutoOn; }
bool pinGrabLiftMoving() {
  if (pinGrabMoving && (liftAtLevel(LIFT_PIN_LEVEL) || pros::millis() - pinGrabAtMs > 2000))
    pinGrabMoving = false;
  return pinGrabMoving;
}
void pinGrabCancel() {
  pinGrabMoving = false;
  pinGrabFast = false;
}
int pinGrabCount() { return pinGrabs; }

bool pinAutoGrabTry() {
  if (pinGrabMoving || pros::millis() - pinGrabAtMs < 600) // one grab at a time
    return false;
  if (!pinDetected()) // the cheap checks first, so the claw shuts the instant the sensor sees the pin
    return false;
  // Only an open claw grabs: once it has shut on the pin the sensor keeps seeing it, and that must
  // not re-trigger until the claws have been opened again.
  const bool clawOpenNow = claw.is_extended() != LIFT_CLAW_OPEN_RETRACTED;
  if (!clawOpenNow)
    return false;
  if (!liftAtStart()) // auto claw only works with the lift in its starting position (angle 0.0)
    return false;
  clawsClose(); // shut the instant the sensor sees the pin: mid-drive, mid-turn, lift anywhere
  runlog::event("auto-grab: pin seen, claws closed");
  pinGrabAtMs = pros::millis();
  pinGrabs = pinGrabs + 1;
  // the lift is at its start position (checked above): carry the pin up to the PIN level
  pinGrabMoving = true;
  pinGrabFast = true;
  lift.move_voltage(12000); // full power up; pinGrabUpdate() takes over near the PIN angle
  return true;
}

static void pinAutoGrabTask(void *) {
  while (true) {
    if (pinAutoOn)
      pinAutoGrabTry();
    pinGrabUpdate();
    pros::delay(2); // fast poll: the claw shuts within a couple of ms of the sensor seeing the pin
  }
}

void pinAutoGrabEnable(bool on) {
  static pros::Task *task = nullptr;
  if (on && task == nullptr)
    task = new pros::Task(pinAutoGrabTask, nullptr, "pin auto-grab");
  pinAutoOn = on;
  if (!on) {
    if (pinGrabMoving && pinGrabFast)
      lift.move_velocity(0); // switched off mid-rise: stop where it is
    pinGrabMoving = false;
    pinGrabFast = false;
  }
}

// Claw watch: its own task, so the claws shut the instant a pin is seen even while the calling
// thread is stuck in a turn, a queued motion, or a waitUntilDone().
static volatile bool pinWatchOn = false;
static volatile bool pinWatchHit = false;

static void pinWatchTask(void *) {
  while (true) {
    if (pinWatchOn && !pinWatchHit && pinDetected()) {
      clawsClose();
      pinWatchHit = true;
      runlog::event("pin seen: claws closing");
    }
    pros::delay(2);
  }
}

void pinCloseWatch(bool on) {
  static pros::Task *task = nullptr;
  if (on && task == nullptr)
    task = new pros::Task(pinWatchTask, nullptr, "pin claw watch");
  if (on)
    pinWatchHit = false;
  pinWatchOn = on;
}

bool pinCloseWatchHit() { return pinWatchHit; }

int liftCurrentLevel() {
  const double pos = lift.get_position();
  int best = LIFT_GROUND_LEVEL;
  for (int i = 1; i < LIFT_LEVEL_COUNT; i++)
    if (std::fabs(pos - levelAngle(i, false)) < std::fabs(pos - levelAngle(best, false)))
      best = i;
  return best;
}

void shootToGoal(int timeoutMs) {
  liftDropAtLevel(liftCurrentLevel(), timeoutMs);
}

void liftDropAtLevel(int level, int timeoutMs) {
  if (level < 0 || level >= LIFT_LEVEL_COUNT)
    return;
  if (liftHasDrop(level))
    liftGoToLevel(level, true, timeoutMs, true); // reach the drop angle first...
  liftOpenClaws();                               // ...then open (GROUND: just open)
}

// ─── Calibration screen ──────────────────────────────────────────────────────
namespace {

constexpr double JOG_STEP_DEG = 10.0;
constexpr int JOG_VELOCITY = 100;
constexpr int DROP_TIMEOUT_MS = 4000;

int selected = 0;
bool goingTo = false;       // a GO TO / DROP / JOG move is in progress
double goTarget = 0;
bool jogHeld = false;       // lift is being driven by R1/R2 right now
bool dropping = false;      // a DROP is under way: claws open when it arrives
double dropVel = 0;         // the controlled drop's current speed (see liftControlledStep)
uint32_t lastTickMs = 0;
int dropLevel = 0;
uint32_t dropStartMs = 0;
char msg[80] = "";
int saveReq = 0;            // taps of SAVE CODE; the laptop writes the source once per new value
bool saveWaiting = false;   // waiting for the laptop's answer
uint32_t saveAskedMs = 0;
bool wasTouched = false;
EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
EdgeButton btnB{pros::E_CONTROLLER_DIGITAL_B};
EdgeButton btnX{pros::E_CONTROLLER_DIGITAL_X};
EdgeButton btnY{pros::E_CONTROLLER_DIGITAL_Y};
EdgeButton btnL1{pros::E_CONTROLLER_DIGITAL_L1};
EdgeButton btnL2{pros::E_CONTROLLER_DIGITAL_L2};
EdgeButton btnLeft{pros::E_CONTROLLER_DIGITAL_LEFT};
EdgeButton btnRight{pros::E_CONTROLLER_DIGITAL_RIGHT};
EdgeButton btnUp{pros::E_CONTROLLER_DIGITAL_UP};
EdgeButton btnDown{pros::E_CONTROLLER_DIGITAL_DOWN};

struct Box {
  int x0, y0, x1, y1;
};
bool inBox(const Box &b, int x, int y) { return x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1; }

// ── layout: level table on the left, live readout + actions on the right ──
constexpr int ROW_X0 = 10, ROW_X1 = 245, ROW_Y0 = 46, ROW_H = 24, ROW_GAP = 2; // 6 rows must end above the footer (y 208)
constexpr int COL_LIFT_X = 112, COL_DROP_X = 176; // absolute x of the two angle columns
Box rowBox(int i) {
  int y0 = ROW_Y0 + i * (ROW_H + ROW_GAP);
  return {ROW_X0, y0, ROW_X1, y0 + ROW_H};
}
constexpr int PANEL_X0 = 252, PANEL_X1 = 475;
constexpr int GAUGE_Y0 = 74, GAUGE_Y1 = 84;
const Box kGoBtn = {255, 90, 362, 114};
const Box kSaveBtn = {368, 90, 470, 114};
const Box kDropBtn = {255, 118, 362, 142};
const Box kSaveDropBtn = {368, 118, 470, 142};
const Box kJogUpBtn = {255, 146, 362, 170};
const Box kJogDownBtn = {368, 146, 470, 170};
const Box kHomeBtn = {255, 174, 310, 198};
const Box kClaw1Btn = {315, 174, 470, 198};
const Box kSaveCodeBtn = {372, 208, 470, 234}; // in the footer, right of the message

void setMsg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void setMsg(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  printf("LIFT CAL: %s\n", msg);
}

double liveAngle() {
  double p = lift.get_position();
  return std::isfinite(p) ? p : 0.0;
}

bool clawOpen(const pros::adi::Pneumatics &c) { return c.is_extended() != LIFT_CLAW_OPEN_RETRACTED; }

// ── laptop mirror ────────────────────────────────────────────────────────
// Every touchable thing goes in BTN so the mirror can tap it too.
struct Button {
  const Box *box;
  const char *label;
};
const Button kButtons[] = {{&kGoBtn, "GO TO"},         {&kSaveBtn, "SAVE LIFT"},
                           {&kDropBtn, "DROP"},        {&kSaveDropBtn, "SAVE DROP"},
                           {&kJogUpBtn, "JOG UP"},     {&kJogDownBtn, "JOG DOWN"},
                           {&kHomeBtn, "HOME"},        {&kClaw1Btn, "CLAW"},
                           {&kSaveCodeBtn, "SAVE CODE"}};

void sendState(bool force = false) {
  static uint32_t lastSend = 0, lastActual = 0;
  static char lastLine[1500] = "";
  uint32_t interval = remoteSendIntervalMs();
  if (!force && pros::millis() - lastSend < interval)
    return;
  lastSend = pros::millis();

  char btns[900];
  int bn = snprintf(btns, sizeof(btns), "%d,%d,%d,%d,BACK;", kBackX0, kBackY0, kBackX1, kBackY1);
  for (int i = 0; i < LIFT_LEVEL_COUNT; i++) {
    Box r = rowBox(i);
    bn += snprintf(btns + bn, sizeof(btns) - bn, "%d,%d,%d,%d,LEVEL %d;", r.x0, r.y0, r.x1, r.y1,
                   i + 1);
  }
  for (const Button &b : kButtons)
    bn += snprintf(btns + bn, sizeof(btns) - bn, "%d,%d,%d,%d,%s;", b.box->x0, b.box->y0,
                   b.box->x1, b.box->y1, b.label);

  char levels[480];
  int n = 0;
  levels[0] = '\0';
  for (int i = 0; i < LIFT_LEVEL_COUNT; i++)
    n += snprintf(levels + n, sizeof(levels) - n, "%s,%.1f,%.1f;", liftLevelName(i),
                  liftLevelAngles[i], liftHasDrop(i) ? liftDropAngles[i] : -1.0);

  char line[1500];
  snprintf(line, sizeof(line),
           "RUI|LIFT_CAL|%s|BTN:%s|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0"
           "|FOOTC:0|PLOT:|TRAIL:|SEL:%d|ACTIVE:-1,-1|PLAN:|PATH:"
           "|LIFT:%d,%.1f,%d,%d,%.1f,%d,%d,%d,%d~%s|MSG:%s",
           liftLevelName(selected), btns, selected, selected, liveAngle(),
           (goingTo || jogHeld) ? 1 : 0, liftLimitSwitch.get_value() ? 1 : 0, LIFT_UPPER_LIMIT,
           claw.is_extended() ? 1 : 0, 0 /* (claw2 was removed) */, dropping ? 1 : 0, saveReq, levels, msg);
  // Skip an unchanged line except for a 1 s heartbeat.
  bool changed = strcmp(line, lastLine) != 0;
  if (!force && !changed && pros::millis() - lastActual < 1000)
    return;
  strcpy(lastLine, line);
  lastActual = pros::millis();
  printf("%s\n", line);
}

// ── drawing ──────────────────────────────────────────────────────────────
void drawButton(const Box &b, const char *label, int fill, int fg, bool enabled = true) {
  using namespace ui;
  if (!enabled) {
    fill = CARD;
    fg = AXIS;
  }
  pros::screen::set_pen(fill);
  pros::screen::fill_rect(b.x0, b.y0, b.x1, b.y1);
  pros::screen::set_pen(enabled ? fg : AXIS);
  pros::screen::draw_rect(b.x0, b.y0, b.x1, b.y1);
  pros::screen::set_eraser(fill);
  pros::screen::set_pen(fg);
  pros::screen::print(pros::E_TEXT_SMALL, b.x0 + 8, b.y0 + (b.y1 - b.y0) / 2 - 6, "%s", label);
  pros::screen::set_eraser(BG);
}

void drawTitle() {
  using namespace ui;
  pros::screen::set_pen(CYAN);
  pros::screen::fill_rect(0, 0, 405, 30);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 7, "LIFT CAL  >  %s", liftLevelName(selected));
  pros::screen::set_eraser(BG);
}

void drawRow(int i) {
  using namespace ui;
  Box r = rowBox(i);
  bool sel = i == selected;
  int fill = sel ? SETTLE_BAND_BG : CARD;
  pros::screen::set_pen(fill);
  pros::screen::fill_rect(r.x0, r.y0, r.x1, r.y1);
  if (sel) { // accent bar on the selected row
    pros::screen::set_pen(CYAN);
    pros::screen::fill_rect(r.x0, r.y0, r.x0 + 3, r.y1);
  }
  pros::screen::set_pen(sel ? CYAN : AXIS);
  pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
  pros::screen::set_eraser(fill);
  int ty = r.y0 + (ROW_H - 12) / 2;
  pros::screen::set_pen(sel ? CYAN : 0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 10, ty, "%d %s", i + 1, liftLevelName(i));
  pros::screen::set_pen(sel ? 0xFFFFFF : 0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, COL_LIFT_X, ty, "%6.1f", liftLevelAngles[i]);
  pros::screen::set_pen(ORANGE);
  if (liftHasDrop(i))
    pros::screen::print(pros::E_TEXT_SMALL, COL_DROP_X, ty, "%6.1f", liftDropAngles[i]);
  else
    pros::screen::print(pros::E_TEXT_SMALL, COL_DROP_X, ty, "claws open");
  pros::screen::set_eraser(BG);
}

void drawTable() {
  using namespace ui;
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(ROW_X0, 32, ROW_X1, ROW_Y0 - 1);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, ROW_X0 + 10, 33, "LEVEL");
  pros::screen::print(pros::E_TEXT_SMALL, COL_LIFT_X, 33, "LIFT");
  pros::screen::set_pen(ORANGE);
  pros::screen::print(pros::E_TEXT_SMALL, COL_DROP_X, 33, "DROP");
  for (int i = 0; i < LIFT_LEVEL_COUNT; i++)
    drawRow(i);
}

// Position bar for the whole travel: the lift (filled), each level's LIFT tick
// (white, the selected one cyan) and DROP tick (orange).
void drawGauge() {
  using namespace ui;
  const int w = PANEL_X1 - 255;
  auto xOf = [&](double deg) { return 255 + (int)(w * clampAngle(deg) / LIFT_UPPER_LIMIT); };
  pros::screen::set_pen(AXIS);
  pros::screen::fill_rect(255, GAUGE_Y0 + 2, PANEL_X1, GAUGE_Y1 - 2);
  pros::screen::set_pen(goingTo || jogHeld ? ORANGE : GREEN);
  pros::screen::fill_rect(255, GAUGE_Y0 + 2, xOf(liveAngle()), GAUGE_Y1 - 2);
  for (int i = 0; i < LIFT_LEVEL_COUNT; i++) {
    pros::screen::set_pen(i == selected ? CYAN : 0xAAAAAA);
    int x = xOf(liftLevelAngles[i]);
    pros::screen::draw_line(x, GAUGE_Y0, x, GAUGE_Y1);
    if (liftHasDrop(i)) {
      pros::screen::set_pen(ORANGE);
      x = xOf(liftDropAngles[i]);
      pros::screen::draw_line(x, GAUGE_Y0 + 2, x, GAUGE_Y1);
    }
  }
}

const char *stateWord() {
  if (dropping)
    return "DROPPING";
  if (jogHeld)
    return "JOGGING";
  if (goingTo)
    return "MOVING";
  return "READY";
}

void drawLiveAngle() {
  using namespace ui;
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(PANEL_X0, 32, PANEL_X1, GAUGE_Y1 + 1);
  pros::screen::set_eraser(BG);
  bool busy = goingTo || jogHeld;
  pros::screen::set_pen(busy ? ORANGE : GREEN);
  pros::screen::print(pros::E_TEXT_LARGE, 255, 34, "%.1f", liveAngle());
  pros::screen::set_pen(busy ? ORANGE : CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 395, 34, "%s", stateWord());
  const bool o1 = clawOpen(claw);
  pros::screen::set_pen(o1 ? GREEN : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 395, 47, "claw %s", o1 ? "open" : "shut");
  pros::screen::set_pen(liftLimitSwitch.get_value() ? GREEN : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 395, 60, "limit switch %s", liftLimitSwitch.get_value() ? "on" : "off");
  drawGauge();
}

void drawClawButtons() {
  using namespace ui;
  bool o1 = clawOpen(claw);
  drawButton(kClaw1Btn, "CLAW", o1 ? 0x143a1e : CARD, o1 ? GREEN : GRAY);
}

void drawActionButtons() {
  using namespace ui;
  bool hasDrop = liftHasDrop(selected);
  drawButton(kGoBtn, "GO TO   B", 0x143a1e, 0xFFFFFF);
  drawButton(kSaveBtn, "SAVE LIFT  A", 0x1c2838, CYAN);
  drawButton(kDropBtn, hasDrop ? "DROP   >" : "OPEN CLAWS >", 0x3a2a14, ORANGE);
  drawButton(kSaveDropBtn, "SAVE DROP  <", 0x1c2838, ORANGE, hasDrop);
  drawButton(kJogUpBtn, "JOG +10", CARD, 0xFFFFFF);
  drawButton(kJogDownBtn, "JOG -10", CARD, 0xFFFFFF);
  drawButton(kHomeBtn, "HOME", CARD, ORANGE);
  drawClawButtons();
}

void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 206, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 211, "%s", msg);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 226, "R1/R2 jog  UP level  DOWN claws  X home");
  pros::screen::set_eraser(BG);
  drawButton(kSaveCodeBtn, "SAVE CODE", 0x143a1e, GREEN);
}

void drawAll() {
  using namespace ui;
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  pros::screen::set_eraser(BG);
  drawTitle();
  drawBackButton();
  pros::screen::set_eraser(BG);
  drawTable();
  drawLiveAngle();
  drawActionButtons();
  drawFooter();
}

// The controller LCD takes about one line per 50 ms, so lines rotate.
void drawControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120)
    return;
  lastDraw = pros::millis();
  switch (line) {
  case 0:
    controller.print(0, 0, "LIFT CAL %d %-6s ", selected + 1, liftLevelName(selected));
    break;
  case 1:
    controller.print(1, 0, "Now %7.1f      ", liveAngle());
    break;
  default:
    if (liftHasDrop(selected))
      controller.print(2, 0, "L%5.0f D%5.0f   ", liftLevelAngles[selected],
                       liftDropAngles[selected]);
    else
      controller.print(2, 0, "L%5.0f  no drop ", liftLevelAngles[selected]);
    break;
  }
  line = (line + 1) % 3;
}

// ── actions ──────────────────────────────────────────────────────────────
void select(int level) {
  level = std::clamp(level, 0, LIFT_LEVEL_COUNT - 1);
  if (level == selected)
    return;
  int old = selected;
  selected = level;
  drawRow(old);
  drawRow(selected);
  drawTitle();
  drawActionButtons();
  drawGauge();
}

void stopLift() {
  goingTo = false;
  jogHeld = false;
  dropping = false;
  lift.move_velocity(0); // brake mode HOLD keeps it where it is
}

void goToSelected() {
  dropping = false;
  goTarget = levelAngle(selected, false);
  goingTo = true;
  lift.move_absolute(goTarget, JOG_VELOCITY);
  setMsg("going to %s lift angle %.1f", liftLevelName(selected), goTarget);
}

// GROUND: just open the claws. Any other level: go to its drop angle, and the
// main loop opens the claws once the lift has arrived.
void dropSelected() {
  if (!liftHasDrop(selected)) {
    stopLift();
    liftOpenClaws();
    drawClawButtons();
    setMsg("%s: claws opened", liftLevelName(selected));
    return;
  }
  dropLevel = selected;
  dropStartMs = pros::millis();
  dropping = true;
  goTarget = levelAngle(dropLevel, true);
  goingTo = true;
  dropVel = 0; // the main loop eases the lift there
  setMsg("dropping at %s (%.1f), claws open on arrival", liftLevelName(dropLevel), goTarget);
}

void saveSelected(int level, bool drop) {
  level = std::clamp(level, 0, LIFT_LEVEL_COUNT - 1);
  if (drop && !liftHasDrop(level)) {
    setMsg("%s has no drop angle: the claws just open", liftLevelName(level));
    return;
  }
  double &slot = drop ? liftDropAngles[level] : liftLevelAngles[level];
  slot = clampAngle(liveAngle());
  setMsg("saved %s %s = %.1f", liftLevelName(level), drop ? "drop" : "lift", slot);
  drawRow(level);
  drawGauge();
}

void setLevel(int level, double deg, bool drop) {
  if (level < 0 || level >= LIFT_LEVEL_COUNT || !std::isfinite(deg))
    return;
  if (drop && !liftHasDrop(level)) {
    setMsg("%s has no drop angle: the claws just open", liftLevelName(level));
    return;
  }
  double &slot = drop ? liftDropAngles[level] : liftLevelAngles[level];
  slot = clampAngle(deg);
  setMsg("%s %s set to %.1f", liftLevelName(level), drop ? "drop" : "lift", slot);
  drawRow(level);
  drawGauge();
}

void jogBy(double deg) {
  dropping = false;
  goTarget = clampAngle(liveAngle() + deg);
  goingTo = true;
  lift.move_absolute(goTarget, JOG_VELOCITY);
  setMsg("jog to %.1f", goTarget);
}

// which is ignored (the robot has one claw; the laptop tools still send 0 / 1). state: 0 extend, 1 retract, 2 toggle.
void setClaw(int which, int state) {
  auto apply = [&](pros::adi::Pneumatics &c) {
    bool extend = state == 2 ? !c.is_extended() : state == 0;
    if (extend)
      c.extend();
    else
      c.retract();
  };
  (void)which;
  apply(claw);
  setMsg("claw %s", clawOpen(claw) ? "open" : "shut");
  drawClawButtons();
}

void toggleAllClaws() {
  // all open -> shut them; otherwise open them
  if (clawOpen(claw))
    setClaw(0, LIFT_CLAW_OPEN_RETRACTED ? 0 : 1);
  else
    setClaw(0, LIFT_CLAW_OPEN_RETRACTED ? 1 : 0);
}

// SAVE CODE: the brain can't write files, so it asks the laptop window (which
// edits src/lift_calibration.cpp) by bumping saveReq in the state line.
void requestSaveCode() {
  saveReq++;
  saveWaiting = true;
  saveAskedMs = pros::millis();
  setMsg("saving to code via the laptop...");
}

void homeLiftHere() {
  stopLift();
  setMsg("homing...");
  drawFooter();
  sendState(true);
  homeLift();
  setMsg("homed, angle 0");
}

} // namespace

void liftCalibrationControl() {
  liftHold();
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("LIFT CAL ACTIVE\n");
  wasTouched = false;
  selected = 0;
  stopLift();
  setMsg("homing lift...");
  drawAll();
  sendState(true);
  homeLift(); // angles are measured from the limit switch, so zero it first
  setMsg("jog to a height, pick a level, then SAVE");
  drawAll();
  sendState(true);

  while (true) {
    sendState();

    // Taps: physical, or from the laptop mirror.
    pros::screen_touch_status_s_t ts = pros::screen::touch_status();
    bool press = ts.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = ts.x, y = ts.y;
    if (ts.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (ts.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    if (!press && takeRemoteTouch(x, y))
      press = true;

    bool doGo = false, doSave = false, doHome = false, doDrop = false, doSaveDrop = false;
    if (press) {
      if (x >= kBackX0 && x <= kBackX1 && y >= kBackY0 && y <= kBackY1) {
        stopLift();
        controller.rumble(".");
        printf("LIFT CAL EXIT\n");
        return; // back to the driver menu
      }
      for (int i = 0; i < LIFT_LEVEL_COUNT; i++)
        if (inBox(rowBox(i), x, y))
          select(i);
      if (inBox(kGoBtn, x, y))
        doGo = true;
      else if (inBox(kSaveBtn, x, y))
        doSave = true;
      else if (inBox(kDropBtn, x, y))
        doDrop = true;
      else if (inBox(kSaveDropBtn, x, y))
        doSaveDrop = true;
      else if (inBox(kJogUpBtn, x, y))
        jogBy(JOG_STEP_DEG);
      else if (inBox(kJogDownBtn, x, y))
        jogBy(-JOG_STEP_DEG);
      else if (inBox(kHomeBtn, x, y))
        doHome = true;
      else if (inBox(kClaw1Btn, x, y))
        setClaw(1, 2);
      else if (inBox(kSaveCodeBtn, x, y))
        requestSaveCode();
    }

    // Controller buttons (and the same keys sent from the laptop mirror).
    // UP steps through the levels (wrapping round); DOWN works the claws.
    if (btnUp.pressed() || takeRemoteKey(RK_UP))
      select((selected + 1) % LIFT_LEVEL_COUNT);
    if (btnDown.pressed() || takeRemoteKey(RK_DOWN))
      toggleAllClaws();
    if (btnA.pressed() || takeRemoteKey(RK_A))
      doSave = true;
    if (btnB.pressed() || takeRemoteKey(RK_B))
      doGo = true;
    if (btnX.pressed() || takeRemoteKey(RK_X))
      doHome = true;
    if (btnRight.pressed() || takeRemoteKey(RK_RIGHT))
      doDrop = true;
    if (btnLeft.pressed() || takeRemoteKey(RK_LEFT))
      doSaveDrop = true;
    if (btnL1.pressed() || takeRemoteKey(RK_L1))
      setClaw(1, 2);
    if (btnL2.pressed() || takeRemoteKey(RK_L2))
      setClaw(2, 2);
    if (btnY.pressed() || takeRemoteKey(RK_Y))
      toggleAllClaws();

    // The laptop window.
    RemoteLiftCommand cmd;
    while (takeRemoteLiftCommand(cmd)) {
      switch (cmd.kind) {
      case RemoteLiftCommand::SEL:
        select(cmd.index);
        break;
      case RemoteLiftCommand::SET:
        setLevel(cmd.index, cmd.value, false);
        break;
      case RemoteLiftCommand::SET_DROP:
        setLevel(cmd.index, cmd.value, true);
        break;
      case RemoteLiftCommand::GO_DROP:
        select(cmd.index);
        doDrop = true;
        break;
      case RemoteLiftCommand::SAVE_DROP:
        saveSelected(cmd.index, true);
        break;
      case RemoteLiftCommand::GO:
        select(cmd.index);
        doGo = true;
        break;
      case RemoteLiftCommand::SAVE:
        saveSelected(cmd.index, false);
        break;
      case RemoteLiftCommand::JOG:
        jogBy(std::clamp((double)cmd.value, -100.0, 100.0));
        break;
      case RemoteLiftCommand::HOME:
        doHome = true;
        break;
      case RemoteLiftCommand::CLAW:
        setClaw(std::clamp(cmd.index, 0, 2), std::clamp((int)cmd.value, 0, 2));
        break;
      case RemoteLiftCommand::SAVED:
        saveWaiting = false;
        if (cmd.index == 1)
          setMsg("saved: the table is written into the code");
        else
          setMsg("save FAILED: the laptop could not write the code");
        break;
      case RemoteLiftCommand::STOP:
        stopLift();
        setMsg("stopped at %.1f", liveAngle());
        break;
      }
    }

    if (doHome)
      homeLiftHere();
    if (doGo)
      goToSelected();
    if (doDrop)
      dropSelected();
    if (doSave)
      saveSelected(selected, false);
    if (doSaveDrop)
      saveSelected(selected, true);

    // R1 / R2 held: drive the lift by hand, within its limits. Letting go
    // stops it; that cancels any GO TO / DROP the held buttons interrupted.
    bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    if (liftLimitPressed())
      lift.tare_position_all(); // the limit switch is home: angle 0
    if (up && liveAngle() < LIFT_UPPER_LIMIT) {
      goingTo = false;
      dropping = false;
      jogHeld = true;
      lift.move_velocity(JOG_VELOCITY);
    } else if (down && !liftLimitPressed()) {
      goingTo = false;
      dropping = false;
      jogHeld = true;
      lift.move_velocity(-JOG_VELOCITY);
    } else if (jogHeld) {
      stopLift();
    }

    uint32_t nowTick = pros::millis();
    int dt = (int)std::min<uint32_t>(nowTick - lastTickMs, 50);
    lastTickMs = nowTick;
    if (dropping) {
      // controlled approach to the drop angle; the claws open once it has arrived
      if (liftControlledStep(goTarget, dropVel, dt)) {
        goingTo = false;
        dropping = false;
        liftOpenClaws();
        drawClawButtons();
        setMsg("dropped at %s (%.1f): claws open", liftLevelName(dropLevel), liveAngle());
      } else if (nowTick - dropStartMs > DROP_TIMEOUT_MS) {
        // never got there (blocked?): don't open the claws over the wrong spot
        stopLift();
        setMsg("drop timed out at %.1f: claws NOT opened", liveAngle());
      }
    } else if (goingTo && std::fabs(liveAngle() - goTarget) <= kArriveTolDeg) {
      goingTo = false;
      setMsg("at %.1f", liveAngle());
    }

    if (saveWaiting && pros::millis() - saveAskedMs > 2500) {
      saveWaiting = false;
      setMsg("no laptop answered: open the laptop window to write the code");
    }

    // The loop ticks every 10 ms so the controlled drop is smooth; the screen
    // only needs redrawing a few times a second.
    static uint32_t lastDrawMs = 0;
    if (nowTick - lastDrawMs >= 40) {
      lastDrawMs = nowTick;
      drawLiveAngle();
      static char shownMsg[80] = "";
      if (strcmp(shownMsg, msg) != 0) { // the footer only changes with the message
        strcpy(shownMsg, msg);
        drawFooter();
      }
      drawControllerUI();
    }
    pros::delay(10);
  }
}
