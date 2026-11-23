#include "odom_reset_cal.hpp"
#include "distance_diag.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "odom_reset_math.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int RED = 0xFF3131;
const int GRAY = 0x777777;
const int GRID = 0x121217;
const int AXIS = 0x2d2d38;
const int RAY = 0x3A4A5A;
const int PINK = 0xFF6B81;
const int FOOTER_BG = 0x18181F;
const float MAX_CORRECTION_IN = 4.0f; // same default as odomReset()
const float FIELD_HALF = odomreset::FIELD_HALF_IN;

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
const Rect kBtn[3] = {{10, 168, 105, 206}, {112, 168, 207, 206}, {214, 168, 309, 206}};
const int READOUT_Y = 58, READOUT_H = 108;

// Field map, right side: a square, one tile-grid line every 24 in, +Y up, +X right.
const Rect kMap = {322, 58, 472, 208};
const int MAP_C_X = (kMap.x0 + kMap.x1) / 2, MAP_C_Y = (kMap.y0 + kMap.y1) / 2;
const float MAP_PPI = (kMap.x1 - kMap.x0) / 2.0f / FIELD_HALF; // pixels per inch

void toPixel(float x, float y, int &px, int &py) {
  const float lim = (kMap.x1 - kMap.x0) / 2 - 1;
  px = MAP_C_X + (int)std::clamp(x * MAP_PPI, -lim, lim);
  py = MAP_C_Y - (int)std::clamp(y * MAP_PPI, -lim, lim);
}

// Last reset: the pose before it (for UNDO) and what it changed.
lemlib::Pose undoPose{0, 0, 0};
bool haveUndo = false;
bool haveLast = false;
float lastDh = 0, lastDx = 0, lastDy = 0;
lemlib::Pose lastBefore{0, 0, 0}; // shown as a gray dot on the map after a reset
char status[80] = "tap RESET with the robot on the field";

float wrap360(float deg) {
  deg = std::fmod(deg, 360.0f);
  return deg < 0 ? deg + 360.0f : deg;
}

void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

void drawButton(const Rect &r, const char *label, uint32_t accent) {
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  fillRoundedRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, accent);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 12, r.y0 + 13, "%s", label);
}

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "ODOM RESET");
  pros::screen::set_pen(0x1a1a1a);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "tap the map to move odometry there");
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
  drawButton(kBtn[0], "RESET+HDG", ORANGE);
  drawButton(kBtn[1], "RESET XY", CYAN);
  drawButton(kBtn[2], "UNDO", RED);
}

void drawStatus() {
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 212, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", status);
}

void setStatus(const char *fmt, double a = 0, double b = 0, double c = 0) {
  snprintf(status, sizeof(status), fmt, a, b, c);
  drawStatus();
}

// "523" / "OOR" / "ERR" for one wall distance.
void mmText(int mm, char *out, size_t n) {
  if (mm >= 0)
    snprintf(out, n, "%d", mm);
  else if (mm == -1)
    snprintf(out, n, "OOR");
  else
    snprintf(out, n, "ERR");
}

// Tilt from the left+right span alone: both look along one line in opposite directions, so their
// summed length is the field width / cos(tilt). Same maths as odomreset::correct(). Returns false
// when either sensor is invalid or the span does not look like two field walls.
bool sensorTilt(int leftMm, int rightMm, float &tiltDeg) {
  if (leftMm < 0 || rightMm < 0)
    return false;
  const float span =
      (leftMm + rightMm) / odomreset::MM_PER_IN + SENSOR_LEFT_OFFSET_IN + SENSOR_RIGHT_OFFSET_IN;
  if (std::fabs(span - odomreset::FIELD_IN) > 6.0f)
    return false;
  tiltDeg = std::acos(std::fmin(1.0f, odomreset::FIELD_IN / span)) * 180.0f / 3.14159265f;
  return true;
}

// ─── Map ────────────────────────────────────────────────────────────────────
void drawField() {
  pros::screen::set_pen(CARD);
  pros::screen::fill_rect(kMap.x0, kMap.y0, kMap.x1, kMap.y1);
  for (float v = -FIELD_HALF; v <= FIELD_HALF + 0.5f; v += 24.0f) {
    int px, py;
    toPixel(v, 0, px, py);
    pros::screen::set_pen(std::fabs(v) < 0.5f ? AXIS : GRID);
    pros::screen::draw_line(px, kMap.y0, px, kMap.y1);
    toPixel(0, v, px, py);
    pros::screen::set_pen(std::fabs(v) < 0.5f ? AXIS : GRID);
    pros::screen::draw_line(kMap.x0, py, kMap.x1, py);
  }
  pros::screen::set_pen(AXIS);
  pros::screen::draw_rect(kMap.x0, kMap.y0, kMap.x1, kMap.y1);
}

// Robot dot with a heading line (heading is compass-style: 0 = +Y, clockwise).
void drawRobot(const lemlib::Pose &p, uint32_t color) {
  int ox, oy, tx, ty;
  toPixel(p.x, p.y, ox, oy);
  const float rad = p.theta * 3.14159265f / 180.0f;
  toPixel(p.x + std::sin(rad) * 14.0f, p.y + std::cos(rad) * 14.0f, tx, ty);
  pros::screen::set_pen(color);
  pros::screen::draw_line(ox, oy, tx, ty);
  pros::screen::draw_circle(ox, oy, 4);
}

void drawCross(float x, float y, uint32_t color) {
  int px, py;
  toPixel(x, y, px, py);
  pros::screen::set_pen(color);
  pros::screen::draw_line(px - 6, py, px + 6, py);
  pros::screen::draw_line(px, py - 6, px, py + 6);
  pros::screen::draw_circle(px, py, 3);
}

// What each wall sensor sees, as a line from the robot to where the beam lands.
void drawRay(const lemlib::Pose &p, int mm, float offsetDeg, float mountIn) {
  if (mm < 0)
    return;
  const float rad = (p.theta + offsetDeg) * 3.14159265f / 180.0f;
  const float len = mm / odomreset::MM_PER_IN + mountIn;
  int ox, oy, ex, ey;
  toPixel(p.x, p.y, ox, oy);
  toPixel(p.x + std::sin(rad) * len, p.y + std::cos(rad) * len, ex, ey);
  pros::screen::set_pen(RAY);
  pros::screen::draw_line(ox, oy, ex, ey);
  pros::screen::set_pen(ORANGE);
  pros::screen::draw_circle(ex, ey, 2);
}

// ─── Live view ──────────────────────────────────────────────────────────────
// One snapshot of everything the screen (and the laptop window) shows.
struct Live {
  lemlib::Pose pose{0, 0, 0};
  int f = -1, l = -1, r = -1;
  bool aligned = false, haveEst = false, tiltOk = false;
  odomreset::Result est;
  float wx = 0, wy = 0, dx = 0, dy = 0, st = 0;
};

Live computeLive() {
  Live lv;
  lv.pose = chassis.getPose();
  lv.f = distanceReadMm(DistanceSide::FRONT);
  lv.l = distanceReadMm(DistanceSide::LEFT);
  lv.r = distanceReadMm(DistanceSide::RIGHT);

  // Where the walls say the robot is -- worked out with the same maths as odomReset() but with a
  // huge correction limit, so the offset is visible even when the real reset would reject it.
  const odomreset::Reading rd[3] = {
      {true, lv.f, 0.0f, SENSOR_FRONT_OFFSET_IN},
      {true, lv.l, -90.0f, SENSOR_LEFT_OFFSET_IN},
      {true, lv.r, 90.0f, SENSOR_RIGHT_OFFSET_IN},
  };
  lv.aligned = odomreset::headingAligned(lv.pose.theta);
  lv.est = odomreset::correct(lv.pose.x, lv.pose.y, lv.pose.theta, rd, 3, 1000.0f, false);
  lv.haveEst = lv.aligned && (lv.est.xOk || lv.est.yOk);
  lv.wx = lv.est.xOk ? lv.est.x : lv.pose.x;
  lv.wy = lv.est.yOk ? lv.est.y : lv.pose.y;
  lv.dx = lv.wx - lv.pose.x;
  lv.dy = lv.wy - lv.pose.y;
  lv.tiltOk = sensorTilt(lv.l, lv.r, lv.st);
  return lv;
}

void drawLive(const Live &lv) {
  const lemlib::Pose &pose = lv.pose;
  const int f = lv.f, l = lv.l, r = lv.r;
  const bool aligned = lv.aligned, haveEst = lv.haveEst;
  const odomreset::Result &est = lv.est;
  const float wx = lv.wx, wy = lv.wy, dx = lv.dx, dy = lv.dy;

  // map
  drawField();
  if (haveLast)
    drawRobot(lastBefore, GRAY);
  drawRay(pose, f, 0.0f, SENSOR_FRONT_OFFSET_IN);
  drawRay(pose, l, -90.0f, SENSOR_LEFT_OFFSET_IN);
  drawRay(pose, r, 90.0f, SENSOR_RIGHT_OFFSET_IN);
  if (haveEst)
    drawCross(wx, wy, GREEN);
  drawRobot(pose, CYAN);

  // text
  const float snapped = std::round(pose.theta / 90.0f) * 90.0f;
  const int wall = ((int)std::lround(snapped) % 360 + 360) % 360;
  const float tilt = odomreset::wrap180(pose.theta - snapped);
  char fs[8], ls[8], rs[8];
  mmText(f, fs, sizeof(fs));
  mmText(l, ls, sizeof(ls));
  mmText(r, rs, sizeof(rs));

  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, READOUT_Y, kMap.x0 - 4, READOUT_Y + READOUT_H);
  pros::screen::set_eraser(BG);

  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y, "odom  X %.2f Y %.2f H %.1f", pose.x, pose.y,
                      wrap360(pose.theta));
  if (haveEst) {
    pros::screen::set_pen(GREEN);
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 17, "walls X %s Y %s", est.xOk ? "" : "-", est.yOk ? "" : "-");
    // numbers on their own line so a missing axis does not shift them
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 34, "      %.2f  %.2f  (green +)", wx, wy);
  } else {
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 17, aligned ? "walls: no valid reading" : "walls: heading >10 deg off a wall");
  }

  if (haveEst) {
    const bool apply = (!est.xOk || std::fabs(dx) <= MAX_CORRECTION_IN) && (!est.yOk || std::fabs(dy) <= MAX_CORRECTION_IN);
    pros::screen::set_pen(apply ? GREEN : ORANGE);
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 51, "off dx %+.2f dy %+.2f  %s", dx, dy,
                        apply ? "applies" : "REJECTS >4in");
  }

  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 68, "F %s  L %s  R %s mm", fs, ls, rs);

  float st;
  if (sensorTilt(l, r, st)) {
    pros::screen::set_pen(CYAN);
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 85, "tilt %+.1f (imu) / %.1f (L+R)", tilt, st);
  } else {
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 85, "wall %d tilt %+.1f (L+R: n/a)", wall, tilt);
  }

  pros::screen::set_pen(GRAY);
  if (haveLast)
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 102, "last dH %+.2f dX %+.2f dY %+.2f", lastDh, lastDx, lastDy);
  else
    pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 102, "last: no reset yet");
}

// ─── Laptop link (tools/odom_reset_cal.py, opened by tools/remote_touch.py) ──
// State line, sent at the usual remote rate:
//   RUI|ODOM_RESET|..|ODOMR:<x>,<y>,<theta>,<F>,<L>,<R>,<aligned>,<xOk>,<yOk>,<wx>,<wy>,<tiltOk>,<tilt>,
//        <haveLast>,<lastX>,<lastY>,<lastTheta>,<dH>,<dX>,<dY>,<haveUndo>,<mountF>,<mountL>,<mountR>,<maxCorr>|MSG:<status>
// Commands from the laptop ("ODOMR ..." lines): RESET <fixHeading 0|1>, UNDO, POSE <x> <y> [<heading>].
struct RemoteCmd {
  enum Kind { RESET, UNDO, POSE } kind;
  int a;
  float x, y;
  float th = 0;       // POSE: heading to set (deg), when hasTh
  bool hasTh = false; // POSE without a heading keeps the current one
};
pros::Mutex g_cmdMutex;
RemoteCmd g_cmds[8];
int g_cmdCount = 0;

bool takeCmd(RemoteCmd &out) {
  bool got = false;
  g_cmdMutex.take();
  if (g_cmdCount > 0) {
    out = g_cmds[0];
    for (int i = 1; i < g_cmdCount; i++)
      g_cmds[i - 1] = g_cmds[i];
    g_cmdCount--;
    got = true;
  }
  g_cmdMutex.give();
  return got;
}

void sendState(const Live &lv, bool force = false) {
  static uint32_t lastSend = 0;
  if (!force && pros::millis() - lastSend < (uint32_t)remoteSendIntervalMs())
    return;
  lastSend = pros::millis();
  char line[700];
  std::snprintf(line, sizeof(line),
                "RUI|ODOM_RESET|ODOM RESET|BTN:%d,%d,%d,%d,BACK|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0"
                "|FOOTC:0|PLOT:|TRAIL:|SEL:0|ACTIVE:-1,-1|PLAN:|PATH:"
                "|ODOMR:%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%d,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f,%.2f,%.1f"
                "|MSG:%s",
                kBack.x0, kBack.y0, kBack.x1, kBack.y1, lv.pose.x, lv.pose.y, lv.pose.theta, lv.f, lv.l, lv.r,
                lv.aligned ? 1 : 0, lv.est.xOk ? 1 : 0, lv.est.yOk ? 1 : 0, lv.wx, lv.wy, lv.tiltOk ? 1 : 0, lv.st,
                haveLast ? 1 : 0, lastBefore.x, lastBefore.y, lastBefore.theta, lastDh, lastDx, lastDy,
                haveUndo ? 1 : 0, SENSOR_FRONT_OFFSET_IN, SENSOR_LEFT_OFFSET_IN, SENSOR_RIGHT_OFFSET_IN,
                MAX_CORRECTION_IN, status);
  std::printf("%s\n", line);
}

// ─── Actions ────────────────────────────────────────────────────────────────
void doReset(bool fixHeading) {
  const lemlib::Pose before = chassis.getPose();
  const bool aligned = odomreset::headingAligned(before.theta);
  float st;
  const bool haveSensorTilt =
      sensorTilt(distanceReadMm(DistanceSide::LEFT), distanceReadMm(DistanceSide::RIGHT), st);

  printf("ODOMRESET TEST %s from (%.2f, %.2f, %.2f)\n", fixHeading ? "XY+HDG" : "XY", before.x, before.y,
         before.theta);
  const bool ok = odomReset(true, true, true, MAX_CORRECTION_IN, fixHeading);
  const lemlib::Pose after = chassis.getPose();

  undoPose = before;
  haveUndo = true;
  haveLast = true;
  lastBefore = before;
  lastDh = odomreset::wrap180(after.theta - before.theta);
  lastDx = after.x - before.x;
  lastDy = after.y - before.y;

  if (!aligned)
    setStatus("SKIPPED: heading %.1f is >10 deg off a wall", wrap360(before.theta));
  else if (!ok)
    setStatus("NO CHANGE: no reading used (obstacle / >4 in off)");
  else if (fixHeading && !haveSensorTilt)
    setStatus("OK, x/y only: heading needs L+R seeing both walls");
  else if (fixHeading)
    setStatus("OK: heading %+.2f deg, x %+.2f, y %+.2f", lastDh, lastDx, lastDy);
  else
    setStatus("OK (x/y only): x %+.2f in, y %+.2f in", lastDx, lastDy);
}

void doUndo() {
  if (!haveUndo) {
    setStatus("nothing to undo");
    return;
  }
  chassis.setPose(undoPose.x, undoPose.y, undoPose.theta);
  haveUndo = false;
  haveLast = false;
  printf("ODOMRESET TEST undo -> (%.2f, %.2f, %.2f)\n", undoPose.x, undoPose.y, undoPose.theta);
  setStatus("pose restored to before the last change");
}

// Move odometry to (nx, ny), heading kept. Simulates drift, so RESET XY has something to fix.
// With `th` (the laptop's start pose, like the path planner's) the heading is set as well.
void doSetXY(float nx, float ny, const float *th = nullptr) {
  const lemlib::Pose before = chassis.getPose();
  nx = std::clamp(nx, -FIELD_HALF, FIELD_HALF);
  ny = std::clamp(ny, -FIELD_HALF, FIELD_HALF);
  undoPose = before;
  haveUndo = true;
  haveLast = false;
  chassis.setPose(nx, ny, th ? *th : before.theta);
  if (th) {
    printf("ODOMRESET TEST start pose -> odom (%.2f, %.2f, %.1f)\n", nx, ny, *th);
    setStatus("odom set to (%.1f, %.1f) hdg %.1f: now RESET", nx, ny, wrap360(*th));
  } else {
    printf("ODOMRESET TEST map tap -> odom (%.2f, %.2f)\n", nx, ny);
    setStatus("odom moved to (%.1f, %.1f): now RESET XY", nx, ny);
  }
}

// Tap on the brain's map.
void doMapTap(int x, int y) {
  doSetXY((x - MAP_C_X) / MAP_PPI, (MAP_C_Y - y) / MAP_PPI);
}

} // namespace

void odomResetRemoteCommand(const char *rest) {
  RemoteCmd c = {RemoteCmd::UNDO, 0, 0, 0, 0, false};
  if (std::sscanf(rest, "RESET %d", &c.a) == 1)
    c.kind = RemoteCmd::RESET;
  else if (std::strncmp(rest, "UNDO", 4) == 0)
    c.kind = RemoteCmd::UNDO;
  else if (std::sscanf(rest, "POSE %f %f %f", &c.x, &c.y, &c.th) == 3) {
    c.kind = RemoteCmd::POSE;
    c.hasTh = true;
  } else if (std::sscanf(rest, "POSE %f %f", &c.x, &c.y) == 2)
    c.kind = RemoteCmd::POSE;
  else
    return;
  g_cmdMutex.take();
  if (g_cmdCount < 8)
    g_cmds[g_cmdCount++] = c;
  g_cmdMutex.give();
}

void odomResetCalControl() {
  haveUndo = false;
  haveLast = false;
  snprintf(status, sizeof(status), "tap RESET with the robot on the field");
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("ODOM RESET TEST ACTIVE\n");
  drawFrame();
  drawStatus();

  bool wasTouched = false;
  while (true) {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = touch.x, y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    // A tap from the laptop mirror counts like a real one (see driver_menu.cpp).
    if (!press && takeRemoteTouch(x, y))
      press = true;

    if (press) {
      if (inRect(kBack, x, y)) {
        controller.rumble(".");
        printf("ODOM RESET TEST EXIT\n");
        return; // back to the driver menu
      }
      if (inRect(kBtn[0], x, y))
        doReset(true);
      else if (inRect(kBtn[1], x, y))
        doReset(false);
      else if (inRect(kBtn[2], x, y))
        doUndo();
      else if (inRect(kMap, x, y))
        doMapTap(x, y);
    }

    // commands from the laptop window
    RemoteCmd rc;
    while (takeCmd(rc)) {
      if (rc.kind == RemoteCmd::RESET)
        doReset(rc.a != 0);
      else if (rc.kind == RemoteCmd::UNDO)
        doUndo();
      else
        doSetXY(rc.x, rc.y, rc.hasTh ? &rc.th : nullptr);
    }

    const Live lv = computeLive();
    drawLive(lv);
    sendState(lv);
    pros::delay(100);
  }
}
