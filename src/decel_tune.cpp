#include "decel_tune.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int GREEN = 0x2ECC71;
const int RED = 0xFF3131;
const int ORANGE = 0xFF8C00;
const int GRAY = 0x777777;
const int FOOTER_BG = 0x18181F;

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
const Rect kRun = {350, 58, 470, 98};
const Rect kReturn = {350, 106, 470, 146};
const Rect kStop = {350, 154, 470, 194};

// settings survive leaving and re-entering the screen
struct Param {
  const char *name;
  double value, lo, hi, step;
  const char *unit;
};
Param params[6] = {
    {"DIST", 48, 12, 120, 6, "in"},
    {"SPEED", 127, 40, 127, 10, ""},
    {"RANGE", 10, 0, 40, 2, "in"},
    {"END", 40, 15, 120, 5, ""},
    {"STEPS", 0, 0, 8, 1, ""},
    {"OWN", 1, 0, 1, 1, ""}, // 1 = our own final approach, 0 = LemLib's (decelOwnFinish)
};
enum { P_DIST, P_SPEED, P_RANGE, P_END, P_STEPS, P_OWN };
const int kParamCount = 6;

const int kRowY0 = 58, kRowH = 24;
Rect rowMinus(int i) { return {240, kRowY0 + i * kRowH, 280, kRowY0 + i * kRowH + 21}; }
Rect rowPlus(int i) { return {290, kRowY0 + i * kRowH, 330, kRowY0 + i * kRowH + 21}; }

void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

void button(const Rect &r, uint32_t color, const char *label, int textDx = 8) {
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + textDx, r.y0 + (r.y1 - r.y0) / 2 - 6, "%s", label);
}

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "DECEL TUNE");
  pros::screen::set_pen(0x1A1A1A);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "clear floor ahead, then tap RUN");
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
}

void drawParams() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, kRowY0 - 2, 340, kRowY0 + kParamCount * kRowH);
  for (int i = 0; i < kParamCount; i++) {
    const int y = kRowY0 + i * kRowH;
    pros::screen::set_eraser(BG);
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 10, y + 6, "%s", params[i].name);
    pros::screen::set_pen(0xFFFFFF);
    if (i == P_STEPS && params[i].value < 2)
      pros::screen::print(pros::E_TEXT_MEDIUM, 90, y, "off");
    else if (i == P_OWN)
      pros::screen::print(pros::E_TEXT_MEDIUM, 90, y, "%s", params[i].value >= 1 ? "own" : "LemLib");
    else
      pros::screen::print(pros::E_TEXT_MEDIUM, 90, y, "%.0f %s", params[i].value, params[i].unit);
    button(rowMinus(i), CARD, "-", 17);
    button(rowPlus(i), CARD, "+", 17);
  }
}

void adjust(int i, int dir) {
  Param &p = params[i];
  p.value = std::clamp(p.value + dir * p.step, p.lo, p.hi);
  if (i == P_STEPS && p.value == 1)
    p.value = dir > 0 ? 2 : 0; // 1 stage is the same as none
}

} // namespace

void decelTuneControl() {
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("DECEL TUNE ACTIVE\n");

  bool wasTouched = false;
  bool haveStart = false;
  lemlib::Pose startPose(0, 0, 0);
  char result[80] = "no run yet";
  uint32_t resultColor = GRAY;
  bool running = false;

  // Mirrors the screen to tools/decel_tune.py (RUI|DECEL_TUNE line), paced like the other screens.
  auto sendState = [&](bool force = false) {
    static uint32_t lastSend = 0, lastActual = 0;
    static char lastLine[400] = "";
    if (!force && pros::millis() - lastSend < (uint32_t)remoteSendIntervalMs())
      return;
    lastSend = pros::millis();
    char line[400];
    snprintf(line, sizeof(line),
             "RUI|DECEL_TUNE|DECEL TUNE|BTN:%d,%d,%d,%d,BACK|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0"
             "|FOOTC:0|PLOT:|TRAIL:|SEL:0|ACTIVE:-1,-1|PLAN:|PATH:|DECEL:%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d|MSG:%s",
             kBack.x0, kBack.y0, kBack.x1, kBack.y1, params[P_DIST].value, params[P_SPEED].value,
             params[P_RANGE].value, params[P_END].value, params[P_STEPS].value, params[P_OWN].value,
             running ? 1 : 0,
             haveStart ? 1 : 0, result);
    if (!force && strcmp(line, lastLine) == 0 && pros::millis() - lastActual < 1000)
      return;
    strcpy(lastLine, line);
    lastActual = pros::millis();
    printf("%s\n", line);
  };

  auto touchPress = [&](int &x, int &y) {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    x = touch.x;
    y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    if (!press && takeRemoteTouch(x, y))
      press = true;
    return press;
  };

  // One drive to (tx, ty) with the current settings; STOP (or any touch on STOP) cancels it.
  auto drive = [&](float tx, float ty, bool forwards) {
    lemlib::MoveToPointParams mp;
    mp.forwards = forwards;
    mp.maxSpeed = (float)params[P_SPEED].value;
    const float range = (float)params[P_RANGE].value;
    const float endSpeed = (float)params[P_END].value;
    const int steps = (int)params[P_STEPS].value;
    decelOwnFinish = params[P_OWN].value >= 1;
    const uint32_t t0 = pros::millis();
    if (steps >= 2)
      moveToPointSmooth(tx, ty, 5000, mp, range, endSpeed, true, steps);
    else
      moveToPointDecel(tx, ty, 5000, mp, range, endSpeed, true);
    pros::delay(10);
    running = true;
    int idleMs = 0; // queued stages leave brief gaps; settled = idle for 150 ms
    bool cancelled = false;
    while (idleMs < 150) {
      idleMs = chassis.isInMotion() ? 0 : idleMs + 10;
      int x, y;
      bool stop = touchPress(x, y) && inRect(kStop, x, y);
      RemoteDecelCommand c;
      while (takeRemoteDecelCommand(c))
        if (c.kind == RemoteDecelCommand::STOP)
          stop = true;
      if (stop) {
        chassis.cancelAllMotions();
        cancelled = true;
        break;
      }
      sendState();
      pros::delay(10);
    }
    running = false;
    const uint32_t ms = pros::millis() - t0 - 150;
    pros::delay(200); // let it roll to a stop before measuring
    const lemlib::Pose p = chassis.getPose();
    const float dx = tx - p.x, dy = ty - p.y;
    const float err = std::hypot(dx, dy);
    // + = past the target, - = short of it, along the direction of travel
    const float travelX = forwards ? std::sin(p.theta * (float)M_PI / 180.0f)
                                   : -std::sin(p.theta * (float)M_PI / 180.0f);
    const float travelY = forwards ? std::cos(p.theta * (float)M_PI / 180.0f)
                                   : -std::cos(p.theta * (float)M_PI / 180.0f);
    const float along = -(dx * travelX + dy * travelY);
    if (cancelled) {
      snprintf(result, sizeof(result), "stopped");
      resultColor = ORANGE;
    } else {
      snprintf(result, sizeof(result), "%.2fs   %s %.1f in   (error %.1f in)", ms / 1000.0f,
               along >= 0 ? "past" : "short", std::fabs(along), err);
      resultColor = std::fabs(along) <= 1.0f ? GREEN : (along > 0 ? RED : ORANGE);
    }
    printf("DECEL RUN dist %.0f speed %.0f range %.0f end %.0f steps %d finish %s -> %s\n",
           params[P_DIST].value, params[P_SPEED].value, params[P_RANGE].value, params[P_END].value,
           steps, decelOwnFinish ? "own" : "lemlib", result);
  };

  drawFrame();
  drawParams();
  sendState(true);
  while (true) {
    int x, y;
    bool wantRun = false, wantReturn = false;
    RemoteDecelCommand cmd;
    bool changed = false;
    while (takeRemoteDecelCommand(cmd)) {
      if (cmd.kind == RemoteDecelCommand::SET && cmd.index >= 0 && cmd.index < kParamCount) {
        Param &p = params[cmd.index];
        p.value = std::clamp((double)cmd.value, p.lo, p.hi);
        if (cmd.index == P_STEPS && p.value == 1)
          p.value = 0;
        changed = true;
      } else if (cmd.kind == RemoteDecelCommand::RUN) {
        wantRun = true;
      } else if (cmd.kind == RemoteDecelCommand::RETURN) {
        wantReturn = true;
      }
    }
    if (changed)
      drawParams();
    if (touchPress(x, y)) {
      if (inRect(kBack, x, y)) {
        controller.rumble(".");
        printf("DECEL TUNE EXIT\n");
        return; // back to the driver menu
      }
      for (int i = 0; i < kParamCount; i++) {
        if (inRect(rowMinus(i), x, y)) {
          adjust(i, -1);
          changed = true;
        } else if (inRect(rowPlus(i), x, y)) {
          adjust(i, +1);
          changed = true;
        }
      }
      if (changed) {
        drawParams();
        printf("DECEL RANGE %.0f END %.0f STEPS %.0f\n", params[P_RANGE].value, params[P_END].value,
               params[P_STEPS].value);
      }
      if (inRect(kRun, x, y))
        wantRun = true;
      else if (inRect(kReturn, x, y))
        wantReturn = true;
    }
    {
      if (wantRun) {
        startPose = chassis.getPose();
        haveStart = true;
        const float h = startPose.theta * (float)M_PI / 180.0f;
        const float d = (float)params[P_DIST].value;
        snprintf(result, sizeof(result), "running...");
        resultColor = CYAN;
        drive(startPose.x + std::sin(h) * d, startPose.y + std::cos(h) * d, true);
      } else if (wantReturn && haveStart) {
        snprintf(result, sizeof(result), "returning...");
        resultColor = CYAN;
        drive(startPose.x, startPose.y, false);
      }
    }
    sendState();

    button(kRun, 0x1E7A45, "RUN", 40);
    button(kReturn, haveStart ? CARD : 0x101015, "RETURN", 28);
    button(kStop, 0x7A1E1E, "STOP", 38);

    pros::screen::set_pen(FOOTER_BG);
    pros::screen::fill_rect(0, 212, 480, 240);
    pros::screen::set_eraser(FOOTER_BG);
    pros::screen::set_pen(resultColor);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", result);

    pros::screen::set_pen(BG);
    pros::screen::fill_rect(0, 200, 480, 211);
    pros::screen::set_eraser(BG);
    pros::screen::set_pen(GRAY);
    // (hint line removed: the six rows need the space)
    pros::delay(30);
  }
}
