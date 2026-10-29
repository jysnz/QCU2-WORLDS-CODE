#include "distance_diag.hpp"
#include "driver_menu.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/serial.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int RED = 0xFF3131;
const int GRAY = 0x777777;
const int AXIS = 0x2D2D38;
const int FOOTER_BG = 0x18181F;

const uint32_t STALE_MS = 1000;      // no frame for this long counts as stale
const int MAX_RANGE_MM = 1200;       // VL53L0X range, used to scale the bar
const int VALUE_OUT_OF_RANGE = -1;   // sent by the ESP32 when nothing is in range
const int VALUE_SENSOR_ERROR = -2;   // sent by the ESP32 when the VL53L0X did not start
const int FRAME_RATE_WINDOW_MS = 1000;

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
const Rect kReset = {10, 178, 150, 208};

// What has arrived over the link so far.
struct Link {
  int value = 0;         // last value received: mm, or -1 / -2 (see above)
  uint32_t lastFrameMs = 0;
  uint32_t frames = 0;   // parsed frames since the page opened
  uint32_t badLines = 0; // lines that did not parse
  uint32_t bytes = 0;    // bytes received
  bool portError = false;
  // frames per second over the last window
  uint32_t windowStartMs = 0;
  uint32_t windowFrames = 0;
  float fps = 0;
};

enum class State { NO_DATA, BAD_DATA, STALE, NOT_FOUND, OUT_OF_RANGE, OK };

void handleLine(Link &L, const char *line, uint32_t nowMs) {
  int value;
  if (std::sscanf(line, "D:%d", &value) != 1 || value < VALUE_SENSOR_ERROR) {
    L.badLines++;
    return;
  }
  L.value = value;
  L.lastFrameMs = nowMs;
  L.frames++;
  L.windowFrames++;
}

// Reads everything waiting on the port and parses each complete line.
void pump(const pros::Serial &serial, Link &L, char *buf, size_t cap, size_t &n, uint32_t nowMs) {
  int32_t avail = serial.get_read_avail();
  if (avail < 0 || avail == INT32_MAX) { // PROS_ERR: nothing is configured on this port
    L.portError = true;
    return;
  }
  L.portError = false;
  while (avail-- > 0) {
    int32_t c = serial.read_byte();
    if (c < 0)
      break;
    L.bytes++;
    if (c == '\n') {
      buf[n] = '\0';
      if (n > 0)
        handleLine(L, buf, nowMs);
      n = 0;
    } else if (c != '\r') {
      if (n + 1 < cap) {
        buf[n++] = (char)c;
      } else {
        n = 0; // runaway line, drop it
        L.badLines++;
      }
    }
  }
  if (nowMs - L.windowStartMs >= (uint32_t)FRAME_RATE_WINDOW_MS) {
    L.fps = L.windowFrames * 1000.0f / (nowMs - L.windowStartMs);
    L.windowFrames = 0;
    L.windowStartMs = nowMs;
  }
}

State classify(const Link &L, uint32_t nowMs) {
  if (L.bytes == 0)
    return State::NO_DATA;
  if (L.frames == 0)
    return State::BAD_DATA;
  if (nowMs - L.lastFrameMs > STALE_MS)
    return State::STALE;
  if (L.value == VALUE_SENSOR_ERROR)
    return State::NOT_FOUND;
  if (L.value == VALUE_OUT_OF_RANGE)
    return State::OUT_OF_RANGE;
  return State::OK;
}

const char *stateName(State s) {
  switch (s) {
  case State::NO_DATA: return "NO DATA";
  case State::BAD_DATA: return "BAD DATA";
  case State::STALE: return "STALE";
  case State::NOT_FOUND: return "SENSOR NOT FOUND";
  case State::OUT_OF_RANGE: return "OUT OF RANGE";
  case State::OK: return "OK";
  }
  return "";
}

const char *stateHint(State s) {
  switch (s) {
  case State::NO_DATA: return "check RS-485 A/B, common GND, port, ESP32 power";
  case State::BAD_DATA: return "bytes arrive but do not parse: check baud rate";
  case State::STALE: return "frames stopped: ESP32 reset or cable loose";
  case State::NOT_FOUND: return "ESP32 is alive: check VL53L0X wiring (SDA 21, SCL 22)";
  case State::OUT_OF_RANGE: return "ESP32 and sensor fine: move an object closer";
  case State::OK: return "sensor chain working";
  }
  return "";
}

uint32_t stateColor(State s) {
  switch (s) {
  case State::OK: return GREEN;
  case State::OUT_OF_RANGE:
  case State::STALE: return ORANGE;
  default: return RED;
  }
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

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "DISTANCE");
  pros::screen::set_pen(0x1A1A1A);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "VL53L0X via ESP32, port %d", DISTANCE_PORT);
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");

  fillRoundedRect(kReset.x0, kReset.y0, kReset.x1, kReset.y1, 8, CARD);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, kReset.x0 + 14, kReset.y0 + 10, "RESET COUNTS");
}

void drawReadout(const Link &L, State s, uint32_t nowMs) {
  // status pill
  uint32_t color = stateColor(s);
  fillRoundedRect(10, 58, 300, 88, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 22, 62, "%s", stateName(s));

  // distance
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(10, 94, 470, 172);
  pros::screen::set_eraser(BG);
  bool valid = s == State::OK;
  pros::screen::set_pen(valid ? 0xFFFFFF : GRAY);
  if (valid) {
    pros::screen::print(pros::E_TEXT_LARGE, 10, 96, "%d mm", L.value);
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 96, "%.1f cm", L.value / 10.0f);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 118, "%.2f in", L.value / 25.4f);
  } else {
    pros::screen::print(pros::E_TEXT_LARGE, 10, 96, "-- mm");
  }

  // bar scaled to the sensor's range
  pros::screen::set_pen(AXIS);
  pros::screen::fill_rect(10, 140, 470, 152);
  if (valid) {
    int w = (int)(460.0f * std::min(L.value, MAX_RANGE_MM) / MAX_RANGE_MM);
    pros::screen::set_pen(GREEN);
    pros::screen::fill_rect(10, 140, 10 + w, 152);
  }
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 156, "0");
  pros::screen::print(pros::E_TEXT_SMALL, 430, 156, "%d mm", MAX_RANGE_MM);

  // counters
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(160, 176, 470, 210);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(GRAY);
  uint32_t age = L.frames ? nowMs - L.lastFrameMs : 0;
  pros::screen::print(pros::E_TEXT_SMALL, 165, 178, "frames %u  bad %u  bytes %u", (unsigned)L.frames,
                      (unsigned)L.badLines, (unsigned)L.bytes);
  pros::screen::print(pros::E_TEXT_SMALL, 165, 194, "%.1f Hz  last %u ms ago  raw %d", L.fps,
                      (unsigned)age, L.value);

  // hint
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 212, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(L.portError ? RED : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", L.portError ? "serial port error: check DISTANCE_PORT" : stateHint(s));
}

} // namespace

void distanceDiagControl() {
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("DISTANCE DIAG ACTIVE (port %d, %d baud)\n", DISTANCE_PORT, DISTANCE_BAUD);

  pros::Serial serial((std::uint8_t)DISTANCE_PORT, DISTANCE_BAUD);
  serial.flush();

  Link link;
  link.windowStartMs = pros::millis();
  char buf[48];
  size_t n = 0;
  State lastPrinted = State::NO_DATA;
  bool first = true;

  drawFrame();
  bool wasTouched = false;
  while (true) {
    uint32_t nowMs = pros::millis();
    pump(serial, link, buf, sizeof(buf), n, nowMs);
    State s = classify(link, nowMs);

    // print the state to `pros terminal` when it changes, so a wiring fault is on record
    if (first || s != lastPrinted) {
      printf("DISTANCE %s | frames %u bad %u bytes %u | raw %d\n", stateName(s),
             (unsigned)link.frames, (unsigned)link.badLines, (unsigned)link.bytes, link.value);
      lastPrinted = s;
      first = false;
    }

    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = touch.x, y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    if (!press && takeRemoteTouch(x, y))
      press = true;

    if (press) {
      if (inRect(kBack, x, y)) {
        controller.rumble(".");
        printf("DISTANCE DIAG EXIT\n");
        return; // back to the driver menu
      }
      if (inRect(kReset, x, y)) {
        // zero the counters only; the live reading and the link state stay as they are
        link.frames = 0;
        link.badLines = 0;
      }
    }

    drawReadout(link, s, nowMs);
    pros::delay(50);
  }
}
