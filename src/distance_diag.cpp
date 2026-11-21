#include "distance_diag.hpp"
#include "driver_menu.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/serial.hpp"
#include <algorithm>
#include <cstdint>
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
const int AXIS = 0x2D2D38;
const int FOOTER_BG = 0x18181F;

const uint32_t STALE_MS = 1000;      // no frame for this long counts as stale
const int MAX_RANGE_MM = 1200;       // VL53L0X range, used to scale the bar
const int VALUE_OUT_OF_RANGE = -1;   // sent by the ESP32 when nothing is in range
const int VALUE_SENSOR_ERROR = -2;   // sent by the ESP32 when the VL53L0X did not start
const int FRAME_RATE_WINDOW_MS = 1000;
const int SENSOR_COUNT = 3;          // VL53L0X sensors on the ESP32

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

// Layout (480x240): header, three side cards, calibration panel, footer.
const Rect kBack = {400, 10, 470, 34};
const Rect kCard[SENSOR_COUNT] = {{6, 46, 156, 150}, {165, 46, 315, 150}, {324, 46, 474, 150}};
const Rect kPanel = {5, 154, 475, 205};
const Rect kMinus = {10, 172, 40, 200};
const Rect kPlus = {124, 172, 154, 200};
const Rect kSet = {160, 172, 236, 200};
const Rect kClear = {242, 172, 304, 200};
const Rect kSrc = {310, 172, 380, 200};
const Rect kSave = {388, 172, 468, 200};
const Rect kReset = {404, 210, 472, 236};

const char *kSideName[SENSOR_COUNT] = {"BACK", "LEFT", "RIGHT"};
const int REF_MIN_MM = 20, REF_MAX_MM = 1000, REF_STEP_MM = 10;

// Calibration, kept while the program runs. `pend` is the sensor (S0..S2 on the ESP32) the user has picked for
// each side, shown live; SAVE copies it to `src`, which is what robot code reads (see distance_diag.hpp).
// `offset` is added to the raw reading. The pending picks always stay a permutation (one sensor per side).
struct SideCfg {
  int src;
  int pend;
  int offset;
};
SideCfg g_cfg[SENSOR_COUNT] = {{0, 0, 0}, {1, 1, 0}, {2, 2, 0}};

bool dirty() {
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (g_cfg[i].pend != g_cfg[i].src)
      return true;
  return false;
}

// Gives `side` the sensor `src`; whichever side had it gets this side's old sensor.
void assignSource(int side, int src) {
  if (side < 0 || side >= SENSOR_COUNT || src < 0 || src >= SENSOR_COUNT)
    return;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (i != side && g_cfg[i].pend == src)
      g_cfg[i].pend = g_cfg[side].pend;
  g_cfg[side].pend = src;
}

void saveAssignments() {
  for (int i = 0; i < SENSOR_COUNT; i++)
    g_cfg[i].src = g_cfg[i].pend;
  printf("DISTANCE SAVED: BACK=S%d LEFT=S%d RIGHT=S%d\n", g_cfg[0].src, g_cfg[1].src, g_cfg[2].src);
}

// Commands from the laptop window ("DIST ..." lines, see distanceRemoteCommand), queued by the listener task
// and applied by the page loop.
struct RemoteCmd {
  enum Kind { SEL, REF, SETREF, CLEAR, SRC, SAVE } kind;
  int a, b;
};
pros::Mutex g_cmdMutex;
RemoteCmd g_cmds[16];
int g_cmdCount = 0;

int calibrated(int raw, int offset) { return raw >= 0 ? std::max(0, raw + offset) : raw; }

// What has arrived over the link so far.
struct Link {
  int value[SENSOR_COUNT] = {VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR}; // last values: mm, or -1 / -2
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
  int v[SENSOR_COUNT];
  int got = std::sscanf(line, "D:%d,%d,%d", &v[0], &v[1], &v[2]);
  if (got == 1) { // legacy single-sensor frame: the other two are unknown
    v[1] = v[2] = VALUE_SENSOR_ERROR;
  } else if (got != SENSOR_COUNT) {
    L.badLines++;
    return;
  }
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (v[i] < VALUE_SENSOR_ERROR) {
      L.badLines++;
      return;
    }
  }
  for (int i = 0; i < SENSOR_COUNT; i++)
    L.value[i] = v[i];
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

// State of one sensor given the link is alive.
State sensorState(int value) {
  if (value == VALUE_SENSOR_ERROR)
    return State::NOT_FOUND;
  if (value == VALUE_OUT_OF_RANGE)
    return State::OUT_OF_RANGE;
  return State::OK;
}

// Overall state: link problems first, then OK if any sensor reads, else the best of the rest.
State classify(const Link &L, uint32_t nowMs) {
  if (L.bytes == 0)
    return State::NO_DATA;
  if (L.frames == 0)
    return State::BAD_DATA;
  if (nowMs - L.lastFrameMs > STALE_MS)
    return State::STALE;
  bool anyRange = false;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    State st = sensorState(L.value[i]);
    if (st == State::OK)
      return State::OK;
    if (st == State::OUT_OF_RANGE)
      anyRange = true;
  }
  return anyRange ? State::OUT_OF_RANGE : State::NOT_FOUND;
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
  case State::NOT_FOUND: return "ESP32 is alive: no VL53L0X answers (SDA 21, SCL 22)";
  case State::OUT_OF_RANGE: return "ESP32 and sensors fine: move an object closer";
  case State::OK: return "tap a card, set its SRC sensor, then SAVE";
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

// Rounded button with its label centred (small font is about 7 px per character).
void drawButton(const Rect &r, const char *label, uint32_t bg, uint32_t fg) {
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 6, bg);
  pros::screen::set_eraser(bg);
  pros::screen::set_pen(fg);
  int w = (int)std::strlen(label) * 7;
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + ((r.x1 - r.x0) - w) / 2, r.y0 + ((r.y1 - r.y0) - 12) / 2, "%s", label);
}

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 40, 8, CARD);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_MEDIUM, 16, 7, "DISTANCE");
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 16, 25, "3x VL53L0X  port %d", DISTANCE_PORT);
  drawButton(kBack, "< BACK", AXIS, 0xDDDDDD);
  drawButton(kReset, "RESET", AXIS, 0xDDDDDD);
}

void drawStatus(State s) {
  uint32_t color = stateColor(s);
  fillRoundedRect(150, 9, 390, 36, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0x000000);
  const char *name = stateName(s);
  int w = (int)std::strlen(name) * 11; // medium font is about 11 px per character
  pros::screen::print(pros::E_TEXT_MEDIUM, 150 + (240 - w) / 2, 12, "%s", name);
}

void drawCard(int side, const Link &L, bool live, bool selected) {
  const Rect &r = kCard[side];
  int raw = L.value[g_cfg[side].pend];
  State ss = live ? sensorState(raw) : State::NO_DATA;
  bool valid = ss == State::OK;
  uint32_t accent = ss == State::NOT_FOUND ? RED : valid ? GREEN : GRAY;
  int shown = calibrated(raw, g_cfg[side].offset);

  if (selected)
    fillRoundedRect(r.x0 - 2, r.y0 - 2, r.x1 + 2, r.y1 + 2, 10, CYAN);
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  pros::screen::set_eraser(CARD);

  pros::screen::set_pen(selected ? CYAN : 0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, r.x0 + 10, r.y0 + 6, "%s", kSideName[side]);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, r.x1 - 30, r.y0 + 10, "S%d", g_cfg[side].pend);

  pros::screen::set_pen(valid ? 0xFFFFFF : GRAY);
  if (valid)
    pros::screen::print(pros::E_TEXT_LARGE, r.x0 + 10, r.y0 + 28, "%d", shown);
  else
    pros::screen::print(pros::E_TEXT_LARGE, r.x0 + 10, r.y0 + 28, "--");
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, r.x1 - 30, r.y0 + 44, "mm");

  // range bar
  int bx0 = r.x0 + 10, bx1 = r.x1 - 10, by = r.y0 + 70;
  pros::screen::set_pen(AXIS);
  pros::screen::fill_rect(bx0, by, bx1, by + 8);
  if (valid) {
    int w = (int)((bx1 - bx0) * (float)std::min(shown, MAX_RANGE_MM) / MAX_RANGE_MM);
    pros::screen::set_pen(GREEN);
    pros::screen::fill_rect(bx0, by, bx0 + w, by + 8);
  }

  // state tag and offset
  const char *tag = valid ? "OK" : ss == State::OUT_OF_RANGE ? "OUT OF RANGE" : ss == State::NOT_FOUND ? "NO SENSOR" : "NO DATA";
  pros::screen::set_pen(accent);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 10, r.y0 + 84, "%s", tag);
  pros::screen::set_pen(g_cfg[side].offset ? ORANGE : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, r.x1 - 62, r.y0 + 84, "%+dmm", g_cfg[side].offset);
}

void drawPanel(const Link &L, int sel, int ref, bool live) {
  fillRoundedRect(kPanel.x0, kPanel.y0, kPanel.x1, kPanel.y1, 8, CARD);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 14, 157, "CALIBRATE %s: hold a target at the reference, tap SET REF", kSideName[sel]);

  drawButton(kMinus, "-", AXIS, 0xFFFFFF);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, 58, 178, "%4d mm", ref);
  drawButton(kPlus, "+", AXIS, 0xFFFFFF);

  bool canSet = live && sensorState(L.value[g_cfg[sel].pend]) == State::OK;
  drawButton(kSet, "SET REF", canSet ? GREEN : AXIS, canSet ? 0x000000 : GRAY);
  drawButton(kClear, "CLEAR", AXIS, g_cfg[sel].offset ? ORANGE : GRAY);
  char src[12];
  std::snprintf(src, sizeof(src), "SRC S%d", g_cfg[sel].pend);
  drawButton(kSrc, src, AXIS, CYAN);
  drawButton(kSave, dirty() ? "SAVE" : "SAVED", dirty() ? ORANGE : AXIS, dirty() ? 0x000000 : GRAY);
}

void drawFooter(const Link &L, State s, uint32_t nowMs) {
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 208, 400, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(GRAY);
  uint32_t age = L.frames ? nowMs - L.lastFrameMs : 0;
  pros::screen::print(pros::E_TEXT_SMALL, 10, 211, "frames %u  bad %u  %.1f Hz  last %u ms", (unsigned)L.frames,
                      (unsigned)L.badLines, L.fps, (unsigned)age);
  pros::screen::set_pen(L.portError ? RED : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 225, "%s", L.portError ? "serial port error: check DISTANCE_PORT" : stateHint(s));
}

void drawReadout(const Link &L, State s, uint32_t nowMs, int sel, int ref) {
  bool live = s == State::OK || s == State::OUT_OF_RANGE || s == State::NOT_FOUND;
  drawStatus(s);
  for (int i = 0; i < SENSOR_COUNT; i++)
    drawCard(i, L, live, i == sel);
  drawPanel(L, sel, ref, live);
  drawFooter(L, s, nowMs);
}

// State line for the laptop window (tools/distance_calibration.py), picked up by tools/remote_touch.py:
//   RUI|DISTANCE|..|BTN:..|DIST:<sel>,<ref>,<state>,<dirty>,<saved x3>,<pending x3>,<offset x3>,<raw x3>,<fps>|MSG:..
void sendState(const Link &L, State s, int sel, int ref) {
  static char lastLine[700] = "";
  static uint32_t lastMs = 0;
  char btns[300];
  int n = std::snprintf(btns, sizeof(btns), "%d,%d,%d,%d,BACK;", kBack.x0, kBack.y0, kBack.x1, kBack.y1);
  for (int i = 0; i < SENSOR_COUNT; i++)
    n += std::snprintf(btns + n, sizeof(btns) - n, "%d,%d,%d,%d,%s;", kCard[i].x0, kCard[i].y0, kCard[i].x1,
                       kCard[i].y1, kSideName[i]);
  const struct {
    const Rect *r;
    const char *l;
  } b[] = {{&kMinus, "-"}, {&kPlus, "+"}, {&kSet, "SET REF"}, {&kClear, "CLEAR"}, {&kSrc, "SRC"}, {&kSave, "SAVE"}};
  for (const auto &e : b)
    n += std::snprintf(btns + n, sizeof(btns) - n, "%d,%d,%d,%d,%s;", e.r->x0, e.r->y0, e.r->x1, e.r->y1, e.l);
  char line[700];
  std::snprintf(line, sizeof(line),
                "RUI|DISTANCE|%s|BTN:%s|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0|FOOTC:0|PLOT:|TRAIL:|SEL:%d"
                "|ACTIVE:-1,-1|PLAN:|PATH:|DIST:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f|MSG:%s",
                kSideName[sel], btns, sel, sel, ref, (int)s, dirty() ? 1 : 0, g_cfg[0].src, g_cfg[1].src,
                g_cfg[2].src, g_cfg[0].pend, g_cfg[1].pend, g_cfg[2].pend, g_cfg[0].offset, g_cfg[1].offset,
                g_cfg[2].offset, L.value[0], L.value[1], L.value[2], L.fps, stateName(s));
  uint32_t now = pros::millis();
  if (std::strcmp(line, lastLine) == 0 && now - lastMs < 1000)
    return;
  std::strcpy(lastLine, line);
  lastMs = now;
  std::printf("%s\n", line);
}

// ── Background reader ────────────────────────────────────────────────────────
// Robot code (autonomous, odomReset) needs readings without the DISTANCE page open. One task owns the
// serial port and keeps the last RING frames; the page takes the port over while it is open (paused flag).
pros::Serial &sharedSerial() {
  static pros::Serial s((std::uint8_t)DISTANCE_PORT, DISTANCE_BAUD);
  return s;
}

struct Sample {
  int v[SENSOR_COUNT];
  uint32_t t;
};
const int RING = 16;
Sample g_ring[RING];
int g_head = 0, g_count = 0;
pros::Mutex g_ringMutex;
volatile bool g_bgPaused = false;
bool g_readerStarted = false;

void readerLoop(void *) {
  pros::Serial &serial = sharedSerial();
  Link link;
  link.windowStartMs = pros::millis();
  char buf[48];
  size_t n = 0;
  uint32_t seen = 0;
  while (true) {
    if (!g_bgPaused) {
      pump(serial, link, buf, sizeof(buf), n, pros::millis());
      if (link.frames != seen) {
        seen = link.frames;
        g_ringMutex.take();
        Sample &s = g_ring[g_head];
        for (int i = 0; i < SENSOR_COUNT; i++)
          s.v[i] = link.value[i];
        s.t = link.lastFrameMs;
        g_head = (g_head + 1) % RING;
        g_count = std::min(g_count + 1, RING);
        g_ringMutex.give();
      }
    } else {
      n = 0; // the page is reading; drop any half line
    }
    pros::delay(5);
  }
}

} // namespace

void distanceStartReader() {
  if (g_readerStarted)
    return;
  g_readerStarted = true;
  static pros::Task task(readerLoop, nullptr, "distance reader");
  (void)task;
}

int distanceReadMm(DistanceSide side, int samples, int maxAgeMs) {
  const int idx = g_cfg[(int)side].src;
  const uint32_t now = pros::millis();
  int vals[RING];
  int n = 0;
  g_ringMutex.take();
  for (int k = 0; k < std::min(samples, g_count); k++) {
    const Sample &s = g_ring[(g_head - 1 - k + 2 * RING) % RING];
    if (now - s.t > (uint32_t)maxAgeMs)
      break;
    if (s.v[idx] >= 0)
      vals[n++] = s.v[idx];
  }
  g_ringMutex.give();
  if (n == 0)
    return -1;
  std::nth_element(vals, vals + n / 2, vals + n);
  return distanceCalibrated(side, vals[n / 2]);
}

int distanceSensorIndex(DistanceSide side) { return g_cfg[(int)side].src; }
int distanceOffsetMm(DistanceSide side) { return g_cfg[(int)side].offset; }
void setDistanceOffsetMm(DistanceSide side, int offsetMm) { g_cfg[(int)side].offset = offsetMm; }
void distanceRemoteCommand(const char *rest) {
  RemoteCmd c = {RemoteCmd::SEL, 0, 0};
  if (std::sscanf(rest, "SEL %d", &c.a) == 1)
    c.kind = RemoteCmd::SEL;
  else if (std::sscanf(rest, "REF %d", &c.a) == 1)
    c.kind = RemoteCmd::REF;
  else if (std::strncmp(rest, "SETREF", 6) == 0)
    c.kind = RemoteCmd::SETREF;
  else if (std::strncmp(rest, "CLEAR", 5) == 0)
    c.kind = RemoteCmd::CLEAR;
  else if (std::sscanf(rest, "SRC %d %d", &c.a, &c.b) == 2)
    c.kind = RemoteCmd::SRC;
  else if (std::strncmp(rest, "SAVE", 4) == 0)
    c.kind = RemoteCmd::SAVE;
  else
    return;
  g_cmdMutex.take();
  if (g_cmdCount < 16)
    g_cmds[g_cmdCount++] = c;
  g_cmdMutex.give();
}

int distanceCalibrated(DistanceSide side, int rawMm) { return calibrated(rawMm, g_cfg[(int)side].offset); }

void distanceDiagControl() {
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("DISTANCE DIAG ACTIVE (port %d, %d baud)\n", DISTANCE_PORT, DISTANCE_BAUD);

  // take the port over from the background reader while this page is open
  struct Pause {
    Pause() { g_bgPaused = true; }
    ~Pause() { g_bgPaused = false; }
  } pauseReader;
  pros::delay(20);
  pros::Serial &serial = sharedSerial();
  serial.flush();

  Link link;
  link.windowStartMs = pros::millis();
  char buf[48];
  size_t n = 0;
  State lastPrinted = State::NO_DATA;
  bool first = true;
  int sel = 0;    // selected side: 0 BACK, 1 LEFT, 2 RIGHT
  int ref = 100;  // reference distance for SET REF, mm

  drawFrame();
  bool wasTouched = false;
  while (true) {
    uint32_t nowMs = pros::millis();
    pump(serial, link, buf, sizeof(buf), n, nowMs);
    State s = classify(link, nowMs);

    // print the state to `pros terminal` when it changes, so a wiring fault is on record
    if (first || s != lastPrinted) {
      printf("DISTANCE %s | frames %u bad %u bytes %u | raw %d,%d,%d\n", stateName(s),
             (unsigned)link.frames, (unsigned)link.badLines, (unsigned)link.bytes, link.value[0], link.value[1], link.value[2]);
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

    // set the reference reading on the selected side: offset = ref - raw
    auto setRef = [&]() {
      int raw = link.value[g_cfg[sel].pend];
      if (s != State::STALE && raw >= 0) {
        g_cfg[sel].offset = ref - raw;
        controller.rumble(".");
        printf("DISTANCE CAL %s: raw %d -> %d mm, offset %+d\n", kSideName[sel], raw, ref, g_cfg[sel].offset);
      }
    };
    auto save = [&]() {
      saveAssignments();
      controller.rumble(".");
    };

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
      for (int i = 0; i < SENSOR_COUNT; i++)
        if (inRect(kCard[i], x, y))
          sel = i;
      if (inRect(kMinus, x, y))
        ref = std::max(REF_MIN_MM, ref - REF_STEP_MM);
      if (inRect(kPlus, x, y))
        ref = std::min(REF_MAX_MM, ref + REF_STEP_MM);
      if (inRect(kClear, x, y))
        g_cfg[sel].offset = 0;
      if (inRect(kSrc, x, y))
        assignSource(sel, (g_cfg[sel].pend + 1) % SENSOR_COUNT);
      if (inRect(kSet, x, y))
        setRef();
      if (inRect(kSave, x, y))
        save();
    }

    // commands from the laptop window
    RemoteCmd cmds[16];
    int count = 0;
    g_cmdMutex.take();
    for (; count < g_cmdCount; count++)
      cmds[count] = g_cmds[count];
    g_cmdCount = 0;
    g_cmdMutex.give();
    for (int i = 0; i < count; i++) {
      switch (cmds[i].kind) {
      case RemoteCmd::SEL: sel = std::clamp(cmds[i].a, 0, SENSOR_COUNT - 1); break;
      case RemoteCmd::REF: ref = std::clamp(cmds[i].a, REF_MIN_MM, REF_MAX_MM); break;
      case RemoteCmd::SETREF: setRef(); break;
      case RemoteCmd::CLEAR: g_cfg[sel].offset = 0; break;
      case RemoteCmd::SRC: assignSource(cmds[i].a, cmds[i].b); break;
      case RemoteCmd::SAVE: save(); break;
      }
    }

    sendState(link, s, sel, ref);
    drawReadout(link, s, nowMs, sel, ref);
    pros::delay(50);
  }
}
