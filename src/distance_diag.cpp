// -----------------------------------------------------------------------------
// DISTANCE page -- see include/distance_diag.hpp. Reads the ESP32's frames on a smart port, shows
// whether the VL53L0X chain works, and calibrates an offset. Drawn on a Canvas like the other pages.
// -----------------------------------------------------------------------------

#include "distance_diag.hpp"
#include "canvas.hpp"
#include "claw_auto.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/serial.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// -- Palette (same as the driver menu) --
const std::uint32_t BG = 0x0A0A0F, CARD = 0x161B22, CYAN = 0x00F0FF, ORANGE = 0xFF8C00, GREEN = 0x2ECC71;
const std::uint32_t RED = 0xFF4D4D, GRAY = 0x777777, AXIS = 0x2D2D38, FOOTER_BG = 0x18181F, WHITE = 0xFFFFFF;

const std::uint32_t STALE_MS = 1000;   // no frame for this long counts as stale
const int MAX_RANGE_MM = 1200;         // VL53L0X range, used to scale the bar
const int VALUE_NO_TARGET = -1;
const int VALUE_SENSOR_ERROR = -2;
const int SLOTS = 3;                   // values per frame, fixed by the wire format
const int N = DISTANCE_SENSOR_COUNT;   // sensors shown
static_assert(DISTANCE_SENSOR_COUNT >= 1 && DISTANCE_SENSOR_COUNT <= 3, "DISTANCE_SENSOR_COUNT is 1 to 3");

// ─── What has arrived over the link ──────────────────────────────────────────
struct Link {
    int filtered[SLOTS] = {VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR};
    int raw[SLOTS] = {VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR, VALUE_SENSOR_ERROR};
    bool haveRaw = false;          // an R: line has been seen
    std::uint32_t lastFrameMs = 0;
    std::uint32_t frames = 0;      // frames parsed since the counters were reset
    std::uint32_t badLines = 0;    // lines that did not parse
    std::uint32_t bytes = 0;       // bytes received
    bool portError = false;
    float fps = 0;
    std::uint32_t windowStartMs = 0, windowFrames = 0;
};

pros::Mutex mtx;
Link link;
int offsetMm[SLOTS] = {DISTANCE_DEFAULT_OFFSET_MM, DISTANCE_DEFAULT_OFFSET_MM, DISTANCE_DEFAULT_OFFSET_MM};
bool readerStarted = false;

// Parses "<tag>:a,b,c". A single value (an older one-sensor sketch) fills the first slot.
bool parseValues(const char *line, char tag, int out[SLOTS]) {
    if (line[0] != tag || line[1] != ':') return false;
    int v[SLOTS];
    int got = std::sscanf(line + 2, "%d,%d,%d", &v[0], &v[1], &v[2]);
    if (got < 1) return false;
    for (int i = got; i < SLOTS; i++) v[i] = VALUE_SENSOR_ERROR; // fewer sensors than slots: the rest are unused
    for (int i = 0; i < SLOTS; i++)
        if (v[i] < VALUE_SENSOR_ERROR) return false;
    for (int i = 0; i < SLOTS; i++) out[i] = v[i];
    return true;
}

void handleLine(Link &L, const char *line, std::uint32_t nowMs) {
    int v[SLOTS];
    if (parseValues(line, 'D', v)) {
        for (int i = 0; i < SLOTS; i++) L.filtered[i] = v[i];
        L.lastFrameMs = nowMs;
        L.frames++;
        L.windowFrames++;
    } else if (parseValues(line, 'R', v)) {
        for (int i = 0; i < SLOTS; i++) L.raw[i] = v[i];
        L.haveRaw = true;
    } else {
        L.badLines++;
    }
}

// Reads everything waiting on the port and parses each complete line.
void pump(const pros::Serial &serial, Link &L, char *buf, size_t cap, size_t &n, std::uint32_t nowMs) {
    std::int32_t avail = serial.get_read_avail();
    if (avail < 0 || avail == INT32_MAX) { // PROS_ERR: nothing is configured on this port
        L.portError = true;
        return;
    }
    L.portError = false;
    while (avail-- > 0) {
        std::int32_t c = serial.read_byte();
        if (c < 0) break;
        L.bytes++;
        if (c == '\n') {
            buf[n] = '\0';
            if (n > 0) handleLine(L, buf, nowMs);
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
    if (nowMs - L.windowStartMs >= 1000) {
        L.fps = L.windowFrames * 1000.0f / (nowMs - L.windowStartMs);
        L.windowFrames = 0;
        L.windowStartMs = nowMs;
    }
}

// ─── Calibration offsets, kept on the SD card ────────────────────────────────
const char *kCalPath = "/usd/distance_cal.txt";

void loadOffsets() {
    FILE *f = std::fopen(kCalPath, "r");
    if (!f) return;
    char line[64];
    while (std::fgets(line, sizeof(line), f)) {
        int i, v;
        if (std::sscanf(line, "OFFSET %d %d", &i, &v) == 2 && i >= 0 && i < SLOTS) offsetMm[i] = std::clamp(v, -1000, 1000);
    }
    std::fclose(f);
}

bool saveOffsets() {
    FILE *f = std::fopen(kCalPath, "w");
    if (!f) return false;
    for (int i = 0; i < SLOTS; i++) std::fprintf(f, "OFFSET %d %d\n", i, offsetMm[i]);
    std::fclose(f);
    return true;
}

void readerTask(void *) {
    pros::Serial serial(DISTANCE_PORT, DISTANCE_BAUD);
    char buf[96];
    size_t n = 0;
    while (true) {
        mtx.take();
        pump(serial, link, buf, sizeof(buf), n, pros::millis());
        mtx.give();
        pros::delay(5);
    }
}

Link snapshot() {
    mtx.take();
    Link copy = link;
    mtx.give();
    return copy;
}

// ─── Status ──────────────────────────────────────────────────────────────────
enum class State { NO_DATA, BAD_DATA, STALE, NOT_FOUND, OUT_OF_RANGE, OK };

State sensorState(int value) {
    if (value == VALUE_SENSOR_ERROR) return State::NOT_FOUND;
    if (value == VALUE_NO_TARGET) return State::OUT_OF_RANGE;
    return State::OK;
}

// Link problems first; then OK if any sensor reads, else the best of the rest.
State classify(const Link &L, std::uint32_t nowMs) {
    if (L.bytes == 0) return State::NO_DATA;
    if (L.frames == 0) return State::BAD_DATA;
    if (nowMs - L.lastFrameMs > STALE_MS) return State::STALE;
    bool anyRange = false;
    for (int i = 0; i < N; i++) {
        State st = sensorState(L.filtered[i]);
        if (st == State::OK) return State::OK;
        if (st == State::OUT_OF_RANGE) anyRange = true;
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
    case State::NO_DATA: return "Check RS-485 A/B, common GND, smart port 2 and the ESP32's power.";
    case State::BAD_DATA: return "Bytes arrive but do not parse: check the baud rate, or swap RS-485 A and B.";
    case State::STALE: return "Frames stopped: the ESP32 reset, or a cable came loose.";
    case State::NOT_FOUND: return "The ESP32 is alive but no VL53L0X answers: check SDA (21) and SCL (22).";
    case State::OUT_OF_RANGE: return "ESP32 and sensor are fine: put an object within 1.2 m.";
    case State::OK: return "Working. Offset -60 by default: a reading of 60 mm shows as 0. SET REF re-calibrates.";
    }
    return "";
}

std::uint32_t stateColor(State s) {
    switch (s) {
    case State::OK: return GREEN;
    case State::OUT_OF_RANGE:
    case State::STALE: return ORANGE;
    default: return RED;
    }
}

// ─── Layout (480 x 240) ──────────────────────────────────────────────────────
struct Rect {
    int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

Canvas frame;
const Rect kBack = {396, 14, 468, 40};
const Rect kClaw = {345, 57, 466, 81}; // claw state; a tap toggles the claw by hand
const int kCardY0 = 88, kCardY1 = 170;
const Rect kPanel = {5, 176, 475, 214};
const Rect kMinus = {12, 181, 44, 209};
const Rect kPlus = {128, 181, 160, 209};
const Rect kSet = {170, 181, 252, 209};
const Rect kClear = {260, 181, 330, 209};
const Rect kReset = {338, 181, 466, 209};

Rect cardRect(int i) {
    const int x0 = 15, x1 = 465, gap = 8;
    int w = (x1 - x0 - (N - 1) * gap) / N;
    int cx = x0 + i * (w + gap);
    return {cx, kCardY0, cx + w, kCardY1};
}

void button(const Rect &r, const char *label, std::uint32_t bg, std::uint32_t fg) {
    frame.fillRoundRect(r.x0, r.y0, r.x1, r.y1, 6, bg);
    frame.text(FONT_SMALL, (r.x0 + r.x1) / 2, r.y0 + (r.y1 - r.y0 - 12) / 2, label, fg, Canvas::CENTER);
}

int calibrated(int value, int offset) { return value >= 0 ? std::max(0, value + offset) : value; }

void drawCard(int i, const Link &L, bool live, bool selected) {
    Rect r = cardRect(i);
    State ss = live ? sensorState(L.filtered[i]) : State::NO_DATA;
    bool valid = ss == State::OK;
    int shown = calibrated(L.filtered[i], offsetMm[i]);

    frame.card(r.x0, r.y0, r.x1, r.y1, 8, CARD, selected ? CYAN : AXIS);
    char title[16];
    std::snprintf(title, sizeof(title), N > 1 ? "SENSOR %d" : "VL53L0X", i + 1);
    frame.text(FONT_SMALL, r.x0 + 12, r.y0 + 8, title, selected ? CYAN : WHITE);

    const char *tag = valid ? "OK" : ss == State::OUT_OF_RANGE ? "OUT OF RANGE" : ss == State::NOT_FOUND ? "NO SENSOR" : "NO DATA";
    std::uint32_t accent = valid ? GREEN : (ss == State::NOT_FOUND ? RED : ss == State::OUT_OF_RANGE ? ORANGE : GRAY);
    const bool wide = N == 1;  // one wide card, or up to three narrow ones laid out differently
    if (wide) frame.text(FONT_SMALL, r.x1 - 12, r.y0 + 8, tag, accent, Canvas::RIGHT);
    else frame.text(FONT_SMALL, r.x0 + 12, r.y0 + 46, tag, accent);

    char buf[24];
    if (valid) std::snprintf(buf, sizeof(buf), "%d", shown);
    else std::snprintf(buf, sizeof(buf), "--");
    frame.text(FONT_LARGE, r.x0 + 12, r.y0 + 24, buf, valid ? WHITE : GRAY);
    frame.text(FONT_SMALL, r.x0 + 12 + frame.textWidth(FONT_LARGE, buf) + 6, r.y0 + 34, "mm", GRAY);

    // raw reading and offset, to the right of the number (side by side when the card is wide)
    int tx = wide ? r.x0 + 200 : r.x0 + 12;
    int ty = wide ? r.y0 + 26 : r.y0 + 57;
    if (L.haveRaw && live) {
        if (L.raw[i] >= 0) std::snprintf(buf, sizeof(buf), "raw %d", L.raw[i]);
        else std::snprintf(buf, sizeof(buf), "raw --");
    } else {
        std::snprintf(buf, sizeof(buf), "raw n/a");
    }
    frame.text(FONT_SMALL, tx, ty, buf, GRAY);
    if (wide) std::snprintf(buf, sizeof(buf), "offset %+d mm", offsetMm[i]);
    else std::snprintf(buf, sizeof(buf), "%+d mm", offsetMm[i]);
    frame.text(FONT_SMALL, wide ? tx + 110 : r.x1 - 12, ty, buf, offsetMm[i] != DISTANCE_DEFAULT_OFFSET_MM ? ORANGE : GRAY,
               wide ? Canvas::LEFT : Canvas::RIGHT);

    // range bar
    int bx0 = r.x0 + 12, bx1 = r.x1 - 12, by = wide ? r.y1 - 16 : r.y1 - 7, bh = wide ? 6 : 4;
    frame.fillRect(bx0, by, bx1, by + bh, AXIS);
    if (valid) {
        int w = (int)((bx1 - bx0) * (float)std::min(shown, MAX_RANGE_MM) / MAX_RANGE_MM);
        frame.fillRect(bx0, by, bx0 + std::max(w, 2), by + bh, GREEN);
    }
}

void draw(const Link &L, std::uint32_t nowMs, int sel, int ref, const char *note) {
    State st = classify(L, nowMs);
    bool live = st == State::OK || st == State::OUT_OF_RANGE || st == State::NOT_FOUND;

    frame.fillRect(0, 0, Canvas::W, Canvas::H, BG);
    frame.fillRoundRect(5, 5, 475, 50, 8, CYAN);
    char crumb[40];
    std::snprintf(crumb, sizeof(crumb), "VL53L0X on smart port %d", DISTANCE_PORT);
    frame.text(FONT_MEDIUM, 20, 9, "DISTANCE", 0x000000);
    frame.text(FONT_SMALL, 20, 31, crumb, 0x1A1A1A);
    frame.fillRoundRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
    frame.text(FONT_SMALL, kBack.x0 + 12, kBack.y0 + 7, "< BACK", 0xCCCCCC);

    // status pill + link numbers
    frame.fillRoundRect(15, 57, 190, 81, 8, stateColor(st));
    frame.text(FONT_SMALL, 102, 63, stateName(st), 0x000000, Canvas::CENTER);
    char info[64];
    std::snprintf(info, sizeof(info), "%.0f/s  %lu frames  %lu bad", L.fps, (unsigned long)L.frames, (unsigned long)L.badLines);
    frame.text(FONT_SMALL, 200, 63, info, GRAY);
    // the claw: it closes by itself when the reading is 0..10 mm (see the page loop); tap to toggle it
    bool clawOn = clawPistonExtended();
    frame.fillRoundRect(kClaw.x0, kClaw.y0, kClaw.x1, kClaw.y1, 8, clawOn ? GREEN : AXIS);
    frame.text(FONT_SMALL, (kClaw.x0 + kClaw.x1) / 2, kClaw.y0 + 6, clawOn ? "CLAW ON" : "CLAW OFF", clawOn ? 0x000000 : WHITE,
               Canvas::CENTER);

    for (int i = 0; i < N; i++) drawCard(i, L, live, i == sel);

    // calibration panel
    frame.card(kPanel.x0, kPanel.y0, kPanel.x1, kPanel.y1, 8, CARD, AXIS);
    button(kMinus, "-", AXIS, WHITE);
    char refText[24];
    std::snprintf(refText, sizeof(refText), "REF %d mm", ref);
    frame.text(FONT_SMALL, (kMinus.x1 + kPlus.x0) / 2, kPanel.y0 + 13, refText, WHITE, Canvas::CENTER);
    button(kPlus, "+", AXIS, WHITE);
    bool canSet = st == State::OK && sensorState(L.filtered[sel]) == State::OK;
    button(kSet, "SET REF", canSet ? GREEN : AXIS, canSet ? 0x000000 : GRAY);
    button(kClear, "DEFAULT", AXIS, offsetMm[sel] != DISTANCE_DEFAULT_OFFSET_MM ? ORANGE : GRAY);
    button(kReset, "RESET COUNTERS", AXIS, 0xDDDDDD);

    // footer: a note about the last action, or what the current status means
    frame.fillRect(0, 218, Canvas::W, Canvas::H, FOOTER_BG);
    frame.text(FONT_SMALL, 10, 223, note[0] ? note : stateHint(st), note[0] ? GREEN : (st == State::OK ? GRAY : stateColor(st)));
    frame.present();

    // The laptop window draws this page from one line:
    //   RUI|DISTANCE|<link state>|<card state>|<fps>|<frames>|<bad>|<shown mm>|<raw mm>|<offset>|<default?>|<ref>|
    //                <claw on?>|<can set ref?>|<footer is a note?>|<footer text>          (-1 = no value; the card state is -1 = no data)
    {
        State cs = live ? sensorState(L.filtered[sel]) : State::NO_DATA;
        int shown = cs == State::OK ? calibrated(L.filtered[sel], offsetMm[sel]) : -1;
        int rawMm = (L.haveRaw && live && L.raw[sel] >= 0) ? L.raw[sel] : -1;
        bool canSet = st == State::OK && cs == State::OK;
        char line[300];
        std::snprintf(line, sizeof(line), "RUI|DISTANCE|%d|%d|%.0f|%lu|%lu|%d|%d|%d|%d|%d|%d|%d|%d|%s", (int)st, (int)cs, L.fps,
                      (unsigned long)L.frames, (unsigned long)L.badLines, shown, rawMm, offsetMm[sel],
                      offsetMm[sel] == DISTANCE_DEFAULT_OFFSET_MM ? 1 : 0, ref, clawPistonExtended() ? 1 : 0, canSet ? 1 : 0,
                      note[0] ? 1 : 0, note[0] ? note : stateHint(st));
        driverMenuPublish(line);
    }
}

} // namespace

// ─── Public API ──────────────────────────────────────────────────────────────
void distanceStartReader() {
    if (readerStarted) return;
    readerStarted = true;
    loadOffsets();
    static pros::Task reader(readerTask, nullptr, "distance link");
}

int distanceReadMm(int sensor, int maxAgeMs) {
    if (sensor < 0 || sensor >= SLOTS) return VALUE_SENSOR_ERROR;
    Link L = snapshot();
    if (L.frames == 0 || (int)(pros::millis() - L.lastFrameMs) > maxAgeMs) return VALUE_NO_TARGET;
    return calibrated(L.filtered[sensor], offsetMm[sensor]);
}

int distanceReadRawMm(int sensor, int maxAgeMs) {
    if (sensor < 0 || sensor >= SLOTS) return VALUE_SENSOR_ERROR;
    Link L = snapshot();
    if (L.frames == 0 || (int)(pros::millis() - L.lastFrameMs) > maxAgeMs) return VALUE_NO_TARGET;
    if (L.haveRaw && L.raw[sensor] >= 0) return calibrated(L.raw[sensor], offsetMm[sensor]);
    return calibrated(L.filtered[sensor], offsetMm[sensor]);
}

int distanceOffsetMm(int sensor) { return sensor >= 0 && sensor < SLOTS ? offsetMm[sensor] : 0; }

void setDistanceOffsetMm(int sensor, int offset) {
    if (sensor < 0 || sensor >= SLOTS) return;
    offsetMm[sensor] = std::clamp(offset, -1000, 1000);
    saveOffsets();
}

// ─── The page ────────────────────────────────────────────────────────────────
void distanceDiagControl() {
    distanceStartReader();
    pros::delay(100); // let the dashboard task notice the screen is taken

    int sel = 0, ref = 100;
    char note[64] = "";
    std::uint32_t noteUntilMs = 0, lastDrawMs = 0;
    bool wasTouched = true; // a touch already down when the page opens is not a tap
    ClawAutoGrab autoClaw;  // closes the claw when sensor 1 reads 0..10 mm (once per object)

    while (true) {
        if (driverMenuLeaveRequested()) return; // top-right corner held, or LEAVE from the laptop

        pros::screen_touch_status_s_t t = pros::screen::touch_status();
        bool down = t.touch_status != pros::E_TOUCH_RELEASED;
        bool tap = down && !wasTouched;
        wasTouched = down;
        int tx = t.x, ty = t.y;
        int rx, ry;
        if (takeRemoteTouch(rx, ry)) { // a click in the laptop window counts like a tap
            tap = true;
            tx = rx;
            ty = ry;
        }
        std::uint32_t now = pros::millis();

        if (tap) {
            Link L = snapshot();
            if (inRect(kBack, tx, ty)) {
                controller.rumble(".");
                return;
            }
            for (int i = 0; i < N; i++)
                if (inRect(cardRect(i), tx, ty)) sel = i;
            if (inRect(kClaw, tx, ty)) {
                toggleClawPiston();
                std::snprintf(note, sizeof(note), clawPistonExtended() ? "Claw ON (by hand)" : "Claw OFF (by hand)");
                noteUntilMs = now + 2000;
            }
            if (inRect(kMinus, tx, ty)) ref = std::max(20, ref - 10);
            if (inRect(kPlus, tx, ty)) ref = std::min(MAX_RANGE_MM, ref + 10);
            if (inRect(kSet, tx, ty)) {
                if (classify(L, now) == State::OK && sensorState(L.filtered[sel]) == State::OK) {
                    // offset = what it should read - what it reads (without any earlier offset)
                    setDistanceOffsetMm(sel, ref - L.filtered[sel]);
                    std::snprintf(note, sizeof(note), "Offset set to %+d mm (saved)", offsetMm[sel]);
                } else {
                    std::snprintf(note, sizeof(note), "No reading to calibrate with: fix the status first.");
                }
                noteUntilMs = now + 3000;
            }
            if (inRect(kClear, tx, ty)) {
                setDistanceOffsetMm(sel, DISTANCE_DEFAULT_OFFSET_MM);
                std::snprintf(note, sizeof(note), "Offset back to the default %+d mm (saved)", DISTANCE_DEFAULT_OFFSET_MM);
                noteUntilMs = now + 3000;
            }
            if (inRect(kReset, tx, ty)) {
                mtx.take();
                link.frames = link.badLines = link.bytes = 0;
                link.fps = 0;
                link.windowFrames = 0;
                link.windowStartMs = now;
                mtx.give();
                std::snprintf(note, sizeof(note), "Counters reset");
                noteUntilMs = now + 2000;
            }
        }
        // Auto-claw: the reading (with its offset) is 0..10 mm -> the claw turns on. The lift is not touched here.
        if (autoClaw.update(distanceReadMm(0), true, clawPistonExtended(), false, now)) {
            setClawPiston(true);
            std::snprintf(note, sizeof(note), "In the claw (0-10 mm): claw ON");
            noteUntilMs = now + 3000;
            controller.rumble(".");
        }
        if (note[0] && now >= noteUntilMs) note[0] = '\0';

        if (now - lastDrawMs >= 100 || tap) {
            lastDrawMs = now;
            draw(snapshot(), now, sel, ref, note);
        }
        pros::delay(20);
    }
}
