// -----------------------------------------------------------------------------
// DRIVER MENU -- see include/driver_menu.hpp for what it is and how to open it.
// Touchscreen home screen shown at the start of opcontrol(); the look (cyan header,
// cards with a cyan accent bar, BACK at the top-right) follows the `debi` branch.
// Everything is drawn on an off-screen Canvas and presented in one copy, like the
// dashboard and the lift tuner, so nothing flickers.
// -----------------------------------------------------------------------------

#include "driver_menu.hpp"
#include "canvas.hpp"
#include "distance_diag.hpp"
#include "fieldctl/fieldctl.hpp"
#include "lift_tuning.hpp"
#include "motors.hpp"
#include "odom_setup.hpp"
#include "pid_tuner.hpp"
#include "laptop_plan.hpp"
#include "path_planner.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "screen_task.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

bool driverMenuActive = false;

// -- Laptop link (tools/remote_touch.py) -------------------------------------------------------
// Rides on the USB cable `pros terminal` uses.
//   robot -> laptop   RUI|<screen>|<data>   a few times a second: what the menu currently shows
//                       RUI|HOME|label~sub;label~sub..           the cards
//                       RUI|TEMPS|title:name,temp,conn,over;..#title:..   motor temperatures
//                       RUI|TEST_CONFIRM|   /   RUI|TEST_RUN|done|aborted|label,port,rev,state;..
//                       RUI|TOOL|PID|PLANNER|LIFT|INTAKE   a full-screen tool is open
//                       RUI|DRIVE|          the menu is closed, the robot is driving
//   laptop -> robot   TOUCH x y     a synthetic tap on the brain screen (brain coordinates, 480x240)
//                     LEAVE         same as holding the top-right corner: leave the open tool
//                     RATE ms       how often to send RUI lines (default 150; slower over radio)
//                     LT ...        passed on to the lift / intake tuner (tools/lift_tuner.py)
// This is the ONLY task that reads the USB input (two readers would steal each other's lines),
// so the lift tuner's "LT" lines arrive through here too. It stands down while VEXNET field
// control is on, because field control reads the same input then.
static pros::Mutex linkMtx;
static std::string linkState = "RUI|DRIVE|"; // until the menu says otherwise: the robot is just driving
static bool linkTouchPending = false;
static int linkTouchX = 0, linkTouchY = 0;
static int linkIntervalMs = 150;
static std::atomic<bool> remoteLeave{false};

static std::string sf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static std::string sf(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

static void publish(const std::string &line) {
    linkMtx.take();
    linkState = line;
    linkMtx.give();
}

static bool takeLinkTouch(int &x, int &y) {
    linkMtx.take();
    bool got = linkTouchPending;
    if (got) {
        x = linkTouchX;
        y = linkTouchY;
        linkTouchPending = false;
    }
    linkMtx.give();
    return got;
}

// A tool that can be mirrored on the laptop publishes its own screen's state here (instead of the plain
// "RUI|TOOL|name|" line runTool() sent when it opened).
void driverMenuPublish(const char *line) { publish(line); }

// -- Hooks for the PID tuner (see driver_menu.hpp) --
static std::atomic<bool> remoteKeys[RK_COUNT];
static std::atomic<bool> remotePidStop{false};
static std::vector<RemotePidCommand> remotePidCmds; // oldest first, guarded by linkMtx
static const char *const kRemoteKeyNames[RK_COUNT] = {"LEFT", "RIGHT", "UP", "DOWN", "A",  "B",
                                                       "X",    "L1",    "L2", "Y",    "R1", "R2"};
bool takeRemoteKey(RemoteKey k) { return remoteKeys[k].exchange(false); }
bool takeRemoteTouch(int &x, int &y) { return takeLinkTouch(x, y); }
int remoteSendIntervalMs() {
    linkMtx.take();
    int ms = linkIntervalMs;
    linkMtx.give();
    return ms;
}
bool takeRemotePidCommand(RemotePidCommand &out) {
    linkMtx.take();
    bool got = !remotePidCmds.empty();
    if (got) {
        out = remotePidCmds.front();
        remotePidCmds.erase(remotePidCmds.begin());
    }
    linkMtx.give();
    return got;
}
bool takeRemotePidStop() { return remotePidStop.exchange(false); }

// "PID <verb> ..." from tools/pid_tuner.py. Returns false if the line is not one of them.
static bool parsePidLine(const char *rest) {
    if (strncmp(rest, "STOP", 4) == 0) {
        remotePidStop = true;
        return true;
    }
    RemotePidCommand c;
    if (sscanf(rest, "SET %d %d %f", &c.mode, &c.index, &c.value) == 3) c.kind = RemotePidCommand::SET;
    else if (sscanf(rest, "MODE %d", &c.mode) == 1) c.kind = RemotePidCommand::MODE;
    else if (sscanf(rest, "SEL %d", &c.index) == 1) c.kind = RemotePidCommand::SEL;
    else if (sscanf(rest, "DIGIT %d", &c.index) == 1) c.kind = RemotePidCommand::DIGIT;
    else if (sscanf(rest, "TEST %d %d", &c.mode, &c.index) == 2) c.kind = RemotePidCommand::TEST;
    else if (sscanf(rest, "SWEEP %d", &c.mode) == 1) c.kind = RemotePidCommand::SWEEP;
    else if (strncmp(rest, "CAL", 3) == 0) c.kind = RemotePidCommand::CAL;
    else return false;
    linkMtx.take();
    if (remotePidCmds.size() < 32) remotePidCmds.push_back(c);
    linkMtx.give();
    return true;
}

static void linkReader(void *) {
    char line[200];
    while (true) {
        if (fieldctl::vexnet()) { // field control reads the input while VEXNET is on
            pros::delay(50);
            continue;
        }
        if (!fgets(line, sizeof(line), stdin)) {
            pros::delay(10);
            continue;
        }
        int a, b;
        if (strncmp(line, "TOUCH ", 6) == 0 && sscanf(line + 6, "%d %d", &a, &b) == 2) {
            linkMtx.take();
            linkTouchPending = true;
            linkTouchX = a;
            linkTouchY = b;
            linkMtx.give();
        } else if (strncmp(line, "LEAVE", 5) == 0) {
            remoteLeave = true;
        } else if (strncmp(line, "PID ", 4) == 0) {
            parsePidLine(line + 4);
        } else if (strncmp(line, "KEY ", 4) == 0) {
            char name[16];
            if (sscanf(line + 4, "%15s", name) == 1)
                for (int k = 0; k < RK_COUNT; k++)
                    if (strcmp(name, kRemoteKeyNames[k]) == 0) remoteKeys[k] = true;
        } else if (strncmp(line, "RATE ", 5) == 0 && sscanf(line + 5, "%d", &a) == 1) {
            linkMtx.take();
            linkIntervalMs = std::clamp(a, 100, 1000);
            linkMtx.give();
        } else if (strncmp(line, "LT ", 3) == 0) {
            liftTuningHandleLine(line);
        } else {
            laptopPlanHandleLine(line); // PLAN / POSE / DRIVE / ROBOT HOME from the laptop's map planner
        }
    }
}

static void linkWriter(void *) {
    while (true) {
        linkMtx.take();
        std::string s = linkState;
        int ms = linkIntervalMs;
        linkMtx.give();
        if (!fieldctl::vexnet()) {
            printf("%s\n", s.c_str());
            liftTuningSendTelemetry(); // only prints while the lift tuner window is open
        }
        pros::delay(ms);
    }
}

// Starts the laptop link (idempotent). main.cpp calls it from initialize(), so the laptop window connects as soon
// as the program runs, not only after the menu has been opened.
static void startLink() {
    static bool started = false;
    if (started) return;
    started = true;
    static pros::Task reader(linkReader, nullptr, "menu link in");
    static pros::Task writer(linkWriter, nullptr, "menu link out");
}

void driverMenuStartLink() { startLink(); }

// Hold the top-right corner of the screen this long to leave a tool.
static const uint32_t kLeaveHoldMs = 800;
static const int kLeaveX0 = 396, kLeaveY1 = 46;

bool driverMenuLeaveRequested() {
    if (remoteLeave.exchange(false)) return true; // "LEAVE" from the laptop link
    static uint32_t since = 0;
    static bool fired = false;
    pros::screen_touch_status_s_t t = pros::screen::touch_status();
    bool inCorner = t.touch_status != pros::E_TOUCH_RELEASED && t.x >= kLeaveX0 && t.y <= kLeaveY1;
    if (!inCorner) {
        since = 0;
        fired = false;
        return false;
    }
    uint32_t now = pros::millis();
    if (!since) since = now;
    if (!fired && now - since >= kLeaveHoldMs) {
        fired = true; // once per hold
        return true;
    }
    return false;
}

namespace {

// -- Palette (debi's) --
const std::uint32_t BG = 0x0A0A0F, CARD = 0x161B22, CYAN = 0x00F0FF, ORANGE = 0xFF8C00, GREEN = 0x2ECC71;
const std::uint32_t RED = 0xFF4D4D, GRAY = 0x777777, AXIS = 0x2D2D38, FOOTER_BG = 0x18181F, WHITE = 0xFFFFFF;

struct Rect {
    int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

Canvas frame;
const Rect kBack = {396, 14, 468, 40};

enum class Screen { HOME, TEMPS, TEST_CONFIRM, TEST_RUN };

// Cleared frame with the rounded header bar every screen shares.
void header(const char *title, const char *crumb, bool back) {
    frame.fillRect(0, 0, Canvas::W, Canvas::H, BG);
    frame.fillRoundRect(5, 5, 475, 50, 8, CYAN);
    frame.text(FONT_MEDIUM, 20, crumb ? 9 : 16, title, 0x000000);
    if (crumb) frame.text(FONT_SMALL, 20, 31, crumb, 0x1A1A1A);
    if (back) {
        frame.fillRoundRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
        frame.text(FONT_SMALL, kBack.x0 + 12, kBack.y0 + 7, "< BACK", 0xCCCCCC);
    }
}

void footer(const char *text, std::uint32_t color) {
    frame.fillRect(0, 218, Canvas::W, Canvas::H, FOOTER_BG);
    frame.text(FONT_SMALL, 10, 223, text, color);
}

// -- HOME: a 3-wide grid of cards --
struct Item {
    const char *label;
    const char *sub;
};
enum { ITEM_DRIVE, ITEM_PID, ITEM_PLANNER, ITEM_TEST, ITEM_LIFT, ITEM_INTAKE, ITEM_TEMPS, ITEM_DISTANCE, ITEM_ODOM, ITEM_COUNT };
const Item kItems[ITEM_COUNT] = {
    {"DRIVE", "joystick driving"},
    {"PID TUNING", "angular + lateral"},
    {"PATH PLANNER", "drive to a point"},
    {"TEST MOTORS", "drivetrain, 2 s each"},
    {"LIFT TUNER", "levels, combos, angles"},
    {"INTAKE TUNER", "combos on L1 / L2"},
    {"MOTOR TEMPS", "every motor, live"},
    {"DISTANCE", "sensor check + calibrate"},
    {"ODOM SETUP", "pose, trackers, calibrate"},
};
const int kGridX0 = 15, kGridX1 = 465, kGridY0 = 58, kRowH = 48, kGap = 4, kCols = 3;

Rect itemRect(int i) {
    int colW = (kGridX1 - kGridX0 - (kCols - 1) * kGap) / kCols;
    int x0 = kGridX0 + (i % kCols) * (colW + kGap);
    int y0 = kGridY0 + (i / kCols) * (kRowH + kGap);
    return {x0, y0, x0 + colW, y0 + kRowH};
}

void drawHome() {
    header("DRIVER MENU", nullptr, false);
    for (int i = 0; i < ITEM_COUNT; i++) {
        Rect r = itemRect(i);
        frame.card(r.x0, r.y0, r.x1, r.y1, 8, CARD, AXIS);
        frame.fillRoundRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, CYAN);
        const Font &f = frame.textWidth(FONT_MEDIUM, kItems[i].label) <= r.x1 - r.x0 - 32 ? FONT_MEDIUM : FONT_SMALL;
        frame.text(f, r.x0 + 14, r.y0 + 5, kItems[i].label, WHITE);
        frame.text(FONT_SMALL, r.x0 + 14, r.y1 - 20, kItems[i].sub, GRAY);
        frame.text(FONT_SMALL, r.x1 - 16, r.y0 + 6, ">", CYAN);
    }
    footer("To leave a tool, hold the top-right corner of this screen for 1 second.", GRAY);
    frame.present();
    std::string st = "RUI|HOME|";
    for (int i = 0; i < ITEM_COUNT; i++) st += sf("%s%s~%s", i ? ";" : "", kItems[i].label, kItems[i].sub);
    publish(st);
}

// -- MOTOR TEMPS --
struct TempSection {
    const char *label;
    std::uint32_t color;
    std::vector<dashboard::MotorReading> motors;
};

std::uint32_t tempColor(const dashboard::MotorReading &m) {
    if (!m.connected) return GRAY;
    if (m.overTemp || m.temp >= dashboard::HOT_C) return RED;
    return m.temp >= dashboard::WARM_C ? ORANGE : CYAN;
}

void drawTemps() {
    std::vector<TempSection> sections;
    sections.push_back({"LEFT DRIVETRAIN", CYAN, dashboard::readMotors(left_motor_group, "L")});
    sections.push_back({"RIGHT DRIVETRAIN", ORANGE, dashboard::readMotors(right_motor_group, "R")});
    std::vector<dashboard::MotorReading> mech = dashboard::readMotors(lift, "Lift");
    for (const dashboard::MotorReading &r : dashboard::readMotors(intake2, "Intake")) mech.push_back(r);
    mech.push_back(dashboard::readMotor(claw_arm, "CARM", "Arm"));
    sections.push_back({"LIFT, INTAKE, CLAW ARM", CYAN, mech});

    std::string st = "RUI|TEMPS|";
    for (size_t s = 0; s < sections.size(); s++) {
        st += sf("%s%s:", s ? "#" : "", sections[s].label);
        for (size_t i = 0; i < sections[s].motors.size(); i++) {
            const dashboard::MotorReading &m = sections[s].motors[i];
            st += sf("%s%s,%.1f,%d,%d", i ? ";" : "", m.name, m.connected ? m.temp : 0.0, m.connected ? 1 : 0, m.overTemp ? 1 : 0);
        }
    }
    publish(st);

    header("MOTOR TEMPS", nullptr, true);
    const int left = 15, right = 465, gap = 6, perRow = 5, tileH = 38, pitch = 53;
    int tileW = (right - left - (perRow - 1) * gap) / perRow;
    double hottest = 0;
    bool anyMissing = false;
    for (size_t s = 0; s < sections.size(); s++) {
        int labelY = 55 + (int)s * pitch;
        frame.text(FONT_SMALL, left, labelY, sections[s].label, sections[s].color);
        for (size_t i = 0; i < sections[s].motors.size() && i < (size_t)perRow; i++) {
            const dashboard::MotorReading &m = sections[s].motors[i];
            int x = left + (int)i * (tileW + gap), y = labelY + 15;
            std::uint32_t col = tempColor(m);
            frame.card(x, y, x + tileW, y + tileH, 5, CARD, AXIS);
            frame.fillCircle(x + tileW - 9, y + 9, 4, col);
            frame.text(FONT_SMALL, x + 7, y + 3, m.name, GRAY);
            char buf[16];
            if (m.connected) {
                snprintf(buf, sizeof(buf), "%.1f\xC2\xB0""C", m.temp);
                hottest = std::max(hottest, m.temp);
            } else {
                snprintf(buf, sizeof(buf), "no motor");
                anyMissing = true;
            }
            frame.text(FONT_SMALL, x + 7, y + 17, buf, col);
            double frac = m.connected ? std::clamp((m.temp - 20.0) / 50.0, 0.0, 1.0) : 0.0; // 20C..70C
            int barX = x + 7, barY = y + tileH - 7, barW = tileW - 14;
            frame.fillRoundRect(barX, barY, barX + barW, barY + 4, 2, AXIS);
            if ((int)(barW * frac) >= 4) frame.fillRoundRect(barX, barY, barX + (int)(barW * frac), barY + 4, 2, col);
        }
    }
    char msg[48];
    snprintf(msg, sizeof(msg), "hottest %.1f\xC2\xB0""C%s", hottest, anyMissing ? "   (a motor is not answering)" : "");
    footer(msg, hottest >= dashboard::HOT_C ? RED : (hottest >= dashboard::WARM_C ? ORANGE : CYAN));
    frame.present();
}

// -- TEST MOTORS --
struct TestEntry {
    char label[16];
    int port;
    bool reversed;
    int state; // 0 waiting, 1 spinning, 2 answered, 3 no answer
    int rpm;       // speed measured during the spin
    int elapsedMs; // how long it has been spinning
};
std::vector<TestEntry> tests;
bool testFinished = false, testAborted = false;
const int kTestSpinMs = 2000;  // each motor turns this long
const int kTestSpinUpMs = 500; // speed samples before this are ignored
const int kTestPauseMs = 300;  // lets it stop before the next one starts

void addTests(const char *prefix, const pros::AbstractMotor &motors) {
    std::vector<std::int8_t> ports = motors.get_port_all();
    for (size_t i = 0; i < ports.size(); i++) {
        TestEntry e{};
        snprintf(e.label, sizeof(e.label), "%s%d", prefix, (int)i + 1);
        e.port = std::abs((int)ports[i]);
        e.reversed = ports[i] < 0;
        tests.push_back(e);
    }
}

void drawConfirm() {
    header("TEST MOTORS", "drivetrain only", true);
    frame.text(FONT_MEDIUM, 20, 70, "Put the robot on blocks first.", WHITE);
    frame.text(FONT_SMALL, 20, 98, "The drive motors turn one at a time, 2 seconds each.", GRAY);
    frame.text(FONT_SMALL, 20, 116, "Every wheel should roll forward. If one rolls backward, flip", GRAY);
    frame.text(FONT_SMALL, 20, 134, "the sign of its port in main.cpp. BACK stops the test.", GRAY);
    frame.card(150, 160, 330, 205, 8, CARD, CYAN);
    frame.text(FONT_MEDIUM, 240, 172, "START", GREEN, Canvas::CENTER);
    frame.present();
    publish("RUI|TEST_CONFIRM|");
}
const Rect kStart = {150, 160, 330, 205};

void drawTest() {
    header("TEST MOTORS", testFinished ? "done" : "running...", true);
    const int left = 15, gap = 8, top = 71, tileH = 25, pitch = 29;
    const int tileW = (465 - left - gap) / 2;
    const int perSide = (int)left_motor_group.size(); // column 1: left drive, column 2: right drive
    frame.text(FONT_SMALL, left, 54, "LEFT DRIVE", CYAN);
    frame.text(FONT_SMALL, left + tileW + gap, 54, "RIGHT DRIVE", ORANGE);
    int answered = 0, failed = 0;
    for (size_t i = 0; i < tests.size(); i++) {
        const TestEntry &e = tests[i];
        int col = (int)i < perSide ? 0 : 1, row = (int)i - col * perSide;
        int x = left + col * (tileW + gap), y = top + row * pitch;
        std::uint32_t edge = e.state == 1 ? CYAN : (e.state == 3 ? RED : AXIS);
        frame.card(x, y, x + tileW, y + tileH, 5, CARD, edge);
        char name[24];
        snprintf(name, sizeof(name), "%s  P%d%s", e.label, e.port, e.reversed ? " rev" : "");
        frame.text(FONT_SMALL, x + 8, y + 5, name, WHITE);
        char status[24] = "";
        if (e.state == 1) snprintf(status, sizeof(status), "%.1f s  %d rpm", (kTestSpinMs - e.elapsedMs) / 1000.0, e.rpm);
        else if (e.state == 2) snprintf(status, sizeof(status), "OK  %d rpm", e.rpm);
        else if (e.state == 3) snprintf(status, sizeof(status), "NO ANSWER");
        std::uint32_t sc = e.state == 3 ? RED : (e.state == 2 ? GREEN : CYAN);
        frame.text(FONT_SMALL, x + tileW - 8, y + 5, status, sc, Canvas::RIGHT);
        if (e.state == 1) { // progress through the 2 seconds
            int w = (tileW - 16) * std::clamp(e.elapsedMs, 0, kTestSpinMs) / kTestSpinMs;
            frame.fillRect(x + 8, y + tileH - 4, x + 8 + w, y + tileH - 2, CYAN);
        }
        answered += e.state == 2;
        failed += e.state == 3;
    }
    char msg[64];
    if (testAborted) snprintf(msg, sizeof(msg), "stopped. %d answered, %d did not.", answered, failed);
    else if (testFinished) snprintf(msg, sizeof(msg), "%d of %d motors answered.%s", answered, (int)tests.size(), failed ? "  Check the red ones." : "");
    else snprintf(msg, sizeof(msg), "testing %d of %d...  wheel should roll forward", answered + failed + 1, (int)tests.size());
    footer(msg, failed ? RED : (testFinished ? GREEN : CYAN));
    frame.present();
    std::string st = sf("RUI|TEST_RUN|%d|%d|", testFinished ? 1 : 0, testAborted ? 1 : 0);
    for (size_t i = 0; i < tests.size(); i++) // label,port,rev,state,rpm,elapsedMs
        st += sf("%s%s,%d,%d,%d,%d,%d", i ? ";" : "", tests[i].label, tests[i].port, tests[i].reversed ? 1 : 0,
                 tests[i].state, tests[i].rpm, tests[i].elapsedMs);
    publish(st);
}

// Spins each drive motor on its own for kTestSpinMs and sees whether it answered. BACK stops it at once.
void runMotorTest() {
    tests.clear();
    addTests("L", left_motor_group);
    addTests("R", right_motor_group);
    testFinished = testAborted = false;
    bool wasTouched = true;
    auto stopRequested = [&] {
        pros::screen_touch_status_s_t t = pros::screen::touch_status();
        bool down = t.touch_status != pros::E_TOUCH_RELEASED;
        int rx, ry;
        bool remoteBack = takeLinkTouch(rx, ry) && inRect(kBack, rx, ry); // BACK clicked on the laptop
        bool tapped = down && !wasTouched && inRect(kBack, t.x, t.y);
        wasTouched = down;
        return tapped || remoteBack;
    };
    for (TestEntry &e : tests) {
        e.state = 1;
        // The configured sign is kept, so positive voltage should roll every wheel forward.
        pros::Motor m(e.reversed ? -e.port : e.port);
        m.move_voltage(6000);
        double sum = 0;
        int samples = 0;
        const std::uint32_t start = pros::millis();
        std::uint32_t lastDrawMs = 0;
        while (!testAborted && (int)(pros::millis() - start) < kTestSpinMs) {
            e.elapsedMs = (int)(pros::millis() - start);
            const double v = m.get_actual_velocity();
            const bool valid = std::isfinite(v) && std::fabs(v) < 1000.0; // an unplugged motor reads a huge number
            e.rpm = valid ? (int)std::lround(std::fabs(v)) : 0;
            if (valid && e.elapsedMs >= kTestSpinUpMs) {
                sum += std::fabs(v);
                samples++;
            }
            if (!lastDrawMs || pros::millis() - lastDrawMs >= 100) {
                lastDrawMs = pros::millis();
                drawTest();
            }
            testAborted = stopRequested();
            pros::delay(20);
        }
        m.move_voltage(0);
        if (testAborted) {
            e.state = 0;
            e.rpm = 0;
            break;
        }
        const double average = samples ? sum / samples : 0.0;
        e.rpm = (int)std::lround(average);
        e.state = average > 5.0 ? 2 : 3;
        drawTest();
        for (int waited = 0; waited < kTestPauseMs && !testAborted; waited += 20) {
            testAborted = stopRequested();
            pros::delay(20);
        }
        if (testAborted) break;
    }
    testFinished = true;
    drawTest();
}

// Runs one of the full-screen tools. They return when the driver holds the screen's top-right
// corner; wait for the finger to lift so that touch doesn't also press something here.
void runTool(const char *name, void (*tool)()) {
    publish(sf("RUI|TOOL|%s|", name)); // the laptop shows which tool the brain is in (it cannot mirror them)
    tool();
    while (pros::screen::touch_status().touch_status != pros::E_TOUCH_RELEASED) pros::delay(20);
}

} // namespace

void driverMenuControl() {
    driverMenuActive = true;
    startLink();
    pros::delay(100); // let the dashboard task notice and stop drawing
    controller.rumble("-");
    Screen screen = Screen::HOME;
    bool redraw = true, wasTouched = true; // a touch already down when the menu opens is not a tap
    uint32_t lastDrawMs = 0;

    while (true) {
        pros::screen_touch_status_s_t t = pros::screen::touch_status();
        bool down = t.touch_status != pros::E_TOUCH_RELEASED;
        bool tap = down && !wasTouched;
        wasTouched = down;
        uint32_t nowMs = pros::millis();
        int tx = t.x, ty = t.y;
        int rx, ry;
        if (takeLinkTouch(rx, ry)) { // a click in the laptop window counts exactly like a tap on the screen
            tap = true;
            tx = rx;
            ty = ry;
        }

        if (tap) {
            switch (screen) {
            case Screen::HOME:
                for (int i = 0; i < ITEM_COUNT; i++) {
                    if (!inRect(itemRect(i), tx, ty)) continue;
                    redraw = true;
                    switch (i) {
                    case ITEM_DRIVE:
                        publish("RUI|DRIVE|");
                        driverMenuActive = false;
                        return;
                    case ITEM_PID: runTool("PID", pidTunerControl); break;
                    case ITEM_PLANNER: runTool("PLANNER", laptopPlanScreen); break; // the map planner; the old editor is a button on it
                    case ITEM_LIFT: runTool("LIFT", liftTunerControl); break;
                    case ITEM_INTAKE: runTool("INTAKE", intakeTunerControl); break;
                    case ITEM_DISTANCE: runTool("DISTANCE", distanceDiagControl); break;
                    case ITEM_ODOM: runTool("ODOM", odomSetupControl); break;
                    case ITEM_TEST: screen = Screen::TEST_CONFIRM; break;
                    case ITEM_TEMPS: screen = Screen::TEMPS; break;
                    }
                    wasTouched = true;
                    break;
                }
                break;
            case Screen::TEMPS:
                if (inRect(kBack, tx, ty)) {
                    screen = Screen::HOME;
                    redraw = true;
                }
                break;
            case Screen::TEST_CONFIRM:
                if (inRect(kBack, tx, ty)) {
                    screen = Screen::HOME;
                    redraw = true;
                } else if (inRect(kStart, tx, ty)) {
                    screen = Screen::TEST_RUN;
                    runMotorTest();
                    wasTouched = true;
                }
                break;
            case Screen::TEST_RUN:
                if (inRect(kBack, tx, ty)) {
                    screen = Screen::HOME;
                    redraw = true;
                }
                break;
            }
        }

        // Temperatures change on their own; everything else only redraws after a tap.
        if (screen == Screen::TEMPS && nowMs - lastDrawMs >= 250) redraw = true;
        if (redraw) {
            redraw = false;
            lastDrawMs = nowMs;
            switch (screen) {
            case Screen::HOME: drawHome(); break;
            case Screen::TEMPS: drawTemps(); break;
            case Screen::TEST_CONFIRM: drawConfirm(); break;
            case Screen::TEST_RUN: break; // drawn by runMotorTest()
            }
        }
        pros::delay(20);
    }
}
