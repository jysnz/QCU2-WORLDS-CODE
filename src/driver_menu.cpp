// -----------------------------------------------------------------------------
// DRIVER MENU -- see include/driver_menu.hpp for what it is and how to open it.
// Touchscreen home screen shown at the start of opcontrol(); the look (cyan header,
// cards with a cyan accent bar, BACK at the top-right) follows the `debi` branch.
// Everything is drawn on an off-screen Canvas and presented in one copy, like the
// dashboard and the lift tuner, so nothing flickers.
// -----------------------------------------------------------------------------

#include "driver_menu.hpp"
#include "canvas.hpp"
#include "lift_tuning.hpp"
#include "motors.hpp"
#include "pid_tuner.hpp"
#include "point_planner.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "screen_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

bool driverMenuActive = false;

// Hold the top-right corner of the screen this long to leave a tool.
static const uint32_t kLeaveHoldMs = 800;
static const int kLeaveX0 = 396, kLeaveY1 = 46;

bool driverMenuLeaveRequested() {
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
enum { ITEM_DRIVE, ITEM_PID, ITEM_PLANNER, ITEM_TEST, ITEM_LIFT, ITEM_TEMPS, ITEM_COUNT };
const Item kItems[ITEM_COUNT] = {
    {"DRIVE", "joystick driving"},
    {"PID TUNING", "angular + lateral"},
    {"PATH PLANNER", "drive to a point"},
    {"TEST MOTORS", "does each one spin"},
    {"LIFT TUNER", "levels, combos, angles"},
    {"MOTOR TEMPS", "every motor, live"},
};
const int kGridX0 = 15, kGridX1 = 465, kGridY0 = 62, kRowH = 70, kGap = 8, kCols = 3;

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
        frame.text(f, r.x0 + 14, r.y0 + 14, kItems[i].label, WHITE);
        frame.text(FONT_SMALL, r.x0 + 14, r.y1 - 22, kItems[i].sub, GRAY);
        frame.text(FONT_SMALL, r.x1 - 16, r.y0 + 14, ">", CYAN);
    }
    footer("To leave a tool, hold the top-right corner of this screen for 1 second.", GRAY);
    frame.present();
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
};
std::vector<TestEntry> tests;
bool testFinished = false, testAborted = false;

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
    header("TEST MOTORS", "spins every motor briefly", true);
    frame.text(FONT_MEDIUM, 20, 70, "Put the robot on blocks first.", WHITE);
    frame.text(FONT_SMALL, 20, 98, "Every motor turns for about a third of a second, one at a time.", GRAY);
    frame.text(FONT_SMALL, 20, 116, "You will see which ones answer. The direction is set by the sign", GRAY);
    frame.text(FONT_SMALL, 20, 134, "of the port in main.cpp (minus = reversed).", GRAY);
    frame.card(150, 160, 330, 205, 8, CARD, CYAN);
    frame.text(FONT_MEDIUM, 240, 172, "START", GREEN, Canvas::CENTER);
    frame.present();
}
const Rect kStart = {150, 160, 330, 205};

void drawTest() {
    header("TEST MOTORS", testFinished ? "done" : "running...", true);
    const int cols = 3, rowsPerCol = 5, left = 15, gap = 8, tileH = 28, top = 58;
    int tileW = (465 - left - (cols - 1) * gap) / cols;
    // Column 1: left drive. Column 2: right drive. Column 3: everything else.
    int answered = 0, failed = 0;
    for (size_t i = 0; i < tests.size(); i++) {
        const TestEntry &e = tests[i];
        int col = (int)i / rowsPerCol, row = (int)i % rowsPerCol;
        int x = left + col * (tileW + gap), y = top + row * (tileH + 5);
        std::uint32_t edge = e.state == 1 ? CYAN : (e.state == 3 ? RED : AXIS);
        frame.card(x, y, x + tileW, y + tileH, 5, CARD, edge);
        char name[24];
        snprintf(name, sizeof(name), "%s  P%d%s", e.label, e.port, e.reversed ? " rev" : "");
        frame.text(FONT_SMALL, x + 8, y + 7, name, WHITE);
        const char *status = e.state == 1 ? "..." : e.state == 2 ? "OK" : e.state == 3 ? "NO ANSWER" : "";
        std::uint32_t sc = e.state == 3 ? RED : (e.state == 2 ? GREEN : CYAN);
        frame.text(FONT_SMALL, x + tileW - 8, y + 7, status, sc, Canvas::RIGHT);
        answered += e.state == 2;
        failed += e.state == 3;
    }
    char msg[64];
    if (testAborted) snprintf(msg, sizeof(msg), "stopped. %d answered, %d did not.", answered, failed);
    else if (testFinished) snprintf(msg, sizeof(msg), "%d of %d motors answered.%s", answered, (int)tests.size(), failed ? "  Check the red ones." : "");
    else snprintf(msg, sizeof(msg), "testing %d of %d...", answered + failed + 1, (int)tests.size());
    footer(msg, failed ? RED : (testFinished ? GREEN : CYAN));
    frame.present();
}

// Spins each motor on a fresh handle for a moment and sees whether it answered.
void runMotorTest() {
    tests.clear();
    addTests("L", left_motor_group);
    addTests("R", right_motor_group);
    addTests("Lift", lift);
    addTests("In", intake2);
    addTests("Arm", claw_arm);
    testFinished = testAborted = false;
    bool wasTouched = true;
    for (TestEntry &e : tests) {
        pros::screen_touch_status_s_t t = pros::screen::touch_status();
        bool down = t.touch_status != pros::E_TOUCH_RELEASED;
        if (down && !wasTouched && inRect(kBack, t.x, t.y)) {
            testAborted = true;
            break;
        }
        wasTouched = down;
        e.state = 1;
        drawTest();
        pros::Motor m(e.port);
        m.move_voltage(6000);
        pros::delay(300);
        double v = m.get_actual_velocity();
        m.move_voltage(0);
        pros::delay(200);
        e.state = std::isfinite(v) && std::fabs(v) > 1.0 ? 2 : 3;
    }
    for (TestEntry &e : tests)
        if (e.state == 1) e.state = 0;
    testFinished = true;
    drawTest();
}

// Runs one of the full-screen tools. They return when the driver holds the screen's top-right
// corner; wait for the finger to lift so that touch doesn't also press something here.
void runTool(void (*tool)()) {
    tool();
    while (pros::screen::touch_status().touch_status != pros::E_TOUCH_RELEASED) pros::delay(20);
}

} // namespace

void driverMenuControl() {
    driverMenuActive = true;
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

        if (tap) {
            switch (screen) {
            case Screen::HOME:
                for (int i = 0; i < ITEM_COUNT; i++) {
                    if (!inRect(itemRect(i), t.x, t.y)) continue;
                    redraw = true;
                    switch (i) {
                    case ITEM_DRIVE:
                        driverMenuActive = false;
                        return;
                    case ITEM_PID: runTool(pidTunerControl); break;
                    case ITEM_PLANNER: runTool(pointPlannerControl); break;
                    case ITEM_LIFT: runTool(liftTunerControl); break;
                    case ITEM_TEST: screen = Screen::TEST_CONFIRM; break;
                    case ITEM_TEMPS: screen = Screen::TEMPS; break;
                    }
                    wasTouched = true;
                    break;
                }
                break;
            case Screen::TEMPS:
                if (inRect(kBack, t.x, t.y)) {
                    screen = Screen::HOME;
                    redraw = true;
                }
                break;
            case Screen::TEST_CONFIRM:
                if (inRect(kBack, t.x, t.y)) {
                    screen = Screen::HOME;
                    redraw = true;
                } else if (inRect(kStart, t.x, t.y)) {
                    screen = Screen::TEST_RUN;
                    runMotorTest();
                    wasTouched = true;
                }
                break;
            case Screen::TEST_RUN:
                if (inRect(kBack, t.x, t.y)) {
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
