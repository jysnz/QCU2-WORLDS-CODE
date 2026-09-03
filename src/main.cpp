#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "pid_tuner.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({-16, 17, -18, 19, -20}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({11, -2, 13, -14, 15}, pros::MotorGears::green);
pros::MotorGroup arm({1, 10}, pros::MotorGears::green);
pros::Motor intake1(8, pros::MotorGears::green);
pros::Motor intake2(9, pros::MotorGears::green);

pros::adi::Pneumatics clamp('A', false);

pros::Imu imu(9);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ─── LemLib Setup ───
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 15, lemlib::Omniwheel::NEW_325, 458, 2);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(10, 0, 28, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(1.5, 0, 10, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

// ─── UI & Auton Selector State ───
enum UITab { TAB_TEMPS, TAB_AUTON };
UITab currentTab = TAB_TEMPS;

int currentAutonIndex = 0;
const std::vector<std::string> autonNames = {
    "2v2 Left Red",
    "2v2 Left Blue",
    "2v2 Right Red",
    "2v2 Right Blue",
    "Skills Challenge",
    "SkillsV1",
    "SoloAWP_Red_Right"
};

void saveAutonSelection() {
    FILE* usd_file = fopen("/usd/auton_selection.txt", "w");
    if (usd_file) {
        fprintf(usd_file, "%d", currentAutonIndex);
        fclose(usd_file);
    }
}

void loadAutonSelection() {
    FILE* usd_file = fopen("/usd/auton_selection.txt", "r");
    if (usd_file) {
        fscanf(usd_file, "%d", &currentAutonIndex);
        fclose(usd_file);
        if (currentAutonIndex >= (int)autonNames.size()) currentAutonIndex = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// initialize
// ─────────────────────────────────────────────────────────────────────────────
void initialize() {
    loadAutonSelection();
    
    // Ensure legacy LLEMU is NOT active to prevent drawing conflicts
    chassis.calibrate();

    // ── Cyber-HUD On-screen UI ──
    static pros::Task screen_task([]() {
        const int BG_DARK = 0x0A0A0F;
        const int CARD_BG = 0x161B22;
        const int ACCENT_CYAN = 0x00F0FF;
        const int ACCENT_ORANGE = 0xFF8C00;
        const int ACCENT_RED = 0xFF3131;
        const int ACCENT_DARK_GRAY = 0x22222B;
        const int ACCENT_GREEN = 0x2ECC71;
        const int TEXT_DIM = 0x6E7681;

        // Fakes a rounded rectangle (a cross of two rects + 4 corner circles) so cards,
        // tabs and buttons don't look like flat blocky slabs.
        auto fillRoundedRect = [](int x0, int y0, int x1, int y1, int r, uint32_t color) {
            pros::screen::set_pen(color);
            pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
            pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
            pros::screen::fill_circle(x0 + r, y0 + r, r);
            pros::screen::fill_circle(x1 - r, y0 + r, r);
            pros::screen::fill_circle(x0 + r, y1 - r, r);
            pros::screen::fill_circle(x1 - r, y1 - r, r);
        };

        int tempsScroll = 0;
        int autonScroll = 0;

        // ── Dirty-state tracking to eliminate unnecessary full-screen redraws (flicker fix) ──
        // The old loop cleared and redrew the ENTIRE screen every 35ms even when nothing
        // on screen had actually changed, which is what caused the visible flicker. Now we
        // only repaint when something the user can see has actually changed.
        bool forceRedraw = true;
        UITab lastTab = currentTab;
        int lastTempsScroll = INT32_MIN;
        int lastAutonScroll = INT32_MIN;
        int lastAutonIndex = -1;
        int lastImu = INT32_MIN;
        int lastBattery = INT32_MIN;
        std::vector<int> lastTemps; // rounded to nearest 0.5C so tiny sensor jitter doesn't trigger redraws

        while (true) {
            // The PID tuner owns the brain screen while active
            if (pidTunerActive) {
                pros::delay(100);
                forceRedraw = true; // repaint fully once we get control back
                continue;
            }

            pros::screen_touch_status_s_t status = pros::screen::touch_status();

            // ── Input Detection ──
            static bool wasTouched = false;
            if (status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
                // 1. Header Tab Switch
                if (status.y < 50) {
                    if (status.x < 240) currentTab = TAB_TEMPS;
                    else currentTab = TAB_AUTON;
                }

                // 2. Scroll Buttons (Far Right Column: 430 to 480)
                if (status.x > 430) {
                    if (status.y > 60 && status.y < 130) { // UP Arrow
                        if (currentTab == TAB_AUTON) autonScroll += 55;
                        else tempsScroll += 60;
                    }
                    else if (status.y > 140 && status.y < 210) { // DOWN Arrow
                        if (currentTab == TAB_AUTON) autonScroll -= 55;
                        else tempsScroll -= 60;
                    }
                }

                // 3. Selection in Auton Tab
                if (currentTab == TAB_AUTON && status.x < 420 && status.y > 50 && status.y < 215) {
                    for (int i = 0; i < (int)autonNames.size(); i++) {
                        int itemY = 60 + (i * 55) + autonScroll;
                        if (status.y >= itemY && status.y <= itemY + 50) {
                            currentAutonIndex = i;
                            saveAutonSelection();
                            break;
                        }
                    }
                }
                wasTouched = true;
            } else if (status.touch_status == pros::E_TOUCH_RELEASED) {
                wasTouched = false;
            }

            // Clamp Scrolling
            if (autonScroll > 0) autonScroll = 0;
            int maxAutonScroll = -((int)autonNames.size() * 55 - 140);
            if (autonScroll < maxAutonScroll && maxAutonScroll < 0) autonScroll = maxAutonScroll;

            // Each section = a label line + a row of tiles.
            // Drivetrain sections show 5 tiles/row; the arm and intake sections show 2 tiles/row.
            const int tempRowHeight = 66;
            const int numTempRows = 4; // LEFT drivetrain, RIGHT drivetrain, ARM, INTAKE
            int tempsContentHeight = numTempRows * tempRowHeight;
            int minTempsScroll = (tempsContentHeight > 160) ? -(tempsContentHeight - 160) : 0;
            if (tempsScroll > 0) tempsScroll = 0;
            if (tempsScroll < minTempsScroll) tempsScroll = minTempsScroll;

            // ── Gather live data once per frame ──
            std::vector<double> leftTemps = left_motor_group.get_temperature_all();
            std::vector<double> rightTemps = right_motor_group.get_temperature_all();
            std::vector<double> armTemps = arm.get_temperature_all();
            std::vector<double> intakeTemps = {intake1.get_temperature(), intake2.get_temperature()};
            std::vector<std::int8_t> leftPorts = left_motor_group.get_port_all();
            std::vector<std::int8_t> rightPorts = right_motor_group.get_port_all();
            std::vector<std::int8_t> armPorts = arm.get_port_all();
            std::vector<std::int8_t> intakePorts = {intake1.get_port(), intake2.get_port()};

            int imuNow = (int)(imu.get_heading() * 10);
            int batteryNow = (int)pros::battery::get_capacity();

            // Build a rounded snapshot of every motor temp for change detection
            std::vector<int> tempsNow;
            for (double t : leftTemps) tempsNow.push_back((int)(t * 2));  // 0.5C resolution
            for (double t : rightTemps) tempsNow.push_back((int)(t * 2));
            for (double t : armTemps) tempsNow.push_back((int)(t * 2));
            for (double t : intakeTemps) tempsNow.push_back((int)(t * 2));

            // Touch-driven changes (tab/scroll/selection) redraw instantly. Live telemetry
            // (motor temps, IMU heading, battery) is throttled to a few times a second instead —
            // while a motor is spinning its temperature (and the IMU heading, while driving)
            // changes almost every loop, which was forcing a full-screen clear+redraw ~28x/sec
            // and is what caused the visible flicker as soon as a motor was connected and running.
            bool interactiveChanged = forceRedraw ||
                                       currentTab != lastTab ||
                                       tempsScroll != lastTempsScroll ||
                                       autonScroll != lastAutonScroll ||
                                       currentAutonIndex != lastAutonIndex;
            bool telemetryChanged = imuNow != lastImu ||
                                     batteryNow != lastBattery ||
                                     tempsNow != lastTemps;

            static uint32_t lastTelemetryRedraw = 0;
            const uint32_t TELEMETRY_REDRAW_INTERVAL_MS = 300;
            uint32_t nowMs = pros::millis();
            bool telemetryDue = telemetryChanged && (nowMs - lastTelemetryRedraw >= TELEMETRY_REDRAW_INTERVAL_MS);

            bool dataChanged = interactiveChanged || telemetryDue;

            if (!dataChanged) {
                pros::delay(35);
                continue;
            }

            if (telemetryChanged) lastTelemetryRedraw = nowMs;

            // ── RENDERING (only happens when something actually changed) ──
            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);

            // Subtle Grid
            pros::screen::set_pen(0x121217);
            for(int i=0; i<480; i+=40) pros::screen::draw_line(i, 0, i, 240);
            for(int i=0; i<240; i+=40) pros::screen::draw_line(0, i, 480, i);

            // ── Content Area Rendering (With Clipping Check) ──
            if (currentTab == TAB_TEMPS) {
                const int rowLeft = 15, rowRight = 428;
                const int labelH = 14;  // space reserved for the section name above its tiles
                const int tileH = 46;

                auto drawMotorTile = [&](int x, int y, int tileW, const char *label, double temp) {
                    // Clipping: Only draw if the tile is between the header (50) and footer (215)
                    if (y + tileH < 55 || y > 215) return;

                    uint32_t col_ = (temp < 45) ? ACCENT_CYAN : (temp < 55 ? ACCENT_ORANGE : ACCENT_RED);

                    fillRoundedRect(x, y, x + tileW, y + tileH, 5, CARD_BG);

                    // Status dot instead of a harsh left bar — sits top-right, glanceable at a distance
                    pros::screen::set_pen(col_);
                    pros::screen::fill_circle(x + tileW - 9, y + 9, 4);

                    // print() paints an opaque box behind text using the eraser color (black by
                    // default) — match it to the tile background so text doesn't get a black halo
                    pros::screen::set_eraser(CARD_BG);
                    pros::screen::set_pen(TEXT_DIM);
                    pros::screen::print(pros::E_TEXT_SMALL, x + 8, y + 5, label);
                    pros::screen::set_pen(col_);
                    pros::screen::print(pros::E_TEXT_SMALL, x + 8, y + 20, "%.1fC", temp);

                    // Mini heat gauge along the bottom (0-70C range)
                    int barX = x + 8, barY = y + tileH - 9, barW = tileW - 16, barH = 4;
                    double frac = (temp - 20.0) / 50.0;
                    if (frac < 0) frac = 0;
                    if (frac > 1) frac = 1;
                    fillRoundedRect(barX, barY, barX + barW, barY + barH, 2, ACCENT_DARK_GRAY);
                    int fillW = (int)(barW * frac);
                    if (fillW >= 4) fillRoundedRect(barX, barY, barX + fillW, barY + barH, 2, col_);
                };

                // Draws a section label followed by a row of evenly-sized tiles, one per motor.
                auto drawSection = [&](int sectionY, const char *sectionLabel, uint32_t labelColor,
                                        std::vector<double> &temps, std::vector<std::int8_t> &ports,
                                        const char *namePrefix, int tilesPerRow) {
                    int labelY = sectionY;
                    int tileY = sectionY + labelH;

                    // Skip the whole section if it's fully outside the visible content area
                    if (tileY + tileH < 55 || labelY > 215) return;

                    if (labelY >= 50 && labelY + labelH <= 215) {
                        pros::screen::set_eraser(BG_DARK);
                        pros::screen::set_pen(labelColor);
                        pros::screen::print(pros::E_TEXT_SMALL, rowLeft, labelY, sectionLabel);
                    }

                    int gap = 6;
                    int tileW = (rowRight - rowLeft - (tilesPerRow - 1) * gap) / tilesPerRow;
                    for (int i = 0; i < (int)temps.size() && i < tilesPerRow; i++) {
                        int x = rowLeft + i * (tileW + gap);
                        char label[20];
                        snprintf(label, sizeof(label), "%s %d P%d", namePrefix, i + 1, (int)ports[i]);
                        drawMotorTile(x, tileY, tileW, label, temps[i]);
                    }
                };

                int yBase = 55 + tempsScroll;

                // Section 1: LEFT drivetrain motors (5 tiles/row)
                drawSection(yBase, "LEFT DRIVETRAIN", ACCENT_CYAN, leftTemps, leftPorts, "L", 5);

                // Section 2: RIGHT drivetrain motors (5 tiles/row)
                drawSection(yBase + tempRowHeight, "RIGHT DRIVETRAIN", ACCENT_ORANGE, rightTemps, rightPorts, "R", 5);

                // Section 3: ARM motors (2 tiles/row, named)
                drawSection(yBase + tempRowHeight * 2, "ARM", 0xFFFFFF, armTemps, armPorts, "Arm Motor", 2);

                // Section 4: INTAKE motors (2 tiles/row, named)
                drawSection(yBase + tempRowHeight * 3, "INTAKE", ACCENT_GREEN, intakeTemps, intakePorts, "Intake Motor", 2);

            } else {
                for (int i = 0; i < (int)autonNames.size(); i++) {
                    int itemY = 60 + (i * 55) + autonScroll;

                    // Clipping Check
                    if (itemY + 50 < 55 || itemY > 215) continue;

                    bool isSelected = (i == currentAutonIndex);
                    fillRoundedRect(20, itemY, 420, itemY + 50, 6, isSelected ? 0x1c2838 : CARD_BG);

                    if (isSelected) {
                        pros::screen::set_pen(ACCENT_ORANGE);
                        pros::screen::draw_rect(20, itemY, 420, itemY + 50);
                    }

                    // Selection indicator: filled dot when active, hollow ring otherwise
                    int dotX = 40, dotY = itemY + 25;
                    if (isSelected) {
                        pros::screen::set_pen(ACCENT_ORANGE);
                        pros::screen::fill_circle(dotX, dotY, 6);
                    } else {
                        pros::screen::set_pen(0x3a3a45);
                        pros::screen::draw_circle(dotX, dotY, 6);
                    }

                    pros::screen::set_eraser(isSelected ? 0x1c2838 : CARD_BG);
                    pros::screen::set_pen(isSelected ? 0xFFFFFF : 0x8b949e);
                    pros::screen::print(pros::E_TEXT_MEDIUM, 60, itemY + 15, "%s", autonNames[i].c_str());
                }
            }

            // ── Static Overlays (Drawn last to prevent overlap) ──

            // Header Tabs — rounded pills; inactive tabs stay flush with the background instead
            // of a heavy gray slab, so the active tab reads clearly at a glance
            fillRoundedRect(5, 5, 238, 50, 8, currentTab == TAB_TEMPS ? ACCENT_CYAN : CARD_BG);
            fillRoundedRect(242, 5, 475, 50, 8, currentTab == TAB_AUTON ? ACCENT_ORANGE : CARD_BG);

            pros::screen::set_eraser(currentTab == TAB_TEMPS ? ACCENT_CYAN : CARD_BG);
            pros::screen::set_pen(currentTab == TAB_TEMPS ? 0x001417 : TEXT_DIM);
            pros::screen::print(pros::E_TEXT_SMALL, 60, 20, "SYSTEM_THERMALS");
            pros::screen::set_eraser(currentTab == TAB_AUTON ? ACCENT_ORANGE : CARD_BG);
            pros::screen::set_pen(currentTab == TAB_AUTON ? 0x1a0d00 : TEXT_DIM);
            pros::screen::print(pros::E_TEXT_SMALL, 305, 20, "MISSION_CONFIG");

            // Scroll Buttons Sidebar
            fillRoundedRect(435, 60, 475, 130, 8, CARD_BG); // Up Button
            fillRoundedRect(435, 140, 475, 210, 8, CARD_BG); // Down Button
            pros::screen::set_eraser(CARD_BG);
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::print(pros::E_TEXT_MEDIUM, 448, 85, "^");
            pros::screen::print(pros::E_TEXT_MEDIUM, 448, 165, "v");

            // Footer Diagnostics — battery dot color reflects charge level at a glance
            pros::screen::set_pen(0x18181F);
            pros::screen::fill_rect(0, 215, 480, 240);
            pros::screen::set_eraser(0x18181F);

            double batteryPct = pros::battery::get_capacity();
            uint32_t battCol = (batteryPct > 60) ? ACCENT_GREEN : (batteryPct > 30 ? ACCENT_ORANGE : ACCENT_RED);
            pros::screen::set_pen(ACCENT_GREEN);
            pros::screen::fill_circle(20, 227, 4);
            pros::screen::set_pen(TEXT_DIM);
            pros::screen::print(pros::E_TEXT_SMALL, 30, 222, "ONLINE  |  IMU %.1f", imu.get_heading());

            pros::screen::set_pen(battCol);
            pros::screen::fill_circle(230, 227, 4);
            pros::screen::set_pen(TEXT_DIM);
            pros::screen::print(pros::E_TEXT_SMALL, 240, 222, "BAT %.0f%%  |  %s", batteryPct, autonNames[currentAutonIndex].c_str());

            // Update dirty-tracking snapshot
            lastTab = currentTab;
            lastTempsScroll = tempsScroll;
            lastAutonScroll = autonScroll;
            lastAutonIndex = currentAutonIndex;
            lastImu = imuNow;
            lastBattery = batteryNow;
            lastTemps = tempsNow;
            forceRedraw = false;

            pros::delay(35);
        }
    });
}

void opcontrol() {
    // Hold DPAD-LEFT when driver control starts to enter the PID tuner
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        pidTunerControl();
    } else {
        jawheadControl();
    }
}
void autonomous() { runAutonomous(); }