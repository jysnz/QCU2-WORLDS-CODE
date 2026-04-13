#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({-11, -12, -13, -14}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({1, 2, 3, 4}, pros::MotorGears::green);
pros::MotorGroup intake({16, -18}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor matchloader(5, pros::MotorGears::green);
pros::Motor descore(15, pros::MotorGears::green);
pros::Motor arm(6, pros::MotorGears::red);
pros::Motor gate(17, pros::MotorGears::green);

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
UITab currentTab = TAB_AUTON;

int currentAutonIndex = 0;
int scrollOffset = 0; 
const std::vector<std::string> autonNames = {
    "2v2 Left",
    "2v2 Right Red",
    "2v2 Right Blue",
    "Skills Challenge",
    "Do Nothing"
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
    
    pros::Task catapult_control(catapultTask, nullptr, "Catapult Task");
    pros::Task intake_control(intakeTask, nullptr, "Intake Task");

    catapult_arm.tare_position();
    matchloader.tare_position();
    descore.tare_position();

    // REMOVED: pros::lcd::initialize() to prevent screen glitching conflicts
    chassis.calibrate();

    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // ── Cyber-HUD On-screen UI ──
    static pros::Task screen_task([]() {
        const int BG_DARK = 0x0A0A0F;
        const int CARD_BG = 0x161B22;
        const int ACCENT_CYAN = 0x00F0FF;
        const int ACCENT_ORANGE = 0xFF8C00;
        const int ACCENT_RED = 0xFF3131;
        const int ACCENT_DARK_GRAY = 0x22222B;

        int tempsScroll = 0;
        int autonScroll = 0;

        while (true) {
            pros::screen_touch_status_s_t status = pros::screen::touch_status();
            
            // ── Input Detection ──
            static bool wasTouched = false;
            if (status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
                // 1. Header Tab Switch
                if (status.y < 45) {
                    if (status.x < 240) currentTab = TAB_TEMPS;
                    else currentTab = TAB_AUTON;
                }
                
                // 2. Scroll Buttons (Far Right Column)
                if (status.x > 430) {
                    if (status.y > 60 && status.y < 130) { // UP Arrow
                        if (currentTab == TAB_AUTON) autonScroll += 40;
                        else tempsScroll += 40;
                    }
                    else if (status.y > 140 && status.y < 210) { // DOWN Arrow
                        if (currentTab == TAB_AUTON) autonScroll -= 40;
                        else tempsScroll -= 40;
                    }
                }

                // 3. Selection in Auton Tab
                if (currentTab == TAB_AUTON && status.x < 420 && status.y > 50) {
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

            if (tempsScroll > 0) tempsScroll = 0;
            if (tempsScroll < -120) tempsScroll = -120; // 4 rows of 65px cards

            // ── Clear & Grid ──
            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);
            
            pros::screen::set_pen(0x121217);
            for(int i=0; i<480; i+=30) pros::screen::draw_line(i, 0, i, 240);
            for(int i=0; i<240; i+=30) pros::screen::draw_line(0, i, 480, i);

            // ── Navigation Tabs ──
            pros::screen::set_pen(currentTab == TAB_TEMPS ? ACCENT_CYAN : ACCENT_DARK_GRAY);
            pros::screen::fill_rect(5, 5, 238, 45);
            pros::screen::set_pen(currentTab == TAB_AUTON ? ACCENT_ORANGE : ACCENT_DARK_GRAY);
            pros::screen::fill_rect(242, 5, 475, 45);
            
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_SMALL, 60, 18, "SYSTEM_THERMALS");
            pros::screen::print(pros::E_TEXT_SMALL, 305, 18, "MISSION_CONFIG");

            // ── Scroll Buttons Sidebar ──
            pros::screen::set_pen(ACCENT_DARK_GRAY);
            pros::screen::fill_rect(435, 60, 475, 130); // Up Button
            pros::screen::fill_rect(435, 140, 475, 210); // Down Button
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_MEDIUM, 445, 85, "^");
            pros::screen::print(pros::E_TEXT_MEDIUM, 445, 165, "v");

            if (currentTab == TAB_TEMPS) {
                // ── Diagnostic Card Layout (Original Request) ──
                auto drawTechCard = [&](int x, int y, const char *name, double temp) {
                    if (y + 55 < 50 || y > 215) return;
                    int w = 205, h = 55;
                    uint32_t col = (temp < 45) ? ACCENT_CYAN : (temp < 55 ? ACCENT_ORANGE : ACCENT_RED);
                    pros::screen::set_pen(CARD_BG);
                    pros::screen::fill_rect(x, y, x + w, y + h);
                    pros::screen::set_pen(col);
                    pros::screen::fill_rect(x, y, x + 4, y + h);
                    pros::screen::set_pen(0xAAAAAA);
                    pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 8, name);
                    pros::screen::set_pen(col);
                    pros::screen::print(pros::E_TEXT_MEDIUM, x + 12, y + 24, "%.1f C", temp);
                    
                    // Small progress bar in card
                    pros::screen::set_pen(0x33333F);
                    pros::screen::draw_rect(x + 105, y + 32, x + w - 10, y + 42);
                    int fill = (int)((std::min(temp, 70.0) / 70.0) * (w - 115));
                    pros::screen::set_pen(col);
                    pros::screen::fill_rect(x + 106, y + 33, x + 106 + fill, y + 41);
                };

                int yBase = 60 + tempsScroll;
                drawTechCard(15, yBase, "[ DRIVE_L ]", left_motor_group.get_temperature());
                drawTechCard(225, yBase, "[ DRIVE_R ]", right_motor_group.get_temperature());
                drawTechCard(15, yBase + 65, "[ INTAKE ]", intake.get_temperature());
                drawTechCard(225, yBase + 65, "[ CATAPULT ]", catapult_arm.get_temperature());
                drawTechCard(15, yBase + 130, "[ MATCHLOAD ]", matchloader.get_temperature());
                drawTechCard(225, yBase + 130, "[ DESCORE ]", descore.get_temperature());
                drawTechCard(15, yBase + 195, "[ ARM ]", arm.get_temperature());
                drawTechCard(225, yBase + 195, "[ GATE ]", gate.get_temperature());
            } else {
                // ── Refined Auton Selection List ──
                for (int i = 0; i < (int)autonNames.size(); i++) {
                    int itemY = 60 + (i * 55) + autonScroll;
                    if (itemY + 50 < 50 || itemY > 215) continue;
                    
                    bool isSelected = (i == currentAutonIndex);
                    pros::screen::set_pen(isSelected ? 0x1c2838 : CARD_BG);
                    pros::screen::fill_rect(20, itemY, 420, itemY + 50);
                    
                    pros::screen::set_pen(isSelected ? ACCENT_ORANGE : 0x2d2d38);
                    pros::screen::draw_rect(20, itemY, 420, itemY + 50);
                    
                    if(isSelected) {
                        pros::screen::set_pen(ACCENT_ORANGE);
                        pros::screen::fill_rect(20, itemY, 26, itemY + 50);
                    }

                    pros::screen::set_pen(isSelected ? 0xFFFFFF : 0x777777);
                    pros::screen::print(pros::E_TEXT_MEDIUM, 45, itemY + 15, isSelected ? "> %s <" : "  %s", autonNames[i].c_str());
                }
            }

            // Footer Diagnostics
            pros::screen::set_pen(0x18181F);
            pros::screen::fill_rect(0, 215, 480, 240);
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::print(pros::E_TEXT_SMALL, 15, 222, "OS_READY // IMU: %.1f // BAT: %.0f%% // %s", 
                                imu.get_heading(), pros::battery::get_capacity(), autonNames[currentAutonIndex].c_str());

            pros::delay(30);
        }
    });
}

void opcontrol() { catapultControl(); }
void autonomous() { runAutonomous(); }