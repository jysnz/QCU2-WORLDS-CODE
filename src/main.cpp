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

// ─── Auton Selector State ───
int currentAutonIndex = 0;
int scrollOffset = 0; // For vertical scrolling
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

    pros::lcd::initialize();
    chassis.calibrate();

    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // ── Cyber-HUD with Scrollable Selector ──
    static pros::Task screen_task([]() {
        const int BG_DARK = 0x0A0A0F;
        const int CARD_BG = 0x161B22;
        const int ACCENT_CYAN = 0x00F0FF;
        const int ACCENT_ORANGE = 0xFF8C00;
        const int ACCENT_RED = 0xFF3131;

        int lastTouchY = 0;
        bool isDragging = false;

        while (true) {
            pros::screen_touch_status_s_t status = pros::screen::touch_status();
            
            // ── Scroll & Select Logic ──
            if (status.touch_status == pros::E_TOUCH_PRESSED) {
                if (status.x > 240) { // Touching the right side (Selector area)
                    if (!isDragging) {
                        lastTouchY = status.y;
                        isDragging = true;
                    }
                    
                    // Handle scrolling
                    int deltaY = status.y - lastTouchY;
                    if (std::abs(deltaY) > 5) {
                        scrollOffset += deltaY;
                        lastTouchY = status.y;
                    }
                }
            } else if (status.touch_status == pros::E_TOUCH_RELEASED) {
                if (isDragging && std::abs(status.y - lastTouchY) < 10) {
                    // It was a tap, not a scroll
                    // Calculate which item was clicked based on scrollOffset
                    for (int i = 0; i < (int)autonNames.size(); i++) {
                        int itemY = 50 + (i * 45) + scrollOffset;
                        if (status.y >= itemY && status.y <= itemY + 40) {
                            currentAutonIndex = i;
                            saveAutonSelection();
                            break;
                        }
                    }
                }
                isDragging = false;
            }

            // Keep scroll in bounds
            int maxScroll = 0;
            int minScroll = -((int)autonNames.size() * 45 - 150);
            if (scrollOffset > maxScroll) scrollOffset = maxScroll;
            if (scrollOffset < minScroll) scrollOffset = minScroll;

            // ── Rendering ──
            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);
            
            // Header
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::draw_rect(5, 5, 475, 235);
            pros::screen::draw_line(5, 40, 475, 40);
            pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "CORE_OS // AUTON_ACTIVE: %s", autonNames[currentAutonIndex].c_str());

            // ── Left Side: Diagnostics ──
            auto drawTechCard = [&](int x, int y, const char *name, double temp) {
                int w = 210, h = 42;
                pros::screen::set_pen(CARD_BG);
                pros::screen::fill_rect(x, y, x + w, y + h);
                pros::screen::set_pen(temp > 50 ? ACCENT_RED : ACCENT_CYAN);
                pros::screen::fill_rect(x, y, x + 3, y + h);
                pros::screen::set_pen(0xAAAAAA);
                pros::screen::print(pros::E_TEXT_SMALL, x + 10, y + 6, name);
                pros::screen::set_pen(temp > 50 ? ACCENT_RED : ACCENT_CYAN);
                pros::screen::print(pros::E_TEXT_SMALL, x + 10, y + 22, "T: %.1fC", temp);
            };

            drawTechCard(15, 50, "DRIVETRAIN", (left_motor_group.get_temperature() + right_motor_group.get_temperature())/2.0);
            drawTechCard(15, 100, "CATAPULT", catapult_arm.get_temperature());
            drawTechCard(15, 150, "INTAKE", intake.get_temperature());
            drawTechCard(15, 200, "BATTERY", pros::battery::get_capacity());

            // ── Right Side: Scrollable Auton List ──
            // Clip the drawing area for the list
            for (int i = 0; i < (int)autonNames.size(); i++) {
                int itemY = 50 + (i * 45) + scrollOffset;
                if (itemY > 30 && itemY < 230) {
                    bool isSelected = (i == currentAutonIndex);
                    pros::screen::set_pen(isSelected ? 0x1c2533 : CARD_BG);
                    pros::screen::fill_rect(240, itemY, 460, itemY + 40);
                    
                    pros::screen::set_pen(isSelected ? ACCENT_ORANGE : 0x333344);
                    pros::screen::draw_rect(240, itemY, 460, itemY + 40);
                    
                    pros::screen::set_pen(isSelected ? 0xFFFFFF : 0x888888);
                    pros::screen::print(pros::E_TEXT_SMALL, 250, itemY + 12, isSelected ? ">> %s" : "   %s", autonNames[i].c_str());
                }
            }

            pros::delay(20);
        }
    });
}

void opcontrol() { catapultControl(); }
void autonomous() { runAutonomous(); }