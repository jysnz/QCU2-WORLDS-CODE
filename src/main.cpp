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

    // ── Cyber-HUD with Auton Selector ──
    static pros::Task screen_task([]() {
        const int BG_DARK = 0x0A0A0F;
        const int CARD_BG = 0x161B22;
        const int ACCENT_CYAN = 0x00F0FF;
        const int ACCENT_ORANGE = 0xFF8C00;
        const int ACCENT_RED = 0xFF3131;

        while (true) {
            pros::screen_touch_status_s_t status = pros::screen::touch_status();
            
            // Check for tap on the Auton Card area to cycle
            static bool wasTouched = false;
            // FIXED: Using pros:: namespace for touch enums
            if (status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
                // If top right quadrant is tapped
                if (status.x > 240 && status.y > 40 && status.y < 100) {
                    currentAutonIndex = (currentAutonIndex + 1) % autonNames.size();
                    saveAutonSelection();
                }
                wasTouched = true;
            } else if (status.touch_status == pros::E_TOUCH_RELEASED) {
                wasTouched = false;
            }

            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);
            
            // Grid lines
            pros::screen::set_pen(0x1a1a25);
            for(int i=0; i<480; i+=20) pros::screen::draw_line(i, 0, i, 240);
            for(int i=0; i<240; i+=20) pros::screen::draw_line(0, i, 480, i);

            // Header
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::draw_rect(5, 5, 475, 235);
            pros::screen::draw_line(5, 40, 475, 40);
            pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "CORE_OS // AUTON_ACTIVE: %s", autonNames[currentAutonIndex].c_str());

            // Battery
            double bat = pros::battery::get_capacity();
            pros::screen::set_pen(0x333344);
            pros::screen::fill_rect(380, 10, 465, 30);
            pros::screen::set_pen(bat > 25 ? ACCENT_CYAN : ACCENT_RED);
            pros::screen::fill_rect(380, 10, 380 + (int)(bat * 0.85), 30);

            // Tech Cards
            auto drawTechCard = [&](int x, int y, const char *name, double temp, bool highlight = false) {
                int w = 215, h = 45;
                pros::screen::set_pen(highlight ? 0x1c2533 : CARD_BG);
                pros::screen::fill_rect(x, y, x + w, y + h);
                pros::screen::set_pen(highlight ? ACCENT_ORANGE : ACCENT_CYAN);
                pros::screen::fill_rect(x, y, x + 4, y + h);
                pros::screen::set_pen(0xAAAAAA);
                pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 8, name);
                pros::screen::set_pen(temp > 55 ? ACCENT_RED : (temp > 45 ? ACCENT_ORANGE : ACCENT_CYAN));
                pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 24, "TEMP: %.1fC", temp);
            };

            // Selected Auton Card (Top Right)
            pros::screen::set_pen(0x1c2533);
            pros::screen::fill_rect(245, 50, 460, 95);
            pros::screen::set_pen(ACCENT_ORANGE);
            pros::screen::draw_rect(245, 50, 460, 95);
            pros::screen::print(pros::E_TEXT_SMALL, 255, 58, "SELECTED_ROUTINE:");
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_MEDIUM, 255, 72, "> %s", autonNames[currentAutonIndex].c_str());

            // Motor Temps
            drawTechCard(20, 50, "DRIVETRAIN", (left_motor_group.get_temperature() + right_motor_group.get_temperature())/2.0);
            drawTechCard(20, 105, "CATAPULT", catapult_arm.get_temperature());
            drawTechCard(20, 160, "INTAKE", intake.get_temperature());
            drawTechCard(245, 105, "MATCHLOAD", matchloader.get_temperature());
            drawTechCard(245, 160, "DESCORE", descore.get_temperature());

            // Footer
            pros::screen::set_pen(imu.is_calibrating() ? ACCENT_ORANGE : ACCENT_CYAN);
            pros::screen::print(pros::E_TEXT_SMALL, 20, 215, "SYSTEM_READY // IMU_HDG: %.2f", imu.get_heading());

            pros::delay(50);
        }
    });
}

void opcontrol() { catapultControl(); }
void autonomous() { runAutonomous(); }