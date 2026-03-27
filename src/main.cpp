#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"


// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions (declared extern in motors.hpp)
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({-11, -12, -13, -14},
                                  pros::MotorGears::green);
pros::MotorGroup right_motor_group({1, 2, 3, 4}, pros::MotorGears::green);
pros::MotorGroup intake({16, -18}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor matchloader(5, pros::MotorGears::red);
pros::Motor discore(15, pros::MotorGears::green);
pros::Motor arm(6, pros::MotorGears::red);
pros::Motor gate(17, pros::MotorGears::green);


pros::Imu imu(9);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);
pros::adi::Ultrasonic ultrasonic('A', 'B');
pros::Controller controller(pros::E_CONTROLLER_MASTER);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

// ─── LemLib drivetrain & PID setup ───────────────────────────────────────────
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group,
                              15, // track width (in)
                              lemlib::Omniwheel::NEW_325,
                              458, // rpm
                              2    // horizontal drift
);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(10, 0, 25, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(1.8, 0.02, 10, 0, 0, 0, 0, 0, 0);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

// ─────────────────────────────────────────────────────────────────────────────
// initialize
// ─────────────────────────────────────────────────────────────────────────────
void initialize() {
    pros::Task catapult_control(catapultTask, nullptr, "Catapult Task");

    catapult_arm.tare_position();
    matchloader.tare_position();
    discore.tare_position();

    pros::lcd::initialize();
    chassis.calibrate();

    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // ── Cyber-HUD On-screen Diagnostics ────────────────────────────────────────
    static pros::Task screen_task([]() {
        const int BG_DARK = 0x0A0A0F;        // Near-black blue
        const int CARD_BG = 0x161B22;        // Deep tech-slate
        const int ACCENT_CYAN = 0x00F0FF;    // Neon Cyan
        const int ACCENT_ORANGE = 0xFF8C00;  // Warning Orange
        const int ACCENT_RED = 0xFF3131;     // Critical Red

        while (true) {
            // ── Background Layer ──
            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);

            // ── UI Framing (Tactical HUD Lines) ──
            pros::screen::set_pen(0x222233);
            for (int i = 0; i < 240; i += 4) { // Scanline effect
                pros::screen::draw_line(0, i, 480, i);
            }
            
            // Outer Frame
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::draw_rect(5, 5, 475, 235);
            pros::screen::draw_line(0, 40, 480, 40); // Header divider

            // ── Header Display ──
            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "CORE_OS // THERMAL_DYNAMICS");
            
            // ── Battery HUD ──
            double bat = pros::battery::get_capacity();
            uint32_t bat_col = (bat > 50) ? ACCENT_CYAN : (bat > 20 ? ACCENT_ORANGE : ACCENT_RED);
            
            pros::screen::set_pen(0x333344);
            pros::screen::fill_rect(340, 10, 460, 30); // Battery track
            int bat_fill = (int)((bat / 100.0) * 120);
            pros::screen::set_pen(bat_col);
            pros::screen::fill_rect(340, 10, 340 + bat_fill, 30);
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_SMALL, 375, 14, "PWR: %.0f%%", bat);

            // ── Modern Robotic Card Renderer ──
            auto drawTechCard = [&](int x, int y, const char* name, double temp) {
                int w = 215, h = 52;
                uint32_t status_col = (temp < 45) ? ACCENT_CYAN : (temp < 55 ? ACCENT_ORANGE : ACCENT_RED);

                // Card Shadow/Backing
                pros::screen::set_pen(CARD_BG);
                pros::screen::fill_rect(x, y, x + w, y + h);
                
                // Left Indicator Strip
                pros::screen::set_pen(status_col);
                pros::screen::fill_rect(x, y, x + 4, y + h);

                // Label & Temp
                pros::screen::set_pen(0xAAAAAA);
                pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 8, name);
                
                pros::screen::set_pen(status_col);
                pros::screen::print(pros::E_TEXT_MEDIUM, x + 12, y + 24, "%.1f°C", temp);

                // Geometric HUD bits
                pros::screen::set_pen(0x333344);
                pros::screen::draw_rect(x + 100, y + 28, x + w - 10, y + 38); // Progress border
                
                double bar_pct = std::min(temp / 70.0, 1.0);
                pros::screen::set_pen(status_col);
                pros::screen::fill_rect(x + 102, y + 30, x + 102 + (int)(bar_pct * (w - 114)), y + 36);
                
                // Tech corner detail
                pros::screen::set_pen(status_col);
                pros::screen::draw_line(x + w - 10, y, x + w, y + 10);
            };

            // Temperature Data
            double d_temp = (left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2.0;
            double c_temp = catapult_arm.get_temperature();

            // ── Component Grid ──
            drawTechCard(20, 55,  "[ DRIVETRAIN ]", d_temp);
            drawTechCard(20, 112, "[ LEVER ]",    c_temp);
            drawTechCard(20, 169, "[ INTAKE ]", intake.get_temperature());

            drawTechCard(245, 55,  "[ MATCHLOAD ]", matchloader.get_temperature());
            drawTechCard(245, 112, "[ DESCORE ]",  discore.get_temperature());
            drawTechCard(245, 169, "[ CATAPULT ]",  arm.get_temperature());

            // ── Footer Diagnostics ──
            pros::screen::set_pen(0x444455);
            pros::screen::print(pros::E_TEXT_SMALL, 20, 222, "HW_STATUS: OK // FAN_SPD: AUTO // TEMP_CEILING: 70C");

            pros::delay(200);
        }
    });
}
// ─────────────────────────────────────────────────────────────────────────────
// opcontrol & autonomous
// ─────────────────────────────────────────────────────────────────────────────
void opcontrol() { catapultControl(); }
void autonomous() { runAutonomous(); }