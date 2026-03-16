#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions (declared extern in motors.hpp)
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group ({-11, -12, -13, -14}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({  1,   2,   3,   4}, pros::MotorGears::green);

pros::Motor catapult_arm(7,  pros::MotorGears::red);
pros::Motor intake      (16, pros::MotorGears::green);
pros::Motor matchloader (5,  pros::MotorGears::red);
pros::Motor discore     (15, pros::MotorGears::green);
pros::Motor arm         (6,  pros::MotorGears::red);

pros::Imu            imu               (10);
pros::Rotation       horizontal_encoder(20);
pros::adi::Encoder   vertical_encoder  ('C', 'D', true);
pros::adi::Ultrasonic ultrasonic        ('A', 'B');
pros::Controller     controller        (pros::E_CONTROLLER_MASTER);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel  (&vertical_encoder,
                                                lemlib::Omniwheel::NEW_325, -2.5);

// ─── LemLib drivetrain & PID setup ───────────────────────────────────────────
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    10,                          // track width (in)
    lemlib::Omniwheel::NEW_325,
    400,                         // rpm
    2                            // horizontal drift
);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ControllerSettings lateral_controller (10, 0, 25, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller (1.8, 0, 10, 0, 0, 0,  0, 0,   0);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve   (3, 10, 1.019);

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
    left_motor_group.tare_position();
    right_motor_group.tare_position();

    pros::lcd::initialize();
    chassis.calibrate();

    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // ── On-screen diagnostics ────────────────────────────────────────────────
    static pros::Task screen_task([]() {
        const int P_X      = 240;
        const int BG_COLOR = 0x202020;

        while (true) {
            pros::screen::set_pen(BG_COLOR);
            pros::screen::fill_rect(P_X, 0, 480, 240);

            // Battery indicator
            double bat   = pros::battery::get_capacity();
            int bat_y    = 20, bat_h = 30, bat_w = 70;
            int icon_x   = P_X + 130;

            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_LARGE, P_X + 10, bat_y + 3, "BAT: %3.0f%%", bat);

            uint32_t bat_col = (bat > 60) ? 0x00FFFF : (bat > 30 ? 0xFFA500 : 0xFF0000);
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::draw_rect(icon_x, bat_y, icon_x + bat_w, bat_y + bat_h);
            pros::screen::fill_rect(icon_x + bat_w, bat_y + 8,
                                    icon_x + bat_w + 5, bat_y + bat_h - 8);
            int fill = (int)((bat / 100.0) * (bat_w - 4));
            pros::screen::set_pen(bat_col);
            pros::screen::fill_rect(icon_x + 2, bat_y + 2,
                                    icon_x + 2 + fill, bat_y + bat_h - 2);

            // Temperature rows
            auto drawRow = [&](int row_idx, const char *label, double temp) {
                int row_h = 40, start_y = 70;
                int y     = start_y + (row_idx * row_h);

                pros::screen::set_pen(0xFFFFFF);
                pros::screen::print(pros::E_TEXT_MEDIUM, P_X + 10, y + 8, label);

                int bar_x = P_X + 80, bar_w = 100, bar_h = 16, bar_y = y + 6;
                pros::screen::set_pen(0x404040);
                pros::screen::fill_rect(bar_x, bar_y, bar_x + bar_w, bar_y + bar_h);

                double stress  = std::min(temp / 60.0, 1.0);
                int    fill_w  = (int)(stress * bar_w);
                uint32_t col   = (temp > 55) ? 0xFF0000 : (temp > 45 ? 0xFFA500 : 0x00FF00);
                pros::screen::set_pen(col);
                pros::screen::fill_rect(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h);

                pros::screen::set_pen(0xFFFFFF);
                pros::screen::print(pros::E_TEXT_SMALL, bar_x + bar_w + 10, y + 8, "%.0fC", temp);
            };

            double d_temp = (left_motor_group.get_temperature() +
                             right_motor_group.get_temperature()) / 2.0;

            drawRow(0, "Drive",  d_temp);
            drawRow(1, "Cata",   catapult_arm.get_temperature());
            drawRow(2, "Intake", intake.get_temperature());
            drawRow(3, "Load",   matchloader.get_temperature());
            drawRow(4, "Disc",   discore.get_temperature());

            pros::delay(200);
        }
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// opcontrol & autonomous
// ─────────────────────────────────────────────────────────────────────────────
void opcontrol()  { catapultControl(); }
void autonomous() { runAutonomous();   }