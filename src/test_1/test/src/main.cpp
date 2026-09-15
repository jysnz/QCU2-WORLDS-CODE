#include "main.h"
#include "pros/misc.hpp"
#include <algorithm>   // std::clamp
#include <cmath>       // std::pow
#include <cstdlib>     // std::abs
#include <vector>      // std::vector



bool claw_state = false;        // The current state (false = open, true = closed)
bool was_y_pressed = false;     // Tracks the button state from the previous loop
pros::ADIDigitalOut claw_solenoid('G'); // Your pneumatic port
// Motor Group Configurations]

pros::MotorGroup left_mg({-11, -12}, pros::MotorGears::green);
pros::MotorGroup right_mg({1, 2}, pros::MotorGears::green);

// Claw Motors (individual, not grouped)
pros::Motor arm(13, pros::MotorGears::red);
pros::Motor claw_right(7);

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ── Arm lift state ──

const double ARM_LIFT_DEGREES = -800.0;  

bool arm_up = false;   

// ── Gravity feedforward tuning ──
const double ARM_HORIZONTAL_DEG = -400.0;   
const int    ARM_VOLTAGE        = 12000;    
const int    GRAVITY_FF_VOLTAGE = 3000;     

// ── Drivetrain control tuning ──
const int    JOY_DEADZONE = 5;     
const double EXPO_CURVE   = 2.0;   
const int    MAX_SPEED    = 127;
const int    SLOW_SPEED   = 60;    

static bool slowMode = false;      
static bool wasSlowModeTapped = false;


int applyExpo(int raw) {
    if (std::abs(raw) < JOY_DEADZONE) return 0;
    double normalized = raw / 127.0;
    double sign   = (normalized < 0) ? -1.0 : 1.0;
    double curved = sign * std::pow(std::abs(normalized), EXPO_CURVE);
    return (int)(curved * 127.0);
}


int gravityFeedforward(double pos) {
    double angleFromHorizontal = (pos - ARM_HORIZONTAL_DEG) * (M_PI / 180.0);
    double factor = std::cos(angleFromHorizontal);
    factor = std::clamp(factor, 0.0, 1.0);
    return (int)(GRAVITY_FF_VOLTAGE * factor);
}

void initialize() {
    pros::lcd::shutdown();
    arm.set_reversed(false);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();

    static pros::Task screen_task([]() {
        const int BG_DARK          = 0x0A0A0F;
        const int CARD_BG          = 0x161B22;
        const int ACCENT_CYAN      = 0x00F0FF;
        const int ACCENT_ORANGE    = 0xFF8C00;
        const int ACCENT_RED       = 0xFF3131;

        while (true) {
            pros::screen::set_pen(BG_DARK);
            pros::screen::fill_rect(0, 0, 480, 240);

            pros::screen::set_pen(0x121217);
            for (int i = 0; i < 480; i += 40) pros::screen::draw_line(i, 0, i, 240);
            for (int i = 0; i < 240; i += 40) pros::screen::draw_line(0, i, 480, i);

            pros::screen::set_pen(ACCENT_CYAN);
            pros::screen::fill_rect(5, 5, 475, 50);
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_MEDIUM, 170, 20, "SYSTEM_THERMALS");

            auto drawTechCard = [&](int x, int y, int w, int h, const char* name, double temp, bool connected) {
                uint32_t col = !connected ? ACCENT_RED
                            : (temp < 35) ? ACCENT_CYAN
                            : (temp < 40) ? ACCENT_ORANGE
                            : ACCENT_RED;

                pros::screen::set_pen(CARD_BG);
                pros::screen::fill_rect(x, y, x + w, y + h);
                pros::screen::set_pen(col);
                pros::screen::fill_rect(x, y, x + 4, y + h);

                pros::screen::set_pen(0xAAAAAA);
                pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 8, name);
                pros::screen::set_pen(col);
                if (connected) {
                    pros::screen::print(pros::E_TEXT_MEDIUM, x + 12, y + 26, "%.1f C", temp);
                } else {
                    pros::screen::print(pros::E_TEXT_MEDIUM, x + 12, y + 26, "NULL");
                }
            };

            std::vector<double> left_temps  = left_mg.get_temperature_all();
            std::vector<double> right_temps = right_mg.get_temperature_all();
            while (left_temps.size()  < 2) left_temps.push_back(PROS_ERR_F);
            while (right_temps.size() < 2) right_temps.push_back(PROS_ERR_F);

            bool l1_ok = left_temps[0]  != PROS_ERR_F;
            bool l2_ok = left_temps[1]  != PROS_ERR_F;
            bool r1_ok = right_temps[0] != PROS_ERR_F;
            bool r2_ok = right_temps[1] != PROS_ERR_F;

            double claw_l_temp = arm.get_temperature();
            double claw_r_temp = claw_right.get_temperature();
            bool claw_l_ok = claw_l_temp != PROS_ERR_F;
            bool claw_r_ok = claw_r_temp != PROS_ERR_F;

            int card_w = 140, card_h = 70, gap = 10, row1_y = 58;
            int row2_y = row1_y + card_h + gap;

            drawTechCard(15,                    row1_y, card_w, card_h, "[ L1 - P11 ]",    left_temps[0],  l1_ok);
            drawTechCard(15 + 1*(card_w + gap),  row1_y, card_w, card_h, "[ L2 - P12 ]",    left_temps[1],  l2_ok);
            drawTechCard(15 + 2*(card_w + gap),  row1_y, card_w, card_h, "[ R1 - P1 ]",     right_temps[0], r1_ok);
            drawTechCard(15 + 3*(card_w + gap),  row1_y, card_w, card_h, "[ R2 - P2 ]",     right_temps[1], r2_ok);
            drawTechCard(15,                    row2_y, card_w, card_h, "[ ARM - P13 ]",    claw_l_temp, claw_l_ok);
            drawTechCard(15 + 1*(card_w + gap), row2_y, card_w, card_h, "[ CLAW - P7 ]",   claw_r_temp, claw_r_ok);

            pros::delay(250);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        int throttle = applyExpo(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int turn = applyExpo(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            slowMode = !slowMode;
        }

        int speedLimit = slowMode ? SLOW_SPEED : MAX_SPEED;
        left_mg.move((throttle + turn) * speedLimit / MAX_SPEED);
        right_mg.move((throttle - turn) * speedLimit / MAX_SPEED);

        bool yPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        if (yPressed && !was_y_pressed) {
            claw_state = !claw_state;
            claw_solenoid.set_value(claw_state);
        }
        was_y_pressed = yPressed;

        int armPower = 0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) armPower = ARM_VOLTAGE;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) armPower = -ARM_VOLTAGE;
        arm.move_voltage(armPower + gravityFeedforward(arm.get_position()));

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            claw_right.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            claw_right.move(-127);
        } else {
            claw_right.brake();
        }

        pros::delay(20);
    }
}

		