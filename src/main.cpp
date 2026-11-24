#include "main.h"
#include "autonomous.hpp"
#include "driver_menu.hpp"
#include "fieldctl/fieldctl.hpp"
#include "functions.hpp"
#include "lift_tuning.hpp"
#include "pid_tuner.hpp"
#include "path_planner.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "screen_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// VEXNET field control
// ─────────────────────────────────────────────────────────────────────────────
// true:  robot waits disabled until the VEXNET gamemaster starts a match
// false: field control off - normal driving and autonomous (practice)
// This is only the state at boot -- the VEXNET switch on the brain screen flips it live.
const bool VEXNET_ENABLED = false;

// true:  when driver control starts, open the driver menu straight away
// false: open it only if DPAD-LEFT is held as driver control starts (otherwise the robot just drives)
constexpr bool START_IN_DRIVER_MENU = false;

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({-8, 9, 10, -11, 12}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({-3, 4, -5, 6, -7}, pros::MotorGears::green);
pros::MotorGroup intake2({17, -16}, pros::MotorGears::green);
pros::MotorGroup lift({14, -18}, pros::MotorGears::green);
pros::Motor claw_arm(19);  // Y = forward, RIGHT = backward (see jawheadControl)

// ADI ports for the two new pistons -- change the letters here if they are wired elsewhere.
pros::adi::Pneumatics lift_arm_orientation('A', false);  // RIGHT toggles it
pros::adi::Pneumatics claw_piston('B', false);

pros::Imu imu(15);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ─── LemLib Setup ───
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 15, lemlib::Omniwheel::NEW_325, 458, 2);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(10, 0, 28, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(5.60, 0.001, 28.590, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

// ─────────────────────────────────────────────────────────────────────────────
// initialize
// ─────────────────────────────────────────────────────────────────────────────
void initialize() {
    fieldctl::setVexnet(VEXNET_ENABLED);
    driverMenuStartLink(); // the laptop window (tools/remote_touch.py) can connect from now on

    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // (No current limit set on the lift now -- add yours here: lift.set_current_limit_all(<mA>);)
    // Lift position is counted from wherever it sits at startup, so power on
    // with the lift fully down - that becomes 0.
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    lift.tare_position_all();

    claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // holds its position when Y / RIGHT are released
    liftTuningLoad();  // lift levels / start position / motor functions from the SD card (or the built-in defaults)
    dashboard::showSplash("Calibrating IMU...", "Keep the robot still");
    chassis.calibrate();

    dashboard::ScreenConfig screen;
    screen.readMotors = [](dashboard::State &s) {
        s.leftDrive.motors = dashboard::readMotors(left_motor_group, "L");
        s.rightDrive.motors = dashboard::readMotors(right_motor_group, "R");
        s.centerLeft = {"LIFT", dashboard::readMotors(lift, "Lift")};
        auto mechanisms = dashboard::readMotors(intake2, "Intake");
        mechanisms.push_back(dashboard::readMotor(claw_arm, "CARM", "Claw Arm"));
        s.centerRight = {"MECHANISMS", mechanisms};
    };
    // The driver menu and the tools it opens (PID tuner / point planner / lift tuner) own the brain screen
    screen.screenBusy = [] { return driverMenuActive || pidTunerActive || pathPlannerActive || liftTunerActive; };
    dashboard::startScreenTask(screen);
}

void opcontrol() {
    // Hold DPAD-LEFT when driver control starts to open the driver menu (PID TUNING / PATH PLANNER /
    // LIFT TUNER / TEST MOTORS / MOTOR TEMPS -- see driver_menu.cpp). With nothing held the robot
    // just drives. Tapping DRIVE in the menu also falls through to normal driving.
    // A single instantaneous read here misses the press almost every time -- you can only start
    // pressing it once you see/feel driver control begin -- so poll for a short window instead.
    bool enterMenu = START_IN_DRIVER_MENU;
    for (int i = 0; i < 15 && !enterMenu; i++) { // ~300ms at 20ms/tick
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            enterMenu = true;
            break;
        }
        pros::delay(20);
    }

    if (enterMenu) driverMenuControl();
    jawheadControl();
}
void autonomous() {
    homeLift(); // lowers the lift to the hard stop and calls that 0 (once per run; driver control reuses it)
    runAutonomous();
}