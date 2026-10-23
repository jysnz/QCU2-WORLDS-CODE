#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "pid_tuner.hpp"
#include "point_planner.hpp"
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
#include "fieldctl/fieldctl.hpp"
#include "screen_task.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// VEXNET field control
// ─────────────────────────────────────────────────────────────────────────────
// true:  robot waits disabled until the VEXNET gamemaster starts a match
// false: field control off - normal driving and autonomous (practice)
// This is only the state at boot -- the VEXNET switch on the brain screen flips it live.
const bool VEXNET_ENABLED = false;

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({3, 4}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({-1, -2}, pros::MotorGears::green);

pros::MotorGroup lift({-5, 6}, pros::MotorGears::red);
pros::Motor claw(8);

pros::adi::Pneumatics clamp('B', false);
pros::adi::DigitalIn liftBumper('A');

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
    dashboard::showSplash("Calibrating IMU...", "Keep the robot still");
    chassis.calibrate();
    claw.tare_position();
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    dashboard::ScreenConfig screen;
    screen.readMotors = [](dashboard::State &s) {
        s.leftDrive.motors = dashboard::readMotors(left_motor_group, "L");
        s.rightDrive.motors = dashboard::readMotors(right_motor_group, "R");
        s.centerLeft = {"LIFT", dashboard::readMotors(lift, "Lift")};
        s.centerRight = {"MECHANISMS", {dashboard::readMotor(claw, "CLAW", "Claw")}};
    };
    // The PID tuner / point planner own the brain screen while active
    screen.screenBusy = [] { return pidTunerActive || pointPlannerActive; };
    dashboard::startScreenTask(screen);
}

void opcontrol() {
    // Hold DPAD-LEFT when driver control starts to enter the PID tuner, or
    // DPAD-UP to enter the point planner. Normal driving runs if neither is
    // held. A single instantaneous read here misses the press almost every
    // time -- you can only start pressing it once you see/feel driver
    // control begin, which is after this line would have already sampled
    // false. Poll for a short window instead so a hold that starts right at
    // opcontrol() still gets caught.
    //
    // NOTE: this used to initialize enterTuner to `true` and never set it
    // back to `false`, so the tuner ran unconditionally and jawheadControl()
    // (normal driving) never did. Fixed here -- both now default to `false`
    // and only flip on an actual press.
    homeLift();

    bool enterTuner = false;
    bool enterPlanner = false;
    for (int i = 0; i < 15; i++) { // ~300ms at 20ms/tick
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            enterTuner = true;
            break;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            enterPlanner = true;
            break;
        }
        pros::delay(20);
    }

    if (enterTuner) {
        pidTunerControl();
    } else if (enterPlanner) {
        pointPlannerControl();
    } else {
        jawheadControl();
    }
}
void autonomous() {
    homeLift();
    runAutonomous();
}