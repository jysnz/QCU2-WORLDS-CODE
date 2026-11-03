#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "pid_tuner.hpp"
#include "auto_tuner.hpp"
#include "driver_menu.hpp"
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
pros::MotorGroup left_motor_group({1, 2}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({-4, -5}, pros::MotorGears::green);
pros::Motor claw(12);
pros::Motor lift(11);
pros::adi::Pneumatics clamp('B', false);

// What the IMU reads after the robot is turned a true 360 degrees (SPIN CAL on the ODOM SETUP
// screen measures it). 360.0 = no correction; measure it on this robot before trusting it.
constexpr double INERTIAL_SCALE = 360.0;
ScaledImu imu(15, INERTIAL_SCALE);
// No tracking wheels on axl yet -- placeholder ports, not used by odometry (see motors.hpp).
pros::Rotation verticalRotation(20);
pros::Rotation horizontalRotation(19);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ─── LemLib Setup ───
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 15, lemlib::Omniwheel::NEW_325, 458, 2);
// Offsets in inches from the tracking center (vertical: left of center is negative,
// horizontal: behind center is negative). Placeholders until wheels are fitted on axl.
constexpr float VERTICAL_WHEEL_DIAMETER = 2;
constexpr float HORIZONTAL_WHEEL_DIAMETER = 2;
lemlib::TrackingWheel verticalTracking(&verticalRotation, VERTICAL_WHEEL_DIAMETER, 0);
lemlib::TrackingWheel horizontalTracking(&horizontalRotation, HORIZONTAL_WHEEL_DIAMETER, 0);
// Odometry runs on the IMU + drive motors alone until kUseTrackingWheels (motors.hpp) is true.
lemlib::OdomSensors sensors(kUseTrackingWheels ? &verticalTracking : nullptr, nullptr,
                            kUseTrackingWheels ? &horizontalTracking : nullptr, nullptr, &imu);
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
    // calibrate() can return while the IMU is still settling (its rotation reads infinite until
    // it's done, which turns the odometry pose into NaN). Wait it out (up to 5 s).
    for (int i = 0; i < 250 && imu.is_calibrating(); i++)
        pros::delay(20);
    resetPoseAfterCalibration();

    dashboard::ScreenConfig screen;
    screen.readMotors = [](dashboard::State &s) {
        s.leftDrive.motors = dashboard::readMotors(left_motor_group, "L");
        s.rightDrive.motors = dashboard::readMotors(right_motor_group, "R");
        s.centerLeft = {"MECHANISMS",
                        {dashboard::readMotor(lift, "LIFT", "Lift"), dashboard::readMotor(claw, "CLAW", "Claw")}};
    };
    // The PID tuner / auto tuner / driver menu own the brain screen while active
    screen.screenBusy = [] { return pidTunerActive || autoTunerActive || driverMenuActive; };
    dashboard::startScreenTask(screen);
}

void opcontrol() {
    // Hold DPAD-LEFT when driver control starts to enter the calibration menu (PID TUNING /
    // AUTO PID TUNE / PATH PLANNER / ODOM SETUP / ... -- see driver_menu.cpp). With nothing held
    // the robot just drives. Tapping DRIVE in the menu also falls through to normal driving.
    // A single instantaneous read misses the press almost every time -- you can only start
    // pressing it once you see/feel driver control begin -- so poll for a short window instead.
    bool enterMenu = false;
    for (int i = 0; i < 15; i++) { // ~300ms at 20ms/tick
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            enterMenu = true;
            break;
        }
        pros::delay(20);
    }

    if (enterMenu)
        driverMenuControl();

    jawheadControl();
}
void autonomous() { runAutonomous(); }