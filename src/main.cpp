#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "imu_drift.hpp"
#include "lift_calibration.hpp"
#include "pid_tuner.hpp"
#include "auto_tuner.hpp"
#include "driver_menu.hpp"
#include "lemlib/api.hpp"
#include "run_log.hpp"
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
#include "vision.hpp"

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

LiftGroup lift({-5, 6}, pros::MotorGears::red);

pros::adi::Pneumatics claw('B', true);
pros::adi::Pneumatics claw2('C', true);
pros::adi::DigitalIn pinSensor('D'); // IR pin sensor (Pneumatics can't read its port, so DigitalIn)
pros::adi::DigitalIn liftLimitSwitch('A');

// What the IMU reads after the robot is turned a true 360 degrees (356.05 measured on the
// 15scrimcalibration robot). 360.0 = no correction.
constexpr double INERTIAL_SCALE = 356.05;
ScaledImu imu(16, INERTIAL_SCALE);
pros::Rotation verticalRotation(15);
pros::Rotation horizontalRotation(-14); // negative port = reversed, so pushing right reads positive
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Vision vision_sensor(20, pros::E_VISION_ZERO_CENTER);
// Filled at runtime by the driver menu's VISION screen from the signature
// slot stored on the sensor (captured with the VEX Vision Utility on a PC).
pros::vision_signature_s_t GOAL_SIG{};

// ─── LemLib Setup ───
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 10.25, lemlib::Omniwheel::NEW_325, 350, 2);
// Offsets in inches from the tracking center (vertical: left of center is negative,
// horizontal: behind center is negative)
constexpr float VERTICAL_WHEEL_DIAMETER = 2;   // effective diameter in inches, calibrated from 48in pushes
constexpr float HORIZONTAL_WHEEL_DIAMETER = 2; // effective diameter in inches, calibrated from 48in pushes
lemlib::TrackingWheel verticalTracking(&verticalRotation, VERTICAL_WHEEL_DIAMETER, -0.04);
lemlib::TrackingWheel horizontalTracking(&horizontalRotation, HORIZONTAL_WHEEL_DIAMETER, -1.472);
// Set to false to run odometry on the IMU + drive motors alone (to tell whether a problem is
// the tracking wheels or something else).
constexpr bool USE_TRACKING_WHEELS = true;
lemlib::OdomSensors sensors(USE_TRACKING_WHEELS ? &verticalTracking : nullptr, nullptr,
                            USE_TRACKING_WHEELS ? &horizontalTracking : nullptr, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(10, 0, 28, 3, 1, 100, 3, 500, 20);
// Exit conditions (1 deg for 100 ms / 3 deg for 500 ms) end a turn once it
// has settled -- with them at 0 every turn ran for its whole timeout and
// kD kept twitching the robot around the target the entire time.
lemlib::ControllerSettings angular_controller(4, 0.001, 43, 0, 1, 100, 3, 500, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

MotionChassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

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
    startImuDriftCorrection(); // learns the IMU's heading creep while the robot sits still
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    dashboard::ScreenConfig screen;
    screen.readMotors = [](dashboard::State &s) {
        s.leftDrive.motors = dashboard::readMotors(left_motor_group, "L");
        s.rightDrive.motors = dashboard::readMotors(right_motor_group, "R");
        s.centerLeft = {"LIFT", dashboard::readMotors(lift, "Lift")};
    };
    // The PID tuner / auto tuner / driver menu own the brain screen while active
    screen.screenBusy = [] { return pidTunerActive || autoTunerActive || driverMenuActive; };
    dashboard::startScreenTask(screen);
}

void opcontrol() {
    // Hold DPAD-LEFT when driver control starts to enter the calibration menu (PID TUNING /
    // AUTO PID TUNE / PATH PLANNER / ODOM SETUP / ... -- see driver_menu.cpp). With nothing held
    // the robot just drives. Tapping DRIVE in the menu also falls through to normal driving.
    // A single instantaneous read here misses the press almost every time -- you can only start
    // pressing it once you see/feel driver control begin -- so poll for a short window instead.
    bool enterMenu = false;
    for (int i = 0; i < 15; i++) { // ~300ms at 20ms/tick
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            enterMenu = true;
            break;
        }
        pros::delay(20);
        pros::lcd::print(0, "Press DPAD-LEFT to enter menu");

    }

    pinAutoGrabEnable(false); // (the menu's plan runs stay in charge of the lift)
    if (enterMenu)
        driverMenuControl();

    homeLift(); // lower the lift to the limit switch (angle 0) before driver control starts
    jawheadControl();
}
void autonomous() {
    pinAutoGrabEnable(false); // autonomous only auto-grabs where the plan says so (autoGrabPin)
    homeLift();
    runlog::begin("autonomous"); // recorded; sent to the laptop afterwards (tools/run_reports/)
    runAutonomous();
    runlog::end("DONE");
    runlog::end("ALL DONE")
}