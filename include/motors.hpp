#pragma once

#include <cmath>
#include "pros/rtos.hpp"

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ─── Drivetrain ──────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// ─── Subsystem motors ────────────────────────────────────────────────────────
extern pros::MotorGroup lift; // ports 5 / 6
extern pros::Motor claw; // port 8

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics clamp;
extern pros::adi::DigitalIn liftBumper; // port A

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu; // port 16
extern pros::Rotation verticalRotation; // port 15
extern pros::Rotation horizontalRotation; // port 14
extern pros::Controller controller;

extern lemlib::Chassis chassis;

// Zeroes the odometry pose and makes sure it stays a real number. While the IMU recalibrates its
// rotation reads infinite, and the first odometry update after it settles adds (finite - inf) =
// -inf to the pose, which stays NaN until it is reset. Resetting again once the IMU has settled
// clears it. Returns false if the pose is still invalid after all attempts.
inline bool resetPoseAfterCalibration() {
    for (int attempt = 0; attempt < 10; attempt++) {
        chassis.setPose(0, 0, 0);
        pros::delay(60);
        lemlib::Pose p = chassis.getPose();
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta)) return true;
    }
    return false;
}

// ─── LemLib chassis ──────────────────────────────────────────────────────────
