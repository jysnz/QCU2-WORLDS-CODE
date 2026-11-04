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
extern pros::Motor lift; // port 11
extern pros::Motor claw; // port 12

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics clamp; // ADI B

// ─── Sensors ─────────────────────────────────────────────────────────────────
// IMU whose readings are scaled by 360 / scale, so a turn that reads `scale` degrees counts
// as exactly 360 (same idea as mikLib's inertial scale). LemLib reads heading through
// get_rotation(), so the correction reaches odometry without patching LemLib.
class ScaledImu : public pros::Imu {
  double scale;

public:
  ScaledImu(std::uint8_t port, double scale) : pros::Imu(port), scale(scale) {}
  double get_rotation() const override { return pros::Imu::get_rotation() * 360.0 / scale; }
  double get_heading() const override {
    double h = pros::Imu::get_heading() * 360.0 / scale;
    return h >= 360.0 ? h - 360.0 : h; // scaling can push a 359.x reading past 360
  }
  double rawRotation() const { return pros::Imu::get_rotation(); } // unscaled, for SPIN CAL
};
extern ScaledImu imu; // port 15
// The axl robot has no tracking wheels. These two exist so ODOM SETUP / the tuners build; they sit
// on unused ports and odometry ignores them while kUseTrackingWheels is false. Fit the wheels,
// set their real ports and offsets in main.cpp, then flip this to true.
inline constexpr bool kUseTrackingWheels = true;
extern pros::Rotation verticalRotation;   // port 17
extern pros::Rotation horizontalRotation; // port 18
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
