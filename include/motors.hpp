#pragma once

#include <cmath>
#include "pros/rtos.hpp"

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup intake2;
extern pros::MotorGroup lift;
extern pros::Motor claw_arm; // port 19: Y = forward, RIGHT = backward

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics lift_arm_orientation; // RIGHT toggles it
extern pros::adi::Pneumatics claw_piston;          // B toggles it (the claw is pneumatic only)

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
    double h = std::fmod(pros::Imu::get_heading() * 360.0 / scale, 360.0);
    return h < 0 ? h + 360.0 : h; // scaling can push a reading outside 0..360
  }
  double rawRotation() const { return pros::Imu::get_rotation(); } // unscaled, for calibration
};
extern ScaledImu imu;
extern pros::Rotation verticalRotation;   // port 1 (reversed)
extern pros::Rotation horizontalRotation; // port 20 (reversed)
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;

// Zeroes the odometry pose and makes sure it stays a real number. While the IMU recalibrates its
// rotation reads infinite, and the first odometry update after it settles adds (finite - inf) =
// -inf to the pose, which stays NaN until it is reset. Resetting again once the IMU has settled
// clears it. Returns false if the pose is still invalid after all attempts. (From the debi branch.)
inline bool resetPoseAfterCalibration() {
    for (int attempt = 0; attempt < 10; attempt++) {
        chassis.setPose(0, 0, 0);
        pros::delay(60);
        lemlib::Pose p = chassis.getPose();
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta)) return true;
    }
    return false;
}