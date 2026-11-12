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
// A MotorGroup whose get_position() survives a missing motor. PROS's own MotorGroup::get_position()
// reads only the first motor (index 0); if that one is unplugged it returns infinity, so the lift
// believes it is miles up and every "go down to angle 0" / "wait until it arrives" loop breaks.
// This one averages the motors that do answer, so the lift works on one motor or on both.
class LiftGroup : public pros::MotorGroup {
public:
  LiftGroup(std::initializer_list<std::int8_t> ports, pros::MotorGears gears)
      : pros::MotorGroup(ports, gears) {}
  double get_position(const std::uint8_t index = 0) const override {
    if (index != 0) return pros::MotorGroup::get_position(index);
    double sum = 0;
    int n = 0;
    for (double p : get_position_all()) {
      if (std::isfinite(p)) {
        sum += p;
        n++;
      }
    }
    return n ? sum / n : pros::MotorGroup::get_position(0); // none answer: the usual error value
  }
};
extern LiftGroup lift; // ports 5 / 6 (either one alone is enough)

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics claw;  // ADI B, on = extended
extern pros::adi::DigitalIn pinSensor; // ADI D, the IR sensor that sees a pin
extern pros::adi::DigitalIn liftLimitSwitch; // port A

// ─── Sensors ─────────────────────────────────────────────────────────────────
// IMU whose readings are scaled by 360 / scale, so a turn that reads `scale` degrees counts
// as exactly 360 (same idea as mikLib's inertial scale). LemLib reads heading through
// get_rotation(), so the correction reaches odometry without patching LemLib.
class ScaledImu : public pros::Imu {
  double scale;
  // Heading drift removed so far (degrees, scaled units), kept up to date by the task in
  // imu_drift.cpp. A float so the 32-bit CPU reads and writes it in one go, no lock needed.
  volatile float driftDeg = 0;

public:
  ScaledImu(std::uint8_t port, double scale) : pros::Imu(port), scale(scale) {}
  double get_rotation() const override {
    return pros::Imu::get_rotation() * 360.0 / scale - driftDeg;
  }
  double get_heading() const override {
    double h = pros::Imu::get_heading() * 360.0 / scale - driftDeg;
    h = std::fmod(h, 360.0);
    return h < 0 ? h + 360.0 : h; // scaling / drift can push a reading outside 0..360
  }
  void addDrift(double deg) { driftDeg = driftDeg + (float)deg; }  // used by imu_drift.cpp
  double drift() const { return driftDeg; }
  double rawRotation() const { return pros::Imu::get_rotation(); } // unscaled, for SPIN CAL
};
extern ScaledImu imu; // port 16
extern pros::Rotation verticalRotation; // port 15
extern pros::Rotation horizontalRotation; // port 14
extern pros::Controller controller;

// The LemLib chassis, with its motion-queue hooks made public so our own motions (see
// ownApproachTask in functions.cpp) can sit in the same queue as chassis.moveToPoint & co:
// chassis.waitUntilDone(), isInMotion() and cancelMotion() then work on them too.
class MotionChassis : public lemlib::Chassis {
  public:
    using lemlib::Chassis::Chassis;
    using lemlib::Chassis::requestMotionStart;
    using lemlib::Chassis::endMotion;
};
extern MotionChassis chassis;

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
