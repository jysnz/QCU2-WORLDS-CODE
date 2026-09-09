#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::Motor intake1;
extern pros::Motor intake2;
extern pros::Motor lift1;
extern pros::Motor lift2;
extern pros::Motor bunchy;
extern pros::Motor bunchArm;

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics clamp;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Rotation rotation_sensor;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;