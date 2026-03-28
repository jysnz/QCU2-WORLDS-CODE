#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup intake;

extern pros::Motor catapult_arm;
extern pros::Motor matchloader;
extern pros::Motor descore;
extern pros::Motor arm;
extern pros::Motor gate;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;