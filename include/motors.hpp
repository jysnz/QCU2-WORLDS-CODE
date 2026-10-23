#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup intake2;
extern pros::MotorGroup lift;
extern pros::Motor claw;

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics clamp;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;