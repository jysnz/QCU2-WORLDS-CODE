#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

// ─── Drivetrain ──────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// ─── Subsystem motors ────────────────────────────────────────────────────────
extern pros::Motor lift; // port 11
extern pros::Motor claw; // port 12

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics clamp;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;
