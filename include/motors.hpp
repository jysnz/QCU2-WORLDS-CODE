#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::Motor flipMotor;
extern pros::Motor liftMotor;

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics pneumaticA;
extern pros::adi::Pneumatics pneumaticB;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;