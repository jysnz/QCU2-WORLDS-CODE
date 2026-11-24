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
extern pros::Motor claw_arm; // port 19: Y = forward, RIGHT = backward

// ─── Pneumatics ──────────────────────────────────────────────────────────────
extern pros::adi::Pneumatics lift_arm_orientation; // RIGHT toggles it
extern pros::adi::Pneumatics claw_piston;          // B toggles it (the claw is pneumatic only)

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Controller controller;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;