#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// ─── Motors ──────────────────────────────────────────────────────────────────
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern pros::Motor catapult_arm;
extern pros::Motor intake;
extern pros::Motor matchloader;
extern pros::Motor discore;
extern pros::Motor arm;
extern pros::Motor gate;

// ─── Sensors ─────────────────────────────────────────────────────────────────
extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::adi::Encoder vertical_encoder;
extern pros::adi::Ultrasonic ultrasonic;
extern pros::Controller controller;

// ─── LemLib tracking wheels ──────────────────────────────────────────────────
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

// ─── LemLib chassis ──────────────────────────────────────────────────────────
extern lemlib::Chassis chassis;