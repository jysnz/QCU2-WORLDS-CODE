#include "vision.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// ─── Signature
// ──────────────────────────────────────────────────────────────── Replace
// these values with what VEXcode gives you after training on the goal
pros::vision_signature_s_t GOAL_SIG = pros::Vision::signature_from_utility(
    1,            // signature ID (1–7)
    -1243, -561,  // uMin, uMax  ← replace with your trained values
    -902,         // uMean       ← replace with your trained values
    -3547, -2871, // vMin, vMax  ← replace with your trained values
    -3209,        // vMean       ← replace with your trained values
    2.500,        // range
    0             // signature type
);

// ─── Sensor instance
// ──────────────────────────────────────────────────────────
pros::Vision vision_sensor(8); // ← change 8 to your actual port

// ─── Constants
// ────────────────────────────────────────────────────────────────
const int VISION_FOV_WIDTH_VAL = 316; // V5 vision sensor is 316px wide
const int VISION_CENTER_X = VISION_FOV_WIDTH / 2; // 158
const int VISION_MIN_AREA =
    500;                      // ignore objects smaller than this (noise filter)
const double ALIGN_KP = 0.15; // tune: lower = smoother, higher = faster
const double ALIGN_THRESHOLD = 10; // pixels from center to consider aligned
const int ALIGN_TIMEOUT = 3000;    // ms before giving up

// ─── Align to goal
// ────────────────────────────────────────────────────────────
void alignToGoal() {
  vision_sensor.set_signature(1, &GOAL_SIG);

  uint32_t startTime = pros::millis();

  while (pros::millis() - startTime < ALIGN_TIMEOUT) {

    // Get the largest object matching signature 1
    pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, 1);

    // No object found or too small (noise) — hold position and wait
    if (vision_sensor.get_object_count() == 0 ||
        (obj.width * obj.height) < VISION_MIN_AREA) {
      left_motor_group.move_velocity(0);
      right_motor_group.move_velocity(0);
      pros::delay(10);
      continue;
    }

    // Error: how far the goal center is from the camera center (px)
    double error = obj.x_middle_coord - VISION_CENTER_X;

    // Close enough — stop
    if (std::abs(error) <= ALIGN_THRESHOLD) {
      left_motor_group.move_velocity(0);
      right_motor_group.move_velocity(0);
      break;
    }

    // Proportional turn toward the goal
    double turnSpeed = error * ALIGN_KP;

    // Clamp to safe range
    if (turnSpeed > 60)
      turnSpeed = 60;
    if (turnSpeed < -60)
      turnSpeed = -60;

    // Minimum speed so motors don't stall
    if (turnSpeed > 0 && turnSpeed < 10)
      turnSpeed = 10;
    if (turnSpeed < 0 && turnSpeed > -10)
      turnSpeed = -10;

    left_motor_group.move_velocity(turnSpeed);
    right_motor_group.move_velocity(-turnSpeed);

    pros::delay(10);
  }

  // Always hard stop when done
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}