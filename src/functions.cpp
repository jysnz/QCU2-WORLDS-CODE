#include "functions.hpp"
#include "goals.hpp"
#include "motors.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>


// ─── Operator control ────────────────────────────────────────────────────────
void jawheadControl() {
  const int MAX_SPEED = 127;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;

  // When true, the IMU corrects drivetrain drift during driver control.
  static bool imu_status = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;
  static bool wasDownHeld = false;
  static bool driveReversed = false;
  static uint32_t lastReverseToggleMs = 0;
  static bool wasBHeld = false;
  static bool clampOn = false;
  static uint32_t lastClampToggleMs = 0;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;

  while (true) {

    bool armLiftUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool armLiftDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool clawOpen = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool clawClose = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
  
    uint32_t nowMs = pros::millis();

    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (driveReversed) {
      move = -move;
    }

    if (imu_status && std::abs(move) > INU_CORRECTION_MIN_MOVE &&
        std::abs(turn) < IMU_CORRECTION_MAX_TURN) {
      if (!headingLocked) {
        targetHeading = imu.get_heading();
        headingLocked = true;
      }
      double currentHeading = imu.get_heading();
      double headingError = targetHeading - currentHeading;
      while (headingError > 180.0)
        headingError -= 360.0;
      while (headingError < -180.0)
        headingError += 360.0;

      if (std::abs(headingError) > IMU_CORRECTION_THRESHOLD) {
        int correction = (int)(headingError * IMU_CORRECTION_KP);
        correction = std::clamp(correction, -20, 20);
        turn += correction;
      }
    } else {
      headingLocked = false;
    }

    left_motor_group.move(std::clamp(move + turn, -MAX_SPEED, MAX_SPEED));
    right_motor_group.move(std::clamp(move - turn, -MAX_SPEED, MAX_SPEED));

    pros::delay(10);

    if(armLiftUp) {
      lift.move_velocity(100);
    } else if(armLiftDown) {
      lift.move_velocity(-100);
    } else {
      lift.move_velocity(0);
    }

    if(clawOpen) {
      claw.move_velocity(100);
    } else if(clawClose) {
      claw.move_velocity(-100);
    } else {
      claw.move_velocity(0);
    }
  }
}

// Autonomous helpers
// Drives to the intended final pose (x, y, heading) and holds it for the rest of the autonomous
// period: if the robot ends short or gets pushed, it drives and turns back. The target is passed in
// rather than read from the odometry, because the last move can time out or settle early and the
// measured pose would then lock in the error. Never returns; PROS stops the autonomous task when
// the period ends.
void holdEndPosition(float x, float y, float theta) {
    const lemlib::Pose home(x, y, theta);
    constexpr float kPosTol = 1.0f;  // inches
    constexpr float kAngTol = 2.0f;  // degrees

    while (true) {
        const lemlib::Pose p = chassis.getPose();
        const float dx = home.x - p.x, dy = home.y - p.y;
        // A target at a goal is reached by touching the goal, since its centre can't be reached.
        const bool there = atTarget(p.x, p.y, home.x, home.y, kPosTol);
        const float dist = there ? 0.0f : std::hypot(dx, dy);
        float headErr = std::fmod(home.theta - p.theta + 540.0f, 360.0f) - 180.0f; // -180..180

        if (std::isfinite(dist) && std::isfinite(headErr)) {
            if (dist > kPosTol) {
                // Drive home, going round goals and rerouting if it stalls. A goal home sits at
                // is an intentional contact and is not avoided.
                moveToPointSafe(home.x, home.y, 2500);
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            } else if (std::abs(headErr) > kAngTol) {
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            }
        }
        pros::delay(20);
    }
}
