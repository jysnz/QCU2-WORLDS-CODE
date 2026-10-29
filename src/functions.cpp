#include "functions.hpp"
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


void homeLift() {
  const int HOME_VELOCITY = -100;       // negative = counter-clockwise (down)
  const uint32_t HOME_TIMEOUT_MS = 5000; // give up if the bumper is never hit

  uint32_t startMs = pros::millis();
  lift.move_velocity(HOME_VELOCITY);
  while (!liftBumper.get_value() && pros::millis() - startMs < HOME_TIMEOUT_MS) {
    pros::delay(10);
  }
  lift.move_velocity(0); // brake mode HOLD keeps it in place
  lift.tare_position_all();
}

// ─── Operator control ────────────────────────────────────────────────────────
void jawheadControl() {
  const int MAX_SPEED = 127;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;
  // Upper lift travel limit in motor degrees from the bumper home position
  // (see homeLift()). The lower end is stopped by the bumper itself.
  const double LIFT_UPPER_LIMIT = 482.3;

  // When true, the IMU corrects drivetrain drift during driver control.
  static bool imu_status = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;
  static bool wasDownHeld = false;
  static bool driveReversed = false;
  static uint32_t lastReverseToggleMs = 0;
  static bool wasAHeld = false;
  static bool clawsOn = true; // both claws are built extended (on) in main.cpp
  static uint32_t lastClawToggleMs = 0;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;

  while (true) {

    bool armLiftUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool armLiftDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  
    uint32_t nowMs = pros::millis();

    // Tap DOWN to reverse which way the drivetrain drives (flips "forward"
    // on the joystick to drive the robot backward, and vice versa).
    // Debounced (like the claw toggle below) so switch bounce on the physical
    // button can't register as two rapid taps wthat cancel each other out.
    bool driveReverseToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool driveReverseToggleTapped = driveReverseToggleHeld && !wasDownHeld;
    wasDownHeld = driveReverseToggleHeld;
    if (driveReverseToggleTapped && (nowMs - lastReverseToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastReverseToggleMs = nowMs;
      driveReversed = !driveReversed;
    }

    // Tap A to switch claw and claw2 on (extend) and off (retract) together.
    // Edge-detected (so holding A doesn't rapid-fire) and debounced like the DOWN button above.
    bool clawToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool clawToggleTapped = clawToggleHeld && !wasAHeld;
    wasAHeld = clawToggleHeld;
    if (clawToggleTapped && (nowMs - lastClawToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastClawToggleMs = nowMs;
      clawsOn = !clawsOn;
      if (clawsOn) {
        claw.extend();
        claw2.extend();
      } else {
        claw.retract();
        claw2.retract();
      }
    }

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

    // Show the lift angle on the controller while calibrating the limits.
    // The controller screen only updates about every 50ms, so print every
    // 100ms. Row 1 keeps clear of fieldctl's status on row 0.
    static uint32_t lastLiftPrintMs = 0;
    if (nowMs - lastLiftPrintMs >= 100) {
      lastLiftPrintMs = nowMs;
      controller.print(1, 0, "Lift: %7.1f   ", lift.get_position());
    }

    // The bumper is the home position: whenever it's pressed the lift angle is 0.
    if (liftBumper.get_value()) {
      lift.tare_position_all();
    }

    if (armLiftUp && lift.get_position() < LIFT_UPPER_LIMIT) {
      lift.move_velocity(100);
    } else if (armLiftDown && !liftBumper.get_value()) {
      lift.move_velocity(-100);
    } else {
      lift.move_velocity(0);
    }
  }
}