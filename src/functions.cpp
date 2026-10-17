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
  static bool wasYHeld = false;
  static bool clawOpen = false;
  static uint32_t lastClawToggleMs = 0;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;

  while (true) {

    bool armLiftUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool armLiftDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  
    uint32_t nowMs = pros::millis();

    // Tap DOWN to reverse which way the drivetrain drives (flips "forward"
    // on the joystick to drive the robot backward, and vice versa).
    // Debounced (like the clamp below) so switch bounce on the physical
    // button can't register as two rapid taps wthat cancel each other out.
    bool driveReverseToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool driveReverseToggleTapped = driveReverseToggleHeld && !wasDownHeld;
    wasDownHeld = driveReverseToggleHeld;
    if (driveReverseToggleTapped && (nowMs - lastReverseToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastReverseToggleMs = nowMs;
      driveReversed = !driveReversed;
    }

    // Tap B to switch the clamp cleanly on/off (edge-detected so holding B
    // doesn't rapid-fire clamp.toggle() every loop and leave it in a
    // random state).
    bool clampToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool clampToggleTapped = clampToggleHeld && !wasBHeld;
    wasBHeld = clampToggleHeld;
    if (clampToggleTapped && (nowMs - lastClampToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastClampToggleMs = nowMs;
      clampOn = !clampOn;
      if (clampOn) {
        clamp.extend();
      } else {
        clamp.retract();
      }
    }

    // Tap Y to open the claw, tap again to close it.
    bool clawToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool clawToggleTapped = clawToggleHeld && !wasYHeld;
    wasYHeld = clawToggleHeld;
    if (clawToggleTapped && (nowMs - lastClawToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastClawToggleMs = nowMs;
      clawOpen = !clawOpen;
      if (clawOpen) {
        claw.move_absolute(1500, 100);
      } else {
        claw.move_absolute(0, 100);
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

    if(armLiftUp) {
      lift.move_velocity(100);
    } else if(armLiftDown) {
      lift.move_velocity(-100);
    } else {
      lift.move_velocity(0);
    }
  }
}