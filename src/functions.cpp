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


// Hold L1 to close the claw (counter-clockwise); releasing L1 stops it and it
// stays closed. Tap L2 to open the claw (clockwise) until it stalls against the
// hardstop, then hold there. Pressing L1 also cancels an L2 run.
void clawControl() {
  const int CLAW_SPEED = 200;               // green gearset max velocity (rpm)
  const uint32_t CLAW_STALL_GRACE_MS = 250; // ignore stall while spinning up
  const double CLAW_STALL_VELOCITY = 5.0;   // rpm - below this counts as stalled
  const int CLAW_STALL_TICKS = 5;           // consecutive slow ticks to confirm

  static bool wasL2Held = false;
  static bool openRunning = false;
  static uint32_t openStartMs = 0;
  static int slowTicks = 0;

  bool closeHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
  bool openHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
  bool openTapped = openHeld && !wasL2Held;
  wasL2Held = openHeld;
  uint32_t nowMs = pros::millis();

  if (closeHeld) {
    openRunning = false;
    claw.move_velocity(-CLAW_SPEED);
    return;
  }

  if (openTapped && !openRunning) {
    openRunning = true;
    openStartMs = nowMs;
    slowTicks = 0;
  }

  if (openRunning) {
    claw.move_velocity(CLAW_SPEED);
    if (nowMs - openStartMs >= CLAW_STALL_GRACE_MS &&
        std::abs(claw.get_actual_velocity()) < CLAW_STALL_VELOCITY) {
      slowTicks++;
    } else {
      slowTicks = 0;
    }
    if (slowTicks >= CLAW_STALL_TICKS) {
      openRunning = false;
    }
  }

  if (!openRunning) {
    claw.move_velocity(0); // brake mode HOLD keeps it in place
  }
}

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

    clawControl();

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

    if (armLiftUp) {
      lift.move_velocity(100);
    } else if (armLiftDown && !liftBumper.get_value()) {
      lift.move_velocity(-100);
    } else {
      lift.move_velocity(0);
    }
  }
}