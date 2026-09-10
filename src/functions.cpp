#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

static void testMotorGroupDirections(const char *label,
                                      pros::MotorGroup &group, int &y) {
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, y, "%s", label);
  y += 20;

  std::vector<std::int8_t> ports = group.get_port_all();

  for (int i = 0; i < group.size(); i++) {
    std::int8_t configuredPort = ports[i]; // negative = reversed in code
    std::uint8_t rawPort = std::abs(configuredPort);

    // Fresh, non-reversed handle on the raw port so the result reflects the
    // motor's actual wiring, not the sign already applied by the group.
    pros::Motor testMotor(rawPort);
    testMotor.move_voltage(6000);
    pros::delay(300);
    double velocity = testMotor.get_actual_velocity();
    testMotor.move_voltage(0);
    pros::delay(200); // let it coast to a stop before testing the next one

    const char *spinDirection =
        (velocity > 1)    ? "+"
        : (velocity < -1) ? "-"
                           : "? (no movement)";
    const char *configuredAs = (configuredPort < 0) ? "-" : "+";

    pros::screen::print(pros::E_TEXT_SMALL, 10, y,
                         "Port %2d: spins %s | configured %s", (int)rawPort,
                         spinDirection, configuredAs);
    y += 16;
  }

  y += 10;
}

void testDrivetrainMotorDirections() {
  pros::screen::erase();
  int y = 10;
  testMotorGroupDirections("LEFT DRIVE", left_motor_group, y);
  testMotorGroupDirections("RIGHT DRIVE", right_motor_group, y);
}

void drivetrainReset(){
  left_motor_group.tare_position();
  right_motor_group.tare_position();
}

void liftControl() {
  const double LIFT_TOP_TARGET = 1500.0;    // lift1 motor degrees - top hardstop target
  const double LIFT_BOTTOM_TARGET = 0.0;   // lift1 motor degrees - bottom hardstop target
  const int LIFT_SPEED = 200;              // green gearset max velocity (rpm)

  bool liftUpHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
  bool liftDownHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

  if (liftUpHeld) {
    lift1.move_absolute(LIFT_TOP_TARGET, LIFT_SPEED);
    lift2.move_absolute(-LIFT_TOP_TARGET, LIFT_SPEED);
  } else if (liftDownHeld) {
    lift1.move_absolute(LIFT_BOTTOM_TARGET, LIFT_SPEED);
    lift2.move_absolute(-LIFT_BOTTOM_TARGET, LIFT_SPEED);
  } else {
    lift1.move_velocity(0);
    lift2.move_velocity(0);
  }
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
  static bool wasYHeld = false;
  static bool clampOn = false;
  static uint32_t lastClampToggleMs = 0;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;

  while (true) {
    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool intakeBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    uint32_t nowMs = pros::millis();

    // Tap DOWN to reverse which way the drivetrain drives (flips "forward"
    // on the joystick to drive the robot backward, and vice versa).
    // Debounced (like the clamp below) so switch bounce on the physical
    // button can't register as two rapid taps wthat cancel each other out.
    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool downTapped = downHeld && !wasDownHeld;
    wasDownHeld = downHeld;
    if (downTapped && (nowMs - lastReverseToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastReverseToggleMs = nowMs;
      driveReversed = !driveReversed;
    }

    // Tap Y to switch the clamp cleanly on/off (edge-detected so holding Y
    // doesn't rapid-fire clamp.toggle() every loop and leave it in a
    // random state).
    bool yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool yTapped = yHeld && !wasYHeld;
    wasYHeld = yHeld;
    if (yTapped && (nowMs - lastClampToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastClampToggleMs = nowMs;
      clampOn = !clampOn;
      if (clampOn) {
        clamp.extend();
      } else {
        clamp.retract();
      }
    }

    liftControl();

    static bool wasL1Held = false;
    static bool wasL2Held = false;
    const int BUNCHARM_SPEED = 100;             // red gearset max velocity (rpm)
    const double BUNCHARM_POS_TARGET = -1500.0;    // bunchArm motor degrees - L1 target (placeholder, tune to the mechanism)
    const double BUNCHARM_NEG_TARGET = 0.0;     // bunchArm motor degrees - L2/home target (placeholder, tune to the mechanism)

    bool l1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l1Tapped = l1Held && !wasL1Held;
    wasL1Held = l1Held;

    bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool l2Tapped = l2Held && !wasL2Held;
    wasL2Held = l2Held;

    if (l1Tapped) {
      bunchArm.move_absolute(BUNCHARM_POS_TARGET, BUNCHARM_SPEED);
    } else if (l2Tapped) {
      bunchArm.move_absolute(BUNCHARM_NEG_TARGET, BUNCHARM_SPEED);
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

    if (intakeForward) {
      intake1.move_velocity(600);
      intake2.move_velocity(-600);
      bunchy.move_velocity(-200);
    } else if (intakeBackward) {
      intake1.move_velocity(-600);
      intake2.move_velocity(600);
      bunchy.move_velocity(200);
    } else {
      intake1.move_velocity(0);
      intake2.move_velocity(0);
      bunchy.move_velocity(0);
    }

    pros::delay(10);
  }
}