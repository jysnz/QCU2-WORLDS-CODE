#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>

// ─── Drivetrain motor direction test ──────────────────────────────────────────
// Spins each drivetrain motor by itself (using a fresh, unreversed handle on
// its port, independent of how left_motor_group/right_motor_group currently
// have it configured) and reports whether it physically turned "+" or "-" on
// the brain screen. Use this to figure out which ports need a negative sign
// in the MotorGroup port lists in main.cpp.
//
// Run this on its own (e.g. as the selected autonomous routine) - do not run
// it at the same time as catapultControl(), since both drive the motors.
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

// ─── Operator control ────────────────────────────────────────────────────────
void catapultControl() {
  const int MAX_SPEED = 127;
  const int SLOW_SPEED = 50;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;

  // When true, the IMU corrects drivetrain drift during driver control.
  static bool imu_status = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;

  while (true) {
    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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

    int maxSpeed = downHeld ? SLOW_SPEED : MAX_SPEED;
    left_motor_group.move(std::clamp(move + turn, -maxSpeed, maxSpeed));
    right_motor_group.move(std::clamp(move - turn, -maxSpeed, maxSpeed));
  }
}

// ─── Lift + flip control ─────────────────────────────────────────────────────
// Drives a motor toward targetAngle using the motor's own built-in encoder as
// feedback (no external rotation sensor). Shared by the lift and flip motors
// below. move_absolute() runs the motor's onboard profiled-movement PID, so
// this just has to (re)issue the command and report whether the motor has
// settled within toleranceDeg. Stops the motor (rather than reporting bogus
// progress) if the encoder read fails - get_position() returns PROS_ERR
// (INT32_MAX) when the motor isn't plugged in / on a bad port. Returns true
// once the motor has arrived (within tolerance) or the encoder can't confirm
// a position at all - either way there's nothing left for this motor to do
// right now, which callers use to know when it's safe to move on to
// something that should wait for it.
static bool moveTowardAngle(pros::Motor &motor, double targetAngle, int speed,
                             double toleranceDeg) {
  std::int32_t rawPosition = motor.get_position();
  bool encoderOk = rawPosition != INT32_MAX;

  if (!encoderOk) {
    motor.move_velocity(0);
    return true;
  }

  double angleError = targetAngle - rawPosition;
  if (std::abs(angleError) <= toleranceDeg) {
    motor.move_velocity(0);
    return true;
  }

  motor.move_absolute(targetAngle, speed);
  return false;
}

// R1 tap -> lift to LIFT_R1_ANGLE, flip synced to FLIP_R1_ANGLE.
// R2 tap -> lift to LIFT_R2_ANGLE, flip synced to FLIP_R2_ANGLE.
// A tap   -> flip overridden to FLIP_A_ANGLE, independent of the lift, until
//            the next R1/R2 tap re-syncs it.
// After an R1/R2 tap, the lift moves first and the flip holds still until
// the lift settles at its target - only then does the flip start moving
// toward its paired angle. An A tap is a direct manual command, so it moves
// the flip immediately without waiting on the lift.
// Runs as its own task (see opcontrol()) so both motors move in the
// background alongside catapultControl.
void liftFlipControl() {
  const double LIFT_R1_ANGLE = 0.0;
  const double LIFT_R2_ANGLE = -10.0;
  const double FLIP_R1_ANGLE = 0.0;    // paired with lift's R1 target
  const double FLIP_R2_ANGLE = -200.0; // paired with lift's R2 target
  const double FLIP_A_ANGLE = 90.0;    // manual override, independent of lift
  const int LIFT_SPEED = 100;
  const int FLIP_SPEED = 100;
  const double ANGLE_TOLERANCE = 1.0; // degrees

  // Start each target at wherever the lift/flip actually are right now,
  // not a hardcoded angle - otherwise the motors would immediately drive
  // toward LIFT_R1_ANGLE/FLIP_R1_ANGLE the instant driver control begins,
  // before the driver has tapped anything. If an encoder read fails, fall
  // back to its R1 angle since that's the only value we have to guess with.
  auto currentAngleOr = [](pros::Motor &motor, double fallback) {
    std::int32_t rawPosition = motor.get_position();
    return (rawPosition != INT32_MAX) ? (double)rawPosition : fallback;
  };

  static double liftTarget = currentAngleOr(liftMotor, LIFT_R1_ANGLE);
  static double flipTarget = currentAngleOr(flipMotor, FLIP_R1_ANGLE);
  static bool wasR1Held = false;
  static bool wasR2Held = false;
  static bool wasAHeld = false;
  // While true, the flip holds still and waits for the lift to reach
  // liftTarget before it's allowed to start moving toward flipTarget.
  static bool flipWaitingOnLift = false;

  while (true) {
    bool r1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool aHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool r1Tapped = r1Held && !wasR1Held;
    bool r2Tapped = r2Held && !wasR2Held;
    bool aTapped = aHeld && !wasAHeld;
    wasR1Held = r1Held;
    wasR2Held = r2Held;
    wasAHeld = aHeld;

    if (r1Tapped) {
      liftTarget = LIFT_R1_ANGLE;
      flipTarget = FLIP_R1_ANGLE; // re-sync - overrides any earlier A tap
      flipWaitingOnLift = true;   // lift moves first, flip waits its turn
    } else if (r2Tapped) {
      liftTarget = LIFT_R2_ANGLE;
      flipTarget = FLIP_R2_ANGLE;
      flipWaitingOnLift = true;
    }

    if (aTapped) {
      flipTarget = FLIP_A_ANGLE; // manual override until next R1/R2 tap
      flipWaitingOnLift = false; // direct command - don't wait on the lift
    }

    bool liftSettled =
        moveTowardAngle(liftMotor, liftTarget, LIFT_SPEED, ANGLE_TOLERANCE);
    if (liftSettled) {
      flipWaitingOnLift = false;
    }

    if (flipWaitingOnLift) {
      flipMotor.move_velocity(0); // hold - not the flip's turn yet
    } else {
      moveTowardAngle(flipMotor, flipTarget, FLIP_SPEED, ANGLE_TOLERANCE);
    }

    pros::delay(10);
  }
}