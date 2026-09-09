#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

// ─── Drivetrain motor direction test ──────────────────────────────────────────
// Spins each drivetrain motor by itself (using a fresh, unreversed handle on
// its port, independent of how left_motor_group/right_motor_group currently
// have it configured) and reports whether it physically turned "+" or "-" on
// the brain screen. Use this to figure out which ports need a negative sign
// in the MotorGroup port lists in main.cpp.
//
// Run this on its own (e.g. as the selected autonomous routine) - do not run
// it at the same time as jawheadControl(), since both drive the motors.
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

// ─── Lift control ────────────────────────────────────────────────────────────
// Holding UP drives lift1/lift2 up; holding DOWN drives them down. Letting go
// of either stops (and holds, via brake mode) the lift wherever it is - there
// is no target-angle seeking. lift1 and lift2 are mirrored on the lift, so
// they always spin opposite each other (one forward, one reversed) to move
// together.
//
// The very first time DOWN is tapped (per power cycle), it doesn't just move
// the lift - it homes it: the lift drives itself down (no need to keep
// holding the button) until it stalls against the physical bottom, and that
// stalled position becomes the lift's zero/starting position (the rotation
// sensor is tared there). After that one-time homing, DOWN goes back to
// normal hold-to-lower control.
//
// rotation_sensor tracks the lift's position via get_position() - a
// continuous, tare-able centidegree count (NOT get_angle(), which is a fixed
// 0-360 wrap that set_position()/reset_position() can't zero) - and enforces
// a hard top limit so the lift stops there even while UP is still held.
//
// NOTE: LIFT_MAX_ANGLE is a placeholder - tune it to the lift's actual peak
// position (degrees above the homed bottom). Also confirm on the robot that
// UP actually raises the lift and DOWN/homing lowers it; if they're
// backwards, swap the two move_velocity() sign pairs below (and in the
// homing block), and confirm get_position() increases as the lift goes up
// from the homed bottom - flip the rotation_sensor port sign in main.cpp if
// it decreases instead, so the LIFT_MAX_ANGLE guard trips at the right end.
void liftControl() {
  const double LIFT_MAX_ANGLE = 180.0; // degrees above the homed bottom - peak/top limit
  const int LIFT_SPEED = 200;          // green gearset max velocity (rpm)

  // Homing stall detection: same idea as bunchArm's tap-to-run-until-stall -
  // "stalled" means the motor is commanded to move but its actual velocity
  // has sat near zero for a bit, i.e. it's jammed against the bottom.
  const uint32_t HOMING_STARTUP_GRACE_MS = 300; // ignore the stall check right after starting (still spinning up from rest)
  const uint32_t HOMING_STALL_TIME_MS = 150;    // velocity must stay ~0 this long to count as stalled
  const double HOMING_STALL_VELOCITY = 5.0;     // rpm

  static bool liftHomed = false;
  static bool homingInProgress = false;
  static bool wasDownHeld = false;
  static uint32_t homingStartMs = 0;
  static uint32_t homingZeroSinceMs = 0;

  bool liftUpHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
  bool liftDownHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
  uint32_t nowMs = pros::millis();

  bool downTapped = liftDownHeld && !wasDownHeld;
  wasDownHeld = liftDownHeld;
  if (!liftHomed && !homingInProgress && downTapped) {
    homingInProgress = true;
    homingStartMs = nowMs;
    homingZeroSinceMs = 0;
  }

  if (homingInProgress) {
    lift1.move_velocity(LIFT_SPEED);
    lift2.move_velocity(-LIFT_SPEED);

    if (nowMs - homingStartMs > HOMING_STARTUP_GRACE_MS) {
      double actualVel = std::abs(lift1.get_actual_velocity());
      if (actualVel < HOMING_STALL_VELOCITY) {
        if (homingZeroSinceMs == 0) homingZeroSinceMs = nowMs;
        if (nowMs - homingZeroSinceMs > HOMING_STALL_TIME_MS) {
          lift1.move_velocity(0);
          lift2.move_velocity(0);
          rotation_sensor.reset_position(); // stalled at the bottom - this is now the lift's zero/starting position
          homingInProgress = false;
          liftHomed = true;
        }
      } else {
        homingZeroSinceMs = 0; // still actually moving, reset the stall timer
      }
    }
    return; // homing owns the lift motors until it finishes
  }

  // get_position() returns PROS_ERR (INT32_MAX) if the sensor read fails
  // (not plugged in, bad port, etc.) - treat that as "position unknown" and
  // fall back to letting the driver move freely rather than silently locking
  // out a direction.
  std::int32_t rawPosition = rotation_sensor.get_position();
  bool sensorOk = rawPosition != INT32_MAX;
  double liftAngle = sensorOk ? rawPosition / 100.0 : 0.0;

  if (liftUpHeld && (!sensorOk || liftAngle < LIFT_MAX_ANGLE)) {
    lift1.move_velocity(-LIFT_SPEED);
    lift2.move_velocity(LIFT_SPEED);
  } else if (liftDownHeld && (!sensorOk || liftAngle > 0.0)) {
    lift1.move_velocity(LIFT_SPEED);
    lift2.move_velocity(-LIFT_SPEED);
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

    // Tap L1 to spin bunchArm positive until it stalls, tap L2 for negative -
    // it keeps spinning on its own (no need to hold the button) until a
    // "stall" is detected, then stops. Stall here means bunchArm is being
    // commanded to move but its actual velocity has sat near zero for a bit,
    // i.e. it's jammed against something/at its limit - not a hard current
    // threshold, since that varies by gearset.
    static bool wasL1Held = false;
    static bool wasL2Held = false;
    static int bunchArmDir = 0; // 0 = idle, 1 = running +, -1 = running -
    static uint32_t bunchArmStartMs = 0;
    static uint32_t bunchArmZeroSinceMs = 0;
    const int BUNCHARM_SPEED = 100;                   // red gearset max velocity (rpm)
    const uint32_t BUNCHARM_STARTUP_GRACE_MS = 300;    // ignore the stall check right after starting (still spinning up from rest)
    const uint32_t BUNCHARM_STALL_TIME_MS = 150;       // velocity must stay ~0 this long to count as stalled
    const double BUNCHARM_STALL_VELOCITY = 5.0;        // rpm

    bool l1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l1Tapped = l1Held && !wasL1Held;
    wasL1Held = l1Held;

    bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool l2Tapped = l2Held && !wasL2Held;
    wasL2Held = l2Held;

    if (l1Tapped) {
      bunchArmDir = 1;
      bunchArmStartMs = nowMs;
      bunchArmZeroSinceMs = 0;
    } else if (l2Tapped) {
      bunchArmDir = -1;
      bunchArmStartMs = nowMs;
      bunchArmZeroSinceMs = 0;
    }

    if (bunchArmDir != 0) {
      bunchArm.move_velocity(bunchArmDir * BUNCHARM_SPEED);

      if (nowMs - bunchArmStartMs > BUNCHARM_STARTUP_GRACE_MS) {
        double actualVel = std::abs(bunchArm.get_actual_velocity());
        if (actualVel < BUNCHARM_STALL_VELOCITY) {
          if (bunchArmZeroSinceMs == 0) bunchArmZeroSinceMs = nowMs;
          if (nowMs - bunchArmZeroSinceMs > BUNCHARM_STALL_TIME_MS) {
            bunchArmDir = 0; // stalled - stop
            bunchArm.move_velocity(0);
          }
        } else {
          bunchArmZeroSinceMs = 0; // still actually moving, reset the stall timer
        }
      }
    } else {
      bunchArm.move_velocity(0);
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
      bunchy.move_velocity(200);
    } else if (intakeBackward) {
      intake1.move_velocity(-600);
      intake2.move_velocity(600);
      bunchy.move_velocity(-200);
    } else {
      intake1.move_velocity(0);
      intake2.move_velocity(0);
      bunchy.move_velocity(0);
    }

    pros::delay(10);
  }
}