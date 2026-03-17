#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

// ─── Constants ───────────────────────────────────────────────────────────────
const float PI = 3.14159f;
const double wheelDiameter = 3.25; // inches
const double ticksPerRev = 360.0;  // motor degrees per revolution

// ─── Catapult state
// ───────────────────────────────────────────────────────────
enum CatapultState { CAT_IDLE, CAT_FIRING, CAT_RELOADING };
static CatapultState catState = CAT_IDLE;
static int catAttempts = 0;
static bool shotSuccess = false;
static int stalledTime = 0;

const int LOAD_POS = 0;
const int FIRE_POS = -600;
const int CAT_SPEED = 200;
const int STALL_TIME = 250;
const int CHECK_DELAY = 10;
const int MAX_ATTEMPTS = 10;

// ─── Intake stall detection state
// ─────────────────────────────────────────────
static int intakeCurrentVelocity =
    0; // Current direction: 1=forward, -1=reverse, 0=stopped
static int intakeStallTime = 0;       // Time motor has been stalled
static bool intakeAutoReverse = true; // Enable/disable auto-reverse on stall

const int INTAKE_STALL_THRESHOLD = 5; // Velocity threshold to detect stall
const int INTAKE_STALL_DELAY = 300;   // ms before reversing on stall
const int INTAKE_REVERSE_DURATION =
    200; // ms to run in reverse after stall detect

// ─── Catapult outtake timing
// ──────────────────────────────────────────────────
static int catReloadTime = 0;         // Time spent in reload state
static bool catShouldOuttake = false; // Flag to trigger outtake after reload

// ─── Odometry helpers
// ─────────────────────────────────────────────────────────
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * (float)wheelDiameter;
}

double inchesToDegrees(double inches) {
  double wheelCircumference = PI * wheelDiameter;
  return (inches / wheelCircumference) * 360.0;
}

// ─── Intake stall detection task (runs in background) ────────────────────────
void intakeTask(void *) {
  while (true) {
    double currentVel = intake.get_actual_velocity();

    // Determine current intake direction based on velocity
    if (std::abs(currentVel) > INTAKE_STALL_THRESHOLD) {
      intakeCurrentVelocity = (currentVel > 0) ? 1 : -1;
      intakeStallTime = 0; // Reset stall timer when moving
    } else if (intakeCurrentVelocity != 0) {
      // Motor is stalled (was moving, now stopped)
      intakeStallTime += CHECK_DELAY;

      // If stalled long enough, reverse direction
      if (intakeStallTime >= INTAKE_STALL_DELAY && intakeAutoReverse) {
        int savedDirection = intakeCurrentVelocity;
        int reverseVelocity = -savedDirection * 600; // Opposite direction
        intake.move_velocity(reverseVelocity);

        // Run in reverse for a duration, but check if pause was pressed
        uint32_t reverseStart = pros::millis();
        while (pros::millis() - reverseStart < INTAKE_REVERSE_DURATION) {
          if (intakeCurrentVelocity == 0) {
            // Pause was pressed, stop immediately
            intake.move_velocity(0);
            intakeStallTime = 0;
            break;
          }
          pros::delay(10);
        }

        // Resume original direction only if not paused
        if (intakeCurrentVelocity != 0) {
          int resumeVelocity = savedDirection * 600;
          intake.move_velocity(resumeVelocity);
        }
        intakeStallTime = 0;
      }
    }

    pros::delay(CHECK_DELAY);
  }
}

// ─── Catapult task (runs in background) ──────────────────────────────────────
void catapultTask(void *) {
  while (true) {
    double pos = catapult_arm.get_position();
    double vel = std::abs(catapult_arm.get_actual_velocity());

    switch (catState) {
    case CAT_IDLE:
      break;

    case CAT_FIRING:
      if (pos <= FIRE_POS + 25) {
        shotSuccess = true;
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);
        catReloadTime = 0;
        break;
      }
      if (vel < 5)
        stalledTime += CHECK_DELAY;
      else
        stalledTime = 0;

      if (stalledTime >= STALL_TIME) {
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);
        catReloadTime = 0;
      }
      break;

    case CAT_RELOADING:
      catReloadTime += CHECK_DELAY;
      if (std::abs(pos - LOAD_POS) < 10) {
        // Trigger outtake when catapult reaches home position
        if (shotSuccess) {
          catShouldOuttake = true;
          intake.move_velocity(-600); // Outtake
          pros::delay(200);
          intake.move_velocity(0); // Stop after brief outtake
        }

        if (shotSuccess || ++catAttempts >= MAX_ATTEMPTS) {
          catState = CAT_IDLE;
          catAttempts = 0;
          shotSuccess = false;
          catShouldOuttake = false;
        } else {
          catState = CAT_FIRING;
          stalledTime = 0;
          catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);
        }
      }
      break;
    }

    pros::delay(CHECK_DELAY);
  }
}

void startCatapultShoot() {
  if (catState != CAT_IDLE)
    return;
  catAttempts = 0;
  stalledTime = 0;
  shotSuccess = false;
  catState = CAT_FIRING;
  catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);
}

// ─── Drive functions
// ──────────────────────────────────────────────────────────
void drive_for_inches(double maxSpeed, double inches) {
  double targetDegrees = inchesToDegrees(std::abs(inches));
  double direction = (inches > 0) ? 1.0 : -1.0;

  left_motor_group.tare_position();
  right_motor_group.tare_position();
  imu.tare_rotation();

  const double accelRate = 2.0;
  const double decelStart = 0.6;
  const double kP = 0.8;

  double currentSpeed = 0;
  double decelPoint = targetDegrees * decelStart;
  uint32_t startTime = pros::millis();
  uint32_t timeout = (uint32_t)(std::abs(inches) * 100);

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;
    double heading = imu.get_rotation();
    double correction = heading * kP;

    if (avgPos < decelPoint) {
      currentSpeed += accelRate;
      if (currentSpeed > maxSpeed)
        currentSpeed = maxSpeed;
    } else {
      double remaining = targetDegrees - avgPos;
      currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
      if (currentSpeed < 10)
        currentSpeed = 10;
    }

    if (avgPos >= targetDegrees - 2 || pros::millis() - startTime > timeout)
      break;

    left_motor_group.move_velocity((currentSpeed + correction) * direction);
    right_motor_group.move_velocity((currentSpeed - correction) * direction);
    pros::delay(10);
  }

  // Smooth stop
  double lastSpeed = std::max(currentSpeed, 10.0);

  while (lastSpeed > 0) {
    double heading = imu.get_rotation();
    double correction = heading * kP;
    left_motor_group.move_velocity((lastSpeed + correction) *
                                   direction); // swap
    right_motor_group.move_velocity((lastSpeed - correction) *
                                    direction); // swap
    lastSpeed -= 2;
    pros::delay(10);
  }

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void turn_to_heading(double targetHeading, double maxSpeed) {
  const double kP = 1.5;
  const double threshold = 1.0;

  uint32_t startTime = pros::millis();
  const uint32_t timeout = 3000;

  while (true) {
    double heading = imu.get_rotation();
    double error = targetHeading - heading;

    while (error > 180)
      error -= 360;
    while (error < -180)
      error += 360;

    if (std::abs(error) <= threshold || pros::millis() - startTime > timeout)
      break;

    double speed = error * kP;
    if (speed > maxSpeed)
      speed = maxSpeed;
    if (speed < -maxSpeed)
      speed = -maxSpeed;
    if (speed > 0 && speed < 10)
      speed = 10;
    if (speed < 0 && speed > -10)
      speed = -10;

    left_motor_group.move_velocity(speed);
    right_motor_group.move_velocity(-speed);
    pros::delay(10);
  }

  // Smooth stop
  double lastSpeed = 15;
  double heading = imu.get_rotation();
  double error = targetHeading - heading;
  double stopDir = (error > 0) ? 1.0 : -1.0;

  while (lastSpeed > 0) {
    left_motor_group.move_velocity(lastSpeed * stopDir);
    right_motor_group.move_velocity(-lastSpeed * stopDir);
    lastSpeed -= 2;
    pros::delay(10);
  }

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

// ─── Utility functions
// ────────────────────────────────────────────────────────
void wall_reset(int voltage, int settleTime) {
  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(voltage);

  int stalled = 0;
  while (stalled < settleTime) {
    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());
    if (lv < 3 && rv < 3)
      stalled += 10;
    else
      stalled = 0;
    pros::delay(10);
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);
  pros::delay(50);
}

void wall_reset_v2(int voltage, int settleTime, int direction, int timeout) {
  voltage = std::abs(voltage) * direction;
  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(voltage);

  int stalled = 0, elapsed = 0;
  while (stalled < settleTime && elapsed < timeout) {
    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());
    if (lv < 3 && rv < 3)
      stalled += 10;
    else
      stalled = 0;
    pros::delay(10);
    elapsed += 10;
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);
  pros::delay(50);
}

void catapultShootForAuto(double SPEED) {
  const int L_POS = 0, F_POS = -600;
  const int ST = 250, CD = 10, MA = 10;

  for (int attempt = 0; attempt < MA; attempt++) {
    bool success = false;
    catapult_arm.move_absolute(F_POS, SPEED);
    int st = 0;

    while (true) {
      pros::delay(CD);
      double pos = catapult_arm.get_position();
      double vel = std::abs(catapult_arm.get_actual_velocity());
      if (pos <= F_POS + 25) {
        success = true;
        break;
      }
      if (vel < 5)
        st += CD;
      else
        st = 0;
      if (st >= ST)
        break;
    }

    catapult_arm.move_absolute(L_POS, SPEED);
    while (std::abs(catapult_arm.get_position() - L_POS) > 10)
      pros::delay(10);

    if (success)
      return;
  }
  catapult_arm.move_velocity(0);
}

// ─── Operator control
// ─────────────────────────────────────────────────────────
void catapultControl() {
  const int MAX_SPEED = 127;
  const int SLOW_SPEED = 50;
  const double IMU_CORRECTION_KP =
      0.6; // Proportional gain for heading correction
  const int IMU_CORRECTION_MIN_MOVE = 15; // Min move input to enable correction
  const int IMU_CORRECTION_MAX_TURN =
      5; // Max turn input to keep correction active
  const double IMU_CORRECTION_THRESHOLD =
      2.0; // Degrees of drift to trigger correction

  static bool controlsReversed = false;
  static bool wasDownHeld = false;
  static bool armRaised = false;
  static bool wasArmHeld = false;
  static double targetHeading = 0.0; // Target heading for straight driving
  static bool headingLocked =
      false; // Whether we're correcting for a target heading

  while (true) {
    pros::delay(20);

    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool catapultBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool armHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool armTapped = armHeld && !wasArmHeld;
    wasArmHeld = armHeld;
    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool downTapped = downHeld && !wasDownHeld;
    wasDownHeld = downHeld;

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (downTapped)
      controlsReversed = !controlsReversed;
    if (controlsReversed)
      move = -move;

    // ─── IMU Heading Correction for Straight Driving ──────────────────────
    // Lock onto heading when driving straight with minimal turn input
    if (std::abs(move) > IMU_CORRECTION_MIN_MOVE &&
        std::abs(turn) < IMU_CORRECTION_MAX_TURN) {
      if (!headingLocked) {
        // Lock onto current heading
        targetHeading = imu.get_heading();
        headingLocked = true;
      }

      // Calculate heading error
      double currentHeading = imu.get_heading();
      double headingError = targetHeading - currentHeading;

      // Normalize error to [-180, 180]
      while (headingError > 180.0)
        headingError -= 360.0;
      while (headingError < -180.0)
        headingError += 360.0;

      // Apply correction only if drift is significant
      if (std::abs(headingError) > IMU_CORRECTION_THRESHOLD) {
        int correction = (int)(headingError * IMU_CORRECTION_KP);
        correction =
            std::clamp(correction, -20, 20); // Limit correction magnitude
        turn += correction;
      }
    } else {
      // Release heading lock when player starts turning or stops moving
      headingLocked = false;
    }
    // ─────────────────────────────────────────────────────────────────────

    int maxSpeed = downHeld ? SLOW_SPEED : MAX_SPEED;
    left_motor_group.move(std::clamp(move + turn, -maxSpeed, maxSpeed));
    right_motor_group.move(std::clamp(move - turn, -maxSpeed, maxSpeed));

    if (catapultBtn)
      startCatapultShoot();

    if (armTapped) {
      armRaised = !armRaised;
      arm.move_absolute(armRaised ? 900 : 0, 200);
    }

    if (discoreDown)
      discore.move_absolute(0, 200);
    else if (discoreUp)
      discore.move_absolute(800, 200);

    if (matchLoadUp && !matchLoadDown) {
      matchloader.move_absolute(0, 100);
      discore.move_absolute(800, 200);
    } else if (matchLoadDown && !matchLoadUp) {
      matchloader.move_absolute(1400, 100);
      discore.move_absolute(0, 200);
    }

    if (intakePause) {
      intake.move_velocity(0);
      intakeCurrentVelocity = 0;
      intakeStallTime = 0;
    } else if (intakeForward && !intakeReverse) {
      intake.move_velocity(600);
      intakeCurrentVelocity = 1; // Track that intake is moving forward
    } else if (intakeReverse && !intakeForward) {
      intake.move_velocity(-600);
      intakeCurrentVelocity = -1; // Track that intake is moving reverse
    }
  }
}