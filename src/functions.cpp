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
static bool intakeWasManual =
    false; // Tracks if intake was already running before shoot
static bool smoothShootEnabled = false; // Enable smooth deceleration on shoot
static uint32_t lastMidGoalFireTime =
    0; // Timestamp of last fire in mid goal state

const int LOAD_POS = 0;
const int FIRE_POS = -800;
const int CAT_SPEED = 200;
const int CAT_LOW_SPEED = 80;
const int STALL_TIME = 250;
const int CHECK_DELAY = 10;
const int MAX_ATTEMPTS = 10;
const int MID_GOAL_FIRE_DELAY = 500; // ms delay after firing in mid goal state

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

bool isMatchloadHoming = false;
bool isDescoreHoming = false;
bool isLeverResetting = false;

// ─── Odometry helpers
// ─────────────────────────────────────────────────────────
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * (float)wheelDiameter;
}

double inchesToDegrees(double inches) {
  double wheelCircumference = PI * wheelDiameter;
  return (inches / wheelCircumference) * 360.0;
}

// ─── Descore Homing Task ─────────────────────────────────────────────────────
void descoreHomingTask(void *param) {
  isDescoreHoming = true;

  // Step 1: Move positively by 40
  descore.move_absolute(180, 100);
  pros::delay(500); // Wait for movement to finish

  // Step 3: Tare and finish
  descore.tare_position();
  descore.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  isDescoreHoming = false;
}

void startDescoreHoming() { pros::Task homing_task(descoreHomingTask); }

void descoreHoming() { startDescoreHoming(); }

void matchloadHomingTask(void *param) {
  isMatchloadHoming = true; // Block other functions from using the motor

  matchloader.move_voltage(5000);
  pros::delay(150);

  int timeout = 0;
  while (std::abs(matchloader.get_actual_velocity()) > 2) {
    pros::delay(20);
    timeout += 20;
    if (timeout > 2000)
      break;
  }

  matchloader.move_voltage(0);
  pros::delay(100);

  // Move up slightly
  matchloader.move_relative(-40, 100);

  // Wait for movement to finish
  pros::delay(500);

  matchloader.tare_position();
  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  isMatchloadHoming = false; // Release the lock
}

// Wrapper for the Task
void startMatchloadHoming() { pros::Task homing_task(matchloadHomingTask); }

// Legacy function name for compatibility (calls the task)
void matchloadHoming() { startMatchloadHoming(); }

void drivetrainLock() {
  left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void curve_imu(double distance, double targetHeading, double maxSpeed,
               double kp, double minSpeed) {
  double targetPosition = inchesToDegrees(distance);
  left_motor_group.tare_position();
  right_motor_group.tare_position();

  while (true) {
    double currentPosition = (std::abs(left_motor_group.get_position()) +
                              std::abs(right_motor_group.get_position())) /
                             2.0;
    double error = targetPosition - currentPosition;

    if (std::abs(error) < 10)
      break;

    double currentHeading = imu.get_heading();
    double headingError = targetHeading - currentHeading;

    while (headingError > 180.0)
      headingError -= 360.0;
    while (headingError < -180.0)
      headingError += 360.0;

    int correction = (int)(headingError * kp);
    int speed = (error > 0) ? maxSpeed : -maxSpeed;

    // Slow down as we approach the target
    if (std::abs(error) < 300) {
      double scaledSpeed = (std::abs(error) / 300.0) * maxSpeed;
      speed = (error > 0) ? std::max(scaledSpeed, minSpeed)
                          : -std::max(scaledSpeed, minSpeed);
    }

    left_motor_group.move(speed + correction);
    right_motor_group.move(speed - correction);

    pros::delay(10);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void delay(int delay) { pros::delay(delay); }

void gateOpen() { gate.move_absolute(0,200); }

void gateMidOpen() { gate.move_absolute(0, 200); }

void gateClose() { gate.move_absolute(-550, 200); }

void descoreUp() {
  if (isDescoreHoming)
    return;
  descore.move_absolute(0, 200);
}

void descoreDown() {
  if (isDescoreHoming)
    return;
  descore.move_absolute(-210, 200);
}

void descoreDownMiddle() { descore.move_absolute(300, 200); }

void matchloadUp() {
  if (isMatchloadHoming)
    return;
  matchloader.move_absolute(-450, 200);
}

void matchloadDown() {
  if (isMatchloadHoming)
    return;
  matchloader.move_absolute(0, 200);
}

void gateCloseMid() { gate.move_absolute(-400, 200); }

void gateReset() { gate.move_absolute(0, 200); }

void midGoalArm() {
  arm.move_absolute(1300, 200);
  descore.move_absolute(-300, 200);
  gate.move_absolute(-450, 200);
}

void underGoalArm() {
  arm.move_absolute(2000, 200);
  gate.move_absolute(0, 200); // Ensure gate is closed in under goal position
  descore.move_absolute(-320, 200);
}

void longGoalArm() {
  arm.move_absolute(0, 200);
  descoreUp();
}

void intakeBlock() { intake.move_velocity(-600); }

void outtakeBlock(double speed) { intake.move_velocity(speed); }

void intakeStop() { intake.move_velocity(0); }

void reset() { chassis.setPose(0, 0, 0); }

void leverReset() {
  isLeverResetting = true; // Block other functions from using the motor

  catapult_arm.move_voltage(5000); // Spin negatively at high voltage
  pros::delay(150);

  int timeout = 0;
  while (std::abs(catapult_arm.get_actual_velocity()) > 2) {
    pros::delay(20);
    timeout += 20;
    if (timeout > 2000)
      break; // Safety timeout
  }

  catapult_arm.move_voltage(0);
  pros::delay(100);

  catapult_arm.tare_position();

  isLeverResetting = false; // Release the lock
}

// --- ARM STATE TRACKING ---
enum ArmState { LONG_GOAL, MID_GOAL, UNDER_GOAL };
static ArmState currentArmState = LONG_GOAL; // Default position

// ─── Catapult task (Updated with Dynamic Intake Logic) ───────────────────────
void catapultTask(void *) {
  while (true) {
    // Skip processing if lever is being reset
    if (isLeverResetting) {
      pros::delay(CHECK_DELAY);
      continue;
    }

    double pos = catapult_arm.get_position();
    double vel = std::abs(catapult_arm.get_actual_velocity());

    int dynamicFirePos = (currentArmState == MID_GOAL) ? -700 : FIRE_POS;
    double remaining = std::abs(dynamicFirePos - pos);

    switch (catState) {
    case CAT_IDLE:
      break;

    case CAT_FIRING:
      // In Firing state, command the gate to stay open to ensure it doesn't
      // close prematurely
      // gateOpen();

      if (pos <= dynamicFirePos + 25) {
        shotSuccess = true;
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);
        intake.move_velocity(600); // Outtake
        pros::delay(350);

        // Firing success: Rotate positive to bring the next ball in
        intake.move_velocity(-600);

        catReloadTime = 0;
        break;
      }

      // Apply voltage to drive to firing position
      if (smoothShootEnabled) {
        double error = std::abs(dynamicFirePos - pos);

        int voltage = 12000; // full power

        if (error < 400)
          voltage = 9000;
        if (error < 250)
          voltage = 7000;
        if (error < 150)
          voltage = 5000;
        if (error < 80)
          voltage = 3500;

        catapult_arm.move_voltage(-voltage);
      } else {
        // Apply constant high voltage to drive through ball resistance
        catapult_arm.move_voltage(-12000);
      }

      // Stall detection: only trigger if we're not moving AND far from target
      // Increased threshold to 150ms for more reliability with ball
      if (remaining > 100 && vel < 3)
        stalledTime += CHECK_DELAY;
      else
        stalledTime = 0;

      // Increased stall time to 500ms to allow more motor effort
      if (stalledTime >= 500) {
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);

        // Stall case: reverse intake to clear jam
        intake.move_velocity(600);

        catReloadTime = 0;
      }
      break;

    case CAT_RELOADING:
      catReloadTime += CHECK_DELAY;

      // If the arm is returning after a stall, ensure it switches to positive
      // rotation once it begins moving back toward home.
      if (intake.get_target_velocity() < 0 && std::abs(pos - LOAD_POS) > 50) {
        intake.move_velocity(-600);
      }

      if (std::abs(pos - LOAD_POS) < 10) {
        // Stop automatic intake move only if it wasn't intaking manually before
        if (!intakeWasManual) {
          intake.move_velocity(0);
        }

        if (shotSuccess) {
          catShouldOuttake = true;
        }

        if (shotSuccess || ++catAttempts >= MAX_ATTEMPTS) {
          catState = CAT_IDLE;
          catAttempts = 0;
          shotSuccess = false;
          catShouldOuttake = false;
        } else {
          catState = CAT_FIRING;
          stalledTime = 0;

          // Clear path again for the retry
          intake.move_velocity(600);

          catapult_arm.move_absolute(dynamicFirePos, CAT_SPEED);
        }
      }
      break;
    }

    pros::delay(CHECK_DELAY);
  }
}

void startCatapultShoot(bool smoothShoot, bool lowSpeed) {
  gate.set_brake_mode(
      pros::E_MOTOR_BRAKE_HOLD); // Ensure gate doesn't resist movement
  if (catState != CAT_IDLE)
    return;

  // Check 500ms cooldown delay if in mid goal state (asynchronous)
  if (currentArmState == MID_GOAL) {
    uint32_t currentTime = pros::millis();
    uint32_t timeSinceLastFire = currentTime - lastMidGoalFireTime;
    if (timeSinceLastFire < MID_GOAL_FIRE_DELAY) {
      return; // Still in cooldown, ignore fire request
    }
  }

  int dynamicFirePos = (currentArmState == MID_GOAL) ? -700 : FIRE_POS;
  int dynamicGatePos = (currentArmState == MID_GOAL) ? -40 : 30;

  // Update state: Is the intake currently being used by the driver?
  intakeWasManual =
      (std::abs(intake.get_actual_velocity()) > INTAKE_STALL_THRESHOLD);

  // Start with intake negative to gather the ball
  // If catapult_arm stalls (blocked by a ball), catapultTask will switch to
  // positive (outtake) to clear jam
  intake.move_velocity(-600);

  // Command gate to open when firing begins
  gate.move_absolute(-200, 200);
  if (currentArmState == MID_GOAL) {
    gateMidOpen();
            } else {
    gateOpen();
  }

  catAttempts = 0;
  stalledTime = 0;
  shotSuccess = false;
  smoothShootEnabled = smoothShoot;
  catState = CAT_FIRING;

  // Record fire time for mid goal cooldown
  if (currentArmState == MID_GOAL) {
    lastMidGoalFireTime = pros::millis();
  }
  // Don't use move_absolute - let voltage control in catapultTask handle it

  // For Autonomous: wait until the arm is back at home before returning control
  // Only if this is being called from a blocking context (like Autonomous)
  // Check if autonomous or competition state
  while (pros::competition::is_autonomous() && catState != CAT_IDLE) {
    pros::delay(10);
  }
}

// ─── Intake Task (Position-Based Control) ───────────────────────────────────
void intakeTask(void *) {
  static bool wasOutOfPosition = false; // Track previous position state

  while (true) {
    double catPos = catapult_arm.get_position();
    double returnThreshold = 15.0; // Position tolerance for "returned to home"

    // Check if catapult_arm is NOT at starting position (LOAD_POS)
    bool isOutOfPosition = std::abs(catPos - LOAD_POS) > returnThreshold;

    if (isOutOfPosition) {
      // Catapult arm is away from starting position: RUN OUTTAKE
      // Only outtake if catapult is actively firing (not already handled by
      // catapultTask)
      if (catState == CAT_FIRING || catState == CAT_RELOADING) {
        // Let catapultTask handle the intake during firing
      } else {
        // If manually triggered outtake is needed, uncomment below:
        // outtakeBlock(600);
      }
      wasOutOfPosition = true;
    } else {
      // Catapult arm has RETURNED to starting position: RUN INTAKE
      if (wasOutOfPosition) {
        // Just returned to home: start intake
        if (catState == CAT_IDLE) {
          intakeBlock(); // Run intake (velocity -600)
          wasOutOfPosition = false;
        }
      }
    }

    pros::delay(20); // Check position every 20ms
  }
}

// ─── Operator control ────────────────────────────────────────────────────────
void catapultControl() {
  descoreHoming();
  leverReset();
  const int MAX_SPEED = 127;
  const int SLOW_SPEED = 50;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;

  static bool controlsReversed = false;
  static bool wasDownHeld = false;
  static bool armRaised = false; // "Resting" state of the arm (0 or 2000)
  static double targetHeading = 0.0;
  static bool headingLocked = false;

  // --- ARM STATE TRACKING ---
  static bool wasArmHeld = false;
  static uint32_t pressStartTime = 0;
  static double lastDiscorePos = 0.0;
  static ArmState lastKnownState = LONG_GOAL;
  static bool stateChanged = true; // Force first execution
  static bool lastCatBtn = false;

  static bool matchLoadToggled = false;
  static bool wasMatchLoadTapped = false;
  static bool hasMatchloadHomed =
      false; // Flag to track if first homing has occurred

  while (true) {
    // 1. Gate Logic Base
    // Only apply if we haven't just changed states or if we are idle
    if (catState == CAT_IDLE && !pros::competition::is_autonomous()) {
      if (currentArmState == LONG_GOAL) {
        gateClose();
      } else if (currentArmState == MID_GOAL) {
        midGoalArm();
      }
      // Note: UNDER_GOAL gate position is handled in underGoalArm() once
    }

    pros::delay(20);
    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool catapultBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool descoreHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool matchLoadTapped = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // Toggle logic for Matchload (Button Y)
    if (matchLoadTapped && !wasMatchLoadTapped) {
      // Trigger homing sequence only on the very first tap of the button
      if (!hasMatchloadHomed) {
        matchloadHoming();
        hasMatchloadHomed = true;
      } else {
        matchLoadToggled = !matchLoadToggled;
      }
    }
    wasMatchLoadTapped = matchLoadTapped;

    // Arm logic state tracking
    bool armHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool downTapped = downHeld && !wasDownHeld;
    wasDownHeld = downHeld;

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (downTapped)
      controlsReversed = !controlsReversed;
    if (controlsReversed)
      move = -move;

    if (std::abs(move) > INU_CORRECTION_MIN_MOVE &&
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

    if (catapultBtn) {
      startCatapultShoot(true, false);
    }

    // Logic to detect Tap vs. Hold
    if (armHeld && !wasArmHeld) {
      pressStartTime = pros::millis();
    }

    // IMMEDIATE TRANSITION TO UNDER_GOAL IF HELD
    if (armHeld && (pros::millis() - pressStartTime > 200) &&
        currentArmState != UNDER_GOAL) {
      currentArmState = UNDER_GOAL;
      stateChanged = true;
    }

    // ─── STATE TRANSITION LOGIC (Taps) ───
    if (wasArmHeld && !armHeld) {
      // Button was just released - check how long it was held
      uint32_t holdTime = pros::millis() - pressStartTime;

      if (holdTime <= 500) {
        // SHORT TAP: Cycle between Long and Mid
        if (currentArmState == LONG_GOAL) {
          currentArmState = MID_GOAL;
        } else {
          currentArmState = LONG_GOAL;
        }
        stateChanged = true;
      }
    }

    // Update wasArmHeld at the very end of the loop or after checking taps
    wasArmHeld = armHeld;

    // 5. Arm & Descore execution
    // Manual Descore Control (Only if Button B is actually pressed)
    if (descoreHeld) {
      descoreDown();
    } else {
      // If B is released, check if we need to return to a state-defined
      // position
      if (stateChanged) {
        // Apply the state-specific defaults ONLY when state changes to avoid
        // constant fighting
        switch (currentArmState) {
        case LONG_GOAL:
          longGoalArm();
          break;
        case MID_GOAL:
          midGoalArm();
          break;
        case UNDER_GOAL:
          underGoalArm();
          break;
        }
        stateChanged = false;
      } else if (currentArmState == LONG_GOAL) {
        // In the normal LONG_GOAL state, release = return to UP position
        descoreUp();
      }
    }

    // Toggle matchload based on matchLoadToggled state (Button Y)
    if (matchLoadToggled) {
      matchloadUp();
    } else {
      matchloadDown();
    }

    // Manual intake controls
    if (catState == CAT_IDLE) {
      if (intakePause) {
        intake.move_velocity(0);
      } else if (intakeForward && !intakeReverse) {
        outtakeBlock(100);
      } else if (intakeReverse && !intakeForward) {
        intakeBlock();
      }
    }
  }
}