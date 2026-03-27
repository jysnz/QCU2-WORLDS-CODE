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

void drivetrainLock() {
  left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void gateOpen() { gate.move_absolute(-120, 200); }

void gateClose() { gate.move_absolute(-240, 200); }

void descoreUp() { discore.move_absolute(0, 200); }

void descoreDown() { discore.move_absolute(150, 200); }

void descoreDownMiddle() { discore.move_absolute(300, 200); }

void matchloadUp() { matchloader.move_absolute(-350, 200); }

void matchloadDown() { matchloader.move_absolute(0, 200); }

void midGoalArm() { arm.move_absolute(1300, 200); }

void underGoalArm() { arm.move_absolute(2200, 200); }

void longGoalArm() { arm.move_absolute(0, 200); }

void intakeBlock() { intake.move_velocity(-600); }

void outtakeBlock() { intake.move_velocity(600); }

void reset() { chassis.setPose(0, 0, 0); }

void leverReset() {
  catapult_arm.move_velocity(-50);
  int lv = catapult_arm.get_actual_velocity();

  if (lv <= 3) {
    catapult_arm.move_velocity(0);
    catapult_arm.tare_position();
  }
}

// ─── Catapult task (Updated with Dynamic Intake Logic) ───────────────────────
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
        intake.move_velocity(600); // Outtake
        pros::delay(350);

        // Firing success: Rotate positive to bring the next ball in
        intake.move_velocity(-600);

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

        // Stall case: Ensure intake stays negative to clear jam
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

  // Update state: Is the intake currently being used by the driver?
  intakeWasManual =
      (std::abs(intake.get_actual_velocity()) > INTAKE_STALL_THRESHOLD);

  // Start with intake positive to gather the ball
  // If catapult_arm stalls (blocked by a ball), catapultTask will switch to
  // negative
  intake.move_velocity(-600);

  // Command gate to open to -120 when firing begins
  gate.move_absolute(-120, 200);

  catAttempts = 0;
  stalledTime = 0;
  shotSuccess = false;
  catState = CAT_FIRING;
  catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);

  // For Autonomous: wait until the arm is back at home before returning control
  // Only if this is being called from a blocking context (like Autonomous)
  // Check if autonomous or competition state
  while (pros::competition::is_autonomous() && catState != CAT_IDLE) {
    pros::delay(10);
  }
}

// ─── Operator control ────────────────────────────────────────────────────────
void catapultControl() {
  const int MAX_SPEED = 127;
  const int SLOW_SPEED = 50;
  const double IMU_CORRECTION_KP = 0.6;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 2.0;

  static bool controlsReversed = false;
  static bool wasDownHeld = false;
  static bool armRaised = false; // "Resting" state of the arm (0 or 2000)
  static bool wasArmHeld = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;
  static double lastDiscorePos = 0.0;

  while (true) {
    // ONLY reset the gate to -220 if the catapult is not currently shooting.
    // This allows the gate.move_absolute(-100) in startCatapultShoot to
    // persist.
    if (catState == CAT_IDLE) {
      gateOpen();
    }

    pros::delay(20);
    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool catapultBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    // Arm logic state tracking
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
      startCatapultShoot();
    }

    // ─── ARM MOMENTARY HOLD LOGIC ───
    if (armTapped) {
      armRaised = !armRaised;
      lastDiscorePos = discore.get_position();
    }

    if (armHeld) {
      midGoalArm();

      // Move descore down until it stalls
      if (std::abs(discore.get_actual_velocity()) < 5 &&
          std::abs(discore.get_position()) > 200) {
        discore.move_velocity(0);
      } else {
        discore.move_velocity(100); // Adjust speed if needed
      }
    } else {
      arm.move_absolute(armRaised ? 1300 : 0, 200);
      if (wasArmHeld) {
        discore.move_absolute(lastDiscorePos, 200);
      }
    }

    if (discoreUp)
      descoreUp();
    else if (discoreDown)
      descoreDown();

    if (matchLoadDown && !matchLoadUp) {
      matchloadDown();
    } else if (matchLoadUp && !matchLoadDown) {
      matchloadUp();
    }

    // Manual intake controls
    if (catState == CAT_IDLE) {
      if (intakePause) {
        intake.move_velocity(0);
      } else if (intakeForward && !intakeReverse) {
        outtakeBlock();
      } else if (intakeReverse && !intakeForward) {
        intakeBlock();
      }
    }
  }
}