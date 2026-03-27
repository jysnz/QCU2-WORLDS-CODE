#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

// ─── Constants ───────────────────────────────────────────────────────────────
const float PI = 3.14159f;
const double wheelDiameter = 3.25; // inches
const double ticksPerRev = 360.0;  // motor degrees per revolution

// ─── Catapult state ───────────────────────────────────────────────────────────
enum CatapultState { CAT_IDLE, CAT_FIRING, CAT_RELOADING };
static CatapultState catState = CAT_IDLE;
static int catAttempts = 0;
static bool shotSuccess = false;
static int stalledTime = 0;
static bool intakeWasManual = false; // Tracks if intake was already running before shoot

const int LOAD_POS = 0;
const int FIRE_POS = -600;
const int CAT_SPEED = 200;
const int STALL_TIME = 250;
const int CHECK_DELAY = 10;
const int MAX_ATTEMPTS = 10;

// ─── Intake stall detection state ─────────────────────────────────────────────
static int intakeCurrentVelocity = 0; // Current direction: 1=forward, -1=reverse, 0=stopped
static int intakeStallTime = 0;       // Time motor has been stalled
static bool intakeAutoReverse = true; // Enable/disable auto-reverse on stall

const int INTAKE_STALL_THRESHOLD = 5; // Velocity threshold to detect stall
const int INTAKE_STALL_DELAY = 300;   // ms before reversing on stall
const int INTAKE_REVERSE_DURATION = 200; // ms to run in reverse after stall detect

// ─── Catapult outtake timing ──────────────────────────────────────────────────
static int catReloadTime = 0;         // Time spent in reload state
static bool catShouldOuttake = false; // Flag to trigger outtake after reload

// ─── Odometry helpers ─────────────────────────────────────────────────────────
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

    if (std::abs(currentVel) > INTAKE_STALL_THRESHOLD) {
      intakeCurrentVelocity = (currentVel > 0) ? 1 : -1;
      intakeStallTime = 0; 
    } else if (intakeCurrentVelocity != 0) {
      intakeStallTime += CHECK_DELAY;

      if (intakeStallTime >= INTAKE_STALL_DELAY && intakeAutoReverse) {
        int savedDirection = intakeCurrentVelocity;
        int reverseVelocity = -savedDirection * 600;
        int intakeMoveVel = reverseVelocity;
        intake.move_velocity(intakeMoveVel);

        uint32_t reverseStart = pros::millis();
        while (pros::millis() - reverseStart < INTAKE_REVERSE_DURATION) {
          if (intakeCurrentVelocity == 0) {
            intake.move_velocity(0);
            intakeStallTime = 0;
            break;
          }
          pros::delay(10);
        }

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
        
        // Firing success: Rotate positive to bring the next ball in
        intake.move_velocity(600);
        
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
        intake.move_velocity(-600);
        
        catReloadTime = 0;
      }
      break;

    case CAT_RELOADING:
      catReloadTime += CHECK_DELAY;
      
      // If the arm is returning after a stall, ensure it switches to positive 
      // rotation once it begins moving back toward home.
      if (intake.get_target_velocity() < 0 && std::abs(pos - LOAD_POS) > 50) {
          intake.move_velocity(600);
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
          intake.move_velocity(-600);
          
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
  intakeWasManual = (std::abs(intake.get_actual_velocity()) > INTAKE_STALL_THRESHOLD);
  
  // Force intake negative immediately to clear the ball
  intake.move_velocity(-600);

  // Command gate to open to -150 when firing begins
  gate.move_absolute(-120 , 200);
  
  catAttempts = 0;
  stalledTime = 0;
  shotSuccess = false;
  catState = CAT_FIRING;
  catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);
}

// ─── Utility functions ───────────────────────────────────────────────────────
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
    
    // Autonomous start: intake negative to clear
    intake.move_velocity(-600);
    catapult_arm.move_absolute(F_POS, SPEED);
    int st = 0;

    while (true) {
      pros::delay(CD);
      double pos = catapult_arm.get_position();
      double vel = std::abs(catapult_arm.get_actual_velocity());
      if (pos <= F_POS + 25) {
        success = true;
        // Intake positive on firing position reach
        intake.move_velocity(600);
        break;
      }
      if (vel < 5)
        st += CD;
      else
        st = 0;
      if (st >= ST) {
          break;
      }
    }

    // Move to Load position
    catapult_arm.move_absolute(L_POS, SPEED);
    intake.move_velocity(600);
    
    while (std::abs(catapult_arm.get_position() - L_POS) > 10)
      pros::delay(10);
    
    intake.move_velocity(0);

    if (success)
      return;
  }
  catapult_arm.move_velocity(0);
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

  while (true) {
    // ONLY reset the gate to -220 if the catapult is not currently shooting.
    // This allows the gate.move_absolute(-100) in startCatapultShoot to persist.
    if (catState == CAT_IDLE) {
      gate.move_absolute(-240, 200);
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

    if (std::abs(move) > INU_CORRECTION_MIN_MOVE && std::abs(turn) < IMU_CORRECTION_MAX_TURN) {
      if (!headingLocked) {
        targetHeading = imu.get_heading();
        headingLocked = true;
      }
      double currentHeading = imu.get_heading();
      double headingError = targetHeading - currentHeading;
      while (headingError > 180.0) headingError -= 360.0;
      while (headingError < -180.0) headingError += 360.0;

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

    if (catapultBtn){
      startCatapultShoot();
      intake.move_velocity(600);
    }

    // ─── ARM MOMENTARY HOLD LOGIC ───
    if (armTapped) {
      armRaised = !armRaised;
      discore.move_absolute(500, 200);
    }

    if (armHeld) {
      arm.move_absolute(2200, 200);
    } else {
      arm.move_absolute(armRaised ? 1300 : 0, 200);
    }

    if (discoreUp)
      discore.move_absolute(0, 200);
    else if (discoreDown)
      discore.move_absolute(500, 200);

    if (matchLoadDown && !matchLoadUp) {
      matchloader.move_absolute(0, 100);
      discore.move_absolute(500, 200);
    } else if (matchLoadUp && !matchLoadDown) {
      matchloader.move_absolute(-1400, 100);
      discore.move_absolute(0, 200);
    }

    // Manual intake controls
    if (catState == CAT_IDLE) {
        if (intakePause) {
          intake.move_velocity(0);
        } else if (intakeForward && !intakeReverse) {
          intake.move_velocity(600);
        } else if (intakeReverse && !intakeForward) {
          intake.move_velocity(-600);
        }
    }
  }
}