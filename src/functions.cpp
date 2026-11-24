#include "functions.hpp"
#include "lift_tuning.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

// Spins a single fresh, non-reversed handle on rawPort for a moment and
// reports which way it physically turned, alongside how it's currently
// configured (reversed or not) - shared by both the MotorGroup and
// individual-motor variants below.
static void printMotorDirectionLine(const char *name, std::uint8_t rawPort,
                                     bool configuredReversed, int &y) {
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
  const char *configuredAs = configuredReversed ? "-" : "+";

  pros::screen::print(pros::E_TEXT_SMALL, 10, y,
                       "%s (port %2d): spins %s | configured %s", name,
                       (int)rawPort, spinDirection, configuredAs);
  y += 16;
}

static void testMotorGroupDirections(const char *label,
                                      pros::MotorGroup &group, int &y) {
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, y, "%s", label);
  y += 20;

  std::vector<std::int8_t> ports = group.get_port_all();

  for (int i = 0; i < group.size(); i++) {
    std::int8_t configuredPort = ports[i]; // negative = reversed in code
    std::uint8_t rawPort = std::abs(configuredPort);
    char name[8];
    snprintf(name, sizeof(name), "#%d", i + 1);
    // Fresh, non-reversed handle on the raw port so the result reflects the
    // motor's actual wiring, not the sign already applied by the group.
    printMotorDirectionLine(name, rawPort, configuredPort < 0, y);
  }

  y += 10;
}

// Same idea as testMotorGroupDirections(), but for a labeled group of
// standalone pros::Motor instances (intake2, arm, etc.)
// rather than a MotorGroup - each entry is {display name, motor}.
static void
testMotorsDirections(const char *label,
                      std::vector<std::pair<const char *, pros::Motor *>>
                          motors,
                      int &y) {
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, y, "%s", label);
  y += 20;

  for (auto &entry : motors) {
    const char *name = entry.first;
    pros::Motor *motor = entry.second;
    std::uint8_t rawPort = std::abs(motor->get_port());
    printMotorDirectionLine(name, rawPort, motor->is_reversed(), y);
  }

  y += 10;
}

// Runs the spin-direction test on every motor on the robot, grouped by
// subsystem, and reports the results on the brain screen. Run this on its
// own (e.g. as the selected autonomous routine) - do not run it at the same
// time as jawheadControl(), since both drive the motors.
void testAllMotorDirections() {
  pros::screen::erase();
  int y = 10;
  testMotorGroupDirections("LEFT DRIVE", left_motor_group, y);
  testMotorGroupDirections("RIGHT DRIVE", right_motor_group, y);
  testMotorGroupDirections("LIFT", lift, y);
  testMotorGroupDirections("INTAKE", intake2, y);
  testMotorsDirections("CLAW ARM", {{"Arm", &claw_arm}}, y);
}

// --- Pistons ---
static bool clawPistonOut = false;
static bool liftOrientationOut = false;

void setClawPiston(bool extended) {
  clawPistonOut = extended;
  if (extended) claw_piston.extend(); else claw_piston.retract();
}
void toggleClawPiston() { setClawPiston(!clawPistonOut); }
bool clawPistonExtended() { return clawPistonOut; }

void setLiftArmOrientation(bool extended) {
  liftOrientationOut = extended;
  if (extended) lift_arm_orientation.extend(); else lift_arm_orientation.retract();
}
void toggleLiftArmOrientation() { setLiftArmOrientation(!liftOrientationOut); }
bool liftArmOrientationExtended() { return liftOrientationOut; }

void drivetrainReset(){
  left_motor_group.tare_position();
  right_motor_group.tare_position();
}


// ─── Operator control ────────────────────────────────────────────────────────
void jawheadControl() {
  // false = arcade: left stick Y drives forward/back, right stick X turns left/right.
  // true = tank (left stick -> left side, right stick -> right side), for testing each side.
  const bool kTankDrive = false;
  const int MAX_SPEED = 127;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;

  // When true, the IMU corrects drivetrain drift during driver control.
  static bool imu_status = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;
  bool wasClawPistonBtn = false, wasOrientBtn = false;
  uint32_t lastClawPistonMs = 0, lastOrientMs = 0;

  // Applies the lift tuner's START POSITION (if enabled) before the driver takes over.
  liftTuningBeginDriver();

  while (true) {
    bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);   // L1 = intake
    bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);  // L2 = outtake
    uint32_t nowMs = pros::millis();

    bool liftForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);   // R1 = lift forward
    bool liftBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);  // R2 = lift backward

    // Show lift position on the controller while tuning LIFT_FORWARD_LIMIT.
    // The controller screen only updates about every 50ms, so print every
    // 100ms. Row 1 keeps clear of fieldctl's status on row 0.
    static uint32_t lastLiftPrintMs = 0;
    static bool printDriveDiag = false;
    if (nowMs - lastLiftPrintMs >= 100) {
      lastLiftPrintMs = nowMs;
      if (!printDriveDiag) {
        controller.print(1, 0, "Lift:%6.0f %-5s", lift.get_position(), liftTuningArmed() ? "ARMED" : "");
      } else {
        // TEMP drive diagnostic (row 2): Ly = left stick Y (what the code sends the left side),
        // Lv = average actual rpm of the left motors. Delete this block when the left side is fixed.
        std::vector<double> lv = left_motor_group.get_actual_velocity_all();
        double sum = 0;
        for (double v : lv) sum += v;
        controller.print(2, 0, "Ly%4d Lv%4.0f   ", controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                         lv.empty() ? 0.0 : sum / lv.size());
      }
      printDriveDiag = !printDriveDiag;
    }

    // B toggles the claw piston, RIGHT toggles the lift-arm orientation (edge-detected + debounced).
    bool clawPistonHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (clawPistonHeld && !wasClawPistonBtn && nowMs - lastClawPistonMs >= BUTTON_DEBOUNCE_MS) {
      lastClawPistonMs = nowMs;
      toggleClawPiston();
    }
    wasClawPistonBtn = clawPistonHeld;
    bool orientHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (orientHeld && !wasOrientBtn && nowMs - lastOrientMs >= BUTTON_DEBOUNCE_MS) {
      lastOrientMs = nowMs;
      toggleLiftArmOrientation();
    }
    wasOrientBtn = orientHeld;

    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (!kTankDrive && imu_status && std::abs(move) > INU_CORRECTION_MIN_MOVE &&
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

    if (kTankDrive) {
      // Tank drive for testing: left stick Y drives the left side, right stick Y the right side.
      left_motor_group.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      right_motor_group.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    } else {
      left_motor_group.move(std::clamp(move + turn, -MAX_SPEED, MAX_SPEED));
      right_motor_group.move(std::clamp(move - turn, -MAX_SPEED, MAX_SPEED));
    }

    // Forward (clockwise) stops once the lift reaches the forward limit (LIFT LIMIT in the lift
    // tuner). Backward (R2) is not limited, so the lift can always be brought back down. Taking
    // over by hand cancels a drive-to-level; with neither held, a level target keeps holding.
    // While an angle combo that moves the lift is running (or R1/R2 are still held after it), the
    // buttons are ignored; liftTuningStep() below drives the lift for the combo.
    if (liftTuningManualLiftBlocked()) {
      lift.move_velocity(0);
    } else if (liftForward && lift.get_position() < liftTuningForwardLimit()) {
      liftTuningCancelTarget();
      lift.move_velocity(200);
    } else if (liftBackward) {
      liftTuningCancelTarget();
      lift.move_velocity(-200);
    } else if (!liftTuningTargetActive()) {
      lift.move_velocity(0);
    }

    // Claw arm (port 19): hold X for forward, hold A for backward. Released = it holds (brake HOLD).
    // If it moves the wrong way, flip the sign of the port in main.cpp: pros::Motor claw_arm(-19).
    const int CLAW_ARM_SPEED = 100;  // rpm (green gearset max is 200)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      claw_arm.move_velocity(CLAW_ARM_SPEED);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      claw_arm.move_velocity(-CLAW_ARM_SPEED);
    } else {
      claw_arm.move_velocity(0);
    }

    if (intakeIn) {
      intake2.move_velocity(-600);
    } else if (intakeOut) {
      intake2.move_velocity(600);
    } else {
      intake2.move_velocity(0);
    }

    // Lift tuner motor functions (angle triggers + scoring combos) -- after the manual controls
    // above so a firing function wins for its duration.
    liftTuningStep();

    pros::delay(10);
  }
}