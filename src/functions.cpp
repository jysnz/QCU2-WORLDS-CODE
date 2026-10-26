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
// standalone pros::Motor instances (the claw, etc.)
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
  testMotorsDirections("CLAW", {{"claw", &claw}}, y);
}

void drivetrainReset(){
  left_motor_group.tare_position();
  right_motor_group.tare_position();
}

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
  // Upper lift travel limit in motor degrees from the bumper home position
  // (see homeLift()). The lower end is stopped by the bumper itself.
  const double LIFT_UPPER_LIMIT = 482.3;

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

    // The bumper is the home position: whenever it's pressed the lift angle is 0.
    if (liftBumper.get_value()) {
      lift.tare_position_all();
    }

    if (armLiftUp && lift.get_position() < LIFT_UPPER_LIMIT) {
      lift.move_velocity(100);
    } else if (armLiftDown && !liftBumper.get_value()) {
      lift.move_velocity(-100);
    } else {
      lift.move_velocity(0);
    }
  }
}