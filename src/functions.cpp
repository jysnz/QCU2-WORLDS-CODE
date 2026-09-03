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
  static bool wasDownHeld = false;
  static bool wasL1Held = false;
  static bool armExtended = false;

  while (true) {
    bool l1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l1Tapped = l1Held && !wasL1Held;
    wasL1Held = l1Held;

    if (l1Tapped) {
      armExtended = !armExtended;
    }

    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool downTapped = downHeld && !wasDownHeld;
    wasDownHeld = downHeld;

    if (downTapped) {
      clamp.toggle();
    }

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

    if (armExtended) {
      arm.move_velocity(-100);
    } else {
      arm.move_velocity(0);
    }
  }
}