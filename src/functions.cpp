#include "claw_auto.hpp"
#include "distance_diag.hpp"
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

static void driveSide(pros::MotorGroup &side, int power) {
  if (power != 0) {
    side.move(power);
  } else {
    side.brake(); // applies the brake mode, which is COAST here: the wheels spin freely
  }
}

void driveByHand(int left, int right) {
  left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  driveSide(left_motor_group, left);
  driveSide(right_motor_group, right);
}

// ─── Lift homing ─────────────────────────────────────────────────────────────
static bool liftHomed = false;

bool liftIsHomed() { return liftHomed; }

bool updateLiftLimitSwitch() {
  static pros::Mutex switchMtx; // the watcher task and the drive loops both get here
  static bool zeroedOnPress = false;
  // The lift hits the switch while moving, then bounces or sinks a little. So the zero set on contact is
  // only a first one: it is set again once the lift is still.
  const double STILL_BAND_DEG = 0.3;  // moving less than this...
  const uint32_t STILL_MS = 150;      // ...for this long = at rest
  const uint32_t BOUNCE_WAIT_MS = 1000; // how long after a hit a rest off the switch still counts
  const double BOUNCE_MAX_DEG = 20.0; // further from zero than this is not a bounce
  static uint32_t hitMs = 0;          // when the switch was last hit; 0 = no bounce being followed
  static double stillPos = 0;
  static uint32_t stillSinceMs = 0;

  switchMtx.take();
  const auto state = lift_limit_switch.get_value();
  const uint32_t now = pros::millis();
  if (state == 0) zeroedOnPress = false;
  if (state == 1 && !zeroedOnPress) {
    liftHomed = lift.tare_position_all() == 1;
    zeroedOnPress = liftHomed;
    if (zeroedOnPress) {
      hitMs = now;
      stillPos = 0;
      stillSinceMs = now;
    }
  }

  const double pos = liftPositionDeg();
  if (std::fabs(pos - stillPos) > STILL_BAND_DEG) {
    stillPos = pos;
    stillSinceMs = now;
  }
  const bool still = now - stillSinceMs >= STILL_MS;
  // A lift driven well away, or one that has taken too long, is no longer bouncing.
  if (hitMs && (std::fabs(pos) > BOUNCE_MAX_DEG || now - hitMs > BOUNCE_WAIT_MS)) hitMs = 0;
  if (still && (state == 1 || hitMs) && std::fabs(pos) <= BOUNCE_MAX_DEG) {
    if (std::fabs(pos) > 0.5 && lift.tare_position_all() == 1) {
      stillPos = 0;
      stillSinceMs = now;
    }
    hitMs = 0; // at rest and zeroed: this hit is dealt with
  }
  switchMtx.give();
  return state == 1;
}

void startLiftLimitSwitchWatcher() {
  static pros::Task watcher([] {
    while (true) {
      updateLiftLimitSwitch();
      pros::delay(5);
    }
  });
}

// The lift's angle in degrees: the average of the motors that answer. (lift.get_position() reads only the
// first motor, so one motor with a little backlash against the other made the lift look a few degrees off.)
double liftPositionDeg() {
  double sum = 0;
  int n = 0;
  for (double p : lift.get_position_all())
    if (std::isfinite(p) && std::fabs(p) < 1e6) {
      sum += p;
      n++;
    }
  return n ? sum / n : lift.get_position();
}

bool liftAtFloor() {
  const bool pressed = updateLiftLimitSwitch(); // first, so a new press zeroes the angle
  return pressed || liftPositionDeg() <= LIFT_FLOOR_DEG + LIFT_STOP_MARGIN_DEG;
}

void stopLift() {
  if (liftPositionDeg() < LIFT_FLOOR_DEG - 1.0) {
    lift.move_absolute(LIFT_FLOOR_DEG, 80);
  } else {
    lift.move_velocity(0);
  }
}

int liftDownRpm(int requested) {
  const double RPM_PER_DEG = 1.5; // full speed (200 rpm) only above about 133 degrees
  const int CREEP_RPM = 10;       // slowest approach, so it still gets there
  if (liftAtFloor()) return 0;
  const int allowed = std::max(CREEP_RPM, static_cast<int>(liftPositionDeg() * RPM_PER_DEG));
  return std::min(requested, allowed);
}

static constexpr double CLAW_PICKUP_MAX_DEG = 20.0;

bool liftInClawPickupRange() {
  if (!liftHomed) return false;
  const double angle = liftPositionDeg();
  return angle >= LIFT_FLOOR_DEG && angle <= CLAW_PICKUP_MAX_DEG;
}

void homeLift() {
  if (updateLiftLimitSwitch()) {
    lift.move_velocity(0);
    return;
  }
  if (liftHomed) return;
  const int HOME_RPM = 100;
  const uint32_t TIMEOUT_MS = 4000;
  controller.print(1, 0, "Homing lift...  ");
  lift.move_velocity(-HOME_RPM);
  const uint32_t start = pros::millis();
  while (!updateLiftLimitSwitch() && pros::millis() - start < TIMEOUT_MS) {
    pros::delay(10);
  }
  lift.move_velocity(0);
  // The switch set 0 while the lift was still moving down: bring back whatever it travelled past that.
  if (liftHomed) lift.move_absolute(LIFT_FLOOR_DEG, 60);
  controller.rumble(liftHomed ? "." : "--");
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
  // DOWN toggles an inverted drivetrain: what was forward becomes backward. Starts normal every time.
  bool driveInverted = false, wasInvertBtn = false;
  uint32_t lastInvertMs = 0;
  // Tapping B (the claw) also raises the lift, so it clears whatever the claw just took.
  const double CLAW_TAP_LIFT_DEG = 280;
  const uint32_t CLAW_TAP_LIFT_TIMEOUT_MS = 1500; // give up if the lift cannot get there
  bool nudgeActive = false;
  double nudgeTarget = 0.0;
  uint32_t nudgeUntilMs = 0;
  // Auto-grab closes the claw and raises the lift when the sensor detects an object in pickup range.
  const bool CLAW_AUTO_GRAB = true;
  ClawAutoGrab autoGrab;
  int autoGrabMm = -1; // the last calibrated sensor reading, shown on the controller
  // Raises the lift by CLAW_TAP_LIFT_DEG from its current position.
  auto raiseLift = [&](uint32_t now) {
    nudgeTarget = std::max(0.0, liftPositionDeg()) + CLAW_TAP_LIFT_DEG; // from a negative reading it is counted from 0
    nudgeUntilMs = now + CLAW_TAP_LIFT_TIMEOUT_MS;
    nudgeActive = true;
    liftTuningCancelTarget();
    lift.move_absolute(nudgeTarget, 200);
  };

  // Applies the lift tuner's START POSITION (if enabled) before the driver takes over.
  liftTuningBeginDriver();
  distanceStartReader(); // the claw auto-grab reads the distance sensor
  homeLift();

  while (true) {
    bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);   // L1 = intake
    bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);  // L2 = outtake
    uint32_t nowMs = pros::millis();

    bool liftForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);   // R1 = lift forward
    bool liftBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);  // R2 = lift backward
    const bool liftInPickupRange = liftInClawPickupRange();

    // Show lift position on the controller while tuning LIFT_FORWARD_LIMIT.
    // The controller screen only updates about every 50ms, so print every
    // 100ms. Row 1 keeps clear of fieldctl's status on row 0.
    static uint32_t lastLiftPrintMs = 0;
    static bool printDriveDiag = false;
    if (nowMs - lastLiftPrintMs >= 100) {
      lastLiftPrintMs = nowMs;
      if (!printDriveDiag) {
        controller.print(1, 0, "Lift:%6.0f %-5s", liftPositionDeg(), liftTuningArmed() ? "ARMED" : "");
      } else {
        // Row 2: which way the drivetrain is facing (DOWN flips it).
        // and the claw sensor: D = the calibrated reading in mm (-1 no target, -2 sensor dead), GRAB = armed.
        // Every condition the auto-grab needs, so a grab that does not fire can be explained at a glance:
        //   D = reading in mm, A = armed, Z = lift in pickup range, C = claw extended, B = busy.
        controller.print(2, 0, "%c D%-3d A%dZ%dC%dB%d", driveInverted ? 'R' : 'f', autoGrabMm, autoGrab.armed ? 1 : 0,
                         liftInPickupRange ? 1 : 0, clawPistonExtended() ? 1 : 0,
                         (nudgeActive || liftTuningManualLiftBlocked()) ? 1 : 0);
      }
      printDriveDiag = !printDriveDiag;
    }

    // B toggles the claw piston, RIGHT toggles the lift-arm orientation (edge-detected + debounced).
    bool clawPistonHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (clawPistonHeld && !wasClawPistonBtn && nowMs - lastClawPistonMs >= BUTTON_DEBOUNCE_MS) {
      lastClawPistonMs = nowMs;
      toggleClawPiston();
      if (liftPositionDeg() <= CLAW_PICKUP_MAX_DEG) raiseLift(nowMs);
    }
    wasClawPistonBtn = clawPistonHeld;
    if (CLAW_AUTO_GRAB) {
      bool busy = nudgeActive || liftTuningManualLiftBlocked();
      autoGrabMm = distanceReadRawMm(0); // raw: the filtered value lags
      if (autoGrab.update(autoGrabMm, liftInPickupRange, clawPistonExtended(), busy, nowMs)) {
        setClawPiston(true);
        raiseLift(nowMs);
        controller.rumble(".");
      }
    }
    bool orientHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (orientHeld && !wasOrientBtn && nowMs - lastOrientMs >= BUTTON_DEBOUNCE_MS) {
      lastOrientMs = nowMs;
      toggleLiftArmOrientation();
    }
    wasOrientBtn = orientHeld;
    bool invertHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (invertHeld && !wasInvertBtn && nowMs - lastInvertMs >= BUTTON_DEBOUNCE_MS) {
      lastInvertMs = nowMs;
      driveInverted = !driveInverted;
      controller.rumble(driveInverted ? "-" : "."); // long = reversed, short = back to normal
    }
    wasInvertBtn = invertHeld;

    auto stick = [](pros::controller_analog_e_t axis) {
      const int value = controller.get_analog(axis);
      return std::abs(value) <= DRIVE_STICK_DEADBAND ? 0 : value;
    };
    int move = stick(pros::E_CONTROLLER_ANALOG_LEFT_Y) * (driveInverted ? -1 : 1);
    int turn = stick(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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
      // Inverted: the sides swap and flip, so the robot drives the other way round.
      int leftY = stick(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightY = stick(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      driveByHand(driveInverted ? -rightY : leftY, driveInverted ? -leftY : rightY);
    } else {
      driveByHand(std::clamp(move + turn, -MAX_SPEED, MAX_SPEED), std::clamp(move - turn, -MAX_SPEED, MAX_SPEED));
    }

    // R1 raises the lift. R2 lowers it as far as 0 (or the bottom switch), never below.
    // Taking over by hand cancels a drive-to-level; with neither held, a level target keeps holding.
    // While an angle combo that moves the lift is running (or R1/R2 are still held after it), the
    // buttons are ignored; liftTuningStep() below drives the lift for the combo.
    if (liftTuningManualLiftBlocked()) {
      nudgeActive = false;
      lift.move_velocity(0);
    } else if (liftForward) {
      nudgeActive = false; // taking over by hand cancels the claw-tap lift
      liftTuningCancelTarget();
      lift.move_velocity(200);
    } else if (liftBackward) {
      nudgeActive = false;
      liftTuningCancelTarget();
      const int downRpm = liftDownRpm(200);
      if (downRpm > 0) {
        lift.move_velocity(-downRpm);
      } else {
        stopLift();
      }
    } else if (nudgeActive) {
      // move_absolute() is already running; stop (and hold, brake HOLD) once it arrives or times out.
      if (std::fabs(liftPositionDeg() - nudgeTarget) < 3.0 || (int32_t)(nowMs - nudgeUntilMs) >= 0) {
        nudgeActive = false;
        stopLift();
      }
    } else if (!liftTuningTargetActive()) {
      stopLift();
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
