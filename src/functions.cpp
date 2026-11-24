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

// ─── Lift homing ─────────────────────────────────────────────────────────────
// The lift's angle is only known relative to where it was zeroed. homeLift() finds the real bottom:
// it runs the lift down until it stalls against the hard stop, then calls that angle 0. Driver control
// uses the same stall detection when R2 is held (see jawheadControl()).
static bool liftHomed = false;

// Fastest lift motor, in rpm. An unplugged motor answers with a huge number; those are ignored.
static double liftSpeedRpm() {
  double fastest = 0;
  for (double v : lift.get_actual_velocity_all())
    if (std::isfinite(v) && std::fabs(v) < 1000.0) fastest = std::max(fastest, std::fabs(v));
  return fastest;
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

// Waits (up to maxMs) until the lift has stopped moving: its angle stays within 0.3 degrees for 120 ms.
// A lift that has just been pushed into the hard stop springs back a few degrees when it is released, so it
// must be allowed to settle before that angle is called 0.
static void waitForLiftToSettle(uint32_t maxMs) {
  uint32_t t0 = pros::millis(), stillSince = t0;
  double last = liftPositionDeg();
  while (pros::millis() - t0 < maxMs) {
    pros::delay(20);
    double p = liftPositionDeg();
    if (std::fabs(p - last) > 0.3) {
      last = p;
      stillSince = pros::millis();
    } else if (pros::millis() - stillSince >= 120) {
      return;
    }
  }
}

// Lowers the lift until it stalls, then zeroes its angle. Runs once per program run (at the start of
// autonomous or driver control, whichever comes first), so driver control does not drop a lift that
// autonomous raised. Blocks for about half a second to a couple of seconds.
void homeLift() {
  if (liftHomed) return;
  const int HOME_RPM = 100;                     // gentle: it ends against the hard stop
  const uint32_t SPIN_UP_MS = 250;              // ignore the speed while it gets going
  const uint32_t STALL_MS = 120;                // below STALL_RPM for this long = stalled
  const double STALL_RPM = 5.0;
  const uint32_t TIMEOUT_MS = 4000;
  controller.print(1, 0, "Homing lift...  ");
  lift.move_velocity(-HOME_RPM);
  uint32_t start = pros::millis(), stalledSince = 0;
  bool stalled = false;
  pros::delay(SPIN_UP_MS);
  while (pros::millis() - start < TIMEOUT_MS) {
    if (liftSpeedRpm() < STALL_RPM) {
      if (!stalledSince) stalledSince = pros::millis();
      if (pros::millis() - stalledSince >= STALL_MS) {
        stalled = true;
        break;
      }
    } else {
      stalledSince = 0;
    }
    pros::delay(10);
  }
  lift.move_velocity(0);
  if (stalled) {
    waitForLiftToSettle(600); // let it spring back off the stop first, or it would read ~5 degrees right away
    lift.tare_position_all(); // the bottom is 0
    liftHomed = true;
    controller.rumble(".");
  } else {
    controller.rumble("--"); // never stalled: do not guess where zero is
  }
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
  const double LIFT_AT_ZERO_DEG = 3.0; // "the lift is at 0": at or BELOW this (a lift that reads negative still counts)
  // Claw auto-grab: something in the claw (the sensor reads 0 to 10 mm, after its -60 offset) closes the claw
  // and raises the lift, but only with the lift at 0. Set false to switch it off.
  const bool CLAW_AUTO_GRAB = true;
  ClawAutoGrab autoGrab;
  int autoGrabMm = -1; // the last calibrated sensor reading, shown on the controller
  // Holding R2 until the lift stops turning means it is on the hard stop: that angle becomes 0.
  const uint32_t FLOOR_STALL_GRACE_MS = 250; // ignore the speed while it gets going
  const uint32_t FLOOR_STALL_MS = 120;       // stopped this long = stalled
  const double FLOOR_STALL_RPM = 5.0;
  uint32_t lowerSinceMs = 0, stalledSinceMs = 0;
  // After a stall on the floor the lift springs back a little: it settles first, then that angle becomes 0.
  bool floorSettling = false;
  double settleLastPos = 0;
  uint32_t settleStillSinceMs = 0, settleDeadlineMs = 0;
  const double SETTLE_BAND_DEG = 0.3;
  const uint32_t SETTLE_STILL_MS = 120, SETTLE_MAX_MS = 600;
  auto settleStep = [&](uint32_t now) {
    lift.move_velocity(0);
    double p = liftPositionDeg();
    if (std::fabs(p - settleLastPos) > SETTLE_BAND_DEG) {
      settleLastPos = p;
      settleStillSinceMs = now;
    }
    if (now - settleStillSinceMs >= SETTLE_STILL_MS || (int32_t)(now - settleDeadlineMs) >= 0) {
      lift.tare_position_all();
      floorSettling = false;
    }
  };
  // Raises the lift by CLAW_TAP_LIFT_DEG (from wherever it is; callers only do it at 0).
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
  // Then find the real bottom: the lift goes down until it stalls and that is 0 (this replaces the START
  // POSITION when the lift has not been homed yet this run).
  homeLift();

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
        controller.print(1, 0, "Lift:%6.0f %-5s", liftPositionDeg(), liftTuningArmed() ? "ARMED" : "");
      } else {
        // Row 2: which way the drivetrain is facing (DOWN flips it).
        // and the claw sensor: D = the calibrated reading in mm (-1 no target, -2 sensor dead), GRAB = armed.
        // Every condition the auto-grab needs, so a grab that does not fire can be explained at a glance:
        //   D = reading in mm (0..10 is in the claw)   A = armed   Z = lift at 0   C = claw extended (on)
        //   B = busy (lift moving / settling / locked). It fires when D is 0..10, A=1, Z=1, C=0 and B=0.
        controller.print(2, 0, "%c D%-3d A%dZ%dC%dB%d", driveInverted ? 'R' : 'f', autoGrabMm, autoGrab.armed ? 1 : 0,
                         liftPositionDeg() <= LIFT_AT_ZERO_DEG ? 1 : 0, clawPistonExtended() ? 1 : 0,
                         (nudgeActive || floorSettling || liftTuningManualLiftBlocked()) ? 1 : 0);
      }
      printDriveDiag = !printDriveDiag;
    }

    // B toggles the claw piston, RIGHT toggles the lift-arm orientation (edge-detected + debounced).
    bool clawPistonHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (clawPistonHeld && !wasClawPistonBtn && nowMs - lastClawPistonMs >= BUTTON_DEBOUNCE_MS) {
      lastClawPistonMs = nowMs;
      toggleClawPiston();
      // Raise the lift by CLAW_TAP_LIFT_DEG, but only from the bottom (angle 0): with the lift already up,
      // tapping the claw leaves it where it is.
      if (liftPositionDeg() <= LIFT_AT_ZERO_DEG) raiseLift(nowMs);
    }
    wasClawPistonBtn = clawPistonHeld;
    // Auto-grab: something in the claw while the lift is at 0 -> claw on, lift up (see ClawAutoGrab).
    if (CLAW_AUTO_GRAB) {
      bool atZero = liftPositionDeg() <= LIFT_AT_ZERO_DEG;
      bool busy = nudgeActive || floorSettling || liftTuningManualLiftBlocked();
      autoGrabMm = distanceReadMm(0);
      if (autoGrab.update(autoGrabMm, atZero, clawPistonExtended(), busy, nowMs)) {
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

    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * (driveInverted ? -1 : 1);
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
      // Inverted: the sides swap and flip, so the robot drives the other way round.
      int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      left_motor_group.move(driveInverted ? -rightY : leftY);
      right_motor_group.move(driveInverted ? -leftY : rightY);
    } else {
      left_motor_group.move(std::clamp(move + turn, -MAX_SPEED, MAX_SPEED));
      right_motor_group.move(std::clamp(move - turn, -MAX_SPEED, MAX_SPEED));
    }

    // R1 drives the lift forward (no top limit right now -- add your own check on lift.get_position()
    // here) and R2 backward. R2 stops at the floor: at angle 0 it does nothing, and if the lift stalls on
    // the hard stop while R2 is held, that angle is made 0.
    // Taking over by hand cancels a drive-to-level; with neither held, a level target keeps holding.
    // While an angle combo that moves the lift is running (or R1/R2 are still held after it), the
    // buttons are ignored; liftTuningStep() below drives the lift for the combo.
    if (!liftBackward) {
      lowerSinceMs = 0;
      stalledSinceMs = 0;
    }
    if (liftTuningManualLiftBlocked()) {
      nudgeActive = false;
      floorSettling = false;
      lift.move_velocity(0);
    } else if (liftForward) {
      nudgeActive = false; // taking over by hand cancels the claw-tap lift
      floorSettling = false;
      liftTuningCancelTarget();
      lift.move_velocity(200);
    } else if (floorSettling) {
      // just stalled on the floor: hold, let it spring back, then call where it rests 0 (R2 is ignored meanwhile)
      nudgeActive = false;
      settleStep(nowMs);
    } else if (liftBackward) {
      nudgeActive = false;
      liftTuningCancelTarget();
      double downPos = liftPositionDeg();
      if (downPos <= LIFT_FLOOR_DEG) {
        lift.move_velocity(0); // at the floor (0): no further down
      } else {
        // Slow down on the last 40 degrees so it cannot coast past 0.
        lift.move_velocity((int)std::clamp(downPos * 5.0, 60.0, 200.0) * -1);
        if (!lowerSinceMs) lowerSinceMs = nowMs;
        if (nowMs - lowerSinceMs >= FLOOR_STALL_GRACE_MS && liftSpeedRpm() < FLOOR_STALL_RPM) {
          if (!stalledSinceMs) stalledSinceMs = nowMs;
          if (nowMs - stalledSinceMs >= FLOOR_STALL_MS) { // on the hard stop: let it settle, then zero it
            lift.move_velocity(0);
            floorSettling = true;
            settleLastPos = liftPositionDeg();
            settleStillSinceMs = nowMs;
            settleDeadlineMs = nowMs + SETTLE_MAX_MS;
            stalledSinceMs = 0;
            lowerSinceMs = 0;
          }
        } else {
          stalledSinceMs = 0;
        }
      }
    } else if (nudgeActive) {
      // move_absolute() is already running; stop (and hold, brake HOLD) once it arrives or times out.
      if (std::fabs(liftPositionDeg() - nudgeTarget) < 3.0 || (int32_t)(nowMs - nudgeUntilMs) >= 0) {
        nudgeActive = false;
        lift.move_velocity(0);
      }
    } else if (!liftTuningTargetActive()) {
      lift.move_velocity(0);
    }
    // Safety net: 0 is the only limit. If the lift ever ends up below it (coasting, a combo, anything), bring it
    // back up to 0 and hold there. Not while R1 is pushing it up, nor while it settles on the floor.
    if (!liftForward && !floorSettling && !nudgeActive && liftPositionDeg() < LIFT_FLOOR_DEG - 1.0) {
      nudgeActive = false;
      lift.move_absolute(LIFT_FLOOR_DEG, 80);
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