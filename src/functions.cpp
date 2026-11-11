#include "functions.hpp"
#include "lift_calibration.hpp"
#include "motors.hpp"
#include "run_log.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>


void homeLift() {
  const int HOME_VELOCITY = -100;       // negative = counter-clockwise (down)
  const uint32_t HOME_TIMEOUT_MS = 5000; // give up if the limit switch is never hit
  const uint32_t HOME_EXTRA_DOWN_MS = 150; // keep going down this long after the switch is tapped

  uint32_t startMs = pros::millis();
  liftHold(); // (a finished plan run leaves the lift relaxed)
  // First bring the lift to angle 0.0 (if it is above it and the switch isn't already pressed)...
  // (lift.get_position() averages the motors that answer, so one missing motor is fine; if none
  // answer it is infinite and this step is skipped -- the loop below still runs on the switch.)
  const double startPos = lift.get_position();
  if (!liftLimitSwitch.get_value() && std::isfinite(startPos) && startPos > 0.0) {
    lift.move_absolute(0.0, 100);
    while (!liftLimitSwitch.get_value() && lift.get_position() > 0.5 &&
           pros::millis() - startMs < HOME_TIMEOUT_MS) {
      pros::delay(10);
    }
  }
  // ...then keep going down until the limit switch is pressed: that spot is the real 0.
  lift.move_velocity(HOME_VELOCITY);
  while (!liftLimitSwitch.get_value() && pros::millis() - startMs < HOME_TIMEOUT_MS) {
    pros::delay(10);
  }
  // The switch was tapped: keep driving down a little further before stopping (only if it
  // really was the switch that ended the loop, not the timeout).
  if (liftLimitSwitch.get_value())
    pros::delay(HOME_EXTRA_DOWN_MS);
  lift.move_velocity(0); // brake mode HOLD keeps it in place
  lift.tare_position_all();
}

// ─── Operator control ────────────────────────────────────────────────────────
void jawheadControl() {
  liftHold(); // (a finished plan run leaves the lift relaxed)
  const int MAX_SPEED = 127;
  const double IMU_CORRECTION_KP = 0.8;
  const int INU_CORRECTION_MIN_MOVE = 15;
  const int IMU_CORRECTION_MAX_TURN = 5;
  const double IMU_CORRECTION_THRESHOLD = 1.0;

  // When true, the IMU corrects drivetrain drift during driver control.
  static bool imu_status = false;
  static double targetHeading = 0.0;
  static bool headingLocked = false;
  static bool wasAHeld = false;
  static bool clawsOn = true; // both claws are built extended (on) in main.cpp
  static uint32_t lastClawToggleMs = 0;
  const uint32_t BUTTON_DEBOUNCE_MS = 300;
  int seenPinGrabs = pinGrabCount();
  pinAutoGrabEnable(true); // IR sensor: a pin with the lift at start closes the claws, lifts to PIN

  while (true) {

    bool armLiftUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool armLiftDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  
    uint32_t nowMs = pros::millis();

    if (pinGrabCount() != seenPinGrabs) { // the sensor closed the claws: B now opens them
      seenPinGrabs = pinGrabCount();
      clawsOn = true;
    }
    if (armLiftUp || armLiftDown)
      pinGrabCancel(); // the driver takes the lift over

    // Tap DOWN to switch claw and claw2 on (extend) and off (retract) together.
    // Edge-detected (so holding it doesn't rapid-fire) and debounced so switch
    // bounce on the physical button can't register as two rapid taps that cancel out.
    // (DOWN no longer reverses the drivetrain.)
    bool clawToggleHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool clawToggleTapped = clawToggleHeld && !wasAHeld;
    wasAHeld = clawToggleHeld;
    if (clawToggleTapped && (nowMs - lastClawToggleMs >= BUTTON_DEBOUNCE_MS)) {
      lastClawToggleMs = nowMs;
      clawsOn = !clawsOn;
      if (clawsOn) {
        clawsClose(); // shuts both claws
      } else {
        claw.retract();
        claw2.retract();
      }
    }

    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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

    // The limit switch is the home position: whenever it's pressed the lift angle is 0.
    const bool limitDown = liftLimitPressed(); // debounced: 30 ms of steady contact
    if (limitDown) {
      lift.tare_position_all();
    }

    if (armLiftUp && lift.get_position() < LIFT_UPPER_LIMIT) {
      lift.move_velocity(100);
    } else if (armLiftDown && !limitDown) { // down until the switch is touched, whatever the angle reads
      lift.move_velocity(-100);
    } else if (!pinGrabLiftMoving()) { // (an auto-grab is carrying it up to the PIN level)
      lift.move_velocity(0);
    }
  }
}

// ─── Autonomous helpers ──────────────────────────────────────────────────────
// Default top speed for driveTo / turnTo (0-127).
static constexpr int kDefaultPathSpeed = 120;

void driveTo(float x, float y, bool forwards, int maxSpeed, int timeoutMs) {
    chassis.moveToPoint(x, y, timeoutMs, {.forwards = forwards, .maxSpeed = (float)(maxSpeed > 0 ? maxSpeed : kDefaultPathSpeed)});
    chassis.waitUntilDone();
}

namespace {
// A pin is driven to along a STRAIGHT LINE. LemLib's moveToPoint keeps steering at the target point,
// so any heading error left by the turn before it (or picked up on the way) becomes a curve, and
// the claw tip arrives off the pin. This drive instead fixes the line from where the robot is once
// it faces the pin to the pin itself, holds that heading with the IMU, and only leans on it a little
// (kLineKCross degrees per inch, at most kLineMaxCross) to come back onto the line if it is pushed
// off. It slows over the last decelRange inches, stops when the claw tip is on the pin (or the
// sensor has seen it), and creeps up to kSearchIn further if the sensor has not seen it by then.
struct StraightPinArgs {
    float px, py;      // the pin
    float reach;       // centre -> claw tip, inches
    int timeoutMs;     // for the drive itself (the turn has its own)
    float maxSpeed;    // 0-127
    float decelRange;  // inches
    float decelSpeed;  // 0-127
};
// Steering on the line. (Run 2026-11-11: with 1.5 deg/in and 1.5 power/deg a 3.6 deg facing error
// was still there 12 in later and the robot ended 1.2 in off the line -- 5 power of steering does
// not turn the robot, and 1.5 deg/in needs ~40 in to come back onto the line.)
// The heading is aimed at the pin's live bearing (so odometry drift or a slip on the way cannot
// leave the robot parallel to, but off, the line) until kAimFreezeIn from the stop, then held: the
// bearing swings wildly when the pin is a few inches away.
constexpr float kAimFreezeIn = 6.0f;      // inches before the stop where the aim is frozen
constexpr float kGentleIn = 3.0f;         // inches before the stop driven at kLineCreep (a pin is
                                          // pushed out of the claw by a fast arrival)
constexpr float kLineKHeading = 2.5f;     // power per degree of heading error
constexpr float kLineKRate = 0.04f;       // power per deg/s of turning: damps the steering
constexpr float kLineMaxTurn = 35.0f;     // power
constexpr float kLineDeadband = 0.3f;     // degrees: no steering for less than this
constexpr float kLineAccelStep = 5.0f;    // power gained per 10 ms (gentle: a hard launch slips a
                                          // wheel and yaws the robot off the line)
// Squaring up on the line before the drive. LemLib's turn PID gives a few degrees of error too
// little power to move the robot at all (it sat 3.6 deg off for a second), so the last degrees are
// done here by holding a slow turning SPEED, measured from the heading: the power is whatever it
// takes to turn at that speed, so it rises until the robot breaks loose and comes straight back
// off when it does. History, all from runs on 2026-11-11:
//  - power raised until it moved, dropped at the first movement: crept in jerks, too slow;
//  - ...kept while it moved: it takes more power to start this robot than to keep it turning, so
//    it jumped 3 deg at a time and swung 1.5 deg either side of the pin;
//  - the motors' own speed control (move_velocity): squared up to 0.2-0.7 deg, but the robot also
//    rolled 2 in BACKWARDS while doing it (both sides; not understood), so it is not used.
constexpr float kFaceDoneDeg = 0.6f;      // facing the pin to within this is good enough
constexpr float kFaceHoldDeg = 1.0f;      // ...and once there, it may rock back out to this
constexpr int kFaceSettleTicks = 8;       // ...held for this many 10 ms ticks
constexpr float kFaceRatePerDeg = 6.0f;   // deg/s of turning asked for per degree still to turn
constexpr float kFaceMinRate = 8.0f;      // deg/s: the slowest it is asked to turn
constexpr float kFaceMaxRate = 50.0f;
constexpr float kFaceStartPower = 12.0f;  // where the power starts (below what moves the robot)
constexpr float kFaceKi = 12.5f;          // power per second, per deg/s too slow
constexpr float kFaceKp = 0.5f;           // power per deg/s too slow / too fast, at once
constexpr float kFaceMaxPower = 60.0f;
constexpr float kFaceMovingDegS = 8.0f;   // LemLib's turn: slower than this counts as stalled
constexpr int kFaceTimeoutMs = 900;
constexpr float kFaceTakeOverDeg = 8.0f;  // LemLib's turn is cut short once it is this close...
constexpr int kFaceStallTicks = 6;        // ...and has not been turning for this many 10 ms ticks
constexpr float kLineCreep = 22.0f;       // power: the slowest it ever drives
constexpr float kLineTol = 0.4f;          // inches
constexpr float kBrakeLeadIn = 5.0f;    // inches before the slow zone where the braking starts
// How far past the nominal stop it looks for the pin. Was 2.0; in the runs of 2026-11-11 the pin
// was in the claw at the nominal stop but the sensor had not fired, and the 2 in of "looking"
// pushed the pin out of the claw before it closed.
constexpr float kSearchIn = 0.5f;
constexpr uint32_t kSeenStopMs = 120;     // from the sensor seeing the pin to standing still

float wrap180(float a) { return std::fmod(a + 540.0f, 360.0f) - 180.0f; }

void straightPinTask(void *p) {
    const StraightPinArgs a = *static_cast<StraightPinArgs *>(p);
    delete static_cast<StraightPinArgs *>(p);

    // 1. face the pin, and make sure it really is facing it before the drive starts
    // LemLib does the big turn. It stalls a few degrees short and then waits out its timeout (run
    // 2026-11-11: 5 deg off and not moving for 0.5 s), so once it is close and has stopped turning
    // it is cut short and the squaring-up below takes over.
    chassis.turnToPoint(a.px, a.py, 1000, {.forwards = true, .maxSpeed = (int)a.maxSpeed}, true);
    pros::delay(30); // the turn has begun: isInMotion() is true from here
    {
        float prevTheta = chassis.getPose().theta;
        int stalled = 0;
        while (chassis.isInMotion()) {
            pros::delay(10);
            const lemlib::Pose now = chassis.getPose();
            const float bearing = std::atan2(a.px - now.x, a.py - now.y) * 180.0f / (float)M_PI;
            const float rate = (now.theta - prevTheta) * 100.0f; // deg/s
            prevTheta = now.theta;
            const bool close = std::fabs(wrap180(bearing - now.theta)) < kFaceTakeOverDeg;
            stalled = close && std::fabs(rate) < kFaceMovingDegS ? stalled + 1 : 0;
            if (stalled >= kFaceStallTicks) {
                chassis.cancelMotion();
                break;
            }
        }
    }
    lemlib::Pose pose = chassis.getPose();
    float lineHeading = std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI;
    const float afterTurn = wrap180(lineHeading - pose.theta);
    if (std::fabs(afterTurn) > 10.0f) {
        runlog::event("pin line: turn did not finish (cancelled?), not driving");
        return; // far from facing the pin: most likely cancelled during the turn
    }

    // From here on it is one motion in LemLib's queue (so isInMotion() / cancelMotion() work on it):
    // square up on the line, then drive it.
    chassis.requestMotionStart();
    if (chassis.isInMotion() && std::fabs(afterTurn) > kFaceDoneDeg) {
        const uint32_t f0 = pros::millis();
        int settled = 0, side = 0;
        float prevTheta = pose.theta, rate = 0, held = 0;
        while (chassis.isInMotion() && (int)(pros::millis() - f0) < kFaceTimeoutMs) {
            pose = chassis.getPose();
            // (the bearing to the pin, not a fixed heading: turning on the spot shifts the centre)
            lineHeading = std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI;
            const float err = wrap180(lineHeading - pose.theta);
            rate = 0.5f * rate + 0.5f * (pose.theta - prevTheta) * 100.0f; // deg/s, smoothed
            prevTheta = pose.theta;
            float power = 0;
            // once inside, it stays "done" out to kFaceHoldDeg: the robot rocks back a few tenths of
            // a degree when the power comes off, and re-starting for that wasted 0.4 s per pin
            if (std::fabs(err) <= (settled > 0 ? kFaceHoldDeg : kFaceDoneDeg)) {
                side = 0;
                if (++settled >= kFaceSettleTicks)
                    break;
            } else {
                settled = 0;
                const int nowSide = err > 0 ? 1 : -1;
                if (nowSide != side) // starting, or it went past: begin gently from this side
                    held = kFaceStartPower * (float)nowSide;
                side = nowSide;
                const float want = (float)nowSide *
                                   std::clamp(kFaceRatePerDeg * std::fabs(err), kFaceMinRate, kFaceMaxRate);
                const float slow = want - rate; // + = not turning fast enough yet
                held = std::clamp(held + kFaceKi * slow * 0.01f, -kFaceMaxPower, kFaceMaxPower);
                power = std::clamp(held + kFaceKp * slow, -kFaceMaxPower, kFaceMaxPower);
            }
            left_motor_group.move((int)power);
            right_motor_group.move((int)-power);
            pros::delay(10);
        }
        left_motor_group.move(0);
        right_motor_group.move(0);
        pros::delay(40); // let it stop turning before the line is fixed
        pose = chassis.getPose();
    }
    if (!chassis.isInMotion()) {
        chassis.endMotion(); // cancelled: hand the queue on, like every way out below
        return;
    }

    // 2. the line: from here to the pin; the robot's centre stops `reach` short of it
    const float p0x = pose.x, p0y = pose.y;
    const float dist = std::hypot(a.px - p0x, a.py - p0y);
    if (dist < 1e-3f) {
        chassis.endMotion();
        return;
    }
    const float ux = (a.px - p0x) / dist, uy = (a.py - p0y) / dist;
    lineHeading = std::atan2(ux, uy) * 180.0f / (float)M_PI;
    // (an event holds 55 characters) "was" = how far off the turn left it, before squaring up
    runlog::event("pin line: brg %.1f dist %.1f face %.1f was %.1f", lineHeading, dist,
                  wrap180(lineHeading - pose.theta), afterTurn);
    float stopAt = dist - a.reach; // along the line
    if (stopAt <= kLineTol) {
        chassis.endMotion();
        return; // the claw tip is already on the pin
    }

    // 3. drive it
    const uint32_t t0 = pros::millis();
    float prevTheta = pose.theta;
    float prevMag = 0;
    float aimHeading = lineHeading;
    uint32_t seenAt = 0;
    bool searching = false;
    bool inDecel = false;
    while (chassis.isInMotion() && (int)(pros::millis() - t0) < a.timeoutMs) {
        pose = chassis.getPose();
        const float rx = pose.x - p0x, ry = pose.y - p0y;
        const float along = rx * ux + ry * uy;   // how far along the line
        const float remaining = stopAt - along;
        const bool seen = pinCloseWatchHit();    // the IR sensor saw the pin: claws are closing
        if (seen && seenAt == 0)
            seenAt = pros::millis();

        float mag;
        if (seen) {
            const uint32_t since = pros::millis() - seenAt;
            if (since >= kSeenStopMs)
                break;
            mag = prevMag * (1.0f - (float)since / (float)kSeenStopMs);
        } else if (remaining <= kLineTol) {
            if (searching)
                break;
            searching = true; // arrived without the sensor seeing it: look a little further on
            stopAt += kSearchIn;
            continue;
        } else {
            // full speed, brake over kBrakeLeadIn inches so it reaches decelSpeed as it enters the last
            // decelRange inches, and hold decelSpeed there (the sensor sees the pin in this stretch)
            float target = a.maxSpeed;
            if (a.decelRange > 0 && a.decelSpeed < a.maxSpeed) {
                if (remaining <= a.decelRange)
                    target = a.decelSpeed;
                else if (remaining < a.decelRange + kBrakeLeadIn)
                    target = a.decelSpeed + (a.maxSpeed - a.decelSpeed) *
                                                (remaining - a.decelRange) / kBrakeLeadIn;
                if (remaining <= a.decelRange && !inDecel) {
                    inDecel = true;
                    runlog::event("pin decel zone: along %.1f of %.1f, command %.0f", along, stopAt, prevMag);
                }
            }
            target = std::max(target, std::min(kLineCreep, a.decelSpeed));
            if (remaining <= kGentleIn)
                target = std::min(target, kLineCreep);
            if (searching)
                target = kLineCreep;
            mag = std::min(target, prevMag + kLineAccelStep);
        }
        prevMag = mag;

        // hold the line's heading, leaning a little towards the line when off it
        if (remaining > kAimFreezeIn)
            aimHeading = std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI;
        const float desired = aimHeading;
        float err = wrap180(desired - pose.theta);
        const float rate = (pose.theta - prevTheta) * 100.0f; // deg/s
        prevTheta = pose.theta;
        float turn = std::fabs(err) < kLineDeadband
                         ? 0.0f
                         : std::clamp(kLineKHeading * err - kLineKRate * rate, -kLineMaxTurn, kLineMaxTurn);
        float l = mag + turn, r = mag - turn;
        const float biggest = std::max(std::fabs(l), std::fabs(r));
        if (biggest > 127.0f) {
            l *= 127.0f / biggest;
            r *= 127.0f / biggest;
        }
        left_motor_group.move((int)l);
        right_motor_group.move((int)r);
        pros::delay(10);
    }
    left_motor_group.move(0);
    right_motor_group.move(0);
    pose = chassis.getPose();
    {
        const float rx = pose.x - p0x, ry = pose.y - p0y;
        runlog::event("pin line end: along %.1f of %.1f off line %.2f heading %.1f seen %d",
                      rx * ux + ry * uy, stopAt, -rx * uy + ry * ux, pose.theta,
                      pinCloseWatchHit() ? 1 : 0);
    }
    chassis.endMotion();
}
} // namespace

void goToPinDrive(float x, float y, int timeoutMs, int maxSpeed, bool forwards, float reach,
                  float decelRange, float decelSpeed) {
    const float speed = (float)(maxSpeed > 0 ? maxSpeed : kDefaultPathSpeed);
    const lemlib::Pose p = chassis.getPose();
    const float dx = x - p.x, dy = y - p.y;
    const float dist = std::hypot(dx, dy);
    if (forwards && reach > 0) {
        if (dist <= reach) {
            runlog::event("pin already within reach");
            return;  // the claw is already on the pin
        }
        // face the pin, then drive a straight line to it (see straightPinTask)
        auto *a = new StraightPinArgs{x, y, reach, timeoutMs, speed, decelRange, decelSpeed};
        pros::Task task(straightPinTask, a, "pin straight drive");
        pros::delay(20); // the turn has begun: isInMotion() is true from here
        return;
    }
    // (a backwards grab, or no reach given: the old way) full speed, then slowing to decelSpeed over
    // the last decelRange inches
    moveToPointDecel(x, y, timeoutMs, {.forwards = forwards, .maxSpeed = speed}, decelRange,
                     decelSpeed, true);
}

void goToPin(float x, float y, int level, int timeoutMs, int maxSpeed, bool forwards, float reach,
             float decelRange, float decelSpeed) {
    runlog::event("pin target x=%.2f y=%.2f reach=%.2f level=%d", x, y, reach, level);
    liftOpenClaws();
    // the IR sensor closes the claws the moment it sees the pin, even mid-drive or mid-turn
    pinCloseWatch(true);
    goToPinDrive(x, y, timeoutMs, maxSpeed, forwards, reach, decelRange, decelSpeed);
    pros::delay(10);
    // wait for the motion; it counts as finished only after it stays idle (queued moves have gaps)
    int idleMs = 0;
    // (keep this long: between the cruise stage and the decel approach the chassis is briefly idle,
    // and returning in that gap lets the next motion cancel the approach at the decel-start point)
    while (idleMs < 150) {
        idleMs = chassis.isInMotion() ? 0 : idleMs + 10;
        pros::delay(10);
    }
    runlog::event("pin drive ended seen=%d", pinCloseWatchHit() ? 1 : 0);
    pinCloseWatch(false);
    clawsClose();
    runlog::event("pin claws closed");
    pros::delay(80);  // let the claws shut on the pin before it is lifted
    liftGoToLevel(level, true, 1500);  // the lift rises only after the pin is gathered
}

void relocalizeFromWall(int wall, float edgeIn, bool backwards, int timeoutMs, int power,
                        bool setHeading, float wallAt, float maxCorrectionIn) {
    constexpr float kFieldHalfIn = 72.0f;
    constexpr double kStallRpm = 8.0;     // both sides slower than this = against the wall
    constexpr uint32_t kSpinUpMs = 250;   // ignore the start: the motors are still speeding up
    constexpr uint32_t kStallMs = 120;    // ...and it has to stay stalled this long
    const int dir = backwards ? -1 : 1;
    power = std::clamp(power, 20, 127);

    // a motion still running would fight the push into the wall
    chassis.cancelMotion();
    pros::delay(20);
    // Where odometry says it is BEFORE the push. Pushing into a wall makes the wheels slip and the
    // robot twist a little, which smears the odometry along the wall and in heading; only the
    // coordinate across the wall is worth overwriting, so the rest is taken from here.
    const lemlib::Pose before = chassis.getPose();

    const uint32_t t0 = pros::millis();
    uint32_t stalledSince = 0;
    left_motor_group.move(dir * power);
    right_motor_group.move(dir * power);
    while (pros::millis() - t0 < (uint32_t)std::max(timeoutMs, 300)) {
        pros::delay(10);
        if (pros::millis() - t0 < kSpinUpMs)
            continue;
        auto avg = [](const std::vector<double> &v) {
            double s = 0;
            for (double x : v)
                s += std::fabs(x);
            return v.empty() ? 0.0 : s / v.size();
        };
        const bool slow = avg(left_motor_group.get_actual_velocity_all()) < kStallRpm &&
                          avg(right_motor_group.get_actual_velocity_all()) < kStallRpm;
        if (!slow) {
            stalledSince = 0;
        } else if (stalledSince == 0) {
            stalledSince = pros::millis();
        } else if (pros::millis() - stalledSince >= kStallMs) {
            break;
        }
    }
    left_motor_group.move(0);
    right_motor_group.move(0);
    pros::delay(150); // let it settle flat against the wall

    float x = before.x, y = before.y, theta = before.theta;
    // the wall's heading when the back (or the front) is against it
    float facing = 0; // the heading the robot has with its front pointing away from the wall
    switch (wall) {
    // (the surface is the field wall unless the caller says where it really is)
    case 0: y = (std::isnan(wallAt) ? -kFieldHalfIn : wallAt) + edgeIn; facing = 0;   break; // bottom: front +Y
    case 1: y = (std::isnan(wallAt) ? kFieldHalfIn : wallAt) - edgeIn;  facing = 180; break; // top: front -Y
    case 2: x = (std::isnan(wallAt) ? -kFieldHalfIn : wallAt) + edgeIn; facing = 90;  break; // left: front +X
    default: x = (std::isnan(wallAt) ? kFieldHalfIn : wallAt) - edgeIn; facing = 270; break; // right: front -X
    }
    // how far the wall moves the position: the coordinate across it, odometry's value against the wall's
    const float correction = (wall < 2) ? y - before.y : x - before.x;
    float newTheta = theta;
    if (setHeading)
        newTheta = std::fmod(facing + (backwards ? 0.0f : 180.0f), 360.0f); // front at the wall: turned round
    const float turn = std::fmod(newTheta - before.theta + 540.0f, 360.0f) - 180.0f;
    printf("RELOC wall=%d odometry says %.2f, wall says %.2f -> correction %+.2f in, heading %+.1f deg\n",
           wall, (wall < 2) ? before.y : before.x, (wall < 2) ? y : x, correction, turn);
    if (std::fabs(correction) > maxCorrectionIn) {
        // too big to be drift: leave odometry as it was (the position it had before the push)
        printf("RELOC ignored: more than %.1f in -- check edgeIn / wallAt\n", maxCorrectionIn);
        return;
    }
    if (setHeading && std::fabs(turn) > 10.0f)
        newTheta = theta; // the robot was not square on the wall: do not trust the wall's heading
    chassis.setPose(x, y, newTheta);
}

bool decelOwnFinish = true;

namespace {
// The last stretch of a decel move (see decelOwnFinish in functions.hpp): a motion of ours in
// LemLib's queue. It holds `cap` power, then eases onto the point: forward power = kApproachP *
// (distance left along the line), never below kCreep so it still arrives. It steers toward the
// point until kSteerCutoff inches out, then drives straight (odometry noise near the point would
// only make it wiggle). Stops within kFinishTol of the point or once it has passed it.
struct ApproachArgs {
    float tx, ty;
    int timeoutMs;
    float cap;         // 0-127, the decelSpeed
    bool forwards;
    bool chain;        // minSpeed != 0: hand over to the next motion at exitRange, don't stop
    float exitRange;
};
constexpr float kApproachP = 8.0f;      // power per inch left
constexpr float kCreep = 20.0f;         // power, the slowest it creeps in
constexpr float kFinishTol = 0.5f;      // inches
constexpr float kSteerCutoff = 4.0f;    // inches from the point where steering stops
constexpr float kSteerP = 2.0f;         // power per degree of heading error
constexpr float kSteerMax = 30.0f;
constexpr float kAccelStep = 20.0f;     // power gained per 10 ms (same as the chassis' slew)

void ownApproachTask(void *p) {
    const ApproachArgs a = *static_cast<ApproachArgs *>(p);
    delete static_cast<ApproachArgs *>(p);
    chassis.requestMotionStart(); // waits here until the stage before it has ended
    const uint32_t t0 = pros::millis();
    lemlib::Pose pose = chassis.getPose();
    float ux = a.tx - pose.x, uy = a.ty - pose.y;
    const float len = std::hypot(ux, uy);
    if (len > 1e-3f) {
        ux /= len;
        uy /= len;
    }
    const float dir = a.forwards ? 1.0f : -1.0f;
    const float cap = std::clamp(a.cap, 0.0f, 127.0f);
    float prevMag = cap; // the stage before hands over at about this power: don't restart from 0
    while (chassis.isInMotion() && (int)(pros::millis() - t0) < a.timeoutMs) {
        pose = chassis.getPose();
        const float dx = a.tx - pose.x, dy = a.ty - pose.y;
        const float dist = std::hypot(dx, dy);
        const float along = dx * ux + dy * uy; // < 0: it has gone past the point
        if (a.chain ? dist < a.exitRange : (dist < kFinishTol || along < kFinishTol))
            break;
        float mag = std::clamp(kApproachP * std::max(along, 0.0f), std::min(kCreep, cap), cap);
        if (mag > prevMag)
            mag = std::min(mag, prevMag + kAccelStep);
        prevMag = mag;
        float turn = 0;
        if (dist > kSteerCutoff) {
            float bearing = std::atan2(dx, dy) * 180.0f / (float)M_PI; // compass: 0 = +y, clockwise
            float err = bearing - (a.forwards ? pose.theta : pose.theta + 180.0f);
            err = std::fmod(err + 540.0f, 360.0f) - 180.0f;
            turn = std::clamp(kSteerP * err, -kSteerMax, kSteerMax);
        }
        const float lat = dir * mag;
        left_motor_group.move((int)std::clamp(lat + turn, -127.0f, 127.0f));
        right_motor_group.move((int)std::clamp(lat - turn, -127.0f, 127.0f));
        pros::delay(10);
    }
    if (!a.chain || !chassis.isInMotion()) { // done, timed out or cancelled: stop
        left_motor_group.move(0);
        right_motor_group.move(0);
    }
    chassis.endMotion();
}

// Queues the final stretch to (x, y) behind whatever is already queued. Like an async
// chassis.moveToPoint: returns after giving the task time to join the queue.
void queueOwnApproach(float x, float y, int timeoutMs, const lemlib::MoveToPointParams &params,
                      float cap) {
    auto *a = new ApproachArgs{x, y, timeoutMs, cap, params.forwards,
                               params.minSpeed != 0 && params.earlyExitRange > 0,
                               params.earlyExitRange};
    pros::Task task(ownApproachTask, a, "decel approach");
    pros::delay(10);
}
} // namespace

void moveToPointDecel(float x, float y, int timeoutMs, lemlib::MoveToPointParams params,
                      float decelRange, float decelSpeed, bool async) {
    lemlib::Pose pose = chassis.getPose();
    float dist = std::hypot(x - pose.x, y - pose.y);
    // too short to have a "far" part, or nothing to slow down to: just drive it
    if (decelRange <= 0 || decelSpeed >= params.maxSpeed || dist <= decelRange * 1.25f) {
        chassis.moveToPoint(x, y, timeoutMs, params, async);
        return;
    }
    // stage 1: full speed to the point `decelRange` short of the target, handed off without stopping
    float ux = (x - pose.x) / dist, uy = (y - pose.y) / dist;
    lemlib::MoveToPointParams cruise = params;
    cruise.minSpeed = std::max(params.minSpeed, decelSpeed);
    cruise.earlyExitRange = 2;
    chassis.moveToPoint(x - ux * decelRange, y - uy * decelRange, timeoutMs, cruise, true);
    // stage 2: queued behind it, the last stretch capped at decelSpeed
    lemlib::MoveToPointParams approach = params;
    approach.maxSpeed = decelSpeed;
    if (decelOwnFinish)
        queueOwnApproach(x, y, timeoutMs, approach, decelSpeed);
    else
        chassis.moveToPoint(x, y, timeoutMs, approach, true);
    if (!async)
        chassis.waitUntilDone();
}

void moveToPointSmooth(float x, float y, int timeoutMs, lemlib::MoveToPointParams params,
                       float decelRange, float decelSpeed, bool async, int steps) {
    lemlib::Pose pose = chassis.getPose();
    float dist = std::hypot(x - pose.x, y - pose.y);
    if (decelRange <= 0 || decelSpeed >= params.maxSpeed || dist <= decelRange * 1.25f) {
        chassis.moveToPoint(x, y, timeoutMs, params, async);
        return;
    }
    steps = std::clamp(steps, 2, 8);
    float ux = (x - pose.x) / dist, uy = (y - pose.y) / dist;

    // speed limit of ramp stage i (1..steps): near maxSpeed at first, decelSpeed on the last
    auto stageSpeed = [&](int i) {
        return decelSpeed + (params.maxSpeed - decelSpeed) * (float)(steps - i) / (float)steps;
    };

    // cruise: full speed to the point `decelRange` short of the target, handed on at stage 1's speed
    lemlib::MoveToPointParams cruise = params;
    cruise.minSpeed = std::max(params.minSpeed, stageSpeed(1));
    cruise.earlyExitRange = 2;
    chassis.moveToPoint(x - ux * decelRange, y - uy * decelRange, timeoutMs, cruise, true);

    // ramp: each stage ends a little closer, capped lower, and exits into the next one still moving
    for (int i = 1; i <= steps; i++) {
        float left = decelRange * (float)(steps - i) / (float)steps;   // inches still to go at its end
        lemlib::MoveToPointParams stage = params;
        stage.maxSpeed = stageSpeed(i);
        if (i < steps) {
            stage.minSpeed = stageSpeed(i + 1);
            stage.earlyExitRange = 1.5f;
        }                                                              // last stage keeps the caller's min / exit
        if (i == steps && decelOwnFinish)
            queueOwnApproach(x, y, timeoutMs, stage, stage.maxSpeed);
        else
            chassis.moveToPoint(x - ux * left, y - uy * left, timeoutMs, stage, true);
    }
    if (!async)
        chassis.waitUntilDone();
}

void turnTo(float heading, int maxSpeed, int timeoutMs) {
    chassis.turnToHeading(heading, timeoutMs, {.maxSpeed = maxSpeed > 0 ? maxSpeed : kDefaultPathSpeed});
    chassis.waitUntilDone();
}

// Drives to the intended final pose (x, y, heading) and holds it for the rest of the autonomous
// period: if the robot ends short or gets pushed, it drives and turns back. The target is passed in
// rather than read from the odometry, because the last move can time out or settle early and the
// measured pose would then lock in the error. Never returns; PROS stops the autonomous task when
// the period ends.
void holdEndPosition(float x, float y, float theta) {
    const lemlib::Pose home(x, y, theta);
    constexpr float kPosTol = 1.0f;  // inches
    constexpr float kAngTol = 2.0f;  // degrees

    while (true) {
        const lemlib::Pose p = chassis.getPose();
        const float dx = home.x - p.x, dy = home.y - p.y;
        const float dist = std::hypot(dx, dy);
        float headErr = std::fmod(home.theta - p.theta + 540.0f, 360.0f) - 180.0f; // -180..180

        if (std::isfinite(dist) && std::isfinite(headErr)) {
            if (dist > kPosTol) {
                // drive in reverse if the spot is behind the robot, so it doesn't spin around first
                const float toPoint = std::atan2(dx, dy) * 180.0f / (float)M_PI; // 0 = +Y
                const float rel = std::fmod(toPoint - p.theta + 540.0f, 360.0f) - 180.0f;
                chassis.moveToPoint(home.x, home.y, 1500, {.forwards = std::abs(rel) <= 90.0f});
                chassis.waitUntilDone();
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            } else if (std::abs(headErr) > kAngTol) {
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            }
        }
        pros::delay(20);
    }
}
