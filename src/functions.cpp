#include "functions.hpp"
#include "goals.hpp"
#include "lift_calibration.hpp"
#include "motors.hpp"
#include "run_log.hpp"
#include "vision.hpp"
#include "vision_calib.hpp"
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

// The vision assist's knobs (see functions.hpp). Slot 1 = YEL, the pin's
// captured signature; 0 turns the assist off and the drive steers by
// odometry only, exactly as before. pinVisionRangeIn is only the
// stop-and-square-up distance. pinVisionCenterPx is the x_middle reading
// that means "aimed at the pin": the sensor runs zero-centred, so the code's
// centre is 0 (the brain's Vision Utility screen shows the same point as
// ~158-160 of its 0..316 scale). If field logs show raw ~+158 when the robot
// is square to the pin, the zero-centre flag is not applying -- set this to
// 158 then.
int pinVisionSig = 1;
float pinVisionRangeIn = 5.0f;
float pinVisionCenterPx = 0.0f;

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
// off. With the camera on the pin (from however far out it sees it) it
// steers straight at the pin, braking over the last kVisionBrakeIn inches
// down to kVisionApproach -- the coast from full speed used to carry it past
// the stop point, too close for the camera to see the pin. pinVisionRangeIn
// (5 in) short of the pin it halts, squares up on it by camera (on fresh
// sightings only), then creeps in at LOW speed, driving on until the IR
// sensor closes the claws, pushing at most kRunInPushIn (2 in) past the
// nominal stop point -- and at most kRunInMs, so a missed pin can never
// stall the plan: after that the motion ends and the next one starts.
// (Without a camera the plan's own decel applies, and the creep-in runs
// from the nominal stop.)
struct StraightPinArgs {
    float px, py;      // the pin
    float reach;       // centre -> claw tip, inches
    int timeoutMs;     // for the drive itself (the turn has its own)
    float maxSpeed;    // 0-127
    float decelRange;  // inches
    float decelSpeed;  // 0-127
    bool straight;     // wallPin: face the pin, then drive straight through it (no camera, no re-aiming)
    bool useVision;    // false: the camera never steers or stops the drive
    bool useIR;        // false: the IR sensor is not used; the claws shut on a timed fast grab instead
};
// Steering on the line. (Run 2026-11-11: with 1.5 deg/in and 1.5 power/deg a 3.6 deg facing error
// was still there 12 in later and the robot ended 1.2 in off the line -- 5 power of steering does
// not turn the robot, and 1.5 deg/in needs ~40 in to come back onto the line.)
constexpr float kLineKCross = 4.0f;       // degrees of heading per inch off the line
constexpr float kLineMaxCross = 12.0f;    // ...never more than this
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
constexpr float kLineTol = 0.4f;          // inches: close enough to the stop point is on it
constexpr float kBrakeLeadIn = 5.0f;      // inches before the slow zone where the braking starts
constexpr uint32_t kSeenStopMs = 120;     // from the sensor seeing the pin to standing still
// Vision assist (pinVisionSig / pinVisionRangeIn in functions.hpp): the
// sensor's FOV is ~63 deg across its 316 px, so a pixel off-centre is about
// 0.2 deg of heading to the pin (x_middle_coord is 0 at the centre -- the
// sensor was made with E_VISION_ZERO_CENTER). While a sighting is fresh the
// vision correction replaces the odometry line-lean entirely; when the pin
// is centred in view the robot is aligned and keeps driving straight, and
// the IR sensor closes the claws on contact as always.
constexpr float kVisionDegPerPx = 0.20f;  // deg of heading per px off-centre
constexpr float kVisionMaxDeg = 15.0f;    // ...never more than this
constexpr int kVisionAlignedPx = 14;      // within this of centre = aligned
constexpr int kVisionMinWidth = 4;        // px: the smallest blob that counts
constexpr uint32_t kVisionFreshMs = 200;  // steer by a sighting this long after it
constexpr uint32_t kVisionReadMs = 30;    // read the sensor at most this often
// The stop-and-square-up: if the camera has LOST the pin by pinVisionRangeIn
// short of it, the drive halts there, turns in place until the pin is
// centred, then creeps in. With the pin in view the whole way there is no
// stop and no deceleration -- the vision steering has it lined up, and it
// starts the moment the camera sees the pin, however far out that is.
constexpr uint32_t kVisionAlignMs = 450;  // how long the square-up may take
constexpr float kVisionTurnP = 1.2f;      // power per px off-centre
constexpr float kVisionTurnMax = 50.0f;   // power
constexpr int kVisionTurnMin = 12;        // slow turns stall: never ask less
constexpr int kVisionSettleTicks = 8;     // 10 ms ticks centred before it is done
// Close in, the pin's blob is huge, the sensor sees it only intermittently
// and x_middle jitters frame to frame -- so sightings are smoothed with an
// EMA, and the square-up stops waiting after this long without one:
constexpr float kVisErrAlpha = 0.4f;      // EMA weight of a new sighting
constexpr uint32_t kVisionAlignFreshMs = 90; // align turns only on readings this new
constexpr float kVisionBearingTol = 2.0f; // deg: geometrically aligned on the bearing
// The approach brake: full speed would coast 2-3 in past the vision stop,
// and 1-3 in from the pin the camera cannot see it any more -- so it brakes
// over the last stretch and comes into the stop slowly enough to stop on it:
constexpr float kVisionBrakeIn = 14.0f;   // brake over the last this many inches
constexpr float kVisionApproach = 25.0f;  // ...down to this power at the vision stop (was 40: the robot
                                          // was still doing 20 in/s when the IR fired, ~150 ms before
                                          // it reached the pin -- too little time for the claws)
// The creep-in after the square-up: LOW speed, straight at the pin, until
// the IR sensor closes the claws -- never longer than this, so a miss can't
// stall the plan (the next motion starts either way).
// The speed the claw tip arrives at, the same for every pin whatever the call passes as decelSpeed:
// the claw timing (IR reaction, kPinClawLatencyMs) is only right for one arrival speed.
constexpr float kPinArrivalSpeed = 25.0f; // power
// Creep-in speed. Run 2026-11-11 18:08: from a standstill 25 did not move the robot (2.8 in in 2 s, the
// IR never fired and the claws shut only at the timeout), so it starts at this power at once, no ramp.
constexpr float kRunInSpeed = 30.0f;      // power
// Camera sanity gate: a blob only counts if it is where odometry says the pin should appear.
// (A pin is really 1-3 in from its coordinates, 10-15 deg at the 5 in stop: 12 deg rejected the real
// pin in both approaches of the 2026-11-11 run, so the camera was never used. The -20 / +22 deg
// pair that hunted came with sign flips, which kSquareMaxFlips catches.)
constexpr float kVisionGateDeg = 22.0f;   // a blob further than this from where odometry says the pin is
                                          // is another object
// The square-up may only turn this far from the bearing to the pin, and it gives up on the camera (and
// turns onto the odometry bearing instead) if the camera's error changes sign twice: it was hunting.
constexpr float kSquareMaxDeg = 20.0f;
// At the stop the robot stands still this long LOOKING with the camera before it decides anything:
// slow, look, correct the heading, then creep in until the IR sees the pin.
constexpr uint32_t kVisionLookMs = 300;
// The heading is corrected ON THE MOVE: the camera steers from this far before the stop (while the
// robot is braking), so it arrives at the stop already lined up. The stop-and-pivot is off.
constexpr float kVisionSteerIn = 14.0f;
// A FAR pin (further than this) is looked for with the camera before the drive starts: the robot, already
// facing the pin's odometry bearing, stands still, lets the camera find the yellow pin, turns onto it, and
// drives the line it is then pointing along. A pin nearer than this uses the existing approach unchanged.
constexpr float kFarPinIn = 24.0f;
constexpr bool kFarPreAim = false;        // (off: the camera now only works near the pin, see below)
// THE CAMERA WINDOW: the camera is only used within kVisionWindowIn of the pin (distance of the claw tip),
// and the robot is already at the slow kVisionApproach speed when it enters. It steers on the move down
// to kVisionFreezeIn, then the heading is frozen (the blob jitters right at the pin and made it wobble),
// and the robot rolls on at the slow speed until the IR closes the claws. No stop, no square-up.
// wallPin: it drives STRAIGHT until the IR sees a pin -- not stopping at the pin's coordinates -- and
// gives up (the claws shut and it ends) this far past them, or at the drive's timeout.
constexpr float kStraightOverrunIn = 6.0f;
constexpr float kVisionWindowIn = 7.0f;
constexpr float kVisionFreezeIn = 2.0f;
constexpr bool kStopForVision = false;
constexpr bool kVisionPivot = false;
constexpr uint32_t kVisionHoldMs = 600;   // a sighting this recent counts as "lined up" at the stop
constexpr int kSquareMaxFlips = 2;
constexpr uint32_t kRunInMs = 1000;       // give up grabbing after this long
constexpr float kRunInDeadband = 2.0f;    // degrees: the creep-in does not steer for less than this
constexpr float kRunInPushIn = 2.0f;      // ...or after pushing this far past the stop point

// Fast grab (kPinGrabByIR == false): the claws are shut when the claw tip is within
// speed * kPinClawLatencyMs + kGrabBiasIn of the pin, so they grip as it arrives.
constexpr float kGrabBiasIn = 0.0f;       // + shuts earlier, - later (fine tuning of the grab point)
constexpr float kSpeedSmooth = 0.6f;      // smoothing of the measured speed (higher = steadier, laggier)

float wrap180(float a) { return std::fmod(a + 540.0f, 360.0f) - 180.0f; }

void straightPinTask(void *p) {
    const StraightPinArgs a = *static_cast<StraightPinArgs *>(p);
    delete static_cast<StraightPinArgs *>(p);
    const bool byIR = kPinGrabByIR && a.useIR; // IR sensor closes the claws (else the timed fast grab)

    // wallPin (a.straight): turn to face the pin as usual, then drive straight through to it -- the camera
    // is not used (visionOk below), so nothing re-aims on the way; the IR closes the claws.
    auto aimAt = [&](float fx, float fy) {
        return std::atan2(a.px - fx, a.py - fy) * 180.0f / (float)M_PI;
    };
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
            const float bearing = aimAt(now.x, now.y);
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
    float lineHeading = aimAt(pose.x, pose.y);
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
            lineHeading = aimAt(pose.x, pose.y);
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
    float ux = (a.px - p0x) / dist, uy = (a.py - p0y) / dist;
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
    uint32_t t0 = pros::millis();
    float prevTheta = pose.theta;
    float prevMag = 0;
    uint32_t seenAt = 0;
    float prevAlong = 0, speedIn = 0; // in/s along the line
    bool fired = false;               // fast grab: the claws have been told to shut
    bool inDecel = false;
    bool visBraked = false; // logged the approach brake already?
    // vision assist state (fresh only inside the zone, so stale sightings
    // from elsewhere on the field can never steer)
    uint32_t lastVisReadMs = 0, lastVisSeenMs = 0;
    int lastVisErrPx = 0;
    float visErrSmooth = 0; // the EMA behind lastVisErrPx (close in the raw px jitters)
    bool visErrInit = false;
    bool loggedZone = false, loggedSeen = false;
    const bool visionOk = a.useVision && !a.straight && vision_sensor.get_object_count() != PROS_ERR; // (wall pin: no camera)
    bool visionStopped = false; // did the drive end at the vision stop?
    // every sighting of the pin goes through this: the raw px is folded into
    // an exponential average, then shifted by pinVisionCenterPx, so the
    // steering and the square-up act on a smooth "aimed at the pin = 0" error
    auto noteSighting = [&](const pros::vision_object_s_t &o, uint32_t nowMs) {
        visErrSmooth = visErrInit
                           ? kVisErrAlpha * (float)o.x_middle_coord + (1.0f - kVisErrAlpha) * visErrSmooth
                           : (float)o.x_middle_coord;
        visErrInit = true;
        lastVisErrPx = (int)std::lround(visErrSmooth - pinVisionCenterPx);
        lastVisSeenMs = nowMs;
    };
    bool loggedReject = false;
    // is this blob where odometry says the pin should be? (see kVisionGateDeg)
    auto plausible = [&](const pros::vision_object_s_t &o) {
        const lemlib::Pose now = chassis.getPose();
        const float dx = a.px - now.x, dy = a.py - now.y;
        const float bearingErr = wrap180(std::atan2(dx, dy) * 180.0f / (float)M_PI - now.theta);
        const float seenDeg = ((float)o.x_middle_coord - pinVisionCenterPx) * kVisionDegPerPx;
        const bool ok = std::fabs(seenDeg - bearingErr) <= kVisionGateDeg;
        if (!ok && !loggedReject) {
            loggedReject = true;
            runlog::event("pin vision: blob rejected, seen %.0f exp %.0f deg", seenDeg, bearingErr);
        }
        return ok;
    };
    // turn in place onto the pin until the camera has it centred. When the
    // camera cannot see it (too close, blob fills the frame) it falls back
    // to the geometry: the robot's ACTUAL heading versus the bearing to the
    // pin's position, turning onto that instead of sitting blind. Shared by
    // the mid-drive pivots and the final square-up.
    auto squareUp = [&]() {
        const uint32_t tA = pros::millis();
        int settled = 0;
        pose = chassis.getPose();
        const float bearing0 = std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI;
        bool cameraOff = false; // gave up on the camera for this square-up
        int lastSign = 0, flips = 0;
        while (chassis.isInMotion() && (int)(pros::millis() - tA) < kVisionAlignMs) {
            const uint32_t nowMs = pros::millis();
            if (pinCloseWatchHit())
                break; // the IR saw it while turning: claws are on, done
            if (nowMs - lastVisReadMs >= kVisionReadMs) {
                lastVisReadMs = nowMs;
                const pros::vision_object_s_t o = vision_sensor.get_by_sig(0, pinVisionSig);
                if (o.signature == (uint16_t)pinVisionSig && o.width >= kVisionMinWidth && plausible(o))
                    noteSighting(o, nowMs);
            }
            int power = 0;
            if (!cameraOff) {
                const float away = std::fabs(wrap180(chassis.getPose().theta - bearing0));
                if (away > kSquareMaxDeg || flips >= kSquareMaxFlips) {
                    cameraOff = true;
                    runlog::event("pin vision: camera dropped, %.0f deg off, %d flips", away, flips);
                }
            }
            if (!cameraOff && nowMs - lastVisSeenMs <= kVisionAlignFreshMs) {
                // only act on genuinely fresh readings: squaring up on a
                // stale error from the drive is what left the heading off
                if (std::abs(lastVisErrPx) <= kVisionAlignedPx) {
                    if (++settled >= kVisionSettleTicks)
                        break;
                } else {
                    settled = 0;
                    const int sign = lastVisErrPx > 0 ? 1 : -1;
                    if (lastSign != 0 && sign != lastSign)
                        flips++;
                    lastSign = sign;
                    power = (int)std::clamp(kVisionTurnP * (float)lastVisErrPx,
                                            -kVisionTurnMax, kVisionTurnMax);
                    if (std::abs(power) < kVisionTurnMin) // slow turns stall: push
                        power = lastVisErrPx > 0 ? kVisionTurnMin : -kVisionTurnMin;
                }
            } else if (!cameraOff && lastVisSeenMs != 0 && nowMs - lastVisSeenMs <= kVisionHoldMs) {
                // the camera steered it onto the pin on the way in and has lost it only now (too
                // close): the heading is already right -- do not turn it back to the odometry bearing
                if (++settled >= kVisionSettleTicks)
                    break;
            } else if (!cameraOff && nowMs - tA < kVisionLookMs) {
                // still looking: stand still until the camera has had its chance to see the pin
                settled = 0;
            } else {
                // no fresh sighting: turn onto the BEARING to the pin
                // (odometry knows where the pin is even when the camera
                // cannot see it). The camera still wins when it is fresh.
                pose = chassis.getPose();
                const float bearing = std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI;
                const float bErr = wrap180(bearing - pose.theta);
                if (std::fabs(bErr) <= kVisionBearingTol) {
                    if (++settled >= kVisionSettleTicks)
                        break;
                } else {
                    settled = 0;
                    power = (int)std::clamp(kLineKHeading * bErr, -kVisionTurnMax, kVisionTurnMax);
                    if (std::abs(power) < kVisionTurnMin)
                        power = bErr > 0 ? kVisionTurnMin : -kVisionTurnMin;
                }
            }
            left_motor_group.move(power);
            right_motor_group.move(-power);
            pros::delay(10);
        }
        left_motor_group.move(0);
        right_motor_group.move(0);
        if (lastVisSeenMs == 0 || pros::millis() - lastVisSeenMs > 1000)
            runlog::event("pin vision: pin not seen, aimed by odometry");
        else
            runlog::event("pin vision: squared up at %+d px", lastVisErrPx);
    };
    // 2b. a far pin: correct the heading on the camera BEFORE driving (the pin's coordinates are only as good
    // as odometry, and a few degrees here is several inches at the pin). Near pins skip this.
    bool farAimed = false; // the camera aimed the line before the drive: nothing re-aims it afterwards
    if (kFarPreAim && dist > kFarPinIn && visionOk && pinVisionSig > 0 && byIR) {
        lastVisSeenMs = 0; // only sightings made from here count
        squareUp();
        pose = chassis.getPose();
        if (lastVisSeenMs != 0 && pros::millis() - lastVisSeenMs < 1000) {
            // the camera had it: drive the line the robot now points along (the camera's pin, not odometry's)
            farAimed = true;
            lineHeading = pose.theta;
            ux = std::sin(lineHeading * (float)M_PI / 180.0f);
            uy = std::cos(lineHeading * (float)M_PI / 180.0f);
            runlog::event("pin far %.0f in: aimed by camera, %.1f deg off odometry", dist,
                          wrap180(lineHeading - std::atan2(a.px - pose.x, a.py - pose.y) * 180.0f / (float)M_PI));
        } else {
            runlog::event("pin far %.0f in: camera did not see it", dist);
        }
        pros::delay(40);
        t0 = pros::millis(); // the timeout is for the drive itself
    }
    while (chassis.isInMotion() && (int)(pros::millis() - t0) < a.timeoutMs) {
        const uint32_t nowMs = pros::millis();
        pose = chassis.getPose();
        const float rx = pose.x - p0x, ry = pose.y - p0y;
        const float along = rx * ux + ry * uy;   // how far along the line
        const float lat = -rx * uy + ry * ux;    // + = left of the line
        const float remaining = stopAt - along;
        speedIn = kSpeedSmooth * speedIn + (1.0f - kSpeedSmooth) * (along - prevAlong) * 100.0f;
        prevAlong = along;
        if (!byIR && !fired) {
            // the sweet spot: far enough ahead of the pin that the claws are shut as the tip arrives
            const float lead = std::max(
                kLineTol, std::max(0.0f, speedIn) * (float)kPinClawLatencyMs / 1000.0f + kGrabBiasIn);
            if (remaining <= lead) {
                clawsClose();
                fired = true;
                runlog::event("pin grab: %.1f in/s lead %.2f left %.2f", speedIn, lead, remaining);
            }
        }
        // IR mode: the sensor saw the pin. Fast grab: the claws were shut by the prediction above.
        const bool seen = byIR ? pinCloseWatchHit() : fired;
        if (seen && seenAt == 0)
            seenAt = pros::millis();

        // vision assist: the moment the camera has the pin -- however far
        // out -- it steers straight at it instead of the odometry line-lean,
        // so it is lined up before it arrives and no deceleration is needed
        // The camera works from pinVisionRangeIn + kVisionSteerIn out: further than that the drive is by
        // odometry alone; visionAssist is just "there is a camera and a signature", which the brake and
        // the stop at pinVisionRangeIn need.
        const bool visionAssist = visionOk && pinVisionSig > 0;
        // (a far pin the camera already aimed at is NOT steered again: re-aiming on the close-range camera
        // is what swung the heading ~12 deg at the end of the run of 2026-11-11 18:16)
        const bool visionOn = visionAssist && !farAimed && remaining <= kVisionWindowIn && remaining > kVisionFreezeIn;
        if (visionOn && !loggedZone) {
            loggedZone = true;
            runlog::event("pin vision: camera steering on, sig %d", pinVisionSig);
        }
        if (visionOn && nowMs - lastVisReadMs >= kVisionReadMs) {
            lastVisReadMs = nowMs;
            const pros::vision_object_s_t o = vision_sensor.get_by_sig(0, pinVisionSig);
            if (o.signature == (uint16_t)pinVisionSig && o.width >= kVisionMinWidth && plausible(o)) {
                noteSighting(o, nowMs);
                if (!loggedSeen) {
                    loggedSeen = true;
                    // raw x_middle as the sensor reports it: when the robot
                    // is square to the pin this should be ~0 (zero-centred)
                    // or ~+158/160 (screen scale) -- that tells which
                    // convention the sensor is really using
                    runlog::event("pin vis: err %+d raw %+d", lastVisErrPx, (int)o.x_middle_coord);
                }
            }
        }
        const bool visionFresh = visionOn && nowMs - lastVisSeenMs <= kVisionFreshMs;

        // the vision pivot: with the pin in view but off-centre, stop, turn
        // in place onto the corrected heading, settle 50 ms, then drive on
        // -- far more accurate than arcing the correction over at speed
        if (kVisionPivot && byIR && visionOn && visionFresh && seenAt == 0 &&
            remaining > pinVisionRangeIn && std::abs(lastVisErrPx) > kVisionAlignedPx) {
            runlog::event("pin vision: pivot %.1f in out, err %+d px", remaining, lastVisErrPx);
            squareUp();
            pros::delay(50); // settle on the new heading before driving on
            continue;
        }

        // the vision stop: pinVisionRangeIn short of the pin, ALWAYS halt
        // here (pin in view or not) -- no deceleration, no braking: full
        // speed right up to this point, a hard stop, then the square-up and
        // the creep-in put the claw on the pin slowly
        if (kStopForVision && byIR && visionAssist && remaining <= pinVisionRangeIn && seenAt == 0) {
            runlog::event("pin vision: stopped %.1f in short, aligning", remaining);
            visionStopped = true;
            break;
        }

        float mag;
        if (seen) {
            // fast grab: keep the speed until the claws have gripped, then ease to a stop
            const uint32_t hold = byIR ? 0u : (uint32_t)kPinClawLatencyMs;
            const uint32_t since = pros::millis() - seenAt;
            if (since >= hold + kSeenStopMs)
                break;
            mag = since < hold ? prevMag
                               : prevMag * (1.0f - (float)(since - hold) / (float)kSeenStopMs);
        } else if (a.straight && remaining < -kStraightOverrunIn) {
            // wall pin: it has driven this far past the pin's coordinates and the IR never saw a pin
            runlog::event("wall pin: no pin seen %.1f in past", -remaining);
            break;
        } else if (remaining <= kLineTol && !a.straight) {
            // The claw tip is on the pin's coordinates and the IR has not fired: the pin is in the claw
            // (the IR beam sits a little deeper than the claw tip), so shut the claws NOW instead of
            // creeping on for the IR (run 2026-11-11: the pin was in the claw, the robot crept 1-2 s
            // and the claws closed only when the drive timed out).
            if (byIR) {
                clawsClose();
                runlog::event("pin arrived, IR silent: claws closed");
            }
            visionStopped = false;
            break;
        } else {
            // the approach brake: the camera path decelerates over the last
            // kVisionBrakeIn inches down to kVisionApproach, so the vision
            // stop lands where it should (the coast from full speed used to
            // carry it past the stop, too close for the camera to square up)
            float target = a.maxSpeed;
            const float toVisStop = remaining - kVisionWindowIn; // slow by the time the camera window starts
            if (byIR && visionAssist && toVisStop < kVisionBrakeIn) {
                target = kVisionApproach + (a.maxSpeed - kVisionApproach) *
                                                std::max(toVisStop, 0.0f) / kVisionBrakeIn;
                if (!visBraked) {
                    visBraked = true;
                    runlog::event("pin vision: braking %.1f in out, power %.0f", toVisStop, target);
                }
            } else if ((!visionAssist || !byIR) && a.decelRange > 0 && a.decelSpeed < a.maxSpeed) {
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
            if (a.straight && remaining <= a.decelRange + kBrakeLeadIn)
                target = std::min(target, std::max(a.decelSpeed, kLineCreep)); // slow where the pin is expected
            if (a.straight && remaining <= 0.0f)
                target = std::min(target, kPinArrivalSpeed); // past its coordinates: creep until the IR fires
            mag = std::min(target, prevMag + kLineAccelStep);
        }
        prevMag = mag;

        // hold the line's heading, leaning a little towards the line when off
        // it -- or, when the vision assist has the pin in view, steering
        // straight at the pin instead of the line
        const float desired = lineHeading +
                              (visionFresh
                                   ? std::clamp(kVisionDegPerPx * (float)lastVisErrPx,
                                                -kVisionMaxDeg, kVisionMaxDeg)
                                   : std::clamp(kLineKCross * lat, -kLineMaxCross, kLineMaxCross));
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

    if (visionStopped) {
        // 3b. square up on the pin: turn in place until the camera has it
        // centred (kVisionAlignedPx), or give up after kVisionAlignMs and
        // just drive the line in. Without a camera there is nothing to
        // square to: go straight to the creep-in.
        if (kStopForVision && visionOk && pinVisionSig > 0 && !farAimed) {
            squareUp();
            pros::delay(50); // settle on the new heading before the last stretch
        }

        // 3c. the last stretch: creep straight in at LOW SPEED and keep going
        // until the IR sensor sees the pin and the claws close on it -- the
        // IR stops it the moment of contact, and it never pushes further
        // than kRunInPushIn (2 in) past the nominal stop point. If nothing
        // is grabbed within kRunInMs, give up: the motion ends and the plan
        // moves on to its next step.
        const uint32_t tRunIn = pros::millis();
        prevMag = kRunInSpeed; // (no ramp: see kRunInSpeed)
        prevTheta = chassis.getPose().theta;
        const float holdTheta = prevTheta;
        while (chassis.isInMotion() && (int)(pros::millis() - tRunIn) < kRunInMs) {
            pose = chassis.getPose();
            const float rx = pose.x - p0x, ry = pose.y - p0y;
            const float along = rx * ux + ry * uy;
            if (along >= stopAt + kRunInPushIn)
                break; // pushed the extra distance without the IR: done
            const bool seen = pinCloseWatchHit();
            if (seen && seenAt == 0)
                seenAt = pros::millis();

            float mag;
            if (seen) {
                const uint32_t since = pros::millis() - seenAt;
                if (since >= kSeenStopMs)
                    break;
                mag = prevMag * (1.0f - (float)since / (float)kSeenStopMs);
            } else {
                mag = std::min(kRunInSpeed, prevMag + kLineAccelStep);
            }
            prevMag = mag;

            // NO correcting any more: the heading was set by the square-up, and steering on the camera
            // or the line this close made the robot wobble in front of the pin, so the IR never got a
            // clean look. It holds the heading it has, and only a real drift (> kRunInDeadband) is steered.
            float err = wrap180(holdTheta - pose.theta);
            const float rate = (pose.theta - prevTheta) * 100.0f;
            prevTheta = pose.theta;
            float turn = std::fabs(err) < kRunInDeadband
                             ? 0.0f
                             : std::clamp(kLineKHeading * err - kLineKRate * rate,
                                          -kLineMaxTurn, kLineMaxTurn);
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
    }

    pose = chassis.getPose();
    {
        const float rx = pose.x - p0x, ry = pose.y - p0y;
        runlog::event("pin line end: along %.1f of %.1f off line %.2f heading %.1f seen %d visErr %d",
                      rx * ux + ry * uy, stopAt, -rx * uy + ry * ux, pose.theta,
                      pinCloseWatchHit() ? 1 : 0, lastVisErrPx);
    }
    chassis.endMotion();
}
} // namespace

void goToPinDrive(float x, float y, int timeoutMs, int maxSpeed, bool forwards, float reach,
                  float decelRange, float decelSpeed, bool wallPin, bool useVision, bool useIR) {
    const float speed = (float)(maxSpeed > 0 ? maxSpeed : kDefaultPathSpeed);
    // the camera follows the yellow pin whenever it sees it, so the yellow signature has to be on
    // the sensor (it used to be installed only by opening the VISION screen)
    if (useVision && !visionEnsureYellow())
        runlog::event("pin vision: sensor not answering");
    const lemlib::Pose p = chassis.getPose();
    const float dx = x - p.x, dy = y - p.y;
    const float dist = std::hypot(dx, dy);
    if (forwards && reach > 0) {
        if (dist <= reach) {
            runlog::event("pin already within reach");
            return;  // the claw is already on the pin
        }
        // face the pin, then drive a straight line to it (see straightPinTask)
        // (the arrival speed is the same for every pin: decelSpeed from the call is not used here)
        auto *a = new StraightPinArgs{x, y, reach, timeoutMs, speed, decelRange,
                                      std::min(kPinArrivalSpeed, speed), wallPin, useVision, useIR};
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
             float decelRange, float decelSpeed, bool wallPin, bool useVision, bool useIR) {
    runlog::event("pin target x=%.2f y=%.2f reach=%.2f level=%d", x, y, reach, level);
    liftOpenClaws();
    // IR mode: the sensor closes the claws the moment it sees the pin, even mid-drive or mid-turn
    if (kPinGrabByIR && useIR)
        pinCloseWatch(true);
    goToPinDrive(x, y, timeoutMs, maxSpeed, forwards, reach, decelRange, decelSpeed, wallPin, useVision,
                 useIR);
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
        // A target at a goal is reached by touching the goal, since its centre can't be reached.
        const bool there = atTarget(p.x, p.y, home.x, home.y, kPosTol);
        const float dist = there ? 0.0f : std::hypot(dx, dy);
        float headErr = std::fmod(home.theta - p.theta + 540.0f, 360.0f) - 180.0f; // -180..180

        if (std::isfinite(dist) && std::isfinite(headErr)) {
            if (dist > kPosTol) {
                // Drive home, going round goals and rerouting if it stalls. A goal home sits at
                // is an intentional contact and is not avoided.
                moveToPointSafe(home.x, home.y, 2500);
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
