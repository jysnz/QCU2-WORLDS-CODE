#include "imu_drift.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace {
constexpr uint32_t TICK_MS = 20;
constexpr uint32_t MIN_WINDOW_MS = 2500;   // stand still at least this long to trust a measurement
constexpr uint32_t MAX_WINDOW_MS = 4000;   // ...and re-measure this often while parked
constexpr double MOTOR_STILL_RPM = 2.0;    // drive motors below this count as stopped
constexpr double WHEEL_STILL_CDEG_S = 20;  // rotation sensors: centidegrees/s counted as stopped
constexpr double MAX_RATE = 0.30;          // deg/s, hard cap on the estimate (real IMUs: ~0.01-0.1)
constexpr double BLEND = 0.35;             // how much each new measurement moves the estimate

volatile float rate = 0;   // deg/s currently being subtracted
bool started = false;

} // namespace

bool robotStandingStill() {
  for (double v : left_motor_group.get_actual_velocity_all())
    if (!std::isfinite(v) || std::fabs(v) > MOTOR_STILL_RPM) return false;
  for (double v : right_motor_group.get_actual_velocity_all())
    if (!std::isfinite(v) || std::fabs(v) > MOTOR_STILL_RPM) return false;
  double vv = verticalRotation.get_velocity(), hv = horizontalRotation.get_velocity();
  return std::isfinite(vv) && std::isfinite(hv) && std::fabs(vv) < WHEEL_STILL_CDEG_S &&
         std::fabs(hv) < WHEEL_STILL_CDEG_S;
}

namespace {

// Folds a finished stationary window into the drift estimate.
void measure(double rawNow, double rawStart, uint32_t lenMs) {
  double measured = (rawNow - rawStart) / (lenMs / 1000.0); // deg/s of creep while standing still
  measured = std::clamp(measured, -MAX_RATE, MAX_RATE);
  rate = (float)(rate * (1 - BLEND) + measured * BLEND);
}

void driftTask(void *) {
  bool inWindow = false;
  uint32_t windowStart = 0;
  double windowStartRaw = 0;   // heading (scaled, drift term excluded) when the window opened
  double lastRaw = 0;
  uint32_t last = pros::millis();
  while (true) {
    pros::delay(TICK_MS);
    uint32_t now = pros::millis();
    double dt = (now - last) / 1000.0;
    last = now;
    // the IMU's own reading with the drift term added back: the heading before any correction
    double raw = imu.get_rotation() + imu.drift();
    if (!std::isfinite(raw)) { inWindow = false; continue; } // IMU still calibrating / missing

    imu.addDrift(rate * dt);   // remove the drift estimated so far, a little each tick

    if (!robotStandingStill()) {
      // the robot just started moving: use the stand-still that just ended, if it was long enough
      if (inWindow && now - windowStart >= MIN_WINDOW_MS) measure(lastRaw, windowStartRaw, now - windowStart);
      inWindow = false;
      continue;
    }
    if (!inWindow) { inWindow = true; windowStart = now; windowStartRaw = raw; lastRaw = raw; continue; }
    lastRaw = raw;
    if (now - windowStart >= MAX_WINDOW_MS) {          // parked a long time: re-measure now
      measure(raw, windowStartRaw, now - windowStart);
      windowStart = now;
      windowStartRaw = raw;
    }
  }
}
} // namespace

void startImuDriftCorrection() {
  if (started) return;
  started = true;
  pros::Task(driftTask, nullptr, "imu drift");
}
double imuDriftRate() { return rate; }
double imuDriftTotal() { return imu.drift(); }
