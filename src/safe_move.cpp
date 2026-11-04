#include "goals.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace {

constexpr uint32_t kStallGraceMs = 600;  // ignore the start of a move, while the robot accelerates
constexpr uint32_t kStallWindowMs = 400; // not moving for this long while driving = stalled
constexpr float kStallMoveIn = 0.5f;     // moved less than this in the window = not moving
constexpr float kArriveIn = 1.5f;        // closer than this to the target = there
constexpr int kMaxReroutes = 3;

float dist(float ax, float ay, float bx, float by) { return std::hypot(ax - bx, ay - by); }

bool isForwards(const lemlib::Pose &p, float tx, float ty) {
  const float toPoint = std::atan2(tx - p.x, ty - p.y) * 180.0f / (float)M_PI; // 0 = +Y
  const float rel = std::fmod(toPoint - p.theta + 540.0f, 360.0f) - 180.0f;
  return std::fabs(rel) <= 90.0f;
}

// Drives to (wx, wy) and watches for a stall (and, if contactGoal >= 0, for touching that goal).
// Returns 1 = arrived, 0 = timed out or finished
// elsewhere, -1 = stalled (the motion has been cancelled).
int driveLeg(float wx, float wy, int timeoutMs, bool forwards, int maxSpeed, int contactGoal) {
  chassis.moveToPoint(wx, wy, timeoutMs, {.forwards = forwards, .maxSpeed = (float)maxSpeed});
  pros::delay(20);
  const uint32_t start = pros::millis();
  lemlib::Pose ref = chassis.getPose();
  uint32_t refMs = start;
  while (chassis.isInMotion()) {
    const uint32_t now = pros::millis();
    const lemlib::Pose p = chassis.getPose();
    if (std::isfinite(p.x) && std::isfinite(p.y)) {
      // Aimed at a goal and now touching it: that is the end of the move, stop pushing.
      if (contactGoal >= 0 && touchingGoal(contactGoal, p.x, p.y)) {
        chassis.cancelMotion();
        return 1;
      }
      if (dist(p.x, p.y, ref.x, ref.y) >= kStallMoveIn) {
        ref = p;
        refMs = now;
      } else if (now - start >= kStallGraceMs && now - refMs >= kStallWindowMs &&
                 dist(p.x, p.y, wx, wy) > 3.0f) {
        chassis.cancelMotion();
        return -1;
      }
    }
    pros::delay(20);
  }
  const lemlib::Pose p = chassis.getPose();
  return dist(p.x, p.y, wx, wy) <= kArriveIn + 1.5f ? 1 : 0;
}

} // namespace

bool moveToPointSafe(float x, float y, int timeoutMs, bool forwards, int maxSpeed) {
  const uint32_t begin = pros::millis();
  int reroutes = 0;
  const int goalIdx = goalAtTarget(x, y); // >= 0: the move ends at a goal on purpose

  while (pros::millis() - begin < (uint32_t)timeoutMs) {
    const lemlib::Pose p = chassis.getPose();
    if (atTarget(p.x, p.y, x, y, kArriveIn))
      return true;

    float wx, wy;
    nextSafePoint(p.x, p.y, x, y, wx, wy);
    const bool direct = (wx == x && wy == y);
    const int left = timeoutMs - (int)(pros::millis() - begin);
    const bool fwd = direct ? forwards : isForwards(p, wx, wy);

    const int result = driveLeg(wx, wy, left, fwd, maxSpeed, direct ? goalIdx : -1);
    if (result == 1 && direct)
      return true;
    if (result == -1) {
      // Stalled. If the move is aimed at a goal, the contact is intentional: stop pushing and
      // report success rather than reroute.
      if (goalIdx >= 0 && direct)
        return true;
      if (++reroutes > kMaxReroutes)
        return false;
      const lemlib::Pose q = chassis.getPose();
      float sx, sy;
      sidestepPoint(q.x, q.y, x, y, reroutes, sx, sy);
      driveLeg(sx, sy, 1200, isForwards(q, sx, sy), maxSpeed, -1); // step aside, then plan again
    }
    // result 0 on a detour leg: loop and plan again from wherever it ended
  }
  return false;
}
