#include "goals.hpp"
#include <cmath>

// Read off tools/field.png (500 px = 144 in, origin at the centre):
//   x = (px - 250) * 0.288,  y = (250 - py) * 0.288
// They are good to about an inch. Replace them with measured values if you have them.
const Goal kGoals[] = {
    {-48.4, -23.0, kGoalRadius}, // red,  pixel (82, 330)
    {-24.5, -47.5, kGoalRadius}, // red,  pixel (165, 415)
    {24.0, 49.0, kGoalRadius},   // blue, pixel (333, 80)
    {47.0, 24.5, kGoalRadius},   // blue, pixel (413, 165)
};
const int kGoalCount = sizeof(kGoals) / sizeof(kGoals[0]);

namespace {

constexpr float kFieldHalf = 72.0f;
constexpr float kWallMargin = 6.0f; // keep detour points this far inside the field walls

float dist(float ax, float ay, float bx, float by) { return std::hypot(ax - bx, ay - by); }

// Distance from point (px, py) to the segment a->b, and where along it (0..1) the closest point is.
float segmentDistance(float ax, float ay, float bx, float by, float px, float py, float &tOut) {
  const float dx = bx - ax, dy = by - ay;
  const float len2 = dx * dx + dy * dy;
  float t = len2 > 1e-6f ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0f;
  t = std::fmax(0.0f, std::fmin(1.0f, t));
  tOut = t;
  return dist(px, py, ax + t * dx, ay + t * dy);
}

// Cost of a detour point: path length through it, with a large penalty for leaving the field or
// ending up inside another goal's keep-out zone.
float detourCost(float fx, float fy, float tx, float ty, float cx, float cy, int skipGoal) {
  float cost = dist(fx, fy, cx, cy) + dist(cx, cy, tx, ty);
  if (std::fabs(cx) > kFieldHalf - kWallMargin || std::fabs(cy) > kFieldHalf - kWallMargin)
    cost += 1000.0f;
  for (int i = 0; i < kGoalCount; i++) {
    if (i != skipGoal && dist(cx, cy, kGoals[i].x, kGoals[i].y) < goalClearance(i))
      cost += 1000.0f;
  }
  return cost;
}

} // namespace

void nextSafePoint(float fromX, float fromY, float toX, float toY, float &outX, float &outY) {
  outX = toX;
  outY = toY;

  // Goals the target sits at are an intentional contact, so they are not avoided.
  bool avoid[16];
  const int n = kGoalCount < 16 ? kGoalCount : 16;
  for (int i = 0; i < n; i++)
    avoid[i] = dist(toX, toY, kGoals[i].x, kGoals[i].y) > goalClearance(i) + kGoalIntentSlack;

  // Already inside a goal's keep-out zone (for example after being pushed into it): back straight
  // out of it first.
  for (int i = 0; i < n; i++) {
    if (!avoid[i])
      continue;
    const float d = dist(fromX, fromY, kGoals[i].x, kGoals[i].y);
    if (d < goalClearance(i) - 0.5f) {
      float ux = 1.0f, uy = 0.0f;
      if (d > 0.01f) {
        ux = (fromX - kGoals[i].x) / d;
        uy = (fromY - kGoals[i].y) / d;
      }
      outX = kGoals[i].x + ux * (goalClearance(i) + kGoalDetourMargin);
      outY = kGoals[i].y + uy * (goalClearance(i) + kGoalDetourMargin);
      return;
    }
  }

  // Find the first goal the straight way crosses.
  int blocker = -1;
  float bestT = 2.0f;
  for (int i = 0; i < n; i++) {
    if (!avoid[i])
      continue;
    float t;
    const float d = segmentDistance(fromX, fromY, toX, toY, kGoals[i].x, kGoals[i].y, t);
    if (d < goalClearance(i) && t < bestT) {
      bestT = t;
      blocker = i;
    }
  }
  if (blocker < 0)
    return; // clear

  // Go round the blocking goal: to a point beside it, on whichever side is shorter and stays inside
  // the field and out of the other goals.
  const float dx = toX - fromX, dy = toY - fromY;
  const float len = std::hypot(dx, dy);
  const float px = len > 1e-6f ? -dy / len : 1.0f; // unit vector perpendicular to the way
  const float py = len > 1e-6f ? dx / len : 0.0f;
  const float r = goalClearance(blocker) + kGoalDetourMargin;
  const float ax = kGoals[blocker].x + px * r, ay = kGoals[blocker].y + py * r;
  const float bx = kGoals[blocker].x - px * r, by = kGoals[blocker].y - py * r;
  if (detourCost(fromX, fromY, toX, toY, ax, ay, blocker) <=
      detourCost(fromX, fromY, toX, toY, bx, by, blocker)) {
    outX = ax;
    outY = ay;
  } else {
    outX = bx;
    outY = by;
  }
}

int goalAtTarget(float x, float y) {
  int best = -1;
  float bestD = 1e9f;
  for (int i = 0; i < kGoalCount; i++) {
    const float d = dist(x, y, kGoals[i].x, kGoals[i].y);
    if (d <= goalClearance(i) + kGoalIntentSlack && d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

bool touchingGoal(int i, float x, float y) {
  return dist(x, y, kGoals[i].x, kGoals[i].y) <= goalClearance(i) + 1.5f;
}

bool atTarget(float px, float py, float tx, float ty, float tol) {
  if (dist(px, py, tx, ty) <= tol)
    return true;
  const int gi = goalAtTarget(tx, ty);
  return gi >= 0 && touchingGoal(gi, px, py);
}

void sidestepPoint(float fromX, float fromY, float toX, float toY, int attempt, float &outX,
                   float &outY) {
  const float dx = toX - fromX, dy = toY - fromY;
  const float len = std::hypot(dx, dy);
  const float ux = len > 1e-6f ? dx / len : 0.0f, uy = len > 1e-6f ? dy / len : 1.0f;
  const float px = -uy, py = ux; // unit vector to the left of the way
  // Step sideways and a little back from the obstacle.
  const float ax = fromX + px * kSidestep - ux * 4.0f, ay = fromY + py * kSidestep - uy * 4.0f;
  const float bx = fromX - px * kSidestep - ux * 4.0f, by = fromY - py * kSidestep - uy * 4.0f;
  const float ca = detourCost(fromX, fromY, toX, toY, ax, ay, -1);
  const float cb = detourCost(fromX, fromY, toX, toY, bx, by, -1);
  bool useA = ca <= cb;
  // Both sides legal (cost below the 1000 penalty): alternate so a retry tries the other side.
  if (ca < 1000.0f && cb < 1000.0f && (attempt % 2) == 1)
    useA = !useA;
  outX = useA ? ax : bx;
  outY = useA ? ay : by;
}
