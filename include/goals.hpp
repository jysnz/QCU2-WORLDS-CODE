#pragma once

// Goal positions, in field coordinates: inches, origin at field centre, heading 0 = +Y, clockwise +
// (the same coordinates as the path planner).
struct Goal {
  float x, y;    // centre of the goal
  float radius;  // half the goal's footprint, in inches (it is treated as a circle)
};

extern const Goal kGoals[];
extern const int kGoalCount;

// Assumed sizes, in inches. MEASURE both and set them: they decide how close the robot goes.
constexpr float kGoalRadius = 6.0f;  // from the goal's centre to its outer edge
constexpr float kRobotRadius = 8.0f; // from the robot's centre (tracking centre) to its outer edge

// How close the robot's centre may get to goal i's centre without touching it: the goal's radius
// plus the robot's radius. Inside this distance the robot is touching (colliding with) the goal.
inline float goalClearance(int i) { return kGoals[i].radius + kRobotRadius; }

// Extra room, in inches, left beside the goal when detouring around it.
constexpr float kGoalDetourMargin = 3.0f;

// A move is treated as an intentional contact with a goal when its target lies within that goal's
// clearance + kGoalIntentSlack of its centre (the target is on the goal, or touching it). Such a
// goal is not avoided.
constexpr float kGoalIntentSlack = 4.0f;

// Picks the next point to drive to when going from (fromX, fromY) to (toX, toY) without running
// into a goal. Returns the target itself when the straight way is clear, otherwise a point beside
// the blocking goal (or, if the robot is already inside a goal's keep-out zone, a point just
// outside it). A goal the target sits at is never avoided, because that contact is intentional.
// Calling it again from the new position continues the detour.
void nextSafePoint(float fromX, float fromY, float toX, float toY, float &outX, float &outY);

// Index of the goal a move ending at (x, y) is aimed at (an intentional contact), or -1 if the
// target is not at a goal.
int goalAtTarget(float x, float y);

// True if a robot centred at (x, y) is touching goal i.
bool touchingGoal(int i, float x, float y);

// True when a robot at (px, py) has reached the target (tx, ty): within tol inches of it, OR the
// target is at a goal and the robot is already touching that goal. The goal's centre is where the
// path ends, but the robot can never reach it, so touching the goal counts as arriving.
bool atTarget(float px, float py, float tx, float ty, float tol);

// A point to step aside to when the way from (fromX, fromY) to (toX, toY) is blocked by something
// that is not a goal (another robot). It is kSidestep inches to one side of the way, picking the
// side that stays inside the field and out of the goals. `attempt` alternates the preferred side so
// repeated tries do not keep hitting the same obstacle.
constexpr float kSidestep = 16.0f;
void sidestepPoint(float fromX, float fromY, float toX, float toY, int attempt, float &outX,
                   float &outY);

// Blocking drive to (x, y) that reroutes itself. It goes round goals in the way (nextSafePoint),
// and if the robot stalls on the way (stops making progress while still driving) it concludes
// something other than the target is in the way and sidesteps and retries, up to a few times. A
// stall at a goal the move is aimed at is intentional and is not rerouted. Returns true if it got
// there. maxSpeed is 0-127.
bool moveToPointSafe(float x, float y, int timeoutMs, bool forwards = true, int maxSpeed = 127);
