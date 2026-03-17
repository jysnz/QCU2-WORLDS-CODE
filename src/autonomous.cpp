#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"

// ─── Test routine
// ─────────────────────────────────────────────────────────────
void test() {
  chassis.moveToPoint(0, 37, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 3000);
  chassis.waitUntilDone();

  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, 15, 3000, {.forwards = false});
}

void path() {
  // Path
  chassis.moveToPoint(-45.556, -46.882, 3000);
  turn_to_heading(270, 100);
  chassis.moveToPoint(-58.417, -47.15, 3000);
  turn_to_heading(90, 100);
  chassis.moveToPoint(-31.086, -47.418, 90);
}

// ─── Main autonomous entry point
// ────────────────────────────────────────────── Select which routine runs
// here.
void runAutonomous() { test(); }