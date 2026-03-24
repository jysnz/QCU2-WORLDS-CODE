#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"

// ─── Test routine
// ─────────────────────────────────────────────────────────────
void test() {
  chassis.moveToPoint(0, 30, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  intake.move_velocity(-600);
  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 30});
  pros::delay(2000);
  chassis.moveToPoint(0, -17, 3000, {.forwards = false});
  chassis.waitUntilDone();
  startCatapultShoot();
  pros::delay(1000);
  startCatapultShoot();
  pros::delay(2000);

  chassis.waitUntilDone();

  chassis.moveToPoint(0, 15, 3000, {.maxSpeed = 30});
  pros::delay(700);
  chassis.waitUntilDone();

  chassis.moveToPoint(0, 0, 3000);
  chassis.waitUntilDone();

  startCatapultShoot();
  pros::delay(1000);
  chassis.moveToPoint(0, 17, 3000, {.maxSpeed = 30});
  chassis.waitUntilDone();

  pros::delay(2000);
  chassis.moveToPoint(0, -10, 3000, {.forwards = false});
  chassis.waitUntilDone();
  matchloader.move_absolute(-1400, 100);
  chassis.turnToHeading(123, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 28, 3000);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  
  intake.move_velocity(600);
  // matchloader.move_absolute(0, 100);
}

void path() {
  // Path
  chassis.moveToPoint(-45.556, -46.882, 3000);
  turn_to_heading(270, 100);
  chassis.moveToPoint(-58.417, -47.15, 3000);
  turn_to_heading(90, 100);
  chassis.moveToPoint(-31.086, -47.418, 90);
}

void parkingtest(){
    chassis.moveToPoint(0, 100, 3000, {.maxSpeed = 400});
    chassis.waitUntilDone();

    chassis.turnToHeading(45, 1000);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, 10, 3000);
    chassis.waitUntilDone();

    chassis.turnToHeading(120, 1000);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, -13, 3000);
    chassis.waitUntilDone();
}

// ─── Main autonomous entry point
// ────────────────────────────────────────────── Select which routine runs
// here.
void runAutonomous() { test(); }