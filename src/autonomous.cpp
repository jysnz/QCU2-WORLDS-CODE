#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"

// ─── Test routine
// ─────────────────────────────────────────────────────────────
void test() {
  gate.move_absolute(-240, 200);
  chassis.moveToPoint(0, 37, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  intake.move_velocity(-600);
  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 14, 3000, {.maxSpeed = 30});
  pros::delay(2000);
  chassis.moveToPoint(0, -21, 3000, {.forwards = false});
  chassis.waitUntilDone();
  startCatapultShoot();
  pros::delay(1000);
  startCatapultShoot();
  gate.move_absolute(-240, 200);
  pros::delay(2000);

  chassis.waitUntilDone();

  chassis.moveToPoint(0, 19, 3000, {.maxSpeed = 30});
  pros::delay(700);
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -4, 3000);
  chassis.waitUntilDone();

  startCatapultShoot();
  pros::delay(1000);
  chassis.moveToPoint(0, 21, 3000, {.maxSpeed = 30});
  chassis.waitUntilDone();

  pros::delay(2000);
  chassis.setPose(0,0,0);
  chassis.moveToPoint(0, -12, 3000, {.forwards = false});
  chassis.waitUntilDone();
  matchloader.move_absolute(-1300, 100);
  chassis.turnToHeading(129, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 33, 3000);
  chassis.waitUntilDone();
  // intake.move_velocity(0);
  
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
    startCatapultShoot();
}

//chassis.moveToPoint(0, 10, 3000);
//chassis.turnToHeading(90, 1000);

void skills(){
  chassis.moveToPoint(0, 37, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(87, 1000);
  chassis.waitUntilDone();
  
  reset();

  chassis.moveToPoint(0, -22, 3000, {.forwards = false});
  chassis.waitUntilDone();

  startCatapultShoot();

  pros::delay(500);
  leverReset();

  intakeBlock();

  gateClose();

  chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); 
  drivetrainLock();

  midGoalArm();
}

// ─── Main autonomous entry point
// ────────────────────────────────────────────── Select which routine runs
// here.
void runAutonomous() { skills(); }