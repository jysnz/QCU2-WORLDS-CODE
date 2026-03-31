#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"

// ─── Test routine
// ─────────────────────────────────────────────────────────────
void twoVtwo() {
  gateClose();
  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 100}); // Go to matchload  
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  intakeBlock();
  reset();
  gateClose();

  chassis.moveToPoint(0, 10.5, 3000, {.maxSpeed = 50}); // Go to intake
  chassis.waitUntilDone();
  pros::delay(300);

  chassis.moveToPoint(0, -25, 3000, {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1
  reset();

  intakeBlock();

  chassis.moveToPoint(0, 32, 3000, {.maxSpeed = 40}); //Forward to intake enemyballs
  chassis.waitUntilDone();
  pros::delay(350);

  startCatapultShoot();

  gateClose();
  
  pros::delay(3000);
  reset();

  chassis.moveToPoint(0, -13, 3000, {.forwards = false, .maxSpeed = 40});

  chassis.turnToHeading(133, 1000);
  chassis.waitUntilDone();
  reset();

  matchloadUp();

  chassis.moveToPoint(0, 41, 3000, {.maxSpeed = 40}); //Go to middle goal
  chassis.waitUntilDone();

  outtakeBlock(100); //Outtake to descore
  pros::delay(1200);

  chassis.moveToPoint(0, 14, 3000, {.forwards = false, .maxSpeed = 40}); //Go back to shoot
  chassis.waitUntilDone();

  chassis.turnToHeading(52, 1000);
  chassis.waitUntilDone();
  reset();

  descoreDown();
  chassis.moveToPoint(0, 15, 3000, {.minSpeed = 127} ); //Go forward descore
  chassis.waitUntilDone();
  matchloadDown();

}

void path() {
  chassis.moveToPoint(0, 5, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();
}

void parkingtest() {
  chassis.moveToPose(-14, -30, 0, 1500, {.forwards = false, .lead = 0.3});
  chassis.moveToPoint(-14, -85, 3000, {.forwards = false});

  chassis.moveToPose(0, 120, 0, 1500, {.forwards = false, .lead = 0.8});
}

void curve() {
  chassis.setPose(0, 0, 90);

  // go around obstacle to prevent hitting it
  chassis.moveToPose(
      48, -24, 90, 2000, {.minSpeed = 72, .earlyExitRange = 8}
      // a minSpeed of 72 means that the chassis will slow down as
      // it approaches the target point, but it won't come to a full stop

      // an earlyExitRange of 8 means the movement will exit 8" away from
      // the target point
  );

  // go to target position
  chassis.moveToPose(64, 3, 0, 2000);
}

// chassis.moveToPoint(0, 10, 3000);
// chassis.turnToHeading(90, 1000);

void skills() {
  chassis.moveToPoint(0, 38, 3000); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(87, 1000); // Turn to matchload
  chassis.waitUntilDone();
  reset();

  chassis.moveToPoint(0, -22, 3000, {.forwards = false}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1

  pros::delay(500);

  intakeBlock();

  chassis.moveToPoint(0, 17, 3000, {.maxSpeed = 40}); // Go to intake
  gateClose();
  drivetrainLock();

  midGoalArm();

  reset();

  

  // chassis.moveToPoint(0, 70, 3000); //Go forward to 2nd matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(35, 1000); //Turn to 2nd matchload
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, -22, 3000,{.forwards = false}); //Go shootttt 2
  // chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 2

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 2
  // drivetrainLock();

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go back to shoot
  // 3 chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 3

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 3
  // drivetrainLock();

  // chassis.moveToPoint(0, -10, 3000, {.forwards = false}); //middle of field
  // shoot chassis.waitUntilDone();

  // chassis.turnToHeading(-90, 1000); //adjust to other side of field
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 75, 3000); //Go forward to other side of field
  // chassis.waitUntilDone();

  // chassis.turnToHeading(90, 1000); //Turn to 3rd matchload
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 17, 3000, {.maxSpeed = 40}); //Go to 3rd intake
  // chassis.waitUntilDone();

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go to shoot 4
  // chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 4

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 4
  // drivetrainLock();

  // chassis.moveToPoint(0, 10, 3000); //Adjust to 4th matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(-135, 1000); //Turn to 4th matchload
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 17, 3000); //Go forward to 4th matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(-30, 1000); //Turn to 4th matchload
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 70, 3000); //Go forward to 4th matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(32, 1000); //Turn to 4th matchload
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go forward to
  // shoot 5 chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 5

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 5
  // drivetrainLock();

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go back to shoot
  // 6 chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 6

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 6
  // drivetrainLock();

  // chassis.turnToHeading(70, 1000); //turn to parking
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 22, 3000,{.maxSpeed = 60}); //Go to parking
  // chassis.waitUntilDone();

  // chassis.turnToHeading(20, 1000); //Turn to parking
  // chassis.waitUntilDone();
  // reset();

  // chassis.moveToPoint(0, 20, 3000,{.maxSpeed = 60}); //Go to parking
  // chassis.waitUntilDone();
}

// ─── Main autonomous entry point
// ────────────────────────────────────────────── Select which routine runs
// here.
void runAutonomous() { twoVtwo(); }