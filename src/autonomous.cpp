#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

// ─── Test routine
// ─────────────────────────────────────────────────────────────

void twoVtwo_left() {
  gateClose();
  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  intakeBlock();
  reset();
  gateClose();

  chassis.moveToPoint(0, 10.5, 3000, {.maxSpeed = 50}); // Go to intake
  chassis.waitUntilDone();
  pros::delay(300);

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1
  reset();

  intakeBlock();

  chassis.moveToPoint(0, 32, 3000,
                      {.maxSpeed = 40}); // Forward to intake enemyballs
  chassis.waitUntilDone();
  pros::delay(350);

  startCatapultShoot();

  gateClose();

  pros::delay(3000);
  reset();

  chassis.moveToPoint(0, -8.5, 3000, {.forwards = false, .maxSpeed = 40});
  chassis.waitUntilDone();

  chassis.turnToHeading(47, 1000);
  chassis.waitUntilDone();
  reset();

  matchloadUp();
  midGoalArm();

  chassis.moveToPoint(-3, -54, 3000,
                      {
                          .forwards = false,
                          .maxSpeed = 70,
                      }); // Go to middle goal
  chassis.waitUntilDone();

  startCatapultShoot();

  chassis.moveToPoint(0, -22, 3000, {.maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  reset();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();
  reset();
  longGoalArm();

  descoreDown();
  chassis.moveToPoint(
      0, -15, 3000, {.forwards = false, .minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
}
void twoVtwo_right() {
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

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1
  reset();

  intakeBlock();

  chassis.moveToPoint(0, 32, 3000,
                      {.maxSpeed = 40}); // Forward to intake enemyballs
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

  chassis.moveToPoint(0, 40, 3000, {.maxSpeed = 40}); // Go to middle goal
  chassis.waitUntilDone();

  outtakeBlock(100); // Outtake to descore
  pros::delay(1200);

  chassis.moveToPoint(0, 14, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  chassis.turnToHeading(52, 1000);
  chassis.waitUntilDone();
  reset();

  descoreDown();
  chassis.moveToPoint(0, 15, 3000, {.minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
}

void path() {
  chassis.moveToPoint(0, 5, 3000);
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();
}

void parkingtest() {
    chassis.setPose(0, 0, 90);

    // go around obstacle to prevent hitting it
    chassis.moveToPose(
        48,
        -24,
        90,
        2000,
        {.minSpeed=72, .earlyExitRange=8}
        // a minSpeed of 72 means that the chassis will slow down as
        // it approaches the target point, but it won't come to a full stop

        // an earlyExitRange of 8 means the movement will exit 8" away from
        // the target point
    );

    // go to target position
    chassis.moveToPose(64, 3, 0, 2000);
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
  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  reset();

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 70}); 
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1

  reset();
  
  chassis.moveToPoint(0, 5, 3000); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false});
  chassis.waitUntilDone();

  reset();
  gateClose();
  intakeBlock();

  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  reset();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  matchloadUp();
  intakeStop();

  matchloadUp();
  chassis.turnToHeading(-145, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(-13, 100, 3500, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -25, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  
  startCatapultShoot();
  startCatapultShoot();

  reset();
  chassis.moveToPoint(0, 5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadDown();

  reset();

  pros::delay(300);
  gateClose();
  intakeBlock();

  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  chassis.moveToPoint(0, 0, 3000, {.forwards = false, .maxSpeed = 40});
  chassis.waitUntilDone();

  startCatapultShoot();
  startCatapultShoot();

  reset();
  chassis.moveToPoint(0, 5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();
  
  reset();

  chassis.moveToPoint(0, 92.5, 3500, {.maxSpeed = 80});
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -13, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot();

  //2nd part
  intakeBlock();
  reset();
  gateClose();

  chassis.moveToPoint(0, 38, 3500, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();
  pros::delay(500);

  reset();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  matchloadUp();
  intakeStop();

  matchloadUp();
  chassis.turnToHeading(-145, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(-13, 100, 3500, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -27, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  startCatapultShoot();
  startCatapultShoot();

  reset();
  chassis.moveToPoint(0, 5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadDown();

  reset();

  pros::delay(300);
  gateClose();
  intakeBlock();

  chassis.moveToPoint(0, 40, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  chassis.moveToPoint(0, -2, 3000, {.forwards = false, .maxSpeed = 70});
  chassis.waitUntilDone();

  startCatapultShoot();
  startCatapultShoot();

  reset();
  chassis.moveToPoint(0, 5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadUp();

  reset();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();

  chassis.moveToPoint(-58, 32, 3000, {.minSpeed = 127});
  chassis.waitUntilDone();


}

// ─── Main autonomous entry point
// ────────────────────────────────────────────── Select which routine runs
// here.
void runAutonomous() { skills(); }
