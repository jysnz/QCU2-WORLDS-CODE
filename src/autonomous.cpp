#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

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

  chassis.moveToPoint(-1, -54, 3000,
                      {
                          .forwards = false,
                          .maxSpeed = 100,
                      }); // Go to middle goal
  chassis.waitUntilDone();

  startCatapultShoot(true);
  startCatapultShoot(true);

  chassis.moveToPoint(-1, -20.5, 3000, {.maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  reset();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();
  reset();
  longGoalArm();

  descoreDown();
  chassis.moveToPoint(
      0, -12, 3000, {.forwards = false, .minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
}
void twoVtwo_right() {
  gateClose();
  chassis.moveToPoint(0, 36.5, 3000, {.maxSpeed = 100}); // Go to matchload
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

  chassis.moveToPoint(0, -11, 3000, {.forwards = false, .maxSpeed = 40});

  chassis.turnToHeading(133, 1000);
  chassis.waitUntilDone();
  reset();

  matchloadUp();

  chassis.moveToPoint(-1, 40.5, 3000, {.maxSpeed = 100}); // Go to middle goal
  chassis.waitUntilDone();

  outtakeBlock(100); // Outtake to descore
  pros::delay(1200);

  chassis.moveToPoint(-1, 48, 3000, {.maxSpeed = 100}); // Go forward descore
  chassis.waitUntilDone();

  chassis.moveToPoint(-1, 17, 3000,
                      {.forwards = false, .maxSpeed = 70}); // Go back to shoot
  chassis.waitUntilDone();

  chassis.turnToHeading(48, 1000);
  chassis.waitUntilDone();
  reset();
  matchloadUp();

  descoreDown();
  chassis.moveToPoint(0, 12, 3000, {.minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
  intakeStop();


}

void skills() {
  // chassis.moveToPoint(0, 30, 3000, {.maxSpeed = 100}); // Go to matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(90, 1000); // Turn to matchload
  // chassis.waitUntilDone();
  // reset();
  // intakeBlock();

  // gateClose();
  // chassis.moveToPoint(0, 14, 3000, {.maxSpeed = 40}); // Go to intake
  // chassis.waitUntilDone();
  // drivetrainLock();

  // reset();

  // chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();
  // matchloadUp();

  // chassis.turnToHeading(-145, 1000);
  // chassis.waitUntilDone();

  // reset();

  // chassis.moveToPoint(-13, 100, 3500, {.maxSpeed = 100});
  // chassis.waitUntilDone();
  
  // reset();

  // chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();

  // chassis.turnToHeading(-90, 1000);
  // chassis.waitUntilDone();

  // reset();

  // chassis.moveToPoint(0, -20, 1500, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();

  // matchloadDown();

  // reset();
  // chassis.moveToPoint(0, 7, 3000, {.maxSpeed = 100}); // Go to matchload
  // chassis.waitUntilDone();

  // chassis.moveToPoint(0, -7, 3000, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();

  // startCatapultShoot();
  // startCatapultShoot();

  // reset();

  // gateClose();

  // chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 40}); // Go to intake
  // chassis.waitUntilDone();
  // drivetrainLock();

  // chassis.moveToPoint(0, 0, 3000, {.forwards = false, .maxSpeed = 40});
  // chassis.waitUntilDone();

  // reset();
  // chassis.moveToPoint(0, 7, 3000, {.maxSpeed = 100}); // Go to matchload
  // chassis.waitUntilDone();

  // chassis.moveToPoint(0, -7, 3000, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();

  // startCatapultShoot();
  // startCatapultShoot();

  // reset();
  // intakeStop();

  matchloadUp(); // extra 2 points

  intakeBlock();

  chassis.moveToPoint(17, 17, 1000, {.maxSpeed = 100});
  chassis.waitUntilDone();
  
  reset();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, 2, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(22, -16, 1000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot();
  startCatapultShoot();

  reset();
  matchloadDown();

  chassis.moveToPoint(-14, 9, 1000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, 81.5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  reset();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();
  reset();
  
  gateClose();
  intakeBlock();

  chassis.moveToPoint(0, 24, 3000, {.maxSpeed = 40});// Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntilDone();

  chassis.turnToHeading(145, 1000);
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

  reset();
  chassis.moveToPoint(0, 5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadDown();

  startCatapultShoot();
  startCatapultShoot();

  reset();

  pros::delay(300);
  intakeBlock();
  gateClose();
  chassis.moveToPoint(0, 40, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  chassis.moveToPoint(0, -2, 3000, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntilDone();

  reset();
  chassis.moveToPoint(0, 7, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -7, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot();
  startCatapultShoot();

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
void runAutonomous() { twoVtwo_left(); }
