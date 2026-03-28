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

void curve(){
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

//chassis.moveToPoint(0, 10, 3000);
//chassis.turnToHeading(90, 1000);

void skills(){
  chassis.moveToPoint(0, 38, 3000); //Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(87, 1000); //Turn to matchload
  chassis.waitUntilDone();
  reset();

  chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); //Shoot 1

  pros::delay(500); 

  intakeBlock();


  chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 
  gateClose();
  drivetrainLock();

  midGoalArm();
 
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

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go back to shoot 3
  // chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 3

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 3
  // drivetrainLock();

  // chassis.moveToPoint(0, -10, 3000, {.forwards = false}); //middle of field shoot
  // chassis.waitUntilDone();

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

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go forward to shoot 5
  // chassis.waitUntilDone();

  // startCatapultShoot(); //Shoot 5

  // pros::delay(500);
  // leverReset();

  // intakeBlock();

  // gateClose();

  // chassis.moveToPoint(0, 17, 3000,{.maxSpeed = 40}); //Go to intake 5
  // drivetrainLock();

  // chassis.moveToPoint(0, -22, 3000, {.forwards = false}); //Go back to shoot 6
  // chassis.waitUntilDone();

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
void runAutonomous() { skills(); }