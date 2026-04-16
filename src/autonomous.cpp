#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

extern int currentAutonIndex;

void twoVtwo_left() {
  gateClose();
  chassis.moveToPoint(0, 38, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  intakeBlock();
  reset();
  gateClose();

  chassis.moveToPoint(0, 11.5, 3000, {.maxSpeed = 50}); // Go to intake
  chassis.waitUntilDone();
  // pros::delay(180);

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1
  reset();

  chassis.moveToPoint(0, 10, 3000); 
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -10, 1000, {.forwards = false});
  chassis.waitUntilDone();

  reset();

  intakeBlock();

  chassis.moveToPoint(0, 32, 3000,
                      {.maxSpeed = 40}); // Forward to intake enemyballs
  chassis.waitUntilDone();
  pros::delay(500);

  startCatapultShoot();
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

  chassis.moveToPoint(-1, -53, 3000,
                      {
                          .forwards = false,
                          .maxSpeed = 100,
                      }); // Go to middle goal
  chassis.waitUntilDone();

  startCatapultShoot(false, true);
  startCatapultShoot(false, true);

  chassis.moveToPoint(-1, -20.5, 3000, {.maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  reset();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();
  reset();
  longGoalArm();

  descoreDown();
  chassis.moveToPoint(
      0, -14, 3000, {.forwards = false, .minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();

  gateReset();
}

void twoVtwo_right_red() {
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
  pros::delay(230);

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1
  reset();

  chassis.moveToPoint(0, 10, 3000); 
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -10, 1000, {.forwards = false});
  chassis.waitUntilDone();

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

  chassis.moveToPoint(0, -12, 3000, {.forwards = false, .maxSpeed = 40});

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

  chassis.moveToPoint(-1, 16, 3000,
                      {.forwards = false, .maxSpeed = 70}); // Go back to shoot
  chassis.waitUntilDone();

  chassis.turnToHeading(48, 1000);
  chassis.waitUntilDone();
  reset();
  matchloadUp();
  descoreDown();

  chassis.moveToPoint(0, 14, 3000, {.minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
  intakeStop();
  
  
  gateReset();
}

void twoVtwo_right_blue() {
  gateClose();
  chassis.moveToPoint(0, 38.5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  intakeBlock();
  reset();
  gateClose();

  chassis.moveToPoint(0, 10.5, 3000, {.maxSpeed = 50}); // Go to intake
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -25, 3000,
                      {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  chassis.waitUntilDone();

  startCatapultShoot(); // Shoot 1
  startCatapultShoot(); // Shoot 1

  reset();
  chassis.moveToPoint(0, 10, 3000); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -10, 1000, {.forwards = false});
  chassis.waitUntilDone();

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

  chassis.moveToPoint(0, -12, 3000, {.forwards = false, .maxSpeed = 40});

  chassis.turnToHeading(133, 1000);
  chassis.waitUntilDone();
  reset();

  matchloadUp();

  chassis.moveToPoint(-1, 40.5, 3000, {.maxSpeed = 100}); // Go to middle goal
  chassis.waitUntilDone();

  outtakeBlock(100); // Outtake to descore
  pros::delay(1200);

  chassis.moveToPoint(-1, 42, 3000, {.maxSpeed = 100}); // Go forward descore
  chassis.waitUntilDone();

  chassis.moveToPoint(-1, 38, 3000, {.forwards = false, .maxSpeed = 100}); 
  chassis.moveToPoint(-1, 42, 3000, {.maxSpeed = 100}); // Go forward descore
  chassis.waitUntilDone();

  chassis.moveToPoint(-1, 15, 3000,
                      {.forwards = false, .maxSpeed = 70}); // Go back to shoot
  chassis.waitUntilDone();

  chassis.turnToHeading(48, 1000);
  chassis.waitUntilDone();
  reset();

  descoreDown();
  chassis.moveToPoint(0, 12, 3000, {.minSpeed = 127}); // Go forward descore
  chassis.waitUntilDone();
  intakeStop();
  
  gateReset(); 
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

//   matchloadUp(); // extra 2 points

//   intakeBlock();

//   chassis.moveToPoint(17, 17, 1000, {.maxSpeed = 100});
//   chassis.waitUntilDone();
  
//   reset();

//   chassis.moveToPoint(0, -5, 1000, {.forwards = false, .maxSpeed = 100});
//   chassis.waitUntilDone();
//   chassis.moveToPoint(0, 2, 1000, {.maxSpeed = 100});
//   chassis.waitUntilDone();

//   reset();  

//   chassis.moveToPoint(28, -18, 1500, {.forwards = false, .maxSpeed = 100});
//   chassis.waitUntilDone();

//   startCatapultShoot(false, true);
//   startCatapultShoot(false, true);

//   reset();
//   matchloadDown();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, 92.5, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  reset();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();
  reset();
  
  gateClose();
  intakeBlock();

  chassis.moveToPoint(0, 20, 2000, {.maxSpeed = 40});// Go to intake
  chassis.waitUntilDone();
  drivetrainLock();

  reset();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 80});
  chassis.waitUntilDone();

  chassis.turnToHeading(-145, 1000);
  chassis.waitUntilDone();  

  matchloadUp();
  intakeStop();

  reset();

  chassis.moveToPoint(-13, 100, 3500, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -20, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  reset();
  chassis.moveToPoint(0, 5, 1000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -5, 1000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadDown();

  startCatapultShoot(false, true);
  startCatapultShoot(false, true);

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
  chassis.moveToPoint(0, 7, 1000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -7, 1000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot(false, true);
  startCatapultShoot(false, true);

  matchloadUp();

  intakeBlock();

  chassis.moveToPoint(17, 17, 1000, {.maxSpeed = 100});
  chassis.waitUntilDone();
  
  reset();

  chassis.moveToPoint(0, -5, 1000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, 2, 1000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();  

  chassis.moveToPoint(28, -18, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot(false, true);
  startCatapultShoot(false, true);

  reset();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();

  chassis.moveToPoint(-58, 32, 3000, {.minSpeed = 127});
  chassis.waitUntilDone();

}

void skillsV1() {
  chassis.moveToPoint(0, 30, 3000, {.maxSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000); // Turn to matchload
  chassis.waitUntilDone();
  reset();

  intakeBlock();
  gateClose();

  chassis.moveToPoint(0, 14, 3000, {.maxSpeed = 40});
  pros::delay(600);
  
  reset();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  matchloadUp();
  intakeStop();

  matchloadUp();
  chassis.turnToHeading(-140, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(-13, 115, 3500, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -25, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  startCatapultShoot(true, true);
  pros::delay(250);
  startCatapultShoot(true, true);

  intakeBlock();

  reset();
  chassis.moveToPoint(0, 7, 2000, {.minSpeed = 80}); // Go to matchload
  chassis.waitUntilDone();

  gateClose();

  chassis.moveToPoint(0, -7, 2000, {.forwards = false, .minSpeed = 80});
  chassis.waitUntilDone();

  matchloadDown();

  reset();

  chassis.moveToPoint(0, 39, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();
  pros::delay(300);
  intakeStop();

  chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot(true, true);
  pros::delay(250);
  startCatapultShoot(true, true);

  reset();
  chassis.moveToPoint(0, 7, 2000, {.minSpeed = 100}); // Go to matchload
  chassis.waitUntilDone();

  gateClose();

  chassis.moveToPoint(0, -7, 2000, {.forwards = false, .minSpeed = 80});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 80});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();
  
  reset();

  chassis.moveToPoint(0, 92.5, 3500, {.maxSpeed = 80});
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();

  //2nd part
  intakeBlock();
  reset();

  chassis.moveToPoint(0, 19, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();
  pros::delay(300);
  intakeStop();

  reset();

  chassis.moveToPoint(0, -10, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  matchloadUp();
  chassis.turnToHeading(-140, 1000);
  chassis.waitUntilDone();

  intakeStop();

  reset();

  chassis.moveToPoint(-13, 115, 3500, {.maxSpeed = 100});
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();

  reset();

  chassis.moveToPoint(0, -27, 1500, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  startCatapultShoot(true, true);
  pros::delay(250);
  startCatapultShoot(true, true);

  reset();
  chassis.moveToPoint(0, 7, 2000, {.minSpeed = 80}); // Go to matchload
  chassis.waitUntilDone();

  gateClose();

  chassis.moveToPoint(0, -7, 2000, {.forwards = false, .minSpeed = 80});
  chassis.waitUntilDone();

  intakeBlock();

  matchloadDown();

  reset();

  pros::delay(300);
  intakeBlock();

  chassis.moveToPoint(0, 40, 3000, {.maxSpeed = 40}); // Go to intake
  chassis.waitUntilDone();
  drivetrainLock();
  intakeStop();

  chassis.moveToPoint(0, -3, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();

  startCatapultShoot(true, true);
  pros::delay(250);
  startCatapultShoot(true, true);

  // reset();
  // chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100}); // Go to matchload
  // chassis.waitUntilDone();

  // chassis.moveToPoint(0, -5, 3000, {.forwards = false, .maxSpeed = 100});
  // chassis.waitUntilDone();

  matchloadUp();

  reset();

  chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToHeading(-45, 1000);
  chassis.waitUntilDone();

  chassis.moveToPoint(-58, 32, 3000, {.minSpeed = 127});
  chassis.waitUntilDone();


}

void SoloAWP_Red_Right() {
  // gateClose();
  // chassis.moveToPoint(0, 36.5, 3000, {.maxSpeed = 100}); // Go to matchload
  // chassis.waitUntilDone();

  // chassis.turnToHeading(90, 1000); // Turn to matchload
  // chassis.waitUntilDone();
  // intakeBlock();
  // reset();
  // gateClose();

  // chassis.moveToPoint(0, 10.5, 3000, {.maxSpeed = 50}); // Go to intake
  // chassis.waitUntilDone();
  // pros::delay(230);

  // chassis.moveToPoint(0, -25, 3000,
  //                     {.forwards = false, .maxSpeed = 40}); // Go back to shoot
  // chassis.waitUntilDone();

  // startCatapultShoot(); // Shoot 1
  // startCatapultShoot(); // Shoot 1
  // reset();

  chassis.moveToPoint(0, 10, 3000); 
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -10, 2000, {.forwards = false});
  chassis.waitUntilDone();

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

  chassis.moveToPoint(0, -12, 3000, {.forwards = false, .maxSpeed = 40});

  chassis.turnToHeading(133, 1000);
  chassis.waitUntilDone();
  reset();

  matchloadUp();

  chassis.moveToPoint(-1, 40.5, 3000, {.maxSpeed = 100}); // Go to middle goal
  chassis.waitUntilDone();

  outtakeBlock(100); // Outtake to descore
  pros::delay(1200);

  reset();

  chassis.moveToPoint(0, -15, 3000, {.forwards = false, .maxSpeed = 80}); // Go back to shoot
  chassis.waitUntilDone();
  underGoalArm();
  chassis.turnToHeading(83, 1000);
  chassis.waitUntilDone();

  intakeBlock();
  reset();

  chassis.moveToPoint(0, 26, 3000, {.maxSpeed = 70});
  chassis.waitUntilDone();
  
  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();

  // reset();

  // chassis.moveToPoint(0, 10, 2000);
  // chassis.waitUntilDone();
  // midGoalArm();

  // chassis.turnToHeading(45, 1000);
  // chassis.waitUntilDone();

  // reset();

  // chassis.moveToPoint(0, -10, 2000);
  // chassis.waitUntilDone();

  // startCatapultShoot(true, true);
  // longGoalArm();

  // gateReset();
}

void runAutonomous() {
    switch (currentAutonIndex) {
        case 0: // 2v2 Left
            twoVtwo_left();
            break;
        case 1: // 2v2 Right Red
            twoVtwo_right_red();
            break;
        case 2: // 2v2 Right Blue
            twoVtwo_right_blue();
            break;
        case 3: // Skills
            skills();
            break;
        case 4:
            skillsV1();
            break;
        case 5:
            // SoloAWP_Red_Right
             SoloAWP_Red_Right();
             break;
        default:
            break;
    }
}

