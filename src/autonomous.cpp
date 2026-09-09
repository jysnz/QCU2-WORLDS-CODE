#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

void thirty_sec(){
    chassis.moveToPoint(0, -10, 3000);
    chassis.moveToPoint(0, 10, 3000);
    chassis.moveToPoint(0, -10, 3000);
    chassis.waitUntilDone();

    chassis.moveToPose(25, 25, 45, 3000);
    //Shoot the pin and cup
    drivetrainReset();

    chassis.moveToPoint(0, -10, 3000);
    chassis.waitUntilDone();
    chassis.moveToPose(25, 7, 75, 3000);
    chassis.waitUntilDone();
    //Get the pin and cup 
    
    

}

// Auton selector is dropped for now - runAutonomous() ignores
// currentAutonIndex and just runs whatever's written here directly.
void runAutonomous() {
    thirty_sec();
}

