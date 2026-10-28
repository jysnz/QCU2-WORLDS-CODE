#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

void thirty_sec(){
    chassis.moveToPoint(0, 24, 1000); // Move to matchload
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000); // Turn to matchload
    chassis.waitUntilDone();
    
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    thirty_sec();
}

