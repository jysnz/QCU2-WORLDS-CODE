#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

void thirty_sec(){
    chassis.moveToPoint(0, 5, 3000, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -12, 3000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000); // Turn to matchload
    chassis.moveToPoint(7, 16, 3000, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();
    
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    thirty_sec();
}

