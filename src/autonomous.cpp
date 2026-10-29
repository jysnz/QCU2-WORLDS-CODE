#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

void thirty_sec(){
    chassis.setPose(0.351, -67.586, 0); // x, y, heading in degrees (0 = facing +Y); not a timeout
    chassis.moveToPoint(0.654, -48.203, 4000);
    chassis.waitUntilDone(); // moves are async: without this the next call replaces the one running
    chassis.turnToHeading(90, 4000);
    chassis.waitUntilDone();
    chassis.moveToPoint(23.369, -48.203, 4000);
    chassis.waitUntilDone();
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    thirty_sec();
    holdEndPosition();
}

