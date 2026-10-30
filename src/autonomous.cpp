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

void scrimmageAutoAWP() {
    // Robot is placed facing the first point (Jerry.io assumes it faces away: 118.811).
    chassis.setPose(-2.070, -60.135, 298.811);
    driveTo(-14.804, -53.131, true);  
    turnTo(65.225);
    driveTo(-23.081, -56.951, false);  
    turnTo(0);
    driveTo(-23.081, -62.681, false); 
    driveTo(-23.081, -54.086, true);  
    turnTo(108.667);
    driveTo(-46.639, -46.127, false); 
    turnTo(92.603);
    driveTo(-25.628, -47.083, true);  
    turnTo(3.918);
    driveTo(-24.036, -23.843, true);   
    turnTo(90.807);
    driveTo(-46.639, -23.525, false);  
    turnTo(283.158);
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    scrimmageAutoAWP();
    holdEndPosition(-46.639, -23.525, 283.158); // last waypoint of scrimmageAutoAWP
}

