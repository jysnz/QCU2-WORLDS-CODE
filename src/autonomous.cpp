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

void test(){
    chassis.setPose(-0.767, -59.835, 297.553);
    driveTo(-14.776, -52.526, true);

}

// BEGIN planner: thirtyseconds  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void thirtyseconds() {
    chassis.setPose(0.10, -64.10, 0.00);  // start pose
    chassis.swingToHeading(290.90, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 1: swingToHeading hdg 290.9 LEFT 2000ms
    chassis.waitUntilDone();  // 2: waitUntilDone
    chassis.turnToHeading(295.41, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: turnToHeading hdg 295.4 2000ms
    chassis.waitUntilDone();  // 4: waitUntilDone
    chassis.moveToPoint(-19.20, -50.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-19.2, -50.3) 2000ms
    chassis.waitUntilDone();  // 6: waitUntilDone
    chassis.swingToHeading(137.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: swingToHeading hdg 137.2 RIGHT BWD 2000ms
    chassis.waitUntilDone();  // 8: waitUntilDone
    chassis.turnToHeading(316.22, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: turnToHeading hdg 316.2 2000ms
    chassis.waitUntilDone();  // 10: waitUntilDone
    chassis.moveToPoint(-24.50, -24.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (-24.5, -24.3) 2000ms
    chassis.waitUntilDone();  // 12: waitUntilDone
    chassis.turnToHeading(225.58, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: turnToHeading hdg 225.6 2000ms
    chassis.waitUntilDone();  // 14: waitUntilDone
    chassis.moveToPoint(-49.50, -48.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-49.5, -48.8) 2000ms
    chassis.waitUntilDone();  // 16: waitUntilDone
    chassis.turnToHeading(87.98, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 88.0 2000ms
    chassis.waitUntilDone();  // 18: waitUntilDone
    chassis.moveToPoint(-26.80, -48.00, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: moveToPoint (-26.8, -48.0) 2000ms
    chassis.waitUntilDone();  // 20: waitUntilDone
    chassis.turnToHeading(239.08, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 239.1 2000ms
    chassis.waitUntilDone();  // 22: waitUntilDone
    chassis.moveToPoint(-45.00, -58.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-45.0, -58.9) 2000ms
    chassis.waitUntilDone();  // 24: waitUntilDone
    chassis.turnToHeading(268.16, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: turnToHeading hdg 268.2 2000ms
    chassis.waitUntilDone();  // 26: waitUntilDone
    chassis.moveToPoint(-60.60, -59.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: moveToPoint (-60.6, -59.4) 2000ms
    chassis.waitUntilDone();  // 28: waitUntilDone
    chassis.turnToHeading(72.86, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 72.9 2000ms
    chassis.waitUntilDone();  // 30: waitUntilDone
    chassis.moveToPoint(-28.50, -49.50, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-28.5, -49.5) BWD 2000ms
    chassis.waitUntilDone();  // 32: waitUntilDone
    chassis.moveToPoint(-38.68, -55.86, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 6.00});  // 33: moveToPoint (-38.7, -55.9) 2000ms
    chassis.waitUntilDone();  // 34: waitUntilDone
    chassis.moveToPoint(-49.87, -59.83, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 6.00});  // 35: moveToPoint (-49.9, -59.8) 2000ms
    chassis.waitUntilDone();  // 36: waitUntilDone
    chassis.moveToPoint(-60.10, -59.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-60.1, -59.9) 2000ms
    chassis.waitUntilDone();  // 38: waitUntilDone
    chassis.turnToHeading(71.52, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 39: turnToHeading hdg 71.5 2000ms
    chassis.waitUntilDone();  // 40: waitUntilDone
    chassis.moveToPoint(-24.50, -48.00, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: moveToPoint (-24.5, -48.0) BWD 2000ms
    chassis.waitUntilDone();  // 42: waitUntilDone
}
// END planner: thirtyseconds

// No auton selector - only one autonomous routine.
void runAutonomous() {
    scrimmageAutoAWP();
    holdEndPosition(-46.639, -23.525, 283.158); // last waypoint of scrimmageAutoAWP
}

