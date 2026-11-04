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

// BEGIN planner: test  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void test() {
    chassis.setPose(-0.70, -62.50, 0.00);  // start pose
    chassis.moveToPoint(-0.40, -24.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 1: moveToPoint (-0.4, -24.8) 2000ms
    chassis.waitUntilDone();  // 2: waitUntilDone
    chassis.turnToHeading(317.80, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: turnToHeading hdg 317.8 2000ms
    chassis.waitUntilDone();  // 4: waitUntilDone
    chassis.moveToPoint(-23.70, 0.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-23.7, 0.9) 2000ms
    chassis.waitUntilDone();  // 6: waitUntilDone
    chassis.turnToHeading(44.27, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: turnToHeading hdg 44.3 2000ms
    chassis.waitUntilDone();  // 8: waitUntilDone
    chassis.moveToPoint(-0.40, 24.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: moveToPoint (-0.4, 24.8) 2000ms
    chassis.waitUntilDone();  // 10: waitUntilDone
    chassis.turnToHeading(358.76, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 358.8 2000ms
    chassis.waitUntilDone();  // 12: waitUntilDone
    chassis.moveToPoint(-0.90, 47.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: moveToPoint (-0.9, 47.9) 2000ms
    chassis.waitUntilDone();  // 14: waitUntilDone
    chassis.turnToHeading(226.12, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 226.1 2000ms
    chassis.waitUntilDone();  // 16: waitUntilDone
    chassis.moveToPoint(-24.30, 25.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: moveToPoint (-24.3, 25.4) 2000ms
    chassis.waitUntilDone();  // 18: waitUntilDone
    chassis.turnToHeading(313.03, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: turnToHeading hdg 313.0 2000ms
    chassis.waitUntilDone();  // 20: waitUntilDone
    chassis.moveToPoint(-48.40, 47.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: moveToPoint (-48.4, 47.9) 2000ms
    chassis.waitUntilDone();  // 22: waitUntilDone
    chassis.turnToHeading(132.43, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: turnToHeading hdg 132.4 2000ms
    chassis.waitUntilDone();  // 24: waitUntilDone
    chassis.moveToPoint(-24.00, 25.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: moveToPoint (-24.0, 25.6) 2000ms
    chassis.waitUntilDone();  // 26: waitUntilDone
    chassis.turnToHeading(223.72, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: turnToHeading hdg 223.7 2000ms
    chassis.waitUntilDone();  // 28: waitUntilDone
    chassis.moveToPoint(-48.10, 0.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: moveToPoint (-48.1, 0.4) 2000ms
    chassis.waitUntilDone();  // 30: waitUntilDone
    chassis.turnToHeading(133.81, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: turnToHeading hdg 133.8 2000ms
    chassis.waitUntilDone();  // 32: waitUntilDone
    chassis.moveToPoint(-23.50, -23.20, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: moveToPoint (-23.5, -23.2) 2000ms
    chassis.waitUntilDone();  // 34: waitUntilDone
    chassis.turnToHeading(94.01, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 94.0 2000ms
    chassis.waitUntilDone();  // 36: waitUntilDone
    chassis.moveToPoint(-0.70, -24.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-0.7, -24.8) 2000ms
    chassis.waitUntilDone();  // 38: waitUntilDone
}
// END planner: test

// No auton selector - only one autonomous routine.
void runAutonomous() {
    test(); // the plan saved from tools/path_planner.py
    holdEndPosition(-0.70, -24.80, 94.01); // last waypoint of test() -- update if the plan changes
}

