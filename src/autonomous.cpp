#include "lift_calibration.hpp"
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
    while (chassis.isInMotion()) pros::delay(10);  // 2: waitUntilDone
    chassis.turnToHeading(295.41, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: turnToHeading hdg 295.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.moveToPoint(-19.20, -50.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-19.2, -50.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.swingToHeading(137.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: swingToHeading hdg 137.2 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    chassis.turnToHeading(316.22, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: turnToHeading hdg 316.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 10: waitUntilDone
    chassis.moveToPoint(-24.50, -24.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (-24.5, -24.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.turnToHeading(225.58, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: turnToHeading hdg 225.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    chassis.moveToPoint(-49.50, -48.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-49.5, -48.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.turnToHeading(87.98, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 88.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    chassis.moveToPoint(-26.80, -48.00, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: moveToPoint (-26.8, -48.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 20: waitUntilDone
    chassis.turnToHeading(239.08, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 239.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-45.00, -58.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-45.0, -58.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    chassis.turnToHeading(268.16, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: turnToHeading hdg 268.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 26: waitUntilDone
    chassis.moveToPoint(-60.60, -59.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: moveToPoint (-60.6, -59.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.turnToHeading(72.86, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 72.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-28.50, -49.50, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-28.5, -49.5) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.moveToPoint(-38.68, -55.86, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 6.00});  // 33: moveToPoint (-38.7, -55.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    chassis.moveToPoint(-49.87, -59.83, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 6.00});  // 35: moveToPoint (-49.9, -59.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-60.10, -59.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-60.1, -59.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    chassis.turnToHeading(71.52, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 39: turnToHeading hdg 71.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 40: waitUntilDone
    chassis.moveToPoint(-24.50, -48.00, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: moveToPoint (-24.5, -48.0) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
}
// END planner: thirtyseconds

// BEGIN planner: auto_30scrim  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void auto_30scrim() {
    chassis.setPose(-0.90, -63.90, 1.20);  // start pose
    homeLift();  // 1: homeLift
    claw.retract(); claw2.retract();  // 2: claw BOTH OFF
    clawsClose();  // 3: claw BOTH ON
    liftGoToLevel(0, true, 2000);  // 4: liftToLevel GROUND LIFT (wait)
    chassis.swingToHeading(296.70, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: swingToHeading hdg 296.7 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.moveToPoint(-17.60, -51.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-17.6, -51.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    chassis.swingToHeading(140.80, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: swingToHeading hdg 140.8 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(327.44, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 327.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    chassis.moveToPoint(-21.30, -28.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: moveToPoint (-21.3, -28.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    clawsClose();  // 16: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 17: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(192.13, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: turnToHeading hdg 192.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    chassis.moveToPoint(-23.60, -40.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: moveToPoint (-23.6, -40.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    liftDropAtLevel(1, 2000);  // 22: liftToLevel LOW DROP (+ open claws)
    claw.retract(); claw2.retract();  // 23: claw BOTH OFF
    chassis.swingToHeading(60.10, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: swingToHeading hdg 60.1 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    chassis.turnToHeading(235.66, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 235.7 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    chassis.moveToPoint(-43.20, -44.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: moveToPoint (-43.2, -44.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    clawsClose();  // 30: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 31: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(101.92, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 101.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    chassis.moveToPoint(-32.30, -46.70, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 34: moveToPoint (-32.3, -46.7) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    liftDropAtLevel(2, 2000);  // 36: liftToLevel MID DROP (+ open claws)
    claw.retract(); claw2.retract();  // 37: claw BOTH OFF
    chassis.swingToHeading(285.40, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: swingToHeading hdg 285.4 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    chassis.turnToHeading(101.50, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: turnToHeading hdg 101.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    chassis.turnToHeading(264.14, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 42: turnToHeading hdg 264.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 43: waitUntilDone
    chassis.moveToPoint(-51.60, -59.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: moveToPoint (-51.6, -59.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    chassis.turnToHeading(271.49, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 46: turnToHeading hdg 271.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 47: waitUntilDone
    chassis.moveToPoint(-63.10, -59.00, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: moveToPoint (-63.1, -59.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    clawsClose();  // 50: claw BOTH ON
    liftGoToLevel(3, true, 2000);  // 51: liftToLevel HIGH LIFT (wait)
    liftDropAtLevel(3, 2000);  // 52: liftToLevel HIGH DROP (+ open claws)
    claw.retract(); claw2.retract();  // 53: claw BOTH OFF
}
// END planner: auto_30scrim

// BEGIN planner: auto_30  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void auto_30() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(295.90, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 295.9 LEFT 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 6: liftToLevel GROUND LIFT (wait)
    chassis.moveToPoint(-19.10, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-19.1, -50.8) 2000ms
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    chassis.swingToHeading(149.00, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: swingToHeading hdg 149.0 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(328.65, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 328.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    chassis.moveToPoint(-21.60, -27.50, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-21.6, -27.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.turnToHeading(335.80, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 335.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(188.51, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 188.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -41.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -41.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    liftDropAtLevel(1, 2000);  // 25: liftToLevel LOW DROP (+ open claws)
}
// END planner: auto_30

// BEGIN planner: newauto  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void newauto() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(295.90, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 295.9 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 6: liftToLevel GROUND LIFT (wait)
    chassis.moveToPoint(-19.10, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-19.1, -50.8) 2000ms
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    chassis.swingToHeading(149.00, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: swingToHeading hdg 149.0 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(328.65, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 328.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    chassis.moveToPoint(-20.70, -28.10, 2000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-20.7, -28.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.turnToHeading(320.50, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 320.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(188.51, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 188.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -41.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -41.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    liftDropAtLevel(1, 2000);  // 25: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(59.60, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: swingToHeading hdg 59.6 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    homeLift();  // 28: homeLift
    chassis.turnToHeading(237.87, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 237.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-43.50, -44.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-43.5, -44.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    clawsClose();  // 33: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 34: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(102.19, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 102.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-30.50, -46.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-30.5, -46.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    liftDropAtLevel(2, 2000);  // 39: liftToLevel MID DROP (+ open claws)
    chassis.turnToHeading(298.80, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: turnToHeading hdg 298.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    homeLift();  // 43: homeLift
    chassis.turnToHeading(271.90, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: turnToHeading hdg 271.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    clawsClose();  // 46: claw BOTH ON
    liftGoToLevel(0, true, 2000);  // 47: liftToLevel GROUND LIFT (wait)
    chassis.swingToHeading(31.40, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: swingToHeading hdg 31.4 RIGHT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.turnToHeading(41.83, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: turnToHeading hdg 41.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
    chassis.moveToPoint(-7.20, -8.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 52: moveToPoint (-7.2, -8.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 53: waitUntilDone
}
// END planner: newauto

// BEGIN planner: newnew  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void newnew() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    liftGoToLevel(1, false, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(295.90, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 295.9 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-19.10, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-19.1, -50.8) 2000ms
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    chassis.swingToHeading(149.00, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: swingToHeading hdg 149.0 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(328.65, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 328.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    chassis.moveToPoint(-19.40, -30.00, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-19.4, -30.0) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.turnToHeading(320.50, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 320.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(188.51, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 188.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -41.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -41.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    liftDropAtLevel(1, 2000);  // 25: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(59.60, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: swingToHeading hdg 59.6 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    homeLift();  // 28: homeLift
    chassis.turnToHeading(237.87, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 237.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-42.40, -43.80, 2000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-42.4, -43.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    clawsClose();  // 33: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 34: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(102.19, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 102.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-30.50, -46.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-30.5, -46.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    liftDropAtLevel(2, 2000);  // 39: liftToLevel MID DROP (+ open claws)
    chassis.turnToHeading(298.80, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: turnToHeading hdg 298.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    homeLift();  // 43: homeLift
    chassis.turnToHeading(271.90, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: turnToHeading hdg 271.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    clawsClose();  // 46: claw BOTH ON
    homeLift();  // 47: homeLift
    chassis.swingToHeading(31.40, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: swingToHeading hdg 31.4 RIGHT 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.moveToPoint(-7.20, -8.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: moveToPoint (-7.2, -8.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
}
// END planner: newnew

// BEGIN planner: last  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void last() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    clawsClose();  // 1: claw BOTH ON
    homeLift();  // 2: homeLift
    liftGoToLevel(0, true, 1000);  // 3: liftToLevel GROUND LIFT (wait)
    chassis.swingToHeading(298.60, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 298.6 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-19.10, -51.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-19.1, -51.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    chassis.swingToHeading(147.90, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: swingToHeading hdg 147.9 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 10: waitUntilDone
    chassis.turnToHeading(330.00, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 330.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    moveToPointDecel(-21.60, -29.40, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 40);  // 13: moveToPoint (-21.6, -29.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    clawsClose();  // 15: claw BOTH ON
}
// END planner: last

// BEGIN planner: lastlast  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void lastlast() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    clawsClose();  // 1: claw BOTH ON
    homeLift();  // 2: homeLift
    liftGoToLevel(0, true, 2000);  // 3: liftToLevel GROUND LIFT (wait)
    chassis.swingToHeading(298.60, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 298.6 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.80, -51.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.8, -51.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    pros::delay(500);  // 8: wait 500 ms
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    while (chassis.isInMotion()) pros::delay(10);  // 10: waitUntilDone
    chassis.swingToHeading(155.50, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: swingToHeading hdg 155.5 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.turnToHeading(328.18, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: turnToHeading hdg 328.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    homeLift();  // 15: homeLift
    moveToPointDecel(-21.30, -29.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 40);  // 16: moveToPoint (-21.3, -29.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    clawsClose();  // 18: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 19: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(186.15, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: turnToHeading hdg 186.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    chassis.moveToPoint(-23.50, -39.20, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: moveToPoint (-23.5, -39.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    pros::delay(500);  // 24: wait 500 ms
    liftDropAtLevel(1, 2000);  // 25: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(56.60, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: swingToHeading hdg 56.6 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    homeLift();  // 28: homeLift
    chassis.turnToHeading(232.09, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 232.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    moveToPointDecel(-42.70, -43.00, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 40);  // 31: moveToPoint (-42.7, -43.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    clawsClose();  // 33: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 34: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(106.39, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 106.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-30.20, -46.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-30.2, -46.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    liftDropAtLevel(2, 2000);  // 39: liftToLevel MID DROP (+ open claws)
}
// END planner: lastlast

// BEGIN planner: last3  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void last3() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(299.40, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 299.4 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    moveToPointDecel(-18.80, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 6: moveToPoint (-18.8, -50.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    chassis.swingToHeading(151.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: swingToHeading hdg 151.2 RIGHT BWD 2000ms
    homeLift();  // 10: homeLift
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(327.96, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 328.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    moveToPointDecel(-20.70, -29.20, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 14: moveToPoint (-20.7, -29.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    clawsClose();  // 16: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 17: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(189.20, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: turnToHeading hdg 189.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    chassis.moveToPoint(-22.90, -41.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: moveToPoint (-22.9, -41.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    liftDropAtLevel(1, 2000);  // 22: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(68.00, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: swingToHeading hdg 68.0 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    homeLift();  // 25: homeLift
    chassis.turnToHeading(236.51, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 236.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    moveToPointDecel(-43.50, -44.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 28: moveToPoint (-43.5, -44.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    clawsClose();  // 30: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 31: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(102.79, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 102.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    moveToPointDecel(-30.80, -47.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 40);  // 34: moveToPoint (-30.8, -47.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    liftDropAtLevel(2, 2000);  // 36: liftToLevel MID DROP (+ open claws)
    chassis.turnToHeading(277.52, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: turnToHeading hdg 277.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    chassis.turnToHeading(44.62, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: turnToHeading hdg 44.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
}
// END planner: last3

// BEGIN planner: last4  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void last4() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(299.40, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 299.4 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    moveToPointDecel(-18.80, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 6: moveToPoint (-18.8, -50.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    chassis.swingToHeading(151.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: swingToHeading hdg 151.2 RIGHT BWD 2000ms
    homeLift();  // 10: homeLift
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(327.96, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 328.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    moveToPointDecel(-20.70, -29.20, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 14: moveToPoint (-20.7, -29.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    clawsClose();  // 16: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 17: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(189.20, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: turnToHeading hdg 189.2 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    chassis.moveToPoint(-22.90, -41.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: moveToPoint (-22.9, -41.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    liftDropAtLevel(1, 2000);  // 22: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(68.00, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: swingToHeading hdg 68.0 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    homeLift();  // 25: homeLift
    chassis.turnToHeading(236.51, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 236.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    moveToPointDecel(-43.50, -44.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 28: moveToPoint (-43.5, -44.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    clawsClose();  // 30: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 31: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(102.79, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 102.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    chassis.turnToHeading(277.52, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 34: turnToHeading hdg 277.5 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.turnToHeading(44.62, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: turnToHeading hdg 44.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    chassis.turnToHeading(97.85, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 39: turnToHeading hdg 97.8 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 40: waitUntilDone
    chassis.moveToPoint(-31.90, -46.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: moveToPoint (-31.9, -46.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    liftDropAtLevel(2, 2000);  // 43: liftToLevel MID DROP (+ open claws)
    homeLift();  // 44: homeLift
}
// END planner: last4

// BEGIN planner: last6  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void last6() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(299.40, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 299.4 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    moveToPointDecel(-18.80, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 6: moveToPoint (-18.8, -50.8) 2000ms
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    pros::delay(800);  // 9: wait 800 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.swingToHeading(151.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: swingToHeading hdg 151.2 RIGHT BWD 2000ms
    homeLift();  // 12: homeLift
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    chassis.turnToHeading(327.96, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: turnToHeading hdg 328.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    moveToPointDecel(-21.30, -28.90, 2000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 20);  // 16: moveToPoint (-21.3, -28.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    pros::delay(500);  // 18: wait 500 ms
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(189.20, 3000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 40, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 189.2 3000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -43.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -43.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    pros::delay(500);  // 25: wait 500 ms
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(75.70, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: swingToHeading hdg 75.7 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.turnToHeading(241.87, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 241.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    moveToPointDecel(-43.80, -45.20, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 31: moveToPoint (-43.8, -45.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    clawsClose();  // 33: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 34: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(100.60, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 40, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 100.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-29.7, -47.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
    pros::delay(500);  // 39: wait 500 ms
    liftDropAtLevel(2, 2000);  // 40: liftToLevel MID DROP (+ open claws)
}
// END planner: last6

// BEGIN planner: test7  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void test7() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(299.40, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 299.4 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    moveToPointDecel(-18.80, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 6: moveToPoint (-18.8, -50.8) 2000ms
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    pros::delay(800);  // 9: wait 800 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.swingToHeading(151.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: swingToHeading hdg 151.2 RIGHT BWD 2000ms
    homeLift();  // 12: homeLift
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    chassis.turnToHeading(327.96, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: turnToHeading hdg 328.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    moveToPointDecel(-21.30, -28.90, 2000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 30);  // 16: moveToPoint (-21.3, -28.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    pros::delay(1000);  // 18: wait 1000 ms
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(189.20, 3000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 40, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 189.2 3000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -43.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -43.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    pros::delay(500);  // 25: wait 500 ms
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(75.70, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: swingToHeading hdg 75.7 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.turnToHeading(241.87, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 241.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    moveToPointDecel(-43.80, -45.20, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 31: moveToPoint (-43.8, -45.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    pros::delay(1000);  // 33: wait 1000 ms
    clawsClose();  // 34: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 35: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(100.60, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 40, .minSpeed = 0, .earlyExitRange = 0.00});  // 36: turnToHeading hdg 100.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 37: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: moveToPoint (-29.7, -47.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    pros::delay(500);  // 40: wait 500 ms
    liftDropAtLevel(2, 2000);  // 41: liftToLevel MID DROP (+ open claws)
}
// END planner: test7

// BEGIN planner: lasttttttttttt  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void lasttttttttttt() {
    chassis.setPose(-0.90, -63.60, 0.00);  // start pose
    homeLift();  // 1: homeLift
    clawsClose();  // 2: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 3: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(299.40, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: swingToHeading hdg 299.4 LEFT 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    moveToPointDecel(-18.80, -50.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 6: moveToPoint (-18.8, -50.8) 2000ms
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    pros::delay(800);  // 9: wait 800 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.swingToHeading(151.20, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: swingToHeading hdg 151.2 RIGHT BWD 2000ms
    homeLift();  // 12: homeLift
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    chassis.turnToHeading(327.96, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: turnToHeading hdg 328.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    moveToPointDecel(-21.30, -28.90, 3000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 30);  // 16: moveToPoint (-21.3, -28.9) 3000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    pros::delay(1000);  // 18: wait 1000 ms
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(189.20, 3000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 189.2 3000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-23.70, -43.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-23.7, -43.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    pros::delay(500);  // 25: wait 500 ms
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(75.70, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: swingToHeading hdg 75.7 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.turnToHeading(241.87, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 241.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    moveToPointDecel(-43.80, -45.20, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 31: moveToPoint (-43.8, -45.2) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    pros::delay(1000);  // 33: wait 1000 ms
    clawsClose();  // 34: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 35: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(100.60, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 40, .minSpeed = 0, .earlyExitRange = 0.00});  // 36: turnToHeading hdg 100.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 37: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: moveToPoint (-29.7, -47.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    pros::delay(500);  // 40: wait 500 ms
    liftDropAtLevel(2, 2000);  // 41: liftToLevel MID DROP (+ open claws)
}
// END planner: lasttttttttttt

// BEGIN planner: debi  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void debi() {
    chassis.setPose(-0.40, -63.90, 0.00);  // start pose
    liftGoToLevel(0, true, 2000);  // 1: liftToLevel GROUND LIFT (wait)
    chassis.swingToHeading(296.90, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: swingToHeading hdg 296.9 LEFT 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.moveToPoint(-18.80, -50.80, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: moveToPoint (-18.8, -50.8) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    pros::delay(500);  // 6: wait 500 ms
    liftGoToLevel(1, true, 2000);  // 7: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 8: wait 500 ms
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    homeLift();  // 10: homeLift
    chassis.turnToHeading(254.86, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 254.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.moveToPoint(-1.80, -46.20, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: moveToPoint (-1.8, -46.2) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    chassis.turnToHeading(313.78, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 313.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    moveToPointDecel(-19.90, -28.30, 1000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 17: moveToPoint (-19.9, -28.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 21: wait 500 ms
    chassis.turnToHeading(193.36, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToHeading hdg 193.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    chassis.moveToPoint(-22.60, -41.40, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: moveToPoint (-22.6, -41.4) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    pros::delay(500);  // 26: wait 500 ms
    liftDropAtLevel(1, 2000);  // 27: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 28: homeLift
    chassis.turnToHeading(311.26, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 311.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-33.20, -32.10, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-33.2, -32.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.turnToHeading(223.72, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: turnToHeading hdg 223.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    moveToPointDecel(-44.10, -43.50, 1000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 35: moveToPoint (-44.1, -43.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    pros::delay(500);  // 37: wait 500 ms
    clawsClose();  // 38: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 39: liftToLevel MID LIFT (wait)
    pros::delay(500);  // 40: wait 500 ms
    chassis.turnToHeading(102.30, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: turnToHeading hdg 102.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 43: moveToPoint (-29.7, -47.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 44: waitUntilDone
    liftDropAtLevel(2, 2000);  // 45: liftToLevel MID DROP (+ open claws)
    homeLift();  // 46: homeLift
    chassis.moveToPoint(-44.90, -43.80, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 47: moveToPoint (-44.9, -43.8) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 48: waitUntilDone
    chassis.turnToHeading(44.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 49: turnToHeading hdg 44.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 50: waitUntilDone
    chassis.moveToPoint(-8.30, -6.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 51: moveToPoint (-8.3, -6.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 52: waitUntilDone
}
// END planner: debi

// BEGIN planner: debi1  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void debi1() {
    chassis.setPose(-0.40, -63.90, 0.00);  // start pose
    liftGoToLevel(1, true, 2000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(296.90, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: swingToHeading hdg 296.9 LEFT 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.moveToPoint(-21.70, -50.00, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: moveToPoint (-21.7, -50.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 6: liftToLevel GROUND LIFT (wait)
    pros::delay(500);  // 7: wait 500 ms
    claw.retract(); claw2.retract();  // 8: claw BOTH OFF
    homeLift();  // 9: homeLift
    chassis.turnToHeading(254.86, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: turnToHeading hdg 254.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.moveToPoint(-1.80, -46.20, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (-1.8, -46.2) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    chassis.turnToHeading(313.78, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: turnToHeading hdg 313.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    moveToPointDecel(-19.50, -28.80, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 16: moveToPoint (-19.5, -28.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    clawsClose();  // 18: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 19: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 20: wait 500 ms
    chassis.turnToHeading(193.36, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 193.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-22.60, -41.40, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-22.6, -41.4) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    pros::delay(500);  // 25: wait 500 ms
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 27: homeLift
    chassis.turnToHeading(311.26, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: turnToHeading hdg 311.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    chassis.moveToPoint(-33.20, -32.10, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 30: moveToPoint (-33.2, -32.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    chassis.turnToHeading(223.72, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 223.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    moveToPointDecel(-43.50, -42.90, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00}, 30.00, 40);  // 34: moveToPoint (-43.5, -42.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    pros::delay(500);  // 36: wait 500 ms
    clawsClose();  // 37: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 38: liftToLevel MID LIFT (wait)
    pros::delay(500);  // 39: wait 500 ms
    chassis.turnToHeading(102.30, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: turnToHeading hdg 102.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 42: moveToPoint (-29.7, -47.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 43: waitUntilDone
    liftDropAtLevel(2, 2000);  // 44: liftToLevel MID DROP (+ open claws)
    homeLift();  // 45: homeLift
    moveToPointDecel(-44.70, -44.00, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 30.00, 40);  // 46: moveToPoint (-44.7, -44.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 47: waitUntilDone
    chassis.turnToHeading(44.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: turnToHeading hdg 44.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.moveToPoint(-8.30, -6.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: moveToPoint (-8.3, -6.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
}
// END planner: debi1

// BEGIN planner: debi3  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void debi3() {
    chassis.setPose(-0.40, -63.90, 0.00);  // start pose
    clawsClose();  // 1: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 2: liftToLevel LOW LIFT (wait)
    chassis.swingToHeading(296.90, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: swingToHeading hdg 296.9 LEFT 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.moveToPoint(-19.50, -50.50, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-19.5, -50.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    pros::delay(500);  // 8: wait 500 ms
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    homeLift();  // 10: homeLift
    chassis.turnToHeading(254.86, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 254.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.moveToPoint(-3.00, -45.00, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: moveToPoint (-3.0, -45.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    chassis.turnToHeading(313.78, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 313.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    moveToPointDecel(-19.50, -28.50, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 17: moveToPoint (-19.5, -28.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 21: wait 500 ms
    chassis.turnToHeading(193.36, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToHeading hdg 193.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    chassis.moveToPoint(-22.60, -41.40, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: moveToPoint (-22.6, -41.4) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    pros::delay(500);  // 26: wait 500 ms
    liftDropAtLevel(1, 2000);  // 27: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 28: homeLift
    chassis.turnToHeading(311.26, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 311.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-33.20, -32.10, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-33.2, -32.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.turnToHeading(223.72, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: turnToHeading hdg 223.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    moveToPointDecel(-43.70, -42.90, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00}, 30.00, 40);  // 35: moveToPoint (-43.7, -42.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    pros::delay(500);  // 37: wait 500 ms
    clawsClose();  // 38: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 39: liftToLevel MID LIFT (wait)
    pros::delay(500);  // 40: wait 500 ms
    chassis.turnToHeading(102.30, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: turnToHeading hdg 102.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 43: moveToPoint (-29.7, -47.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 44: waitUntilDone
    liftDropAtLevel(2, 2000);  // 45: liftToLevel MID DROP (+ open claws)
    homeLift();  // 46: homeLift
    moveToPointDecel(-44.70, -44.00, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 30.00, 40);  // 47: moveToPoint (-44.7, -44.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 48: waitUntilDone
    chassis.turnToHeading(44.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 49: turnToHeading hdg 44.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 50: waitUntilDone
    chassis.moveToPoint(-8.30, -6.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 51: moveToPoint (-8.3, -6.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 52: waitUntilDone
}
// END planner: debi3

// BEGIN planner: mika  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void mika() {
    chassis.setPose(-0.40, -63.90, 0.00);  // start pose
    clawsClose();  // 1: claw BOTH ON
    chassis.swingToHeading(296.90, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: swingToHeading hdg 296.9 LEFT 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    liftGoToLevel(1, true, 2000);  // 4: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(-19.50, -50.50, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-19.5, -50.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    pros::delay(500);  // 8: wait 500 ms
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    homeLift();  // 10: homeLift
    chassis.turnToHeading(254.86, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 254.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.moveToPoint(-3.00, -45.00, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: moveToPoint (-3.0, -45.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    chassis.turnToHeading(313.78, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 313.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    moveToPointDecel(-19.50, -28.50, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 17: moveToPoint (-19.5, -28.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 21: wait 500 ms
    chassis.turnToHeading(193.36, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToHeading hdg 193.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    chassis.moveToPoint(-22.60, -41.40, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: moveToPoint (-22.6, -41.4) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    pros::delay(500);  // 26: wait 500 ms
    liftDropAtLevel(1, 2000);  // 27: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 28: homeLift
    chassis.turnToHeading(311.26, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 311.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-33.20, -32.10, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-33.2, -32.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.turnToHeading(223.72, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: turnToHeading hdg 223.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    moveToPointDecel(-43.80, -43.00, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 35: moveToPoint (-43.8, -43.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    pros::delay(500);  // 37: wait 500 ms
    clawsClose();  // 38: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 39: liftToLevel MID LIFT (wait)
    pros::delay(500);  // 40: wait 500 ms
    chassis.turnToHeading(102.30, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: turnToHeading hdg 102.3 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    chassis.moveToPoint(-29.70, -47.30, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 43: moveToPoint (-29.7, -47.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 44: waitUntilDone
    liftDropAtLevel(2, 2000);  // 45: liftToLevel MID DROP (+ open claws)
    homeLift();  // 46: homeLift
    moveToPointDecel(-44.70, -44.00, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 30.00, 40);  // 47: moveToPoint (-44.7, -44.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 48: waitUntilDone
    chassis.turnToHeading(44.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 49: turnToHeading hdg 44.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 50: waitUntilDone
    chassis.moveToPoint(-8.30, -6.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 51: moveToPoint (-8.3, -6.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 52: waitUntilDone
}
// END planner: mika

// BEGIN planner: ner  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void ner() {
    chassis.setPose(-0.80, -61.60, 300.30);  // start pose
    liftGoToLevel(0, true, 2000);  // 1: liftToLevel GROUND LIFT (wait)
    pros::delay(250);  // 2: wait 250 ms
    chassis.moveToPoint(-19.70, -50.80, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: moveToPoint (-19.7, -50.8) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    pros::delay(250);  // 5: wait 250 ms
    claw.retract(); claw2.retract();  // 6: claw BOTH OFF
    homeLift();  // 7: homeLift
    chassis.turnToHeading(241.62, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 8: turnToHeading hdg 241.6 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    chassis.moveToPoint(-2.30, -41.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: moveToPoint (-2.3, -41.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.turnToHeading(309.02, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: turnToHeading hdg 309.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    moveToPointDecel(-19.20, -27.80, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 40);  // 14: moveToPoint (-19.2, -27.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    clawsClose();  // 16: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 17: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 18: wait 500 ms
    chassis.turnToHeading(197.71, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: turnToHeading hdg 197.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 20: waitUntilDone
    chassis.moveToPoint(-22.20, -41.90, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: moveToPoint (-22.2, -41.9) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    liftDropAtLevel(1, 2000);  // 23: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 24: homeLift
    chassis.turnToHeading(117.10, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: turnToHeading hdg 117.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 26: waitUntilDone
    chassis.moveToPoint(-34.90, -35.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: moveToPoint (-34.9, -35.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.turnToHeading(228.73, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 228.7 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    moveToPointDecel(-44.00, -43.70, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 20.00, 40);  // 31: moveToPoint (-44.0, -43.7) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    clawsClose();  // 33: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 34: liftToLevel MID LIFT (wait)
    pros::delay(500);  // 35: wait 500 ms
    chassis.turnToHeading(104.57, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 36: turnToHeading hdg 104.6 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 37: waitUntilDone
    chassis.moveToPoint(-31.10, -46.70, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: moveToPoint (-31.1, -46.7) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    liftDropAtLevel(2, 2000);  // 40: liftToLevel MID DROP (+ open claws)
    homeLift();  // 41: homeLift
    chassis.turnToHeading(49.49, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 42: turnToHeading hdg 49.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 43: waitUntilDone
    chassis.moveToPoint(-42.90, -60.60, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: moveToPoint (-42.9, -60.6) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    chassis.turnToHeading(91.24, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 46: turnToHeading hdg 91.2 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 47: waitUntilDone
    chassis.moveToPoint(0.00, -60.60, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: moveToPoint (0.0, -60.6) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.turnToHeading(358.90, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: turnToHeading hdg 358.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
    chassis.moveToPoint(0.00, -63.70, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 52: moveToPoint (0.0, -63.7) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 53: waitUntilDone
    liftGoToLevel(1, true, 2000);  // 54: liftToLevel LOW LIFT (wait)
    homeLift();  // 55: homeLift
}
// END planner: ner

// BEGIN planner: miserable  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void miserable() {
    chassis.setPose(0.40, -29.30, 180.00);  // start pose
    liftGoToLevel(0, true, 2000);  // 3: liftToLevel GROUND LIFT (wait)
    chassis.moveToPoint(0.40, -57.90, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 1: moveToPoint (0.4, -57.9) BWD 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 2: waitUntilDone
    chassis.turnToHeading(292.12, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 292.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.30, -50.30, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.3, -50.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    pros::delay(500);  // 8: wait 500 ms
    claw.retract(); claw2.retract();  // 9: claw BOTH OFF
    chassis.turnToHeading(241.07, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: turnToHeading hdg 241.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.moveToPoint(-3.10, -41.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (-3.1, -41.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    chassis.turnToHeading(312.14, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 312.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    moveToPointDecel(-18.30, -28.60, 1000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 40);  // 17: moveToPoint (-18.3, -28.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 21: wait 500 ms
    chassis.turnToHeading(196.79, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToHeading hdg 196.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    chassis.moveToPoint(-22.40, -41.60, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: moveToPoint (-22.4, -41.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 27: homeLift
    chassis.turnToHeading(122.56, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: turnToHeading hdg 122.6 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    chassis.moveToPoint(-34.30, -34.00, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 30: moveToPoint (-34.3, -34.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    chassis.turnToHeading(223.72, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 223.7 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    moveToPointDecel(-44.10, -43.00, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 40);  // 34: moveToPoint (-44.1, -43.0) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    clawsClose();  // 36: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 37: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(107.13, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: turnToHeading hdg 107.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    chassis.moveToPoint(-30.20, -46.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: moveToPoint (-30.2, -46.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    liftDropAtLevel(2, 2000);  // 42: liftToLevel MID DROP (+ open claws)
    homeLift();  // 43: homeLift
    chassis.turnToHeading(51.46, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: turnToHeading hdg 51.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    chassis.moveToPoint(-47.90, -59.30, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 46: moveToPoint (-47.9, -59.3) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 47: waitUntilDone
    chassis.turnToHeading(90.00, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: turnToHeading hdg 90.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.moveToPoint(0.70, -59.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: moveToPoint (0.7, -59.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
    chassis.turnToHeading(359.90, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 52: turnToHeading hdg 359.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 53: waitUntilDone
    chassis.moveToPoint(0.70, -64.40, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 54: moveToPoint (0.7, -64.4) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 55: waitUntilDone
    liftGoToLevel(1, true, 2000);  // 56: liftToLevel LOW LIFT (wait)
    homeLift();  // 57: homeLift
    chassis.moveToPoint(0.10, -9.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 58: moveToPoint (0.1, -9.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 59: waitUntilDone
}
// END planner: miserable

// BEGIN planner: dhaslkdsklkjlads  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void dhaslkdsklkjlads() {
    chassis.setPose(0.40, -63.30, 180.00);  // start pose
    liftGoToLevel(1, true, 2000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(0.40, -57.90, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (0.4, -57.9) BWD 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(292.12, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 292.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.30, -50.30, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.3, -50.3) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    pros::delay(500);  // 8: wait 500 ms
    liftDropAtLevel(1, 2000);  // 9: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(241.07, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 10: turnToHeading hdg 241.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    chassis.moveToPoint(-3.10, -41.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (-3.1, -41.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    chassis.turnToHeading(312.14, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 312.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    moveToPointDecel(-18.30, -28.60, 1000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 40);  // 17: moveToPoint (-18.3, -28.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    clawsClose();  // 19: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 20: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 21: wait 500 ms
    chassis.turnToHeading(196.79, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToHeading hdg 196.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    chassis.moveToPoint(-22.40, -41.60, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: moveToPoint (-22.4, -41.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    liftDropAtLevel(1, 2000);  // 26: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 27: homeLift
    chassis.turnToHeading(122.56, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: turnToHeading hdg 122.6 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    chassis.moveToPoint(-34.30, -34.00, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 30: moveToPoint (-34.3, -34.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    chassis.turnToHeading(223.72, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToHeading hdg 223.7 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    moveToPointDecel(-42.70, -42.40, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 0.00}, 15.00, 40);  // 34: moveToPoint (-42.7, -42.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    clawsClose();  // 36: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 37: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(107.13, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 38: turnToHeading hdg 107.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 39: waitUntilDone
    chassis.moveToPoint(-30.20, -46.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 40: moveToPoint (-30.2, -46.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 41: waitUntilDone
    liftDropAtLevel(2, 2000);  // 42: liftToLevel MID DROP (+ open claws)
    homeLift();  // 43: homeLift
    chassis.turnToHeading(51.46, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 44: turnToHeading hdg 51.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 45: waitUntilDone
    chassis.moveToPoint(-47.10, -60.60, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 46: moveToPoint (-47.1, -60.6) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 47: waitUntilDone
    chassis.turnToHeading(90.00, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 48: turnToHeading hdg 90.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 49: waitUntilDone
    chassis.moveToPoint(0.70, -59.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 50: moveToPoint (0.7, -59.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 51: waitUntilDone
    chassis.turnToHeading(359.90, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 52: turnToHeading hdg 359.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 53: waitUntilDone
    chassis.moveToPoint(0.70, -64.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 54: moveToPoint (0.7, -64.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 55: waitUntilDone
    liftGoToLevel(1, true, 2000);  // 56: liftToLevel LOW LIFT (wait)
    homeLift();  // 57: homeLift
    chassis.moveToPoint(0.10, -9.10, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 58: moveToPoint (0.1, -9.1) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 59: waitUntilDone
}
// END planner: dhaslkdsklkjlads

// BEGIN planner: debilastlast  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void debilastlast() {
    chassis.setPose(0.40, -63.30, 180.00);  // start pose
    liftGoToLevel(1, true, 1000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(0.40, -57.90, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (0.4, -57.9) BWD 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(292.12, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 292.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-19.40, -50.00, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-19.4, -50.0) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 8: liftToLevel GROUND LIFT (wait)
    pros::delay(500);  // 9: wait 500 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.turnToHeading(241.07, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: turnToHeading hdg 241.1 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    chassis.moveToPoint(-3.10, -41.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 13: moveToPoint (-3.1, -41.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 14: waitUntilDone
    homeLift();  // 15: homeLift
    chassis.turnToHeading(312.14, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 16: turnToHeading hdg 312.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    moveToPointDecel(-18.30, -28.60, 1000, {.forwards = true, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 2.00}, 15.00, 40);  // 18: moveToPoint (-18.3, -28.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    clawsClose();  // 20: claw BOTH ON
    liftGoToLevel(1, true, 2000);  // 21: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 22: wait 500 ms
    chassis.turnToHeading(196.79, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: turnToHeading hdg 196.8 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    chassis.moveToPoint(-22.40, -41.60, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: moveToPoint (-22.4, -41.6) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 26: waitUntilDone
    liftDropAtLevel(1, 2000);  // 27: liftToLevel LOW DROP (+ open claws)
    homeLift();  // 28: homeLift
    chassis.turnToHeading(122.56, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: turnToHeading hdg 122.6 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.moveToPoint(-34.30, -34.00, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: moveToPoint (-34.3, -34.0) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.turnToHeading(223.72, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: turnToHeading hdg 223.7 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    moveToPointDecel(-43.50, -42.70, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 2.00}, 15.00, 40);  // 35: moveToPoint (-43.5, -42.7) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    clawsClose();  // 37: claw BOTH ON
    liftGoToLevel(2, true, 2000);  // 38: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(107.13, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 39: turnToHeading hdg 107.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 40: waitUntilDone
    chassis.moveToPoint(-30.20, -46.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 41: moveToPoint (-30.2, -46.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 42: waitUntilDone
    liftDropAtLevel(2, 2000);  // 43: liftToLevel MID DROP (+ open claws)
    homeLift();  // 44: homeLift
    chassis.turnToHeading(51.46, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 45: turnToHeading hdg 51.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 46: waitUntilDone
    chassis.moveToPoint(-47.10, -60.60, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 47: moveToPoint (-47.1, -60.6) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 48: waitUntilDone
    chassis.turnToHeading(90.00, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 49: turnToHeading hdg 90.0 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 50: waitUntilDone
    chassis.moveToPoint(0.70, -59.30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 51: moveToPoint (0.7, -59.3) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 52: waitUntilDone
    chassis.turnToHeading(359.90, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 53: turnToHeading hdg 359.9 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 54: waitUntilDone
    chassis.moveToPoint(0.70, -64.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 55: moveToPoint (0.7, -64.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 56: waitUntilDone
    liftGoToLevel(1, true, 2000);  // 57: liftToLevel LOW LIFT (wait)
    homeLift();  // 58: homeLift
    chassis.moveToPoint(0.10, -9.10, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 59: moveToPoint (0.1, -9.1) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 60: waitUntilDone
}
// END planner: debilastlast

// BEGIN planner: baisliw  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void baisliw() {
    chassis.setPose(-0.10, -63.90, 359.00);  // start pose
    liftGoToLevel(0, true, 800);  // 1: liftToLevel GROUND LIFT (wait)
    pros::delay(300);  // 2: wait 300 ms
    liftGoToLevel(5, true, 1000);  // 3: liftToLevel PIN LIFT (wait)
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.moveToPoint(-0.40, -48.70, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-0.4, -48.7) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    liftGoToLevel(0, true, 2000);  // 7: liftToLevel GROUND LIFT (wait)
    chassis.turnToHeading(270.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 8: turnToHeading hdg 270.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    moveToPointDecel(-17.80, -48.10, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00}, 10.00, 40);  // 10: moveToPoint (-17.8, -48.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 11: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 12: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 13: wait 500 ms
    claw.retract(); claw2.retract();  // 14: claw BOTH OFF
}
// END planner: baisliw

// BEGIN planner: baliww  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void baliww() {
    chassis.setPose(-0.10, -63.90, 359.00);  // start pose
    homeLift();  // 1: homeLift
    liftGoToLevel(0, true, 800);  // 2: liftToLevel GROUND LIFT (wait)
    pros::delay(300);  // 3: wait 300 ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.moveToPoint(-0.40, -48.70, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: moveToPoint (-0.4, -48.7) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.turnToHeading(270.00, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: turnToHeading hdg 270.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    chassis.moveToPoint(-17.80, -48.10, 1000, {.forwards = true, .maxSpeed = 100, .minSpeed = 0, .earlyExitRange = 0.00});  // 9: moveToPoint (-17.8, -48.1) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 10: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 11: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 12: wait 500 ms
    claw.retract(); claw2.retract();  // 13: claw BOTH OFF
    chassis.moveToPoint(-0.40, -48.10, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 14: moveToPoint (-0.4, -48.1) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 15: waitUntilDone
    homeLift();  // 16: homeLift
    chassis.turnToHeading(317.45, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 317.4 1000ms
    goToPin(-24.00, -24.00, 5, 3000, 127, true, 7.50, 10.00, 40);  // 18: goToPin (-24.0, -24.0) lift PIN 3000ms
}
// END planner: baliww

// BEGIN planner: withdroppin  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void withdroppin() {
    chassis.setPose(-0.40, -63.90, 0.00);  // start pose
    liftGoToLevel(0, true, 2000);  // 1: liftToLevel GROUND LIFT (wait)
    chassis.moveToPoint(-0.10, -48.70, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (-0.1, -48.7) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(272.10, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 272.1 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.00, -48.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.0, -48.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 8: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 9: wait 500 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.moveToPoint(0.40, -48.70, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (0.4, -48.7) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    goToPin(-24.00, -24.00, 5, 3000, 80, true, 7.50, 40.00, 40);  // 13: goToPin (-24.0, -24.0) lift PIN 3000ms
    pros::delay(500);  // 14: wait 500 ms
    liftGoToLevel(1, true, 2000);  // 15: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(175.58, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 16: turnToHeading hdg 175.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    chassis.moveToPoint(-23.20, -41.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: moveToPoint (-23.2, -41.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    liftDropAtLevel(1, 2000);  // 20: liftToLevel LOW DROP (+ open claws)
    chassis.swingToHeading(29.10, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: swingToHeading hdg 29.1 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    homeLift();  // 23: homeLift
    goToPin(-48.00, -48.00, 5, 3000, 80, true, 7.50, 10.00, 40);  // 24: goToPin (-48.0, -48.0) lift PIN 3000ms
    pros::delay(500);  // 25: wait 500 ms
    liftGoToLevel(2, true, 2000);  // 26: liftToLevel MID LIFT (wait)
    chassis.turnToHeading(108.90, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: turnToHeading hdg 108.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.moveToPoint(-30.80, -46.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: moveToPoint (-30.8, -46.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    liftDropAtLevel(2, 2000);  // 31: liftToLevel MID DROP (+ open claws)
}
// END planner: withdroppin

// BEGIN planner: nine  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void nine() {
    chassis.setPose(0.10, -64.10, 0.00);  // start pose
    liftGoToLevel(1, true, 2000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(0.10, -48.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (0.1, -48.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(271.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 271.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-17.50, -48.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-17.5, -48.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 8: liftToLevel PIN LIFT (wait)
    pros::delay(250);  // 9: wait 250 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.moveToPoint(0.40, -48.70, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (0.4, -48.7) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    claw.retract(); claw2.retract();  // 13: claw BOTH OFF
    homeLift();  // 14: homeLift
    goToPin(-24.00, -24.00, 5, 3000, 127, true, 7.50, 10.00, 40);  // 15: goToPin (-24.0, -24.0) lift PIN 3000ms
    liftGoToLevel(1, true, 2000);  // 16: liftToLevel LOW LIFT (wait)
    chassis.turnToHeading(194.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToHeading hdg 194.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    chassis.moveToPoint(-22.10, -41.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: moveToPoint (-22.1, -41.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 20: waitUntilDone
    liftDropAtLevel(1, 2000);  // 21: liftToLevel LOW DROP (+ open claws)
}
// END planner: nine

// BEGIN planner: bastaetonayon  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void bastaetonayon() {
    chassis.setPose(0.10, -64.10, 0.00);  // start pose
    liftGoToLevel(1, true, 800);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(-0.10, -48.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (-0.1, -48.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(271.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 271.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.30, -47.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.3, -47.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 8: liftToLevel PIN LIFT (wait)
    pros::delay(250);  // 9: wait 250 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.moveToPoint(-1.80, -34.00, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (-1.8, -34.0) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    claw.retract(); claw2.retract();  // 13: claw BOTH OFF
    homeLift();  // 14: homeLift
    goToPin(-24.00, -24.00, 1, 3000, 100, true, 5.00, 10.00, 40);  // 15: goToPin (-24.0, -24.0) lift LOW 3000ms
    chassis.turnToHeading(194.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 16: turnToHeading hdg 194.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    chassis.moveToPoint(-23.20, -40.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: moveToPoint (-23.2, -40.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    liftDropAtLevel(1, 2000);  // 20: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(113.88, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 113.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-35.40, -35.40, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-35.4, -35.4) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    homeLift();  // 25: homeLift
    goToPin(-48.00, -48.00, 5, 3000, 100, true, 7.50, 10.00, 40);  // 26: goToPin (-48.0, -48.0) lift PIN 3000ms
    chassis.turnToHeading(107.57, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: turnToHeading hdg 107.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.moveToPoint(-30.50, -46.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: moveToPoint (-30.5, -46.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    liftDropAtLevel(2, 2000);  // 31: liftToLevel MID DROP (+ open claws)
}
// END planner: bastaetonayon

// BEGIN planner: controller_auto  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void controller_auto() {
    chassis.setPose(0.10, -64.10, 0.00);  // start pose
    liftGoToLevel(1, true, 800);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(-0.10, -48.10, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (-0.1, -48.1) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToHeading(271.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToHeading hdg 271.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.30, -47.90, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.3, -47.9) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 8: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 9: wait 500 ms
    claw.retract(); claw2.retract();  // 10: claw BOTH OFF
    chassis.moveToPoint(-1.80, -34.00, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (-1.8, -34.0) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    homeLift();  // 13: homeLift
    claw.retract(); claw2.retract();  // 14: claw BOTH OFF
    goToPin(-24.00, -24.00, 1, 3000, 100, true, 7.00, 20.00, 20);  // 15: goToPin (-24.0, -24.0) lift LOW 3000ms
    chassis.turnToHeading(194.56, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 16: turnToHeading hdg 194.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    chassis.moveToPoint(-23.20, -40.80, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: moveToPoint (-23.2, -40.8) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    liftDropAtLevel(1, 2000);  // 20: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(113.88, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToHeading hdg 113.9 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    chassis.moveToPoint(-35.40, -35.40, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 23: moveToPoint (-35.4, -35.4) BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 24: waitUntilDone
    homeLift();  // 25: homeLift
    goToPin(-47.60, -48.90, 2, 3000, 100, true, 7.00, 15.00, 20);  // 26: goToPin (-47.6, -48.9) lift MID 3000ms
    chassis.turnToHeading(107.57, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 27: turnToHeading hdg 107.6 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 28: waitUntilDone
    chassis.moveToPoint(-30.50, -46.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: moveToPoint (-30.5, -46.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    liftDropAtLevel(2, 2000);  // 31: liftToLevel MID DROP (+ open claws)
}
// END planner: controller_auto

// BEGIN planner: finalllllllllllllllllll  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void finalllllllllllllllllll() {
    chassis.setPose(-0.10, -63.90, 0.00);  // start pose
    liftGoToLevel(1, true, 500);  // 1: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 2: wait 500 ms
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: moveToPoint (0.0, -47.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.turnToHeading(268.43, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: turnToHeading hdg 268.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.moveToPoint(-18.40, -47.70, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-18.4, -47.7) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 9: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 10: wait 500 ms
    claw.retract(); claw2.retract();  // 11: claw BOTH OFF
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (0.0, -47.5) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    goToPin(-24.00, -24.00, 1, 1500, 127, true, 6.00, 15.00, 30);  // 14: goToPin (-24.0, -24.0) lift LOW 1500ms
    chassis.turnToHeading(201.45, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 201.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.moveToPoint(-22.90, -41.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: moveToPoint (-22.9, -41.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    liftDropAtLevel(1, 2000);  // 19: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(120.96, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: turnToHeading hdg 121.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    chassis.moveToPoint(-34.90, -33.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: moveToPoint (-34.9, -33.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    homeLift();  // 24: homeLift
    goToPin(-47.60, -48.90, 2, 3000, 100, true, 7.00, 7.00, 30);  // 25: goToPin (-47.6, -48.9) lift MID 3000ms
    chassis.turnToHeading(106.49, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 106.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    chassis.moveToPoint(-29.60, -47.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: moveToPoint (-29.6, -47.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    liftDropAtLevel(2, 2000);  // 30: liftToLevel MID DROP (+ open claws)
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    homeLift();  // 32: homeLift
    chassis.moveToPoint(-44.00, -42.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: moveToPoint (-44.0, -42.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    chassis.turnToHeading(44.26, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 44.3 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-7.70, -6.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-7.7, -6.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
}
// END planner: finalllllllllllllllllll

// BEGIN planner: upgrade  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void upgrade() {
    chassis.setPose(0.00, -64.30, 0.00);  // start pose
    liftGoToLevel(1, true, 2000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(-6.20, -48.00, 1250, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (-6.2, -48.0) 1250ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1150, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToPoint (-24.0, -48.0) 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-18.80, -48.00, 1150, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-18.8, -48.0) 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 500, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 8: turnToPoint (-24.0, -48.0) 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 10: liftToLevel PIN LIFT (wait)
    claw.retract(); claw2.retract();  // 11: claw BOTH OFF
    chassis.moveToPoint(-4.20, -47.70, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (-4.2, -47.7) BWD 1200ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    homeLift();  // 14: homeLift
    goToPin(-24.00, -24.00, 2, 2150, 127, true, 7.50, 8.00, 30);  // 15: goToPin (-24.0, -24.0) lift MID 2150ms
    chassis.moveToPoint(-24.00, -30.20, 850, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 16: moveToPoint (-24.0, -30.2) 850ms
    while (chassis.isInMotion()) pros::delay(10);  // 17: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1300, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 18: turnToPoint (-24.0, -48.0) 1300ms
    while (chassis.isInMotion()) pros::delay(10);  // 19: waitUntilDone
    chassis.moveToPoint(-24.00, -42.80, 1200, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: moveToPoint (-24.0, -42.8) 1200ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 900, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: turnToPoint (-24.0, -48.0) 900ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    shootToGoal(2000);  // 24: shootToGoal (drop at current level)
    chassis.swingToHeading(61.70, lemlib::DriveSide::RIGHT, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 25: swingToHeading hdg 61.7 RIGHT BWD 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 26: waitUntilDone
    homeLift();  // 27: homeLift
    goToPin(-48.00, -48.00, 2, 2200, 127, true, 7.50, 6.00, 30);  // 28: goToPin (-48.0, -48.0) lift MID 2200ms
    chassis.moveToPoint(-41.80, -48.00, 800, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 29: moveToPoint (-41.8, -48.0) 800ms
    while (chassis.isInMotion()) pros::delay(10);  // 30: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1300, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 31: turnToPoint (-24.0, -48.0) 1300ms
    while (chassis.isInMotion()) pros::delay(10);  // 32: waitUntilDone
    chassis.moveToPoint(-31.20, -48.00, 1150, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: moveToPoint (-31.2, -48.0) 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 900, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToPoint (-24.0, -48.0) 900ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    shootToGoal(2000);  // 37: shootToGoal (drop at current level)
}
// END planner: upgrade

// BEGIN planner: savesave  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void savesave() {
    chassis.setPose(0.00, -63.90, 0.00);  // start pose
    liftGoToLevel(1, true, 2000);  // 1: liftToLevel LOW LIFT (wait)
    chassis.moveToPoint(-6.20, -48.00, 1250, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 2: moveToPoint (-6.2, -48.0) 1250ms
    while (chassis.isInMotion()) pros::delay(10);  // 3: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1150, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 4: turnToPoint (-24.0, -48.0) 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 5: waitUntilDone
    chassis.moveToPoint(-19.80, -48.00, 1150, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 6: moveToPoint (-19.8, -48.0) 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 7: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 500, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 8: turnToPoint (-24.0, -48.0) 500ms
    while (chassis.isInMotion()) pros::delay(10);  // 9: waitUntilDone
    shootToGoal(2000);  // 10: shootToGoal (drop at current level)
    chassis.moveToPoint(-2.30, -47.40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 11: moveToPoint (-2.3, -47.4) BWD 1200ms
    while (chassis.isInMotion()) pros::delay(10);  // 12: waitUntilDone
    homeLift();  // 13: homeLift
    goToPin(-24.00, -24.00, 1, 2600, 127, true, 7.50, 15.00, 30);  // 14: goToPin (-24.0, -24.0) lift LOW 2600ms
    chassis.moveToPoint(-24.00, -30.20, 900, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: moveToPoint (-24.0, -30.2) 900ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1300, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: turnToPoint (-24.0, -48.0) 1300ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    chassis.moveToPoint(-24.00, -43.80, 1200, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 19: moveToPoint (-24.0, -43.8) 1200ms
    while (chassis.isInMotion()) pros::delay(10);  // 20: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 850, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 21: turnToPoint (-24.0, -48.0) 850ms
    while (chassis.isInMotion()) pros::delay(10);  // 22: waitUntilDone
    shootToGoal(2000);  // 23: shootToGoal (drop at current level)
    chassis.turnToHeading(116.29, 1150, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 24: turnToHeading hdg 116.3 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 25: waitUntilDone
    chassis.moveToPoint(-34.30, -35.40, 1150, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: moveToPoint (-34.3, -35.4) BWD 1150ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    homeLift();  // 28: homeLift
    goToPin(-48.00, -48.00, 1, 2050, 127, true, 7.50, 6.00, 30);  // 29: goToPin (-48.0, -48.0) lift LOW 2050ms
    chassis.moveToPoint(-41.80, -48.00, 900, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 30: moveToPoint (-41.8, -48.0) 900ms
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 1300, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 32: turnToPoint (-24.0, -48.0) 1300ms
    while (chassis.isInMotion()) pros::delay(10);  // 33: waitUntilDone
    chassis.moveToPoint(-28.20, -48.00, 1200, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 34: moveToPoint (-28.2, -48.0) 1200ms
    while (chassis.isInMotion()) pros::delay(10);  // 35: waitUntilDone
    chassis.turnToPoint(-24.00, -48.00, 850, {.forwards = true, .direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 36: turnToPoint (-24.0, -48.0) 850ms
    while (chassis.isInMotion()) pros::delay(10);  // 37: waitUntilDone
    shootToGoal(2000);  // 38: shootToGoal (drop at current level)
}
// END planner: savesave

// BEGIN planner: workinggg  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void workinggg() {
    chassis.setPose(0.00, -63.90, 0.00);  // start pose
    liftGoToLevel(1, true, 500);  // 1: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 2: wait 500 ms
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: moveToPoint (0.0, -47.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.turnToHeading(268.43, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: turnToHeading hdg 268.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.moveToPoint(-18.40, -47.70, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-18.4, -47.7) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 9: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 10: wait 500 ms
    claw.retract(); claw2.retract();  // 11: claw BOTH OFF
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (0.0, -47.5) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    goToPin(-24.00, -24.00, 1, 3450, 80, true, 6.00, 15.00, 20);  // 14: goToPin (-24.0, -24.0) lift LOW 3450ms
    chassis.turnToHeading(201.45, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 201.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.moveToPoint(-22.90, -41.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: moveToPoint (-22.9, -41.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    liftDropAtLevel(1, 2000);  // 19: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(120.96, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: turnToHeading hdg 121.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    chassis.moveToPoint(-34.90, -33.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: moveToPoint (-34.9, -33.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    homeLift();  // 24: homeLift
    goToPin(-48.00, -48.00, 2, 2300, 70, true, 7.00, 7.00, 30);  // 25: goToPin (-48.0, -48.0) lift MID 2300ms
    chassis.turnToHeading(106.49, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 106.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    chassis.moveToPoint(-29.60, -47.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: moveToPoint (-29.6, -47.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    liftDropAtLevel(2, 2000);  // 30: liftToLevel MID DROP (+ open claws)
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    homeLift();  // 32: homeLift
    chassis.moveToPoint(-44.00, -42.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: moveToPoint (-44.0, -42.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    chassis.turnToHeading(44.26, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 44.3 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-7.70, -6.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-7.7, -6.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
}
// END planner: workinggg

// BEGIN planner: workingslow  (tools/path_planner.py -- saving this plan again replaces everything up to END)
// Coordinates: inches, origin at field centre, heading 0 = +y, clockwise +.
void workingslow() {
    chassis.setPose(0.00, -63.90, 0.00);  // start pose
    liftGoToLevel(1, true, 500);  // 1: liftToLevel LOW LIFT (wait)
    pros::delay(500);  // 2: wait 500 ms
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 3: moveToPoint (0.0, -47.5) 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 4: waitUntilDone
    chassis.turnToHeading(268.43, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 5: turnToHeading hdg 268.4 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 6: waitUntilDone
    chassis.moveToPoint(-18.40, -47.70, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 7: moveToPoint (-18.4, -47.7) 1500ms
    while (chassis.isInMotion()) pros::delay(10);  // 8: waitUntilDone
    liftGoToLevel(5, true, 2000);  // 9: liftToLevel PIN LIFT (wait)
    pros::delay(500);  // 10: wait 500 ms
    claw.retract(); claw2.retract();  // 11: claw BOTH OFF
    chassis.moveToPoint(0.00, -47.50, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 12: moveToPoint (0.0, -47.5) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 13: waitUntilDone
    goToPin(-24.00, -24.00, 1, 3550, 50, true, 6.00, 10.00, 20);  // 14: goToPin (-24.0, -24.0) lift LOW 3550ms
    chassis.turnToHeading(201.45, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 15: turnToHeading hdg 201.4 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 16: waitUntilDone
    chassis.moveToPoint(-22.90, -41.40, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 17: moveToPoint (-22.9, -41.4) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 18: waitUntilDone
    liftDropAtLevel(1, 2000);  // 19: liftToLevel LOW DROP (+ open claws)
    chassis.turnToHeading(120.96, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 20: turnToHeading hdg 121.0 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 21: waitUntilDone
    chassis.moveToPoint(-34.90, -33.90, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 22: moveToPoint (-34.9, -33.9) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 23: waitUntilDone
    homeLift();  // 24: homeLift
    goToPin(-48.00, -48.00, 2, 2300, 70, true, 7.00, 7.00, 30);  // 25: goToPin (-48.0, -48.0) lift MID 2300ms
    chassis.turnToHeading(106.49, 1000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 26: turnToHeading hdg 106.5 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 27: waitUntilDone
    chassis.moveToPoint(-29.60, -47.50, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 28: moveToPoint (-29.6, -47.5) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 29: waitUntilDone
    liftDropAtLevel(2, 2000);  // 30: liftToLevel MID DROP (+ open claws)
    while (chassis.isInMotion()) pros::delay(10);  // 31: waitUntilDone
    homeLift();  // 32: homeLift
    chassis.moveToPoint(-44.00, -42.40, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 33: moveToPoint (-44.0, -42.4) BWD 1000ms
    while (chassis.isInMotion()) pros::delay(10);  // 34: waitUntilDone
    chassis.turnToHeading(44.26, 2000, {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 35: turnToHeading hdg 44.3 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 36: waitUntilDone
    chassis.moveToPoint(-7.70, -6.60, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 0.00});  // 37: moveToPoint (-7.7, -6.6) 2000ms
    while (chassis.isInMotion()) pros::delay(10);  // 38: waitUntilDone
}
// END planner: workingslow

// No auton selector - only one autonomous routine.
void runAutonomous() {
    workingslow();
}

