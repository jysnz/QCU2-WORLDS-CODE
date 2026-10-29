#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

// Jerry.io "speed 30" -> 30% of full power.
static constexpr int kSpeed = 120;

static void driveTo(float x, float y, bool forwards) {
    chassis.moveToPoint(x, y, 4000, {.forwards = forwards, .maxSpeed = kSpeed});
    chassis.waitUntilDone();
}

static void turnTo(float heading) {
    chassis.turnToHeading(heading, 2000, {.maxSpeed = kSpeed});
    chassis.waitUntilDone();
}

void thirty_sec(){
    chassis.setPose(-0.158, -62.576, 359.556);
    driveTo(-0.463, -23.291, true);   // forward(39.286)
    turnTo(45.735);
    driveTo(23.595, 0.158, true);     // forward(33.596)
    turnTo(313.877);
    driveTo(-0.158, 22.999, true);    // forward(32.953)
    turnTo(45);
    driveTo(-23.303, -0.146, false);  // backward(32.732)
    turnTo(270.444);
    driveTo(-62.588, 0.158, true);    // forward(39.286)
    turnTo(58.241);
    driveTo(-24.217, 23.912, true);   // forward(45.129)
    turnTo(332.835);
    driveTo(-0.463, -22.377, false);  // backward(52.028)
    turnTo(0);
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    thirty_sec();
}

