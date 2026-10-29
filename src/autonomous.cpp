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

// Holds the robot's final pose for the rest of the autonomous period: if it gets pushed, drive and
// turn back to where it ended. Never returns; PROS stops the autonomous task when the period ends.
static void holdEndPosition() {
    const lemlib::Pose home = chassis.getPose();
    constexpr float kPosTol = 1.5f;  // inches
    constexpr float kAngTol = 3.0f;  // degrees

    while (true) {
        const lemlib::Pose p = chassis.getPose();
        const float dx = home.x - p.x, dy = home.y - p.y;
        const float dist = std::hypot(dx, dy);
        float headErr = std::fmod(home.theta - p.theta + 540.0f, 360.0f) - 180.0f; // -180..180

        if (std::isfinite(dist) && std::isfinite(headErr)) {
            if (dist > kPosTol) {
                // drive in reverse if the spot is behind the robot, so it doesn't spin around first
                const float toPoint = std::atan2(dx, dy) * 180.0f / (float)M_PI; // 0 = +Y
                const float rel = std::fmod(toPoint - p.theta + 540.0f, 360.0f) - 180.0f;
                chassis.moveToPoint(home.x, home.y, 1500, {.forwards = std::abs(rel) <= 90.0f});
                chassis.waitUntilDone();
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            } else if (std::abs(headErr) > kAngTol) {
                chassis.turnToHeading(home.theta, 1000);
                chassis.waitUntilDone();
            }
        }
        pros::delay(20);
    }
}

// No auton selector - only one autonomous routine.
void runAutonomous() {
    thirty_sec();
    holdEndPosition();
}

