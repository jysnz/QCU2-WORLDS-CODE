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

