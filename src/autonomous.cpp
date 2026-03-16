#include "autonomous.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"

// ─── Test routine ─────────────────────────────────────────────────────────────
void test() {
    drive_for_inches(80, 15);
    pros::delay(500);
    turn_to_heading(90, 100);
}

// ─── Main autonomous entry point ──────────────────────────────────────────────
// Select which routine runs here.
void runAutonomous() {
    test();
}