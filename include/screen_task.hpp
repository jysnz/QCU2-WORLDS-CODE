#pragma once
#include "dashboard.hpp"
#include "pros/abstract_motor.hpp"
#include <functional>
#include <string>
#include <vector>

// Runs the thermal dashboard on the brain screen: reads the motors, handles touch
// (VEXNET switch, stress alert, auton picker) and redraws only when something changed.
namespace dashboard {

// Tiles for a motor group, one per motor: labeled by port ("P3"), named "<prefix> P3".
std::vector<MotorReading> readMotors(const pros::AbstractMotor &motors, const char *prefix);
// One tile for a single motor: labeled "<label> P8", named "<name> P8".
MotorReading readMotor(const pros::AbstractMotor &motor, const char *label, const char *name);

struct ScreenConfig {
    // Fills leftDrive / rightDrive / centerLeft / centerRight with fresh readings.
    std::function<void(State &)> readMotors;
    // True while something else (PID tuner, point planner) owns the brain screen.
    std::function<bool()> screenBusy = [] { return false; };

    // Optional autonomous selector. selectedAuton is updated when a routine is
    // tapped, then onAutonSelected runs (e.g. to save the choice to the SD card).
    const std::vector<std::string> *autonNames = nullptr;
    int *selectedAuton = nullptr;
    std::function<void()> onAutonSelected = [] {};
};

void startScreenTask(ScreenConfig config);

} // namespace dashboard
