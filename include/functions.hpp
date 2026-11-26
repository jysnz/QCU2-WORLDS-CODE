#pragma once

// ─── Catapult
// ─────────────────────────────────────────────────────────────────
#include <sys/_intsup.h>

void drivetrainReset();

// Manual driving (-127..127 per side). A side given 0 freewheels: no braking, no holding voltage.
void driveByHand(int left, int right);
// A stick reading this close to centre counts as released (a resting stick often reads 1 to 3).
constexpr int DRIVE_STICK_DEADBAND = 5;

// Lowers the lift to the limit switch on the first successful home of a program run.
void homeLift();
bool liftIsHomed();
// Polls the bottom switch and returns whether it is held. Sets the encoder zero on each press, and again
// where the lift comes to rest: on the switch, or just above it after bouncing off.
bool updateLiftLimitSwitch();
// Starts a task (once) that checks the switch every 5 ms, so a tap zeroes the lift in every mode: driver
// control, autonomous, the menu and the tools. Call it from initialize().
void startLiftLimitSwitchWatcher();

void jawheadControl();

void testAllMotorDirections();

// Pistons. RIGHT toggles the lift-arm orientation and B toggles the claw piston in
// driver control; the lift tuner's motor functions use the same helpers so the
// toggles always know the true state.
// The lift's angle in degrees (average of its motors). 0 is the bottom and the ONLY limit: nothing may drive
// the lift below LIFT_FLOOR_DEG.
constexpr double LIFT_FLOOR_DEG = 0.0;
double liftPositionDeg();
// The lift is told to stop this many degrees above 0, so that what it travels while stopping does not take
// it below 0.
constexpr double LIFT_STOP_MARGIN_DEG = 3.0;
// True when the lift must not be driven further down: it is within the stop margin of 0 (or below), or the
// bottom switch is pressed.
bool liftAtFloor();
// The way to stop the lift: it stays where it is, unless it is below 0, in which case it goes back up to 0.
// Call it instead of move_velocity(0), never as well as it: two commands each tick cancel each other.
void stopLift();
// The speed to use when driving the lift down: `requested` rpm far from the bottom, slowing as it gets
// close, and 0 at the floor. Everything that drives the lift down by speed must go through this.
int liftDownRpm(int requested);
bool liftInClawPickupRange();
void setClawPiston(bool extended);
void toggleClawPiston();
bool clawPistonExtended();
void setLiftArmOrientation(bool extended);
void toggleLiftArmOrientation();
bool liftArmOrientationExtended();
