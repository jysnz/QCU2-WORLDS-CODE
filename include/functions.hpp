#pragma once

// ─── Catapult
// ─────────────────────────────────────────────────────────────────
#include <sys/_intsup.h>

void drivetrainReset();

// Runs the lift down until it stalls on the hard stop, then makes that angle 0. Once per program run
// (the first call does it; later calls return at once). Called at the start of autonomous and of
// driver control.
void homeLift();

void jawheadControl();

void testAllMotorDirections();

// Pistons. RIGHT toggles the lift-arm orientation and B toggles the claw piston in
// driver control; the lift tuner's motor functions use the same helpers so the
// toggles always know the true state.
// The lift's angle in degrees (average of its motors). 0 is the bottom and the ONLY limit: nothing may drive
// the lift below LIFT_FLOOR_DEG.
constexpr double LIFT_FLOOR_DEG = 0.0;
double liftPositionDeg();
void setClawPiston(bool extended);
void toggleClawPiston();
bool clawPistonExtended();
void setLiftArmOrientation(bool extended);
void toggleLiftArmOrientation();
bool liftArmOrientationExtended();