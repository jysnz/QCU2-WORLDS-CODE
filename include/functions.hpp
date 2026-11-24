#pragma once

// ─── Catapult
// ─────────────────────────────────────────────────────────────────
#include <sys/_intsup.h>

void drivetrainReset();

void jawheadControl();

void testAllMotorDirections();

// Pistons. RIGHT toggles the lift-arm orientation and B toggles the claw piston in
// driver control; the lift tuner's motor functions use the same helpers so the
// toggles always know the true state.
void setClawPiston(bool extended);
void toggleClawPiston();
bool clawPistonExtended();
void setLiftArmOrientation(bool extended);
void toggleLiftArmOrientation();
bool liftArmOrientationExtended();