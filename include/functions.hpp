#pragma once

void jawheadControl();

// Drives the lift counter-clockwise until the bumper is pressed, then stops and
// zeroes the lift position there. Blocks until done (or a timeout).
void homeLift();

// Holds the robot's current pose for the rest of the autonomous period: if it gets pushed it
// drives and turns back. Call it last in an autonomous routine. Never returns; PROS stops the
// autonomous task when the period ends.
void holdEndPosition();
