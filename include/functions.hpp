#pragma once

void jawheadControl();

// Drives the lift counter-clockwise until the bumper is pressed, then stops and
// zeroes the lift position there. Blocks until done (or a timeout).
void homeLift();

// Drives to the given final pose (x, y, heading) and holds it for the rest of the autonomous
// period: if the robot ends short or gets pushed it drives and turns back. Pass the routine's last
// waypoint. Call it last in an autonomous routine. Never returns; PROS stops the autonomous task
// when the period ends.
void holdEndPosition(float x, float y, float theta);

// Blocking Jerry.io-style moves in field coordinates (inches, degrees clockwise from +Y).
// driveTo drives to (x, y) -- backwards if `forwards` is false -- and waits for it to finish;
// turnTo turns in place to an absolute heading and waits. maxSpeed 0 = default (120).
// timeoutMs caps how long the move may take.
void driveTo(float x, float y, bool forwards = true, int maxSpeed = 0, int timeoutMs = 4000);
void turnTo(float heading, int maxSpeed = 0, int timeoutMs = 4000);
