#pragma once

void jawheadControl();

// Drives to the given final pose (x, y, heading) and holds it for the rest of the autonomous
// period: if the robot ends short or gets pushed it drives and turns back. Pass the routine's last
// waypoint. Call it last in an autonomous routine. Never returns; PROS stops the autonomous task
// when the period ends.
void holdEndPosition(float x, float y, float theta);
