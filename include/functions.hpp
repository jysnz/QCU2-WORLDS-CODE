#pragma once

void jawheadControl();

// Drives the lift counter-clockwise until the bumper is pressed, then stops and
// zeroes the lift position there. Blocks until done (or a timeout).
void homeLift();
