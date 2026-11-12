#pragma once

// ─── IR Pin Sensor Test ──────────────────────────────────────────────────────
// Enter by tapping "IR TEST" on the driver menu; tap BACK to return.
//
// Shows the IR sensor on ADI port D live, so you can see whether it works before relying on
// auto-grab:
//   raw value        what get_value() returns right now (0, 1, or ERR if the port can't be read)
//   status           PIN DETECTED / NOTHING / NO SENSOR, using the current polarity
//   changes seen     how many times the raw value has flipped since the page opened. If it stays
//                    0 while you wave a pin in front of the sensor, the sensor is not reaching
//                    the brain (wiring / wrong port / sensor dead), not a software problem.
//   lift / at start  the lift angle and whether auto-grab's "lift is at 0" rule is met
//   would grab       YES when auto-grab would fire right now (pin seen AND lift at start)
//
// Buttons:
//   FLIP POLARITY    swaps which raw value means "pin" (live, no recompile). Once it is right, set
//                    the default in lift_calibration.cpp (pinSensorActiveHigh).
//   AUTO-GRAB        turns the real auto-grab on / off for testing: with the lift at 0, put a pin
//                    in front of the sensor and the claws should close and the lift should rise.
//   OPEN CLAWS       opens both claws again between tests
//   HOME LIFT        runs the lift back down to the limit switch
void pinSensorDiagControl();
