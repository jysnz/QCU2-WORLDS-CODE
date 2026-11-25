#pragma once

// ─── Odometry Setup ──────────────────────────────────────────────────────────
// Enter by tapping "ODOM SETUP" on the driver menu; tap BACK to return.
//
// Live readout of the odometry pose and both tracking wheels:
//
//   X / Y / Heading   odometry pose (heading wrapped to 0..360)
//   Rotation          IMU total rotation in degrees (continuous, not wrapped)
//   Forward_Tracker   vertical tracking wheel, inches travelled since ZERO
//   Sideways_Tracker  horizontal tracking wheel, inches travelled since ZERO (positive = right)
//
// Buttons:
//   ZERO        zeroes the pose and both tracker readouts. Push the robot 24 in
//               forward: Forward_Tracker and Y should read 24. A tracker that
//               counts the wrong way means its Rotation sensor needs reversing.
//   WHEEL CAL   push by hand a tape-measured 48 in forward, then 48 in sideways
//               (tap DONE after each). Prints the effective wheel diameters.
//   SPIN CAL    turn the robot in place by hand (3+ full turns, tap DONE). Prints
//               the tracking wheel offsets: tracker distance / heading in radians.
//
// Results are shown on the brain and printed to `pros terminal`; copy them into
// VERTICAL_WHEEL_DIAMETER / HORIZONTAL_WHEEL_DIAMETER and the TrackingWheel offsets
// in main.cpp. Do WHEEL CAL first -- SPIN CAL's distances use the diameters in main.cpp.
void odomSetupControl();
