#pragma once

// ─── Odom Reset Test ─────────────────────────────────────────────────────────
// Enter by tapping "ODOM RESET" on the driver menu; tap BACK to return.
//
// A bench test for odomReset() (see functions.hpp): put the robot on the field with the
// left / right (and front) wall sensors facing walls, tap a button, and see what the reset did.
//
// Field map (right side, like the path planner's: +Y up, +X right, a line every 24 in):
//   cyan dot + line   where odometry thinks the robot is, and its heading
//   green cross       where the wall sensors say the robot is (needs heading within 10 deg of a wall)
//   grey lines        what each wall sensor sees; the orange dot is where its beam lands
//   grey dot          the odometry pose before the last reset
//   TAP THE MAP       moves odometry to that spot (heading kept), to simulate drift. RESET XY should
//                     then pull the cyan dot onto the green cross. UNDO puts it back.
//
// Live readout:
//   odom / walls      odometry X, Y, heading and the X, Y the walls give
//   off               the gap between them, and whether odomReset() would apply it (limit 4 in)
//   F / L / R         calibrated wall distances in mm (OOR = no target, ERR = sensor missing)
//   tilt              heading tilt from the nearest wall angle (IMU) and the tilt from the left+right span
//                     (independent of the IMU), shown only when both side sensors see the two field walls
//
// Buttons:
//   RESET + HDG   odomReset(true, true, true, 4.0, true): x/y AND heading from the walls
//   RESET XY      odomReset(true, true, true, 4.0, false): x/y only, heading untouched
//   UNDO          puts the pose back to what it was before the last reset, so the same test can be repeated
//
// What to expect:
//   - odomReset corrects the ODOMETRY pose. It does not turn the robot.
//   - The heading fix sets the heading to the TRUE tilt measured from the walls (for example 90 + 8),
//     not to exactly 90. Squaring up to exactly 90 only happens if the robot really is square.
//   - It only runs when the heading is within 10 deg of a wall angle, and the heading part needs both the
//     left and right sensors to see the walls. Below about 3 deg of tilt the span is too flat to measure.
// The result of each press is shown on the brain and printed to `pros terminal` ("ODOMRESET ...").
void odomResetCalControl();

// Laptop window (tools/odom_reset_cal.py): the screen mirrors itself as "RUI|ODOM_RESET|..." lines, and the
// window sends back "ODOMR RESET 0|1", "ODOMR UNDO" and "ODOMR POSE x y". Called by the remote-touch
// listener task in driver_menu.cpp for each "ODOMR ..." line (the text after "ODOMR ").
void odomResetRemoteCommand(const char *rest);
