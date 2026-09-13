#pragma once

// ─── Point Planner ────────────────────────────────────────────────────────
// A minimal on-robot autonomous planner: dial in an (x, y) field target
// with the controller and run a single chassis.moveToPoint() to it --
// useful for quickly checking "does the robot actually drive to this field
// position" without editing/rebuilding autonomous.cpp and redownloading
// each time you want to try a different point.
// Enter by HOLDING DPAD-UP when driver control starts.
//
// Controls inside the planner:
//   LEFT/RIGHT   select field to edit (X <-> Y)
//   L1/L2        move the digit cursor (coarser/finer -- same idea as the
//                PID tuner: x1 -> x10 -> x100, or x1 -> x0.1 -> x0.01)
//   UP/DOWN      adjust the selected field by the current digit's step
//   Y            toggle forwards / backwards approach
//   B            reset chassis pose to (0, 0, 0) -- a known reference point
//                to plan from
//   A            run chassis.moveToPoint() to the dialed-in (x, y)
//   X            cancel the run in progress immediately (chassis stops on
//                the spot -- has no effect when nothing is running)
//
// The brain screen shows a top-down field plot: pink crosshair = target,
// green trail = the path actually driven. Footer and controller screen
// show the last run's final position and distance error. Full CSV
// telemetry streams over `pros terminal`.
void pointPlannerControl();

// True while the planner owns the brain screen; the HUD task in main.cpp
// must not draw while this is set.
extern bool pointPlannerActive;
