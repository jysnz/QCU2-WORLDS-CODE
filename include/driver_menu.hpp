#pragma once

// --- Driver Menu -----------------------------------------------------------
// The touchscreen entry point for driver control (same idea and look as the
// `debi` branch's menu, with only the tools this branch has).
//
// HOLD DPAD-LEFT when driver control starts to open it (or set
// START_IN_DRIVER_MENU in main.cpp to always open it). With nothing held the
// robot just drives.
//
//   [ DRIVE ] [ PID TUNING ] [ PATH PLANNER ]
//   [ TEST MOTORS ] [ LIFT TUNER ] [ MOTOR TEMPS ]
//
//   DRIVE         closes the menu; driver control carries on normally.
//   PID TUNING    the on-robot LemLib PID tuner (see pid_tuner.hpp).
//   PATH PLANNER  the point planner (see point_planner.hpp).
//   LIFT TUNER    the on-robot lift tuner; also what tools/lift_tuner.py talks to.
//   TEST MOTORS   spins every motor briefly, one at a time, and reports which
//                 ones answered. Put the robot on blocks first.
//   MOTOR TEMPS   live temperature of every motor.
//
// LEAVING A TOOL: PID TUNING, PATH PLANNER and LIFT TUNER own the whole screen
// and the controller. To get back to this menu, press and HOLD the top-right
// corner of the brain's screen for about a second.
void driverMenuControl();

// True while the menu (or one of the tools it launched) owns the brain screen;
// the dashboard task must not draw while this is set.
extern bool driverMenuActive;

// For the tools launched from the menu: call once per loop tick. Returns true
// (once) when the driver has held the top-right corner of the screen, which
// means "go back to the menu" -- the tool should clean up and return.
bool driverMenuLeaveRequested();
