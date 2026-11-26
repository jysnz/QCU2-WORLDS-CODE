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
//   [ TEST MOTORS ] [ LIFT TUNER ] [ INTAKE TUNER ]
//   [ MOTOR TEMPS ] [ DISTANCE ]
//
//   DRIVE         closes the menu; driver control carries on normally.
//   PID TUNING    the on-robot LemLib PID tuner (see pid_tuner.hpp).
//   PATH PLANNER  the on-brain movement editor from the debi branch (see path_planner.hpp).
//   LIFT TUNER    the on-robot lift tuner; also what tools/lift_tuner.py talks to.
//   INTAKE TUNER  its own screen for the intake buttons: pick which combo plays when
//                 L1 / L2 is pressed or released. Also talks to tools/lift_tuner.py
//                 (its Intake tab).
//   TEST MOTORS   spins each drivetrain motor for 2 s, one at a time, and reports which
//                 ones answered and how fast. Put the robot on blocks first.
//   MOTOR TEMPS   live temperature of every motor.
//   DISTANCE      checks the VL53L0X distance sensor (see distance_diag.hpp): is it working, what does
//                 it read, and a calibration offset.
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

// For a tool whose screen the laptop window (tools/remote_touch.py) can mirror: send that screen's state as one
// "RUI|<SCREEN>|..." line. Call it every time the tool redraws.
void driverMenuPublish(const char *line);

// Starts the USB link to the laptop window (tools/remote_touch.py). Safe to call more than once. Call it from
// initialize(): until then the brain sends nothing, and the window can never connect.
void driverMenuStartLink();

// ─── Hooks the PID tuner (copied from the debi branch) expects ───────────────
// On debi these carry the laptop window's key presses and PID commands. Here only the screen taps from
// tools/remote_touch.py are real; the others always say "nothing arrived", so the tuner works from the
// brain and the controller alone.
enum RemoteKey {
  RK_LEFT, RK_RIGHT, RK_UP, RK_DOWN, RK_A, RK_B, RK_X, RK_L1, RK_L2, RK_Y, RK_R1, RK_R2,
  RK_COUNT
};
bool takeRemoteKey(RemoteKey k);              // always false
bool takeRemoteTouch(int &x, int &y);         // a tap from the laptop window, once
int remoteSendIntervalMs();                   // how often the laptop link sends (ms)

struct RemotePidCommand {
  enum Kind { SET, MODE, SEL, DIGIT, TEST, SWEEP, CAL, AUTO_START, AUTO_MODE } kind;
  int mode = 0;   // 0 = angular, 1 = lateral
  int index = 0;
  float value = 0;
};
bool takeRemotePidCommand(RemotePidCommand &out); // always false
bool takeRemotePidStop();                         // always false
