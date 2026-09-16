#pragma once

// ─── Driver Menu ────────────────────────────────────────────────────────
// The touchscreen entry point for driver control. Shown as soon as
// opcontrol() starts, replacing the old "hold DPAD-LEFT/UP" scheme:
//
//   HOME              [ PID TUNING ] [ PATH PLANNER ] [ TEST MOTORS ] [ DRIVE ]
//     - PID TUNING      hands off to pidTunerControl() (see pid_tuner.hpp);
//                       its own BACK button returns here to the home menu.
//     - TEST MOTORS     spins every drivetrain motor one at a time and
//                       reports, per port, whether it's configured as
//                       reversed ("Negative") or not ("Positive").
//     - DRIVE           dismisses the menu; opcontrol() falls through to
//                       normal joystick driving.
//     - PATH PLANNER    [ ANGULAR ] [ LATERAL ]                  [BACK]
//         - ANGULAR       [TURN TO HEADING] [TURN TO POINT]
//                         [SWING TO HEADING] [SWING TO POINT]    [BACK]
//         - LATERAL       [MOVE TO POINT] [MOVE TO POSE]         [BACK]
//         - (any motion)  an EDIT screen listing every parameter that
//           motion's chassis.* call accepts, edited with the controller --
//           same digit-cursor idea as the PID tuner:
//             LEFT/RIGHT   select field
//             L1/L2        move the digit cursor (numeric fields only)
//             UP/DOWN      adjust the field (numeric: by the digit step;
//                          choice fields like forwards/direction/side:
//                          cycle to the next option)
//             A            run the motion
//             X            cancel a run in progress
//             B            reset chassis pose to (0, 0, 0)
//           Tapping BACK (top-right) returns to the ANGULAR/LATERAL menu
//           without leaving driver control.
//
// Tapping DRIVE returns from this function so opcontrol() can fall through
// to normal driving; picking PID TUNING or a motion hands off to a loop
// that owns the brain screen until its own BACK button is tapped, at which
// point this menu resumes.
//
// Every screen's touchscreen buttons can also be tapped remotely from a
// laptop over the same USB cable used by `pros terminal` -- see the
// "Remote touch bridge" section in driver_menu.cpp and
// tools/remote_touch.py for the companion app.
void driverMenuControl();

// True while this menu (or one of its Path Planner screens) owns the brain
// screen; the HUD task in main.cpp must not draw while this is set.
extern bool driverMenuActive;

// ─── Remote bridge hooks shared with the PID tuner ──────────────────────
// The stdin listener started by driverMenuControl() keeps running while
// pidTunerControl() owns the screen, so the tuner reads the laptop's
// input through these instead of the menu loop. See the "Remote touch
// bridge" section in driver_menu.cpp for the wire format.

// Controller buttons the laptop can press by name ("KEY <name>").
enum RemoteKey {
  RK_LEFT, RK_RIGHT, RK_UP, RK_DOWN, RK_A, RK_B, RK_X, RK_L1, RK_L2, RK_Y, RK_R1, RK_R2,
  RK_COUNT
};
// One-shot reads: true once per line received, then cleared.
bool takeRemoteKey(RemoteKey k);
bool takeRemoteTouch(int &x, int &y);
// How often the laptop asked for state lines (ms) -- 150 on a cable, 300
// over the controller's radio. Used to pace anything else streamed to it.
int remoteSendIntervalMs();

// "PID <verb> ..." lines from tools/pid_tuner.py, queued in arrival order.
struct RemotePidCommand {
  enum Kind { SET, MODE, SEL, DIGIT, TEST, SWEEP } kind;
  int mode = 0;   // 0 = angular, 1 = lateral   (SET / MODE / TEST / SWEEP)
  int index = 0;  // gain 0..2, test 0..2, or digit exponent
  float value = 0;
};
bool takeRemotePidCommand(RemotePidCommand &out);
// "PID STOP": true once per request. Checked mid-motion by the tuner.
bool takeRemotePidStop();
