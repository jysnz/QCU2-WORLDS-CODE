#pragma once

// ─── Driver Menu ────────────────────────────────────────────────────────
// The touchscreen entry point for driver control. Shown as soon as
// opcontrol() starts, replacing the old "hold DPAD-LEFT/UP" scheme:
//
//   HOME              [ PID TUNING ] [ AUTON BUILDER ] [ PATH PLANNER ]
//                     [ TEST MOTORS ] [ DRIVE ]
//     - PID TUNING      hands off to pidTunerControl() (see pid_tuner.hpp);
//                       its own BACK button returns here to the home menu.
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
// The AUTON BUILDER entry opens the Brain touchscreen drag/drop editor.
void driverMenuControl();

// True while this menu (or one of its Path Planner screens) owns the brain
// screen; the HUD task in main.cpp must not draw while this is set.
extern bool driverMenuActive;
