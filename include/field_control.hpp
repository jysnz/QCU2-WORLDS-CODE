#pragma once

// ─── Scrimmage field control ────────────────────────────────────────────
// A stand-in for the real VEX field controller, driven from laptops:
//
//   tools/field_gamemaster.py   (gamemaster laptop)  picks the period,
//          │ LAN                runs the clock, enables / disables
//   tools/field_relay.py        (one per alliance)   forwards the state to
//          │ USB -> controller -> radio               every controller plugged in
//   this module                 (on the brain)       obeys it
//
// The relay sends one line a few times a second (read by the driver menu's
// stdin listener, see driver_menu.cpp):
//   FC <A|D> <0|1> <secsLeft> <runId>   period (A = autonomous, D = driver),
//                                       enabled, clock, and an id that
//                                       changes every time the gamemaster
//                                       starts a period
//   FC OFF                              release: back to normal practice
//
// The first FC line "engages" field control for the rest of the program:
//   - the driver menu (and the PID / auto tuners) close, so players can't
//     dodge it from the touchscreen
//   - DISABLED            every motor stops; joysticks do nothing
//   - AUTO + enabled      runAutonomous() runs once in its own task; a
//                         disable (or a new runId) kills it, just like the
//                         real field kills the autonomous task
//   - DRIVER + enabled    jawheadControl() drives as normal
//   - no FC line for kHeartbeatTimeoutMs -> treated as DISABLED (unplugged
//     laptop / dropped radio stops the robot instead of leaving it running)
// The brain answers with "FCS|<A|D>|<0|1>|<autonRunning>|<runId>" lines so
// the laptops can show that each robot is listening.
//
// This is NOT the real competition state: pros::competition::* still says
// "not connected", and initialize()/autonomous()/opcontrol() are not
// re-triggered -- only the behavior above changes.

// Called once from initialize(): starts the supervisor task.
void fieldControlInit();

// Feeds one "FC ..." line from the stdin listener. Returns false if the
// line isn't one of ours.
bool fieldControlHandleLine(const char *line);

// True once the gamemaster has taken over (until "FC OFF").
bool fieldControlEngaged();

// True when joystick driving is allowed: field control not engaged, or
// engaged + enabled + DRIVER period.
bool fieldControlDriverAllowed();
