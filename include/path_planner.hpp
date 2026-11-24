#pragma once

// ─── Path Planner (the debi branch's on-brain editor) ────────────────────────
// Open it from the driver menu (hold DPAD-LEFT when driver control starts, then tap PATH PLANNER).
//
//   PATH PLANNER      [ ANGULAR ] [ LATERAL ]                      [< BACK]
//     ANGULAR           [TURN HDG] [TURN PT] [SWING HDG] [SWING PT]
//     LATERAL           [MOVE PT] [MOVE POSE]
//     (any motion)      an EDIT screen listing every parameter that motion's chassis.* call takes,
//                       edited with the controller:
//                         LEFT/RIGHT   select field
//                         L1/L2        move the digit cursor (numeric fields)
//                         UP/DOWN      adjust the field (numeric: by the digit step; choice fields
//                                      like FWD / DIR / SIDE cycle to the next option)
//                         A            run the motion
//                         X            cancel a run in progress
//                         B            reset the chassis pose to (0, 0, 0)
//
// The EDIT screen shows the real C++ call the current values produce, a field plot (pink crosshair or
// heading arrow = target, green trail = the path the robot drove) and the last run's final position and
// error. Tapping BACK steps up one level; BACK on the first screen returns to the driver menu. Holding
// the brain screen's top-right corner for a second also returns to the menu from anywhere in here.
// A run's CSV samples stream over `pros terminal`.
void pathPlannerControl();

// True while the planner owns the brain screen; the dashboard task must not draw while this is set.
extern bool pathPlannerActive;
