#pragma once

// --- Map path planner (laptop) -----------------------------------------------------------------------
// The brain half of debi's laptop path planner (tools/path_planner.py, the one with the field map). Tap
// PATH PLANNER in the driver menu: the brain shows this screen, and tools/remote_touch.py opens the planner
// window on the laptop. You build the route on the map; the laptop sends it here; this runs it with LemLib
// and sends the robot's pose, the path it drove and the plan's progress back.
//
// Laptop -> brain (one line each, over the same USB link as the menu):
//   PLAN CLEAR                         forget the plan on the robot
//   PLAN ADD k x y th ms fwd dir side max min exit lead drift     append one block (k = block kind, below)
//   PLAN RUN [n]                       run all blocks, or only block n
//   PLAN STOP                          cancel the run
//   POSE x y th                        set the chassis pose (the planner's start pose)
//   DRIVE l r                          drive the sides (WASD on the laptop); stops 0.4 s after the last one
//   ROBOT HOME                         claw on, lift homed: the robot's starting configuration
// Brain -> laptop:  RUI|PLANNER|<status 0 idle 1 running 2 done 3 cancelled>|<block>|<count>|<x>|<y>|<th>|<x,y;x,y;..>
//
// Block kinds the robot runs: 0-5 the six LemLib motions, 6 wait, 7 setPose, 8 waitUntilDone, 10 claw
// (extend / retract), 11 homeLift. The kinds that need hardware this robot does not have (9 liftToLevel,
// 12 goToPin, 13 shootToGoal, 14 autoGrabPin, 16 odomReset) are skipped, and the screen says how many.
void laptopPlanScreen();

// Called by the USB link reader for every line it does not handle itself; ignores lines that are not planner
// commands.
void laptopPlanHandleLine(const char *line);
