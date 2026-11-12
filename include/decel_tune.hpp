#pragma once

// ─── Decel Tune ──────────────────────────────────────────────────────────────
// Enter by tapping "DECEL TUNE" on the driver menu; tap BACK to return.
//
// Tunes the smooth deceleration used by moveToPoint / Grab Pin steps (moveToPointDecel and
// moveToPointSmooth). Put the robot with clear floor ahead of it, set the numbers, tap RUN and watch.
//
//   DIST    inches driven forward from where the robot stands
//   SPEED   top speed (0-127), the cruise speed of the real step
//   RANGE   inches before the target where the slow-down starts (the planner's "decel range")
//   END     speed (0-127) it finishes the last stretch at (the planner's "decel speed")
//   STEPS   0 = one step down to END; 2-8 = a ramp of that many stages (moveToPointSmooth)
//
//   RUN     drive forward DIST       RETURN  drive back to where RUN started (same settings, reversed)
//   STOP    cancel the drive         BACK    leave
//
// After each run it shows the time, and how far past (+) or short of (-) the target the robot
// stopped. Too much overshoot: raise RANGE or lower END. Too slow to arrive: lower RANGE or raise
// END. Copy the final RANGE / END into the planner's decel range / decel speed fields (they are
// also printed to the terminal).
void decelTuneControl();
