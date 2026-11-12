#pragma once

// ─── IMU heading drift correction ────────────────────────────────────────────
// An IMU's heading creeps a little even when it isn't turning, and over a long autonomous
// that shows up as the whole pose drifting. This measures that creep whenever the robot is
// provably standing still -- the drive motors AND both tracking-wheel rotation sensors all
// read zero speed -- and subtracts it from the heading odometry uses, continuously.
//
// Start it once after chassis.calibrate() (main.cpp initialize). It then runs in its own
// task and needs no further calls.
//
//   imuDriftRate()   the drift being removed right now, in degrees per second
//   imuDriftTotal()  how much heading has been subtracted so far, in degrees
//
// Only drift measured while stationary is corrected; real turning is never touched, and the
// estimate is capped (MAX_RATE) so a bad reading can't eat a real rotation.
void startImuDriftCorrection();
double imuDriftRate();
double imuDriftTotal();
// True while the drive motors and both tracking wheels all read (near) zero speed.
bool robotStandingStill();
