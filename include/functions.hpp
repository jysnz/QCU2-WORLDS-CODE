#pragma once

#include "lemlib/api.hpp"

void jawheadControl();

// Drives the lift counter-clockwise until the limit switch is pressed, then stops and
// zeroes the lift position there. Blocks until done (or a timeout).
void homeLift(); // lowers the lift until the limit switch is touched, then sets the angle to 0

// Why the last stretch of moveToPointDecel / moveToPointSmooth is driven by our own controller:
// LemLib's moveToPoint replaces its speed limit with max(last power, 60) once the robot is within
// 7.5 in of the target, so a decelSpeed below 60 stops holding and the robot surges just before it
// stops. With decelOwnFinish = true (default) the last stretch is a small controller of ours that
// holds decelSpeed, then eases onto the point in proportion to the distance left. false = the old
// all-LemLib behaviour (the DECEL TUNE screen has a switch to compare them).
extern bool decelOwnFinish;

// moveToPoint that slows down as it nears the target. The drive runs at params.maxSpeed until the
// robot is `decelRange` inches from (x, y), then finishes the last stretch at `decelSpeed` (0-127)
// so it settles on the point instead of arriving at full speed. Implemented as two queued
// moveToPoint calls, so it behaves like chassis.moveToPoint: async = true returns at once (follow it
// with chassis.waitUntilDone()), false blocks until the robot has arrived. decelRange <= 0 or a
// decelSpeed that isn't below maxSpeed makes it a plain moveToPoint.
void moveToPointDecel(float x, float y, int timeoutMs, lemlib::MoveToPointParams params,
                      float decelRange, float decelSpeed, bool async = true);

// Like moveToPointDecel, but the slow-down is a ramp instead of one step: over the last `decelRange`
// inches the speed limit falls in `steps` equal stages (2-8) from params.maxSpeed down to
// `decelSpeed`, each handed to the next without stopping, so the robot eases onto the point.
// Same async / waitUntilDone behaviour as chassis.moveToPoint. A decelRange <= 0, a decelSpeed that
// isn't below maxSpeed, or a move shorter than 1.25 * decelRange makes it a plain moveToPoint.
void moveToPointSmooth(float x, float y, int timeoutMs, lemlib::MoveToPointParams params,
                       float decelRange, float decelSpeed, bool async = true, int steps = 4);

// Queues the drive part of goToPin (returns at once, like chassis.moveToPoint with async = true):
// turns to face the pin, then drives until the robot's front tip -- the claw, `reach` inches ahead
// of the odometry centre -- is on (x, y), i.e. stops `reach` short of it along the line to the pin.
// reach <= 0, driving backwards, or already within reach of the pin: no stop-short / no turn.
// The last `decelRange` inches are driven at `decelSpeed` (0-127) so the robot settles on the pin
// instead of arriving at full speed (see moveToPointDecel); decelRange <= 0 turns that off.
void goToPinDrive(float x, float y, int timeoutMs, int maxSpeed, bool forwards, float reach,
                  float decelRange, float decelSpeed);

// Fetches the pin at (x, y) (field inches): opens the claws, raises the lift to `level` (an index into
// the LIFT CAL table, e.g. LIFT_PIN_LEVEL) while driving, drives so the FRONT TIP of the robot (the
// claw, `reach` inches ahead of its centre) is on the pin, makes sure the lift got to the level, then
// closes the claws. Blocking. timeoutMs caps the drive, maxSpeed 0 = default (120). The path
// planner's goToPin block exports as a call to this.
void goToPin(float x, float y, int level, int timeoutMs = 3000, int maxSpeed = 0, bool forwards = true,
             float reach = 0, float decelRange = 10, float decelSpeed = 40);

// ─── Vision-assisted pin grab (the straight pin drive in functions.cpp) ───
// The MOMENT the camera sees the pin -- however far out -- the drive steers
// straight at the largest object of signature slot pinVisionSig on the V5
// Vision Sensor (slot 1 = YEL, the pin's captured signature; 0 turns the
// assist off) in place of the odometry line-lean, braking over the last
// 10 in down to an approach speed so the vision stop lands where it should.
// pinVisionRangeIn (5 in) short of the pin it halts, squares up on it by
// camera, then creeps in at low speed until the IR pin sensor closes the
// claws -- it pushes at most 2 in past the nominal stop point and grabs
// within 2 s, else moving on to the next motion.
extern int pinVisionSig;
extern float pinVisionRangeIn;
// The x_middle reading that means "aimed at the pin". The sensor is created
// zero-centred (centre of the FOV = 0, range -158..+158), so this defaults
// to 0 -- the brain's Vision Utility screen shows the same point as ~158-160
// on its own 0..316 scale. If the run log shows raw ~+158 when the robot is
// square to the pin, the zero-centre flag is not applying: set this to 158.
extern float pinVisionCenterPx;

// Relocalizes the odometry against a field wall. Drives into the wall (backwards by default, so
// the robot's back is flat against it) until it stalls or timeoutMs runs out, then overwrites the
// coordinate that wall fixes -- and, with setHeading, the heading -- with what is true there:
//   wall 0 = bottom (y = -72), 1 = top (y = +72), 2 = left (x = -72), 3 = right (x = +72).
// edgeIn is the distance from the robot's centre (the odometry point) to the edge that touches the
// wall, in inches (half the robot's length for its back or front). The coordinate along the wall
// is left alone. Blocking. The path planner's relocalize block exports as a call to this.
// wallAt is the coordinate of the surface it pushes on (x for walls 2 / 3, y for 0 / 1); NAN (the
// default) means the field wall at +-72. Give it to relocalize against something else whose
// position is known -- the planner's Shoot at goal does, pressing on the goal's edge.
// The correction is a safety net, not a rule: if the wall says the robot is more than
// maxCorrectionIn from where odometry has it, odometry is left alone (it is far more likely that
// `edgeIn` / `wallAt` is wrong than that odometry drifted that much), and the difference is printed
// ("RELOC ...") so the edge can be fixed: it is what edgeIn is off by when odometry was good.
void relocalizeFromWall(int wall, float edgeIn, bool backwards = true, int timeoutMs = 1500,
                        int power = 60, bool setHeading = true, float wallAt = NAN,
                        float maxCorrectionIn = 3.0f);

// Drives to the given final pose (x, y, heading) and holds it for the rest of the autonomous
// period: if the robot ends short or gets pushed it drives and turns back. Pass the routine's last
// waypoint. Call it last in an autonomous routine. Never returns; PROS stops the autonomous task
// when the period ends.
void holdEndPosition(float x, float y, float theta);

// Blocking Jerry.io-style moves in field coordinates (inches, degrees clockwise from +Y).
// driveTo drives to (x, y) -- backwards if `forwards` is false -- and waits for it to finish;
// turnTo turns in place to an absolute heading and waits. maxSpeed 0 = default (120).
// timeoutMs caps how long the move may take.
void driveTo(float x, float y, bool forwards = true, int maxSpeed = 0, int timeoutMs = 4000);
void turnTo(float heading, int maxSpeed = 0, int timeoutMs = 4000);
