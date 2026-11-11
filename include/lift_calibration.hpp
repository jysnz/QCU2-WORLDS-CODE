#pragma once

// ─── Lift Calibration ────────────────────────────────────────────────────────
// Enter by tapping "LIFT CAL" on the driver menu; tap BACK to return.
//
// The lift has a handful of named levels. Each has a LIFT angle (where the lift
// goes to raise to the level) and, except for the first stage, a DROP angle
// (where it goes to drop the game piece). Angles are motor degrees measured from
// the limit switch home position (0 = limit switch pressed).
//
// DROPPING
//   First stage (GROUND)  has no drop angle: dropping there just opens the claws.
//   Every other level     the lift goes to the level's DROP angle and, once it
//                         has reached it, the claws open on their own.
//
// ON THE BRAIN / CONTROLLER
//   tap a level      select it                         UP    next level (wraps)   DOWN  claws
//   GO TO  (B)       lift to the level's LIFT angle    SAVE LIFT (A)  store the current angle as it
//   DROP   (RIGHT)   drop at the level (see above)     SAVE DROP (LEFT)  store the current angle as DROP
//   JOG +/-          nudge the lift 10 degrees         R1 / R2 (held)  jog continuously
//   HOME   (X)       run down to the limit switch, zero it   C1 / C2 / ALL    claws (L1 / L2 / Y)
//
// FROM A LAPTOP (tools/lift_calibration.py, opened by tools/remote_touch.py
// when the brain enters this screen)
//   Every level is a row with editable LIFT / DROP angles and its own GO / SAVE
//   buttons, a live gauge of the lift, jog and claw controls. SAVE TO CODE (Ctrl+S,
//   or SAVE CODE on the brain while the laptop window is open) writes the whole
//   table into src/lift_calibration.cpp.
//
// Wire format:
//   robot -> laptop  RUI|LIFT_CAL|...|LIFT:<sel>,<live>,<moving>,<limit switch>,<upper>,<claw>,<claw2>,<dropping>,<saveReq>
//                    ~<name>,<lift>,<drop>;...|MSG:..          (drop = -1: level has none)
//   laptop -> robot  LIFT SEL <i>   LIFT SET <i> <deg>   LIFT GO <i>   LIFT SAVE <i>
//                    LIFT SETDROP <i> <deg>   LIFT GODROP <i>   LIFT SAVEDROP <i>
//                    LIFT JOG <deg>   LIFT HOME   LIFT STOP   LIFT SAVED <1 ok|0 failed>
//                    (saveReq counts taps of SAVE CODE on the brain; the laptop writes the
//                    source for each new value and answers LIFT SAVED)
//                    LIFT CLAW <0 both|1 claw|2 claw2> <0 extend|1 retract|2 toggle>
void liftCalibrationControl();

// Number of levels and the table itself. Angles are in motor degrees from the
// limit switch home position. Edited live by the calibration screen; make the new
// values permanent by pasting them into kDefaultAngles / kDefaultDropAngles in
// lift_calibration.cpp (the laptop window's WRITE TO SOURCE does this for you).
// The last level, LIFT_PIN_LEVEL ("PIN"), is the lift height when a pin is fetched.
constexpr int LIFT_LEVEL_COUNT = 6;
constexpr int LIFT_PIN_LEVEL = 5;
extern double liftLevelAngles[LIFT_LEVEL_COUNT];
extern double liftDropAngles[LIFT_LEVEL_COUNT]; // unused for LIFT_GROUND_LEVEL
const char *liftLevelName(int level);

// The first stage. It has no drop angle: dropping there only opens the claws.
constexpr int LIFT_GROUND_LEVEL = 0;
inline bool liftHasDrop(int level) { return level != LIFT_GROUND_LEVEL; }

// Which pneumatic state OPENS the claws. The claws are built extended (the
// state that holds a piece), so open = retracted. Flip this if yours are the
// other way round; tools/lift_calibration.py has the matching CLAW_OPEN_RETRACTED.
constexpr bool LIFT_CLAW_OPEN_RETRACTED = true;
void liftOpenClaws();

// The lift motors normally HOLD: left raised, they keep working against the lift's weight (the
// hum after a run that ends with the lift up). liftRelax() stops that -- the motors only brake, so
// the lift settles under its own weight -- and liftHold() puts the hold back. Every lift move in
// this file, homeLift() and a plan run call liftHold() themselves.
void liftRelax();
void liftHold();

// Shuts the claws (claw / claw2 independently). Does not move the lift.
void clawsClose(bool useClaw = true, bool useClaw2 = true);

// Upper travel limit in motor degrees from the limit switch home position. The lower
// end is stopped by the limit switch itself.
constexpr double LIFT_UPPER_LIMIT = 500.0;

// Sends the lift to a level's LIFT angle (or its DROP angle with drop = true),
// usable from driver control or autonomous. Out-of-range levels are ignored.
// With wait = false it returns at once and the lift keeps moving; with
// wait = true it blocks until the lift is within a few degrees of the angle or
// timeoutMs passes. Does not touch the claws.
void liftGoToLevel(int level, bool wait = false, int timeoutMs = 2000, bool drop = false);

// The controlled move used for every drop: not a flat-out slam and not a crawl.
// The lift speeds up smoothly (LIFT_DROP_ACCEL), cruises at up to LIFT_DROP_MAX_VEL
// and eases off in proportion to how far it still has to go (LIFT_DROP_KP), never
// slower than LIFT_DROP_MIN_VEL so it still arrives. Tune these four to taste.
constexpr double LIFT_DROP_MAX_VEL = 70.0; // rpm (the lift motors are 100 rpm at most)
constexpr double LIFT_DROP_MIN_VEL = 12.0; // rpm, the slowest it creeps in
constexpr double LIFT_DROP_KP = 1.2;       // rpm per degree still to go
constexpr double LIFT_DROP_ACCEL = 0.35;   // rpm gained per millisecond

// One tick of that move. Call it every ~10 ms with the same `vel` variable (start
// it at 0) and the time since the last call. Returns true, with the lift stopped,
// once it is within a few degrees of `targetDeg`.
bool liftControlledStep(double targetDeg, double &vel, int dtMs);

// Drops at a level and blocks until done: GROUND just opens the claws; any other
// level goes to its DROP angle, waits for it (up to timeoutMs) and then opens
// the claws. The path planner's liftToLevel block with position DROP exports as
// a call to this.
void liftDropAtLevel(int level, int timeoutMs = 2000);

// The level whose LIFT angle is closest to where the lift is right now (GROUND when it is down).
int liftCurrentLevel();

// Scores into the goal from wherever the lift is: drops at the current level, so at GROUND it just
// opens the claws, and at any other level it goes to that level's DROP angle and then opens them.
// Blocks until done (timeoutMs caps the lift's move). The planner's shootToGoal block exports as
// a call to this.
void shootToGoal(int timeoutMs = 2000);

// True when the lift is within a few degrees of the level's angle.
bool liftAtLevel(int level, bool drop = false);

// ─── IR pin sensor (ADI D) ───────────────────────────────────────────────────
// The sensor is pinSensor (ADI D, a DigitalIn in main.cpp), read with get_value().
// true: raw 1 means a pin is seen; false: raw 0 does. Change it live with FLIP POLARITY on the
// driver menu's IR TEST screen, then set the default in lift_calibration.cpp.
extern bool pinSensorActiveHigh;

// True while the IR sensor sees a pin. A missing sensor (get_value() returns an error code) is
// never "detected".
bool pinDetected();

// True when the lift is at its starting position (limit switch pressed / angle ~0).
bool liftAtStart();
// The limit switch, debounced: true only once it has read pressed for 30 ms in a row, so a
// flicker (vibration, a loose wire) can't stop the lift or re-zero the angle. Call it every loop.
bool liftLimitPressed();

// Auto-grab: while enabled, whenever the sensor sees a pin AND the lift is at its starting
// position, the claws close and the lift rises to the PIN level. Anywhere else the lift is
// left alone. Runs in a background task, so it works in driver control and autonomous.
// The planner's autoGrabPin block exports as a call to this.
void pinAutoGrabEnable(bool on);
bool pinAutoGrabEnabled();

// One check of the rule above (the task calls it every 10 ms). Returns true if it grabbed.
bool pinAutoGrabTry();

// Claw watch, used by goToPin: while on, a background task closes the claws the instant the sensor
// sees a pin, whatever the robot is doing (driving, turning, waiting). Turn it off when done.
void pinCloseWatch(bool on);
bool pinCloseWatchHit(); // true if it closed the claws since it was turned on

// For driver control, which otherwise commands the lift every loop: true while the lift is on its
// way up to the PIN level after an auto-grab (don't zero its velocity), and how many grabs there
// have been (the driver loop uses it to keep its claw-toggle state in step).
bool pinGrabLiftMoving();
void pinGrabCancel();   // the driver took the lift over
int pinGrabCount();
