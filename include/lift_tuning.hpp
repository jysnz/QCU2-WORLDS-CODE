#pragma once

// --- Lift tuning -------------------------------------------------------------
// Everything about how the lift behaves in driver control that you want to
// change without re-writing code:
//
//   START POSITION   the angle (motor degrees) the lift is assumed to be at
//                    when driver control starts. Off = keep whatever the
//                    encoder says (it is zeroed at boot).
//   LIFT LIMIT       how far X may drive the lift forward.
//   COMBOS           named lists of steps (motors / pistons). Make as many as you
//                    like; every combo is also written into the code as a
//                    function you can call, e.g. combo_Score() in autonomous.
//   SCORING LEVELS   named lift heights, each playing one COMBO once the lift
//                    gets there (test them with "Run level" in the laptop tool;
//                    no driver button is bound to them right now).
//   ANGLE FUNCTIONS  "when the lift passes ANGLE (going up / down / either),
//                    run this COMBO (or one step)". Optionally tied to a
//                    controller button: HOLD = only while it is held, TAP = tap
//                    the button to arm it, then it fires once when the lift gets
//                    there. A combo that moves the lift (e.g. lift BACKWARD)
//                    wins over a held R1; the lift stays under its control until
//                    R1/R2 are released.
//   INTAKE           "when intake button L1 / L2 is pressed (or released), play
//                    this COMBO". The intake keeps spinning as usual.
//
// A STEP is: DEVICE + COMMAND (+ speed and duration for motors) + DELAY.
//   DEVICE   any motor (left/right drive, intake, lift, claw arm)
//            or piston / solenoid (claw piston, lift-arm orientation)
//   COMMAND  motors: FORWARD / BACKWARD / STOP; pistons: EXTEND / RETRACT / TOGGLE
//   DELAY    ms after the previous step in the combo STARTED (0 = at the same
//            time, = the previous step's duration for one-after-the-other)
//
// Edit it three ways:
//   * Hold DPAD-LEFT when driver control starts to open the driver menu, then
//     tap LIFT TUNER -> the on-robot LIFT TUNER (brain screen + controller, see
//     liftTunerControl()). Hold the top-right corner of the screen to go back.
//   * python tools/lift_tuner.py with the brain plugged in over USB -> the
//     laptop UI (same numbers, much easier to edit).
//   * Edit the defaults between the LIFT-TUNING markers in src/lift_tuning.cpp
//     (the laptop tool's "Write to code" button does this for you).
// Changes live in RAM until saved: "SAVE" writes /usd/lift_tuning.txt (loaded
// at boot), "Write to code" makes them the built-in defaults.

constexpr int LIFT_MAX_LEVELS = 8;
constexpr int LIFT_MAX_COMBOS = 8;
constexpr int LIFT_MAX_STEPS = 6;   // per scoring combo
constexpr int LIFT_MAX_ACTIONS = 8; // angle functions

enum LiftActWhen { LIFT_WHEN_RISING = 0, LIFT_WHEN_FALLING = 1, LIFT_WHEN_EITHER = 2 };
// Order matters: the laptop tool and the SD file use these numbers. Motors first, then pistons.
enum LiftDevice {
    LIFT_DEV_LEFT_DRIVE = 0,
    LIFT_DEV_RIGHT_DRIVE = 1,
    LIFT_DEV_INTAKE = 2,
    LIFT_DEV_LIFT = 3,
    LIFT_DEV_CLAW_ARM = 4,
    LIFT_DEV_CLAW_PISTON = 5,
    LIFT_DEV_ORIENT_PISTON = 6,
    LIFT_DEVICE_COUNT = 7
};
constexpr int LIFT_FIRST_PISTON = LIFT_DEV_CLAW_PISTON;
enum LiftCmd { LIFT_CMD_FORWARD = 0, LIFT_CMD_BACKWARD = 1, LIFT_CMD_STOP = 2, LIFT_CMD_EXTEND = 3, LIFT_CMD_RETRACT = 4, LIFT_CMD_TOGGLE = 5 };

struct LiftStep {
    int delayMs;    // after the previous combo step started (angle functions ignore it)
    int device;     // LiftDevice
    int cmd;        // LiftCmd
    int speed;      // rpm, 1..200 (motors only; the sign comes from FORWARD / BACKWARD)
    int durationMs; // how long a motor keeps running (50..10000)
};

struct LiftCombo {
    char name[12];
    int stepCount;
    LiftStep steps[LIFT_MAX_STEPS];
};

struct LiftLevel {
    char name[12];
    double angle;
    int combo; // index into LiftTuning::combos, -1 = none
};

// Which controller button an angle function listens to. 0 = none (fires whenever the lift passes).
enum LiftButton {
    LIFT_BTN_NONE = 0, LIFT_BTN_R1, LIFT_BTN_R2, LIFT_BTN_L1, LIFT_BTN_L2, LIFT_BTN_UP, LIFT_BTN_DOWN,
    LIFT_BTN_LEFT, LIFT_BTN_RIGHT, LIFT_BTN_X, LIFT_BTN_B, LIFT_BTN_Y, LIFT_BTN_A, LIFT_BUTTON_COUNT
};
// HOLD: fires only while the button is held. TAP: tapping the button arms it; it fires once, the next
// time the lift passes the angle (tap again to disarm).
enum LiftBtnMode { LIFT_MODE_HOLD = 0, LIFT_MODE_TAP = 1 };

struct LiftAction {
    double angle; // lift angle that triggers it
    int when;     // LiftActWhen
    LiftStep step;
    int combo = -1;                // >= 0: play this combo instead of the single step above
    int button = LIFT_BTN_NONE;    // LiftButton
    int mode = LIFT_MODE_HOLD;     // LiftBtnMode (ignored with no button)
};

// INTAKE FUNCTIONS: play a combo when an intake button is pressed or released. The intake itself
// keeps working as usual (L1 in, L2 out); the combo runs on top of it.
enum LiftTrigger { LIFT_TRIG_PRESSED = 0, LIFT_TRIG_RELEASED = 1 };
struct LiftIntake {
    int button = LIFT_BTN_L1;       // LiftButton (L1 / L2 are the intake buttons)
    int trigger = LIFT_TRIG_PRESSED; // LiftTrigger
    int combo = -1;                 // index into LiftTuning::combos, -1 = does nothing
};
constexpr int LIFT_MAX_INTAKE = 4;

struct LiftTuning {
    bool startEnabled = false;
    double startPosition = 0.0;
    double forwardLimit = 743.2;
    int comboCount = 0;
    LiftCombo combos[LIFT_MAX_COMBOS] = {};
    int levelCount = 0;
    LiftLevel levels[LIFT_MAX_LEVELS] = {};
    int actionCount = 0;
    LiftAction actions[LIFT_MAX_ACTIONS] = {};
    int intakeCount = 0;
    LiftIntake intake[LIFT_MAX_INTAKE] = {};
};

// -- Driver control --
void liftTuningLoad();           // boot: defaults, then /usd/lift_tuning.txt if present
void liftTuningBeginDriver();    // driver control start: applies the start position
double liftTuningForwardLimit(); // X stops driving the lift forward past this
// Step to the next (+1) / previous (-1) scoring level: the lift drives there and its combo plays.
// Returns the level's name.
const char *liftTuningStepLevel(int direction);
const char *liftTuningLevelName(); // current level ("--" before any was picked)
bool liftTuningTargetActive();     // the lift is driving to a level: don't command it to stop
void liftTuningCancelTarget();     // the driver took over the lift manually
// Call once per driver-loop tick AFTER the manual controls: fires angle functions the lift just
// passed, plays combos, and keeps timed motor steps running.
void liftTuningStep();
// True while an angle combo that moves the lift owns it: ignore R1/R2 until they are released.
// (Without this a still-held R1 would drive the lift straight back up and fire the combo again.)
bool liftTuningManualLiftBlocked();
// True while at least one TAP angle function is armed and waiting for the lift to reach its angle.
bool liftTuningArmed();

// Plays a list of steps. async = false waits until they have all finished (blocking); async = true
// starts them on their own task and returns at once (`steps` must stay valid, e.g. static).
// The generated combo_<Name>(bool async = false) functions below call this -- use them from autonomous().
void liftPlayCombo(const LiftStep *steps, int count, bool async = false);
// Drives the lift to level idx and plays its combo. async = false waits for both; true returns at once.
// The generated level_<Name>(bool async = false) functions call this.
void liftGoToLevel(int idx, bool async = false);

// Generated by tools/lift_tuner.py ("Save"): one function per combo. Don't edit between the markers.
// LIFT-COMBO-DECLS-BEGIN
// LIFT-COMBO-DECLS-END
// -- On-robot tuner (returns when the driver holds the screen's top-right corner) --
void liftTunerControl();
// Feeds one "LT ..." line from the laptop tool to the tuner. Called by the driver menu's link reader.
void liftTuningHandleLine(const char *line);
// The same tuner opened on its INTAKE page: pick which combo plays when L1 / L2 is pressed or
// released. Shares the laptop link with the lift tuner (the laptop tool's Intake tab).
void intakeTunerControl();
// True while the tuner owns the brain screen.
extern bool liftTunerActive;
// Called by the laptop link task: while the laptop window is open, sends it the live lift angle and the
// settings even when no tuner screen is open (driver control, menu).
void liftTuningSendTelemetry();
