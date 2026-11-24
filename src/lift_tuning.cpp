// -----------------------------------------------------------------------------
// LIFT TUNING -- see include/lift_tuning.hpp for what it is. This file has:
//   1. the settings (defaults + SD card file + the text commands that edit them)
//   2. the executor: motor / piston steps, scoring combos, angle functions
//   3. the on-robot LIFT TUNER (brain screen + controller + laptop link)
//
// LAPTOP LINK (tools/lift_tuner.py). Lines on the same USB cable `pros terminal`
// uses, newline terminated:
//   robot -> laptop  LTL|live,goalActive,sel,curLevel,comboRunning,saved   every 150 ms
//                    LTC|startEn,start,limit|<combo>;..|<level>;..|<act>;..  every 1 s / on change
//                      combo = name:<step>/<step>..   step = delay,dev,cmd,speed,dur
//                      level = name:angle:comboIndex (-1 = none)
//                      act   = angle,when,delay,dev,cmd,speed,dur,combo,button,mode
//                              (combo -1 = use the single step; button 0 = none; mode 0 hold / 1 tap)
//   laptop -> robot  LT BEGIN | LT START en deg | LT LIMIT deg | LT CLEAR
//                    LT COMBO name | LT STEP delay dev cmd speed dur  (to the last combo)
//                    LT LEVEL angle name comboIndex
//                    LT ACT angle when dev cmd speed dur [combo button mode] | LT APPLY
//                    (the last three are optional so older SD files and tools still load)
//                    LT GO deg | LT JOG deg | LT STOP | LT SAVE
//                    LT TESTACT i | LT TESTCOMBO i | LT TESTSTEP level step | LT RUNLEVEL i
// The SD file (/usd/lift_tuning.txt) is just the BEGIN..APPLY lines, replayed at boot.
// -----------------------------------------------------------------------------

#include "lift_tuning.hpp"
#include "canvas.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

bool liftTunerActive = false;

// === 1. Settings ============================================================
static LiftTuning cfg;     // live settings
static LiftTuning pending; // being edited by LT BEGIN .. LT APPLY
static pros::Mutex mtx;

// Built-in defaults. The laptop tool's "Write to code" rewrites the block between the markers.
static void defaultTuning(LiftTuning &t) {
    // LIFT-TUNING-BEGIN
    t.startEnabled = false;
    t.startPosition = 0.0;
    t.forwardLimit = 743.2;
    t.comboCount = 0;
    t.levelCount = 3;
    t.levels[0] = {"Low", 150.0, -1};
    t.levels[1] = {"Mid", 400.0, -1};
    t.levels[2] = {"High", 700.0, -1};
    t.actionCount = 0;
    // LIFT-TUNING-END
}

static bool isPiston(int dev) { return dev >= LIFT_FIRST_PISTON; }

static void sanitizeStep(LiftStep &s) {
    s.device = std::clamp(s.device, 0, (int)LIFT_DEVICE_COUNT - 1);
    s.cmd = std::clamp(s.cmd, 0, 5);
    if (isPiston(s.device) && s.cmd < LIFT_CMD_EXTEND)
        s.cmd = s.cmd == LIFT_CMD_FORWARD ? LIFT_CMD_EXTEND : (s.cmd == LIFT_CMD_BACKWARD ? LIFT_CMD_RETRACT : LIFT_CMD_TOGGLE);
    if (!isPiston(s.device) && s.cmd >= LIFT_CMD_EXTEND)
        s.cmd = s.cmd == LIFT_CMD_EXTEND ? LIFT_CMD_FORWARD : (s.cmd == LIFT_CMD_RETRACT ? LIFT_CMD_BACKWARD : LIFT_CMD_STOP);
    s.speed = std::clamp(s.speed, 1, 200);
    s.durationMs = std::clamp(s.durationMs, 50, 10000);
    s.delayMs = std::clamp(s.delayMs, 0, 20000);
}

static void sanitizeTuning(LiftTuning &t) {
    t.comboCount = std::clamp(t.comboCount, 0, LIFT_MAX_COMBOS);
    t.levelCount = std::clamp(t.levelCount, 0, LIFT_MAX_LEVELS);
    t.actionCount = std::clamp(t.actionCount, 0, LIFT_MAX_ACTIONS);
    t.forwardLimit = std::max(t.forwardLimit, 1.0);
    for (int i = 0; i < t.comboCount; i++) {
        t.combos[i].name[sizeof(t.combos[i].name) - 1] = 0;
        t.combos[i].stepCount = std::clamp(t.combos[i].stepCount, 0, LIFT_MAX_STEPS);
        for (int j = 0; j < t.combos[i].stepCount; j++) sanitizeStep(t.combos[i].steps[j]);
    }
    for (int i = 0; i < t.levelCount; i++) {
        t.levels[i].name[sizeof(t.levels[i].name) - 1] = 0;
        if (t.levels[i].combo < 0 || t.levels[i].combo >= t.comboCount) t.levels[i].combo = -1;
    }
    for (int i = 0; i < t.actionCount; i++) {
        t.actions[i].when = std::clamp(t.actions[i].when, 0, 2);
        sanitizeStep(t.actions[i].step);
        if (t.actions[i].combo < 0 || t.actions[i].combo >= t.comboCount) t.actions[i].combo = -1;
        t.actions[i].button = std::clamp(t.actions[i].button, 0, (int)LIFT_BUTTON_COUNT - 1);
        t.actions[i].mode = std::clamp(t.actions[i].mode, 0, 1);
    }
}

// Applies one settings command to `t`. Returns true if the line was a settings command
// (LT BEGIN / APPLY are handled by the callers).
static bool parseSettingLine(const char *line, LiftTuning &t) {
    int a, b, c, d, e, f, g, h;
    double x;
    char name[16];
    char cname[16];
    if (sscanf(line, "LT START %d %lf", &a, &x) == 2) {
        t.startEnabled = a != 0;
        t.startPosition = x;
    } else if (sscanf(line, "LT LIMIT %lf", &x) == 1) {
        t.forwardLimit = x;
    } else if (strncmp(line, "LT CLEAR", 8) == 0) {
        t.comboCount = 0;
        t.levelCount = 0;
        t.actionCount = 0;
    } else if (sscanf(line, "LT COMBO %11s", cname) == 1) {
        if (t.comboCount < LIFT_MAX_COMBOS) {
            LiftCombo &cb = t.combos[t.comboCount++];
            cb = LiftCombo{};
            snprintf(cb.name, sizeof(cb.name), "%s", cname);
        }
    } else if (sscanf(line, "LT LEVEL %lf %11s %d", &x, name, &a) >= 2) {
        if (sscanf(line, "LT LEVEL %lf %11s %d", &x, name, &a) < 3) a = -1;
        if (t.levelCount < LIFT_MAX_LEVELS) {
            LiftLevel &l = t.levels[t.levelCount++];
            l = LiftLevel{};
            snprintf(l.name, sizeof(l.name), "%s", name);
            l.angle = x;
            l.combo = a;
        }
    } else if (sscanf(line, "LT STEP %d %d %d %d %d", &a, &b, &c, &d, &e) == 5) {
        if (t.comboCount > 0 && t.combos[t.comboCount - 1].stepCount < LIFT_MAX_STEPS) {
            LiftCombo &cb = t.combos[t.comboCount - 1];
            cb.steps[cb.stepCount++] = LiftStep{a, b, c, d, e};
        }
    } else if (strncmp(line, "LT ACT ", 7) == 0) {
        // angle when dev cmd speed dur  [combo button mode]  -- the last three are optional
        int n = sscanf(line, "LT ACT %lf %d %d %d %d %d %d %d %d", &x, &a, &b, &c, &d, &e, &f, &g, &h);
        if (n >= 6 && t.actionCount < LIFT_MAX_ACTIONS) {
            LiftAction act{x, a, LiftStep{0, b, c, d, e}};
            if (n >= 9) {
                act.combo = f;
                act.button = g;
                act.mode = h;
            }
            t.actions[t.actionCount++] = act;
        }
    } else {
        return false;
    }
    return true;
}

static void writeSettings(FILE *f, const LiftTuning &t) {
    fprintf(f, "LT BEGIN\nLT CLEAR\nLT START %d %.2f\nLT LIMIT %.2f\n", t.startEnabled ? 1 : 0, t.startPosition, t.forwardLimit);
    for (int i = 0; i < t.comboCount; i++) {
        const LiftCombo &cb = t.combos[i];
        fprintf(f, "LT COMBO %s\n", cb.name);
        for (int j = 0; j < cb.stepCount; j++) {
            const LiftStep &s = cb.steps[j];
            fprintf(f, "LT STEP %d %d %d %d %d\n", s.delayMs, s.device, s.cmd, s.speed, s.durationMs);
        }
    }
    for (int i = 0; i < t.levelCount; i++) fprintf(f, "LT LEVEL %.2f %s %d\n", t.levels[i].angle, t.levels[i].name, t.levels[i].combo);
    for (int i = 0; i < t.actionCount; i++) {
        const LiftAction &a = t.actions[i];
        fprintf(f, "LT ACT %.2f %d %d %d %d %d %d %d %d\n", a.angle, a.when, a.step.device, a.step.cmd, a.step.speed,
                a.step.durationMs, a.combo, a.button, a.mode);
    }
    fprintf(f, "LT APPLY\n");
}

static const char *const kSdPath = "/usd/lift_tuning.txt";

static bool saveToSd() {
    FILE *f = fopen(kSdPath, "w");
    if (!f) return false;
    mtx.take();
    LiftTuning snap = cfg;
    mtx.give();
    writeSettings(f, snap);
    fclose(f);
    return true;
}

void liftTuningLoad() {
    LiftTuning t;
    defaultTuning(t);
    sanitizeTuning(t);
    cfg = t;
    FILE *f = fopen(kSdPath, "r");
    if (!f) return;
    LiftTuning edit = cfg;
    char line[160];
    bool applied = false;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "LT BEGIN", 8) == 0) edit = cfg;
        else if (strncmp(line, "LT APPLY", 8) == 0) {
            sanitizeTuning(edit);
            cfg = edit;
            applied = true;
        } else parseSettingLine(line, edit);
    }
    fclose(f);
    (void)applied;
}

double liftTuningForwardLimit() { return cfg.forwardLimit; }

// === 2. Executor ============================================================
struct MotorSlot {
    bool active = false;
    uint32_t until = 0;
    int velocity = 0;
};
static MotorSlot slots[LIFT_FIRST_PISTON];

static pros::AbstractMotor *motorFor(int dev) {
    switch (dev) {
    case LIFT_DEV_LEFT_DRIVE: return &left_motor_group;
    case LIFT_DEV_RIGHT_DRIVE: return &right_motor_group;
    case LIFT_DEV_INTAKE: return &intake2;
    case LIFT_DEV_LIFT: return &lift;
    case LIFT_DEV_CLAW_ARM: return &claw_arm;
    default: return &claw_arm;
    }
}

static int curLevel = -1;
static bool targetActive = false;
static double targetAngle = 0;
static uint32_t targetSetMs = 0;
static bool comboOnArrival = false;

struct Combo {
    bool active = false;
    int level = -1;
    int next = 0;
    uint32_t fireAt = 0;
};
static Combo combo;
static double lastPos = 0;

static void runStep(LiftStep s, uint32_t nowMs) {
    sanitizeStep(s);
    if (isPiston(s.device)) {
        bool toggle = s.cmd == LIFT_CMD_TOGGLE, out = s.cmd == LIFT_CMD_EXTEND;
        switch (s.device) {
        case LIFT_DEV_CLAW_PISTON: if (toggle) toggleClawPiston(); else setClawPiston(out); break;
        default: if (toggle) toggleLiftArmOrientation(); else setLiftArmOrientation(out); break;
        }
        return;
    }
    MotorSlot &slot = slots[s.device];
    if (s.device == LIFT_DEV_LIFT) targetActive = false; // a lift step overrides a drive-to-level
    if (s.cmd == LIFT_CMD_STOP) {
        slot.active = false;
        motorFor(s.device)->move_velocity(0);
        return;
    }
    int v = s.speed * (s.cmd == LIFT_CMD_BACKWARD ? -1 : 1);
    if (s.device == LIFT_DEV_INTAKE) v = -v; // the intake's "forward" (L1) spins the negative way
    slot = MotorSlot{true, nowMs + s.durationMs, v};
}

static void startCombo(int comboIdx, uint32_t nowMs) {
    if (comboIdx < 0 || comboIdx >= cfg.comboCount || cfg.combos[comboIdx].stepCount == 0) return;
    combo = Combo{true, comboIdx, 0, nowMs + (uint32_t)cfg.combos[comboIdx].steps[0].delayMs};
}

static void goToLevel(int idx, bool withCombo, uint32_t nowMs) {
    idx = std::clamp(idx, 0, std::max(0, cfg.levelCount - 1));
    if (cfg.levelCount == 0) return;
    curLevel = idx;
    combo.active = false;
    targetAngle = cfg.levels[idx].angle;
    targetActive = true;
    targetSetMs = nowMs;
    comboOnArrival = withCombo && cfg.levels[idx].combo >= 0;
    lift.move_absolute(targetAngle, 200);
}

static void resetActionState();

const char *liftTuningStepLevel(int direction) {
    mtx.take();
    uint32_t nowMs = pros::millis();
    if (cfg.levelCount > 0) {
        int next = curLevel < 0 ? (direction > 0 ? 0 : cfg.levelCount - 1) : std::clamp(curLevel + direction, 0, cfg.levelCount - 1);
        goToLevel(next, true, nowMs);
    }
    mtx.give();
    return liftTuningLevelName();
}

const char *liftTuningLevelName() { return curLevel >= 0 && curLevel < cfg.levelCount ? cfg.levels[curLevel].name : "--"; }
bool liftTuningTargetActive() { return targetActive; }
void liftTuningCancelTarget() {
    targetActive = false;
    comboOnArrival = false;
}

void liftTuningBeginDriver() {
    mtx.take();
    if (cfg.startEnabled) lift.set_zero_position_all(cfg.startPosition);
    curLevel = -1;
    targetActive = false;
    combo.active = false;
    for (MotorSlot &s : slots) s.active = false;
    resetActionState();
    lastPos = lift.get_position();
    mtx.give();
}

// Timed motor steps keep driving until they expire. (Caller holds mtx.)
static void serviceSlots(uint32_t nowMs) {
    for (int d = 0; d < LIFT_FIRST_PISTON; d++) {
        if (!slots[d].active) continue;
        if ((int32_t)(nowMs - slots[d].until) >= 0) {
            slots[d].active = false;
            motorFor(d)->move_velocity(0);
        } else {
            motorFor(d)->move_velocity(slots[d].velocity);
        }
    }
}

// -- Angle functions tied to a button --
static const pros::controller_digital_e_t kLiftButtons[LIFT_BUTTON_COUNT] = {
    pros::E_CONTROLLER_DIGITAL_R1, // index 0 = "no button", never read
    pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT,
    pros::E_CONTROLLER_DIGITAL_X,  pros::E_CONTROLLER_DIGITAL_B,    pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A,
};
static bool actPrevHeld[LIFT_MAX_ACTIONS];  // button state last tick (tap detection)
static bool actArmed[LIFT_MAX_ACTIONS];     // TAP: armed, waiting for the lift to reach the angle
static bool actHoldFired[LIFT_MAX_ACTIONS]; // HOLD: already fired during this press
static bool liftLock = false;               // an angle combo owns the lift until R1/R2 are released
static int liftLockButton = LIFT_BTN_NONE;

static void resetActionState() {
    for (int i = 0; i < LIFT_MAX_ACTIONS; i++) actPrevHeld[i] = actArmed[i] = actHoldFired[i] = false;
    liftLock = false;
    liftLockButton = LIFT_BTN_NONE;
}

static bool comboDrivesLift(int comboIdx) {
    if (comboIdx < 0 || comboIdx >= cfg.comboCount) return false;
    for (int j = 0; j < cfg.combos[comboIdx].stepCount; j++)
        if (cfg.combos[comboIdx].steps[j].device == LIFT_DEV_LIFT) return true;
    return false;
}

// Plays what an angle function is set to do: its combo, or its single step.
static void fireAction(const LiftAction &a, uint32_t nowMs) {
    bool drivesLift;
    if (a.combo >= 0 && a.combo < cfg.comboCount) {
        startCombo(a.combo, nowMs);
        drivesLift = comboDrivesLift(a.combo);
    } else {
        runStep(a.step, nowMs);
        drivesLift = a.step.device == LIFT_DEV_LIFT;
    }
    if (drivesLift) {
        liftLock = true;
        liftLockButton = a.button;
    }
}

bool liftTuningManualLiftBlocked() { return liftLock; }

bool liftTuningArmed() {
    for (int i = 0; i < cfg.actionCount; i++)
        if (actArmed[i]) return true;
    return false;
}

void liftTuningStep() {
    uint32_t nowMs = pros::millis();
    double pos = lift.get_position();
    mtx.take();

    // Angle functions: fire when the lift crosses their angle -- and, if one is tied to a button,
    // only while that button is held (HOLD) or after it was tapped (TAP). In the on-robot tuner the
    // buttons belong to the tuner, so button-gated functions wait for driver control.
    for (int i = 0; i < cfg.actionCount; i++) {
        const LiftAction &a = cfg.actions[i];
        bool gated = a.button > LIFT_BTN_NONE && !liftTunerActive;
        bool held = false;
        if (gated) {
            held = controller.get_digital(kLiftButtons[a.button]);
            if (a.mode == LIFT_MODE_TAP && held && !actPrevHeld[i]) actArmed[i] = !actArmed[i];
            if (!held) actHoldFired[i] = false;
            actPrevHeld[i] = held;
        }
        bool rising = lastPos < a.angle && pos >= a.angle;
        bool falling = lastPos > a.angle && pos <= a.angle;
        if (!((rising && a.when != LIFT_WHEN_FALLING) || (falling && a.when != LIFT_WHEN_RISING))) continue;
        if (a.button > LIFT_BTN_NONE) {
            if (!gated) continue;
            if (a.mode == LIFT_MODE_TAP) {
                if (!actArmed[i]) continue;
                actArmed[i] = false; // one shot: tap again to re-arm
            } else {
                if (!held || actHoldFired[i]) continue;
                actHoldFired[i] = true; // once per press
            }
        }
        fireAction(a, nowMs);
    }
    lastPos = pos;

    // The lift lock lifts once the combo is done and R1 / R2 / the trigger button are all let go.
    if (liftLock && !combo.active && !slots[LIFT_DEV_LIFT].active &&
        !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
        !(liftLockButton > LIFT_BTN_NONE && controller.get_digital(kLiftButtons[liftLockButton])))
        liftLock = false;

    // Arrived at a level (or gave up waiting) -> play its combo.
    if (targetActive && (std::fabs(pos - targetAngle) <= 8.0 || nowMs - targetSetMs > 4000)) {
        targetActive = false;
        if (comboOnArrival && curLevel >= 0 && curLevel < cfg.levelCount) startCombo(cfg.levels[curLevel].combo, nowMs);
        comboOnArrival = false;
    }

    // Combo: step i fires at start + sum of delays up to i.
    while (combo.active && (int32_t)(nowMs - combo.fireAt) >= 0) {
        const LiftCombo &cb = cfg.combos[combo.level];
        runStep(cb.steps[combo.next], nowMs);
        combo.next++;
        if (combo.next >= cb.stepCount) combo.active = false;
        else combo.fireAt += (uint32_t)std::clamp(cb.steps[combo.next].delayMs, 0, 20000);
    }

    serviceSlots(nowMs);
    mtx.give();
}

void liftPlayCombo(const LiftStep *steps, int count) {
    if (!steps || count <= 0) return;
    count = std::min(count, LIFT_MAX_STEPS);
    uint32_t at = pros::millis() + (uint32_t)std::clamp(steps[0].delayMs, 0, 20000);
    int next = 0;
    while (true) {
        uint32_t nowMs = pros::millis();
        mtx.take();
        while (next < count && (int32_t)(nowMs - at) >= 0) {
            runStep(steps[next], nowMs);
            next++;
            if (next < count) at += (uint32_t)std::clamp(steps[next].delayMs, 0, 20000);
        }
        serviceSlots(nowMs);
        bool busy = next < count;
        for (const MotorSlot &s : slots) busy = busy || s.active;
        mtx.give();
        if (!busy) break;
        pros::delay(10);
    }
}

// Generated by tools/lift_tuner.py ("Save"): one function per combo. Don't edit between the markers.
// LIFT-COMBO-FUNCS-BEGIN
// LIFT-COMBO-FUNCS-END

// === 3. On-robot tuner ======================================================
namespace {

// Requests from the laptop listener, consumed by the tuner loop.
struct Requests {
    bool go = false, stop = false, save = false;
    double goAngle = 0, jog = 0;
    bool jogPending = false;
    int testAct = -1, testCombo = -1, runLevel = -1, testStepLevel = -1, testStepIdx = -1;
    bool configChanged = false;
};
Requests req;
char statusMsg[40] = "";
uint32_t statusUntilMs = 0;
int savedState = 0; // 0 none, 1 saved, -1 failed

void setStatus(const char *msg) {
    snprintf(statusMsg, sizeof(statusMsg), "%s", msg);
    statusUntilMs = pros::millis() + 2500;
}

void listenerTask(void *) {
    char line[160];
    while (true) {
        if (!fgets(line, sizeof(line), stdin)) {
            pros::delay(10);
            continue;
        }
        if (strncmp(line, "LT ", 3) != 0) continue;
        double x;
        int i, j;
        mtx.take();
        if (strncmp(line, "LT BEGIN", 8) == 0) {
            pending = cfg;
        } else if (strncmp(line, "LT APPLY", 8) == 0) {
            sanitizeTuning(pending);
            cfg = pending;
            if (curLevel >= cfg.levelCount) curLevel = -1;
            combo.active = false;
            resetActionState(); // action indexes may have shifted: nothing stays armed
            req.configChanged = true;
        } else if (parseSettingLine(line, pending)) {
        } else if (sscanf(line, "LT GO %lf", &x) == 1) {
            req.go = true;
            req.goAngle = x;
        } else if (sscanf(line, "LT JOG %lf", &x) == 1) {
            req.jogPending = true;
            req.jog = x;
        } else if (strncmp(line, "LT STOP", 7) == 0) {
            req.stop = true;
        } else if (strncmp(line, "LT SAVE", 7) == 0) {
            req.save = true;
        } else if (sscanf(line, "LT TESTACT %d", &i) == 1) {
            req.testAct = i;
        } else if (sscanf(line, "LT TESTSTEP %d %d", &i, &j) == 2) {
            req.testStepLevel = i;
            req.testStepIdx = j;
        } else if (sscanf(line, "LT TESTCOMBO %d", &i) == 1) {
            req.testCombo = i;
        } else if (sscanf(line, "LT RUNLEVEL %d", &i) == 1) {
            req.runLevel = i;
        }
        mtx.give();
    }
}

// Appends to buf (size n) at offset o.
void put(char *buf, size_t n, size_t &o, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
void put(char *buf, size_t n, size_t &o, const char *fmt, ...) {
    if (o >= n) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + o, n - o, fmt, ap);
    va_end(ap);
    if (w > 0) o += (size_t)w;
}

void sendConfig(const LiftTuning &t) {
    static char buf[2200];
    size_t o = 0;
    put(buf, sizeof(buf), o, "LTC|%d,%.2f,%.2f|", t.startEnabled ? 1 : 0, t.startPosition, t.forwardLimit);
    for (int i = 0; i < t.comboCount; i++) {
        const LiftCombo &cb = t.combos[i];
        put(buf, sizeof(buf), o, "%s%s:", i ? ";" : "", cb.name);
        for (int j = 0; j < cb.stepCount; j++) {
            const LiftStep &s = cb.steps[j];
            put(buf, sizeof(buf), o, "%s%d,%d,%d,%d,%d", j ? "/" : "", s.delayMs, s.device, s.cmd, s.speed, s.durationMs);
        }
    }
    put(buf, sizeof(buf), o, "|");
    for (int i = 0; i < t.levelCount; i++)
        put(buf, sizeof(buf), o, "%s%s:%.2f:%d", i ? ";" : "", t.levels[i].name, t.levels[i].angle, t.levels[i].combo);
    put(buf, sizeof(buf), o, "|");
    for (int i = 0; i < t.actionCount; i++) {
        const LiftAction &a = t.actions[i];
        put(buf, sizeof(buf), o, "%s%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d", i ? ";" : "", a.angle, a.when, a.step.delayMs, a.step.device,
            a.step.cmd, a.step.speed, a.step.durationMs, a.combo, a.button, a.mode);
    }
    printf("%s\n", buf);
}

// -- Brain screen --
const uint32_t C_HEADER = 0x111620, C_DIVIDER = 0x222A36, C_TILE = 0x161C26, C_EDGE = 0x262F3C, C_TRACK = 0x2A323E;
const uint32_t C_TEXT = 0xE6EDF3, C_DIM = 0x8B949E, C_OK = 0x3FB950, C_WARM = 0xF0A830, C_HOT = 0xF85149, C_INFO = 0x58A6FF;
const uint32_t C_GOLD = 0xF2C230;

Canvas frame;

void drawScreen(const LiftTuning &t, double live, int sel, bool goalActive, bool comboRunning) {
    for (int y = 0; y < Canvas::H; y++) frame.fillRect(0, y, Canvas::W, y + 1, mixColor(0x0D1119, 0x07090D, y * 255 / (Canvas::H - 1)));
    frame.fillRect(0, 0, Canvas::W, 40, C_HEADER);
    frame.fillRect(0, 40, Canvas::W, 41, C_DIVIDER);
    frame.text(FONT_LARGE, 14, 7, "LIFT TUNER", C_TEXT);
    char buf[40];
    snprintf(buf, sizeof(buf), "%.1f\xC2\xB0", live);
    frame.text(FONT_LARGE, 466, 7, buf, goalActive ? C_WARM : C_OK, Canvas::RIGHT);
    if (comboRunning) frame.text(FONT_SMALL, 300, 14, "COMBO...", C_WARM, Canvas::RIGHT);

    // Gauge: where the lift is, with the start (gold), levels (blue) and limit (red) marked.
    double hi = std::max({t.forwardLimit, live, t.startPosition, 10.0});
    double lo = std::min({0.0, live, t.startPosition});
    for (int i = 0; i < t.levelCount; i++) hi = std::max(hi, t.levels[i].angle), lo = std::min(lo, t.levels[i].angle);
    hi *= 1.05;
    const int gx0 = 18, gx1 = 44, gy0 = 52, gy1 = 206;
    auto yOf = [&](double a) { return gy1 - (int)((a - lo) / (hi - lo) * (gy1 - gy0)); };
    frame.fillRoundRect(gx0, gy0, gx1, gy1, 5, C_TRACK);
    frame.fillRoundRect(gx0, std::min(yOf(live), gy1 - 4), gx1, gy1, 5, goalActive ? C_WARM : C_OK);
    for (int i = 0; i < t.levelCount; i++) frame.fillRect(gx0 - 6, yOf(t.levels[i].angle), gx1 + 6, yOf(t.levels[i].angle) + 2, C_INFO);
    frame.fillRect(gx0 - 6, yOf(t.forwardLimit), gx1 + 6, yOf(t.forwardLimit) + 2, C_HOT);
    if (t.startEnabled) frame.fillRect(gx0 - 8, yOf(t.startPosition) - 1, gx1 + 8, yOf(t.startPosition) + 2, C_GOLD);

    // Rows: START POSITION, then one per scoring level.
    const int rx0 = 74, rx1 = 470, rowH = 18, ry0 = 48;
    for (int r = 0; r <= t.levelCount; r++) {
        int y0 = ry0 + r * (rowH + 1), y1 = y0 + rowH;
        bool s = r == sel;
        frame.card(rx0, y0, rx1, y1, 5, s ? mixColor(C_TILE, C_INFO, 50) : C_TILE, s ? C_INFO : C_EDGE);
        if (r == 0) {
            frame.text(FONT_SMALL, rx0 + 10, y0 + 2, "START POSITION", t.startEnabled ? C_GOLD : C_DIM);
            snprintf(buf, sizeof(buf), t.startEnabled ? "%.1f" : "%.1f (off)", t.startPosition);
            frame.text(FONT_SMALL, rx1 - 10, y0 + 2, buf, t.startEnabled ? C_TEXT : C_DIM, Canvas::RIGHT);
        } else {
            const LiftLevel &l = t.levels[r - 1];
            frame.text(FONT_SMALL, rx0 + 10, y0 + 2, l.name, s ? C_TEXT : C_DIM);
            frame.text(FONT_SMALL, rx0 + 150, y0 + 2, l.combo >= 0 ? t.combos[l.combo].name : "no combo", C_DIM);
            snprintf(buf, sizeof(buf), "%.1f", l.angle);
            frame.text(FONT_SMALL, rx1 - 10, y0 + 2, buf, C_TEXT, Canvas::RIGHT);
        }
    }

    bool showMsg = pros::millis() < statusUntilMs;
    frame.text(FONT_SMALL, 10, 222, showMsg ? statusMsg : "UP/DN pick  A save here  X/B jog  Y go  R1 start on/off  R2 SD", showMsg ? C_OK : C_DIM);
    frame.present();
}

} // namespace

void liftTunerControl() {
    liftTunerActive = true;
    static pros::Task listener(listenerTask, nullptr, "lift tuner link");
    int sel = 0;
    bool goalActive = false;
    double goalAngle = 0;
    bool jogging = false;
    bool prevUp = false, prevDown = false, prevA = false, prevY = false, prevR1 = false, prevR2 = false;
    uint32_t lastStateMs = 0, lastConfigMs = 0, lastDrawMs = 0, lastPrintMs = 0;
    int printRow = 0;
    liftTuningBeginDriver(); // fresh executor state; the encoder stays as it is (the start position is not applied)
    setStatus("Tuner ready");

    while (true) {
        if (driverMenuLeaveRequested()) { // top-right corner of the screen held: back to the driver menu
            lift.move_velocity(0);
            mtx.take();
            combo.active = false;
            targetActive = false;
            for (MotorSlot &s : slots) {
                if (s.active) motorFor((int)(&s - slots))->move_velocity(0);
                s.active = false;
            }
            mtx.give();
            liftTunerActive = false;
            controller.rumble("-");
            return;
        }
        uint32_t nowMs = pros::millis();
        double live = lift.get_position();

        // -- Laptop requests --
        mtx.take();
        Requests r = req;
        req = Requests{};
        mtx.give();
        if (r.stop) {
            goalActive = false;
            lift.move_velocity(0);
            mtx.take();
            combo.active = false;
            targetActive = false;
            for (MotorSlot &s : slots) {
                if (s.active) motorFor((int)(&s - slots))->move_velocity(0);
                s.active = false;
            }
            mtx.give();
        }
        if (r.jogPending) {
            r.go = true;
            r.goAngle = live + r.jog;
        }
        if (r.go) {
            goalActive = true;
            goalAngle = r.goAngle;
            mtx.take();
            targetActive = false;
            mtx.give();
            lift.move_absolute(goalAngle, 150);
        }
        mtx.take();
        if (r.testAct >= 0 && r.testAct < cfg.actionCount) {
            const LiftAction &ta = cfg.actions[r.testAct];
            if (ta.combo >= 0 && ta.combo < cfg.comboCount) startCombo(ta.combo, nowMs);
            else runStep(ta.step, nowMs);
        }
        if (r.testStepLevel >= 0 && r.testStepLevel < cfg.comboCount && r.testStepIdx >= 0 && r.testStepIdx < cfg.combos[r.testStepLevel].stepCount)
            runStep(cfg.combos[r.testStepLevel].steps[r.testStepIdx], nowMs);
        if (r.testCombo >= 0) startCombo(r.testCombo, nowMs);
        if (r.runLevel >= 0 && r.runLevel < cfg.levelCount) {
            goalActive = false;
            goToLevel(r.runLevel, true, nowMs);
        }
        mtx.give();
        if (r.save) {
            bool ok = saveToSd();
            savedState = ok ? 1 : -1;
            setStatus(ok ? "Saved to SD card" : "No SD card!");
            lastConfigMs = 0;
        }
        if (r.configChanged) {
            lastConfigMs = 0;
            setStatus("Applied from laptop");
        }
        if (goalActive && std::fabs(live - goalAngle) < 3.0) goalActive = false;

        // -- Controller --
        mtx.take();
        int rows = cfg.levelCount + 1;
        mtx.give();
        bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool a = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool jogUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool jogDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (up && !prevUp) sel = (sel + rows - 1) % rows;
        if (down && !prevDown) sel = (sel + 1) % rows;
        sel = std::min(sel, rows - 1);

        if (jogUp || jogDown) {
            jogging = true;
            goalActive = false;
            mtx.take();
            targetActive = false;
            mtx.give();
            lift.move_velocity(jogUp ? 60 : -60);
        } else if (jogging) {
            jogging = false;
            lift.move_velocity(0);
        }
        if (a && !prevA) { // save the live angle into the selected row
            mtx.take();
            if (sel == 0) {
                cfg.startPosition = live;
                cfg.startEnabled = true;
            } else if (sel - 1 < cfg.levelCount) {
                cfg.levels[sel - 1].angle = live;
            }
            mtx.give();
            lastConfigMs = 0;
            setStatus(sel == 0 ? "Start position set" : "Level angle set");
        }
        if (y && !prevY) { // go to the selected row's angle
            mtx.take();
            goalAngle = sel == 0 ? cfg.startPosition : cfg.levels[std::min(sel - 1, cfg.levelCount - 1)].angle;
            targetActive = false;
            mtx.give();
            goalActive = true;
            lift.move_absolute(goalAngle, 150);
        }
        if (r1 && !prevR1) {
            mtx.take();
            cfg.startEnabled = !cfg.startEnabled;
            mtx.give();
            lastConfigMs = 0;
            setStatus(cfg.startEnabled ? "Start position ON" : "Start position OFF");
        }
        if (r2 && !prevR2) {
            bool ok = saveToSd();
            savedState = ok ? 1 : -1;
            setStatus(ok ? "Saved to SD card" : "No SD card!");
        }
        prevUp = up, prevDown = down, prevA = a, prevY = y, prevR1 = r1, prevR2 = r2;

        liftTuningStep(); // angle functions, combos and timed steps work in the tuner too

        // -- Outputs --
        mtx.take();
        LiftTuning snap = cfg;
        bool comboRunning = combo.active;
        int level = curLevel;
        mtx.give();
        if (nowMs - lastStateMs >= 150) {
            lastStateMs = nowMs;
            printf("LTL|%.2f,%d,%d,%d,%d,%d\n", live, goalActive || targetActive ? 1 : 0, sel, level, comboRunning ? 1 : 0, savedState);
        }
        if (nowMs - lastConfigMs >= 1000) {
            lastConfigMs = nowMs;
            sendConfig(snap);
        }
        if (nowMs - lastDrawMs >= 100) {
            lastDrawMs = nowMs;
            drawScreen(snap, live, sel, goalActive || targetActive, comboRunning);
        }
        if (nowMs - lastPrintMs >= 120) { // controller screen: alternate two rows (row 0 is fieldctl's)
            lastPrintMs = nowMs;
            if (printRow == 0) controller.print(1, 0, "Lift %7.1f      ", live);
            else if (sel == 0) controller.print(2, 0, "> START %7.1f  ", snap.startPosition);
            else controller.print(2, 0, "> %-6s %7.1f ", snap.levels[std::min(sel - 1, snap.levelCount - 1)].name,
                                   snap.levels[std::min(sel - 1, snap.levelCount - 1)].angle);
            printRow ^= 1;
        }
        pros::delay(20);
    }
}
