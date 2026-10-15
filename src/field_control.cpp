#include "field_control.hpp"
#include "autonomous.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>

// See field_control.hpp for the protocol and the behavior.

// The relay sends ~4 lines/s; the radio drops one now and then, so allow
// several misses before deciding the laptop is gone.
static const uint32_t kHeartbeatTimeoutMs = 2000;
static const uint32_t kStatusIntervalMs = 500;

struct FieldState {
  bool engaged = false;
  bool autoPeriod = false;
  bool enabled = false;
  int secsLeft = 0;
  int runId = 0;
  uint32_t lastMsgMs = 0;
};
static FieldState fieldState;
static pros::Mutex fieldMutex;

static FieldState snapshot() {
  fieldMutex.take();
  FieldState s = fieldState;
  fieldMutex.give();
  return s;
}

static bool signalLost(const FieldState &s) {
  return pros::millis() - s.lastMsgMs > kHeartbeatTimeoutMs;
}

bool fieldControlEngaged() { return snapshot().engaged; }

bool fieldControlDriverAllowed() {
  FieldState s = snapshot();
  if (!s.engaged)
    return true;
  return s.enabled && !s.autoPeriod && !signalLost(s);
}

bool fieldControlHandleLine(const char *line) {
  if (strncmp(line, "FC ", 3) != 0)
    return false;
  char period;
  int en, secs, run;
  if (strncmp(line + 3, "OFF", 3) == 0) {
    fieldMutex.take();
    fieldState.engaged = false;
    fieldState.enabled = false;
    fieldMutex.give();
  } else if (sscanf(line + 3, "%c %d %d %d", &period, &en, &secs, &run) == 4 &&
             (period == 'A' || period == 'D')) {
    fieldMutex.take();
    fieldState.engaged = true;
    fieldState.autoPeriod = period == 'A';
    fieldState.enabled = en != 0;
    fieldState.secsLeft = secs;
    fieldState.runId = run;
    fieldState.lastMsgMs = pros::millis();
    fieldMutex.give();
  }
  return true;
}

// Same "off" every mechanism uses when its button is released in
// jawheadControl() -- the lift and arm hold (brake mode), the rest stop.
static void stopAllMotors() {
  left_motor_group.move(0);
  right_motor_group.move(0);
  intake1.move_velocity(0);
  intake2.move_velocity(0);
  bunchy.move_velocity(0);
  lift1.move_velocity(0);
  lift2.move_velocity(0);
  bunchArm.move_velocity(0);
}

// ─── Autonomous task ───────────────────────────────────────────────────────
// runAutonomous() runs in its own task so a disable can kill it mid-routine,
// the same way PROS kills the autonomous task when a real field disables.
// LemLib runs each motion in a task of its own (async), so cancelling the
// motions after removing ours also stops whatever was mid-move.
static std::unique_ptr<pros::Task> autonTask;
static std::atomic<bool> autonRunning{false};
static int autonRunId = -1; // the runId the last autonomous was started for

static void startAuton(int runId) {
  autonRunId = runId;
  autonRunning = true;
  printf("FIELD: autonomous start (run %d)\n", runId);
  autonTask = std::make_unique<pros::Task>(
      []() {
        runAutonomous();
        autonRunning = false;
      },
      "field auton");
}

static void killAuton() {
  if (autonRunning) {
    autonTask->remove();
    autonRunning = false;
    printf("FIELD: autonomous killed\n");
  }
  autonTask.reset();
  chassis.cancelAllMotions();
  pros::delay(20); // let LemLib's motion task see the cancel and let go
  stopAllMotors();
}

// ─── Supervisor ───────────────────────────────────────────────────────────
static void describe(const FieldState &s, char *out, size_t n) {
  if (!s.engaged) {
    snprintf(out, n, "PRACTICE          ");
  } else if (signalLost(s)) {
    snprintf(out, n, "NO FIELD SIGNAL   ");
  } else {
    // The controller line fits ~19 characters.
    snprintf(out, n, "%-6s %d:%02d %-3s    ", s.autoPeriod ? "AUTO" : "DRIVER", s.secsLeft / 60,
             s.secsLeft % 60, s.enabled ? "ON" : "OFF");
  }
}

static void supervisorTask() {
  bool wasLive = false;    // engaged + enabled + signal, last tick
  bool wasEngaged = false;
  char shown[24] = "";     // what the controller screen says right now
  uint32_t lastStatusMs = 0;

  while (true) {
    FieldState s = snapshot();
    bool live = s.engaged && s.enabled && !signalLost(s);
    bool wantAuton = live && s.autoPeriod;

    // Autonomous: kill it when it shouldn't run (disabled, driver period,
    // or the gamemaster restarted the period), start it once per runId.
    if (autonTask && (!wantAuton || s.runId != autonRunId))
      killAuton();
    if (wantAuton && !autonTask && s.runId != autonRunId)
      startAuton(s.runId);

    // Losing the right to move (disable, timeout, release) stops everything
    // and cancels any motion in flight (a plan or tuner run started from the
    // menu before the gamemaster took over).
    if ((wasLive && !live) || (wasEngaged && !s.engaged) || (!wasEngaged && s.engaged && !live)) {
      if (autonTask)
        killAuton();
      chassis.cancelAllMotions();
      stopAllMotors();
      printf("FIELD: %s\n", s.engaged ? "disabled" : "released");
    }
    // While disabled, keep holding everything off: anything still running
    // from before (or a laptop command that slipped in) is stopped within
    // one tick rather than only once.
    if (s.engaged && !live)
      stopAllMotors();
    wasLive = live;
    wasEngaged = s.engaged;

    if (s.engaged || shown[0]) {
      char text[24];
      describe(s, text, sizeof(text));
      // The controller only takes one screen update per 50 ms; retry next
      // tick if this one was refused.
      if (strcmp(text, shown) != 0 && controller.print(0, 0, "%s", text) == 1)
        snprintf(shown, sizeof(shown), "%s", text);
    }

    uint32_t now = pros::millis();
    if (now - lastStatusMs >= kStatusIntervalMs) {
      lastStatusMs = now;
      if (s.engaged)
        printf("FCS|%c|%d|%d|%d\n", s.autoPeriod ? 'A' : 'D', live ? 1 : 0, autonRunning ? 1 : 0,
               s.runId);
      else // program is up and listening, just not under field control yet
        printf("FCS|P|0|0|0\n");
    }

    pros::delay(10);
  }
}

void fieldControlInit() {
  static bool started = false;
  if (started)
    return;
  started = true;
  pros::Task(supervisorTask, "field control");
}
