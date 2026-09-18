// ─────────────────────────────────────────────────────────────────────────────
// AUTOMATIC PID TUNER
// ─────────────────────────────────────────────────────────────────────────────
// A relay-style hand tuning recipe, automated: raise kP until the robot
// oscillates, add kD until it's smooth, add kI if an overshoot / steady-state
// error remains, verify. Every run is the same motion the manual tuner's
// small test does (90 deg turn / 24 in drive) with the direction alternating
// run to run so the robot ends up close to where it started.
//
// CONTROLS
//   touch ANGULAR / LATERAL   pick the controller (idle only); Y does too
//   touch START / A           start the tune (IMU calibration first, ~3 s)
//   touch STOP / X            abort the tune; the best gains so far stay
//   touch BACK                leave to the driver menu (aborts a tune)
//   laptop                    PID AUTOSTART m / PID AUTOMODE m / PID STOP
//
// HOW EACH RUN IS JUDGED (classify())
//   The sampling loop records, besides the manual tuner's overshoot / settle
//   / final error: how many times the error changed sign (crossings), how
//   many times the trace changed direction (reversals), and how far it got
//   at all (peakTravel). From those, one verdict per run:
//     no_motion    barely moved (< 10 % of the move)        -> more kP, fast
//     under        never reached the target                 -> more kP
//     bounce       overshot, but no sustained oscillation   -> a bit more kP
//     oscillating  crossed the target >= 2 times with an
//                  overshoot >= max(2.5 bands, 8 % of move) -> this is Pu
//     smooth       overshoot within the settle band         -> kD is enough
//   Phase-specific decisions sit in the run*Phase() functions below.
// ─────────────────────────────────────────────────────────────────────────────

#include "auto_tuner.hpp"
#include "driver_menu.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pid_tuner_shared.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

bool autoTunerActive = false;

// ─── Tune state ──────────────────────────────────────────────────────────────
enum class Phase { IDLE, P_SEARCH, D_SEARCH, I_SEARCH, VERIFY, DONE, ABORTED, FAILED };
static const char *kPhaseNames[] = {"IDLE",   "P-search", "D-search", "I-search",
                                    "VERIFY", "DONE",     "ABORTED",  "FAILED"};

struct TuneState {
  bool angular = true;
  Phase phase = Phase::IDLE;
  int run = 0; // run counter across the whole tune (1-based in the stream)
  float pu = 0, bestD = 0, bestI = 0;
  bool running = false;
  char msg[96] = "pick a controller, clear the area, tap START";
};
static TuneState st;

// STOP aborts the tune but stays on this screen; BACK also leaves it.
static bool stopRequested = false;
static bool exitRequested = false;

// Per-controller sizes, limits and step rules.
struct ModeParams {
  float testSize;      // the run everything is judged on (90 deg / 24 in)
  float verifySize;    // the final check (180 deg / 48 in)
  int timeoutMs, verifyTimeoutMs;
  float band;          // settle band (kAngularSettleBand / kLateralSettleBand)
  float pStart, pMax;  // kP search starts here, gives up past pMax
  float dStepFrac;     // first kD step, as a fraction of Pu
  float dMaxFrac;      // give up on kD past Pu * dMaxFrac
  float iStep, iMax;   // first kI step / cap
};
static const ModeParams kAngularParams = {90, 180, 2500, 3000, kAngularSettleBand,
                                          1.0f, 60.0f, 0.5f, 15.0f, 0.0005f, 0.05f};
static const ModeParams kLateralParams = {24, 48, 3500, 5000, kLateralSettleBand,
                                          2.0f, 100.0f, 0.5f, 15.0f, 0.002f, 0.2f};
static const ModeParams &params() { return st.angular ? kAngularParams : kLateralParams; }
static GainSet &gains() { return st.angular ? angularGains : lateralGains; }

static const int kMaxPRuns = 30, kMaxDRuns = 24, kMaxIRuns = 16;

// ─── Run metrics ─────────────────────────────────────────────────────────────
struct RunMetrics {
  float overshoot = 0;   // worst |error| after the first crossing
  int settleMs = -1;     // first time |error| stayed in-band for kSettleHoldMs
  float finalError = 0;
  int durationMs = 0;
  int crossings = 0;     // error sign changes (with half-a-band hysteresis)
  int reversals = 0;     // direction changes of the trace
  float peakTravel = 0;  // furthest the trace got from the start
  bool cancelled = false;
};

enum Verdict { V_NO_MOTION, V_UNDER, V_BOUNCE, V_OSCILLATING, V_SMOOTH, V_CANCELLED };
static const char *kVerdictNames[] = {"no_motion", "under",  "bounce",
                                      "oscillating", "smooth", "cancelled"};

static RunMetrics lastMetrics;
static Verdict lastVerdict = V_SMOOTH;
static char lastLabel[24] = "none";

static Verdict classify(const RunMetrics &m, float move) {
  const ModeParams &mp = params();
  if (m.cancelled)
    return V_CANCELLED;
  if (m.peakTravel < 0.10f * move)
    return V_NO_MOTION;
  if (m.crossings == 0)
    return V_UNDER;
  float oscTol = std::max(2.5f * mp.band, 0.08f * move);
  if (m.crossings >= 2 && m.overshoot >= oscTol)
    return V_OSCILLATING;
  if (m.overshoot > mp.band)
    return V_BOUNCE;
  return V_SMOOTH;
}

// ─── Laptop mirror (tools/auto_pid_tuner.py via tools/remote_touch.py) ──────
// Same shape as the manual tuner's sendTunerState(): one RUI line whenever
// something changed (throttled), else a 1 s heartbeat. See auto_tuner.hpp
// for the AUTO / MSG segments.
static const int kModeAngX0 = 10, kModeAngY0 = 38, kModeAngX1 = 78, kModeAngY1 = 66;
static const int kModeLatX0 = 82, kModeLatY0 = 38, kModeLatX1 = 150, kModeLatY1 = 66;
static const int kStartX0 = 10, kStartY0 = 72, kStartX1 = 150, kStartY1 = 104;

static void sendAutoState(bool force = false) {
  static uint32_t lastSend = 0, lastActualSend = 0;
  static char lastLine[512] = "";
  const uint32_t kHeartbeatMs = 1000;
  uint32_t interval = remoteSendIntervalMs();
  if (!force && pros::millis() - lastSend < interval)
    return;
  lastSend = pros::millis();

  char line[512];
  snprintf(line, sizeof(line),
           "RUI|AUTO_TUNER|%s|BTN:%d,%d,%d,%d,BACK;%d,%d,%d,%d,ANGULAR;%d,%d,%d,%d,LATERAL;"
           "%d,%d,%d,%d,%s|FIELDS:|FOOTER:|CODE:|MOTORS:|SCROLL:|STEP:0|FOOTC:0|PLOT:|TRAIL:"
           "|SEL:0|ACTIVE:-1,-1|PLAN:|PATH:"
           "|AUTO:%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d|MSG:%s"
           "|RES:%s,%.2f,%d,%.2f,%d",
           st.angular ? "ANGULAR" : "LATERAL", kBackX0, kBackY0, kBackX1, kBackY1, kModeAngX0,
           kModeAngY0, kModeAngX1, kModeAngY1, kModeLatX0, kModeLatY0, kModeLatX1, kModeLatY1,
           kStartX0, kStartY0, kStartX1, kStartY1, st.running ? "STOP" : "START",
           st.angular ? 0 : 1, (int)st.phase, st.run, angularGains.gains[0],
           angularGains.gains[1], angularGains.gains[2], lateralGains.gains[0],
           lateralGains.gains[1], lateralGains.gains[2], st.pu, st.bestD, st.bestI,
           st.running ? 1 : 0, st.msg, lastLabel, lastMetrics.overshoot, lastMetrics.settleMs,
           lastMetrics.finalError, lastMetrics.durationMs);
  bool changed = strcmp(line, lastLine) != 0;
  if (!force && !changed && pros::millis() - lastActualSend < kHeartbeatMs)
    return;
  strcpy(lastLine, line);
  lastActualSend = pros::millis();
  printf("%s\n", line);
}

// ─── Brain screen ────────────────────────────────────────────────────────────
static void drawFooter() {
  using namespace ui;
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 215, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(st.phase == Phase::FAILED || st.phase == Phase::ABORTED ? RED
                        : st.phase == Phase::DONE                                 ? GREEN
                                                                                  : CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", st.msg);
  pros::screen::set_eraser(BG);
}

static void drawButton(int x0, int y0, int x1, int y1, const char *label, int fill, int fg) {
  pros::screen::set_pen(fill);
  pros::screen::fill_rect(x0, y0, x1, y1);
  pros::screen::set_pen(ui::AXIS);
  pros::screen::draw_rect(x0, y0, x1, y1);
  pros::screen::set_eraser(fill);
  pros::screen::set_pen(fg);
  pros::screen::print(pros::E_TEXT_SMALL, x0 + 8, y0 + (y1 - y0) / 2 - 6, "%s", label);
  pros::screen::set_eraser(ui::BG);
}

// Left panel below the buttons: phase / run and the live gains.
static void drawInfoPanel() {
  using namespace ui;
  const GainSet &g = gains();
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(10, 108, 155, 212);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(st.running ? GREEN : GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 110, "%s  run %d", kPhaseNames[(int)st.phase],
                      st.run);
  const int hi = st.angular ? CYAN : ORANGE;
  pros::screen::set_pen(st.phase == Phase::P_SEARCH ? hi : 0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 126, "kP %.3f", g.gains[0]);
  pros::screen::set_pen(st.phase == Phase::I_SEARCH ? hi : 0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 140, "kI %.4f", g.gains[1]);
  pros::screen::set_pen(st.phase == Phase::D_SEARCH ? hi : 0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 154, "kD %.3f", g.gains[2]);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 172, "Pu %.3f", st.pu);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 186, "best D %.3f", st.bestD);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 200, "best I %.4f", st.bestI);
}

static void drawButtons() {
  using namespace ui;
  drawButton(kModeAngX0, kModeAngY0, kModeAngX1, kModeAngY1, "ANGULAR",
             st.angular ? SETTLE_BAND_BG : CARD, st.angular ? CYAN : GRAY);
  drawButton(kModeLatX0, kModeLatY0, kModeLatX1, kModeLatY1, "LATERAL",
             st.angular ? CARD : SETTLE_BAND_BG, st.angular ? GRAY : ORANGE);
  if (st.running)
    drawButton(kStartX0, kStartY0, kStartX1, kStartY1, "STOP  (X)", 0x3a1414, 0xFFFFFF);
  else
    drawButton(kStartX0, kStartY0, kStartX1, kStartY1, "START  (A)", 0x143a1e, 0xFFFFFF);
}

// Full redraw: header, buttons, info panel, empty graph, footer.
static void drawAutoUI() {
  using namespace ui;
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  pros::screen::set_eraser(BG);

  pros::screen::set_pen(st.angular ? CYAN : ORANGE);
  pros::screen::fill_rect(0, 0, 480, 30);
  pros::screen::set_eraser(st.angular ? CYAN : ORANGE);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 7, "AUTO PID // %s",
                      st.angular ? "ANGULAR (turns)" : "LATERAL (drive)");
  pros::screen::set_eraser(BG);
  drawBackButton();
  pros::screen::set_eraser(BG);

  drawButtons();
  drawInfoPanel();

  {
    float previewTarget = params().testSize;
    GraphScale sc = computeScale(0.0f, previewTarget);
    drawValueGraphFrame(sc);
    int x0[1] = {GX}, w[1] = {GW};
    float lvl[1] = {previewTarget};
    drawTargetSteps(sc, x0, w, lvl, 1);
    pros::screen::set_pen(GRAY);
    pros::screen::set_eraser(CARD);
    pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 200, GY + 2, "P -> D -> I -> verify");
    pros::screen::print(pros::E_TEXT_SMALL, GX + GW - 200, GY + 16, "graph: auto tuner window");
    pros::screen::set_eraser(BG);
  }
  drawFooter();
}

static void setMsg(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(st.msg, sizeof(st.msg), fmt, ap);
  va_end(ap);
  printf("AUTO TUNER: %s\n", st.msg);
  drawFooter();
  drawInfoPanel();
  sendAutoState(true);
}

// The controller LCD only accepts ~1 line per 50 ms, so lines rotate.
static void drawControllerUI() {
  static uint32_t lastDraw = 0;
  static int line = 0;
  if (pros::millis() - lastDraw < 120)
    return;
  lastDraw = pros::millis();
  const GainSet &g = gains();
  switch (line) {
  case 0:
    controller.print(0, 0, "AUTO %s %s r%d   ", st.angular ? "ANG" : "LAT",
                     kPhaseNames[(int)st.phase], st.run);
    break;
  case 1:
    controller.print(1, 0, "P%.2f I%.3f D%.1f ", g.gains[0], g.gains[1], g.gains[2]);
    break;
  default:
    controller.print(2, 0, "OS%.1f X%d St%d   ", lastMetrics.overshoot, lastMetrics.crossings,
                     lastMetrics.settleMs);
    break;
  }
  line = (line + 1) % 3;
}

// ─── Input ───────────────────────────────────────────────────────────────────
static bool wasTouched = false;
static EdgeButton btnA{pros::E_CONTROLLER_DIGITAL_A};
static EdgeButton btnX{pros::E_CONTROLLER_DIGITAL_X};
static EdgeButton btnY{pros::E_CONTROLLER_DIGITAL_Y};

static bool inBox(int x, int y, int x0, int y0, int x1, int y1) {
  return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

// One tap (physical, or from the laptop mirror) per press.
static bool takeTap(int &x, int &y) {
  pros::screen_touch_status_s_t s = pros::screen::touch_status();
  bool press = s.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
  if (s.touch_status == pros::E_TOUCH_PRESSED)
    wasTouched = true;
  else if (s.touch_status == pros::E_TOUCH_RELEASED)
    wasTouched = false;
  x = s.x;
  y = s.y;
  if (!press && takeRemoteTouch(x, y))
    press = true;
  return press;
}

// Called from inside every sampling loop and between runs: STOP (touch,
// X, laptop) aborts the tune; BACK aborts it and leaves the screen. Any
// other laptop command that arrives mid-tune is dropped so a queued START
// can't set the robot off again the moment this tune ends.
static void pollAbort() {
  if (takeRemotePidStop() || takeRemoteKey(RK_X) || btnX.pressed())
    stopRequested = true;
  int x, y;
  if (takeTap(x, y)) {
    if (inBox(x, y, kBackX0, kBackY0, kBackX1, kBackY1)) {
      stopRequested = true;
      exitRequested = true;
    } else if (inBox(x, y, kStartX0, kStartY0, kStartX1, kStartY1)) {
      stopRequested = true;
    }
  }
  RemotePidCommand c;
  while (takeRemotePidCommand(c)) {
  }
  btnA.pressed();
  btnY.pressed();
  takeRemoteKey(RK_A);
  takeRemoteKey(RK_Y);
}

// ─── One run ─────────────────────────────────────────────────────────────────
// Zeroes the pose, applies the current gains, runs one turn/drive to
// `target` and samples it at kSampleDelayMs, plotting the trace on the
// brain graph and streaming AT|S samples to the laptop. Returns the raw
// metrics; the caller classifies them and calls finishRun().
static RunMetrics runMove(float target, int timeoutMs, const char *label) {
  const ModeParams &mp = params();
  const bool angular = st.angular;
  const GainSet &g = gains();
  st.run++;

  zeroPoseForRun();
  applyGains(angular);
  const float startVal = 0; // pose was just zeroed
  printf("AT|RUN|%d|%d|%d|%.4f|%.4f|%.4f|%s|%.2f|%.2f|%d\n", st.run, angular ? 1 : 0,
         (int)st.phase, g.gains[0], g.gains[1], g.gains[2], label, target, startVal, timeoutMs);
  drawInfoPanel();
  sendAutoState(true);

  if (angular) {
    chassis.turnToHeading(target, timeoutMs, {}, true);
  } else {
    // A negative target is behind the chassis: drive there backwards
    // rather than turning round to face it (see the manual tuner).
    chassis.moveToPoint(0, target, timeoutMs, {.forwards = target >= 0}, true);
  }
  // isInMotion() can briefly read false right after an async motion is
  // issued, before its task has flagged itself running.
  pros::delay(10);

  GraphScale sc = computeScale(startVal, target);
  drawValueGraphFrame(sc);
  int stepX0[1] = {ui::GX}, stepW[1] = {ui::GW};
  float stepLevel[1] = {target};
  drawTargetSteps(sc, stepX0, stepW, stepLevel, 1);
  pros::screen::set_pen(ui::GRAY);
  pros::screen::set_eraser(ui::CARD);
  pros::screen::print(pros::E_TEXT_SMALL, ui::GX + 4, ui::GY + 2, "run %d  %s  P%.2f I%.4f D%.2f",
                      st.run, label, g.gains[0], g.gains[1], g.gains[2]);
  pros::screen::set_eraser(ui::BG);

  RunMetrics m;
  const uint32_t start = pros::millis();
  const float initialError = angular ? wrap180(target - startVal) : (target - startVal);
  int errSign = initialError >= 0 ? 1 : -1;
  bool crossed = false;
  int inBandMs = 0;
  float prevActual = startVal;
  int velSign = 0;
  // Direction changes smaller than this per sample are odometry noise, not
  // the robot reversing.
  const float velEps = angular ? 0.35f : 0.12f;
  int prevX = ui::GX, prevY = sc.toY(startVal);
  const int csvEvery = csvPeriodMs();
  uint32_t lastCsv = 0;
  bool firstSample = true;

  while (chassis.isInMotion()) {
    pollAbort();
    if (stopRequested) {
      chassis.cancelMotion();
      m.cancelled = true;
    }
    lemlib::Pose pose = chassis.getPose();
    float actualRaw = angular ? pose.theta : pose.y;
    float error = angular ? wrap180(target - actualRaw) : (target - actualRaw);
    // Wrap-corrected actual, so an angular trace never jumps across +-180.
    float actual = target - error;
    uint32_t t = pros::millis() - start;

    if (std::abs(error) > mp.band * 0.5f) {
      int s = error > 0 ? 1 : -1;
      if (s != errSign) {
        errSign = s;
        m.crossings++;
        crossed = true;
      }
    }
    if (crossed)
      m.overshoot = std::max(m.overshoot, std::abs(error));

    if (std::abs(error) < mp.band) {
      inBandMs += kSampleDelayMs;
      if (inBandMs >= kSettleHoldMs && m.settleMs < 0)
        m.settleMs = (int)t - kSettleHoldMs;
    } else {
      inBandMs = 0;
      m.settleMs = -1; // must stay settled: re-arm if it leaves the band
    }
    m.finalError = error;
    m.peakTravel = std::max(m.peakTravel, std::abs(actual - startVal));

    float v = actual - prevActual;
    prevActual = actual;
    if (std::abs(v) > velEps) {
      int s = v > 0 ? 1 : -1;
      if (velSign != 0 && s != velSign)
        m.reversals++;
      velSign = s;
    }

    int x = std::clamp(ui::GX + (int)((float)t / timeoutMs * ui::GW), ui::GX,
                       ui::GX + ui::GW - 1);
    int y = sc.toY(actual);
    pros::screen::set_pen(ui::ACTUAL_LINE);
    pros::screen::draw_line(prevX, prevY, x, y);
    prevX = x;
    prevY = y;

    if (firstSample || t - lastCsv >= (uint32_t)csvEvery) {
      firstSample = false;
      lastCsv = t;
      printf("AT|S|%lu|%.2f|%.2f|%.2f\n", (unsigned long)t, target, actualRaw, error);
    }
    sendAutoState(); // heartbeat for the laptop while the loop is busy
    pros::delay(kSampleDelayMs);
  }
  chassis.waitUntilDone();
  m.durationMs = (int)(pros::millis() - start);
  return m;
}

// Records a run's verdict: laptop AT|END line, brain footer, controller.
static void finishRun(const RunMetrics &m, Verdict v, const char *label) {
  lastMetrics = m;
  lastVerdict = v;
  snprintf(lastLabel, sizeof(lastLabel), "%s", label);
  printf("AT|END|%d|%.2f|%d|%.2f|%d|%d|%d|%s\n", st.run, m.overshoot, m.settleMs, m.finalError,
         m.durationMs, m.crossings, m.reversals, kVerdictNames[v]);
  printf("RESULT run %d %s: overshoot=%.2f settle=%dms final=%.2f crossings=%d reversals=%d "
         "-> %s\n",
         st.run, label, m.overshoot, m.settleMs, m.finalError, m.crossings, m.reversals,
         kVerdictNames[v]);
  controller.rumble(".");
  const GainSet &g = gains();
  // No '|' in here: the laptop splits the RUI line on it (see MSG segment).
  setMsg("run %d %s: os %.1f x%d st%d -> %s // P%.2f I%.4f D%.2f", st.run, label, m.overshoot,
         m.crossings, m.settleMs, kVerdictNames[v], g.gains[0], g.gains[1], g.gains[2]);
}

// Let the chassis come to rest between runs, still listening for STOP.
static void pauseBetweenRuns() {
  uint32_t until = pros::millis() + 500;
  while (pros::millis() < until && !stopRequested) {
    pollAbort();
    sendAutoState();
    pros::delay(20);
  }
}

// Alternate the direction every run so the robot stays roughly put.
static float nextTarget(float size) { return (st.run % 2 == 0) ? size : -size; }

// One judged run with the current gains: run, classify, record, pause.
static Verdict judgedRun(float size, int timeoutMs, const char *label, RunMetrics &out) {
  out = runMove(nextTarget(size), timeoutMs, label);
  Verdict v = classify(out, size);
  finishRun(out, v, label);
  if (!stopRequested)
    pauseBetweenRuns();
  return v;
}

// ─── Phase 1: kP ─────────────────────────────────────────────────────────────
// Everything zeroed, kP raised run by run (x1.6 while the robot doesn't
// even move, x1.3 while it falls short, x1.15 once it's bouncing) until the
// response oscillates. One bisection step back then checks whether the
// midpoint between the last non-oscillating kP and this one oscillates too,
// so a 30 % jump doesn't overshoot the true threshold by much.
static bool runPPhase() {
  const ModeParams &mp = params();
  GainSet &g = gains();
  st.phase = Phase::P_SEARCH;
  g.gains[0] = mp.pStart;
  g.gains[1] = 0;
  g.gains[2] = 0;
  st.pu = st.bestD = st.bestI = 0;
  setMsg("P-search: raising kP until the robot oscillates");

  float prevP = 0;
  RunMetrics m;
  for (int k = 0; k < kMaxPRuns && !stopRequested; k++) {
    Verdict v = judgedRun(mp.testSize, mp.timeoutMs, "P", m);
    if (stopRequested)
      return false;
    if (v == V_OSCILLATING) {
      float hiP = g.gains[0];
      st.pu = hiP;
      if (prevP > 0) {
        g.gains[0] = 0.5f * (prevP + hiP);
        setMsg("oscillating at kP %.2f -- checking the midpoint %.2f", hiP, g.gains[0]);
        Verdict v2 = judgedRun(mp.testSize, mp.timeoutMs, "P-mid", m);
        if (stopRequested)
          return false;
        st.pu = (v2 == V_OSCILLATING) ? g.gains[0] : hiP;
      }
      g.gains[0] = st.pu;
      setMsg("kP recorded: Pu = %.3f (oscillating)", st.pu);
      return true;
    }
    if (g.gains[0] >= mp.pMax) {
      st.phase = Phase::FAILED;
      setMsg("FAILED: kP %.1f without oscillating -- check the drivetrain", g.gains[0]);
      return false;
    }
    prevP = g.gains[0];
    float grow = v == V_NO_MOTION ? 1.6f : v == V_UNDER ? 1.3f : 1.15f;
    g.gains[0] = std::min(mp.pMax, g.gains[0] * grow);
  }
  st.phase = Phase::FAILED;
  setMsg("FAILED: no oscillation after %d runs", kMaxPRuns);
  return false;
}

// ─── Phase 2: kD ─────────────────────────────────────────────────────────────
// kP stays at Pu; kD rises by a step (Pu / 2 to start, doubled whenever a
// step made no visible difference) until the response is smooth. If more
// kD stops reducing the overshoot three runs in a row, or the trace starts
// chattering (lots of small direction changes), the best kD so far is kept.
static bool runDPhase(RunMetrics &bestM) {
  const ModeParams &mp = params();
  GainSet &g = gains();
  st.phase = Phase::D_SEARCH;
  float dStep = st.pu * mp.dStepFrac;
  float dMax = st.pu * mp.dMaxFrac;
  float bestOvershoot = lastMetrics.overshoot; // the Pu run
  float prevOvershoot = bestOvershoot;
  bestM = lastMetrics;
  st.bestD = 0;
  int worse = 0;
  setMsg("D-search: raising kD until the response is smooth");

  RunMetrics m;
  for (int k = 0; k < kMaxDRuns && !stopRequested; k++) {
    g.gains[2] = std::min(dMax, g.gains[2] + dStep);
    Verdict v = judgedRun(mp.testSize, mp.timeoutMs, "D", m);
    if (stopRequested)
      return false;
    if (v == V_SMOOTH) {
      st.bestD = g.gains[2];
      bestM = m;
      setMsg("smooth at kD %.3f", st.bestD);
      return true;
    }
    if (m.reversals > 10 && v != V_OSCILLATING) {
      setMsg("kD %.3f chatters (%d reversals) -- keeping kD %.3f", g.gains[2], m.reversals,
             st.bestD);
      break;
    }
    if (m.overshoot < bestOvershoot - 1e-3f) {
      bestOvershoot = m.overshoot;
      st.bestD = g.gains[2];
      bestM = m;
      worse = 0;
    } else if (++worse >= 3) {
      setMsg("more kD stopped helping -- keeping kD %.3f", st.bestD);
      break;
    }
    if (g.gains[2] >= dMax) {
      setMsg("kD hit its cap -- keeping kD %.3f", st.bestD);
      break;
    }
    if (std::abs(m.overshoot - prevOvershoot) < 0.03f * mp.testSize)
      dStep *= 2; // this step didn't change anything; take bigger ones
    prevOvershoot = m.overshoot;
  }
  g.gains[2] = st.bestD;
  return !stopRequested;
}

// ─── Phase 3: kI ─────────────────────────────────────────────────────────────
// Only if the best kD run still overshoots or stops short of the band. kI
// rises by small steps (doubled when a step changed nothing) until both are
// inside the band; if it makes things worse two runs running, the best kI
// so far is kept.
static bool runIPhase(const RunMetrics &afterD) {
  const ModeParams &mp = params();
  GainSet &g = gains();
  st.phase = Phase::I_SEARCH;
  st.bestI = 0;
  bool needed = afterD.overshoot > mp.band || std::abs(afterD.finalError) > mp.band;
  if (!needed) {
    setMsg("no overshoot / steady error left -- kI stays 0");
    return true;
  }
  setMsg("I-search: overshoot %.2f final %.2f -- raising kI", afterD.overshoot,
         afterD.finalError);
  float iStep = mp.iStep;
  float bestScore = afterD.overshoot + std::abs(afterD.finalError);
  float prevScore = bestScore;
  int worse = 0;

  RunMetrics m;
  for (int k = 0; k < kMaxIRuns && !stopRequested; k++) {
    g.gains[1] = std::min(mp.iMax, g.gains[1] + iStep);
    judgedRun(mp.testSize, mp.timeoutMs, "I", m);
    if (stopRequested)
      return false;
    float score = m.overshoot + std::abs(m.finalError);
    if (m.overshoot <= mp.band && std::abs(m.finalError) <= mp.band) {
      st.bestI = g.gains[1];
      setMsg("overshoot faded at kI %.4f", st.bestI);
      return true;
    }
    if (score < bestScore - 1e-3f) {
      bestScore = score;
      st.bestI = g.gains[1];
      worse = 0;
    } else if (++worse >= 2) {
      setMsg("more kI made it worse -- keeping kI %.4f", st.bestI);
      break;
    }
    if (g.gains[1] >= mp.iMax) {
      setMsg("kI hit its cap -- keeping kI %.4f", st.bestI);
      break;
    }
    if (std::abs(score - prevScore) < 0.03f * mp.testSize)
      iStep *= 2;
    prevScore = score;
  }
  g.gains[1] = st.bestI;
  return !stopRequested;
}

// ─── The whole tune ──────────────────────────────────────────────────────────
static void calibrateBeforeTune() {
  setMsg("CALIBRATING -- keep the robot still (~3 s)");
  controller.print(0, 0, "CALIBRATING...  ");
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  controller.rumble("-");
}

static void runAutoTune() {
  const ModeParams &mp = params();
  GainSet &g = gains();
  st.running = true;
  st.run = 0;
  stopRequested = false;
  takeRemotePidStop(); // drop a stale STOP so it can't abort this tune
  takeRemoteKey(RK_X);
  drawButtons();
  sendAutoState(true);
  printf("AUTO TUNE START (%s)\n", st.angular ? "angular" : "lateral");
  controller.rumble("--");

  calibrateBeforeTune();

  RunMetrics afterD;
  bool ok = runPPhase() && runDPhase(afterD) && runIPhase(afterD);
  if (ok) {
    st.phase = Phase::VERIFY;
    g.gains[0] = st.pu;
    g.gains[1] = st.bestI;
    g.gains[2] = st.bestD;
    setMsg("verifying with the big move (%.0f)", mp.verifySize);
    RunMetrics m;
    Verdict v = judgedRun(mp.verifySize, mp.verifyTimeoutMs, "verify", m);
    if (!stopRequested) {
      st.phase = Phase::DONE;
      setMsg("DONE  kP %.3f  kI %.4f  kD %.3f  (verify: %s, os %.1f)", g.gains[0], g.gains[1],
             g.gains[2], kVerdictNames[v], m.overshoot);
    }
  }
  if (stopRequested) {
    st.phase = Phase::ABORTED;
    // Keep the best of what was found so far in the gain set.
    if (st.pu > 0)
      g.gains[0] = st.pu;
    g.gains[1] = st.bestI;
    g.gains[2] = st.bestD;
    setMsg("ABORTED after run %d -- keeping kP %.3f kI %.4f kD %.3f", st.run, g.gains[0],
           g.gains[1], g.gains[2]);
  }
  applyGains(st.angular); // the chassis drives with the result from now on
  printf("AT|DONE|%d|%.4f|%.4f|%.4f\n", (int)st.phase, g.gains[0], g.gains[1], g.gains[2]);
  printf("AUTO TUNE %s\n", kPhaseNames[(int)st.phase]);
  controller.rumble("---");
  st.running = false;
  drawButtons();
  drawInfoPanel();
  sendAutoState(true);
}

// ─── Main loop ───────────────────────────────────────────────────────────────
void autoTunerControl() {
  autoTunerActive = true;
  loadGainsFromChassisConfig();
  pros::delay(100); // let the HUD task finish its current frame
  st.phase = Phase::IDLE;
  st.running = false;
  st.run = 0;
  snprintf(st.msg, sizeof(st.msg), "pick a controller, clear the area, tap START");
  stopRequested = exitRequested = false;
  wasTouched = false;
  printf("AUTO TUNER ACTIVE\n");
  drawAutoUI();
  sendAutoState(true);

  while (true) {
    sendAutoState(); // throttled internally; keeps the laptop window live

    bool start = false;
    int x, y;
    if (takeTap(x, y)) {
      if (inBox(x, y, kBackX0, kBackY0, kBackX1, kBackY1)) {
        exitRequested = true;
      } else if (inBox(x, y, kModeAngX0, kModeAngY0, kModeAngX1, kModeAngY1)) {
        st.angular = true;
        drawAutoUI();
      } else if (inBox(x, y, kModeLatX0, kModeLatY0, kModeLatX1, kModeLatY1)) {
        st.angular = false;
        drawAutoUI();
      } else if (inBox(x, y, kStartX0, kStartY0, kStartX1, kStartY1)) {
        start = true;
      }
    }
    if (btnY.pressed() || takeRemoteKey(RK_Y)) {
      st.angular = !st.angular;
      drawAutoUI();
    }
    if (btnA.pressed())
      start = true;
    // Stale stop requests / keys that mean nothing while idle.
    btnX.pressed();
    takeRemotePidStop();
    for (int k = 0; k < RK_COUNT; k++)
      takeRemoteKey((RemoteKey)k);

    RemotePidCommand cmd;
    while (takeRemotePidCommand(cmd)) {
      if (cmd.kind == RemotePidCommand::AUTO_MODE || cmd.kind == RemotePidCommand::AUTO_START) {
        bool angular = cmd.mode == 0;
        if (angular != st.angular) {
          st.angular = angular;
          drawAutoUI();
        }
        if (cmd.kind == RemotePidCommand::AUTO_START)
          start = true;
      }
      // Everything else (the manual tuner's SET / TEST / ...) is ignored.
    }

    if (exitRequested) {
      autoTunerActive = false;
      controller.rumble(".");
      printf("AUTO TUNER EXIT\n");
      return; // back to the driver menu
    }
    if (start) {
      drawAutoUI();
      runAutoTune();
      if (exitRequested) {
        autoTunerActive = false;
        printf("AUTO TUNER EXIT\n");
        return;
      }
    }

    drawControllerUI();
    pros::delay(20);
  }
}
