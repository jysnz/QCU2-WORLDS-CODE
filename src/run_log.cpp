#include "run_log.hpp"
#include "imu_drift.hpp"
#include "lift_calibration.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern lemlib::Drivetrain drivetrain;            // main.cpp
extern lemlib::TrackingWheel verticalTracking;   // main.cpp
extern lemlib::TrackingWheel horizontalTracking; // main.cpp

namespace {

constexpr int kMaxSamples = 2400; // 20 ms apart: 48 s, longer than a VEX U autonomous
constexpr int kMaxEvents = 160;
constexpr int kMaxSegs = 96; // sensor-check segments: one per plan step (or per motion)
constexpr uint32_t kPeriodMs = 20;
constexpr int kSamplesPerLine = 5;
constexpr int kLineGapMs = 25;   // between lines: the link (a controller radio) is slow
constexpr int kKeepEvery = 3;    // long runs: outside a pin approach only every 3rd sample is sent
constexpr int kSendAllBelow = 400; // ...but a run this short is sent in full
constexpr int kMaxLines = 1024;  // BEGIN + H + segments + events + sample lines + END

struct Sample {
  uint32_t ms;
  float x, y, th;
  int16_t step;
  uint8_t flags; // 1 pin sensor sees a pin, 2 claw out, 8 a motion is running,
                 // 16 the claw watch has closed the claws, 32 inside a goToPin approach
};

struct Event {
  uint32_t ms;
  float x, y, th;
  int16_t step;
  char what[56];
};

// What the sensors did during one plan step (or one motion, when the run has no steps): the IMU
// against the drive motors, the tracking wheels against the drive motors, tilt, and how much the
// heading moved while the robot was provably standing still.
struct Seg {
  uint32_t ms;     // when it began
  int16_t step;
  float imuDeg;    // heading change the IMU reported (what odometry uses)
  float whlDeg;    // heading change worked out from the left / right drive motors
  float vertIn;    // forward travel: the vertical tracking wheel
  float motIn;     // forward travel: the average of the drive motors
  float horIn;     // sideways tracking wheel travel
  float tilt;      // most the robot tipped (deg)
  float stillDeg;  // heading change while standing still (should be 0)
  uint32_t stillMs;
};

struct Health {
  double imu, left, right, vert, hor; // the readings one tick ago
  bool have;
  bool wasMoving;
  uint32_t lastMs, stillFor;
  float pitch0, roll0;
  int imuBad, vertBad, horBad, motBad; // ticks where the sensor did not answer
  int imuJump, vertJump, horJump;      // readings that moved further than a robot can in one tick
  float rate0, rate1, drift0, drift1;  // the IMU drift correction when the run began / ended
  float leftAbs, rightAbs;             // inches each drive side rolled, either direction
  double leftPos0, rightPos0;          // get_position() of each side when the run began
};

Sample samples[kMaxSamples];
Event events[kMaxEvents];
Seg segs[kMaxSegs];
Health hs;
uint16_t sendIdx[kMaxSamples]; // the samples that are sent, in order
volatile int nSamples = 0;
volatile int nEvents = 0;
volatile int nSegs = 0;
volatile bool rec = false;
volatile bool truncated = false;
volatile int curStep = -1;
volatile bool pinWindow = false;
uint32_t t0 = 0;
uint32_t runMs = 0;
int runId = 0;
char runName[24] = "";
char runStatus[16] = "";

// Sending. The finished recording stays in memory until the next run begins, and every line has a
// number, so the laptop can ask for the ones the link dropped (runlog::resend).
pros::Mutex sendMutex;
uint32_t needBits[kMaxLines / 32];
volatile int needCount = 0;
volatile bool sendReady = false;
int nSend = 0, nSendLines = 0, nLines = 0, sentSegs = 0, sentEvents = 0;

void takeSample() {
  const int n = nSamples;
  if (n >= kMaxSamples) {
    truncated = true;
    return;
  }
  const lemlib::Pose p = chassis.getPose();
  Sample &s = samples[n];
  s.ms = pros::millis() - t0;
  s.x = p.x;
  s.y = p.y;
  s.th = p.theta;
  s.step = (int16_t)curStep;
  s.flags = (uint8_t)((pinDetected() ? 1 : 0) | (claw.is_extended() ? 2 : 0) |
                      (chassis.isInMotion() ? 8 : 0) |
                      (pinCloseWatchHit() ? 16 : 0) | (pinWindow ? 32 : 0));
  nSamples = n + 1;
}

// Inches one side of the drivetrain has rolled, from its motors (the ones that answer).
// Read as raw encoder counts: get_position() depends on each motor's encoder units, and on this
// robot the two sides did not report in the same units (the right side read 360 times too small).
double driveInches(pros::MotorGroup &g) {
  double cartridge = 200, countsPerRev = 900; // green
  const pros::MotorGears gears = g.get_gearing();
  if (gears == pros::MotorGears::red) {
    cartridge = 100;
    countsPerRev = 1800;
  } else if (gears == pros::MotorGears::blue) {
    cartridge = 600;
    countsPerRev = 300;
  }
  std::uint32_t stamp = 0;
  double sum = 0;
  int n = 0;
  for (std::int32_t c : g.get_raw_position_all(&stamp))
    if (c != PROS_ERR) {
      sum += c;
      n++;
    }
  if (n == 0)
    return NAN;
  return sum / n / countsPerRev * M_PI * drivetrain.wheelDiameter * (drivetrain.rpm / cartridge);
}

// The same side through get_position() (in whatever units the motors are set to): only reported,
// so the laptop can tell when the two ways of reading the motors disagree.
double drivePosition(pros::MotorGroup &g) {
  double sum = 0;
  int n = 0;
  for (double p : g.get_position_all())
    if (std::isfinite(p)) {
      sum += p;
      n++;
    }
  return n ? sum / n : NAN;
}

Seg &openSeg(uint32_t now) {
  if (nSegs < kMaxSegs) {
    Seg &g = segs[nSegs];
    g = Seg{};
    g.ms = now - t0;
    g.step = (int16_t)curStep;
    nSegs = nSegs + 1;
  }
  return segs[nSegs - 1]; // (full: everything after goes into the last one)
}

// Called every sample tick: compares the sensors with each other and adds it up per segment.
void healthTick() {
  const uint32_t now = pros::millis();
  const double im = imu.get_rotation();
  const double L = driveInches(left_motor_group), R = driveInches(right_motor_group);
  const bool vOk = verticalRotation.is_installed(), hOk = horizontalRotation.is_installed();
  const double V = vOk ? verticalTracking.getDistanceTraveled() : NAN;
  const double H = hOk ? horizontalTracking.getDistanceTraveled() : NAN;
  const bool moving = chassis.isInMotion();

  const int step = curStep;
  if (nSegs == 0 || segs[nSegs - 1].step != step || (step < 0 && moving && !hs.wasMoving))
    openSeg(now);
  Seg &g = segs[nSegs - 1];
  hs.wasMoving = moving;

  if (!std::isfinite(im))
    hs.imuBad++;
  if (!vOk)
    hs.vertBad++;
  if (!hOk)
    hs.horBad++;
  if (!std::isfinite(L) || !std::isfinite(R))
    hs.motBad++;

  if (hs.have) {
    const uint32_t dt = now - hs.lastMs;
    double dIm = im - hs.imu;
    if (std::isfinite(dIm)) {
      if (std::fabs(dIm) > 30) { // 1500 deg/s: no robot turns that fast, the reading jumped
        hs.imuJump++;
        dIm = NAN;
      } else {
        g.imuDeg += (float)dIm;
      }
    }
    const double dL = L - hs.left, dR = R - hs.right;
    if (std::isfinite(dL) && std::isfinite(dR) && std::fabs(dL) < 5 && std::fabs(dR) < 5) {
      g.whlDeg += (float)((dL - dR) / drivetrain.trackWidth * 180.0 / M_PI);
      g.motIn += (float)((dL + dR) / 2);
      hs.leftAbs += (float)std::fabs(dL);
      hs.rightAbs += (float)std::fabs(dR);
    }
    const double dV = V - hs.vert, dH = H - hs.hor;
    if (std::isfinite(dV)) {
      if (std::fabs(dV) > 5)
        hs.vertJump++;
      else
        g.vertIn += (float)dV;
    }
    if (std::isfinite(dH)) {
      if (std::fabs(dH) > 5)
        hs.horJump++;
      else
        g.horIn += (float)dH;
    }
    // standing still (drive motors and both tracking wheels at rest): the heading must not move
    // (and no motion running: a turn that is stalled or creeping has the motors near zero speed
    // too, and its heading change is not drift)
    if (!moving && robotStandingStill()) {
      hs.stillFor += dt;
      if (hs.stillFor >= 200 && std::isfinite(dIm)) { // (the first 200 ms: still settling)
        g.stillMs += dt;
        g.stillDeg += (float)dIm;
      }
    } else {
      hs.stillFor = 0;
    }
  }
  const double pitch = imu.get_pitch(), roll = imu.get_roll();
  if (std::isfinite(pitch) && std::isfinite(roll)) {
    if (!hs.have) {
      hs.pitch0 = (float)pitch;
      hs.roll0 = (float)roll;
    }
    const float tilt = (float)std::max(std::fabs(pitch - hs.pitch0), std::fabs(roll - hs.roll0));
    g.tilt = std::max(g.tilt, tilt);
  }
  hs.imu = im;
  hs.left = L;
  hs.right = R;
  hs.vert = V;
  hs.hor = H;
  hs.lastMs = now;
  hs.have = true;
}

void samplerTask(void *) {
  while (true) {
    if (rec) {
      takeSample();
      healthTick();
    }
    pros::delay(kPeriodMs);
  }
}

// ── sending ─────────────────────────────────────────────────────────────────────────────────────
// Line n of the finished recording:  0 BEGIN, 1 H (sensor totals), then one G per segment, one E
// per event, the S lines (5 samples each), and END. Each starts "LOG|#<run>.<n>|" and ends
// "|*<hh>", the sum of every character before it (mod 256, hex): the link loses single characters
// as well as whole lines, and a line with a digit missing is worse than a missing line.
void formatBody(int n, char *line, int cap);

void formatLine(int n, char *line, int cap) {
  formatBody(n, line, cap - 6);
  unsigned sum = 0;
  int len = 0;
  for (; line[len]; len++)
    sum += (unsigned char)line[len];
  snprintf(line + len, cap - len, "|*%02X", sum & 0xFFu);
}

void formatBody(int n, char *line, int cap) {
  int w = snprintf(line, cap, "LOG|#%d.%d|", runId, n);
  if (n == 0) {
    snprintf(line + w, cap - w, "BEGIN|%d|%s|%s|%d|%u|%d|%d|%d|%d", runId, runName, runStatus, nSend,
             (unsigned)kPeriodMs, sentEvents, truncated ? 1 : 0, nLines, sentSegs);
    return;
  }
  if (n == 1) {
    snprintf(line + w, cap - w,
             "H|imu=%d|vert=%d|hor=%d|imubad=%d|vbad=%d|hbad=%d|mbad=%d|imujump=%d|vjump=%d|hjump=%d|"
             "rate0=%.4f|rate1=%.4f|drift0=%.3f|drift1=%.3f|hoff=%.3f|track=%.2f|ms=%u|"
             "lin=%.1f|rin=%.1f|lpos=%.2f|rpos=%.2f|lunit=%d|runit=%d|lgear=%d|rgear=%d",
             imu.is_installed() ? 1 : 0, verticalRotation.is_installed() ? 1 : 0,
             horizontalRotation.is_installed() ? 1 : 0, hs.imuBad, hs.vertBad, hs.horBad, hs.motBad,
             hs.imuJump, hs.vertJump, hs.horJump, hs.rate0, hs.rate1, hs.drift0, hs.drift1,
             horizontalTracking.getOffset(), drivetrain.trackWidth, (unsigned)runMs, hs.leftAbs,
             hs.rightAbs, drivePosition(left_motor_group) - hs.leftPos0,
             drivePosition(right_motor_group) - hs.rightPos0,
             (int)left_motor_group.get_encoder_units(), (int)right_motor_group.get_encoder_units(),
             (int)left_motor_group.get_gearing(), (int)right_motor_group.get_gearing());
    return;
  }
  n -= 2;
  if (n < sentSegs) {
    const Seg &g = segs[n];
    snprintf(line + w, cap - w, "G|%u|%d|%.2f|%.2f|%.2f|%.2f|%.2f|%.1f|%u|%.3f", (unsigned)g.ms,
             (int)g.step, g.imuDeg, g.whlDeg, g.vertIn, g.motIn, g.horIn, g.tilt,
             (unsigned)g.stillMs, g.stillDeg);
    return;
  }
  n -= sentSegs;
  if (n < sentEvents) {
    const Event &e = events[n];
    snprintf(line + w, cap - w, "E|%u|%d|%s|%.2f|%.2f|%.1f", (unsigned)e.ms, (int)e.step, e.what,
             e.x, e.y, e.th);
    return;
  }
  n -= sentEvents;
  if (n < nSendLines) {
    w += snprintf(line + w, cap - w, "S|");
    for (int k = n * kSamplesPerLine; k < (n + 1) * kSamplesPerLine && k < nSend; k++) {
      const Sample &s = samples[sendIdx[k]];
      w += snprintf(line + w, cap - w, "%s%u,%.2f,%.2f,%.1f,%d,%d", k % kSamplesPerLine ? ";" : "",
                    (unsigned)s.ms, s.x, s.y, s.th, (int)s.step, (int)s.flags);
    }
    return;
  }
  snprintf(line + w, cap - w, "END|%d|%d", runId, nLines);
}

void need(int n) {
  if (n < 0 || n >= nLines || (needBits[n / 32] >> (n % 32)) & 1u)
    return;
  needBits[n / 32] |= 1u << (n % 32);
  needCount = needCount + 1;
}

// Sends whatever lines are wanted, paced so the link (and the controller's radio) is never
// flooded. Runs for the rest of the program.
void senderTask(void *) {
  static char line[400];
  while (true) {
    int n = -1;
    sendMutex.take();
    if (sendReady && needCount > 0) {
      for (int i = 0; i < nLines; i++)
        if ((needBits[i / 32] >> (i % 32)) & 1u) {
          n = i;
          break;
        }
      if (n >= 0) {
        needBits[n / 32] &= ~(1u << (n % 32));
        needCount = needCount - 1;
        formatLine(n, line, sizeof(line));
      } else {
        needCount = 0;
      }
    }
    sendMutex.give();
    if (n < 0) {
      pros::delay(40);
      continue;
    }
    printf("%s\n", line);
    pros::delay(kLineGapMs);
  }
}

} // namespace

namespace runlog {

bool recording() { return rec; }
bool sending() { return sendReady && needCount > 0; }

void begin(const char *name) {
  static bool started = false;
  if (!started) {
    started = true;
    pros::Task sampler(samplerTask, nullptr, "run log");
    pros::Task sender(senderTask, nullptr, "run log send");
  }
  rec = false;
  pros::delay(kPeriodMs + 5); // let a sample tick that is under way finish
  sendMutex.take();           // a recording still being sent is dropped: the new run matters more
  sendReady = false;
  needCount = 0;
  memset(needBits, 0, sizeof(needBits));
  nSamples = 0;
  nEvents = 0;
  nSegs = 0;
  truncated = false;
  curStep = -1;
  pinWindow = false;
  runId++;
  snprintf(runName, sizeof(runName), "%s", name ? name : "run");
  runStatus[0] = '\0';
  hs = Health{};
  hs.rate0 = (float)imuDriftRate();
  hs.drift0 = (float)imuDriftTotal();
  hs.leftPos0 = drivePosition(left_motor_group);
  hs.rightPos0 = drivePosition(right_motor_group);
  t0 = pros::millis();
  sendMutex.give();
  takeSample(); // the starting pose
  healthTick();
  rec = true;
}

void end(const char *status) {
  if (!rec)
    return;
  rec = false;
  pros::delay(kPeriodMs + 5); // let a sample tick that is under way finish
  takeSample();               // the final pose
  healthTick();
  sendMutex.take();
  snprintf(runStatus, sizeof(runStatus), "%s", status ? status : "DONE");
  hs.rate1 = (float)imuDriftRate();
  hs.drift1 = (float)imuDriftTotal();
  runMs = pros::millis() - t0;
  // which samples go: all of a short run; of a long one every 3rd, plus every sample of a pin
  // approach and every one where something changed (sensor, claws, motion, step)
  const int ns = nSamples;
  nSend = 0;
  for (int i = 0; i < ns; i++) {
    const bool keep = ns <= kSendAllBelow || i % kKeepEvery == 0 || i == ns - 1 ||
                      (samples[i].flags & 32) || samples[i].flags != samples[i - 1].flags ||
                      samples[i].step != samples[i - 1].step;
    if (keep)
      sendIdx[nSend++] = (uint16_t)i;
  }
  nSendLines = (nSend + kSamplesPerLine - 1) / kSamplesPerLine;
  sentSegs = nSegs;
  sentEvents = std::min((int)nEvents, kMaxEvents);
  nLines = std::min(2 + sentSegs + sentEvents + nSendLines + 1, kMaxLines);
  memset(needBits, 0, sizeof(needBits));
  needCount = 0;
  for (int i = 0; i < nLines; i++)
    need(i);
  sendReady = true; // the sender task takes it from here
  sendMutex.give();
}

void resend(const char *spec) {
  if (!spec)
    return;
  char *p = nullptr;
  const long id = strtol(spec, &p, 10);
  if (p == spec)
    return;
  sendMutex.take();
  if (sendReady && (id == 0 || id == runId)) {
    while (*p) {
      while (*p == ' ' || *p == ',')
        p++;
      if (strncmp(p, "ALL", 3) == 0) {
        for (int i = 0; i < nLines; i++)
          need(i);
        break;
      }
      char *q = nullptr;
      long a = strtol(p, &q, 10);
      if (q == p)
        break;
      long b = a;
      p = q;
      if (*p == '-') {
        b = strtol(p + 1, &q, 10);
        if (q == p + 1)
          break;
        p = q;
      }
      a = std::max(a, 0L);
      b = std::min(b, (long)nLines - 1);
      for (long i = a; i <= b; i++)
        need((int)i);
    }
  }
  const int queued = needCount, mine = runId;
  sendMutex.give();
  // so the laptop can tell "the request never reached the robot" from "the answer was lost"
  printf("LOG|ACK|%d|%d\n", mine, queued);
}

void setStep(int idx) { curStep = idx; }

void event(const char *fmt, ...) {
  if (!rec)
    return;
  const int i = __atomic_fetch_add(&nEvents, 1, __ATOMIC_RELAXED);
  if (i >= kMaxEvents) {
    nEvents = kMaxEvents;
    return;
  }
  Event &e = events[i];
  const lemlib::Pose p = chassis.getPose();
  e.ms = pros::millis() - t0;
  e.x = p.x;
  e.y = p.y;
  e.th = p.theta;
  e.step = (int16_t)curStep;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.what, sizeof(e.what), fmt, ap);
  va_end(ap);
  for (char *c = e.what; *c; c++)
    if (*c == '|' || *c == ',' || *c == '\n' || *c == '\r')
      *c = ' ';
  // a goToPin approach is sent at the full 20 ms (the pin analysis needs it)
  if (strncmp(e.what, "pin target", 10) == 0)
    pinWindow = true;
  else if (strncmp(e.what, "pin claws closed", 16) == 0 || strncmp(e.what, "cancelled", 9) == 0)
    pinWindow = false;
}

} // namespace runlog
