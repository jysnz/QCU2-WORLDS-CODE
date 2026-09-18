#pragma once

// ─── PID tuner internals shared with the automatic tuner ─────────────────────
// The manual tuner (src/pid_tuner.cpp) and the automatic one
// (src/auto_tuner.cpp) edit the same two gain sets, rebuild the same LemLib
// PIDs, and draw the same target-vs-actual graph on the brain. Everything
// they share is declared here and defined in pid_tuner.cpp; nothing else in
// the project needs it (use pid_tuner.hpp / auto_tuner.hpp instead).

#include "pros/misc.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

// gains = {kP, kI, kD}. Both tuners edit these in RAM; applyGains() pushes
// one set into the chassis PID it belongs to.
struct GainSet {
  float gains[3];
  float windup;
};
extern GainSet angularGains;
extern GainSet lateralGains;

// Copies lateral_controller / angular_controller (main.cpp) into the two
// GainSets the first time either tuner opens; a no-op after that so an
// auto-tune result survives into the manual tuner and back.
void loadGainsFromChassisConfig();

// Rebuilds chassis.angularPID or chassis.lateralPID from the matching
// GainSet. Only call between motions.
void applyGains(bool angular);

float wrap180(float deg);

// Zeroes the pose (chassis.setPose(0, 0, 0)) and prints POSE ZEROED.
void zeroPoseForRun();

// Settle detection shared by every run: |error| inside the band for
// kSettleHoldMs counts as settled.
extern const float kAngularSettleBand; // degrees
extern const float kLateralSettleBand; // inches
extern const int kSettleHoldMs;
extern const int kSampleDelayMs;

// CSV/sample period while a run is streaming to the laptop: every sample
// on a cable, 10/s over the controller's radio.
int csvPeriodMs();

// ─── Brain screen palette + graph rectangle (cyber-HUD palette, main.cpp) ────
namespace ui {
static const int BG = 0x0A0A0F;
static const int CARD = 0x161B22;
static const int CYAN = 0x00F0FF;
static const int ORANGE = 0xFF8C00;
static const int RED = 0xFF3131;
static const int GREEN = 0x2ECC71;
static const int GRAY = 0x777777;
static const int GRID = 0x121217;
static const int SETTLE_BAND_BG = 0x1c2838;
static const int FOOTER_BG = 0x18181F;
static const int AXIS = 0x2d2d38;
static const int TARGET_LINE = 0xFF6B81; // pink/salmon step (setpoint)
static const int ACTUAL_LINE = 0x2EFF8C; // green trace (measured value)

// Graph rectangle (value-vs-time plot)
static const int GX = 160, GY = 40, GW = 310, GH = 160;
} // namespace ui

// BACK button, top-right of the header, same spot on both tuner screens.
static const int kBackX0 = 410, kBackY0 = 3, kBackX1 = 476, kBackY1 = 27;
void drawBackButton();

// Value <-> pixel mapping for the graph, plus the grid step used to label
// it. minVal/maxVal are always "nice" multiples of step.
struct GraphScale {
  float minVal, maxVal, step;
  int toY(float v) const {
    float t = (v - minVal) / (maxVal - minVal);
    return ui::GY + ui::GH - (int)(std::clamp(t, 0.0f, 1.0f) * ui::GH);
  }
};
GraphScale computeScale(float loVal, float hiVal);
void drawValueGraphFrame(const GraphScale &sc);
void drawTargetSteps(const GraphScale &sc, const int *slotX0, const int *slotW,
                     const float *levels, int n);

// Wraps a digital button so callers can ask "was this just pressed?".
struct EdgeButton {
  pros::controller_digital_e_t id;
  bool prevHeld = false;
  bool held() const;
  bool pressed();
};
