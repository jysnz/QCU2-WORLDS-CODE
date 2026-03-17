#pragma once

#include "pros/vision.hpp"

// ─── Vision sensor ────────────────────────────────────────────────────────────
extern pros::Vision vision_sensor;
extern pros::vision_signature_s_t GOAL_SIG;

// ─── Constants ────────────────────────────────────────────────────────────────
extern const int    VISION_FOV_WIDTH_VAL;
extern const int    VISION_CENTER_X;
extern const int    VISION_MIN_AREA;
extern const double ALIGN_KP;
extern const double ALIGN_THRESHOLD;
extern const int    ALIGN_TIMEOUT;

// ─── Functions ────────────────────────────────────────────────────────────────
void alignToGoal();