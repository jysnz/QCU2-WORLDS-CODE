#pragma once
// Pure maths for odomReset (see functions.hpp): turns wall-distance readings into a corrected x/y.
// No robot dependencies, so it can be checked on a laptop (tools/tests/odom_reset_check.cpp).

#include <cmath>

namespace odomreset {

constexpr float FIELD_HALF_IN = 72.0f;
constexpr float MM_PER_IN = 25.4f;
constexpr float MAX_TILT_DEG = 10.0f; // heading must be this close to a wall-aligned angle

// One sensor reading. `mm` < 0 means invalid (no target / sensor dead / stale).
struct Reading {
    bool use;           // the caller asked for this sensor
    int mm;             // calibrated distance to whatever the sensor sees
    float rayOffsetDeg; // sensor direction relative to the robot heading: front 0, right +90, left -90
    float mountIn;      // distance from the tracking centre to the sensor face, along its ray
};

struct Result {
    bool xOk = false, yOk = false;
    float x = 0, y = 0;
    int used = 0;     // readings that agreed with odometry and were used
    int rejected = 0; // readings dropped (invalid excluded): obstacle / outside field / too far from odometry
    bool headingOk = false; // theta was re-estimated from the left+right span
    float theta = 0;        // corrected heading (deg); equals the input heading when !headingOk
};

inline float wrap180(float a) { return std::fmod(std::fmod(a + 180.0f, 360.0f) + 360.0f, 360.0f) - 180.0f; }

// Field width between opposite walls.
constexpr float FIELD_IN = 2 * FIELD_HALF_IN;

// pose: odometry x, y (in) and heading (deg, clockwise from +Y). Corrects x/y, and the heading when
// fixHeading and both the left and right sensors are valid: they look along one line in opposite
// directions, so their summed length is FIELD_IN / cos(tilt). That gives the SIZE of the tilt; its
// sign (which way the robot is turned off the wall) is taken from the odometry heading. The
// estimate is blind below ~3 deg (cos is flat), so it only helps when the heading is really off.
inline Result correct(float px, float py, float heading, const Reading *r, int n, float maxCorrectionIn,
                      bool fixHeading = false) {
    Result out;
    out.x = px;
    out.y = py;
    out.theta = heading;
    if (fixHeading) {
        const Reading *L = nullptr, *R = nullptr;
        for (int i = 0; i < n; i++) {
            if (!r[i].use || r[i].mm < 0)
                continue;
            if (r[i].rayOffsetDeg < -45.0f && r[i].rayOffsetDeg > -135.0f)
                L = &r[i];
            else if (r[i].rayOffsetDeg > 45.0f && r[i].rayOffsetDeg < 135.0f)
                R = &r[i];
        }
        if (L && R) {
            const float span = (L->mm + R->mm) / MM_PER_IN + L->mountIn + R->mountIn;
            if (std::fabs(span - FIELD_IN) < 6.0f) { // else something other than the two walls is in view
                const float ratio = std::fmin(1.0f, FIELD_IN / span);
                const float tilt = std::acos(ratio) * 180.0f / 3.14159265f;
                const float snapped = std::round(heading / 90.0f) * 90.0f;
                const float sign = wrap180(heading - snapped) < 0 ? -1.0f : 1.0f;
                if (tilt <= MAX_TILT_DEG) {
                    out.theta = snapped + sign * tilt;
                    out.headingOk = true;
                    heading = out.theta; // x/y use the corrected heading
                }
            }
        }
    }
    float sumX = 0, sumY = 0;
    int cntX = 0, cntY = 0;
    for (int i = 0; i < n; i++) {
        if (!r[i].use || r[i].mm < 0)
            continue;
        const float ray = heading + r[i].rayOffsetDeg;               // direction the sensor points
        const float snapped = std::round(ray / 90.0f) * 90.0f;       // the wall-aligned direction it should be near
        const float tilt = wrap180(ray - snapped);
        if (std::fabs(tilt) > MAX_TILT_DEG) {
            out.rejected++;
            continue;
        }
        const float perp = (r[i].mm / MM_PER_IN + r[i].mountIn) * std::cos(tilt * 3.14159265f / 180.0f);
        const int dir = ((int)std::lround(snapped / 90.0f) % 4 + 4) % 4; // 0 +Y, 1 +X, 2 -Y, 3 -X wall
        float v;
        bool isX = (dir == 1 || dir == 3);
        switch (dir) {
        case 0: v = FIELD_HALF_IN - perp; break;
        case 1: v = FIELD_HALF_IN - perp; break;
        case 2: v = -FIELD_HALF_IN + perp; break;
        default: v = -FIELD_HALF_IN + perp; break;
        }
        const float odom = isX ? px : py;
        if (std::fabs(v) > FIELD_HALF_IN || std::fabs(v - odom) > maxCorrectionIn) {
            out.rejected++;
            continue;
        }
        out.used++;
        if (isX) {
            sumX += v;
            cntX++;
        } else {
            sumY += v;
            cntY++;
        }
    }
    if (cntX) {
        out.xOk = true;
        out.x = sumX / cntX;
    }
    if (cntY) {
        out.yOk = true;
        out.y = sumY / cntY;
    }
    return out;
}

// True when the heading is within MAX_TILT_DEG of a wall-aligned angle (0/90/180/270).
inline bool headingAligned(float heading) {
    return std::fabs(wrap180(heading - std::round(heading / 90.0f) * 90.0f)) <= MAX_TILT_DEG;
}

} // namespace odomreset
