#pragma once
#include <cstdint>

// --- Claw auto-grab ---------------------------------------------------------
// When the distance sensor sees something inside the claw, close the claw and raise the lift -- once per
// object, and only with the lift at 0. Header only and free of robot calls, so it can be tested on a PC.
//
// The sensor's readings already include the -60 mm offset (distance_diag.hpp), so "0 to 10 mm" is an object
// sitting in the claw.
struct ClawAutoGrab {
  static constexpr int MAX_MM = 10;             // a reading of 0..MAX_MM means something is in the claw
  static constexpr uint32_t REARM_AFTER_MS = 300;  // out of that zone this long = the object is gone

  bool armed = true;

  // mm: the calibrated reading (-1 no target / no data, -2 sensor dead). Returns true when the claw should
  // close and the lift go up right now.
  //
  // It re-arms as soon as the reading has stayed outside 0..MAX_MM for REARM_AFTER_MS. (An empty claw reads
  // a few mm above MAX_MM, not "nothing", so waiting for a far reading would never re-arm it.)
  bool update(int mm, bool liftAtZero, bool clawClosed, bool busy, uint32_t nowMs) {
    bool inClaw = mm >= 0 && mm <= MAX_MM;
    if (!inClaw) {
      if (!outside_) {
        outside_ = true;
        outsideSinceMs_ = nowMs;
      }
      if (nowMs - outsideSinceMs_ >= REARM_AFTER_MS) armed = true;
      return false;
    }
    outside_ = false;
    if (clawClosed) {  // already holding it: using it up here stops the claw re-closing the moment it is
      armed = false;   // opened again while the object is still in front of the sensor
      return false;
    }
    if (!armed) return false;
    if (!liftAtZero || busy) return false;  // keep waiting: it fires as soon as the lift is at 0
    armed = false;
    return true;
  }

 private:
  bool outside_ = true;
  uint32_t outsideSinceMs_ = 0;
};
