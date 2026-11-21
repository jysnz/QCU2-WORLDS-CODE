#pragma once

// ─── Distance Sensor Diagnostics + Calibration ───────────────────────────────
// Enter by tapping "DISTANCE" on the driver menu; tap BACK to return.
//
// Three VL53L0X time-of-flight sensors (BACK, LEFT, RIGHT) are read by an ESP32, which sends
// its readings to the V5 brain over RS-485 on smart port 20 (see tools/esp32_vl53l0x/esp32_vl53l0x.ino
// for the ESP32 side and the wiring). This page listens on that port and shows whether the
// sensor chain is working and what each side measures.
//
// Wire format, one ASCII line per reading at 115200 baud:
//
//   D:<back>,<left>,<right>\n     (each value: sensor S0, S1, S2 on the ESP32)
//     value >= 0   distance in millimetres
//     value == -1  no target in range
//     value == -2  that VL53L0X did not start (I2C wiring)
//   A legacy "D:<value>" line is still accepted (the other two sensors read as missing).
//
// Calibration (card = BACK / LEFT / RIGHT, tap one to select it):
//   -  / +     reference distance, 10 mm steps
//   SET REF    hold a target at the reference distance, tap: offset = reference - raw reading
//   CLEAR      offset back to 0
//   SRC Sn     which ESP32 sensor (S0..S2) is mounted on this side (swaps with the side that had it)
//   SAVE       commit the sensor-to-side assignment; robot code only sees it after SAVE (orange = unsaved)
//
// The same page is mirrored in the laptop window tools/distance_calibration.py (opened by
// tools/remote_touch.py), which sends "DIST SEL i | REF mm | SETREF | CLEAR | SRC side sensor | SAVE" lines.
//
// Status shown on the page:
//   OK                 fresh frames with a distance
//   OUT OF RANGE       fresh frames, nothing in range (VL53L0X reaches about 1.2 m)
//   SENSOR NOT FOUND   the ESP32 is sending, but no VL53L0X is answering
//   STALE              no frame for 1 s
//   NO DATA            no bytes at all (RS-485 wiring, smart port number or baud rate)
//   BAD DATA           bytes arrive but none parse (baud rate or A/B swapped)
void distanceDiagControl();

// ESP32 sensor S0 is mounted at the front centre of the robot (FRONT is an alias of the old BACK name).
enum class DistanceSide { BACK = 0, FRONT = 0, LEFT = 1, RIGHT = 2 };

// Starts the background task that reads the port, so robot code gets readings without the DISTANCE page
// open (the page takes the port over while it is open). Call once from initialize(); safe to repeat.
void distanceStartReader();

// Median of the last `samples` valid frames no older than `maxAgeMs`, calibrated for `side` (mm).
// Returns -1 when there is no valid recent reading (no target, dead sensor, stale link).
int distanceReadMm(DistanceSide side, int samples = 7, int maxAgeMs = 300);

// Distance from the robot's tracking centre to each sensor face, along the sensor's direction, in inches.
// Measure and set these; they default to half the 15 in footprint.
constexpr float SENSOR_FRONT_OFFSET_IN = 7.5f;
constexpr float SENSOR_LEFT_OFFSET_IN = 7.5f;
constexpr float SENSOR_RIGHT_OFFSET_IN = 7.5f;

// Calibration set on the page; kept until the program restarts. Use these to turn a raw reading
// (mm, or -1 / -2) from the link into a calibrated one.
int distanceSensorIndex(DistanceSide side);                  // ESP32 sensor (0..2) saved for this side (after SAVE)
int distanceOffsetMm(DistanceSide side);                     // offset added to the raw reading
void setDistanceOffsetMm(DistanceSide side, int offsetMm);
int distanceCalibrated(DistanceSide side, int rawMm);        // raw + offset; -1 / -2 pass through unchanged

// Called by the remote-touch listener task for each "DIST ..." line (the text after "DIST ").
void distanceRemoteCommand(const char *rest);

// Smart port the RS-485 transceiver is plugged into, and the baud rate (must match
// the ESP32 sketch).
constexpr int DISTANCE_PORT = 20;
constexpr int DISTANCE_BAUD = 115200;
