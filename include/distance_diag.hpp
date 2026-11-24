#pragma once

// ─── Distance sensor check + calibration ─────────────────────────────────────
// Open it from the driver menu (hold DPAD-LEFT when driver control starts, then tap DISTANCE).
//
// The VL53L0X is read by an ESP32 (VL53L0X_Code/VL53L0X_Code.ino), which filters the readings (median +
// adaptive Kalman) and sends them to the V5 brain over RS-485 on smart port 2. This page listens on that
// port and tells you whether the chain works and what the sensor measures.
//
// Wire format, ASCII lines at 115200 baud, one pair per reading:
//   D:<s0>,<s1>,<s2>\n     filtered distance   (the value robot code should use)
//   R:<s0>,<s1>,<s2>\n     raw readings        (optional; the page shows them next to the filtered ones)
//     value >= 0   distance in millimetres
//     value == -1  no target in range
//     value == -2  that VL53L0X did not start, or the slot is not used
//
// Status the page shows:
//   OK                 fresh frames with a distance
//   OUT OF RANGE       fresh frames, nothing in range (the VL53L0X reaches about 1.2 m)
//   SENSOR NOT FOUND   the ESP32 is sending, but no VL53L0X is answering (check SDA / SCL)
//   STALE              no frame for 1 s (ESP32 reset, cable loose)
//   NO DATA            no bytes at all (RS-485 wiring, smart port number or baud rate)
//   BAD DATA           bytes arrive but none parse (baud rate, or RS-485 A / B swapped)
//
// Calibration: every reading gets an offset added. It starts at DISTANCE_DEFAULT_OFFSET_MM (-60): the sensor
// sits 60 mm back from the claw's face, so a raw reading of 60 mm counts as 0 (and anything closer than
// that as 0 too). To re-calibrate, put a target at a known distance, set REF to it with - / +, tap SET REF:
// the new offset is (reference - the filtered reading). It is saved on the SD card (/usd/distance_cal.txt),
// so it survives a restart. DEFAULT puts it back to -60.
void distanceDiagControl();

// Smart port the RS-485 transceiver is plugged into, the baud rate (must match the ESP32 sketch) and how
// many VL53L0X are connected (1 to 3; set it to the sketch's SENSOR_COUNT).
constexpr int DISTANCE_PORT = 2;
constexpr int DISTANCE_BAUD = 115200;
constexpr int DISTANCE_SENSOR_COUNT = 1;

// The offset every sensor starts with (mm): a raw reading of 60 counts as 0.
constexpr int DISTANCE_DEFAULT_OFFSET_MM = -60;

// Starts the background task that reads the port, so robot code gets readings without the DISTANCE page
// open. The page starts it too. Call once from initialize(); safe to repeat.
void distanceStartReader();

// Filtered distance of `sensor` (0..2) in mm with its calibration offset added. Returns -1 when there is no
// target or no recent frame (older than maxAgeMs), -2 when that sensor is dead or unused.
int distanceReadMm(int sensor = 0, int maxAgeMs = 300);

// The calibration offset (mm) of a sensor, as set on the page.
int distanceOffsetMm(int sensor);
void setDistanceOffsetMm(int sensor, int offsetMm);
