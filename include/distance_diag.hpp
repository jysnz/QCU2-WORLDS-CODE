#pragma once

// ─── Distance Sensor Diagnostics ─────────────────────────────────────────────
// Enter by tapping "DISTANCE" on the driver menu; tap BACK to return.
//
// A VL53L0X time-of-flight sensor is read by an ESP32, which sends its readings to the
// V5 brain over RS-485 on a smart port (see tools/esp32_vl53l0x/esp32_vl53l0x.ino for the
// ESP32 side and the wiring). This page listens on that port and shows whether the
// sensor chain is working and what it measures.
//
// Wire format, one ASCII line per reading at 115200 baud (the same lines the VEXcode
// test program reads):
//
//   D:<value>\n
//     value >= 0   distance in millimetres
//     value == -1  no target in range
//     value == -2  the ESP32 is running but the VL53L0X did not start (I2C wiring)
//
// Status shown on the page:
//   OK                 fresh frames with a distance
//   OUT OF RANGE       fresh frames, nothing in range (VL53L0X reaches about 1.2 m)
//   SENSOR NOT FOUND   the ESP32 is sending, but the VL53L0X is not answering
//   STALE              no frame for 1 s
//   NO DATA            no bytes at all (RS-485 wiring, smart port number or baud rate)
//   BAD DATA           bytes arrive but none parse (baud rate or A/B swapped)
void distanceDiagControl();

// Smart port the RS-485 transceiver is plugged into, and the baud rate (must match
// the ESP32 sketch).
constexpr int DISTANCE_PORT = 20;
constexpr int DISTANCE_BAUD = 115200;
