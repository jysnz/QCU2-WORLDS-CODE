// ESP32 side of the VL53L0X distance sensor for the V5 brain.
//
// Reads the VL53L0X, runs every reading through a median filter and then an adaptive Kalman filter
// (SensorFilters.h), and sends one ASCII line about every 50 ms over RS-485:
//
//   D:<s0>[,<s1>[,<s2>]]\n     FILTERED distance, the value the robot should use
//   R:<s0>[,<s1>[,<s2>]]\n     the raw readings, for comparing and for calibration checks
//     each value >= 0   distance in millimetres
//     each value == -1  no target in range
//     each value == -2  that VL53L0X did not start (I2C / XSHUT wiring)
//
// One value is sent per connected sensor (SENSOR_COUNT): "D:73" with one sensor, "D:73,120" with two.
// The brain fills the missing slots in as unused.
//
// The brain reads this on smart port 2: DISTANCE on the driver menu (src/distance_diag.cpp) shows the
// filtered and raw value, tells you whether the sensor is working, and calibrates an offset.
//
// Wiring
//   VL53L0X   VIN -> 3V3   GND -> GND   SDA -> GPIO21   SCL -> GPIO22
//   RS-485    DI  -> GPIO17 (TX)   RO -> GPIO16 (RX)   RE + DE -> GPIO4 (tied together)
//   RS-485 A / B go to the V5 smart port's RS-485 pair; ground is shared.
//   With SENSOR_COUNT 2 or 3 every extra VL53L0X also needs its XSHUT pin wired (XSHUT_PINS below),
//   because they all power up at the same I2C address. With one sensor, XSHUT is not needed.
//
// Library: "Adafruit VL53L0X" (Arduino Library Manager).  Board: any ESP32 dev board.
//
// Checking it without the robot: open the Serial Monitor (115200) to see the D: / R: lines, or the Serial
// Plotter to see raw / median / kalman drawn together (PLOT_SENSOR below).

#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <esp_system.h>

#include "SensorFilters.h"

// --- Settings you may want to change ----------------------------------------
constexpr int SENSOR_COUNT = 1;  // how many VL53L0X are connected (1 to 3)

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int RS485_RX = 16;    // RO
constexpr int RS485_TX = 17;    // DI
constexpr int RS485_DE_RE = 4;  // HIGH = transmit, LOW = receive
constexpr int XSHUT_PINS[3] = {26, 25, 27};  // XSHUT pin of sensor 0, 1, 2. Sensor 0 is on GPIO26 here (debi used 25 / 26 / 27)
constexpr bool DRIVE_XSHUT_HIGH_FOR_ONE_SENSOR = true;  // see setup(): keeps a single sensor out of reset
constexpr uint8_t I2C_ADDRS[3] = {0x30, 0x31, 0x32};   // only used when SENSOR_COUNT > 1

constexpr uint32_t BAUD = 115200;      // must match DISTANCE_BAUD in distance_diag.hpp on the brain
constexpr uint32_t FRAME_MS = 50;      // time between frames sent to the brain
constexpr uint32_t RETRY_MS = 1000;    // how often to retry a sensor that did not start

// Sensor timing: a longer budget means less noise but fewer readings per second.
constexpr uint32_t TIMING_BUDGET_US = 40000;
constexpr uint16_t RANGE_PERIOD_MS = 50;       // continuous ranging period (>= the budget)
constexpr int MAX_VALID_MM = 1300;             // the VL53L0X reads to about 1.2 m; beyond that it is nothing

// 0..2 = print raw / median / kalman of that sensor for the Serial Plotter, -1 = print the frames instead.
constexpr int PLOT_SENSOR = -1;
constexpr int CONSOLE_SENSOR = 0;  // the one sensor the USB Serial Monitor prints (0, 1 or 2)

// Filter tuning lives in KalmanConfig (SensorFilters.h). The ones to touch first:
//   accelSigma   higher follows a moving target faster but passes more noise, lower is smoother
//   noiseBase / noisePerMm   how much the sensor is trusted at a given distance
//   gateSigma    how far from the prediction a reading must be to count as an outlier
KalmanConfig kalmanConfig;

// --- State ------------------------------------------------------------------
constexpr int SLOTS = 3;  // values per frame, fixed by the wire format
constexpr int RANGE_STATUS_VALID = 0;
constexpr int VALUE_NO_TARGET = -1;
constexpr int VALUE_SENSOR_ERROR = -2;

Adafruit_VL53L0X lox[SENSOR_COUNT];
bool sensorOk[SENSOR_COUNT];
DistanceFilter filters[SENSOR_COUNT];
int filtered[SENSOR_COUNT];   // last filtered value (mm, or -1)
int rawValue[SENSOR_COUNT];   // last raw value (mm, or -1)
uint32_t lastSampleMs[SENSOR_COUNT];
uint32_t lastFrameMs = 0;
uint32_t lastTryMs = 0;

void sendFrame() {
  int d[SLOTS], r[SLOTS];
  for (int i = 0; i < SLOTS; i++) {
    bool used = i < SENSOR_COUNT && sensorOk[i];
    d[i] = used ? filtered[i] : VALUE_SENSOR_ERROR;
    r[i] = used ? rawValue[i] : VALUE_SENSOR_ERROR;
  }
  // Only the connected sensors are sent: "D:73" for one sensor, "D:73,120" for two (the brain fills the rest in).
  char line[80];
  int n = 0;
  n += snprintf(line + n, sizeof(line) - n, "D:");
  for (int i = 0; i < SENSOR_COUNT; i++) n += snprintf(line + n, sizeof(line) - n, "%s%d", i ? "," : "", d[i]);
  n += snprintf(line + n, sizeof(line) - n, "\nR:");
  for (int i = 0; i < SENSOR_COUNT; i++) n += snprintf(line + n, sizeof(line) - n, "%s%d", i ? "," : "", r[i]);
  n += snprintf(line + n, sizeof(line) - n, "\n");
  digitalWrite(RS485_DE_RE, HIGH);
  Serial2.write((const uint8_t *)line, n);
  Serial2.flush();  // wait until the last bit has left before releasing the bus
  digitalWrite(RS485_DE_RE, LOW);

  // The USB console shows one sensor only (CONSOLE_SENSOR); the RS-485 frame above keeps all three slots.
  if (PLOT_SENSOR < 0) Serial.printf("D:%d R:%d\n", d[CONSOLE_SENSOR], r[CONSOLE_SENSOR]);
}

// --- Diagnostics (USB Serial Monitor only; every line starts with "#", so it is not mistaken for a frame) --
const char *resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON: power was lost or dipped (loose / shorted wire, weak USB cable or port)";
    case ESP_RST_BROWNOUT: return "BROWNOUT: the supply voltage dipped (shorted sensor, weak USB power, RS-485 module)";
    case ESP_RST_SW: return "SW: the program restarted itself";
    case ESP_RST_PANIC: return "PANIC: the program crashed";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "WATCHDOG: the program hung";
    case ESP_RST_EXT: return "EXT: the reset pin was pulled";
    default: return "other";
  }
}

// Lists every device that answers on the I2C bus. A working VL53L0X shows up at 0x29.
int i2cScan(bool print) {
  int found = 0;
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      if (print) Serial.printf("# I2C device at 0x%02X%s\n", addr, addr == 0x29 ? "  <- the VL53L0X's default address" : "");
    }
  }
  return found;
}

// Does anything answer at this address?
bool i2cAnswers(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Reads the VL53L0X model-ID register (0xC0): a VL53L0X answers 0xEE. Returns -1 if the read fails.
int readModelId(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0xC0);
  if (Wire.endTransmission(false) != 0) return -1;  // repeated start, no stop
  if (Wire.requestFrom((uint16_t)addr, (size_t)1) != 1) return -1;
  return Wire.read();
}

// Says whether the chip at addr really is a VL53L0X.
void printSensorId(uint8_t addr) {
  int id = readModelId(addr);
  if (id < 0)
    Serial.printf("# 0x%02X: answers, but a register read fails. Suspect the wires: long, loose, or no pull-ups on SDA / SCL.\n", addr);
  else if (id == 0xEE)
    Serial.printf("# 0x%02X: model id 0xEE = a real VL53L0X. The chip is fine; start-up failed over the bus (see the library lines above).\n", addr);
  else
    Serial.printf("# 0x%02X: model id 0x%02X, but a VL53L0X reads 0xEE. This is a DIFFERENT chip (a VL53L1X?) or the bus is noisy. "
                  "This sketch only drives the VL53L0X.\n", addr, id);
}

// "0x29 0x3C" -- every address that answers.
void i2cList(char *buf, size_t cap) {
  size_t n = 0;
  buf[0] = '\0';
  for (int addr = 1; addr < 127 && n + 6 < cap; addr++)
    if (i2cAnswers((uint8_t)addr)) n += snprintf(buf + n, cap - n, "%s0x%02X", n ? " " : "", addr);
  if (n == 0) snprintf(buf, cap, "none");
}

void printBootDiagnostics() {
  Serial.printf("# boot: last reset = %s\n", resetReasonText(esp_reset_reason()));
  Serial.printf("# I2C pins: SDA = GPIO%d, SCL = GPIO%d\n", SDA_PIN, SCL_PIN);
  int n = i2cScan(true);
  if (n > 0) {
    if (i2cAnswers(0x29)) printSensorId(0x29);
    else if (SENSOR_COUNT == 1)
      Serial.println("# Something answers, but not at 0x29, where a VL53L0X lives. Check what else is on the I2C bus.");
    return;
  }
  Serial.println("# I2C scan: NOTHING answers. Checking the usual causes:");
  // The most common wiring mistake: SDA and SCL the wrong way round. Try it swapped, then put it back.
  Wire.end();
  Wire.begin(SCL_PIN, SDA_PIN);
  int swapped = i2cScan(false);
  Wire.end();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (swapped > 0)
    Serial.printf("# -> %d device(s) answer with SDA and SCL SWAPPED. Swap the two wires (SDA to GPIO%d, SCL to GPIO%d).\n",
                  swapped, SDA_PIN, SCL_PIN);
  else
    Serial.printf("# -> not swapped either. Check: VIN to 3V3 and GND to GND (right pins, not shorted), the sensor's "
                  "XSHUT pin (GPIO%d here: wire it there or leave it unconnected), and that the sensor is not damaged.\n",
                  XSHUT_PINS[0]);
}

// Brings up one sensor and starts continuous ranging. Returns true when it answers.
bool beginDebug = false;  // when true the Adafruit library prints why start-up failed

bool beginSensor(int i) {
  bool ok = SENSOR_COUNT == 1 ? lox[i].begin(0x29, beginDebug) : lox[i].begin(I2C_ADDRS[i], beginDebug);
  if (!ok) return false;
  lox[i].setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
  lox[i].startRangeContinuous(RANGE_PERIOD_MS);
  filters[i] = DistanceFilter(kalmanConfig);
  filtered[i] = rawValue[i] = VALUE_NO_TARGET;
  lastSampleMs[i] = millis();
  return true;
}

// Brings up the sensors that are not running yet. With several sensors they are held in reset first and
// woken one at a time, each moved to its own address.
void startSensors() {
  bool anyMissing = false;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (!sensorOk[i]) anyMissing = true;
  if (!anyMissing) return;

  if (SENSOR_COUNT == 1) {
    sensorOk[0] = beginSensor(0);
    return;
  }
  // Only the sensors that are not running are reset; running ones keep their address.
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (!sensorOk[i]) digitalWrite(XSHUT_PINS[i], LOW);
  delay(10);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorOk[i]) continue;
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    sensorOk[i] = beginSensor(i);
    if (!sensorOk[i]) digitalWrite(XSHUT_PINS[i], LOW);  // keep a dead one off the bus
  }
}

// Reads whatever the sensor has finished since last time and feeds it through the filters.
void sampleSensor(int i) {
  if (!lox[i].isRangeComplete()) return;
  uint16_t mm = lox[i].readRange();
  uint8_t status = lox[i].readRangeStatus();
  uint32_t now = millis();
  float dt = (now - lastSampleMs[i]) / 1000.0f;
  lastSampleMs[i] = now;

  bool valid = status == RANGE_STATUS_VALID && mm <= MAX_VALID_MM;
  rawValue[i] = valid ? (int)mm : VALUE_NO_TARGET;
  filtered[i] = filters[i].update(valid, (float)mm, dt);

  if (PLOT_SENSOR == i)
    Serial.printf("raw:%d,median:%d,kalman:%d\n", rawValue[i], filters[i].medianMm(), filtered[i]);
}

void setup() {
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);
  Serial2.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.begin(115200);  // USB, for checking the readings on a laptop
  // One sensor: drive its XSHUT pin high. A sensor wired the debi way (XSHUT on GPIO25) whose board has no
  // pull-up on XSHUT would otherwise stay in reset and be invisible on the bus. If nothing is wired to that
  // pin this does nothing.
  if (SENSOR_COUNT == 1 && DRIVE_XSHUT_HIGH_FOR_ONE_SENSOR) {
    pinMode(XSHUT_PINS[0], OUTPUT);
    digitalWrite(XSHUT_PINS[0], HIGH);
  }
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(300);  // let the USB serial connection and the sensor's power settle
  printBootDiagnostics();
  if (SENSOR_COUNT > 1) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      pinMode(XSHUT_PINS[i], OUTPUT);
      digitalWrite(XSHUT_PINS[i], LOW);
    }
  }
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorOk[i] = false;
    filtered[i] = rawValue[i] = VALUE_SENSOR_ERROR;
  }
  beginDebug = true;
  startSensors();
  lastTryMs = millis();
}

void loop() {
  uint32_t now = millis();

  // keep retrying sensors that did not start, about once a second
  if (now - lastTryMs >= RETRY_MS) {
    lastTryMs = now;
    bool missing = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
      if (!sensorOk[i]) missing = true;
    static int tries = 0;
    tries++;
    beginDebug = (tries % 10 == 1);  // the library's own error text, but not every second
    startSensors();
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (sensorOk[i] || !missing) continue;
      char list[60];
      i2cList(list, sizeof(list));
      Serial.printf("# sensor %d: not found (I2C answering at: %s)\n", i, list);
      if (i2cAnswers(0x29)) printSensorId(0x29);
    }
  }

  for (int i = 0; i < SENSOR_COUNT; i++)
    if (sensorOk[i]) sampleSensor(i);

  if (now - lastFrameMs >= FRAME_MS) {
    lastFrameMs = now;
    sendFrame();
  }
}
