// ESP32 side of the three VL53L0X distance sensors for the V5 brain.
//
// Reads three VL53L0X over one I2C bus and sends one ASCII line about every 50 ms over RS-485:
//
//   D:<s0>,<s1>,<s2>\n
//     each value >= 0   distance in millimetres
//     each value == -1  no target in range
//     each value == -2  that VL53L0X did not start (I2C / XSHUT wiring)
//
// The brain reads this on smart port 20; the DISTANCE page on the driver menu
// (src/distance_diag.cpp) shows S0 / S1 / S2 as BACK / LEFT / RIGHT and lets you calibrate them.
//
// All VL53L0X boards power up at the same I2C address (0x29), so each one has its own XSHUT pin.
// At boot they are held in reset, then woken one at a time and moved to a unique address.
//
// Wiring
//   VL53L0X (all three)  VIN -> 3V3   GND -> GND   SDA -> GPIO21   SCL -> GPIO22 (shared bus)
//   XSHUT                S0 (BACK) -> GPIO25   S1 (LEFT) -> GPIO26   S2 (RIGHT) -> GPIO27
//   RS-485   DI  -> GPIO17 (TX)      RO  -> GPIO16 (RX)  RE + DE -> GPIO4   (tied together)
//   RS-485 A / B go to the V5 smart port's RS-485 pair; ground is shared.
//
// Library: "Adafruit VL53L0X" (Arduino Library Manager). Board: any ESP32 dev board.

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int RS485_RX = 16;    // RO
constexpr int RS485_TX = 17;    // DI
constexpr int RS485_DE_RE = 4;  // HIGH = transmit, LOW = receive

constexpr int SENSOR_COUNT = 3;
constexpr int XSHUT_PINS[SENSOR_COUNT] = {25, 26, 27};
constexpr uint8_t I2C_ADDRS[SENSOR_COUNT] = {0x30, 0x31, 0x32};

constexpr uint32_t BAUD = 115200;       // must match DISTANCE_BAUD in distance_diag.hpp
constexpr uint32_t PERIOD_MS = 50;      // time between frames
constexpr uint32_t RETRY_MS = 1000;     // how often to retry sensors that did not start
constexpr int RANGE_STATUS_OUT_OF_RANGE = 4;  // VL53L0X "phase fail": nothing in range
constexpr int VALUE_OUT_OF_RANGE = -1;
constexpr int VALUE_SENSOR_ERROR = -2;

Adafruit_VL53L0X lox[SENSOR_COUNT];
bool sensorOk[SENSOR_COUNT] = {false, false, false};
uint32_t lastFrameMs = 0;
uint32_t lastTryMs = 0;

void sendFrame(const int *values) {
  char line[40];
  int n = snprintf(line, sizeof(line), "D:%d,%d,%d\n", values[0], values[1], values[2]);
  digitalWrite(RS485_DE_RE, HIGH);
  Serial2.write((const uint8_t *)line, n);
  Serial2.flush();  // wait until the last bit has left before releasing the bus
  digitalWrite(RS485_DE_RE, LOW);
}

// Hold every sensor in reset, then bring up the ones that are not running yet, one at a time.
void startSensors() {
  bool anyMissing = false;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (!sensorOk[i]) anyMissing = true;
  if (!anyMissing) return;

  // Only the sensors that are not running are reset; running ones keep their address.
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (!sensorOk[i]) digitalWrite(XSHUT_PINS[i], LOW);
  delay(10);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorOk[i]) continue;
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    sensorOk[i] = lox[i].begin(I2C_ADDRS[i]);
    if (!sensorOk[i]) digitalWrite(XSHUT_PINS[i], LOW);  // keep a dead one off the bus
  }
}

void setup() {
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);
  Serial2.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.begin(115200);  // USB, for checking the readings on a laptop
  Wire.begin(SDA_PIN, SCL_PIN);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  startSensors();
  lastTryMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastFrameMs < PERIOD_MS) return;
  lastFrameMs = now;

  // keep retrying sensors that did not start, about once a second
  if (now - lastTryMs >= RETRY_MS) {
    lastTryMs = now;
    startSensors();
  }

  int values[SENSOR_COUNT];
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorOk[i]) {
      values[i] = VALUE_SENSOR_ERROR;
      continue;
    }
    VL53L0X_RangingMeasurementData_t measure;
    lox[i].rangingTest(&measure, false);
    values[i] = (measure.RangeStatus == RANGE_STATUS_OUT_OF_RANGE) ? VALUE_OUT_OF_RANGE
                                                                   : (int)measure.RangeMilliMeter;
  }
  sendFrame(values);
  Serial.printf("D:%d,%d,%d\n", values[0], values[1], values[2]);
}
