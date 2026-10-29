// ESP32 side of the VL53L0X distance sensor for the V5 brain.
//
// Reads a VL53L0X over I2C and sends one ASCII line about every 50 ms over RS-485:
//
//   D:<value>\n
//     value >= 0   distance in millimetres
//     value == -1  no target in range
//     value == -2  the VL53L0X did not start (I2C wiring)
//
// The brain reads this on a smart port; the DISTANCE page on the driver menu
// (src/distance_diag.cpp) shows whether it is working.
//
// Wiring
//   VL53L0X  VIN -> ESP32 3V3        SDA -> GPIO21       SCL -> GPIO22      GND -> GND
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

constexpr uint32_t BAUD = 115200;       // must match DISTANCE_BAUD in distance_diag.hpp
constexpr uint32_t PERIOD_MS = 50;      // time between frames
constexpr uint32_t RETRY_MS = 1000;     // how often to retry a sensor that did not start
constexpr int RANGE_STATUS_OUT_OF_RANGE = 4;  // VL53L0X "phase fail": nothing in range
constexpr int VALUE_OUT_OF_RANGE = -1;
constexpr int VALUE_SENSOR_ERROR = -2;

Adafruit_VL53L0X lox;
bool sensorOk = false;
uint32_t lastFrameMs = 0;
uint32_t lastTryMs = 0;

void sendFrame(int value) {
  char line[16];
  int n = snprintf(line, sizeof(line), "D:%d\n", value);
  digitalWrite(RS485_DE_RE, HIGH);
  Serial2.write((const uint8_t *)line, n);
  Serial2.flush();  // wait until the last bit has left before releasing the bus
  digitalWrite(RS485_DE_RE, LOW);
}

void setup() {
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);
  Serial2.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.begin(115200);  // USB, for checking the readings on a laptop
  Wire.begin(SDA_PIN, SCL_PIN);
  sensorOk = lox.begin();
  lastTryMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastFrameMs < PERIOD_MS) return;
  lastFrameMs = now;

  if (!sensorOk) {
    // keep telling the brain the sensor is missing, and retry about once a second
    if (now - lastTryMs >= RETRY_MS) {
      lastTryMs = now;
      sensorOk = lox.begin();
    }
    sendFrame(VALUE_SENSOR_ERROR);
    return;
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  int value = (measure.RangeStatus == RANGE_STATUS_OUT_OF_RANGE) ? VALUE_OUT_OF_RANGE
                                                                 : (int)measure.RangeMilliMeter;
  sendFrame(value);
  Serial.printf("D:%d\n", value);
}
