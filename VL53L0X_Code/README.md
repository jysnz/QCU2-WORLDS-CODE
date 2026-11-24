# VL53L0X_Code

ESP32 sketch for the VL53L0X distance sensor. It filters the readings and sends them to the V5 brain over
RS-485. Open `VL53L0X_Code.ino` in the Arduino IDE (the folder name must stay `VL53L0X_Code`) and upload it.

- Board: any ESP32 dev board. Library: **Adafruit VL53L0X** (Library Manager).
- Wiring and settings are at the top of `VL53L0X_Code.ino`. `SENSOR_COUNT` is 1; set it to 2 or 3 if you
  connect more sensors (they then also need their XSHUT pins).
- On the brain, tap **DISTANCE** in the driver menu to see whether the sensor is working.

## What it does to a reading

```
raw reading -> median of the last 5 -> adaptive Kalman filter -> lag compensation -> sent to the brain
```

| Stage | What it is for |
|---|---|
| Median (5) | Throws away spikes. A lone wild reading never gets through. |
| Adaptive Kalman | Smooths the scatter and tracks a moving target (distance + speed). The noise it assumes grows with distance and adapts to the readings it sees. Readings far from its prediction are treated as outliers; three in a row mean the target really moved, and it restarts there. |
| Lag compensation | The median of 5 describes the target two samples ago; the Kalman filter's speed estimate adds that lag back, so a moving target is not reported late. |

If the sensor sees nothing for 4 samples in a row the output becomes `-1` (no target) and the filters restart.

## What it sends (115200 baud, one line per reading)

```
D:<s0>[,<s1>[,<s2>]]     filtered distance, the value the robot should use
R:<s0>[,<s1>[,<s2>]]     raw readings
```

One value per connected sensor (`SENSOR_COUNT`): `D:73` with one sensor, `D:73,120` with two. Each value is
millimetres, `-1` for no target in range, `-2` for a sensor that did not start. The brain page shows both
lines so you can see what the filters are doing.

The USB Serial Monitor prints one sensor per line, `D:73 R:74`. `CONSOLE_SENSOR` in the sketch picks which one.

## If it does not work

Open the Arduino **Serial Monitor** at 115200 baud. Lines starting with `#` are diagnostics; the rest are frames.

| You see | It means | Check |
|---|---|---|
| `D:-2,-2,-2` / `R:-2,-2,-2` | The ESP32 is fine, but no VL53L0X answered over I2C. | The `# I2C scan` lines list what answers: a working sensor shows up at `0x29`. If nothing answers, the sketch also tries SDA and SCL swapped and says so (`SWAPPED`: swap the two wires). Otherwise check VIN to 3V3, GND to GND, that no wire is shorted, and the sensor's XSHUT pin. |
| `# I2C devices answering: 0` | Nothing at all is on the bus. | The sketch drives the XSHUT pin (GPIO26, XSHUT_PINS[0]) high at boot for a single sensor, so an XSHUT wired there is released. If XSHUT goes elsewhere, leave it unconnected or tie it to 3V3. If you tried all of this, the sensor may be damaged (for example from VIN and GND having been swapped). |
| `rst:0x1 (POWERON_RESET)` again and again | The board lost power. This is not a crash in the program. | A short or swapped VIN / GND on the sensor, a weak USB cable or port, or the RS-485 module pulling too much from 3V3. Try unplugging the sensor and the RS-485 module: if the resets stop, add them back one at a time. |
| `# boot: last reset = ...` | The reason for the last restart, in words. | |
| `D:x,-1,-2` | Working; the sensor sees no target within about 1.2 m. | |

On the brain, the DISTANCE page shows the same thing: NO DATA = nothing arrives over RS-485, SENSOR NOT FOUND =
the ESP32 is sending but the sensor is not answering.

## Tuning (`KalmanConfig` in `SensorFilters.h`)

| Setting | Raise it to... |
|---|---|
| `accelSigma` | follow a moving target faster (more noise gets through) |
| `noiseBase`, `noisePerMm` | trust the sensor less (smoother, slower) |
| `gateSigma` | accept more unusual readings before calling them outliers |
| `resetAfterOutliers` | make the filter slower to believe a real jump |

Set `PLOT_SENSOR` to `0` in the sketch and open the Arduino **Serial Plotter** to watch raw, median and
kalman drawn together.

## Testing the filters without hardware

```
cd VL53L0X_Code/extras
g++ -std=c++17 -I.. test_filters.cpp -o test_filters
./test_filters
```

It runs simulated noisy readings (spikes, steps, a moving target, dropouts) through the filters and prints
PASS or FAIL for each.
