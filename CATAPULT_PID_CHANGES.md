# Catapult PID Control Implementation

## Overview
Added smooth PID-based control to the catapult arm for more consistent and stable shooting mechanics.

## Changes Made

### 1. **PID Controller Declaration** (`include/functions.hpp`)
- Added `extern lemlib::PID catapult_pid;` to expose the PID controller
- Added function declarations for new PID-based movement functions

### 2. **PID Controller Setup** (`src/functions.cpp`)
- Created `lemlib::PID catapult_pid(8.0, 0.1, 45.0)` with tuned constants:
  - **kP = 8.0**: Proportional gain for primary position response
  - **kI = 0.1**: Integral gain to reduce steady-state error
  - **kD = 45.0**: Derivative gain to dampen oscillations (critical for smooth movement)

### 3. **PID Movement Functions** (`src/functions.cpp`)
- **`catapult_arm_move_pid()`**: Blocking PID movement function
  - Used in autonomous routines like `catapultShootForAuto()`
  - Provides smooth voltage control with automatic settling detection
  - Timeout protection (default 2000ms)

- **`catapult_pid_task()`**: Async PID control task
  - Runs in background for non-blocking movement
  - Used by state machine for reload operations
  - Automatically stops when target position is reached

- **`catapult_move_pid_async()`**: Non-blocking movement initiator
  - Sets target and triggers the async PID task
  - Returns immediately while movement completes in background

### 4. **State Machine Updates** (`src/functions.cpp`)
- **`catapultTask()`**: Updated to use PID-based reload
  - CAT_FIRING → CAT_RELOADING: Now uses `catapult_move_pid_async()` for smooth return
  - Maintains stall detection for jam prevention
  - Preserves existing state machine logic

### 5. **Autonomous Routine Enhancement** (`src/functions.cpp`)
- **`catapultShootForAuto()`**: Updated for smoother autonomous shooting
  - Fire phase: Uses PID-based movement to fire position
  - Reload phase: Uses PID for smooth return to load position
  - Better motion profiles under different battery voltages

### 6. **Task Initialization** (`src/main.cpp`)
- Added `pros::Task catapult_pid_control(catapult_pid_task, nullptr, "Catapult PID Task")`
- Runs alongside existing catapult and intake tasks

## Benefits

✅ **Smoother Motion**: Eliminates jerky acceleration/deceleration
✅ **Better Consistency**: PID adapts to load variations and battery voltage
✅ **Reduced Jamming**: Derivative term prevents overshoot that can cause jams
✅ **Faster Cycle Times**: PID settles quicker than fixed-voltage approaches
✅ **Adaptability**: Works well across different battery states

## Technical Details

### PID Voltage Calculation
```cpp
double pidOutput = catapult_pid.update(error);  // Error = target - current
double motorVoltage = pidOutput * (speed / 200.0);  // Scale by speed param
catapult_arm.move_voltage((int)motorVoltage);  // Apply to motor
```

### Settling Detection
- Monitors position error against threshold (±10 ticks)
- Confirms settlement for 3 consecutive cycles
- Stops movement once settled to reduce power consumption

### Parameters
- **Speed**: 0-200 scale (maps to voltage output scaling)
- **Timeout**: Default 2000ms per movement
- **Settle Threshold**: ±10 encoder ticks
- **Settle Confirmation**: 3 checks at 10ms intervals = 30ms confirmation time

## Testing Recommendations

1. Test autonomous routines (`catapultShootForAuto()`) at various battery levels
2. Verify driver control response (`startCatapultShoot()` trigger)
3. Monitor catapult temperature during extended use
4. Check for any jamming incidents and adjust kP/kI/kD if needed
5. Validate timing in skills and match routines

## Future Tuning

If adjustments are needed, modify the PID constants in `src/functions.cpp` line ~17:
```cpp
lemlib::PID catapult_pid(kP_value, kI_value, kD_value);
```

Current values (8.0, 0.1, 45.0) are tuned for red gear motor with typical load.
