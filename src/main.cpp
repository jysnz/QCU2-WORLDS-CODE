#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

bool drive_task_running = false;
bool drive_backward_task_running = false;

std::vector<double> recordedPaths; // Stores inches
bool recordingActive = false;
double liveInches = 0;

enum CatapultState { CAT_IDLE, CAT_FIRING, CAT_RELOADING };

CatapultState catState = CAT_IDLE;

int catAttempts = 0;
bool shotSuccess = false;

const int LOAD_POS = 0;
const int FIRE_POS = -600;
const int SPEED = 200;

const int STALL_TIME = 250;
const int CHECK_DELAY = 10;
const int MAX_ATTEMPTS = 10;

int stalledTime = 0;

// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0, heading = 0.0;

// ms required to turn 90 degrees at TURN_SPEED
const double MS_PER_90_DEG = 660.0;

// --- Constants ---
const double wheelDiameter = 3.25; // inches
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // motor degrees per revolution
double turnCalibration = 1.80;
const double STOP_TOLERANCE =
    5.0; // Stop when within +/- 5 motor degrees of the target

const float PI = 3.14159;
bool in_motion = false;

// PID constants
float kP = 1.0;
float kI = 0.0;
float kD = 0.5;

// --- Motors ---
pros::MotorGroup left_motor_group({-1, -2, -3, -4}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({11, 12, 13, 14}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor intake(19, pros::MotorGears::green);
pros::Motor matchloader(5, pros::MotorGears::red);
pros::Motor discore(15, pros::MotorGears::green);

// Plug 'Ping' into port E, 'Echo' into port F (Change letters as needed)
pros::adi::Ultrasonic ultrasonic('A', 'B');

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Drivetrain ---
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              400,                        // drivetrain rpm
                              2                           // horizontal drift
);

// --- Odometry ---
pros::Imu imu(17);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical wheel
                            nullptr,                  // second vertical (none)
                            &horizontal_tracking_wheel, // horizontal wheel
                            nullptr, // second horizontal (none)
                            &imu     // imu
);

// Lateral & Angular PID
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);

// Expo drive curves
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

double prevLeft = 0.0;
double prevRight = 0.0;

double sign(double x) { return (x > 0) - (x < 0); }

void catapultTask(void *) {
  while (true) {
    double pos = catapult_arm.get_position();
    double vel = std::abs(catapult_arm.get_actual_velocity());

    switch (catState) {

    case CAT_IDLE:
      // Do nothing
      break;

    case CAT_FIRING:
      // Shot completed
      if (pos <= FIRE_POS + 25) {
        shotSuccess = true;
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, SPEED);
        break;
      }

      // Stall detection
      if (vel < 5)
        stalledTime += CHECK_DELAY;
      else
        stalledTime = 0;

      // Jam detected
      if (stalledTime >= STALL_TIME) {
        catState = CAT_RELOADING;
        catapult_arm.move_absolute(LOAD_POS, SPEED);
      }
      break;

    case CAT_RELOADING:
      // Finished reload
      if (std::abs(pos - LOAD_POS) < 10) {

        if (shotSuccess || ++catAttempts >= MAX_ATTEMPTS) {
          // Done
          catState = CAT_IDLE;
          catAttempts = 0;
          shotSuccess = false;
        } else {
          // Retry fire
          catState = CAT_FIRING;
          stalledTime = 0;
          catapult_arm.move_absolute(FIRE_POS, SPEED);
        }
      }
      break;
    }

    pros::delay(CHECK_DELAY);
  }
}

void startCatapultShoot() {
  if (catState != CAT_IDLE)
    return;

  catAttempts = 0;
  stalledTime = 0;
  shotSuccess = false;

  catState = CAT_FIRING;
  catapult_arm.move_absolute(FIRE_POS, SPEED);
}

// --- Helper functions ---
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * wheelDiameter;
}

void updateOdometry() {
  float leftDist = ticksToInches(left_motor_group.get_position());
  float rightDist = ticksToInches(right_motor_group.get_position());
  float distance = (leftDist + rightDist) / 2.0;
  float deltaTheta = (rightDist - leftDist) / trackWidth;

  heading += deltaTheta * (180.0 / PI); // degrees
  x += distance * cos(heading * PI / 180.0);
  y += distance * sin(heading * PI / 180.0);
}

void odometryTask() {
  left_motor_group.tare_position();
  right_motor_group.tare_position();
  prevLeft = 0;
  prevRight = 0;

  while (true) {
    updateOdometry();
    pros::delay(10);
  }
}

int turn_deg_to_ms(double degrees) {
  return static_cast<int>((degrees / 90.0) * MS_PER_90_DEG);
}

// --- New drive_for_inches function ---
double inchesToDegrees(double inches) {
  double wheelCircumference = PI * wheelDiameter;
  double rotations = inches / wheelCircumference;
  return rotations * 360.0; // degrees
}

void turn_left_deg(double degrees, int speed, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_velocity(-speed);
  right_motor_group.move_velocity(speed);

  pros::delay(timeMs);

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(delay);
}

void turn_right_deg(double degrees, int speed, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_velocity(speed);
  right_motor_group.move_velocity(-speed);

  pros::delay(timeMs);

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(delay);
}

void turn_left_deg_vol(double degrees, int voltage, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_voltage(-voltage);
  right_motor_group.move_voltage(voltage);

  pros::delay(timeMs);

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);

  pros::delay(delay);
}

void turn_right_deg_vol(double degrees, int voltage, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(-voltage);

  pros::delay(timeMs);

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);

  pros::delay(delay);
}

bool drive_hit_wall() {
  static int stoppedTime = 0;

  const double VEL_THRESH = 5; // rpm
  const int STOP_TIME_MS = 150;

  double lv = std::abs(left_motor_group.get_actual_velocity());
  double rv = std::abs(right_motor_group.get_actual_velocity());

  if (lv < VEL_THRESH && rv < VEL_THRESH) {
    stoppedTime += 10;
    if (stoppedTime >= STOP_TIME_MS)
      return true;
  } else {
    stoppedTime = 0;
  }

  return false;
}

bool drive_forward_nb_running = false;

void drive_forward_nb_task(void *param) {
  drive_forward_nb_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];

  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelRate = 2.0;
  double currentSpeed = 0;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // 🧱 HIT WALL → STOP
    if (drive_hit_wall())
      break;

    // 🎯 DISTANCE REACHED
    if (avgPos >= targetDegrees - 2)
      break;

    // ACCEL ONLY
    currentSpeed += accelRate;
    if (currentSpeed > maxSpeed)
      currentSpeed = maxSpeed;

    left_motor_group.move_velocity(currentSpeed);
    right_motor_group.move_velocity(currentSpeed);

    pros::delay(10);
  }

  // HARD STOP
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  delete[] args;
  drive_forward_nb_running = false;
}

void drive_for_inches_async_nonblocking(double maxSpeed, double inches) {
  if (drive_forward_nb_running)
    return;

  double *args = new double[2]{maxSpeed, inches};
  pros::Task(drive_forward_nb_task, args, "Drive Forward NB");
}

bool drive_backward_nb_running = false;

void drive_backward_nb_task(void *param) {
  drive_backward_nb_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];

  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelRate = 2.0;
  double currentSpeed = 0;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // 🧱 HIT WALL → STOP
    if (drive_hit_wall())
      break;

    // 🎯 DISTANCE REACHED
    if (avgPos >= targetDegrees - 2)
      break;

    // ACCEL ONLY
    currentSpeed += accelRate;
    if (currentSpeed > maxSpeed)
      currentSpeed = maxSpeed;

    left_motor_group.move_velocity(-currentSpeed);
    right_motor_group.move_velocity(-currentSpeed);

    pros::delay(10);
  }

  // HARD STOP
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  delete[] args;
  drive_backward_nb_running = false;
}

void drive_backward_for_inches_async_nonblocking(double maxSpeed,
                                                 double inches) {
  if (drive_backward_nb_running)
    return;

  double *args = new double[2]{maxSpeed, inches};
  pros::Task(drive_backward_nb_task, args, "Drive Backward NB");
}

void drive_for_inches(double maxSpeed, double inches) {
  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelRate = 2.0;
  const double decelStart = 0.6;

  double currentSpeed = 0;
  double decelPoint = targetDegrees * decelStart;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // ACCELERATION
    if (avgPos < decelPoint) {
      currentSpeed += accelRate;
      if (currentSpeed > maxSpeed)
        currentSpeed = maxSpeed;
    }
    // DECELERATION
    else {
      double remaining = targetDegrees - avgPos;
      currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
      if (currentSpeed < 10)
        currentSpeed = 10; // lower min for smooth stop
    }

    // END CONDITION
    if (avgPos >= targetDegrees - 2)
      break;

    left_motor_group.move_velocity(currentSpeed);
    right_motor_group.move_velocity(currentSpeed);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastSpeed =
      std::max(currentSpeed, 10.0); // start ramp-down from current speed
  while (lastSpeed > 0) {
    left_motor_group.move_velocity(lastSpeed);
    right_motor_group.move_velocity(lastSpeed);
    lastSpeed -= 2; // small decrement for smooth stop
    if (lastSpeed < 0)
      lastSpeed = 0;
    pros::delay(10);
  }

  // Hard stop
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void drive_backward_for_inches(double maxSpeed, double inches) {
  // targetDegrees is the magnitude of the rotation needed (always positive)
  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  // Constant parameters
  const double accelRate = 2.0;
  const double decelStart = 0.6; // Start decelerating at 60% of the distance

  double currentSpeed = 0;
  double decelPoint = targetDegrees * decelStart;

  // We will use a negative speed command to move backward
  double backwardSpeedCommand = 0.0;

  while (true) {
    // Use the absolute value for position tracking, as in the original
    // function. This keeps the acceleration/deceleration logic simple and
    // positive-based.
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // ACCELERATION (same logic as forward)
    if (avgPos < decelPoint) {
      currentSpeed += accelRate;
      if (currentSpeed > maxSpeed)
        currentSpeed = maxSpeed;
    }
    // DECELERATION (same logic as forward)
    else {
      double remaining = targetDegrees - avgPos;
      currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
      if (currentSpeed < 10)
        currentSpeed = 10; // lower min for smooth stop
    }

    // Set the final speed command to be negative for backward movement
    backwardSpeedCommand = -currentSpeed;

    // END CONDITION
    if (avgPos >= targetDegrees - 2)
      break; // Stop a little early

    left_motor_group.move_velocity(backwardSpeedCommand);
    right_motor_group.move_velocity(backwardSpeedCommand);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP (Ramp down to 0) -----
  // We ramp down the NEGATIVE speed towards 0
  double lastSpeedCommand = std::min(
      backwardSpeedCommand, -10.0); // start ramp-down from current speed
  while (lastSpeedCommand < 0) {    // loop while the command is negative
    left_motor_group.move_velocity(lastSpeedCommand);
    right_motor_group.move_velocity(lastSpeedCommand);
    lastSpeedCommand += 2; // small POSITIVE increment to approach 0
    if (lastSpeedCommand > 0)
      lastSpeedCommand = 0;
    pros::delay(10);
  }

  // Hard stop
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void drive_backward_task_fn(void *param) {
  drive_backward_task_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];
  int delayMs = static_cast<int>(args[2]);

  drive_backward_for_inches(maxSpeed, inches);

  // ✅ DELAY BEFORE NEXT MOVEMENT IS ALLOWED
  pros::delay(delayMs);

  delete[] args;
  drive_backward_task_running = false;
}

void drive_backward_inches_async(double maxSpeed, double inches, int delayMs) {
  if (drive_backward_task_running)
    return;

  double *args = new double[3];
  args[0] = maxSpeed;
  args[1] = inches;
  args[2] = static_cast<double>(delayMs);

  pros::Task driveBackwardTask(drive_backward_task_fn, args,
                               "Drive Backward Task");
}

void drive_task_fn(void *param) {
  drive_task_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];
  int delayMs = static_cast<int>(args[2]);

  drive_for_inches(maxSpeed, inches);

  // ✅ DELAY BEFORE NEXT MOVEMENT IS ALLOWED
  pros::delay(delayMs);

  delete[] args;
  drive_task_running = false;
}

void drive_for_inches_async(double maxSpeed, double inches, int delayMs) {
  if (drive_task_running)
    return; // 🚫 block second start

  double *args = new double[3];
  args[0] = maxSpeed;
  args[1] = inches;
  args[2] = static_cast<double>(delayMs);

  pros::Task driveTask(drive_task_fn, args, "Drive Task");
}

void shoot(double velocity = 200) {
  catapult_arm.move_absolute(-550, velocity);
  discore.move_absolute(0, 200);
  pros::delay(300);
  catapult_arm.move_absolute(0, velocity);
}

void drive_for_inches_voltage(double maxVoltage, double inches) {
  double targetDegrees = inchesToDegrees(std::abs(inches));
  double direction = sign(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelStep = 300; // voltage per 10ms
  const double decelStart = 0.6;
  const double minVoltage = 1200; // prevents stall

  double currentVoltage = 0;
  double decelPoint = targetDegrees * decelStart;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // ACCELERATION
    if (avgPos < decelPoint) {
      currentVoltage += accelStep;
      if (currentVoltage > maxVoltage)
        currentVoltage = maxVoltage;
    }
    // DECELERATION
    else {
      double remaining = targetDegrees - avgPos;
      currentVoltage = maxVoltage * (remaining / (targetDegrees - decelPoint));
      if (currentVoltage < minVoltage)
        currentVoltage = minVoltage;
    }

    // END CONDITION (early stop like your velocity code)
    if (avgPos >= targetDegrees - 2)
      break;

    left_motor_group.move_voltage(currentVoltage * direction);
    right_motor_group.move_voltage(currentVoltage * direction);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastVoltage = std::max(currentVoltage, minVoltage);
  while (lastVoltage > 0) {
    left_motor_group.move_voltage(lastVoltage * direction);
    right_motor_group.move_voltage(lastVoltage * direction);
    lastVoltage -= 200; // small ramp-down
    if (lastVoltage < 0)
      lastVoltage = 0;
    pros::delay(10);
  }

  // Hard stop
  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);
}

void drive_for_inches_voltage_simple(double maxVoltage, double inches) {
  double targetDegrees = inchesToDegrees(std::abs(inches));
  double direction = sign(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelStep = 400;
  const double minVoltage = 1200;

  double currentVoltage = 0;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    if (avgPos >= targetDegrees - 2)
      break;

    currentVoltage += accelStep;
    if (currentVoltage > maxVoltage)
      currentVoltage = maxVoltage;

    left_motor_group.move_voltage(currentVoltage * direction);
    right_motor_group.move_voltage(currentVoltage * direction);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastVoltage = std::max(currentVoltage, minVoltage);
  while (lastVoltage > 0) {
    left_motor_group.move_voltage(lastVoltage * direction);
    right_motor_group.move_voltage(lastVoltage * direction);
    lastVoltage -= 200;
    if (lastVoltage < 0)
      lastVoltage = 0;
    pros::delay(10);
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);
}

void wall_reset(int voltage = 8000, int settleTime = 200) {
  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(voltage);

  int stalledTime = 0;

  while (stalledTime < settleTime) {
    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());

    if (lv < 3 && rv < 3) {
      stalledTime += 10;
    } else {
      stalledTime = 0;
    }

    pros::delay(10);
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);

  pros::delay(50);
}

void catapultShoot() {
  const int LOAD_POS = 0;
  const int FIRE_POS = -600;
  const int SPEED = 200;

  const int STALL_TIME = 250;
  const int CHECK_DELAY = 10;
  const int MAX_ATTEMPTS = 10;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {

    bool shotSuccess = false;

    // ---- FIRE ----
    catapult_arm.move_absolute(FIRE_POS, SPEED);

    int stalledTime = 0;

    while (true) {
      pros::delay(CHECK_DELAY);

      double pos = catapult_arm.get_position();
      double vel = std::abs(catapult_arm.get_actual_velocity());

      // ✅ Successful fire
      if (pos <= FIRE_POS + 25) {
        shotSuccess = true;
        break;
      }

      // ❌ Jam detection
      if (vel < 5)
        stalledTime += CHECK_DELAY;
      else
        stalledTime = 0;

      if (stalledTime >= STALL_TIME)
        break;
    }

    // ---- RESET / RELOAD ----
    catapult_arm.move_absolute(LOAD_POS, SPEED);

    while (std::abs(catapult_arm.get_position() - LOAD_POS) > 10) {
      pros::delay(10);
    }

    // ✅ If shot worked, we are done
    if (shotSuccess)
      return;

    // ❌ Otherwise: loop repeats → retry fire
  }

  // Failsafe
  catapult_arm.move_velocity(0);
}

void wall_reset_v2(int voltage = 8000, int settleTime = 200, int direction = 1,
                   int timeout = 1000) {
  // direction:  1 = forward
  //            -1 = backward

  voltage = std::abs(voltage) * direction;

  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(voltage);

  int stalledTime = 0;
  int elapsed = 0;

  while (stalledTime < settleTime && elapsed < timeout) {
    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());

    if (lv < 3 && rv < 3) {
      stalledTime += 10;
    } else {
      stalledTime = 0;
    }

    pros::delay(10);
    elapsed += 10;
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);

  pros::delay(50);
}

void catapultShootForAuto(double SPEED) {
  const int LOAD_POS = 0;
  const int FIRE_POS = -600;

  const int STALL_TIME = 250;
  const int CHECK_DELAY = 10;
  const int MAX_ATTEMPTS = 10;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {

    bool shotSuccess = false;

    // ---- FIRE ----
    catapult_arm.move_absolute(FIRE_POS, SPEED);

    int stalledTime = 0;

    while (true) {
      pros::delay(CHECK_DELAY);

      double pos = catapult_arm.get_position();
      double vel = std::abs(catapult_arm.get_actual_velocity());

      // ✅ Successful fire
      if (pos <= FIRE_POS + 25) {
        shotSuccess = true;
        break;
      }

      // ❌ Jam detection
      if (vel < 5)
        stalledTime += CHECK_DELAY;
      else
        stalledTime = 0;

      if (stalledTime >= STALL_TIME)
        break;
    }

    // ---- RESET / RELOAD ----
    catapult_arm.move_absolute(LOAD_POS, SPEED);

    while (std::abs(catapult_arm.get_position() - LOAD_POS) > 10) {
      pros::delay(10);
    }

    // ✅ If shot worked, we are done
    if (shotSuccess)
      return;

    // ❌ Otherwise: loop repeats → retry fire
  }

  // Failsafe
  catapult_arm.move_velocity(0);
}

void catapultControl() {
  const int MAX_SPEED = 127;
  static bool controlsReversed = false;
  while (true) {
    // pros::lcd::print(0, "X: %f", robot_x);
    // pros::lcd::print(1, "Y: %f", robot_y);
    // pros::lcd::print(2, "Theta: %f", robot_theta * (180/M_PI)); // Convert to
    // degrees
    pros::delay(20);

    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool reverseControlTap =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (reverseControlTap)
      controlsReversed = !controlsReversed;

    if (controlsReversed)
      move = -move;

    int leftMotorSpeed = std::clamp(move + turn, -MAX_SPEED, MAX_SPEED);
    int rightMotorSpeed = std::clamp(move - turn, -MAX_SPEED, MAX_SPEED);

    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    if (catapultArm) {
      startCatapultShoot();
    }

    if (discoreDown) {
      discore.move_absolute(0, 200);
    } else if (discoreUp)
      discore.move_absolute(800, 200);

    if (matchLoadUp && !matchLoadDown) {
      matchloader.move_absolute(0, 100);
      discore.move_absolute(800, 200);
    } else if (matchLoadDown && !matchLoadUp) {
      matchloader.move_absolute(1400, 100);
      discore.move_absolute(0, 200);
    }

    if (intakeForward && !intakeReverse)
      intake.move_velocity(200);
    else if (intakeReverse && !intakeForward)
      intake.move_velocity(-200);
    if (intakePause)
      intake.move_velocity(0);

    pros::delay(20);
  }
}

// --- Initialize ---
void initialize() {
  pros::Task catapult_control(catapultTask, nullptr, "Catapult Task");
  catapult_arm.tare_position();
  matchloader.tare_position();
  discore.tare_position();
  pros::lcd::initialize();
  chassis.calibrate();
  left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  static pros::Task screen_task([]() {
    const int P_X = 240;
    const int P_W = 240;
    const int BG_COLOR = 0x202020;

    while (true) {
      pros::screen::set_pen(BG_COLOR);
      pros::screen::fill_rect(P_X, 0, 480, 240);

      double bat = pros::battery::get_capacity();
      int bat_y = 20;
      int bat_h = 30;
      int bat_w = 70;
      int icon_x = P_X + 130;

      pros::screen::set_pen(0xFFFFFF);
      pros::screen::print(pros::E_TEXT_LARGE, P_X + 10, bat_y + 3,
                          "BAT: %3.0f%%", bat);

      uint32_t bat_col =
          (bat > 60) ? 0x00FFFF : (bat > 30 ? 0xFFA500 : 0xFF0000);
      pros::screen::set_pen(0xFFFFFF);
      pros::screen::draw_rect(icon_x, bat_y, icon_x + bat_w, bat_y + bat_h);
      pros::screen::fill_rect(icon_x + bat_w, bat_y + 8, icon_x + bat_w + 5,
                              bat_y + bat_h - 8);

      int fill = (int)((bat / 100.0) * (bat_w - 4));
      pros::screen::set_pen(bat_col);
      pros::screen::fill_rect(icon_x + 2, bat_y + 2, icon_x + 2 + fill,
                              bat_y + bat_h - 2);

      auto drawRow = [&](int row_idx, const char *label, double temp) {
        int row_h = 40;
        int start_y = 70;
        int y = start_y + (row_idx * row_h);

        pros::screen::set_pen(0xFFFFFF);
        pros::screen::print(pros::E_TEXT_MEDIUM, P_X + 10, y + 8, label);

        int bar_x = P_X + 80;
        int bar_w = 100;
        int bar_h = 16;
        int bar_y = y + 6;

        pros::screen::set_pen(0x404040);
        pros::screen::fill_rect(bar_x, bar_y, bar_x + bar_w, bar_y + bar_h);

        double stress = temp / 60.0;
        if (stress > 1.0)
          stress = 1.0;
        int fill_w = (int)(stress * bar_w);

        uint32_t col = 0x00FF00;
        if (temp > 45)
          col = 0xFFA500;
        if (temp > 55)
          col = 0xFF0000;

        pros::screen::set_pen(col);
        pros::screen::fill_rect(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h);

        pros::screen::set_pen(0xFFFFFF);
        pros::screen::print(pros::E_TEXT_SMALL, bar_x + bar_w + 10, y + 8,
                            "%.0fC", temp);
      };

      double d_temp = (left_motor_group.get_temperature() +
                       right_motor_group.get_temperature()) /
                      2.0;

      drawRow(0, "Drive", d_temp);
      drawRow(1, "Cata", catapult_arm.get_temperature());
      drawRow(2, "Intake", intake.get_temperature());
      drawRow(3, "Load", matchloader.get_temperature());
      drawRow(4, "Disc", discore.get_temperature());

      pros::delay(200);
    }
  });
}

// --- Operator Control ---
void opcontrol() { catapultControl(); }

void twoVtwo() {
  catapult_arm.tare_position();
  matchloader.tare_position();
  discore.tare_position();

  // Go to matchload
  matchloader.move_absolute(1400, 200); // Matchload down
  intake.move_velocity(-200);           // Intake ball
  discore.move_absolute(850, 200);      // Descore up
  drive_for_inches(80, 25);             // Move forward

  pros::delay(500); // Delay

  // Turn to matchload
  turn_right_deg(120, 30, 500); // Turn right

  // Long goal reset and shoot
  wall_reset_v2(
      7000, 200,
      -1); // Wall reset Direction = -1 (Backward), Direction = 1 (Forward)
  pros::delay(500);
  discore.move_absolute(0, 200);
  catapultShootForAuto(100);

  // Long goal reset
  drive_for_inches(80, 3);
  discore.move_absolute(850, 200);
  wall_reset_v2(10000, 200, -1);
  drive_for_inches(80, 3);
  wall_reset_v2(10000, 200, -1);
  pros::delay(500);

  // Gather matchload
  drive_for_inches(40, 10);
  wall_reset_v2(5000, 250);
  pros::delay(500);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000, 100);

  // Drive back to long goal
  drive_backward_for_inches(40, 10);
  wall_reset_v2(8000, 200, -1);
  pros::delay(500);
  matchloader.move_absolute(0, 200);

  // Shoot
  discore.move_absolute(0, 200);
  pros::delay(500);
  catapultShootForAuto(100);
  // pros::delay(250);

  // // Long goal reset
  // drive_for_inches(80, 2);
  // wall_reset_v2(8000, 200, -1);
  // drive_for_inches(80, 2);
  // wall_reset_v2(8000, 200, -1);

  // pros::delay(500);

  // // Descore
  // drive_for_inches(50, 2);
  // pros::delay(500);
  // turn_left_deg(57, 40, 500);
  // drive_backward_for_inches(50, 2.9);
  // pros::delay(250);

  // right_motor_group.move_velocity(-50);
  // pros::delay(750);
  // right_motor_group.move_velocity(0);

  // drive_backward_for_inches(100, 6);
  // discore.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void skillsV3() {
  catapult_arm.tare_position();
  matchloader.tare_position();
  discore.tare_position();
  // // Intake two balls
  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(80, 29);
  wall_reset_v2(7000);
  drive_backward_for_inches(80, 6);
  turn_right_deg(120, 30, 500);
  pros::delay(500);

  // Long goal reset
  wall_reset_v2(7000, 200, -1);
  pros::delay(500);

  // Shoot
  discore.move_absolute(0, 200);
  catapultShootForAuto(200);
  pros::delay(500);

  discore.move_absolute(850, 200);
  matchloader.move_absolute(1400, 200);

  // Long goal reset
  wall_reset_v2(
      7000, 200,
      -1); // Wall reset Direction = -1 (Backward), Direction = 1 (Forward)
  pros::delay(500);

  // Long goal reset
  drive_for_inches(80, 3);
  wall_reset_v2(10000, 200, -1);
  drive_for_inches(80, 3);
  wall_reset_v2(10000, 200, -1);
  pros::delay(500);

  // Gather matchload

  drive_for_inches(50, 15);
  pros::delay(350);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000, 200);

  // Drive back to long goal
  drive_backward_for_inches(40, 10);
  turn_left_deg(10, 50, 250);
  wall_reset_v2(8000, 200, -1);
  pros::delay(500);

  // Shoot
  discore.move_absolute(0, 200);
  pros::delay(500);
  catapultShootForAuto(200);
  matchloader.move_absolute(0, 200);
  pros::delay(250);

  // Long goal reset
  drive_for_inches(80, 2);
  wall_reset_v2(8000, 200, -1);
  drive_for_inches(80, 2);
  wall_reset_v2(8000, 200, -1);

  pros::delay(500);

  // Park
  discore.move_absolute(850, 200);
  drive_for_inches(80, 9);
  turn_right_deg(55, 40, 500);

  drive_for_inches(150, 30);
}

void twovtwoWithMatchload() {
  catapult_arm.tare_position();
  matchloader.tare_position();
  discore.tare_position();

  // Go to matchload
  matchloader.move_absolute(1400, 200); // Matchload down
  discore.move_absolute(850, 200);      // Descore up
  drive_for_inches(80, 24.5);           // Move forward

  pros::delay(500); // Delay

  // Turn to matchload
  turn_right_deg(120, 30, 500); // Turn right

  // Long goal reset
  drive_for_inches(80, 3);
  wall_reset_v2(10000, 200, 1);
  pros::delay(500);

  // Gather matchload
  intake.move_velocity(-200); // Intake ball
  pros::delay(400);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000);

  // Drive back to long goal
  drive_backward_for_inches(40, 10);
  turn_left_deg(10, 50, 250);
  wall_reset_v2(8000, 200, -1);
  pros::delay(500);

  // Shoot
  discore.move_absolute(0, 200);
  pros::delay(500);
  catapultShootForAuto(100);
  pros::delay(250);

  // Long goal reset
  drive_for_inches(80, 2);
  wall_reset_v2(8000, 200, -1);

  // Gather matchload
  intake.move_velocity(-200); // Intake ball
  drive_for_inches(50, 16);
  pros::delay(300);
  drive_backward_for_inches(80, .5); // Backward drive
  wall_reset_v2(4000, 0);

  drive_backward_for_inches(80, 2);
  turn_right_deg(30, 70, 500);
  catapultShootForAuto(100);
  pros::delay(500);
  turn_left_deg(50, 70, 500);
  drive_for_inches(80, 2);

  wall_reset_v2(4000, 100, 1);
  pros::delay(4000);

  // Drive back to long goal
  drive_backward_for_inches(40, 10);
  turn_left_deg(10, 50, 250);
  wall_reset_v2(8000, 200, -1);
  pros::delay(500);

  // Shoot
  catapultShootForAuto(100);
  matchloader.move_absolute(0, 200);
  pros::delay(250);
}

void debug() {
  // Throw away the red balls
  matchloader.move_absolute(1400, 200);
  pros::delay(350);
  drive_for_inches(80, 2);
  intake.move_velocity(0);

  // Turn to wall
  turn_right_deg(150, 90, 0);
  drive_backward_for_inches(80, 3);
  matchloader.move_absolute(-1400, 200);
  turn_right_deg(40, 50, 250);

  // Wall reset
  wall_reset_v2();
}

void skillsV2() {
  // Reset the catapult
  // catapult_home();

  // Intake two balls
  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(120, 29);
  wall_reset_v2(7000);
  drive_backward_for_inches(80, 1.5);

  turn_left_deg(60, 50, 250);

  drive_for_inches(150, 42);
  pros::delay(250);

  // Throw away the red balls
  pros::delay(350);
  drive_for_inches(80, 2);

  // Turn to wall
  turn_right_deg(90, 90, 200);
  drive_backward_for_inches(80, 1.5);
  pros::delay(500);
  turn_right_deg(50, 50, 250);

  // Wall reset
  wall_reset_v2();
  pros::delay(250);

  // Drive backwards for turn
  drive_backward_for_inches(40, 7.4);
  pros::delay(500);

  // Turn to face the long goal
  turn_left_deg(120, 30, 250);

  // // Drive backward to long goal
  wall_reset_v2(6000, 200, -1);
  pros::delay(500);

  // Shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(500);
  catapult_arm.move_absolute(600, 200);
  pros::delay(500);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(500);
  catapult_arm.move_absolute(600, 200);
  pros::delay(500);
  matchloader.move_absolute(1400, 200);

  pros::delay(250);

  // Align the robot to the long goal
  drive_for_inches(50, 2);
  wall_reset_v2(8000, 200, -1);
  pros::delay(500);

  // Gather matchload
  drive_for_inches(40, 9.8);
  pros::delay(350);

  // Intake balls
  discore.move_absolute(800, 200);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);

  // Shoot the gathered balls
  pros::delay(500);
  drive_backward_for_inches(50, 13);
  wall_reset_v2(8000, 200, -1);
  pros::delay(1000);

  discore.move_absolute(0, 200);
  pros::delay(500);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(250);
  catapult_arm.move_absolute(600, 100);
  pros::delay(250);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(250);
  catapult_arm.move_absolute(600, 100);
  pros::delay(250);
  intake.move_velocity(0);

  pros::delay(500);

  // 2nd half
  drive_for_inches(100, 3);
  turn_right_deg(90, 40, 250);
  wall_reset_v2();

  drive_backward_for_inches(80, 53);

  // Turn to face the matchload
  turn_left_deg(125, 30, 250);
  wall_reset_v2(8000, 200, -1);
  drive_for_inches(80, 5);
  wall_reset_v2(8000, 200, -1);

  pros::delay(250);

  // Gather matchload
  drive_for_inches(40, 9.8);
  pros::delay(350);

  // Intake balls
  discore.move_absolute(800, 200);
  intake.move_velocity(-200);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 100);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 100);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 100);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 100);
  drive_backward_for_inches(40, 0.3);
  wall_reset_v2(6000, 1000);

  // Shoot the gathered balls
  pros::delay(500);
  drive_backward_for_inches(50, 13);
  wall_reset_v2(8000, 200, -1);
  pros::delay(1000);

  discore.move_absolute(0, 200);
  pros::delay(500);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(750);
  catapult_arm.move_absolute(600, 100);
  pros::delay(750);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(750);
  catapult_arm.move_absolute(600, 100);
  pros::delay(250);
  intake.move_velocity(0);

  // Park
  drive_for_inches(100, 3);

  matchloader.move_absolute(-1400, 200);

  turn_left_deg(90, 80, 250);

  drive_for_inches(120, 38);
  turn_left_deg(60, 70, 250);

  drive_for_inches(50, 10);
  drive_for_inches(120, 15);
}

void skills() {
  // Intake two balls
  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(80, 31.5);

  pros::delay(2000);

  drive_backward_for_inches(80, 1.5);
  pros::delay(400);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(420);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 35);
  pros::delay(500);
  matchloader.move_absolute(1400, 200);
  drive_for_inches_async_nonblocking(80, 5);

  intake.move_velocity(0);
  pros::delay(500);
  matchloader.move_absolute(0, 200);

  // Turn right to reset
  left_motor_group.move_velocity(100);
  right_motor_group.move_velocity(-100);
  pros::delay(500);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_backward_for_inches(80, 1);

  pros::delay(500);

  // Turn right to reset
  left_motor_group.move_velocity(50);
  right_motor_group.move_velocity(-50);
  pros::delay(510);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches_async_nonblocking(80, 5);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(450);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_backward_for_inches(60, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(100, 5);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  catapult_arm.move_absolute(300, 200);
  matchloader.move_absolute(1400, 200);

  pros::delay(500);

  drive_for_inches(50, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(80, 7);

  pros::delay(500);

  drive_for_inches(50, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(80, 7);

  pros::delay(500);

  drive_for_inches(40, 9);
  pros::delay(350);
  drive_backward_for_inches(40, 2);
  drive_for_inches_async_nonblocking(40, 3);

  intake.move_velocity(-200);
  pros::delay(2500);

  pros::delay(200);

  drive_backward_for_inches(50, 14);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  intake.move_velocity(0);
  catapult_arm.move_absolute(300, 200);

  pros::delay(500);

  drive_for_inches(60, 2);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(460);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 50);
  pros::delay(350);
  drive_for_inches_async_nonblocking(100, 10);

  pros::delay(500);

  drive_backward_for_inches(80, 8);

  // Turn right
  left_motor_group.move_velocity(50);
  right_motor_group.move_velocity(-50);
  pros::delay(460);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  drive_for_inches(60, 3);
  drive_for_inches_async_nonblocking(100, 10);

  intake.move_velocity(-200);
  pros::delay(2500);
  intake.move_velocity(0);

  pros::delay(350);

  drive_backward_for_inches(80, 6);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  matchloader.move_absolute(1400, 200);
  catapult_arm.move_absolute(300, 200);

  pros::delay(500);

  drive_for_inches(80, 3);
  matchloader.move_absolute(0, 200);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(700);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 45);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(500);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 12);
  pros::delay(500);
  drive_for_inches(100, 15);
}

void test() {
  left_motor_group.move(80);
  pros::delay(1800);
  left_motor_group.move(0);
}

void park() {
  pros::delay(3000);
  intake.move_velocity(-200);
  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  drive_backward_for_inches(80, 10);
  drive_for_inches(120, 25);
  matchloader.move_absolute(0, 100);
  pros::delay(7000);
}

void test1() {
  left_motor_group.move_velocity(200);
  right_motor_group.move_velocity(200);
  pros::delay(500);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}
// --- Autonomous ---
void autonomous() { test1(); }

// // Forward fire
// catapult_start(200, false);
// while (catapultState != CATA_IDLE) pros::delay(10);

// // Reverse fire
// catapult_start(200, true);
// while (catapultState != CATA_IDLE) pros::delay(10);

// Driver's control catapult function
//  int speedInput = 0;
//  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
//      speedInput = 200;   // fire forward
//  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
//      speedInput = -200;  // pull back / reverse

// catapult_manual(speedInput);
