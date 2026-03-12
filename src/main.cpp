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
pros::MotorGroup left_motor_group({-11, -12, -13, -14}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({1, 2, 3, 4}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor intake(16, pros::MotorGears::green);
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
pros::Imu imu(10);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(nullptr, // vertical wheel
                            nullptr,                  // second vertical (none)
                            nullptr, // horizontal wheel
                            nullptr, // second horizontal (none)
                            &imu     // imu
);

// Lateral & Angular PID
lemlib::ControllerSettings lateral_controller(10, 0, 25, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(1.8, 0, 10, 0, 0, 0, 0, 0, 0);

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

void drive_for_inches(double maxSpeed, double inches) {
  double targetDegrees = inchesToDegrees(std::abs(inches));
  double direction = (inches > 0) ? 1.0 : -1.0;

  left_motor_group.tare_position();
  right_motor_group.tare_position();
  imu.tare_rotation();

  const double accelRate = 2.0;
  const double decelStart = 0.6;
  const double kP = 0.8;
  const double kD = 6.5; // tune this — dampens overcorrection

  double currentSpeed = 0;
  double decelPoint = targetDegrees * decelStart;

  double lastHeading = 0;
  uint32_t startTime = pros::millis();
  const uint32_t timeout = (uint32_t)(std::abs(inches) * 100);

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    double heading = imu.get_rotation();
    double derivative = heading - lastHeading; // rate of heading change
    lastHeading = heading;

    double correction = (heading * kP) + (derivative * kD);

    if (avgPos < decelPoint) {
      currentSpeed += accelRate;
      if (currentSpeed > maxSpeed)
        currentSpeed = maxSpeed;
    } else {
      double remaining = targetDegrees - avgPos;
      currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
      if (currentSpeed < 10)
        currentSpeed = 10;
    }

    if (avgPos >= targetDegrees - 2 || pros::millis() - startTime > timeout)
      break;

    left_motor_group.move_velocity((currentSpeed - correction) * direction);
    right_motor_group.move_velocity((currentSpeed + correction) * direction);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastSpeed = std::max(currentSpeed, 10.0);
  while (lastSpeed > 0) {
    double heading = imu.get_rotation();
    double derivative = heading - lastHeading;
    lastHeading = heading;

    double correction = (heading * kP) + (derivative * kD);

    left_motor_group.move_velocity((lastSpeed - correction) * direction);
    right_motor_group.move_velocity((lastSpeed + correction) * direction);

    lastSpeed -= 2;
    if (lastSpeed < 0)
      lastSpeed = 0;
    pros::delay(10);
  }

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void turn_to_heading(double targetHeading, double maxSpeed) {
  const double kP = 1.5;
  const double kD = 0.8; // tune this — prevents overshooting the turn
  const double threshold = 1.0;

  double lastError = 0;
  uint32_t startTime = pros::millis();
  const uint32_t timeout = 3000;

  while (true) {
    double heading = imu.get_rotation();
    double error = targetHeading - heading;

    // Normalize to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    double derivative = error - lastError; // rate of error change
    lastError = error;

    if (std::abs(error) <= threshold || pros::millis() - startTime > timeout)
      break;

    double speed = (error * kP) + (derivative * kD);

    // Clamp speed
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    // Minimum speed so motors don't stall
    if (speed > 0 && speed < 10) speed = 10;
    if (speed < 0 && speed > -10) speed = -10;

    left_motor_group.move_velocity(speed);
    right_motor_group.move_velocity(-speed);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastSpeed = 15;
  double heading = imu.get_rotation();
  double error = targetHeading - heading;
  double stopDir = (error > 0) ? 1.0 : -1.0;

  while (lastSpeed > 0) {
    left_motor_group.move_velocity(lastSpeed * stopDir);
    right_motor_group.move_velocity(-lastSpeed * stopDir);
    lastSpeed -= 2;
    pros::delay(10);
  }

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void shoot(double velocity = 200) {
  catapult_arm.move_absolute(-550, velocity);
  discore.move_absolute(0, 200);
  pros::delay(300);
  catapult_arm.move_absolute(0, velocity);
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
  const int SLOW_SPEED = 50;  // Adjust this for how slow you want
  static bool controlsReversed = false;
  static bool wasDownHeld = false;  // Track previous frame state
  
  while (true) {
    pros::delay(20);

    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool downHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool downTapped = downHeld && !wasDownHeld;  // Just pressed this frame
    wasDownHeld = downHeld;  // Store for next frame

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Toggle controls only on TAP
    if (downTapped)
      controlsReversed = !controlsReversed;

    if (controlsReversed)
      move = -move;

    // Apply slow speed if button is HELD
    int maxSpeed = downHeld ? SLOW_SPEED : MAX_SPEED;

    int leftMotorSpeed = std::clamp(move + turn, -maxSpeed, maxSpeed);
    int rightMotorSpeed = std::clamp(move - turn, -maxSpeed, maxSpeed);

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
      intake.move_velocity(600);
    else if (intakeReverse && !intakeForward)
      intake.move_velocity(-600);
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

void test() {
  drive_for_inches(80, 15);
  pros::delay(500);
  turn_to_heading(90, 100);
}

// --- Autonomous ---
void autonomous() { test(); }

