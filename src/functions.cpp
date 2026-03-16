#include "functions.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

// ─── Constants ───────────────────────────────────────────────────────────────
const float PI             = 3.14159f;
const double wheelDiameter = 3.25;   // inches
const double ticksPerRev   = 360.0;  // motor degrees per revolution

// ─── Catapult state ───────────────────────────────────────────────────────────
enum CatapultState { CAT_IDLE, CAT_FIRING, CAT_RELOADING };
static CatapultState catState   = CAT_IDLE;
static int           catAttempts = 0;
static bool          shotSuccess = false;
static int           stalledTime = 0;

const int LOAD_POS     = 0;
const int FIRE_POS     = -600;
const int CAT_SPEED    = 200;
const int STALL_TIME   = 250;
const int CHECK_DELAY  = 10;
const int MAX_ATTEMPTS = 10;

// ─── Odometry helpers ─────────────────────────────────────────────────────────
float ticksToInches(float ticks) {
    return (ticks / ticksPerRev) * PI * (float)wheelDiameter;
}

double inchesToDegrees(double inches) {
    double wheelCircumference = PI * wheelDiameter;
    return (inches / wheelCircumference) * 360.0;
}

// ─── Catapult task (runs in background) ──────────────────────────────────────
void catapultTask(void *) {
    while (true) {
        double pos = catapult_arm.get_position();
        double vel = std::abs(catapult_arm.get_actual_velocity());

        switch (catState) {
        case CAT_IDLE:
            break;

        case CAT_FIRING:
            if (pos <= FIRE_POS + 25) {
                shotSuccess = true;
                catState    = CAT_RELOADING;
                catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);
                break;
            }
            if (vel < 5) stalledTime += CHECK_DELAY;
            else         stalledTime  = 0;

            if (stalledTime >= STALL_TIME) {
                catState = CAT_RELOADING;
                catapult_arm.move_absolute(LOAD_POS, CAT_SPEED);
            }
            break;

        case CAT_RELOADING:
            if (std::abs(pos - LOAD_POS) < 10) {
                if (shotSuccess || ++catAttempts >= MAX_ATTEMPTS) {
                    catState    = CAT_IDLE;
                    catAttempts = 0;
                    shotSuccess = false;
                } else {
                    catState    = CAT_FIRING;
                    stalledTime = 0;
                    catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);
                }
            }
            break;
        }

        pros::delay(CHECK_DELAY);
    }
}

void startCatapultShoot() {
    if (catState != CAT_IDLE) return;
    catAttempts = 0;
    stalledTime = 0;
    shotSuccess = false;
    catState    = CAT_FIRING;
    catapult_arm.move_absolute(FIRE_POS, CAT_SPEED);
}

// ─── Drive functions ──────────────────────────────────────────────────────────
void drive_for_inches(double maxSpeed, double inches) {
    double targetDegrees = inchesToDegrees(std::abs(inches));
    double direction     = (inches > 0) ? 1.0 : -1.0;

    left_motor_group.tare_position();
    right_motor_group.tare_position();
    imu.tare_rotation();

    const double accelRate  = 2.0;
    const double decelStart = 0.6;
    const double kP         = 0.8;

    double   currentSpeed = 0;
    double   decelPoint   = targetDegrees * decelStart;
    uint32_t startTime    = pros::millis();
    uint32_t timeout      = (uint32_t)(std::abs(inches) * 100);

    while (true) {
        double leftPos  = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos   = (leftPos + rightPos) / 2.0;
        double heading  = imu.get_rotation();
        double correction = heading * kP;

        if (avgPos < decelPoint) {
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
        } else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < 10) currentSpeed = 10;
        }

        if (avgPos >= targetDegrees - 2 || pros::millis() - startTime > timeout)
            break;

        left_motor_group.move_velocity((currentSpeed - correction) * direction);
        right_motor_group.move_velocity((currentSpeed + correction) * direction);
        pros::delay(10);
    }

    // Smooth stop
    double lastSpeed = std::max(currentSpeed, 10.0);
    while (lastSpeed > 0) {
        double heading    = imu.get_rotation();
        double correction = heading * kP;
        left_motor_group.move_velocity((lastSpeed - correction) * direction);
        right_motor_group.move_velocity((lastSpeed + correction) * direction);
        lastSpeed -= 2;
        pros::delay(10);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

void turn_to_heading(double targetHeading, double maxSpeed) {
    const double kP        = 1.5;
    const double threshold = 1.0;

    uint32_t startTime = pros::millis();
    const uint32_t timeout = 3000;

    while (true) {
        double heading = imu.get_rotation();
        double error   = targetHeading - heading;

        while (error >  180) error -= 360;
        while (error < -180) error += 360;

        if (std::abs(error) <= threshold || pros::millis() - startTime > timeout)
            break;

        double speed = error * kP;
        if (speed >  maxSpeed)  speed =  maxSpeed;
        if (speed < -maxSpeed)  speed = -maxSpeed;
        if (speed >  0 && speed <  10) speed =  10;
        if (speed <  0 && speed > -10) speed = -10;

        left_motor_group.move_velocity(speed);
        right_motor_group.move_velocity(-speed);
        pros::delay(10);
    }

    // Smooth stop
    double lastSpeed = 15;
    double heading   = imu.get_rotation();
    double error     = targetHeading - heading;
    double stopDir   = (error > 0) ? 1.0 : -1.0;

    while (lastSpeed > 0) {
        left_motor_group.move_velocity(lastSpeed * stopDir);
        right_motor_group.move_velocity(-lastSpeed * stopDir);
        lastSpeed -= 2;
        pros::delay(10);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

// ─── Utility functions ────────────────────────────────────────────────────────
void wall_reset(int voltage, int settleTime) {
    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(voltage);

    int stalled = 0;
    while (stalled < settleTime) {
        double lv = std::abs(left_motor_group.get_actual_velocity());
        double rv = std::abs(right_motor_group.get_actual_velocity());
        if (lv < 3 && rv < 3) stalled += 10;
        else                   stalled  = 0;
        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
    pros::delay(50);
}

void wall_reset_v2(int voltage, int settleTime, int direction, int timeout) {
    voltage = std::abs(voltage) * direction;
    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(voltage);

    int stalled = 0, elapsed = 0;
    while (stalled < settleTime && elapsed < timeout) {
        double lv = std::abs(left_motor_group.get_actual_velocity());
        double rv = std::abs(right_motor_group.get_actual_velocity());
        if (lv < 3 && rv < 3) stalled += 10;
        else                   stalled  = 0;
        pros::delay(10);
        elapsed += 10;
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
    pros::delay(50);
}

void catapultShootForAuto(double SPEED) {
    const int L_POS = 0, F_POS = -600;
    const int ST = 250, CD = 10, MA = 10;

    for (int attempt = 0; attempt < MA; attempt++) {
        bool success = false;
        catapult_arm.move_absolute(F_POS, SPEED);
        int st = 0;

        while (true) {
            pros::delay(CD);
            double pos = catapult_arm.get_position();
            double vel = std::abs(catapult_arm.get_actual_velocity());
            if (pos <= F_POS + 25) { success = true; break; }
            if (vel < 5) st += CD; else st = 0;
            if (st >= ST) break;
        }

        catapult_arm.move_absolute(L_POS, SPEED);
        while (std::abs(catapult_arm.get_position() - L_POS) > 10)
            pros::delay(10);

        if (success) return;
    }
    catapult_arm.move_velocity(0);
}

// ─── Operator control ─────────────────────────────────────────────────────────
void catapultControl() {
    const int MAX_SPEED  = 127;
    const int SLOW_SPEED = 50;
    static bool controlsReversed = false;
    static bool wasDownHeld      = false;
    static bool armRaised        = false;
    static bool wasArmHeld       = false;

    while (true) {
        pros::delay(20);

        bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakePause   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool catapultBtn   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool discoreDown   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool discoreUp     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool matchLoadUp   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool armHeld       = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool armTapped     = armHeld && !wasArmHeld;
        wasArmHeld         = armHeld;
        bool downHeld  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool downTapped = downHeld && !wasDownHeld;
        wasDownHeld    = downHeld;

        int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn =  controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (downTapped) controlsReversed = !controlsReversed;
        if (controlsReversed) move = -move;

        int maxSpeed = downHeld ? SLOW_SPEED : MAX_SPEED;
        left_motor_group.move(std::clamp(move + turn, -maxSpeed, maxSpeed));
        right_motor_group.move(std::clamp(move - turn, -maxSpeed, maxSpeed));

        if (catapultBtn) startCatapultShoot();

        if (armTapped) {
            armRaised = !armRaised;
            arm.move_absolute(armRaised ? 900 : 0, 200);
        }

        if (discoreDown)      discore.move_absolute(0,   200);
        else if (discoreUp)   discore.move_absolute(800, 200);

        if (matchLoadUp && !matchLoadDown) {
            matchloader.move_absolute(0,    100);
            discore.move_absolute(800, 200);
        } else if (matchLoadDown && !matchLoadUp) {
            matchloader.move_absolute(1400, 100);
            discore.move_absolute(0,   200);
        }

        if (intakeForward && !intakeReverse) intake.move_velocity(600);
        else if (intakeReverse && !intakeForward) intake.move_velocity(-600);
        if (intakePause) intake.move_velocity(0);
    }
}