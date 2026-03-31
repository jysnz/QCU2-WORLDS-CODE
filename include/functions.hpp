#pragma once

// ─── Catapult
// ─────────────────────────────────────────────────────────────────
void catapultTask(void *param);
void startCatapultShoot();
void catapultShootForAuto(double speed);

// ─── Intake
// ───────────────────────────────────────────────────────────────────
void intakeTask(void *param);

// Robot functions
void delay(int delay);
void drivetrainLock();
void gateOpen();
void gateClose();
void descoreUp();
void descoreDown();
void descoreDownMiddle();
void matchloadUp();
void matchloadDown();
void midGoalArm();
void underGoalArm();
void intakeBlock();
void outtakeBlock(double speed = 600);
void intakeStop();
void reset();
void leverReset();

// ─── Drive ───────────────────────────────────────────────────────────────────
void drive_for_inches(double maxSpeed, double inches);
void turn_to_heading(double targetHeading, double maxSpeed);

// ─── Utility ─────────────────────────────────────────────────────────────────
void wall_reset(int voltage = 8000, int settleTime = 200);
void wall_reset_v2(int voltage = 8000, int settleTime = 200, int direction = 1,
                   int timeout = 1000);

void intake_auto(int delay, int speed, int direction);

// ─── Persistence ─────────────────────────────────────────────────────────────
void persistenceTask(void *param);
void restoreMotorPositions();

// ─── Operator control
// ─────────────────────────────────────────────────────────
void catapultControl();