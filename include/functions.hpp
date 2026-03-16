#pragma once

// ─── Catapult ─────────────────────────────────────────────────────────────────
void catapultTask(void *param);
void startCatapultShoot();
void catapultShootForAuto(double speed);

// ─── Drive ───────────────────────────────────────────────────────────────────
void drive_for_inches(double maxSpeed, double inches);
void turn_to_heading(double targetHeading, double maxSpeed);

// ─── Utility ─────────────────────────────────────────────────────────────────
void wall_reset(int voltage = 8000, int settleTime = 200);
void wall_reset_v2(int voltage = 8000, int settleTime = 200,
                   int direction = 1,  int timeout   = 1000);

// ─── Operator control ─────────────────────────────────────────────────────────
void catapultControl();