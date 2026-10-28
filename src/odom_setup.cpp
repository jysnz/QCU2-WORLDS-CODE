#include "odom_setup.hpp"
#include "driver_menu.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdio>

extern lemlib::TrackingWheel verticalTracking;
extern lemlib::TrackingWheel horizontalTracking;

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int GRAY = 0x777777;
const int FOOTER_BG = 0x18181F;
const double PI = 3.14159265358979;
const double KNOWN_INCHES = 48.0; // distance pushed during WHEEL CAL

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
const Rect kBtn[3] = {{10, 168, 155, 206}, {165, 168, 310, 206}, {320, 168, 468, 206}};
const int READOUT_Y = 58, READOUT_H = 108;

enum class Mode { IDLE, WHEEL_FWD, WHEEL_SIDE, SPIN };
Mode mode = Mode::IDLE;

// Tracker readouts are relative to the last ZERO.
float fwdZero = 0, sideZero = 0;
// Start values for the calibration in progress.
double rotStart = 0; // WHEEL CAL: rotation sensor, degrees
double rawStart = 0; // SPIN CAL: unscaled IMU rotation, degrees
double imuStart = 0, fwdStart = 0, sideStart = 0;
double fwdDiameter = 0; // result of the forward push, kept until the sideways one finishes
char status[64] = "tap a button";

float wrap360(float deg) {
  deg = std::fmod(deg, 360.0f);
  return deg < 0 ? deg + 360.0f : deg;
}

double rotationDegrees(pros::Rotation &rot) { return rot.get_position() / 100.0; } // centidegrees

void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

void drawButton(const Rect &r, const char *label, uint32_t accent) {
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, CARD);
  fillRoundedRect(r.x0, r.y0, r.x0 + 5, r.y1, 3, accent);
  pros::screen::set_eraser(CARD);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 14, r.y0 + 13, "%s", label);
}

void drawButtons() {
  drawButton(kBtn[0], "ZERO", CYAN);
  if (mode == Mode::IDLE) {
    drawButton(kBtn[1], "WHEEL CAL", ORANGE);
    drawButton(kBtn[2], "SPIN CAL", ORANGE);
  } else {
    drawButton(kBtn[1], "DONE", GREEN);
    drawButton(kBtn[2], "CANCEL", 0xFF3131);
  }
}

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "ODOM SETUP");
  pros::screen::set_pen(0x1a1a1a);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "trackers + pose");
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
  drawButtons();
}

void drawReadout() {
  lemlib::Pose pose = chassis.getPose();
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, READOUT_Y, 480, READOUT_Y + READOUT_H);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y, "X: %.5f", pose.x);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 17, "Y: %.5f", pose.y);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 34, "Heading: %.5f", wrap360(pose.theta));
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 51, "Rotation: %.5f", imu.get_rotation());
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 68, "Forward_Tracker: %.5f",
                      verticalTracking.getDistanceTraveled() - fwdZero);
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 85, "Sideways_Tracker: %.5f",
                      -(horizontalTracking.getDistanceTraveled() - sideZero));
  // Raw sensor angles (deg, after the port's sign): the first thing to look at when a tracker
  // reads the wrong way. LemLib counts rightward travel as negative on the horizontal wheel
  // (that is what makes X rise when pushed right), so H falls here while Sideways_Tracker
  // above is negated to read positive to the right.
  pros::screen::print(pros::E_TEXT_SMALL, 10, READOUT_Y + 102, "raw deg V %.1f  H %.1f",
                      rotationDegrees(verticalRotation), rotationDegrees(horizontalRotation));
}

void drawStatus() {
  pros::screen::set_pen(FOOTER_BG);
  pros::screen::fill_rect(0, 212, 480, 240);
  pros::screen::set_eraser(FOOTER_BG);
  pros::screen::set_pen(GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", status);
}

void setStatus(const char *fmt, double a = 0, double b = 0, double c = 0) {
  snprintf(status, sizeof(status), fmt, a, b, c);
  drawStatus();
}

void zeroAll() {
  chassis.setPose(0, 0, 0);
  fwdZero = verticalTracking.getDistanceTraveled();
  sideZero = horizontalTracking.getDistanceTraveled();
  printf("ODOM ZERO\n");
  setStatus("zeroed");
}

void setMode(Mode m) {
  mode = m;
  drawButtons();
}

// Effective diameter = distance pushed / how far the wheel turned, so it does not
// depend on the diameter currently in main.cpp. Returns -1 if the wheel barely moved.
double wheelDiameter(pros::Rotation &rot, const char *name) {
  double turns = std::abs(rotationDegrees(rot) - rotStart) / 360.0;
  if (turns < 0.05) {
    printf("ODOM %s: wheel barely moved (%.3f turns) - check the port / that it touches the floor\n",
           name, turns);
    return -1;
  }
  double d = KNOWN_INCHES / (PI * turns);
  printf("ODOM %s: %.1f in pushed, %.3f wheel turns -> effective diameter %.3f in\n", name,
         KNOWN_INCHES, turns, d);
  return d;
}

void startWheelCal() {
  fwdDiameter = 0;
  rotStart = rotationDegrees(verticalRotation);
  setMode(Mode::WHEEL_FWD);
  setStatus("push FWD 48in straight, tap DONE");
}

void finishWheelStep() {
  if (mode == Mode::WHEEL_FWD) {
    fwdDiameter = wheelDiameter(verticalRotation, "FORWARD");
    rotStart = rotationDegrees(horizontalRotation);
    setMode(Mode::WHEEL_SIDE);
    setStatus("push SIDEWAYS 48in, tap DONE");
  } else {
    double sideDiameter = wheelDiameter(horizontalRotation, "SIDEWAYS");
    printf("ODOM RESULT: VERTICAL_WHEEL_DIAMETER = %.3f, HORIZONTAL_WHEEL_DIAMETER = %.3f\n",
           fwdDiameter, sideDiameter);
    setMode(Mode::IDLE);
    setStatus("diam V %.3f  H %.3f -> main.cpp", fwdDiameter, sideDiameter);
  }
}

void startSpinCal() {
  imuStart = imu.get_rotation();
  rawStart = imu.rawRotation();
  fwdStart = verticalTracking.getDistanceTraveled();
  sideStart = horizontalTracking.getDistanceTraveled();
  setMode(Mode::SPIN);
  setStatus("spin whole turns (3+), end facing start");
}

// A tracker `d` inches from the turning center sweeps an arc of d * theta (radians)
// as the robot turns, so d = travel / theta. LemLib offsets are signed: the vertical
// wheel is negative left of center (a wheel on the right moves backward in a
// clockwise turn, hence the minus), the horizontal wheel is negative behind center (a
// wheel in front moves right in a clockwise turn, which LemLib reads as negative).
void finishSpinCal() {
  // Whole turns made, from the raw IMU. The scale is what the IMU reads per true 360.
  double rawTurned = imu.rawRotation() - rawStart;
  double turns = std::round(std::abs(rawTurned) / 360.0);
  double theta = (imu.get_rotation() - imuStart) * PI / 180.0;
  double fwdTravel = verticalTracking.getDistanceTraveled() - fwdStart;
  double sideTravel = horizontalTracking.getDistanceTraveled() - sideStart;
  if (std::abs(theta) < PI) { // under half a turn is too short to measure an offset
    printf("ODOM SPIN: only %.1f deg turned - spin at least 3 full turns\n", theta * 180.0 / PI);
    setMode(Mode::IDLE);
    setStatus("spin too short (%.0f deg), try again", theta * 180.0 / PI);
    return;
  }
  double vOffset = -fwdTravel / theta;
  double hOffset = -sideTravel / theta; // LemLib's horizontal travel is negative to the right
  printf("ODOM SPIN: %.1f deg turned | fwd tracker %.3f in | side tracker %.3f in\n",
         theta * 180.0 / PI, fwdTravel, sideTravel);
  printf("ODOM RESULT: vertical offset = %.3f, horizontal offset = %.3f\n", vOffset, hOffset);
  // Only meaningful if the spin was a whole number of turns ending where it started.
  double scale = rawTurned / turns;
  printf("ODOM RESULT: INERTIAL_SCALE = %.2f (%.0f turns, IMU read %.2f deg raw)\n", scale, turns,
         rawTurned);
  setMode(Mode::IDLE);
  setStatus("off V %.2f H %.2f  scale %.2f", vOffset, hOffset, scale);
}

} // namespace

void odomSetupControl() {
  mode = Mode::IDLE;
  snprintf(status, sizeof(status), "tap a button");
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("ODOM SETUP ACTIVE\n");
  drawFrame();
  drawStatus();

  bool wasTouched = false;
  while (true) {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = touch.x, y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    // A tap from the laptop mirror counts like a real one (see driver_menu.cpp).
    if (!press && takeRemoteTouch(x, y))
      press = true;

    if (press) {
      if (inRect(kBack, x, y)) {
        controller.rumble(".");
        printf("ODOM SETUP EXIT\n");
        return; // back to the driver menu
      }
      if (inRect(kBtn[0], x, y)) {
        zeroAll();
      } else if (mode == Mode::IDLE) {
        if (inRect(kBtn[1], x, y))
          startWheelCal();
        else if (inRect(kBtn[2], x, y))
          startSpinCal();
      } else if (inRect(kBtn[1], x, y)) { // DONE
        if (mode == Mode::SPIN)
          finishSpinCal();
        else
          finishWheelStep();
      } else if (inRect(kBtn[2], x, y)) { // CANCEL
        setMode(Mode::IDLE);
        setStatus("cancelled");
      }
    }

    drawReadout();
    pros::delay(50);
  }
}
