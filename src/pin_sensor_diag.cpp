#include "pin_sensor_diag.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "lift_calibration.hpp"
#include "motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cstdint>
#include <cstdio>

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int RED = 0xFF3131;
const int GRAY = 0x777777;
const int FOOTER_BG = 0x18181F;

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
const Rect kFlip = {10, 178, 120, 208};
const Rect kAuto = {125, 178, 235, 208};
const Rect kOpen = {240, 178, 350, 208};
const Rect kHome = {355, 178, 470, 208};

void fillRoundedRect(int x0, int y0, int x1, int y1, int r, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(x0 + r, y0, x1 - r, y1);
  pros::screen::fill_rect(x0, y0 + r, x1, y1 - r);
  pros::screen::fill_circle(x0 + r, y0 + r, r);
  pros::screen::fill_circle(x1 - r, y0 + r, r);
  pros::screen::fill_circle(x0 + r, y1 - r, r);
  pros::screen::fill_circle(x1 - r, y1 - r, r);
}

void button(const Rect &r, uint32_t color, const char *label) {
  fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, color);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 10, "%s", label);
}

void drawFrame() {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "IR SENSOR");
  pros::screen::set_pen(0x1A1A1A);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "pin sensor on ADI port D");
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
}

} // namespace

void pinSensorDiagControl() {
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("IR SENSOR TEST ACTIVE\n");

  const bool autoWasOn = pinAutoGrabEnabled();
  pinAutoGrabEnable(false); // off until the AUTO-GRAB button turns it on
  bool autoOn = false;

  int32_t lastRaw = -12345;
  uint32_t changes = 0;
  uint32_t pinSeenMs = 0; // last time a pin was seen, so a brief blip stays visible
  bool wasTouched = false;
  uint32_t lastPrintMs = 0;

  drawFrame();
  while (true) {
    const uint32_t nowMs = pros::millis();
    const int32_t raw = pinSensor.get_value();
    const bool readable = raw == 0 || raw == 1; // anything else is a PROS error code
    if (raw != lastRaw) {
      if (lastRaw != -12345)
        changes++;
      lastRaw = raw;
    }
    const bool seen = pinDetected();
    if (seen)
      pinSeenMs = nowMs;
    const bool atStart = liftAtStart();
    const bool wouldGrab = seen && atStart;

    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = touch.x, y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    if (!press && takeRemoteTouch(x, y))
      press = true;

    if (press) {
      if (inRect(kBack, x, y)) {
        pinAutoGrabEnable(autoWasOn);
        controller.rumble(".");
        printf("IR SENSOR TEST EXIT\n");
        return; // back to the driver menu
      }
      if (inRect(kFlip, x, y)) {
        pinSensorActiveHigh = !pinSensorActiveHigh;
      } else if (inRect(kAuto, x, y)) {
        autoOn = !autoOn;
        pinAutoGrabEnable(autoOn);
      } else if (inRect(kOpen, x, y)) {
        liftOpenClaws();
      } else if (inRect(kHome, x, y)) {
        homeLift();
      }
    }

    // status pill
    const uint32_t color = !readable ? RED : seen ? GREEN : GRAY;
    fillRoundedRect(10, 58, 300, 88, 8, color);
    pros::screen::set_eraser(color);
    pros::screen::set_pen(0x000000);
    pros::screen::print(pros::E_TEXT_MEDIUM, 22, 62, "%s",
                        !readable ? "NO SENSOR" : seen ? "PIN DETECTED" : "NOTHING");

    // raw value + polarity
    pros::screen::set_pen(BG);
    pros::screen::fill_rect(10, 94, 470, 172);
    pros::screen::set_eraser(BG);
    pros::screen::set_pen(readable ? 0xFFFFFF : RED);
    if (readable)
      pros::screen::print(pros::E_TEXT_LARGE, 10, 94, "raw %d", (int)raw);
    else
      pros::screen::print(pros::E_TEXT_LARGE, 10, 94, "raw ERR");
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 250, 98, "pin = raw %d  (flip if reversed)",
                        pinSensorActiveHigh ? 1 : 0);
    pros::screen::print(pros::E_TEXT_SMALL, 250, 114, "changes seen: %u", (unsigned)changes);
    pros::screen::set_pen(atStart ? GREEN : ORANGE);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 138, "lift %.1f deg  limit switch %d  at start: %s",
                        lift.get_position(), (int)liftLimitSwitch.get_value(), atStart ? "YES" : "NO");
    pros::screen::set_pen(wouldGrab ? GREEN : GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 154, "auto-grab would fire now: %s%s",
                        wouldGrab ? "YES" : "NO",
                        nowMs - pinSeenMs < 1000 && !seen ? "   (pin seen a moment ago)" : "");

    button(kFlip, CARD, "FLIP POLARITY");
    button(kAuto, autoOn ? 0x1E7A45 : CARD, autoOn ? "AUTO-GRAB: ON" : "AUTO-GRAB: OFF");
    button(kOpen, CARD, "OPEN CLAWS");
    button(kHome, CARD, "HOME LIFT");

    // hint
    pros::screen::set_pen(FOOTER_BG);
    pros::screen::fill_rect(0, 212, 480, 240);
    pros::screen::set_eraser(FOOTER_BG);
    pros::screen::set_pen(!readable ? RED : GRAY);
    const char *hint = !readable ? "port D cannot be read: check the cable is in ADI D"
                       : changes == 0 ? "wave a pin in front: raw should change"
                       : !atStart ? "lower the lift to the limit switch (HOME LIFT) for auto-grab"
                                  : "sensor works: turn AUTO-GRAB ON and test";
    pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", hint);

    if (nowMs - lastPrintMs >= 500) { // also on `pros terminal`
      lastPrintMs = nowMs;
      printf("IR raw %d seen %d lift %.1f atStart %d\n", (int)raw, (int)seen, lift.get_position(),
             (int)atStart);
    }
    pros::delay(30);
  }
}
