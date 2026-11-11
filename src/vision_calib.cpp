// ─────────────────────────────────────────────────────────────────────────────
// VISION CALIBRATION — see include/vision_calib.hpp for the overview.
// HOME -> VISION: a full-screen diagnostic for the V5 Vision Sensor.
//   RED / YEL / BLUE  which color to track: signature slot 1 = RED,
//                     2 = YELLOW, 3 = BLUE (captured with the Vision Utility)
//   SIG -/+       any of the sensor's 7 signature slots (controller too)
//   EXP -/+       camera exposure (0..150, like the Vision Utility)
//   FOLLOW        robot pivots to keep the largest object of that color
//                 centered; move the element by hand and it follows
//   Controller:   A toggle follow, X stop follow, UP/DOWN exposure,
//                 LEFT/RIGHT signature slot
// The info panel shows a live object count for each of the three colors at
// once, so you can wave elements past the camera and see what's detected.
// Signatures come from the VEX Vision Utility on a PC (they persist on the
// sensor); the screen reports EMPTY for a slot the utility hasn't filled.
// Mirrored to the laptop mirror (tools/remote_touch.py) as RUI|VISION lines
// with a VIS: segment; the mirror's clicks/keys come back as TOUCH/KEY.
// ─────────────────────────────────────────────────────────────────────────────

#include "vision_calib.hpp"
#include "driver_menu.hpp"
#include "motors.hpp"
#include "vision.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

const int BG = 0x0A0A0F;
const int CARD = 0x161B22;
const int CYAN = 0x00F0FF;
const int ORANGE = 0xFF8C00;
const int GREEN = 0x2ECC71;
const int RED = 0xFF3131;
const int YELLOW = 0xF1C40F;
const int BLUE = 0x4A90F2;
const int GRAY = 0x777777;
const int AXIS = 0x2d2d38;
const int FOOTER_BG = 0x18181F;

struct Rect {
  int x0, y0, x1, y1;
};
bool inRect(const Rect &r, int x, int y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

const Rect kBack = {396, 14, 468, 40};
// Small row (SIG / EXP) above the color row -- clear of the camera panel,
// which now ends at y=150 to make room.
const Rect kSigMinus = {10, 156, 96, 176};
const Rect kSigPlus = {101, 156, 187, 176};
const Rect kExpMinus = {192, 156, 278, 176};
const Rect kExpPlus = {283, 156, 369, 176};
// Color row: YEL/RED/BLUE pick signature slots 1/2/3 (slot 1 already holds
// the team's yellow capture), FOLLOW pivots the robot to keep the selected
// color centered.
const Rect kRed = {10, 182, 100, 212};
const Rect kYel = {105, 182, 195, 212};
const Rect kBlue = {200, 182, 290, 212};
const Rect kFollow = {295, 182, 470, 212};

// The camera-view panel: the sensor's 316x212 FOV squeezed into this box,
// center crosshair in the middle, the detected object drawn as a box.
const Rect kCam = {270, 58, 460, 150};

const char *slotName(int slot) {
  switch (slot) {
  case 1: return "YEL";
  case 2: return "RED";
  case 3: return "BLUE";
  default: return "-";
  }
}

// The team's YELLOW signature, exported from the VEX Vision Utility
// (brightness 50). Re-installed onto the sensor at page entry only if slot
// 1 is empty -- a fresh capture on the sensor always wins.
void installDefaultYellow() {
  const pros::vision_signature_s_t stored = vision_sensor.get_signature(1);
  if (!(stored.u_min == 0 && stored.u_max == 0 && stored.v_min == 0 && stored.v_max == 0))
    return;
  pros::vision_signature_s_t yel =
      pros::Vision::signature_from_utility(1, -401, 187, -107, -6823, -6139, -6481, 2.5, 0);
  vision_sensor.set_signature(1, &yel);
}

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
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + (r.y1 - r.y0) / 2 - 6, "%s", label);
}

void drawFrame(const char *breadcrumb) {
  pros::screen::set_pen(BG);
  pros::screen::fill_rect(0, 0, 480, 240);
  fillRoundedRect(5, 5, 475, 50, 8, CYAN);
  pros::screen::set_eraser(CYAN);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 12, "VISION");
  pros::screen::set_pen(0x1A1A1A);
  pros::screen::print(pros::E_TEXT_SMALL, 20, 30, "%s", breadcrumb);
  fillRoundedRect(kBack.x0, kBack.y0, kBack.x1, kBack.y1, 6, BG);
  pros::screen::set_eraser(BG);
  pros::screen::set_pen(0xCCCCCC);
  pros::screen::print(pros::E_TEXT_SMALL, kBack.x0 + 12, kBack.y0 + 8, "< BACK");
}

void stopDrive() {
  left_motor_group.move(0);
  right_motor_group.move(0);
}

// Number of objects matching one signature in the last snapshot: the
// wrapper has no per-signature count call, so walk get_by_sig until it
// stops returning that signature.
int countForSig(uint16_t sigId) {
  int n = 0;
  for (int i = 0; i < 16; ++i) {
    const pros::vision_object_s_t o = vision_sensor.get_by_sig(i, sigId);
    if (o.signature != sigId)
      break;
    ++n;
  }
  return n;
}

// A controller button edge OR'd with the laptop mirror's KEY presses --
// the two are indistinguishable, same as everywhere else in the menu.
struct EdgeKey {
  pros::controller_digital_e_t id;
  RemoteKey rk;
  bool prevHeld = false;
  bool pressed() {
    bool now = controller.get_digital(id);
    bool edge = now && !prevHeld;
    prevHeld = now;
    return edge || takeRemoteKey(rk);
  }
};

} // namespace

void visionCalibControl() {
  pros::delay(100); // let the HUD task finish its current frame
  controller.rumble("--");
  printf("VISION CAL ACTIVE\n");

  EdgeKey keyA{pros::E_CONTROLLER_DIGITAL_A, RK_A};
  EdgeKey keyX{pros::E_CONTROLLER_DIGITAL_X, RK_X};
  EdgeKey keyUp{pros::E_CONTROLLER_DIGITAL_UP, RK_UP};
  EdgeKey keyDown{pros::E_CONTROLLER_DIGITAL_DOWN, RK_DOWN};
  EdgeKey keyLeft{pros::E_CONTROLLER_DIGITAL_LEFT, RK_LEFT};
  EdgeKey keyRight{pros::E_CONTROLLER_DIGITAL_RIGHT, RK_RIGHT};

  bool wasTouched = false;
  bool followOn = false;
  int sigSlot = 1;
  int exposure = vision_sensor.get_exposure(); // PROS_ERR if the sensor is absent
  if (exposure != PROS_ERR)
    installDefaultYellow();
  uint32_t lastSeenMs = 0;
  uint32_t lastPrintMs = 0;

  // The laptop mirror's state line, paced like every other screen's.
  auto sendState = [&](bool force = false) {
    static uint32_t lastSend = 0, lastActual = 0;
    static char lastLine[600] = "";
    if (!force && pros::millis() - lastSend < (uint32_t)remoteSendIntervalMs())
      return;
    lastSend = pros::millis();

    const int cnt = vision_sensor.get_object_count();
    const bool sensorOk = cnt != PROS_ERR;
    const pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, sigSlot);
    const bool seen = sensorOk && obj.signature == (uint16_t)sigSlot && obj.width >= 4;
    const pros::vision_signature_s_t sig = vision_sensor.get_signature(sigSlot);
    const bool sigEmpty =
        sig.u_min == 0 && sig.u_max == 0 && sig.v_min == 0 && sig.v_max == 0;
    const int cntRed = sensorOk ? countForSig(2) : 0;
    const int cntYel = sensorOk ? countForSig(1) : 0;
    const int cntBlue = sensorOk ? countForSig(3) : 0;
    const char *hint = !sensorOk ? "plug the vision sensor into port 20"
                      : sigEmpty ? "slot is empty -- capture the signature with the VEX Vision Utility"
                      : followOn
                          ? "move the element by hand -- the robot keeps it centered (X stops)"
                          : "tap a color, press A to FOLLOW once its box tracks the element";

    char line[600];
    snprintf(line, sizeof(line),
             "RUI|VISION|VISION|BTN:%d,%d,%d,%d,BACK;%d,%d,%d,%d,SIG -;%d,%d,%d,%d,SIG +;"
             "%d,%d,%d,%d,EXP -;%d,%d,%d,%d,EXP +;%d,%d,%d,%d,RED;%d,%d,%d,%d,YEL;"
             "%d,%d,%d,%d,BLUE;%d,%d,%d,%d,FOLLOW"
             "|FIELDS:|FOOTER:%s|CODE:|MOTORS:|SCROLL:|STEP:0|FOOTC:0|PLOT:|TRAIL:|SEL:0"
             "|ACTIVE:-1,-1|PLAN:|PATH:|VIS:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%lu,%d,%d,%d,%d,%d,%d,%d",
             kBack.x0, kBack.y0, kBack.x1, kBack.y1, kSigMinus.x0, kSigMinus.y0, kSigMinus.x1,
             kSigMinus.y1, kSigPlus.x0, kSigPlus.y0, kSigPlus.x1, kSigPlus.y1, kExpMinus.x0,
             kExpMinus.y0, kExpMinus.x1, kExpMinus.y1, kExpPlus.x0, kExpPlus.y0, kExpPlus.x1,
             kExpPlus.y1, kRed.x0, kRed.y0, kRed.x1, kRed.y1, kYel.x0, kYel.y0, kYel.x1, kYel.y1,
             kBlue.x0, kBlue.y0, kBlue.x1, kBlue.y1, kFollow.x0, kFollow.y0, kFollow.x1, kFollow.y1,
             hint, sensorOk ? 1 : 0, followOn ? 1 : 0, sigSlot, sensorOk ? exposure : 0,
             sensorOk ? cnt : 0, seen ? 1 : 0, seen ? (int)obj.x_middle_coord : 0, sigEmpty ? 1 : 0,
             (int)sig.u_min, (int)sig.u_max, (int)sig.u_mean, (int)sig.v_min, (int)sig.v_max,
             (int)sig.v_mean, (double)sig.range, (unsigned long)sig.rgb, seen ? (int)obj.left_coord : 0,
             seen ? (int)obj.top_coord : 0, seen ? (int)obj.width : 0, seen ? (int)obj.height : 0,
             cntRed, cntYel, cntBlue);
    if (!force && strcmp(line, lastLine) == 0 && pros::millis() - lastActual < 1000)
      return;
    strcpy(lastLine, line);
    lastActual = pros::millis();
    printf("%s\n", line);
  };

  drawFrame("vision sensor on port 20");
  sendState(true);

  while (true) {
    const uint32_t nowMs = pros::millis();

    const int cnt = vision_sensor.get_object_count();
    const bool sensorOk = cnt != PROS_ERR;
    if (sensorOk)
      exposure = vision_sensor.get_exposure();

    pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, sigSlot);
    const bool seen = sensorOk && obj.signature == (uint16_t)sigSlot && obj.width >= 4;
    if (seen)
      lastSeenMs = nowMs;
    const int err = seen ? obj.x_middle_coord : 0; // zero point is the FOV center

    const pros::vision_signature_s_t sig = vision_sensor.get_signature(sigSlot);
    const bool sigEmpty =
        sig.u_min == 0 && sig.u_max == 0 && sig.v_min == 0 && sig.v_max == 0;
    // live object count of each color slot, all at once
    const int cntRed = sensorOk ? countForSig(2) : 0;
    const int cntYel = sensorOk ? countForSig(1) : 0;
    const int cntBlue = sensorOk ? countForSig(3) : 0;

    // touch
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool press = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    int x = touch.x, y = touch.y;
    if (touch.touch_status == pros::E_TOUCH_PRESSED)
      wasTouched = true;
    else if (touch.touch_status == pros::E_TOUCH_RELEASED)
      wasTouched = false;
    if (!press && takeRemoteTouch(x, y))
      press = true;

    const int expNew = std::min(150, std::max(0, exposure + 5));

    if (keyA.pressed() && sensorOk) {
      followOn = !followOn;
      controller.rumble(".");
    }
    if (keyX.pressed() && followOn) {
      followOn = false;
      stopDrive();
      controller.rumble("-");
    }
    if (keyUp.pressed() && sensorOk)
      vision_sensor.set_exposure(expNew);
    if (keyDown.pressed() && sensorOk)
      vision_sensor.set_exposure(std::min(150, std::max(0, exposure - 5)));
    if (keyRight.pressed())
      sigSlot = sigSlot % 7 + 1;
    if (keyLeft.pressed())
      sigSlot = (sigSlot + 5) % 7 + 1;

    if (press) {
      if (inRect(kBack, x, y)) {
        stopDrive();
        controller.rumble(".");
        printf("VISION CAL EXIT\n");
        return; // back to the driver menu
      }
      if (inRect(kSigMinus, x, y))
        sigSlot = (sigSlot + 5) % 7 + 1;
      else if (inRect(kSigPlus, x, y))
        sigSlot = sigSlot % 7 + 1;
      else if (inRect(kRed, x, y))
        sigSlot = 2;
      else if (inRect(kYel, x, y))
        sigSlot = 1;
      else if (inRect(kBlue, x, y))
        sigSlot = 3;
      else if (inRect(kExpMinus, x, y) && sensorOk)
        vision_sensor.set_exposure(std::min(150, std::max(0, exposure - 5)));
      else if (inRect(kExpPlus, x, y) && sensorOk)
        vision_sensor.set_exposure(expNew);
      else if (inRect(kFollow, x, y) && sensorOk)
        followOn = !followOn;
    }

    // follow: pivot so the object drifts back to the FOV center
    if (followOn) {
      if (seen && nowMs - lastSeenMs < 300 && std::abs(err) > 12) {
        const double vel = std::clamp(err * 0.9, -60.0, 60.0);
        left_motor_group.move(vel);   // object right of center: +vel means
        right_motor_group.move(-vel); // turn clockwise toward it
      } else {
        stopDrive();
      }
    }

    // ─── draw ───
    drawFrame(sensorOk ? "vision sensor on port 20" : "vision sensor NOT found on port 20");

    const char *status = !sensorOk ? "NO SENSOR" : followOn ? "FOLLOWING" : seen ? "SEEN" : "SEARCHING";
    const uint32_t statusColor = !sensorOk ? RED : followOn ? GREEN : seen ? CYAN : GRAY;
    fillRoundedRect(10, 58, 255, 84, 8, statusColor);
    pros::screen::set_eraser(statusColor);
    pros::screen::set_pen(0x000000);
    pros::screen::print(pros::E_TEXT_MEDIUM, 22, 62, "%s", status);
    pros::screen::print(pros::E_TEXT_SMALL, 22, 76, seen ? "err %d px" : "", err);

    pros::screen::set_pen(BG);
    pros::screen::fill_rect(10, 90, 255, 152);
    pros::screen::set_eraser(BG);
    pros::screen::set_pen(sensorOk ? 0xFFFFFF : RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 94, "objects total: %d", sensorOk ? cnt : 0);
    // per-color detection, all three at once (slot 1 = YEL, 2 = RED, 3 = BLUE)
    pros::screen::set_pen(sensorOk ? YELLOW : RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 106, "YEL %d", cntYel);
    pros::screen::set_pen(sensorOk ? 0xFF6464 : RED);
    pros::screen::print(pros::E_TEXT_SMALL, 80, 106, "RED %d", cntRed);
    pros::screen::set_pen(sensorOk ? BLUE : RED);
    pros::screen::print(pros::E_TEXT_SMALL, 150, 106, "BLUE %d", cntBlue);
    pros::screen::set_pen(sensorOk ? 0xFFFFFF : RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 118, "exposure %d%s", sensorOk ? exposure : 0,
                        sensorOk ? "" : " (ERR)");
    pros::screen::print(pros::E_TEXT_SMALL, 10, 130, "tracking slot %d (%s)", sigSlot,
                        slotName(sigSlot));
    if (sigEmpty) {
      pros::screen::set_pen(ORANGE);
      pros::screen::print(pros::E_TEXT_SMALL, 10, 142, "slot %d empty - Vision Utility", sigSlot);
    } else if (seen) {
      pros::screen::set_pen(CYAN);
      pros::screen::print(pros::E_TEXT_SMALL, 10, 142, "obj x %d y %d w %d h %d",
                          (int)obj.x_middle_coord, (int)obj.y_middle_coord, (int)obj.width,
                          (int)obj.height);
    } else {
      pros::screen::set_pen(CYAN);
      pros::screen::print(pros::E_TEXT_SMALL, 10, 142, "u %d..%d v %d..%d", (int)sig.u_min,
                          (int)sig.u_max, (int)sig.v_min, (int)sig.v_max);
    }

    // camera view
    pros::screen::set_pen(CARD);
    pros::screen::fill_rect(kCam.x0, kCam.y0, kCam.x1, kCam.y1);
    pros::screen::set_pen(AXIS);
    pros::screen::draw_rect(kCam.x0, kCam.y0, kCam.x1, kCam.y1);
    const int camCx = (kCam.x0 + kCam.x1) / 2, camCy = (kCam.y0 + kCam.y1) / 2;
    const double sx = (kCam.x1 - kCam.x0) / 316.0, sy = (kCam.y1 - kCam.y0) / 212.0;
    pros::screen::set_pen(AXIS);
    pros::screen::draw_line(camCx, kCam.y0, camCx, kCam.y1);
    pros::screen::draw_line(kCam.x0, camCy, kCam.x1, camCy);
    pros::screen::set_pen(GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, kCam.x0 + 4, kCam.y0 + 2, "FOV");
    if (seen) {
      const int bx0 = camCx + (int)(obj.left_coord * sx);
      const int by0 = camCy + (int)(obj.top_coord * sy);
      const int bx1 = bx0 + (int)(obj.width * sx);
      const int by1 = by0 + (int)(obj.height * sy);
      pros::screen::set_pen(std::abs(err) <= 12 ? GREEN : ORANGE);
      pros::screen::draw_rect(bx0, by0, bx1, by1);
    }

    button(kSigMinus, CARD, "SIG -");
    button(kSigPlus, CARD, "SIG +");
    button(kExpMinus, CARD, "EXP -");
    button(kExpPlus, CARD, "EXP +");
    // color buttons: the selected one lights up in its own color
    auto colorBtn = [&](const Rect &r, int slot, uint32_t col, const char *label) {
      const bool sel = sigSlot == slot;
      fillRoundedRect(r.x0, r.y0, r.x1, r.y1, 8, sel ? col : CARD);
      pros::screen::set_eraser(sel ? col : CARD);
      pros::screen::set_pen(sel && col == YELLOW ? 0x000000 : 0xFFFFFF);
      pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 10, r.y0 + 9, "%s", label);
    };
    colorBtn(kRed, 2, RED, "RED");
    colorBtn(kYel, 1, YELLOW, "YEL");
    colorBtn(kBlue, 3, BLUE, "BLUE");
    button(kFollow, followOn ? 0x1E7A45 : CARD, followOn ? "FOLLOW: ON" : "FOLLOW: OFF");

    pros::screen::set_pen(FOOTER_BG);
    pros::screen::fill_rect(0, 215, 480, 240);
    pros::screen::set_eraser(FOOTER_BG);
    pros::screen::set_pen(!sensorOk ? RED : GRAY);
    const char *hint;
    if (!sensorOk)
      hint = "plug the vision sensor into port 20";
    else if (sigEmpty)
      hint = "slot is empty -- capture the signature with the VEX Vision Utility (PC)";
    else if (followOn)
      hint = "move the element by hand -- the robot keeps it centered (X stops)";
    else
      hint = "tap YEL/RED/BLUE, then A (or FOLLOW) to chase that color";
    pros::screen::print(pros::E_TEXT_SMALL, 10, 222, "%s", hint);

    if (nowMs - lastPrintMs >= 500) { // also on `pros terminal`
      lastPrintMs = nowMs;
      printf("VISION sig %d (%s) yel %d red %d blue %d seen %d err %d exp %d follow %d\n", sigSlot,
             slotName(sigSlot), cntYel, cntRed, cntBlue, (int)seen, err, sensorOk ? exposure : 0,
             (int)followOn);
    }
    sendState();
    pros::delay(20);
  }
}
