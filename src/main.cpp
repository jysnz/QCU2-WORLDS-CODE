#include "main.h"
#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/api.hpp"
#include "motors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Motor & sensor definitions
// ─────────────────────────────────────────────────────────────────────────────
pros::MotorGroup left_motor_group({-11, -12, -13, -14},
                                  pros::MotorGears::green);
pros::MotorGroup right_motor_group({1, 2, 10, 4}, pros::MotorGears::green);
pros::MotorGroup intake({16, -18}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor matchloader(5, pros::MotorGears::green);
pros::Motor descore(15, pros::MotorGears::green);
pros::Motor arm(6, pros::MotorGears::red);
pros::Motor gate(17, pros::MotorGears::green);

pros::Imu imu(9);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ─── LemLib Setup ───
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 15,
                              lemlib::Omniwheel::NEW_325, 458, 2);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(10, 0, 28, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(1.5, 0, 10, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

// ─── UI & Auton Selector State ───
enum UITab { TAB_TEMPS, TAB_AUTON };
UITab currentTab = TAB_TEMPS;

int currentAutonIndex = 0;
const std::vector<std::string> autonNames = {
    "2v2 Left Red",     "2v2 Left Blue", "2v2 Right Red",    "2v2 Right Blue",
    "Skills Challenge", "SkillsV1",      "SoloAWP_Red_Right"};

void saveAutonSelection() {
  FILE *usd_file = fopen("/usd/auton_selection.txt", "w");
  if (usd_file) {
    fprintf(usd_file, "%d", currentAutonIndex);
    fclose(usd_file);
  }
}

void loadAutonSelection() {
  FILE *usd_file = fopen("/usd/auton_selection.txt", "r");
  if (usd_file) {
    fscanf(usd_file, "%d", &currentAutonIndex);
    fclose(usd_file);
    if (currentAutonIndex >= (int)autonNames.size())
      currentAutonIndex = 0;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Controller & Brain Auton Selector
// ─────────────────────────────────────────────────────────────────────────────
void selectAutonFromController() {
  bool selecting = true;
  bool lastUpPressed = false;
  bool lastDownPressed = false;
  int autonScroll = 0;

  const int BG_DARK = 0x0A0A0F;
  const int CARD_BG = 0x161B22;
  const int ACCENT_CYAN = 0x00F0FF;
  const int ACCENT_ORANGE = 0xFF8C00;
  const int ACCENT_DARK_GRAY = 0x22222B;

  controller.clear();

  while (selecting) {
    bool upPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool downPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool aPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    // Controller Navigation
    if (upPressed && !lastUpPressed) {
      currentAutonIndex = (currentAutonIndex + 1) % autonNames.size();
    }
    if (downPressed && !lastDownPressed) {
      currentAutonIndex =
          (currentAutonIndex - 1 + autonNames.size()) % autonNames.size();
    }

    lastUpPressed = upPressed;
    lastDownPressed = downPressed;

    // Confirm from controller
    if (aPressed) {
      saveAutonSelection();
      selecting = false;
    }

    // Brain Screen Display with Touch Input
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    static bool wasTouched = false;

    if (status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
      // Scroll UP
      if (status.x > 430 && status.y > 60 && status.y < 130) {
        autonScroll += 55;
      }
      // Scroll DOWN
      else if (status.x > 430 && status.y > 140 && status.y < 210) {
        autonScroll -= 55;
      }
      // Select Auton by Touch
      else if (status.x < 420 && status.y > 50 && status.y < 215) {
        for (int i = 0; i < (int)autonNames.size(); i++) {
          int itemY = 60 + (i * 55) + autonScroll;
          if (status.y >= itemY && status.y <= itemY + 50) {
            currentAutonIndex = i;
            saveAutonSelection();
            selecting = false;
            break;
          }
        }
      }
      wasTouched = true;
    } else if (status.touch_status == pros::E_TOUCH_RELEASED) {
      wasTouched = false;
    }

    // Clamp Scrolling
    if (autonScroll > 0)
      autonScroll = 0;
    int maxAutonScroll = -((int)autonNames.size() * 55 - 140);
    if (autonScroll < maxAutonScroll && maxAutonScroll < 0)
      autonScroll = maxAutonScroll;

    // ── Render Brain Screen ──
    pros::screen::set_pen(BG_DARK);
    pros::screen::fill_rect(0, 0, 480, 240);

    // Subtle Grid
    pros::screen::set_pen(0x121217);
    for (int i = 0; i < 480; i += 40)
      pros::screen::draw_line(i, 0, i, 240);
    for (int i = 0; i < 240; i += 40)
      pros::screen::draw_line(0, i, 480, i);

    // Auton List with Selection
    for (int i = 0; i < (int)autonNames.size(); i++) {
      int itemY = 60 + (i * 55) + autonScroll;

      if (itemY + 50 < 55 || itemY > 215)
        continue;

      bool isSelected = (i == currentAutonIndex);
      pros::screen::set_pen(isSelected ? 0x1c2838 : CARD_BG);
      pros::screen::fill_rect(20, itemY, 420, itemY + 50);

      pros::screen::set_pen(isSelected ? ACCENT_ORANGE : 0x2d2d38);
      pros::screen::draw_rect(20, itemY, 420, itemY + 50);

      if (isSelected) {
        pros::screen::set_pen(ACCENT_ORANGE);
        pros::screen::fill_rect(20, itemY, 26, itemY + 50);
      }

      pros::screen::set_pen(isSelected ? 0xFFFFFF : 0x777777);
      pros::screen::print(pros::E_TEXT_MEDIUM, 45, itemY + 15,
                          isSelected ? "> %s <" : "  %s",
                          autonNames[i].c_str());
    }

    // Header
    pros::screen::set_pen(ACCENT_ORANGE);
    pros::screen::fill_rect(5, 5, 475, 50);
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_SMALL, 150, 20, "SELECT AUTON");

    // Scroll Buttons
    pros::screen::set_pen(ACCENT_DARK_GRAY);
    pros::screen::fill_rect(435, 60, 475, 130);
    pros::screen::fill_rect(435, 140, 475, 210);
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 448, 85, "^");
    pros::screen::print(pros::E_TEXT_MEDIUM, 448, 165, "v");

    // Footer
    pros::screen::set_pen(0x18181F);
    pros::screen::fill_rect(0, 215, 480, 240);
    pros::screen::set_pen(ACCENT_CYAN);
    pros::screen::print(pros::E_TEXT_SMALL, 15, 222,
                        "Brain or Controller: D-Pad + A | Touch to Select");

    // Controller Display
    controller.print(0, 0, "SELECT AUTON   ");
    controller.print(1, 0, "%s            ",
                     autonNames[currentAutonIndex].c_str());
    controller.print(2, 0, "A = OK        ");

    pros::delay(50);
  }

  controller.clear();
  controller.print(0, 0, "Saved!");

  pros::screen::set_pen(BG_DARK);
  pros::screen::fill_rect(0, 0, 480, 240);
  pros::screen::set_pen(ACCENT_ORANGE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 150, 100, "AUTON SAVED");
  pros::screen::print(pros::E_TEXT_MEDIUM, 100, 150, "%s",
                      autonNames[currentAutonIndex].c_str());
  pros::delay(1000);
}

// ─────────────────────────────────────────────────────────────────────────────
// initialize
// ─────────────────────────────────────────────────────────────────────────────
void initialize() {
  loadAutonSelection();
  selectAutonFromController();

  pros::Task catapult_control(catapultTask, nullptr, "Catapult Task");
  pros::Task intake_control(intakeTask, nullptr, "Intake Task");

  catapult_arm.tare_position();
  matchloader.tare_position();
  descore.tare_position();

  // Ensure legacy LLEMU is NOT active to prevent drawing conflicts
  chassis.calibrate();

  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  catapult_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // ── Cyber-HUD On-screen UI ──
  static pros::Task screen_task([]() {
    const int BG_DARK = 0x0A0A0F;
    const int CARD_BG = 0x161B22;
    const int ACCENT_CYAN = 0x00F0FF;
    const int ACCENT_ORANGE = 0xFF8C00;
    const int ACCENT_RED = 0xFF3131;
    const int ACCENT_DARK_GRAY = 0x22222B;

    int tempsScroll = 0;
    int autonScroll = 0;

    while (true) {
      pros::screen_touch_status_s_t status = pros::screen::touch_status();

      // ── Input Detection ──
      static bool wasTouched = false;
      if (status.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
        // 1. Header Tab Switch
        if (status.y < 50) {
          if (status.x < 240)
            currentTab = TAB_TEMPS;
          else
            currentTab = TAB_AUTON;
        }

        // 2. Scroll Buttons (Far Right Column: 430 to 480)
        if (status.x > 430) {
          if (status.y > 60 && status.y < 130) { // UP Arrow
            if (currentTab == TAB_AUTON)
              autonScroll += 55;
            else
              tempsScroll += 65;
          } else if (status.y > 140 && status.y < 210) { // DOWN Arrow
            if (currentTab == TAB_AUTON)
              autonScroll -= 55;
            else
              tempsScroll -= 65;
          }
        }

        // 3. Selection in Auton Tab
        if (currentTab == TAB_AUTON && status.x < 420 && status.y > 50 &&
            status.y < 215) {
          for (int i = 0; i < (int)autonNames.size(); i++) {
            int itemY = 60 + (i * 55) + autonScroll;
            if (status.y >= itemY && status.y <= itemY + 50) {
              currentAutonIndex = i;
              saveAutonSelection();
              break;
            }
          }
        }
        wasTouched = true;
      } else if (status.touch_status == pros::E_TOUCH_RELEASED) {
        wasTouched = false;
      }

      // Clamp Scrolling
      if (autonScroll > 0)
        autonScroll = 0;
      int maxAutonScroll = -((int)autonNames.size() * 55 - 140);
      if (autonScroll < maxAutonScroll && maxAutonScroll < 0)
        autonScroll = maxAutonScroll;

      if (tempsScroll > 0)
        tempsScroll = 0;
      if (tempsScroll < -130)
        tempsScroll = -130;

      // ── RENDERING ──
      // Clear only the content area to reduce flicker
      pros::screen::set_pen(BG_DARK);
      pros::screen::fill_rect(0, 0, 480, 240);

      // Subtle Grid
      pros::screen::set_pen(0x121217);
      for (int i = 0; i < 480; i += 40)
        pros::screen::draw_line(i, 0, i, 240);
      for (int i = 0; i < 240; i += 40)
        pros::screen::draw_line(0, i, 480, i);

      // ── Content Area Rendering (With Clipping Check) ──
      if (currentTab == TAB_TEMPS) {
        auto drawTechCard = [&](int x, int y, const char *name, double temp) {
          // Clipping: Only draw if the card is between the header (50) and
          // footer (215)
          if (y + 55 < 55 || y > 215)
            return;

          int w = 200, h = 55;
          uint32_t col = (temp < 45) ? ACCENT_CYAN
                                     : (temp < 55 ? ACCENT_ORANGE : ACCENT_RED);

          pros::screen::set_pen(CARD_BG);
          pros::screen::fill_rect(x, y, x + w, y + h);
          pros::screen::set_pen(col);
          pros::screen::fill_rect(x, y, x + 4, y + h);

          pros::screen::set_pen(0xAAAAAA);
          pros::screen::print(pros::E_TEXT_SMALL, x + 12, y + 8, name);
          pros::screen::set_pen(col);
          pros::screen::print(pros::E_TEXT_MEDIUM, x + 12, y + 24, "%.1f C",
                              temp);
        };

        int yBase = 60 + tempsScroll;
        drawTechCard(15, yBase, "[ DRIVE_L ]",
                     left_motor_group.get_temperature());
        drawTechCard(220, yBase, "[ DRIVE_R ]",
                     right_motor_group.get_temperature());
        drawTechCard(15, yBase + 65, "[ INTAKE ]", intake.get_temperature());
        drawTechCard(220, yBase + 65, "[ CATAPULT ]",
                     catapult_arm.get_temperature());
        drawTechCard(15, yBase + 130, "[ MATCHLOAD ]",
                     matchloader.get_temperature());
        drawTechCard(220, yBase + 130, "[ DESCORE ]",
                     descore.get_temperature());
        drawTechCard(15, yBase + 195, "[ ARM ]", arm.get_temperature());
        drawTechCard(220, yBase + 195, "[ GATE ]", gate.get_temperature());
      } else {
        for (int i = 0; i < (int)autonNames.size(); i++) {
          int itemY = 60 + (i * 55) + autonScroll;

          // Clipping Check
          if (itemY + 50 < 55 || itemY > 215)
            continue;

          bool isSelected = (i == currentAutonIndex);
          pros::screen::set_pen(isSelected ? 0x1c2838 : CARD_BG);
          pros::screen::fill_rect(20, itemY, 420, itemY + 50);

          pros::screen::set_pen(isSelected ? ACCENT_ORANGE : 0x2d2d38);
          pros::screen::draw_rect(20, itemY, 420, itemY + 50);

          if (isSelected) {
            pros::screen::set_pen(ACCENT_ORANGE);
            pros::screen::fill_rect(20, itemY, 26, itemY + 50);
          }

          pros::screen::set_pen(isSelected ? 0xFFFFFF : 0x777777);
          pros::screen::print(pros::E_TEXT_MEDIUM, 45, itemY + 15,
                              isSelected ? "> %s <" : "  %s",
                              autonNames[i].c_str());
        }
      }

      // ── Static Overlays (Drawn last to prevent overlap) ──

      // Header Tabs
      pros::screen::set_pen(currentTab == TAB_TEMPS ? ACCENT_CYAN
                                                    : ACCENT_DARK_GRAY);
      pros::screen::fill_rect(5, 5, 238, 50);
      pros::screen::set_pen(currentTab == TAB_AUTON ? ACCENT_ORANGE
                                                    : ACCENT_DARK_GRAY);
      pros::screen::fill_rect(242, 5, 475, 50);

      pros::screen::set_pen(0xFFFFFF);
      pros::screen::print(pros::E_TEXT_SMALL, 60, 20, "SYSTEM_THERMALS");
      pros::screen::print(pros::E_TEXT_SMALL, 305, 20, "MISSION_CONFIG");

      // Scroll Buttons Sidebar
      pros::screen::set_pen(ACCENT_DARK_GRAY);
      pros::screen::fill_rect(435, 60, 475, 130);  // Up Button
      pros::screen::fill_rect(435, 140, 475, 210); // Down Button
      pros::screen::set_pen(0xFFFFFF);
      pros::screen::print(pros::E_TEXT_MEDIUM, 448, 85, "^");
      pros::screen::print(pros::E_TEXT_MEDIUM, 448, 165, "v");

      // Footer Diagnostics
      pros::screen::set_pen(0x18181F);
      pros::screen::fill_rect(0, 215, 480, 240);
      pros::screen::set_pen(ACCENT_CYAN);
      pros::screen::print(pros::E_TEXT_SMALL, 15, 222,
                          "OS_READY // IMU: %.1f // BAT: %.0f%% // %s",
                          imu.get_heading(), pros::battery::get_capacity(),
                          autonNames[currentAutonIndex].c_str());

      pros::delay(35); // Slowed down slightly for smoother rendering
    }
  });
}

void opcontrol() { selectAutonFromController(); }
void autonomous() { runAutonomous(); }