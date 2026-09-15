#include "autonomous_builder.hpp"
#include "motors.hpp"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdio>
#include <vector>

namespace {
enum class BlockType {
  MOVE_TO_POINT,
  MOVE_TO_POSE,
  TURN_TO_HEADING,
  TURN_TO_POINT,
  SWING_TO_HEADING,
  SWING_TO_POINT
};

struct Block {
  BlockType type;
  float x = 24.0f;
  float y = 24.0f;
  float heading = 0.0f;
  int timeout = 3000;
  int maxSpeed = 127;
  bool forwards = true;
  bool leftSwing = true;
};

std::vector<Block> blocks;
enum class CursorZone { PALETTE, FLOW };
CursorZone controllerZone = CursorZone::PALETTE;
int controllerIndex = 0;
bool controllerDragging = false;
Block controllerBlock{BlockType::MOVE_TO_POINT};
bool editingBlock = false;
int editingField = 0;
const int kPaletteX0 = 8;
const int kPaletteX1 = 148;
const int kFlowX0 = 158;
const int kFlowX1 = 472;
const int kHeaderY1 = 32;
const int kBlockHeight = 27;

struct Rect { int x0, y0, x1, y1; };
bool inRect(const Rect &r, int x, int y) {
  return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

void fillRect(const Rect &r, int color) {
  pros::screen::set_pen(color);
  pros::screen::fill_rect(r.x0, r.y0, r.x1, r.y1);
}

const char *label(BlockType type) {
  switch (type) {
  case BlockType::MOVE_TO_POINT: return "MOVE PT";
  case BlockType::MOVE_TO_POSE: return "MOVE POSE";
  case BlockType::TURN_TO_HEADING: return "TURN HDG";
  case BlockType::TURN_TO_POINT: return "TURN PT";
  case BlockType::SWING_TO_HEADING: return "SWING HDG";
  case BlockType::SWING_TO_POINT: return "SWING PT";
  }
  return "BLOCK";
}

void blockSummary(const Block &block, char *buffer, size_t length) {
  switch (block.type) {
  case BlockType::MOVE_TO_POINT:
    snprintf(buffer, length, "X%.1f Y%.1f T%d S%d %s", block.x, block.y, block.timeout,
             block.maxSpeed, block.forwards ? "FWD" : "REV");
    break;
  case BlockType::MOVE_TO_POSE:
    snprintf(buffer, length, "X%.1f Y%.1f H%.1f T%d S%d %s", block.x, block.y, block.heading,
             block.timeout, block.maxSpeed, block.forwards ? "FWD" : "REV");
    break;
  case BlockType::TURN_TO_HEADING:
    snprintf(buffer, length, "H%.1f T%d S%d", block.heading, block.timeout, block.maxSpeed);
    break;
  case BlockType::TURN_TO_POINT:
    snprintf(buffer, length, "X%.1f Y%.1f T%d S%d %s", block.x, block.y, block.timeout,
             block.maxSpeed, block.forwards ? "FWD" : "REV");
    break;
  case BlockType::SWING_TO_HEADING:
    snprintf(buffer, length, "H%.1f T%d S%d %s", block.heading, block.timeout, block.maxSpeed,
             block.leftSwing ? "LEFT" : "RIGHT");
    break;
  case BlockType::SWING_TO_POINT:
    snprintf(buffer, length, "X%.1f Y%.1f T%d S%d %s %s", block.x, block.y, block.timeout,
             block.maxSpeed, block.forwards ? "FWD" : "REV", block.leftSwing ? "LEFT" : "RIGHT");
    break;
  }
}

bool isLinear(BlockType type) {
  return type == BlockType::MOVE_TO_POINT || type == BlockType::MOVE_TO_POSE;
}

int fieldCount(BlockType type) {
  switch (type) {
  case BlockType::MOVE_TO_POINT: return 5; // X Y TIME SPEED FWD
  case BlockType::MOVE_TO_POSE: return 6; // X Y HDG TIME SPEED FWD
  case BlockType::TURN_TO_HEADING: return 3; // HDG TIME SPEED
  case BlockType::TURN_TO_POINT: return 5; // X Y TIME SPEED FWD
  case BlockType::SWING_TO_HEADING: return 4; // HDG TIME SPEED SIDE
  case BlockType::SWING_TO_POINT: return 6; // X Y TIME SPEED FWD SIDE
  }
  return 1;
}

const char *fieldName(BlockType type, int field) {
  static const char *movePoint[] = {"X", "Y", "TIME", "SPEED", "FWD"};
  static const char *movePose[] = {"X", "Y", "HDG", "TIME", "SPEED", "FWD"};
  static const char *turnHeading[] = {"HDG", "TIME", "SPEED"};
  static const char *turnPoint[] = {"X", "Y", "TIME", "SPEED", "FWD"};
  static const char *swingHeading[] = {"HDG", "TIME", "SPEED", "SIDE"};
  static const char *swingPoint[] = {"X", "Y", "TIME", "SPEED", "FWD", "SIDE"};
  const char **names = movePoint;
  switch (type) {
  case BlockType::MOVE_TO_POSE: names = movePose; break;
  case BlockType::TURN_TO_HEADING: names = turnHeading; break;
  case BlockType::TURN_TO_POINT: names = turnPoint; break;
  case BlockType::SWING_TO_HEADING: names = swingHeading; break;
  case BlockType::SWING_TO_POINT: names = swingPoint; break;
  default: break;
  }
  return names[field];
}

void adjustField(Block &block, int field, int direction) {
  const char *name = fieldName(block.type, field);
  if (name[0] == 'X') block.x += direction;
  else if (name[0] == 'Y') block.y += direction;
  else if (name[0] == 'H') block.heading += direction * 5.0f;
  else if (name[0] == 'T') block.timeout = std::clamp(block.timeout + direction * 100, 100, 15000);
  else if (name[0] == 'S') {
    if (name[1] == 'P') block.maxSpeed = std::clamp(block.maxSpeed + direction * 5, 1, 127);
    else block.leftSwing = !block.leftSwing;
  } else if (name[0] == 'F') block.forwards = !block.forwards;
}

void drawFieldValue(const Block &block, int field) {
  const char *name = fieldName(block.type, field);
  char value[24];
  if (name[0] == 'X') snprintf(value, sizeof(value), "%.1f", block.x);
  else if (name[0] == 'Y') snprintf(value, sizeof(value), "%.1f", block.y);
  else if (name[0] == 'H') snprintf(value, sizeof(value), "%.1f", block.heading);
  else if (name[0] == 'T') snprintf(value, sizeof(value), "%d", block.timeout);
  else if (name[0] == 'S' && name[1] == 'P') snprintf(value, sizeof(value), "%d", block.maxSpeed);
  else if (name[0] == 'F') snprintf(value, sizeof(value), "%s", block.forwards ? "FWD" : "REV");
  else snprintf(value, sizeof(value), "%s", block.leftSwing ? "LEFT" : "RIGHT");
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::set_eraser(0x121A20);
  pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 8, 202, "%s = %s", name, value);
}

BlockType paletteType(int x, int y) {
  if (x < kPaletteX0 || x > kPaletteX1) return BlockType::MOVE_TO_POINT;
  if (y >= 45 && y < 69) return BlockType::MOVE_TO_POINT;
  if (y >= 72 && y < 96) return BlockType::MOVE_TO_POSE;
  if (y >= 105 && y < 129) return BlockType::TURN_TO_HEADING;
  if (y >= 132 && y < 156) return BlockType::TURN_TO_POINT;
  if (y >= 159 && y < 183) return BlockType::SWING_TO_HEADING;
  return BlockType::SWING_TO_POINT;
}

void drawButton(const Rect &r, const char *text, int color) {
  fillRect(r, color);
  pros::screen::set_pen(0x4A5965);
  pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
  pros::screen::set_eraser(color);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 9, "%s", text);
}

void drawEditor() {
  fillRect({0, 0, 479, 239}, 0x0A0A0F);
  fillRect({0, 0, 479, kHeaderY1}, 0x00A7B8);
  pros::screen::set_eraser(0x00A7B8);
  pros::screen::set_pen(0x000000);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 9, "AUTON // LINEAR + ANGULAR");
  pros::screen::print(pros::E_TEXT_SMALL, 300, 9, editingBlock ? "L/R FIELD  U/D VALUE  A DONE" : "ARROWS  A PICK/DROP");

  pros::screen::set_eraser(0x0A0A0F);
  pros::screen::set_pen(0x56D6E8);
  pros::screen::print(pros::E_TEXT_SMALL, 8, 38, "LINEAR");
  drawButton({kPaletteX0, 45, kPaletteX1, 69}, "MOVE TO POINT", 0x162A32);
  drawButton({kPaletteX0, 72, kPaletteX1, 96}, "MOVE TO POSE", 0x162A32);
  pros::screen::set_pen(0xFFAD66);
  pros::screen::print(pros::E_TEXT_SMALL, 8, 99, "ANGULAR");
  drawButton({kPaletteX0, 105, kPaletteX1, 129}, "TURN HEADING", 0x33271B);
  drawButton({kPaletteX0, 132, kPaletteX1, 156}, "TURN POINT", 0x33271B);
  drawButton({kPaletteX0, 159, kPaletteX1, 183}, "SWING HEADING", 0x33271B);
  drawButton({kPaletteX0, 186, kPaletteX1, 210}, "SWING POINT", 0x33271B);

  fillRect({kFlowX0, 35, kFlowX1, 217}, 0x121A20);
  pros::screen::set_pen(0x56D6E8);
  pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 8, 41, "FLOW // VALUES SHOWN BELOW");
  if (blocks.empty()) {
    pros::screen::set_pen(0x77838C);
    pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 45, 112, "DROP LINEAR / ANGULAR");
  }
  for (size_t i = 0; i < blocks.size() && i < 5; i++) {
    int y = 48 + (int)i * kBlockHeight;
    int color = isLinear(blocks[i].type) ? 0x16414A : 0x49311D;
    fillRect({kFlowX0 + 6, y, kFlowX1 - 6, y + 22}, color);
    pros::screen::set_eraser(color);
    pros::screen::set_pen(0xFFFFFF);
    char summary[64];
    blockSummary(blocks[i], summary, sizeof(summary));
    pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 14, y + 3, "%d %s", (int)i + 1, label(blocks[i].type));
    pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 14, y + 14, "%s", summary);
  }
  if (editingBlock && controllerIndex >= 0 && controllerIndex < (int)blocks.size()) {
    const Block &block = blocks[controllerIndex];
    pros::screen::set_pen(0xFFAD66);
    pros::screen::set_eraser(0x121A20);
    pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 8, 190, "EDIT %s  FIELD %d/%d", label(block.type),
                        editingField + 1, fieldCount(block.type));
    drawFieldValue(block, editingField);
  }
  if (!controllerDragging && controllerZone == CursorZone::FLOW && !blocks.empty()) {
    int index = std::min(controllerIndex, (int)blocks.size() - 1);
    int y = 48 + index * kBlockHeight;
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::draw_rect(kFlowX0 + 5, y - 1, kFlowX1 - 5, y + 23);
  }
  if (controllerZone == CursorZone::PALETTE) {
    static const int paletteY[] = {45, 72, 105, 132, 159, 186};
    int y = paletteY[std::clamp(controllerIndex, 0, 5)];
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::draw_rect(kPaletteX0 - 1, y - 1, kPaletteX1 + 1, std::min(y + 31, 239));
  }
  if (controllerDragging) {
    pros::screen::set_pen(0xFFAD66);
    pros::screen::print(pros::E_TEXT_SMALL, kFlowX0 + 150, 41, "HOLDING %s", label(controllerBlock.type));
  }
  drawButton({158, 219, 230, 239}, "BACK", 0x252D34);
  drawButton({238, 219, 315, 239}, "CLEAR", 0x49252A);
  drawButton({323, 219, 398, 239}, "RUN", 0x176B4A);
  pros::screen::set_eraser(0x0A0A0F);
  pros::screen::set_pen(0xAEBAC4);
  pros::screen::print(pros::E_TEXT_SMALL, 405, 233, "%d/5", (int)blocks.size());
}

void runBlock(const Block &block) {
  switch (block.type) {
  case BlockType::MOVE_TO_POINT:
    chassis.moveToPoint(block.x, block.y, block.timeout,
                        {.forwards = block.forwards, .maxSpeed = (float)block.maxSpeed});
    break;
  case BlockType::MOVE_TO_POSE:
    chassis.moveToPose(block.x, block.y, block.heading, block.timeout,
                       {.forwards = block.forwards, .maxSpeed = (float)block.maxSpeed});
    break;
  case BlockType::TURN_TO_HEADING:
    chassis.turnToHeading(block.heading, block.timeout, {.maxSpeed = (float)block.maxSpeed});
    break;
  case BlockType::TURN_TO_POINT:
    chassis.turnToPoint(block.x, block.y, block.timeout,
                        {.forwards = block.forwards, .maxSpeed = (float)block.maxSpeed});
    break;
  case BlockType::SWING_TO_HEADING:
    chassis.swingToHeading(block.heading, block.leftSwing ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT,
                           block.timeout, {.maxSpeed = (float)block.maxSpeed});
    break;
  case BlockType::SWING_TO_POINT:
    chassis.swingToPoint(block.x, block.y, block.leftSwing ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT,
                         block.timeout, {.forwards = block.forwards, .maxSpeed = (float)block.maxSpeed});
    break;
  }
  chassis.waitUntilDone();
}

void runBlocks() {
  for (const Block &block : blocks)
    runBlock(block);
}
} // namespace

void autonomousBuilderControl() {
  drawEditor();
  bool dragging = false;
  BlockType draggedType = BlockType::MOVE_TO_POINT;
  bool wasTouched = false;
  controllerZone = CursorZone::PALETTE;
  controllerIndex = 0;
  controllerDragging = false;
  editingBlock = false;
  editingField = 0;

  auto pressed = [](pros::controller_digital_e_t button, bool &wasHeld) {
    bool held = controller.get_digital(button);
    bool edge = held && !wasHeld;
    wasHeld = held;
    return edge;
  };
  bool wasA = false, wasB = false, wasX = false, wasY = false, wasR1 = false, wasR2 = false;
  bool wasUp = false, wasDown = false, wasLeft = false, wasRight = false;

  while (true) {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    bool touchPressed = touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched;
    bool released = touch.touch_status == pros::E_TOUCH_RELEASED && wasTouched;
    if (touch.touch_status == pros::E_TOUCH_PRESSED || touch.touch_status == pros::E_TOUCH_HELD)
      wasTouched = true;

    if (touchPressed && touch.x >= kPaletteX0 && touch.x <= kPaletteX1 && touch.y >= 45) {
      dragging = true;
      draggedType = paletteType(touch.x, touch.y);
    }
    if (released) {
      if (dragging && touch.x >= kFlowX0 && touch.x <= kFlowX1 && touch.y < 224 && blocks.size() < 5) {
        blocks.push_back({draggedType});
        controllerZone = CursorZone::FLOW;
        controllerIndex = (int)blocks.size() - 1;
        drawEditor();
      } else if (touch.x >= 158 && touch.x < 230 && touch.y >= 219) {
        return;
      } else if (touch.x >= 238 && touch.x < 315 && touch.y >= 219) {
        blocks.clear();
        controllerZone = CursorZone::PALETTE;
        controllerIndex = 0;
        controllerDragging = false;
        drawEditor();
      } else if (touch.x >= 323 && touch.x < 398 && touch.y >= 219) {
        runBlocks();
        drawEditor();
      } else if (touch.x >= kFlowX0 && touch.x <= kFlowX1 && touch.y >= 48 && touch.y < 48 + (int)blocks.size() * kBlockHeight) {
        int index = (touch.y - 48) / kBlockHeight;
        if (index >= 0 && index < (int)blocks.size()) {
          controllerZone = CursorZone::FLOW;
          controllerIndex = index;
          editingBlock = true;
          editingField = 0;
          drawEditor();
        }
      }
      dragging = false;
      wasTouched = false;
    }
    if (touch.touch_status == pros::E_TOUCH_ERROR)
      wasTouched = false;

    bool a = pressed(pros::E_CONTROLLER_DIGITAL_A, wasA);
    bool b = pressed(pros::E_CONTROLLER_DIGITAL_B, wasB);
    bool x = pressed(pros::E_CONTROLLER_DIGITAL_X, wasX);
    bool y = pressed(pros::E_CONTROLLER_DIGITAL_Y, wasY);
    bool r1 = pressed(pros::E_CONTROLLER_DIGITAL_R1, wasR1);
    bool r2 = pressed(pros::E_CONTROLLER_DIGITAL_R2, wasR2);
    bool up = pressed(pros::E_CONTROLLER_DIGITAL_UP, wasUp);
    bool down = pressed(pros::E_CONTROLLER_DIGITAL_DOWN, wasDown);
    bool left = pressed(pros::E_CONTROLLER_DIGITAL_LEFT, wasLeft);
    bool right = pressed(pros::E_CONTROLLER_DIGITAL_RIGHT, wasRight);
    if (editingBlock && controllerIndex >= 0 && controllerIndex < (int)blocks.size()) {
      Block &block = blocks[controllerIndex];
      if (a) {
        editingBlock = false;
        drawEditor();
      }
      if (left || right) {
        editingField = (editingField + (right ? 1 : fieldCount(block.type) - 1)) % fieldCount(block.type);
        drawEditor();
      }
      if (up || down) {
        adjustField(block, editingField, up ? 1 : -1);
        drawEditor();
      }
      pros::delay(20);
      continue;
    }

    if (y && controllerZone == CursorZone::FLOW && controllerIndex < (int)blocks.size()) {
      editingBlock = true;
      editingField = 0;
      drawEditor();
    }

    // The D-pad is the controller's cursor. Left/right switches between the
    // palette and flow; up/down moves within the active column.
    if (left || right) {
      controllerZone = controllerZone == CursorZone::PALETTE ? CursorZone::FLOW : CursorZone::PALETTE;
      if (controllerZone == CursorZone::FLOW)
        controllerIndex = std::min(controllerIndex, (int)blocks.size());
      else
        controllerIndex = std::clamp(controllerIndex, 0, 5);
      drawEditor();
    }
    if (up || down) {
      int delta = up ? -1 : 1;
      if (controllerZone == CursorZone::PALETTE)
        controllerIndex = std::clamp(controllerIndex + delta, 0, 5);
      else
        controllerIndex = std::clamp(controllerIndex + delta, 0, (int)blocks.size());
      drawEditor();
    }
    if (a) {
      if (!controllerDragging && controllerZone == CursorZone::PALETTE) {
        controllerBlock = {static_cast<BlockType>(controllerIndex)};
        controllerDragging = true;
        controllerZone = CursorZone::FLOW;
        controllerIndex = std::min(controllerIndex, (int)blocks.size());
      } else if (controllerDragging && controllerZone == CursorZone::FLOW && blocks.size() < 5) {
        blocks.insert(blocks.begin() + controllerIndex, controllerBlock);
        controllerDragging = false;
        drawEditor();
      } else if (!controllerDragging && controllerZone == CursorZone::FLOW && controllerIndex < (int)blocks.size()) {
        controllerBlock = blocks[controllerIndex];
        blocks.erase(blocks.begin() + controllerIndex);
        controllerDragging = true;
        controllerIndex = std::min(controllerIndex, (int)blocks.size());
        drawEditor();
      }
    }
    if (x && !controllerDragging && controllerZone == CursorZone::FLOW && controllerIndex < (int)blocks.size()) {
      blocks.erase(blocks.begin() + controllerIndex);
      controllerIndex = std::min(controllerIndex, (int)blocks.size());
      drawEditor();
    }
    if (b) {
      blocks.clear();
      controllerZone = CursorZone::PALETTE;
      controllerIndex = 0;
      controllerDragging = false;
      editingBlock = false;
      drawEditor();
    }
    if (r1) {
      runBlocks();
      drawEditor();
    }
    if (r2) {
      controllerDragging = false;
      return;
    }
    pros::delay(20);
  }
}
