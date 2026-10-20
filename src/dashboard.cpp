#include "dashboard.hpp"
#include "canvas.hpp"
#include <algorithm>
#include <cstdio>

namespace dashboard {

// ── Palette ──
const uint32_t BG_TOP = 0x0D1119;
const uint32_t BG_BOTTOM = 0x07090D;
const uint32_t HEADER = 0x111620;
const uint32_t DIVIDER = 0x222A36;
const uint32_t TILE = 0x161C26;
const uint32_t TILE_EDGE = 0x262F3C;
const uint32_t TRACK = 0x2A323E;     // empty part of gauges / switch track
const uint32_t TEXT = 0xE6EDF3;
const uint32_t TEXT_DIM = 0x8B949E;
const uint32_t TEXT_MUTED = 0x5C6570; // unplugged motors
const uint32_t OK = 0x3FB950;
const uint32_t WARM = 0xF0A830;
const uint32_t HOT = 0xF85149;
const uint32_t INFO = 0x58A6FF;
const uint32_t BRAND_GOLD = 0xF2C230; // from the logo's border

// ── Layout (480x240). Coordinates: x0/y0 inclusive, x1/y1 exclusive. ──
// Header: logo + team name on the left, battery / field mode / VEXNET switch on the right.
// Body is a top-down view of the robot: left drive on the far left, right drive on the
// far right, and the thermal summary + mechanisms in the middle.
const int HEADER_H = 40;
const int BATT_X1 = 250;
const int MODE_X0 = 258, MODE_X1 = 352;
const int SWITCH_X0 = 360, SWITCH_X1 = 474;
const int PILL_Y0 = 7, PILL_Y1 = 33;

const int TILE_H = 31, TILE_GAP = 3;
const int SIDE_W = 88;
const int COL_LEFT_DRIVE = 6, COL_RIGHT_DRIVE = 480 - 6 - SIDE_W;
const int SIDE_TITLE_Y = 46, SIDE_TILES_Y = 63;
const size_t SIDE_MAX_TILES = 5;

const int CENTER_X0 = 102, CENTER_X1 = 378;
const int SUMMARY_Y0 = 47, SUMMARY_Y1 = 97;
const int CENTER_COL_GAP = 8;
const int CENTER_TITLE_Y = 103, CENTER_TILES_Y = 120;
const size_t CENTER_MAX_TILES = 3;
const int AUTON_CARD_H = 36;
const int LEGEND_Y = 222;

const int MODAL_X0 = 100, MODAL_Y0 = 38, MODAL_X1 = 380, MODAL_Y1 = 202;
const int DISMISS_X0 = MODAL_X0 + 16, DISMISS_Y0 = MODAL_Y1 - 42;
const int DISMISS_X1 = MODAL_X1 - 16, DISMISS_Y1 = MODAL_Y1 - 14;
const int MODAL_MAX_LINES = 3;

// Auton picker: full screen, routines in a two-column grid under a title bar
const int PICKER_X0 = 10, PICKER_X1 = 470, PICKER_Y0 = 48, PICKER_Y1 = 232;
const int PICKER_GAP = 6;
const int BACK_X0 = 386, BACK_X1 = 470, BACK_Y0 = 7, BACK_Y1 = 33;

const int TOUCH_SLOP = 8;

static Canvas frame;

// Layout from the last render, for the touch hit tests
static int autonCardY0 = -1; // -1 = no auton card on screen
static int pickerCount = 0;  // routines in the picker, 0 = picker closed

struct Rect {
    int x0, y0, x1, y1;
};

static uint32_t tempColor(const MotorReading &m) {
    return !m.connected ? TEXT_MUTED : (m.temp < WARM_C ? OK : (m.temp < HOT_C ? WARM : HOT));
}

static bool inRect(int x, int y, const Rect &r, int slop) {
    return x >= r.x0 - slop && x < r.x1 + slop && y >= r.y0 - slop && y < r.y1 + slop;
}

static void gradientBackground(Canvas &c) {
    for (int y = 0; y < Canvas::H; y++) c.fillRect(0, y, Canvas::W, y + 1, mixColor(BG_TOP, BG_BOTTOM, y * 255 / (Canvas::H - 1)));
}

// Everything that never changes: background, header band, logo + team name.
static const Canvas &background() {
    static Canvas bg;
    static bool built = false;
    if (built) return bg;
    built = true;

    gradientBackground(bg);
    bg.fillRect(0, 0, Canvas::W, HEADER_H, HEADER);
    bg.fillRect(0, HEADER_H, Canvas::W, HEADER_H + 1, DIVIDER);

    bg.image(LOGO_SMALL, 8, (HEADER_H - LOGO_SMALL.h + 1) / 2);
    const int nameX = 8 + LOGO_SMALL.w + 8;
    bg.text(FONT_MEDIUM, nameX, 3, "QCU ROBOTICS", TEXT);
    bg.text(FONT_SMALL, nameX, 21, "MOTOR THERMALS", TEXT_DIM);
    return bg;
}

// Legend: what the tile colors mean
static void drawLegend() {
    struct { uint32_t color; const char *label; } legend[] = {
        {OK, "Under 45°"}, {WARM, "45-55°"}, {HOT, "55°+ (power cut)"}};
    int total = 0;
    for (auto &l : legend) total += 10 + frame.textWidth(FONT_SMALL, l.label) + 12;
    int x = (CENTER_X0 + CENTER_X1) / 2 - (total - 12) / 2;
    for (auto &l : legend) {
        frame.fillCircle(x + 3, LEGEND_Y + 9, 3, l.color);
        frame.text(FONT_SMALL, x + 10, LEGEND_Y, l.label, TEXT_DIM);
        x += 10 + frame.textWidth(FONT_SMALL, l.label) + 12;
    }
}

static void drawBattery(int pct) {
    uint32_t col = pct > 50 ? OK : (pct > 25 ? WARM : HOT);
    char text[8];
    snprintf(text, sizeof(text), "%d%%", pct);
    int textX = BATT_X1 - frame.textWidth(FONT_MEDIUM, text);
    frame.text(FONT_MEDIUM, textX, 11, text, TEXT);

    // Battery icon: outline, terminal nub, proportional fill
    const int bx1 = textX - 8, bx0 = bx1 - 24, by0 = 14, by1 = 26;
    frame.card(bx0, by0, bx1, by1, 3, HEADER, TEXT_DIM);
    frame.fillRect(bx1, by0 + 4, bx1 + 2, by1 - 4, TEXT_DIM);
    int fillW = std::max(2, (bx1 - bx0 - 4) * std::min(100, std::max(0, pct)) / 100);
    frame.fillRoundRect(bx0 + 2, by0 + 2, bx0 + 2 + fillW, by1 - 2, 1, col);
}

static void drawFieldMode(FieldMode mode) {
    static const char *const MODE_TEXT[] = {"PRACTICE", "WAITING", "DRIVER", "DISABLED"};
    const uint32_t modeColors[] = {INFO, WARM, OK, HOT};
    uint32_t col = modeColors[mode];
    frame.card(MODE_X0, PILL_Y0, MODE_X1, PILL_Y1, 13, mixColor(HEADER, col, 30), mixColor(HEADER, col, 110));
    frame.fillCircle(MODE_X0 + 13, (PILL_Y0 + PILL_Y1) / 2, 4, col);
    frame.text(FONT_MEDIUM, MODE_X0 + 23, PILL_Y0 + 3, MODE_TEXT[mode], col);
}

static void drawVexnetSwitch(bool on) {
    frame.card(SWITCH_X0, PILL_Y0, SWITCH_X1, PILL_Y1, 13, on ? mixColor(HEADER, OK, 30) : TILE, on ? OK : TILE_EDGE);
    frame.text(FONT_MEDIUM, SWITCH_X0 + 12, PILL_Y0 + 3, "VEXNET", on ? TEXT : TEXT_DIM);
    // Track + knob: knob right and green = on, left and gray = off
    const int tx1 = SWITCH_X1 - 6, tx0 = tx1 - 34, ty0 = PILL_Y0 + 5, ty1 = PILL_Y1 - 5;
    frame.fillRoundRect(tx0, ty0, tx1, ty1, 8, on ? OK : TRACK);
    frame.fillCircle(on ? tx1 - 8 : tx0 + 8, (ty0 + ty1) / 2, 6, on ? 0xFFFFFF : TEXT_DIM);
}

static void drawSummary(const State &s) {
    int hot = 0, warm = 0, unplugged = 0, total = 0;
    const MotorReading *hottest = nullptr;
    for (const Column *col : {&s.leftDrive, &s.rightDrive, &s.centerLeft, &s.centerRight}) {
        for (const MotorReading &m : col->motors) {
            total++;
            if (!m.connected) {
                unplugged++;
                continue;
            }
            if (m.temp >= HOT_C) hot++;
            else if (m.temp >= WARM_C) warm++;
            if (!hottest || m.temp > hottest->temp) hottest = &m;
        }
    }

    char title[32], detail[40];
    uint32_t col = hot ? HOT : (warm ? WARM : (unplugged ? BRAND_GOLD : OK));
    if (hot && warm) snprintf(title, sizeof(title), "%d HOT, %d WARM", hot, warm);
    else if (hot) snprintf(title, sizeof(title), "%d MOTOR%s HOT", hot, hot == 1 ? "" : "S");
    else if (warm) snprintf(title, sizeof(title), "%d MOTOR%s WARM", warm, warm == 1 ? "" : "S");
    else if (unplugged) snprintf(title, sizeof(title), "CHECK WIRING");
    else snprintf(title, sizeof(title), "ALL COOL");

    if (unplugged) snprintf(detail, sizeof(detail), "%d of %d motors not detected", unplugged, total);
    else if (hot) snprintf(detail, sizeof(detail), "Hot motors are losing power");
    else if (warm) snprintf(detail, sizeof(detail), "Let them rest when you can");
    else snprintf(detail, sizeof(detail), "All %d motors running normally", total);

    frame.card(CENTER_X0, SUMMARY_Y0, CENTER_X1, SUMMARY_Y1, 8, hot ? mixColor(TILE, HOT, 40) : TILE,
               hot || warm ? col : TILE_EDGE);
    frame.fillRoundRect(CENTER_X0 + 5, SUMMARY_Y0 + 8, CENTER_X0 + 9, SUMMARY_Y1 - 8, 2, col);
    frame.text(FONT_LARGE, CENTER_X0 + 18, SUMMARY_Y0 + 3, title, col);
    frame.text(FONT_SMALL, CENTER_X0 + 18, SUMMARY_Y0 + 29, detail, TEXT_DIM);

    // Hottest motor, right-aligned
    if (hottest) {
        char temp[12];
        snprintf(temp, sizeof(temp), "%.0f°", hottest->temp);
        int titleEnd = CENTER_X0 + 18 + frame.textWidth(FONT_LARGE, title);
        int detailEnd = CENTER_X0 + 18 + frame.textWidth(FONT_SMALL, detail);
        int tempX = CENTER_X1 - 12 - frame.textWidth(FONT_LARGE, temp);
        int nameX = CENTER_X1 - 12 - frame.textWidth(FONT_SMALL, hottest->name);
        if (tempX > titleEnd + 12 && nameX > detailEnd + 12) {
            frame.text(FONT_LARGE, CENTER_X1 - 12, SUMMARY_Y0 + 3, temp, tempColor(*hottest), Canvas::RIGHT);
            frame.text(FONT_SMALL, CENTER_X1 - 12, SUMMARY_Y0 + 29, hottest->name, TEXT_DIM, Canvas::RIGHT);
        }
    }
}

// One motor: name top-left, temperature right, heat gauge (20-70C) along the bottom.
static void drawTile(int x, int y, int w, const MotorReading &m) {
    const int x1 = x + w, y1 = y + TILE_H;
    uint32_t col = tempColor(m);
    bool hot = m.connected && m.temp >= HOT_C;
    bool warm = m.connected && m.temp >= WARM_C && !hot;

    // Warm / hot tiles get a colored border and tint so they jump out.
    frame.card(x, y, x1, y1, 6, hot ? mixColor(TILE, HOT, 70) : (warm ? mixColor(TILE, WARM, 30) : TILE),
               hot || warm ? col : TILE_EDGE);
    frame.fillRoundRect(x + 4, y + 6, x + 7, y1 - 6, 1, col); // status stripe

    frame.text(FONT_SMALL, x + 12, y + 3, m.label, m.connected ? TEXT : TEXT_MUTED);
    char text[12];
    if (m.connected) snprintf(text, sizeof(text), "%.0f°", m.temp);
    else snprintf(text, sizeof(text), "N/C");
    frame.text(m.connected ? FONT_LARGE : FONT_MEDIUM, x1 - 8, m.connected ? y + 1 : y + 6, text,
               hot ? 0xFFFFFF : (m.connected ? col : TEXT_MUTED), Canvas::RIGHT);

    // The gauge sits under the label, clear of the temperature
    const int gx0 = x + 12, gx1 = x1 - 8 - frame.textWidth(m.connected ? FONT_LARGE : FONT_MEDIUM, text) - 8;
    const int gy0 = y1 - 10, gy1 = y1 - 6;
    if (gx1 - gx0 >= 12) {
        frame.fillRoundRect(gx0, gy0, gx1, gy1, 2, hot ? mixColor(TRACK, HOT, 80) : TRACK);
        if (m.connected) {
            double frac = std::min(1.0, std::max(0.0, (m.temp - 20.0) / 50.0));
            frame.fillRoundRect(gx0, gy0, gx0 + std::max(4, (int)((gx1 - gx0) * frac)), gy1, 2, col);
        }
    }
}

// Title above, then one tile per motor. Returns the y just below the last tile.
static int drawColumn(int x, int titleY, int tilesY, int w, size_t maxTiles, const Column &col) {
    if (col.motors.empty()) return tilesY;
    frame.text(FONT_SMALL, x + 2, titleY, col.title, TEXT_DIM);
    size_t n = std::min(col.motors.size(), maxTiles);
    for (size_t i = 0; i < n; i++) drawTile(x, tilesY + (int)i * (TILE_H + TILE_GAP), w, col.motors[i]);
    return tilesY + (int)n * (TILE_H + TILE_GAP) - TILE_GAP;
}

static void drawAutonCard(int y0, const State &s) {
    const int y1 = y0 + AUTON_CARD_H;
    frame.card(CENTER_X0, y0, CENTER_X1, y1, 8, TILE, mixColor(TILE, INFO, 120));
    frame.fillRoundRect(CENTER_X0 + 5, y0 + 8, CENTER_X0 + 9, y1 - 8, 2, INFO);
    frame.text(FONT_SMALL, CENTER_X0 + 18, y0 + 10, "AUTON", TEXT_DIM);
    frame.text(FONT_MEDIUM, CENTER_X0 + 66, y0 + 8, (*s.autonNames)[s.selectedAuton].c_str(), TEXT);
    frame.text(FONT_SMALL, CENTER_X1 - 12, y0 + 10, "CHANGE >", INFO, Canvas::RIGHT);
}

// Grid cell for routine i of n, two columns
static Rect pickerCell(int i, int n) {
    int rows = (n + 1) / 2;
    int rowH = std::min(44, (PICKER_Y1 - PICKER_Y0 - PICKER_GAP * (rows - 1)) / rows);
    int colW = (PICKER_X1 - PICKER_X0 - PICKER_GAP) / 2;
    int x0 = PICKER_X0 + (i % 2) * (colW + PICKER_GAP), y0 = PICKER_Y0 + (i / 2) * (rowH + PICKER_GAP);
    return {x0, y0, x0 + colW, y0 + rowH};
}

static void drawAutonPicker(const State &s) {
    gradientBackground(frame);
    frame.fillRect(0, 0, Canvas::W, HEADER_H, HEADER);
    frame.fillRect(0, HEADER_H, Canvas::W, HEADER_H + 1, DIVIDER);
    frame.text(FONT_LARGE, 14, 7, "SELECT AUTONOMOUS", TEXT);

    frame.card(BACK_X0, BACK_Y0, BACK_X1, BACK_Y1, 13, TILE, TILE_EDGE);
    frame.text(FONT_MEDIUM, (BACK_X0 + BACK_X1) / 2, BACK_Y0 + 3, "BACK", TEXT, Canvas::CENTER);

    const std::vector<std::string> &names = *s.autonNames;
    for (int i = 0; i < (int)names.size(); i++) {
        Rect r = pickerCell(i, (int)names.size());
        bool sel = i == s.selectedAuton;
        frame.card(r.x0, r.y0, r.x1, r.y1, 8, sel ? mixColor(TILE, INFO, 50) : TILE, sel ? INFO : TILE_EDGE);
        int textY = (r.y0 + r.y1) / 2 - 10;
        if (sel) frame.fillCircle(r.x0 + 16, (r.y0 + r.y1) / 2, 5, INFO);
        frame.text(FONT_MEDIUM, r.x0 + 30, textY, names[i].c_str(), sel ? TEXT : TEXT_DIM);
    }
}

static void drawStressModal(const std::vector<std::string> &stressed) {
    frame.fillRect(0, 0, Canvas::W, Canvas::H, 0x000000, 170);
    frame.card(MODAL_X0, MODAL_Y0, MODAL_X1, MODAL_Y1, 12, 0x1C1215, HOT);

    // Warning badge + title
    frame.fillCircle(MODAL_X0 + 28, MODAL_Y0 + 26, 11, HOT);
    frame.text(FONT_MEDIUM, MODAL_X0 + 28, MODAL_Y0 + 16, "!", 0xFFFFFF, Canvas::CENTER);
    frame.text(FONT_LARGE, MODAL_X0 + 48, MODAL_Y0 + 11, "MOTOR STRESS", HOT);
    frame.text(FONT_SMALL, MODAL_X0 + 18, MODAL_Y0 + 42, "These motors are limiting their own power:", TEXT_DIM);

    // When the list overflows, the last line becomes "+N more"
    int shown = (int)stressed.size() > MODAL_MAX_LINES ? MODAL_MAX_LINES - 1 : (int)stressed.size();
    int lineY = MODAL_Y0 + 62;
    for (int i = 0; i < shown; i++) {
        frame.fillCircle(MODAL_X0 + 24, lineY + 9, 3, HOT);
        frame.text(FONT_MEDIUM, MODAL_X0 + 34, lineY, stressed[i].c_str(), TEXT);
        lineY += 19;
    }
    if (shown < (int)stressed.size()) {
        char text[24];
        snprintf(text, sizeof(text), "+%d more", (int)stressed.size() - shown);
        frame.text(FONT_MEDIUM, MODAL_X0 + 34, lineY, text, TEXT_DIM);
    }

    frame.card(DISMISS_X0, DISMISS_Y0, DISMISS_X1, DISMISS_Y1, 8, mixColor(0x1C1215, HOT, 60), HOT);
    frame.text(FONT_MEDIUM, (DISMISS_X0 + DISMISS_X1) / 2, DISMISS_Y0 + 4, "TAP TO HIDE FOR 1 MIN", TEXT, Canvas::CENTER);
}

void showSplash(const char *status, const char *hint) {
    gradientBackground(frame);
    const int logoX = 28, logoY = (Canvas::H - LOGO.h) / 2;
    frame.image(LOGO, logoX, logoY);

    const int x = logoX + LOGO.w + 28;
    frame.text(FONT_XLARGE, x, 62, "QCU ROBOTICS", TEXT);
    frame.text(FONT_MEDIUM, x, 98, "VEX V5  |  WORLDS", TEXT_DIM);
    frame.fillRect(x, 126, x + 40, 129, BRAND_GOLD);
    frame.text(FONT_MEDIUM, x, 140, status, TEXT);
    if (hint) frame.text(FONT_SMALL, x, 162, hint, TEXT_DIM);
    frame.present();
}

void render(const State &s) {
    bool hasAutons = s.autonNames && !s.autonNames->empty();
    autonCardY0 = -1;
    pickerCount = 0;

    if (hasAutons && s.autonPickerOpen) {
        drawAutonPicker(s);
        pickerCount = (int)s.autonNames->size();
    } else {
        frame.copyFrom(background().pixels());

        drawBattery(s.battery);
        drawFieldMode(s.fieldMode);
        drawVexnetSwitch(s.vexnet);
        drawSummary(s);

        drawColumn(COL_LEFT_DRIVE, SIDE_TITLE_Y, SIDE_TILES_Y, SIDE_W, SIDE_MAX_TILES, s.leftDrive);
        drawColumn(COL_RIGHT_DRIVE, SIDE_TITLE_Y, SIDE_TILES_Y, SIDE_W, SIDE_MAX_TILES, s.rightDrive);

        // Middle: two columns side by side, or one spanning the middle if the other is empty
        int centerBottom = CENTER_TITLE_Y;
        if (!s.centerLeft.motors.empty() && !s.centerRight.motors.empty()) {
            const int w = (CENTER_X1 - CENTER_X0 - CENTER_COL_GAP) / 2;
            centerBottom = std::max(drawColumn(CENTER_X0, CENTER_TITLE_Y, CENTER_TILES_Y, w, CENTER_MAX_TILES, s.centerLeft),
                                    drawColumn(CENTER_X1 - w, CENTER_TITLE_Y, CENTER_TILES_Y, w, CENTER_MAX_TILES, s.centerRight));
        } else {
            const Column &only = s.centerLeft.motors.empty() ? s.centerRight : s.centerLeft;
            centerBottom = drawColumn(CENTER_X0, CENTER_TITLE_Y, CENTER_TILES_Y, CENTER_X1 - CENTER_X0, CENTER_MAX_TILES, only);
        }

        // The auton card goes under the middle tiles; the legend gives way if there's no room for both
        bool legend = true;
        if (hasAutons) {
            autonCardY0 = std::min(centerBottom + 8, Canvas::H - 4 - AUTON_CARD_H);
            drawAutonCard(autonCardY0, s);
            legend = autonCardY0 + AUTON_CARD_H + 2 <= LEGEND_Y;
        }
        if (legend) drawLegend();
    }

    if (s.stressModalOpen) drawStressModal(s.stressedMotors); // last, so it sits on top

    frame.present();
}

bool hitVexnetSwitch(int x, int y) {
    return pickerCount == 0 && inRect(x, y, {SWITCH_X0, 0, SWITCH_X1, PILL_Y1}, TOUCH_SLOP);
}

bool hitStressDismiss(int x, int y) { return inRect(x, y, {DISMISS_X0, DISMISS_Y0, DISMISS_X1, DISMISS_Y1}, TOUCH_SLOP); }

bool hitAutonCard(int x, int y) {
    return autonCardY0 >= 0 && inRect(x, y, {CENTER_X0, autonCardY0, CENTER_X1, autonCardY0 + AUTON_CARD_H}, 4);
}

int hitAutonPicker(int x, int y) {
    if (pickerCount == 0) return -1;
    if (inRect(x, y, {BACK_X0, BACK_Y0, BACK_X1, BACK_Y1}, TOUCH_SLOP)) return PICKER_BACK;
    for (int i = 0; i < pickerCount; i++)
        if (inRect(x, y, pickerCell(i, pickerCount), PICKER_GAP / 2)) return i;
    return -1;
}

} // namespace dashboard
