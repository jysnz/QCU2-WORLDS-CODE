#pragma once
#include <string>
#include <vector>

// Brain-screen thermal dashboard. main.cpp gathers the live data into a
// dashboard::State and handles touch; everything visual lives in dashboard.cpp.
namespace dashboard {

// Thermal bands. V5 motors start cutting their own power at 55C, so that's "hot".
constexpr double WARM_C = 45, HOT_C = 55;

struct MotorReading {
    char label[12]; // shown on the motor's tile
    char name[16];  // used in the summary / stress list
    double temp;    // PROS_ERR_F (infinity) when the motor is unplugged
    bool connected;
    bool overTemp;       // the motor reports it is cutting power because it's hot
    bool atCurrentLimit; // the motor reports it is drawing as much current as it's allowed
};

// One sensor on the SENSORS tab. `port` says where it is plugged in ("PORT 9" or "ADI D").
struct SensorReading {
    char label[16]; // e.g. "IMU"
    char port[12];  // smart port or ADI port
    char value[16]; // live reading, e.g. "181.4°" / "PRESSED"
    bool connected; // false: shown as N/C (the device does not answer)
};

// The two tabs of the dashboard (THERMALS is the default).
enum Tab { TAB_THERMALS = 0, TAB_SENSORS = 1 };

struct Column {
    const char *title;
    std::vector<MotorReading> motors;
};

// Field mode shown in the header: practice, waiting for a match, driver period,
// held disabled by the field (autonomous / disabled).
enum FieldMode { PRACTICE, WAITING, DRIVER, DISABLED };

struct State {
    // Top-down view of the robot. The drive columns sit on the far left / right
    // (up to 5 tiles each); the two middle columns hold up to 3 tiles each, and if
    // one of them is empty the other spans the whole middle.
    Column leftDrive{"LEFT DRIVE", {}}, rightDrive{"RIGHT DRIVE", {}};
    Column centerLeft{"", {}}, centerRight{"", {}};

    Tab tab = TAB_THERMALS;
    std::vector<SensorReading> sensors; // shown on the SENSORS tab

    int battery = 0; // percent
    FieldMode fieldMode = PRACTICE;
    bool vexnet = false;

    bool stressModalOpen = false;
    std::vector<std::string> stressedMotors; // "L P3  OVER TEMP" etc., shown in the modal

    // Optional autonomous selector: a card in the middle that opens a full-screen
    // picker. Hidden when autonNames is null.
    const std::vector<std::string> *autonNames = nullptr;
    int selectedAuton = 0;
    bool autonPickerOpen = false;
};

// Full-screen logo shown while the robot boots and calibrates.
// hint is an optional second line under the status (e.g. "Keep the robot still").
void showSplash(const char *status, const char *hint = nullptr);

// Draws the whole dashboard off-screen and sends it in one copy.
void render(const State &state);

// Touch targets (screen coordinates), with a little slop for fingers. These use
// the layout from the most recent render().
bool hitVexnetSwitch(int x, int y);
// The header's tab buttons: the tab tapped, or -1.
int hitTab(int x, int y);
bool hitStressDismiss(int x, int y);
bool hitAutonCard(int x, int y);
constexpr int PICKER_BACK = -2;
// Auton picker: the tapped routine's index, PICKER_BACK for the back button, or -1.
int hitAutonPicker(int x, int y);

} // namespace dashboard
