#include "screen_task.hpp"
#include "fieldctl/fieldctl.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>

namespace dashboard {

std::vector<MotorReading> readMotors(const pros::AbstractMotor &motors, const char *prefix) {
    std::vector<double> temps = motors.get_temperature_all();
    std::vector<std::int8_t> ports = motors.get_port_all();
    std::vector<std::int32_t> overTemp = motors.is_over_temp_all();
    std::vector<std::int32_t> overCurrent = motors.is_over_current_all();
    std::vector<MotorReading> out;
    for (size_t i = 0; i < temps.size() && i < ports.size(); i++) {
        MotorReading r;
        int port = std::abs((int)ports[i]);
        snprintf(r.label, sizeof(r.label), "P%d", port);
        snprintf(r.name, sizeof(r.name), "%s P%d", prefix, port);
        r.temp = temps[i];
        r.connected = std::isfinite(temps[i]);
        // An unplugged motor answers these with PROS_ERR (a large positive number),
        // so only an exact 1 from a motor that is actually reporting counts.
        r.overTemp = r.connected && i < overTemp.size() && overTemp[i] == 1;
        r.atCurrentLimit = r.connected && i < overCurrent.size() && overCurrent[i] == 1;
        out.push_back(r);
    }
    return out;
}

MotorReading readMotor(const pros::AbstractMotor &motor, const char *label, const char *name) {
    std::vector<MotorReading> one = readMotors(motor, name);
    MotorReading r = one.empty() ? MotorReading{} : one[0];
    int port = one.empty() ? 0 : std::abs((int)motor.get_port_all()[0]);
    snprintf(r.label, sizeof(r.label), "%s P%d", label, port);
    if (one.empty()) {
        snprintf(r.name, sizeof(r.name), "%s", name);
        r.temp = INFINITY;
    }
    return r;
}

void startScreenTask(ScreenConfig config) {
    static ScreenConfig cfg;
    cfg = std::move(config);

    static pros::Task screen_task([]() {
        // ── Motor stress alert ──
        // "Stressed" = the motor itself is reporting over-temperature or
        // over-current, straight from the V5 firmware -- not just our own display
        // thresholds. The alert pops up over the dashboard; tapping its button
        // hides it, but only for STRESS_MODAL_COOLDOWN_MS -- if the motor is still
        // stressed once that expires, it comes back. Clearing the stress condition
        // entirely resets the dismissal so the next stress event alerts right away.
        const uint32_t STRESS_MODAL_COOLDOWN_MS = 60000; // 1 minute
        bool stressModalOpen = false;
        bool stressModalDismissed = false;
        uint32_t stressDismissedAtMs = 0;

        // Over-current only counts once it has lasted OVER_CURRENT_SUSTAIN_MS.
        // The flag just means "at the current limit", which every motor hits for
        // a moment when accelerating from a stop (and sooner when many motors
        // share the brain's power budget) -- a jammed or stalled mechanism stays there.
        const uint32_t OVER_CURRENT_SUSTAIN_MS = 3000;
        std::map<std::string, uint32_t> overCurrentSinceMs; // by motor name

        bool autonPickerOpen = false;

        // ── Dirty-state tracking: only build and send a frame when something visible changed ──
        bool forceRedraw = true;
        int lastBattery = INT_MIN;
        int lastFieldMode = -1;
        bool lastVexnet = !fieldctl::vexnet();
        bool lastStressModalOpen = false;
        bool lastPickerOpen = false;
        int lastAuton = -1;
        std::string lastStressKey;
        std::vector<int> lastTemps; // rounded to nearest 0.5C so tiny sensor jitter doesn't trigger redraws
        uint32_t lastTelemetryRedraw = 0;
        const uint32_t TELEMETRY_REDRAW_INTERVAL_MS = 250;

        while (true) {
            if (cfg.screenBusy()) {
                pros::delay(100);
                forceRedraw = true;         // repaint fully once we get control back
                overCurrentSinceMs.clear(); // not sampling now, so restart the timers when we resume
                continue;
            }

            pros::screen_touch_status_s_t touch = pros::screen::touch_status();
            uint32_t nowMs = pros::millis();
            bool hasAutons = cfg.autonNames && cfg.selectedAuton && !cfg.autonNames->empty();

            // ── Input Detection ──
            static bool wasTouched = false;
            if (touch.touch_status == pros::E_TOUCH_PRESSED && !wasTouched) {
                if (stressModalOpen) {
                    // The alert owns all touch input while it's up -- only its button does anything.
                    if (hitStressDismiss(touch.x, touch.y)) {
                        stressModalOpen = false;
                        stressModalDismissed = true;
                        stressDismissedAtMs = nowMs;
                    }
                } else if (autonPickerOpen) {
                    int hit = hitAutonPicker(touch.x, touch.y);
                    if (hit >= 0) {
                        *cfg.selectedAuton = hit;
                        cfg.onAutonSelected();
                    }
                    if (hit != -1) autonPickerOpen = false;
                } else if (hitVexnetSwitch(touch.x, touch.y)) {
                    // VEXNET switch: on = robot waits disabled for the VEXNET gamemaster,
                    // off = normal driving (practice).
                    fieldctl::setVexnet(!fieldctl::vexnet());
                } else if (hasAutons && hitAutonCard(touch.x, touch.y)) {
                    autonPickerOpen = true;
                }
                wasTouched = true;
            } else if (touch.touch_status == pros::E_TOUCH_RELEASED) {
                wasTouched = false;
            }

            // ── Gather live data once per frame ──
            State state;
            cfg.readMotors(state);
            state.battery = (int)pros::battery::get_capacity();
            state.vexnet = fieldctl::vexnet();
            state.fieldMode = !state.vexnet               ? PRACTICE
                              : !fieldctl::engaged()      ? WAITING
                              : fieldctl::driverAllowed() ? DRIVER
                                                          : DISABLED;
            if (hasAutons) {
                state.autonNames = cfg.autonNames;
                state.selectedAuton = std::max(0, std::min(*cfg.selectedAuton, (int)cfg.autonNames->size() - 1));
                state.autonPickerOpen = autonPickerOpen;
            }
            const Column *columns[] = {&state.leftDrive, &state.rightDrive, &state.centerLeft, &state.centerRight};

            // ── Motor stress detection ──
            for (const Column *col : columns) {
                for (const MotorReading &m : col->motors) {
                    bool overC = false;
                    if (!m.atCurrentLimit) {
                        overCurrentSinceMs.erase(m.name);
                    } else {
                        uint32_t since = overCurrentSinceMs.emplace(m.name, nowMs).first->second;
                        overC = nowMs - since >= OVER_CURRENT_SUSTAIN_MS;
                    }
                    if (!m.overTemp && !overC) continue;
                    char buf[40];
                    snprintf(buf, sizeof(buf), "%s  %s", m.name,
                             m.overTemp && overC ? "OVER TEMP + CURRENT" : (m.overTemp ? "OVER TEMP" : "OVER CURRENT"));
                    state.stressedMotors.push_back(buf);
                }
            }
            bool isStressed = !state.stressedMotors.empty();

            // Cooldown elapsed? Make the alert eligible to pop up again.
            if (stressModalDismissed && nowMs - stressDismissedAtMs >= STRESS_MODAL_COOLDOWN_MS) {
                stressModalDismissed = false;
            }
            if (!isStressed) {
                // Condition cleared -- hide the alert and forget any pending
                // dismissal so the next stress event alerts right away.
                stressModalOpen = false;
                stressModalDismissed = false;
            } else if (!stressModalDismissed) {
                stressModalOpen = true;
            }
            state.stressModalOpen = stressModalOpen;

            std::string stressKey;
            for (const std::string &s : state.stressedMotors) stressKey += s + "|";

            // Rounded snapshot of every motor temp for change detection
            std::vector<int> tempsNow;
            for (const Column *col : columns)
                for (const MotorReading &m : col->motors) tempsNow.push_back(m.connected ? (int)(m.temp * 2) : -1); // 0.5C resolution

            // Touch-driven changes (VEXNET switch, alert, auton picker) and field mode
            // changes redraw instantly. Live telemetry (motor temps, battery) is
            // throttled to a few times a second -- while driving it changes almost every loop.
            bool interactiveChanged = forceRedraw || state.vexnet != lastVexnet || state.fieldMode != lastFieldMode ||
                                      stressModalOpen != lastStressModalOpen || stressKey != lastStressKey ||
                                      autonPickerOpen != lastPickerOpen || state.selectedAuton != lastAuton;
            bool telemetryChanged = state.battery != lastBattery || tempsNow != lastTemps;
            bool telemetryDue = telemetryChanged && (nowMs - lastTelemetryRedraw >= TELEMETRY_REDRAW_INTERVAL_MS);

            if (!interactiveChanged && !telemetryDue) {
                pros::delay(35);
                continue;
            }
            if (telemetryChanged) lastTelemetryRedraw = nowMs;

            render(state);

            // Update dirty-tracking snapshot
            lastBattery = state.battery;
            lastTemps = tempsNow;
            lastFieldMode = state.fieldMode;
            lastVexnet = state.vexnet;
            lastStressModalOpen = stressModalOpen;
            lastStressKey = stressKey;
            lastPickerOpen = autonPickerOpen;
            lastAuton = state.selectedAuton;
            forceRedraw = false;

            pros::delay(35);
        }
    });
}

} // namespace dashboard
