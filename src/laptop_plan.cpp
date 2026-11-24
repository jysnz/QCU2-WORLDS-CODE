// -----------------------------------------------------------------------------
// Brain half of the laptop's map path planner -- see include/laptop_plan.hpp.
// -----------------------------------------------------------------------------

#include "laptop_plan.hpp"
#include "canvas.hpp"
#include "driver_menu.hpp"
#include "functions.hpp"
#include "motors.hpp"
#include "path_planner.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

// -- Palette (same as the menu) --
const std::uint32_t BG = 0x0A0A0F, CARD = 0x161B22, CYAN = 0x00F0FF, ORANGE = 0xFF8C00, GREEN = 0x2ECC71;
const std::uint32_t RED = 0xFF4D4D, GRAY = 0x777777, AXIS = 0x2D2D38, FOOTER_BG = 0x18181F, WHITE = 0xFFFFFF;

enum Kind {
    TURN_HDG = 0, TURN_PT, SWING_HDG, SWING_PT, MOVE_PT, MOVE_POSE, WAIT, SET_POSE, WAIT_DONE,
    LIFT_LEVEL, CLAW, HOME_LIFT, GO_PIN, SHOOT, AUTO_GRAB, SHOOT_AT, ODOM_RESET, KIND_COUNT
};
enum Status { IDLE = 0, RUNNING = 1, DONE = 2, CANCELLED = 3 };

struct Step {
    int kind = 0;
    float x = 0, y = 0, theta = 0;
    int timeoutMs = 0;
    bool forwards = true;
    int dirIdx = 0, sideIdx = 0;
    float maxSpeed = 127, minSpeed = 0, earlyExit = 0, lead = 0.6f, drift = 0;
};

const size_t kMaxSteps = 80;
const int kMaxTrail = 120;       // points sent to the laptop (the trail is thinned to this)
const size_t kStoredTrail = 1200;

// -- shared with the link reader task --
pros::Mutex mtx;
std::vector<Step> steps;
int runRequest = -2;             // -2 none, -1 all, >= 0 that block
bool stopRequest = false, poseRequest = false, homeRequest = false;
float reqX = 0, reqY = 0, reqTheta = 0;
int driveL = 0, driveR = 0;
std::uint32_t driveAt = 0;
bool driveActive = false;

// -- run state (written by the worker, read by the screen) --
std::atomic<int> status{IDLE};
std::atomic<int> curBlock{-1};
std::atomic<int> skipped{0};
std::atomic<bool> workerExit{false};
std::atomic<bool> cancelRun{false};
std::vector<std::pair<float, float>> trail;   // guarded by mtx

lemlib::AngularDirection dirFrom(int idx) {
    switch (idx) {
    case 1: return lemlib::AngularDirection::CW_CLOCKWISE;
    case 2: return lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
    default: return lemlib::AngularDirection::AUTO;
    }
}
lemlib::DriveSide sideFrom(int idx) { return idx == 1 ? lemlib::DriveSide::RIGHT : lemlib::DriveSide::LEFT; }

// Sleeps up to ms, returning early (true) when the run is cancelled.
bool sleepOrCancel(int ms) {
    std::uint32_t end = pros::millis() + (std::uint32_t)std::max(0, ms);
    while ((std::int32_t)(end - pros::millis()) > 0) {
        if (cancelRun) return true;
        pros::delay(10);
    }
    return cancelRun;
}

// Runs one block. Motions are issued async (like the exported code); waitUntilDone blocks until they finish.
void runStep(const Step &s) {
    switch (s.kind) {
    case TURN_HDG:
        chassis.turnToHeading(s.theta, s.timeoutMs,
                              {.direction = dirFrom(s.dirIdx), .maxSpeed = (int)s.maxSpeed, .minSpeed = (int)s.minSpeed,
                               .earlyExitRange = s.earlyExit},
                              true);
        break;
    case TURN_PT:
        chassis.turnToPoint(s.x, s.y, s.timeoutMs,
                            {.forwards = s.forwards, .direction = dirFrom(s.dirIdx), .maxSpeed = (int)s.maxSpeed,
                             .minSpeed = (int)s.minSpeed, .earlyExitRange = s.earlyExit},
                            true);
        break;
    case SWING_HDG:
        chassis.swingToHeading(s.theta, sideFrom(s.sideIdx), s.timeoutMs,
                               {.direction = dirFrom(s.dirIdx), .maxSpeed = s.maxSpeed, .minSpeed = s.minSpeed,
                                .earlyExitRange = s.earlyExit},
                               true);
        break;
    case SWING_PT:
        chassis.swingToPoint(s.x, s.y, sideFrom(s.sideIdx), s.timeoutMs,
                             {.forwards = s.forwards, .direction = dirFrom(s.dirIdx), .maxSpeed = s.maxSpeed,
                              .minSpeed = s.minSpeed, .earlyExitRange = s.earlyExit},
                             true);
        break;
    case MOVE_PT:  // (the planner's slow-down ramp fields are not used on this robot)
        chassis.moveToPoint(s.x, s.y, s.timeoutMs,
                            {.forwards = s.forwards, .maxSpeed = s.maxSpeed, .minSpeed = s.minSpeed,
                             .earlyExitRange = s.earlyExit},
                            true);
        break;
    case MOVE_POSE:
        chassis.moveToPose(s.x, s.y, s.theta, s.timeoutMs,
                           {.forwards = s.forwards, .horizontalDrift = s.drift, .lead = s.lead, .maxSpeed = s.maxSpeed,
                            .minSpeed = s.minSpeed, .earlyExitRange = s.earlyExit},
                           true);
        break;
    case WAIT: sleepOrCancel(s.timeoutMs); break;
    case SET_POSE: chassis.setPose(s.x, s.y, s.theta); break;
    case WAIT_DONE: chassis.waitUntilDone(); break;
    case CLAW: setClawPiston(s.sideIdx == 0); break; // sideIdx 0 = extend (on), 1 = retract (off)
    case HOME_LIFT: homeLift(); break;
    default: skipped++; break; // hardware this robot does not have
    }
}

// The plan runner: waits for a run request, then goes through the blocks. Lives as long as the screen.
void workerTask(void *) {
    while (!workerExit) {
        int req;
        std::vector<Step> plan;
        mtx.take();
        req = runRequest;
        if (req != -2) {
            runRequest = -2;
            plan = steps;
            trail.clear();
        }
        mtx.give();
        if (req == -2) {
            pros::delay(10);
            continue;
        }
        cancelRun = false;
        skipped = 0;
        status = RUNNING;
        int first = req < 0 ? 0 : req, last = req < 0 ? (int)plan.size() - 1 : std::min(req, (int)plan.size() - 1);
        bool cancelled = false;
        for (int i = first; i <= last && i < (int)plan.size(); i++) {
            if (cancelRun || workerExit) { cancelled = true; break; }
            curBlock = i;
            runStep(plan[i]);
        }
        if (cancelRun || workerExit) cancelled = true;
        if (cancelled) chassis.cancelAllMotions();
        status = cancelled ? CANCELLED : DONE;
    }
}

// -- drawing --
Canvas frame;
const int kBackX0 = 396, kBackY0 = 14, kBackX1 = 468, kBackY1 = 40;
const int kEdX0 = 15, kEdY0 = 168, kEdX1 = 205, kEdY1 = 206;
bool inBox(int x, int y, int x0, int y0, int x1, int y1) { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }

const char *statusName(int s) {
    switch (s) {
    case RUNNING: return "RUNNING";
    case DONE: return "DONE";
    case CANCELLED: return "CANCELLED";
    default: return "IDLE";
    }
}

void draw(int count, float x, float y, float th) {
    frame.fillRect(0, 0, Canvas::W, Canvas::H, BG);
    frame.fillRoundRect(5, 5, 475, 50, 8, ORANGE);
    frame.text(FONT_MEDIUM, 20, 9, "PATH PLANNER", 0x000000);
    frame.text(FONT_SMALL, 20, 31, "map planner on the laptop", 0x1A1A1A);
    frame.fillRoundRect(kBackX0, kBackY0, kBackX1, kBackY1, 6, BG);
    frame.text(FONT_SMALL, kBackX0 + 12, kBackY0 + 7, "< BACK", 0xCCCCCC);

    int st = status;
    frame.card(15, 58, 465, 160, 8, CARD, AXIS);
    frame.text(FONT_SMALL, 27, 66, "Plan on the robot", GRAY);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "%s", statusName(st));
    frame.text(FONT_LARGE, 27, 82, buf, st == RUNNING ? GREEN : (st == CANCELLED ? RED : WHITE));
    int blk = curBlock;
    std::snprintf(buf, sizeof(buf), "%d block%s", count, count == 1 ? "" : "s");
    frame.text(FONT_SMALL, 300, 70, buf, WHITE);
    if (st == RUNNING && blk >= 0) {
        std::snprintf(buf, sizeof(buf), "now: block %d of %d", blk + 1, count);
        frame.text(FONT_SMALL, 300, 90, buf, GREEN);
    }
    std::snprintf(buf, sizeof(buf), "x %.1f   y %.1f   heading %.0f", x, y, th);
    frame.text(FONT_SMALL, 27, 130, buf, CYAN);

    frame.fillRoundRect(kEdX0, kEdY0, kEdX1, kEdY1, 6, AXIS);
    frame.text(FONT_SMALL, (kEdX0 + kEdX1) / 2, kEdY0 + 13, "BRAIN EDITOR", WHITE, Canvas::CENTER);
    frame.text(FONT_SMALL, 215, 180, "the older editor on this screen", GRAY);

    frame.fillRect(0, 218, Canvas::W, Canvas::H, FOOTER_BG);
    int sk = skipped;
    if (sk > 0) {
        std::snprintf(buf, sizeof(buf), "%d block(s) skipped: not available on this robot", sk);
        frame.text(FONT_SMALL, 10, 223, buf, ORANGE);
    } else {
        frame.text(FONT_SMALL, 10, 223, "Plan on the laptop window; Run all sends the route and drives it.", GRAY);
    }
    frame.present();
}

} // namespace

// --- Public ---------------------------------------------------------------------------------------------
void laptopPlanHandleLine(const char *line) {
    if (std::strncmp(line, "PLAN ", 5) == 0) {
        const char *rest = line + 5;
        if (std::strncmp(rest, "CLEAR", 5) == 0) {
            mtx.take();
            if (status != RUNNING) {
                steps.clear();
                status = IDLE;
                curBlock = -1;
            }
            mtx.give();
        } else if (std::strncmp(rest, "ADD ", 4) == 0) {
            Step s;
            int k, fwd;
            if (std::sscanf(rest + 4, "%d %f %f %f %d %d %d %d %f %f %f %f %f", &k, &s.x, &s.y, &s.theta, &s.timeoutMs, &fwd,
                            &s.dirIdx, &s.sideIdx, &s.maxSpeed, &s.minSpeed, &s.earlyExit, &s.lead, &s.drift) == 13) {
                s.kind = std::clamp(k, 0, (int)KIND_COUNT - 1);
                s.forwards = fwd != 0;
                mtx.take();
                if (steps.size() < kMaxSteps && status != RUNNING) steps.push_back(s);
                mtx.give();
            }
        } else if (std::strncmp(rest, "RUN", 3) == 0) {
            int only = -1;
            std::sscanf(rest, "RUN %d", &only);
            mtx.take();
            runRequest = only;
            mtx.give();
        } else if (std::strncmp(rest, "STOP", 4) == 0) {
            mtx.take();
            stopRequest = true;
            mtx.give();
        }
    } else if (std::strncmp(line, "POSE ", 5) == 0) {
        float x, y, t;
        if (std::sscanf(line + 5, "%f %f %f", &x, &y, &t) == 3) {
            mtx.take();
            poseRequest = true;
            reqX = x;
            reqY = y;
            reqTheta = t;
            mtx.give();
        }
    } else if (std::strncmp(line, "DRIVE ", 6) == 0) {
        int l, r;
        if (std::sscanf(line + 6, "%d %d", &l, &r) == 2) {
            mtx.take();
            driveL = std::clamp(l, -127, 127);
            driveR = std::clamp(r, -127, 127);
            driveAt = pros::millis();
            driveActive = true;
            mtx.give();
        }
    } else if (std::strncmp(line, "ROBOT HOME", 10) == 0) {
        mtx.take();
        homeRequest = true;
        mtx.give();
    }
    // NOP, LOG NEED ... and anything else: nothing to do.
}

void laptopPlanScreen() {
    workerExit = false;
    cancelRun = false;
    mtx.take();
    runRequest = -2;
    stopRequest = poseRequest = homeRequest = driveActive = false;
    trail.clear();
    mtx.give();
    pros::Task worker(workerTask, nullptr, "laptop plan");

    std::uint32_t lastDraw = 0, lastTrail = 0;
    bool wasTouched = true;
    while (true) {
        std::uint32_t now = pros::millis();
        if (driverMenuLeaveRequested()) break;

        // -- screen taps (brain or laptop) --
        pros::screen_touch_status_s_t t = pros::screen::touch_status();
        bool down = t.touch_status != pros::E_TOUCH_RELEASED;
        bool tap = down && !wasTouched;
        wasTouched = down;
        int tx = t.x, ty = t.y, rx, ry;
        if (takeRemoteTouch(rx, ry)) {
            tap = true;
            tx = rx;
            ty = ry;
        }
        if (tap && inBox(tx, ty, kBackX0, kBackY0, kBackX1, kBackY1)) break;
        if (tap && inBox(tx, ty, kEdX0, kEdY0, kEdX1, kEdY1)) {
            pathPlannerControl(); // the older on-brain editor; returns on its own BACK
            while (pros::screen::touch_status().touch_status != pros::E_TOUCH_RELEASED) pros::delay(20);
            wasTouched = true;
        }

        // -- requests from the laptop --
        bool stop, doPose, doHome;
        float px, py, pt;
        int dl, dr;
        bool drive;
        mtx.take();
        stop = stopRequest;
        stopRequest = false;
        doPose = poseRequest;
        poseRequest = false;
        px = reqX;
        py = reqY;
        pt = reqTheta;
        doHome = homeRequest;
        homeRequest = false;
        drive = driveActive && now - driveAt < 400;
        if (driveActive && !drive) driveActive = false;
        dl = driveL;
        dr = driveR;
        mtx.give();

        bool running = status == RUNNING;
        if (stop && running) {
            cancelRun = true;
            chassis.cancelAllMotions();
        }
        if (doPose && !running) {
            chassis.setPose(px, py, pt);
            mtx.take();
            trail.clear();
            mtx.give();
        }
        if (doHome && !running) {
            setClawPiston(true);
            homeLift(); // blocks for a second or two
        }
        if (!running) {
            static bool drivingNow = false;
            if (drive) {
                left_motor_group.move(dl);
                right_motor_group.move(dr);
                drivingNow = true;
            } else if (drivingNow) {
                left_motor_group.move(0);
                right_motor_group.move(0);
                drivingNow = false;
            }
        }

        // -- pose + path --
        lemlib::Pose pose = chassis.getPose();
        if (status == RUNNING && now - lastTrail >= 100) {
            lastTrail = now;
            mtx.take();
            if (trail.size() < kStoredTrail) trail.push_back({pose.x, pose.y});
            mtx.give();
        }

        if (now - lastDraw >= 100) {
            lastDraw = now;
            int count;
            static char line[3600];
            mtx.take();
            count = (int)steps.size();
            int n = (int)trail.size();
            int stride = std::max(1, (n + kMaxTrail - 1) / kMaxTrail);
            int pos = std::snprintf(line, sizeof(line), "RUI|PLANNER|%d|%d|%d|%.2f|%.2f|%.2f|", (int)status, (int)curBlock, count,
                                    pose.x, pose.y, pose.theta);
            for (int i = 0; i < n && pos < (int)sizeof(line) - 24; i += stride)
                pos += std::snprintf(line + pos, sizeof(line) - pos, "%s%.1f,%.1f", i ? ";" : "", trail[i].first, trail[i].second);
            mtx.give();
            driverMenuPublish(line);
            draw(count, pose.x, pose.y, pose.theta);
        }
        pros::delay(20);
    }

    // leaving: stop everything the screen started
    cancelRun = true;
    workerExit = true;
    chassis.cancelAllMotions();
    left_motor_group.move(0);
    right_motor_group.move(0);
    worker.join();
    status = IDLE;
    curBlock = -1;
}
