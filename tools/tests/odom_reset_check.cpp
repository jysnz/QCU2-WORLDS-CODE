// Host check of include/odom_reset_math.hpp:  g++ -std=c++17 -Iinclude tools/tests/odom_reset_check.cpp && ./a.out
#include "odom_reset_math.hpp"
#include <cstdio>
using namespace odomreset;

static int fails = 0;
static void expect(bool ok, const char *what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        fails++;
    }
}
static bool near(float a, float b) { return std::fabs(a - b) < 0.05f; }

int main() {
    // facing +Y (heading 0) at x=-10, y=40: front wall is 72-40 = 32 in away from the centre;
    // sensor face 7.5 in out, so it reads 24.5 in = 622 mm
    Reading front = {true, 622, 0, 7.5f};
    Result r = correct(-10, 39, 0, &front, 1, 4);
    expect(r.yOk && near(r.y, 40.0f), "front wall gives y");
    expect(!r.xOk && near(r.x, -10), "front alone leaves x");

    // left + right sensors at heading 0: left wall x=-72, right wall x=+72, robot at x=3
    Reading lr[2] = {{true, (int)((72 + 3 - 7.5f) * 25.4f), -90, 7.5f}, {true, (int)((72 - 3 - 7.5f) * 25.4f), 90, 7.5f}};
    r = correct(2.0f, 0, 0, lr, 2, 4);
    expect(r.xOk && near(r.x, 3.0f), "left+right average gives x");

    // heading 90 (facing +X): front sensor looks at the +X wall
    r = correct(39, 0, 90, &front, 1, 4);
    expect(r.xOk && near(r.x, 72 - 24.5f - 7.5f + 0), "front at heading 90 gives x"); // 72-(24.5+7.5)=40
    expect(near(r.x, 40.0f), "x = 40");

    // heading 180: front looks at the -Y wall
    r = correct(0, -38, 180, &front, 1, 4);
    expect(r.yOk && near(r.y, -72 + 32), "front at heading 180 gives y");

    // too far from odometry -> rejected (obstacle in view)
    r = correct(-10, 30, 0, &front, 1, 4);
    expect(!r.yOk && r.rejected == 1, "reading 10 in off odometry is ignored");

    // invalid / unused
    Reading bad = {true, -1, 0, 7.5f}, off = {false, 622, 0, 7.5f};
    r = correct(0, 39, 0, &bad, 1, 4);
    expect(!r.yOk && r.rejected == 0, "invalid reading is skipped");
    r = correct(0, 39, 0, &off, 1, 4);
    expect(!r.yOk, "unused sensor is skipped");

    // 5 degrees of tilt: perpendicular distance is d*cos(5 deg)
    r = correct(0, 39, 5, &front, 1, 4);
    expect(r.yOk && near(r.y, 72 - 32.0f * std::cos(5 * 3.14159265f / 180)), "tilt is corrected");

    // heading: robot tilted 4 deg in the middle; both side sensors measure along their tilted rays
    {
        float tilt = 4 * 3.14159265f / 180;
        float span = 144.0f / std::cos(tilt);               // total ray length, both ways
        int mm = (int)((span / 2 - 7.5f) * 25.4f);
        Reading s2[2] = {{true, mm, -90, 7.5f}, {true, mm, 90, 7.5f}};
        r = correct(0, 0, 4, s2, 2, 4, true);
        expect(r.headingOk && std::fabs(r.theta - 4.0f) < 0.5f, "heading from span, sign from odometry");
        r = correct(0, 0, -4, s2, 2, 4, true);
        expect(r.headingOk && std::fabs(r.theta + 4.0f) < 0.5f, "negative tilt keeps its sign");
        r = correct(0, 0, 4, s2, 2, 4, false);
        expect(!r.headingOk && near(r.theta, 4.0f), "heading untouched when not asked");
        Reading blocked[2] = {{true, 300, -90, 7.5f}, {true, mm, 90, 7.5f}};
        r = correct(0, 0, 4, blocked, 2, 4, true);
        expect(!r.headingOk, "obstacle on one side: no heading fix");
    }

    // gating
    expect(headingAligned(8) && headingAligned(-91) && headingAligned(182), "aligned headings");
    expect(!headingAligned(20) && !headingAligned(135), "unaligned headings");

    if (!fails)
        printf("odom_reset_check: all passed\n");
    return fails;
}
