// Runs SensorFilters.h against simulated VL53L0X readings on a PC (no hardware needed):
//
//     g++ -std=c++17 -I.. test_filters.cpp -o test_filters && ./test_filters
//
// Noise is made-up but sensor-like: about 3 mm of scatter plus the odd wild reading.
#include "SensorFilters.h"

#include <math.h>
#include <stdio.h>
#include <vector>

static uint32_t rngState = 12345;
static float uniform() {  // 0..1
  rngState = rngState * 1664525u + 1013904223u;
  return (rngState >> 8) / 16777216.0f;
}
static float gauss() {  // roughly N(0, 1)
  return (uniform() + uniform() + uniform() + uniform() - 2.0f) * 1.732f;
}

static int failures = 0;
static void check(const char *name, bool ok, const char *detail) {
  printf("%s  %s   %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok) failures++;
}

struct Stats {
  double sum = 0, sq = 0;
  int n = 0;
  float worst = 0;
  void add(float err) {
    sum += err;
    sq += err * err;
    n++;
    if (fabsf(err) > worst) worst = fabsf(err);
  }
  float rms() const { return n ? sqrtf((float)(sq / n)) : 0; }
};

int main() {
  const float dt = 0.05f;  // 20 samples a second

  // 1. A target that does not move: scatter + 5 % spikes.
  {
    DistanceFilter f;
    Stats raw, out;
    for (int i = 0; i < 600; i++) {
      float truth = 500;
      float z = truth + 3.0f * gauss();
      if (uniform() < 0.05f) z += (uniform() < 0.5f ? -1 : 1) * (80 + 220 * uniform());  // spike
      int y = f.update(true, z, dt);
      if (i >= 40) {  // after warm-up
        raw.add(z - truth);
        out.add((float)y - truth);
      }
    }
    char d[160];
    snprintf(d, sizeof d, "raw rms %.1f mm (worst %.0f)  ->  filtered rms %.2f mm (worst %.1f)", raw.rms(), raw.worst,
             out.rms(), out.worst);
    check("still target: spikes removed, scatter cut", out.rms() < 2.0f && out.worst < 8.0f, d);
  }

  // 2. A step: the target jumps 300 -> 700 mm. It has to follow, not ignore it.
  {
    DistanceFilter f;
    int settledAt = -1;
    for (int i = 0; i < 200; i++) {
      float truth = i < 80 ? 300.0f : 700.0f;
      int y = f.update(true, truth + 3.0f * gauss(), dt);
      if (i >= 80 && settledAt < 0 && fabsf((float)y - 700) < 15) settledAt = i - 80;
    }
    char d[100];
    snprintf(d, sizeof d, "within 15 mm of the new distance after %d samples (%.2f s)", settledAt, settledAt * dt);
    check("step 300 -> 700 mm is followed", settledAt >= 0 && settledAt <= 12, d);
  }

  // 3. A target moving steadily (the robot driving at 150 mm/s): the lag must stay small.
  {
    DistanceFilter f;
    Stats lag;
    for (int i = 0; i < 400; i++) {
      float truth = 200 + 150.0f * i * dt;
      int y = f.update(true, truth + 3.0f * gauss(), dt);
      if (i >= 60) lag.add((float)y - truth);
    }
    char d[100];
    snprintf(d, sizeof d, "mean error %.1f mm, worst %.1f mm at 150 mm/s", (float)(lag.sum / lag.n), lag.worst);
    check("moving target: little lag", fabsf((float)(lag.sum / lag.n)) < 6.0f && lag.worst < 25.0f, d);
  }

  // 4. Two wild readings in a row must not move the output; a lasting change must.
  {
    DistanceFilter f;
    for (int i = 0; i < 60; i++) f.update(true, 400 + 2.0f * gauss(), dt);
    int a = f.update(true, 900, dt);
    int b = f.update(true, 900, dt);
    char d[100];
    snprintf(d, sizeof d, "outputs during a 2-sample burst: %d, %d (truth 400)", a, b);
    check("2-sample burst ignored", abs(a - 400) < 10 && abs(b - 400) < 10, d);
  }

  // 5. Dropouts: a few missed samples are coasted over, a long gap reports NO_TARGET, then it recovers.
  {
    DistanceFilter f;
    for (int i = 0; i < 60; i++) f.update(true, 350 + 2.0f * gauss(), dt);
    int during = 0;
    for (int i = 0; i < 3; i++) during = f.update(false, 0, dt);
    bool shortOk = abs(during - 350) < 10;
    int gone = 0;
    for (int i = 0; i < 8; i++) gone = f.update(false, 0, dt);
    bool goneOk = gone == DistanceFilter::NO_TARGET;
    int back = 0;
    for (int i = 0; i < 12; i++) back = f.update(true, 600 + 2.0f * gauss(), dt);
    char d[140];
    snprintf(d, sizeof d, "3 missed -> %d (350); 11 missed -> %d (-1); recovers at %d (600)", during, gone, back);
    check("dropouts: coast, then NO_TARGET, then recover", shortOk && goneOk && abs(back - 600) < 15, d);
  }

  // 6. The median stage on its own: a lone spike never gets through.
  {
    MedianFilter<5> m;
    float seq[] = {100, 101, 99, 5000, 100, 102};
    float worst = 0;
    for (float v : seq) {
      m.push(v);
      if (m.count() >= 3 && fabsf(m.median() - 100) > worst) worst = fabsf(m.median() - 100);
    }
    char d[80];
    snprintf(d, sizeof d, "median stayed within %.0f mm of 100 through a 5000 mm spike", worst);
    check("median rejects a single spike", worst <= 2.0f, d);
  }

  // 7. The adaptive noise: a noisy sensor makes the filter trust it less than a calm one.
  {
    DistanceFilter calm, noisy;
    for (int i = 0; i < 300; i++) {
      calm.update(true, 800 + 2.0f * gauss(), dt);
      noisy.update(true, 800 + 25.0f * gauss(), dt);
    }
    char d[100];
    snprintf(d, sizeof d, "assumed noise: calm %.1f mm, noisy %.1f mm", calm.noiseMm(), noisy.noiseMm());
    check("measurement noise adapts to the scatter", noisy.noiseMm() > calm.noiseMm() + 2.0f, d);
  }

  printf("\n%s\n", failures ? "SOME CHECKS FAILED" : "all checks passed");
  return failures ? 1 : 0;
}
