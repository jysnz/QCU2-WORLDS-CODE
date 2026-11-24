// Filters for the VL53L0X distance readings.
//
//   raw reading -> MedianFilter (throws away spikes) -> AdaptiveKalman (smooths, tracks motion)
//                                                    -> delay compensation (undoes the median's lag)
//
// Header only and free of Arduino calls, so it also compiles on a PC: see extras/test_filters.cpp.
// All distances are millimetres, all times seconds.
#pragma once

#include <math.h>
#include <stdint.h>

// --- Median filter -----------------------------------------------------------
// Keeps the last N readings and returns their median. A single wild reading (a spike) never survives
// it, whatever its size, while a real change comes through after about N/2 samples.
template <int N>
class MedianFilter {
  static_assert(N >= 1 && N % 2 == 1, "use an odd window size");

 public:
  void reset() {
    count_ = 0;
    head_ = 0;
  }

  void push(float v) {
    buf_[head_] = v;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
  }

  int count() const { return count_; }

  // Median of what has arrived so far (an even count takes the mean of the two middle values).
  float median() const {
    float s[N];
    for (int i = 0; i < count_; i++) s[i] = buf_[i];
    for (int i = 1; i < count_; i++) {  // insertion sort: N is tiny
      float v = s[i];
      int j = i - 1;
      while (j >= 0 && s[j] > v) {
        s[j + 1] = s[j];
        j--;
      }
      s[j + 1] = v;
    }
    if (count_ == 0) return 0;
    if (count_ % 2 == 1) return s[count_ / 2];
    return 0.5f * (s[count_ / 2 - 1] + s[count_ / 2]);
  }

 private:
  float buf_[N] = {};
  int count_ = 0;
  int head_ = 0;
};

// --- Adaptive Kalman filter -------------------------------------------------
// Two states, distance and its rate of change (constant-velocity model), one measurement (distance).
// What makes it "advanced" rather than a plain smoother:
//
//   * Measurement noise follows the distance. The VL53L0X gets noisier the further the target is, so the
//     prior sigma is  noiseBase + noisePerMm * distance.
//   * It also adapts to what it actually sees (innovation-based adaptive estimation): when the readings
//     scatter more than the filter expects, the measurement noise rises and the filter trusts the sensor
//     less; when they are calm, it falls back to the prior. It never drops below the prior.
//   * Outlier gating: a reading further than gateSigma standard deviations from the prediction is not
//     used (the filter coasts on its prediction instead).
//   * Step detection: if resetAfterOutliers readings in a row are rejected, the target really moved (a
//     new object, the robot was bumped). The filter restarts at the new distance instead of ignoring it.
//   * Variable time step: it uses the real time between readings, so a late frame does not skew it.
struct KalmanConfig {
  float accelSigma = 600.0f;      // mm/s^2   how hard the distance may speed up or slow down (process noise)
  float noiseBase = 2.0f;         // mm       measurement noise (1 sigma) at zero distance
  float noisePerMm = 0.007f;      // mm/mm    extra noise per mm of distance (about 1 % of the distance)
  float noiseMax = 40.0f;         // mm       ceiling for the adapted measurement noise
  float adaptRate = 0.08f;        // 0..1     how quickly the noise estimate follows the innovations
  float gateSigma = 3.5f;         // sigma    readings further out than this are outliers
  int resetAfterOutliers = 3;     // in a row = a real step: restart at the new value
  float initialVelocitySigma = 200.0f;  // mm/s   uncertainty about the speed right after a (re)start
  float maxDt = 0.5f;             // s        a longer gap than this is treated as this long
  float lagCompDeadband = 15.0f;  // mm/s     speeds below this are not used to undo the median's lag (it is noise)
};

class AdaptiveKalman {
 public:
  explicit AdaptiveKalman(const KalmanConfig &c = KalmanConfig()) : cfg_(c) {}

  bool started() const { return started_; }
  float distance() const { return d_; }
  float velocity() const { return v_; }
  // The measurement noise (1 sigma, mm) the filter is currently assuming.
  float noise() const { return sqrtf(r_); }

  // (Re)start at distance z, speed 0.
  void reset(float z) {
    started_ = true;
    d_ = z;
    v_ = 0;
    float s = priorSigma(z);
    r_ = s * s;
    innovationVar_ = r_ + s * s;  // what S would be right after a restart
    p00_ = r_;
    p01_ = 0;
    p11_ = cfg_.initialVelocitySigma * cfg_.initialVelocitySigma;
    outliers_ = 0;
  }

  void stop() { started_ = false; }

  // No reading this time: predict only, so the estimate keeps moving at its current speed.
  void coast(float dt) {
    if (started_) predict(clampDt(dt));
  }

  // Predict, then correct with reading z. Returns false if z was rejected as an outlier (the estimate
  // is then just the prediction); true if it was used, including when it restarted the filter.
  bool update(float z, float dt) {
    if (!started_) {
      reset(z);
      return true;
    }
    dt = clampDt(dt);
    predict(dt);

    float y = z - d_;                       // innovation
    float s = p00_ + r_;                    // its predicted variance
    if (y * y > cfg_.gateSigma * cfg_.gateSigma * s) {
      if (++outliers_ >= cfg_.resetAfterOutliers) {
        reset(z);                           // not noise: the distance really jumped
        return true;
      }
      return false;
    }
    outliers_ = 0;

    // correct
    float k0 = p00_ / s, k1 = p01_ / s;
    d_ += k0 * y;
    v_ += k1 * y;
    p11_ -= k1 * p01_;
    p00_ *= (1.0f - k0);
    p01_ *= (1.0f - k0);

    // adapt the measurement noise to the innovations seen so far
    innovationVar_ = (1.0f - cfg_.adaptRate) * innovationVar_ + cfg_.adaptRate * y * y;
    float prior = priorSigma(d_);
    float want = innovationVar_ - (s - r_);  // observed innovation variance minus the state's part
    float lo = prior * prior, hi = cfg_.noiseMax * cfg_.noiseMax;
    r_ = want < lo ? lo : (want > hi ? hi : want);
    return true;
  }

 private:
  float priorSigma(float d) const {
    float s = cfg_.noiseBase + cfg_.noisePerMm * (d > 0 ? d : 0);
    return s < 0.5f ? 0.5f : s;
  }

  float clampDt(float dt) const {
    if (!(dt > 0.0005f)) return 0.0005f;  // also catches NaN
    return dt > cfg_.maxDt ? cfg_.maxDt : dt;
  }

  void predict(float dt) {
    d_ += v_ * dt;
    float q = cfg_.accelSigma * cfg_.accelSigma;
    float dt2 = dt * dt;
    float p00 = p00_ + 2.0f * dt * p01_ + dt2 * p11_ + q * dt2 * dt2 * 0.25f;
    float p01 = p01_ + dt * p11_ + q * dt2 * dt * 0.5f;
    float p11 = p11_ + q * dt2;
    p00_ = p00;
    p01_ = p01;
    p11_ = p11;
  }

  KalmanConfig cfg_;
  bool started_ = false;
  float d_ = 0, v_ = 0;
  float p00_ = 0, p01_ = 0, p11_ = 0;  // covariance of (distance, velocity)
  float r_ = 0;                        // measurement noise variance in use
  float innovationVar_ = 0;            // running estimate of the innovation variance
  int outliers_ = 0;
};

// --- The whole chain for one sensor -----------------------------------------
// Feed it every sample; it returns the filtered distance in mm, or NO_TARGET when there is nothing
// usable to report.
class DistanceFilter {
 public:
  static const int NO_TARGET = -1;

  explicit DistanceFilter(const KalmanConfig &c = KalmanConfig(), int holdSamples = 4)
      : kf_(c), hold_(holdSamples), deadband_(c.lagCompDeadband) {}

  void reset() {
    med_.reset();
    kf_.stop();
    misses_ = 0;
    lastMedian_ = NO_TARGET;
  }

  // valid: the sensor measured something this time (range status OK). rawMm: the reading. dt: seconds
  // since the previous sample of this sensor.
  int update(bool valid, float rawMm, float dt) {
    if (!valid) {
      // A few missed samples are coasted over; more than that means there is nothing there.
      if (++misses_ > hold_ || !kf_.started()) {
        reset();
        return NO_TARGET;
      }
      kf_.coast(dt);
      return output();
    }
    misses_ = 0;
    med_.push(rawMm);
    lastMedian_ = (int)lroundf(med_.median());
    kf_.update(med_.median(), dt);
    lastDt_ = dt;
    return output();
  }

  int medianMm() const { return lastMedian_; }          // after the median stage, before the Kalman filter
  float velocityMmS() const { return kf_.velocity(); }
  float noiseMm() const { return kf_.noise(); }

 private:
  // The median of N samples describes the target (N-1)/2 samples ago. On a moving target that is a
  // constant lag of speed x delay, so add it back using the Kalman filter's own speed estimate. On a
  // still target the speed is about 0 and this changes nothing.
  int output() const {
    float delay = 0.5f * (float)(med_.count() - 1) * lastDt_;
    float speed = kf_.velocity();
    float mag = fabsf(speed) - deadband_;  // ignore the few mm/s of wobble a still target shows
    if (mag < 0) mag = 0;
    int v = (int)lroundf(kf_.distance() + (speed < 0 ? -mag : mag) * delay);
    return v < 0 ? 0 : v;
  }

  MedianFilter<5> med_;
  AdaptiveKalman kf_;
  int hold_;
  float deadband_;
  int misses_ = 0;
  int lastMedian_ = NO_TARGET;
  float lastDt_ = 0.05f;
};
