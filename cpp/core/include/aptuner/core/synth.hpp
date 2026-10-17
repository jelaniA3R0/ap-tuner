// Loop analysis and gain synthesis against the identified plant, using ArduPilot's exact controller and filter
// structure; closed-loop prediction (frequency domain and time-domain simulation). Port of src/model/synth.js.
#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/plant.hpp"

namespace aptuner::core {

Vec logspace(double f1, double f2, std::size_t n);

struct Margins {
  double gmDb = std::numeric_limits<double>::quiet_NaN();
  double pmDeg = std::numeric_limits<double>::quiet_NaN();
  double wcHz = std::numeric_limits<double>::quiet_NaN();
  double w180Hz = std::numeric_limits<double>::quiet_NaN();
  double ms = 0;   // sensitivity peak max |1/(1+L)|
};
Margins margins(const Vec& freqs, const CVec& L);

struct LoopCtx {
  double fsLoop = 400;
  GyroChain gyro;
  Plant plant;   // K per ArduPilot gain unit (copter: per rad/s)
};

struct OpenLoop {
  CVec L, G, F;
  PidResponse pid;
};
OpenLoop openLoop(const Vec& freqs, const LoopCtx& ctx, const PidGains& g);

struct ClosedLoop {
  CVec L, Tcl, S, G, F;
  PidResponse pid;
};
ClosedLoop closedLoop(const Vec& freqs, const LoopCtx& ctx, const PidGains& g);

double sensitivityEnergy(const Vec& freqs, const CVec& S, double fMax);
double sensitivityBandwidth(const Vec& freqs, const CVec& S);
double bandwidth3db(const Vec& freqs, const CVec& T);

struct SimResult {
  Vec t, y, u, ytrue;
};
struct SimOptions {
  double uMax = std::numeric_limits<double>::infinity();
  double noiseRms = 0;       // gyro noise, deg/s RMS, white at the backend rate
  const Vec* dist = nullptr; // torque disturbance per loop sample (optional)
  unsigned seed = 1;
};
/** time-domain simulation of the rate loop (AC_PID::update_all on the continuous plant) */
SimResult simulateRate(const LoopCtx& ctx, const PidGains& g, const Vec& ref, const SimOptions& opt = {});
SimResult simulateRateStep(const LoopCtx& ctx, const PidGains& g, double stepDegs = 30, double durationS = 1.0,
                           double uMax = std::numeric_limits<double>::infinity());

struct NoisePsd {
  Vec f, psd;
};

struct SynthOptions {
  double gmMinDb = 4, pmMinDeg = 25, msMax = 3.5, osMaxPct = 10;
  double backoff = 0.4;
  double pRange[2] = {0.01, 2.0};
  double dRange[2] = {0.0005, 0.2};
  double iRatio = 1.0;
  PidGains filters;              // FLTT / FLTE / FLTD used for the design
  double uMax = std::numeric_limits<double>::infinity();
  double dNoiseCap = 0;          // 0 = no cap
  std::optional<NoisePsd> noisePsd;
  bool noisePsdIsFiltered = false;
  double rMax = 0.08;
  double stepMag = 30;
  double stepDurS = 1.0;
  double currentP = 0, currentD = 0;
};

struct SynthCandidate {
  double r = 0, P = 0, D = 0, I = 0;
  std::optional<Margins> m;
  double bwHz = 0, sBwHz = 0, sEnergy = 0, noise = 0, overshootPct = 0;
  const char* fail = nullptr;   // nullptr, "fail", "noise" or "dmin"
};

struct OscillationPoints {
  std::optional<double> D, P;
  double backoff = 0, startP = 0;
};

struct SynthResult {
  PidGains gains;
  Margins margins;
  double bandwidthHz = 0, sBwHz = 0, overshootPct = 0, dNoise = 0;
  std::optional<OscillationPoints> oscillationPoints;
  std::vector<SynthCandidate> candidates;
  Vec freqs;
};

/** loop-shaping synthesis (largest P meeting the margins over a sweep of D/P ratios) */
std::optional<SynthResult> synthesize(const LoopCtx& ctx, const SynthOptions& opt);
/** QuikTune on the model: D, then P to the oscillation point, back off; then the margin safety net */
std::optional<SynthResult> synthesizeQuik(const LoopCtx& ctx, const SynthOptions& opt);

}  // namespace aptuner::core
