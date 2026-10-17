// Time-domain identification of the rate plant from ordinary hover data (little stick input). Port of
// src/model/timeid.js:   r_dot(t) = b (sat(u)_lag(t - Td) - trim) - a r(t)
// Equation-error least squares on the low-pass filtered equation over airborne samples (grid over motor lag,
// delay and mixer limit, one trim per ~10 s chunk), then an output-error refinement; lower confidence by design.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/rules.hpp"

namespace aptuner::core {

Vec lpf2(const Vec& x, double fs, double fc);
Vec lag1(const Vec& x, double fs, double tau);

struct TimeIdInput {
  double fs = 0;
  Vec actual;                          // deg/s
  Vec u;                               // controller output (mixer units)
  std::vector<std::uint8_t> airborne;  // empty = all samples valid
};

struct TimeIdOptions {
  std::optional<double> fc;  // equation filter, default yaw 1 Hz, roll/pitch 2 Hz
  double chunkS = 10;
  double tdMax = 0.1;
  Vec taus{0.005, 0.01, 0.02, 0.035, 0.05, 0.075, 0.1, 0.15, 0.2};
  Axis axis = Axis::Roll;
};

struct TimeIdEquationError {
  double K = 0, a = 0;
  std::optional<double> uLimit;
  double r2 = 0, vaf = 0;
};

struct TimeIdFit {
  double K = 0;                  // b, deg/s^2 per unit of u
  double a = 0, tau = 0, Td = 0;
  std::optional<double> uLimit;  // mixer limit found, if any
  double uMaxAbs = 0;
  Vec trims;
  double trimMean = 0, trimMin = 0, trimMax = 0;
  double r2 = 0, vaf = 0;
  double fc = 0, chunkS = 0, airborneS = 0;
  int segments = 0;
  double limitPct = 0;
  double nEff = 0, bRelErr = 0, uRms = 0;
  TimeIdEquationError equationError;
  std::string method;            // "equation error" | "output error"
  double fitQuality = 0;
};

struct TimeIdResult {
  bool ok = false;
  std::string reason;            // why it was rejected (empty when ok)
  std::optional<TimeIdFit> fit;  // present when a fit was computed (also when rejected)
};

TimeIdResult identifyTimeDomain(const TimeIdInput& data, const TimeIdOptions& opt = {});

}  // namespace aptuner::core
