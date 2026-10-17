// Closed-loop step-response estimate from target / actual data (Wiener deconvolution, the method of
// PID-Analyzer, PIDtoolbox and ArduPilot PIDReview) and step-response metrics. Port of src/dsp/stepresp.js.
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "aptuner/core/cx.hpp"

namespace aptuner::core {

struct StepResponseOptions {
  std::size_t windowSize = 512;
  double durationS = 0.5;
  double minInput = 20;
  int maxWindows = 400;
  int overlapDiv = 8;
};

struct StepWindow {
  double t0 = 0;
  Vec resp;
};

struct StepResponse {
  Vec t;
  Vec mean;  // normalised so a perfect response settles at 1.0
  int count = 0;
  std::vector<StepWindow> windows;
};

StepResponse stepResponse(const double* target, const double* actual, std::size_t n, double fs,
                          const StepResponseOptions& opt = {});
inline StepResponse stepResponse(const Vec& target, const Vec& actual, double fs, const StepResponseOptions& opt = {}) {
  return stepResponse(target.data(), actual.data(), target.size() < actual.size() ? target.size() : actual.size(), fs,
                      opt);
}

struct StepMetrics {
  double finalValue = 0, peak = 0, overshootPct = 0;
  double riseTime = 0;      // 10-90 %, NaN when not reached
  double settlingTime = 0;  // 5 % band, NaN when never outside it
  int oscillations = 0;
};

/** metrics of a normalised step response (final value ~1); nullopt for an empty response */
std::optional<StepMetrics> stepMetrics(const Vec& t, const Vec& y);

}  // namespace aptuner::core
