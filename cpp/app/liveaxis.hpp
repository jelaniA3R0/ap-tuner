// Live per-axis data: buffer of PID_TUNING samples, uniform windows for analysis, and the oscillation watchdog
// (QuikTune-style slew-rate detector + spectral check). Port of src/engine/axis.js.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/engine/analysis.hpp"

namespace aptuner::app {

using core::Json;
using core::Vec;

struct Watchdog {
  bool oscillating = false;
  std::vector<std::string> reasons;
  double srateMax = 0, dmodMin = 1, bandRms = 0, peakHz = 0;
  int samples = 0;
  Json toJson() const;
};

class LiveAxis {
 public:
  explicit LiveAxis(std::string name = "", double keepSeconds = 180) : name_(std::move(name)), keep_(keepSeconds) {}
  /** f: PID_TUNING fields (desired / achieved already in deg/s) */
  void push(double ts, const Json& f);
  void clear();
  double seconds() const { return t.size() > 1 ? t.back() - t.front() : 0; }
  double rateHz() const;
  /** uniform dataset over the last `seconds` (0 = all); `since` (s, same clock as push) also drops older samples */
  std::optional<engine::AxisData> window(double seconds = 0, std::optional<double> since = std::nullopt) const;
  /** RMS of the rate target over the last `seconds` */
  double excitation(double seconds = 20) const;
  /** since: ignore samples older than this (s, same clock as push), e.g. those from before a gain change; the spectral
      check then waits until a full window (max(seconds, 3) s) has come in after it */
  Watchdog watchdog(double seconds = 2, double oscSmax = 5, double bandRmsDegs = 8, double dmodMin = 0.9,
                    std::optional<double> since = std::nullopt) const;
  void setBaseline();

  Vec t, target, actual, P, I, D, FF, srate, dmod;
  double lastTs = 0;
  long long count = 0;
  std::optional<double> baselineOsc;

 private:
  std::string name_;
  double keep_;
};

}  // namespace aptuner::app
