// Gyro noise analysis and harmonic-notch recommendation from a log. Port of src/engine/notch.js.
// Data sources (best first): GYR (raw gyro, INS_RAW_LOG_OPT), ISBH/ISBD batches (INS_LOG_BAT_MASK), IMU at the
// logging rate (post-filter). Peaks, harmonic family, repeatability across the flight, a recommended INS_HNTCH_*
// set with its predicted effect and phase lag at 10 Hz.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

struct GyroSamples {
  double fs = 0;
  core::Vec t;   // seconds (empty for batch data)
  core::Vec x, y, z;   // deg/s
  std::string source;
  bool prefilter = false;
  bool batched = false;
};
std::optional<GyroSamples> gyroSamples(const log::LogData& log, const Window& win);

struct Peak {
  double f = 0, amp = 0, prominence = 0, widthHz = 0;
};
std::vector<Peak> findPeaks(const core::Vec& f, const core::Vec& amp, double fMin = 10, double fMax = 1e300,
                            std::size_t maxPeaks = 6);

struct HarmonicMember {
  int k = 0;
  double f = 0, amp = 0;
};
struct HarmonicFamily {
  double f0 = 0, score = 0;
  std::vector<HarmonicMember> members;
};
std::optional<HarmonicFamily> harmonicFamily(const std::vector<Peak>& peaks);

Json analyzeNoise(const log::LogData& log, const log::Params& params, Family family, const Window& win);

}  // namespace aptuner::engine
