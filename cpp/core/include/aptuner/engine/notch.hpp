// Gyro noise analysis and harmonic-notch recommendation from a log. Port of src/engine/notch.js.
// Data sources (best first): GYR (raw gyro, INS_RAW_LOG_OPT), ISBH/ISBD batches (INS_LOG_BAT_MASK), IMU at the
// logging rate (post-filter). Peaks, harmonic family, repeatability across the flight, a recommended INS_HNTCH_*
// set with its predicted effect and phase lag at 10 Hz.
// Only airborne gyro data are used, spectra are averaged over unbroken blocks (logging gaps and batch boundaries
// are never joined), and post-filter data logged too slowly to rule out aliasing get no notch.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/engine/flightdata.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

struct GyroSamples {
  double fs = 0;
  core::Vec t;   // seconds (empty for batch data)
  core::Vec x, y, z;   // deg/s
  core::Blocks blocks;   // unbroken stretches [i0, i1) (a left-out sample, a logging gap or a batch boundary ends one)
  std::string source;
  bool prefilter = false;
  bool batched = false;
};
/**
 * the best gyro data of the first IMU: samples outside the window, outside the airborne stretches (air) or not
 * finite are left out, the rest is cut into unbroken blocks for the spectra
 */
std::optional<GyroSamples> gyroSamples(const log::LogData& log, const Window& win,
                                       const std::optional<Airborne>& air = std::nullopt);

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

/** what the log analysis already knows about the flight */
struct NoiseOpts {
  std::optional<Airborne> air;   // airborne stretches (none for planes)
  core::NotchOp op;              // what the dynamic notch follows over the analysed airborne time
};
/** gyro noise spectrum and notch recommendation over the analysed window; opts = nullptr: worked out from the log */
Json analyzeNoise(const log::LogData& log, const log::Params& params, Family family, const Window& win,
                  const NoiseOpts* opts = nullptr);

}  // namespace aptuner::engine
