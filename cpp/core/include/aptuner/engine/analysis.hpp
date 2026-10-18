// Shared per-axis analysis used by the live tuner and the log analyser: uniformly sampled data ->
// identification -> synthesis -> prediction -> diagnostics. Port of src/engine/analysis.js; results are JSON
// documents with the same keys the JavaScript version produced (the web UI reads them).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/cx.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/core/plant.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

using core::Axis;
using core::Family;
using core::Json;
using core::Vec;

/** one unbroken segment of an axis dataset: grid samples [g0, g1), logged time of g0 */
struct AxisSeg {
  std::size_t g0 = 0, g1 = 0;
  double t0 = 0;
};

/** uniformly sampled rate-loop data of one axis (deg/s; u = PID output) */
struct AxisData {
  double fs = 0, t0 = 0, rawRate = 0;
  std::size_t n = 0;
  Vec target, actual, u;
  std::optional<Vec> P, I, D, FF, srate, dmod, ss;   // PID terms when the log / link has them
  double scaler2 = 1;                                 // plane: mean speed scaler^2
  std::optional<double> throttle;
  std::vector<std::uint8_t> airborne;                 // empty = all valid
  std::vector<std::uint8_t> valid;                    // log analysis: 0 in logging gaps and for corrupt samples (empty = all real)
  std::vector<AxisSeg> segs;                          // log analysis: unbroken segments (empty: t = t0 + i / fs)
  std::vector<std::pair<double, double>> gaps;        // log analysis: logging gaps {t0, t1} (s)
  std::size_t nonFinite = 0, implausible = 0;         // log analysis: masked samples
  std::string source;                                 // log message the data come from
  Json motors;                                        // motor summary (log analysis), null if none
  Json iStats;                                        // I-term summary from a slower PID log, null if none
};

/**
 * the samples an analysis may use: real (valid), airborne and finite (a NaN or infinity in target, actual or u is
 * never used); empty when every sample is usable
 */
std::vector<std::uint8_t> usableMask(const AxisData& data);
/** unbroken runs [i0, i1) of usable samples at least minLen long (empty mask: the whole array) */
core::Blocks blocksOf(const std::vector<std::uint8_t>& mask, std::size_t n, std::size_t minLen = 1);

/** prominence (over the median of its neighbourhood) a rate-error peak needs to count as an oscillation */
inline constexpr double kPeakProminence = 10;
struct LocalPeak {
  std::size_t i = 0;
  double f = 0, prominence = 0;
};
/** strongest true local maximum of a spectrum in (fLo, fHi] by prominence over the median of +-8 bins (own lobe out) */
std::optional<LocalPeak> localPeak(const core::Psd& sp, double fLo, double fHi);

/** gyro filter chain from parameters (notch frequencies resolved at the operating point) */
core::GyroChain gyroChainFromParams(const log::Params& params, const core::NotchOp& op = {});
Json gyroChainJson(const core::GyroChain& g);

struct Resampled {
  double fs = 0, t0 = 0;
  std::size_t n = 0;
  std::vector<Vec> cols;
};
/** linear resampling of irregular samples (t ascending, s) onto a uniform grid at fs; nullopt for < 4 samples */
std::optional<Resampled> resample(const Vec& t, const std::vector<const Vec*>& cols, double fs);
/** average sample rate (n - 1) / span */
double estimateRate(const double* t, std::size_t n);
inline double estimateRate(const Vec& t) { return estimateRate(t.data(), t.size()); }

/** gains of an axis as JSON (axisGains + angleP / accMax / tconst) */
Json currentGains(Family f, Axis axis, const log::Params& params);
/** [{key, from, to, pct}] for the keys whose values differ */
Json describeChanges(const Json& cur, const Json& nw);

/** gyroFilterHz: INS_GYRO_FILTER for the aliasing check (0: 20 Hz) */
Json diagnostics(const AxisData& data, const Json& cur, Family f, Axis axis, double gyroFilterHz = 0);
Json saturationCheck(const AxisData& data, const Json& cur, Family f, const Json& d);
struct GradeCtx {
  std::optional<bool> idOk;   // unset: grade from the data only
  bool lowConf = false;
};
Json gradeAxis(const Json& d, const GradeCtx& ctx = {});
Json planeFFEvents(const AxisData& data, const core::Blocks* blocks = nullptr);

struct AnalyzeArgs {
  Family family = Family::Copter;
  Axis axis = Axis::Roll;
  const AxisData* data = nullptr;
  const log::Params* params = nullptr;
  std::string preset = "normal";
  core::NotchOp gyroOp;
  double scaler2 = 1;
  double fsLoop = 0;            // 0: SCHED_LOOP_RATE or the family default
  Json designFrom;              // original gains to design from (live tuner), null for logs
  bool timeDomain = false;      // allow the hover-data fallback (log analysis only)
};
Json analyzeAxis(const AnalyzeArgs& a);

}  // namespace aptuner::engine
