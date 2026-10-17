// Log (.bin) analysis: per-axis rate-loop data for copter / VTOL / plane, analyzeAxis on each axis, motor and
// I-term summaries, and the gyro-noise / notch analysis. Port of src/engine/loganalysis.js.
#pragma once

#include <optional>
#include <set>
#include <string>

#include "aptuner/core/json.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

/** message types the analysis reads (others are only counted) */
const std::set<std::string>& logWant();

Family detectLogFamily(const log::LogData& log);

struct Window {
  std::optional<double> tStart, tEnd;
};
/** uniform per-axis dataset in the window; nullopt when the log has no usable rate-loop data */
std::optional<AxisData> axisData(const log::LogData& log, Family family, Axis axis, const Window& win);
/** longest flying stretch in a mode that uses the analysed controller: [t0, t1] seconds */
std::optional<std::pair<double, double>> flightWindow(const log::LogData& log, Family family);
Json motorStats(const log::LogData& log, const log::Params& params, Family family, const Window& win);
Json iStats(const log::LogData& log, Family family, Axis axis, const AxisData& d);

struct AnalyzeLogOptions {
  std::string preset = "normal";
  std::optional<Family> family;
  std::optional<double> tStart, tEnd;
};
/** full analysis of a parsed log (file = name reported in the result) */
Json analyzeLog(const log::LogData& log, const std::string& file, const AnalyzeLogOptions& opt = {});
/** read, parse and analyse a .bin file */
Json analyzeLogFile(const std::string& path, const AnalyzeLogOptions& opt = {});

}  // namespace aptuner::engine
