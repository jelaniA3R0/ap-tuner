// Log (.bin) analysis: per-axis rate-loop data for copter / VTOL / plane, analyzeAxis on each axis, motor and
// I-term summaries, and the gyro-noise / notch analysis. Port of src/engine/loganalysis.js.
// Data integrity (audit 2026-10-17): the file type is checked; time stamps are cleaned (repeated and out-of-order
// records, clock resets); the rate data stay at their native rate and are split at logging gaps (nothing is
// interpolated across a gap); impossible or non-numeric samples are masked; every metric and spectrum uses
// airborne samples only; the analysis window never spans a change of a tuning parameter or a fixed-wing / VTOL
// transition; and all of it is reported in plain language.
#pragma once

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/engine/flightdata.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

/** message types the analysis reads (others are only counted) */
const std::set<std::string>& logWant();
/** tables whose time stamps the analysis relies on: checked and cleaned after parsing */
const std::vector<std::string>& logTimed();

Family detectLogFamily(const log::LogData& log);

/** parameters the analysis depends on (gains, filters, notch, loop rate, mixer, frame), not housekeeping */
bool isTuningParam(const std::string& name);
struct ParamChangeRec {
  double t = 0;
  std::string name;
  std::optional<double> from;   // unset: the parameter had no boot value
  double to = 0;
};
struct ParamChanges {
  std::vector<ParamChangeRec> all, tuning;
};
/** in-flight parameter changes, all of them and the tuning-relevant ones */
ParamChanges parameterChanges(const log::LogData& log);
/** the parameter values in effect at time t: boot values with every logged change up to t applied */
log::Params paramsAt(const log::LogData& log, std::optional<double> t);

/** ArduPlane VTOL (Q) modes: 17-23 and 25 (24 THERMAL and 26 AUTOLAND fly as a plane) */
bool isQMode(double mode);
/** seconds left out after a switch between fixed-wing and VTOL flight (Q_TRANSITION_MS default) */
inline constexpr double kTransitionS = 5;

struct ChosenWindow {
  bool none = false;    // modes are logged but none of them uses the analysed controller
  std::string reason;   // why (none)
  double tStart = 0, tEnd = 0;
  std::optional<double> mode;
  std::optional<std::pair<double, double>> transition;   // {from, to} mode numbers
  double rangeT0 = 0, rangeT1 = 0;                        // the mode segment (or explicit range) it lies in
  bool explicitRange = false;
  std::vector<ParamChangeRec> cutBy;                      // tuning changes inside that range
};
/**
 * The analysis window: the longest stretch flown (throttle above idle) in a mode that uses the analysed controller,
 * without the first 5 s after a fixed-wing / VTOL switch, never across a change of a tuning parameter. An explicit
 * [tStart, tEnd] is cut the same way at tuning changes.
 */
std::optional<ChosenWindow> chooseWindow(const log::LogData& log, Family family,
                                         const std::optional<ThrottleStream>& thr,
                                         const std::vector<ParamChangeRec>& tuning,
                                         std::optional<double> tStart = std::nullopt,
                                         std::optional<double> tEnd = std::nullopt);
/** longest flying stretch that the analysis uses: [t0, t1] seconds */
std::optional<std::pair<double, double>> flightWindow(const log::LogData& log, Family family);

/** time (s) of grid sample g of an axis dataset (each segment keeps its logged start time) */
double gridTime(const AxisData& d, std::size_t g);
/** grid sample nearest to time t (s) inside a segment, or -1 */
long long gridIndex(const AxisData& d, double t);

/** what the analysis already knows about the flight (throttle stream, airborne stretches, notch operating point) */
struct AxisCtx {
  std::optional<ThrottleStream> thr;
  std::optional<Airborne> air;
  std::optional<core::NotchOp> op;
};
/**
 * The per-axis dataset at the native rate of the rate-loop log: unbroken segments one after the other on a grid
 * at the mean sample rate inside them, missing and corrupt samples marked (valid = 0), airborne mask (copter /
 * VTOL). nullopt when the log has no usable rate-loop data. ctx = nullptr: worked out from the log.
 */
std::optional<AxisData> axisData(const log::LogData& log, Family family, Axis axis, const Window& win,
                                 const AxisCtx* ctx = nullptr);
/** lift-motor outputs while flying (airborne stretches of the window); null if none */
Json motorStats(const log::LogData& log, const log::Params& params, Family family, const Window& win,
                const std::optional<Airborne>& air = std::nullopt);
/** I-term summary of an axis from the PID log, usable airborne samples of the data window only; null if none */
Json iStats(const log::LogData& log, Family family, Axis axis, const AxisData& d);

struct AnalyzeLogOptions {
  std::string preset = "normal";
  std::optional<Family> family;
  std::optional<double> tStart, tEnd;
};
/** full analysis of a parsed log (file = name reported in the result); the time stamps are cleaned in place */
Json analyzeLog(log::LogData& log, const std::string& file, const AnalyzeLogOptions& opt = {});
/** read, parse and analyse a .bin file */
Json analyzeLogFile(const std::string& path, const AnalyzeLogOptions& opt = {});

}  // namespace aptuner::engine
