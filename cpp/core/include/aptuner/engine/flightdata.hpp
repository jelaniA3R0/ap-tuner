// Flight-phase helpers shared by the log analysis and the notch analysis: the lift-throttle stream as a 0..1
// fraction, the airborne stretches, and means of other logged signals over the analysed (airborne) time.
// Port of src/engine/flightdata.js.
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/log/dfreader.hpp"

namespace aptuner::engine {

/** seconds kept out after lift-off and before touch-down (ground contact, ground effect) */
inline constexpr double kAfterTakeoffS = 3, kBeforeLandingS = 2;

/** an ArduPlane log (CTUN.ThO is in percent there, 0..1 in ArduCopter) */
bool isPlaneLog(const log::LogData& log, core::Family family);

/** throttle out as a 0..1 fraction from QTUN (VTOL lift motors) or CTUN (ArduPlane CTUN.ThO is in percent) */
struct ThrottleStream {
  core::Vec t, v;   // seconds, 0..1
  std::string name;
  bool percent = false;
};
std::optional<ThrottleStream> throttleStream(const log::LogData& log, core::Family family);

/** analysis window (s); unset = the whole log */
struct Window {
  std::optional<double> tStart, tEnd;
};

using Intervals = std::vector<std::pair<double, double>>;   // sorted, disjoint [t0, t1)
struct Airborne {
  Intervals intervals;
  double lim = 0, hover = 0;
};
/**
 * airborne stretches (copter / VTOL): throttle above 60% of the median hover throttle in [tA, tB] (at least 0.15),
 * 3 s after lift-off and 2 s before touch-down left out (not at a gap in the throttle log), a sample held for at
 * most max(0.5 s, 3 throttle intervals); nullopt with fewer than 10 throttle samples above 0.15 in the window
 */
std::optional<Airborne> airborneIntervals(const ThrottleStream& thr, double tA, double tB);
bool inIntervals(const Intervals& iv, double t);

/** mean of a column over the window and (when given) the intervals, values above minVal only; nullopt if none */
std::optional<double> windowMean(const log::MsgTable* m, const std::string& field, const Window& win,
                                 const Intervals* iv, double minVal);

/** samples of RPM sensor 1 or 2 (instance format or the old rpm1 / rpm2 fields) as a two-column table */
std::optional<log::MsgTable> rpmSensor(const log::LogData& log, int sensor);

/** what the dynamic notch follows over the analysed airborne time (throttle, ESC, RPM, FFT, IMU.GHz) */
core::NotchOp notchOperatingPoint(const log::LogData& log, core::Family family, const Window& win,
                                  const std::optional<Airborne>& air);

}  // namespace aptuner::engine
