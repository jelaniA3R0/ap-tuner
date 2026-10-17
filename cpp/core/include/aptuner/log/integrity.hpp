// Time-stamp integrity of log streams (audit finding F-07): duplicate and out-of-order stamps, clock resets,
// gaps, and the split of a stream into unbroken segments. Port of src/log/integrity.js. Nothing here
// interpolates: callers get the indices of the samples that can be trusted and the places where data are missing.
#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/log/dfreader.hpp"

namespace aptuner::log {

/** a step longer than this many typical sample intervals is a gap (data missing) */
inline constexpr double kGapFactor = 2.5;

struct TimeClean {
  std::vector<std::size_t> keep;   // indices of a strictly increasing, trustworthy time base
  std::size_t outOfOrder = 0, duplicates = 0;
  std::optional<std::pair<double, double>> reset;   // {from, to} when the clock restarts
  std::size_t droppedAfterReset = 0;
};
/**
 * Check the stamps of one stream (log order): repeated stamps are duplicates; a stamp ahead of most of the next
 * stamps, or behind the last good one while the stream carries on, is a single bad record; a jump back that the
 * following records keep is a reset and ends the usable part.
 */
TimeClean cleanTimes(const Vec& t);

/** median of the positive steps of t[i0, i1) (0 when there are none) */
double medianStep(const Vec& t, std::size_t i0, std::size_t i1);
inline double medianStep(const Vec& t) { return medianStep(t, 0, t.size()); }

struct GapSplit {
  std::vector<std::pair<std::size_t, std::size_t>> segments;   // [i0, i1)
  std::vector<std::pair<double, double>> gaps;                 // {t0, t1}
  double medianDt = 0;
};
/** split a strictly increasing time base into unbroken segments (a step > kGapFactor x the median step is a gap) */
GapSplit splitAtGaps(const Vec& t, std::size_t i0, std::size_t i1);
inline GapSplit splitAtGaps(const Vec& t) { return splitAtGaps(t, 0, t.size()); }

struct TimeIssue {
  std::size_t outOfOrder = 0, duplicates = 0, droppedAfterReset = 0;
  std::optional<std::pair<double, double>> reset;   // {fromS, toS}
};
/**
 * Clean the time bases of the named tables in place (bad rows removed). Multi-instance tables are checked per
 * instance (field I / Instance / instance); GYR uses its sample time (SampleUS). Returns the tables with problems.
 */
std::map<std::string, TimeIssue> cleanLogTimes(LogData& log, const std::vector<std::string>& names);

}  // namespace aptuner::log
