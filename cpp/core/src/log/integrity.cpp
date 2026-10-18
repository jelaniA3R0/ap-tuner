#include "aptuner/log/integrity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aptuner::log {

namespace {
/** look-ahead used to tell a single bad stamp from a real jump of the clock */
constexpr std::size_t kLookahead = 8;
}  // namespace

TimeClean cleanTimes(const Vec& t) {
  TimeClean c;
  const std::size_t n = t.size();
  double last = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; i++) {
    const double v = t[i];
    if (!std::isfinite(v)) {
      c.outOfOrder++;
      continue;
    }
    if (v == last) {
      c.duplicates++;
      continue;
    }
    const std::size_t jEnd = std::min(n - 1, i + kLookahead);
    if (v > last) {
      // ahead of the stream: most of the next stamps fall back between the last good one and this one
      std::size_t between = 0;
      for (std::size_t j = i + 1; j <= jEnd; j++)
        if (t[j] > last && t[j] < v) between++;
      if (between * 2 > jEnd - i) {
        c.outOfOrder++;
        continue;
      }
      c.keep.push_back(i);
      last = v;
      continue;
    }
    // behind the stream: a single bad stamp if the stream carries on after the last good stamp (or ends too soon to tell)
    bool carriesOn = i + 2 >= n;
    for (std::size_t j = i + 1; j <= jEnd && !carriesOn; j++)
      if (t[j] > last) carriesOn = true;
    if (carriesOn) {
      c.outOfOrder++;
      continue;
    }
    c.reset = std::make_pair(last, v);
    c.droppedAfterReset = n - i;
    break;
  }
  return c;
}

double medianStep(const Vec& t, std::size_t i0, std::size_t i1) {
  Vec d;
  for (std::size_t i = i0 + 1; i < i1; i++) {
    const double x = t[i] - t[i - 1];
    if (x > 0) d.push_back(x);
  }
  if (d.empty()) return 0;
  std::sort(d.begin(), d.end());
  return d[d.size() / 2];
}

GapSplit splitAtGaps(const Vec& t, std::size_t i0, std::size_t i1) {
  GapSplit out;
  out.medianDt = medianStep(t, i0, i1);
  if (i1 <= i0) return out;
  const double lim = kGapFactor * out.medianDt;
  std::size_t s = i0;
  for (std::size_t i = i0 + 1; i < i1; i++) {
    if (out.medianDt > 0 && t[i] - t[i - 1] > lim) {
      out.segments.emplace_back(s, i);
      out.gaps.emplace_back(t[i - 1], t[i]);
      s = i;
    }
  }
  out.segments.emplace_back(s, i1);
  return out;
}

std::map<std::string, TimeIssue> cleanLogTimes(LogData& log, const std::vector<std::string>& names) {
  std::map<std::string, TimeIssue> report;
  for (const std::string& name : names) {
    MsgTable* m = log.msg(name);
    if (!m || !m->n) continue;
    const Vec* tAll = name == "GYR" && m->col("SampleUS") ? m->col("SampleUS") : m->col("TimeUS");
    if (!tAll) continue;
    const Vec* inst = nullptr;
    for (const char* k : {"I", "Instance", "instance"})
      if (!inst) inst = m->col(k);
    std::map<double, std::vector<std::size_t>> groups;   // ascending instance
    for (std::size_t i = 0; i < m->n; i++) groups[inst ? (*inst)[i] : 0].push_back(i);
    std::vector<std::uint8_t> keepRow(m->n, 0);
    TimeIssue r;
    for (const auto& [g, rows] : groups) {
      Vec tg(rows.size());
      for (std::size_t k = 0; k < rows.size(); k++) tg[k] = (*tAll)[rows[k]];
      const TimeClean c = cleanTimes(tg);
      for (std::size_t k : c.keep) keepRow[rows[k]] = 1;
      r.outOfOrder += c.outOfOrder;
      r.duplicates += c.duplicates;
      r.droppedAfterReset += c.droppedAfterReset;
      if (c.reset && !r.reset) r.reset = std::make_pair(c.reset->first / 1e6, c.reset->second / 1e6);
    }
    std::vector<std::size_t> idx;
    for (std::size_t i = 0; i < m->n; i++)
      if (keepRow[i]) idx.push_back(i);
    if (idx.size() == m->n) continue;
    report[name] = r;
    for (auto& [lab, a] : m->num) {
      Vec b(idx.size());
      for (std::size_t k = 0; k < idx.size(); k++) b[k] = a[idx[k]];
      a = std::move(b);
    }
    for (auto& [lab, a] : m->str) {
      std::vector<std::string> b;
      for (std::size_t i : idx) b.push_back(a[i]);
      a = std::move(b);
    }
    for (auto& [lab, a] : m->i16) {
      std::vector<std::array<std::int16_t, 32>> b;
      for (std::size_t i : idx) b.push_back(a[i]);
      a = std::move(b);
    }
    m->n = idx.size();
  }
  return report;
}

}  // namespace aptuner::log
