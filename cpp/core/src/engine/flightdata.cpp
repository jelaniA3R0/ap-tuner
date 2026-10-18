#include "aptuner/engine/flightdata.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aptuner/log/integrity.hpp"

namespace aptuner::engine {

using core::Family;
using core::Vec;

bool isPlaneLog(const log::LogData& log, Family family) {
  std::string txt;
  for (std::size_t i = 0; i < log.text.size(); i++) {
    if (i) txt += ' ';
    txt += log.text[i].msg;
  }
  if (txt.find("ArduPlane") != std::string::npos) return true;
  if (txt.find("ArduCopter") != std::string::npos) return false;
  return family == Family::Plane || family == Family::Vtol;
}

std::optional<ThrottleStream> throttleStream(const log::LogData& log, Family family) {
  const log::MsgTable* q = log.msg("QTUN");
  const log::MsgTable* ct = family == Family::Vtol && q && q->col("ThO") ? q : log.msg("CTUN");
  if (!ct || !ct->col("ThO") || !ct->n || !ct->col("TimeUS")) return std::nullopt;
  ThrottleStream s;
  s.name = ct == q ? "QTUN" : "CTUN";
  s.percent = ct != q && isPlaneLog(log, family);
  const Vec& tUs = *ct->col("TimeUS");
  const Vec& thO = *ct->col("ThO");
  s.t.resize(ct->n);
  s.v.resize(ct->n);
  for (std::size_t i = 0; i < ct->n; i++) {
    s.t[i] = tUs[i] / 1e6;
    s.v[i] = s.percent ? thO[i] / 100 : thO[i];
  }
  return s;
}

bool inIntervals(const Intervals& iv, double t) {
  long long lo = 0, hi = static_cast<long long>(iv.size()) - 1;
  while (lo <= hi) {
    const long long m = (lo + hi) >> 1;
    const auto& x = iv[static_cast<std::size_t>(m)];
    if (t < x.first)
      hi = m - 1;
    else if (t >= x.second)
      lo = m + 1;
    else
      return true;
  }
  return false;
}

std::optional<Airborne> airborneIntervals(const ThrottleStream& thr, double tA, double tB) {
  const std::size_t n = thr.t.size();
  Vec vals;
  for (std::size_t i = 0; i < n; i++)
    if (thr.t[i] >= tA && thr.t[i] <= tB && thr.v[i] > 0.15) vals.push_back(thr.v[i]);
  if (vals.size() < 10) return std::nullopt;
  std::sort(vals.begin(), vals.end());
  const double median = vals[vals.size() / 2];
  Airborne a;
  a.lim = std::max(0.15, 0.6 * median);
  const double hold = std::max(0.5, 3 * log::medianStep(thr.t));
  std::size_t i = 0;
  bool afterGap = false;
  while (i < n) {
    if (!(thr.v[i] > a.lim)) {
      i++;
      afterGap = false;
      continue;
    }
    std::size_t j = i;
    while (j + 1 < n && thr.v[j + 1] > a.lim && thr.t[j + 1] - thr.t[j] <= hold) j++;
    // a run that ends because the throttle log has a gap (the next sample still flies) is not a landing, and the run
    // after such a gap is not a take-off: no ground margins there
    const bool endsAtGap = j + 1 < n && thr.v[j + 1] > a.lim;
    const double start = thr.t[i], end = j + 1 < n ? std::min(thr.t[j + 1], thr.t[j] + hold) : thr.t[j] + hold;
    const double x0 = start + (afterGap ? 0 : kAfterTakeoffS), x1 = end - (endsAtGap ? 0 : kBeforeLandingS);
    if (x1 > x0) a.intervals.emplace_back(x0, x1);
    afterGap = endsAtGap;
    i = j + 1;
  }
  Vec hov;
  for (std::size_t k = 0; k < n; k++)
    if (thr.t[k] >= tA && thr.t[k] <= tB && inIntervals(a.intervals, thr.t[k])) hov.push_back(thr.v[k]);
  std::sort(hov.begin(), hov.end());
  a.hover = hov.empty() ? median : hov[hov.size() / 2];
  return a;
}

std::optional<double> windowMean(const log::MsgTable* m, const std::string& field, const Window& win,
                                 const Intervals* iv, double minVal) {
  if (!m || !m->col(field) || !m->col("TimeUS")) return std::nullopt;
  const Vec& tUs = *m->col("TimeUS");
  const Vec& v = *m->col(field);
  double s = 0, c = 0;
  for (std::size_t i = 0; i < m->n; i++) {
    const double t = tUs[i] / 1e6;
    if (win.tStart && (t < *win.tStart || t > win.tEnd.value_or(0))) continue;
    if (iv && !inIntervals(*iv, t)) continue;
    if (!(v[i] > minVal)) continue;
    s += v[i];
    c++;
  }
  if (c == 0) return std::nullopt;
  return s / c;
}

std::optional<log::MsgTable> rpmSensor(const log::LogData& log, int sensor) {
  const log::MsgTable* r = log.msg("RPM");
  if (!r || !r->n || !r->col("TimeUS")) return std::nullopt;
  log::MsgTable o;
  if (r->col("rpm1")) {
    const Vec* c = sensor == 2 ? r->col("rpm2") : r->col("rpm1");
    if (!c) return std::nullopt;
    o.n = r->n;
    o.num["TimeUS"] = *r->col("TimeUS");
    o.num["rpm"] = *c;
    return o;
  }
  if (!r->col("RPM")) return std::nullopt;
  const Vec* inst = r->col("I") ? r->col("I") : r->col("Instance");
  if (!inst) {
    if (sensor != 1) return std::nullopt;
    o.n = r->n;
    o.num["TimeUS"] = *r->col("TimeUS");
    o.num["rpm"] = *r->col("RPM");
    return o;
  }
  Vec t, v;
  for (std::size_t i = 0; i < r->n; i++)
    if ((*inst)[i] == sensor - 1) {
      t.push_back((*r->col("TimeUS"))[i]);
      v.push_back((*r->col("RPM"))[i]);
    }
  if (t.empty()) return std::nullopt;
  o.n = t.size();
  o.num["TimeUS"] = std::move(t);
  o.num["rpm"] = std::move(v);
  return o;
}

core::NotchOp notchOperatingPoint(const log::LogData& log, Family family, const Window& win,
                                  const std::optional<Airborne>& air) {
  const Intervals* iv = air ? &air->intervals : nullptr;
  core::NotchOp op;
  const auto thr = throttleStream(log, family);
  if (air) {
    op.throttle = air->hover;
  } else if (thr) {
    Vec v;
    for (std::size_t i = 0; i < thr->t.size(); i++)
      if ((!win.tStart || (thr->t[i] >= *win.tStart && thr->t[i] <= win.tEnd.value_or(0))) && thr->v[i] > 0.15)
        v.push_back(thr->v[i]);
    std::sort(v.begin(), v.end());
    if (!v.empty()) op.throttle = v[v.size() / 2];
  }
  if (const auto esc = windowMean(log.msg("ESC"), "RPM", win, iv, 100)) op.escRpm = *esc;
  const auto r1 = rpmSensor(log, 1), r2 = rpmSensor(log, 2);
  if (r1)
    if (const auto m1 = windowMean(&*r1, "rpm", win, iv, 0)) op.rpm = *m1;
  if (r2)
    if (const auto m2 = windowMean(&*r2, "rpm", win, iv, 0)) op.rpm2 = *m2;
  if (const auto fft = windowMean(log.msg("FTN1"), "PkAvg", win, iv, 0)) op.fftHz = *fft;
  const log::MsgTable* imu = log.msg("IMU");
  if (imu && imu->col("GHz") && imu->n) {
    // the gyro filters run at the IMU's own rate: the first instance (the usual primary), median value
    const Vec& ghz = *imu->col("GHz");
    const Vec* inst = imu->col("I");
    Vec v;
    for (std::size_t i = 0; i < imu->n; i++)
      if ((!inst || (*inst)[i] == (*inst)[0]) && ghz[i] > 0) v.push_back(ghz[i]);
    std::sort(v.begin(), v.end());
    if (!v.empty()) op.backendHz = v[v.size() / 2];
  }
  return op;
}

}  // namespace aptuner::engine
