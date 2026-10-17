#include "liveaxis.hpp"

#include <algorithm>
#include <cmath>

#include "aptuner/core/fft.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/spectra.hpp"

namespace aptuner::app {

Json Watchdog::toJson() const {
  Json r = Json::array();
  for (const auto& s : reasons) r.push(s);
  return Json::obj({{"oscillating", oscillating}, {"reasons", r}, {"srateMax", srateMax}, {"dmodMin", dmodMin},
                    {"bandRms", bandRms}, {"peakHz", peakHz}, {"samples", samples}});
}

void LiveAxis::push(double ts, const Json& f) {
  t.push_back(ts);
  target.push_back(f["desired"].numOr(std::nan("")));
  actual.push_back(f["achieved"].numOr(std::nan("")));
  P.push_back(f["P"].numOr(std::nan("")));
  I.push_back(f["I"].numOr(std::nan("")));
  D.push_back(f["D"].numOr(std::nan("")));
  FF.push_back(f["FF"].numOr(std::nan("")));
  srate.push_back(f["SRate"].isNumber() ? f["SRate"].num() : 0);
  dmod.push_back(f["PDmod"].isNumber() ? f["PDmod"].num() : 1);
  count++;
  lastTs = ts;
  const double cut = ts - keep_;
  if (t.size() > 10 && t[0] < cut) {
    std::size_t k = 0;
    while (k < t.size() && t[k] < cut) k++;
    for (Vec* a : {&t, &target, &actual, &P, &I, &D, &FF, &srate, &dmod}) a->erase(a->begin(), a->begin() + static_cast<std::ptrdiff_t>(k));
  }
}

void LiveAxis::clear() {
  for (Vec* a : {&t, &target, &actual, &P, &I, &D, &FF, &srate, &dmod}) a->clear();
  baselineOsc.reset();
}

double LiveAxis::rateHz() const {
  const std::size_t n = std::min<std::size_t>(t.size(), 400);
  return engine::estimateRate(t.data() + (t.size() - n), n);
}

std::optional<engine::AxisData> LiveAxis::window(double seconds, std::optional<double> since) const {
  std::size_t i0 = 0;
  std::optional<double> cut;
  if (seconds > 0) cut = lastTs - seconds;
  if (since && (!cut || *since > *cut)) cut = since;
  if (cut)
    while (i0 < t.size() && t[i0] < *cut) i0++;
  const std::size_t m = t.size() - i0;
  if (m < 20) return std::nullopt;
  const Vec tt(t.begin() + static_cast<std::ptrdiff_t>(i0), t.end());
  const double raw = engine::estimateRate(tt);
  const double fs = raw >= 80 ? 100 : raw >= 40 ? 50 : raw >= 20 ? 25 : raw >= 8 ? 10 : 5;
  auto sl = [&](const Vec& v) { return Vec(v.begin() + static_cast<std::ptrdiff_t>(i0), v.end()); };
  Vec u(m);
  for (std::size_t i = 0; i < m; i++) u[i] = P[i0 + i] + I[i0 + i] + D[i0 + i] + FF[i0 + i];
  const Vec tar = sl(target), act = sl(actual), p = sl(P), ii = sl(I), d = sl(D), ff = sl(FF), sr = sl(srate), dm = sl(dmod);
  const auto rs = engine::resample(tt, {&tar, &act, &u, &p, &ii, &d, &ff, &sr, &dm}, fs);
  if (!rs) return std::nullopt;
  engine::AxisData a;
  a.fs = fs;
  a.t0 = rs->t0;
  a.n = rs->n;
  a.rawRate = raw;
  a.target = rs->cols[0];
  a.actual = rs->cols[1];
  a.u = rs->cols[2];
  a.P = rs->cols[3];
  a.I = rs->cols[4];
  a.D = rs->cols[5];
  a.FF = rs->cols[6];
  a.srate = rs->cols[7];
  a.dmod = rs->cols[8];
  return a;
}

double LiveAxis::excitation(double seconds) const {
  const double cut = lastTs - seconds;
  double s = 0, n = 0;
  for (std::size_t k = t.size(); k-- > 0 && t[k] >= cut;) {
    s += target[k] * target[k];
    n++;
  }
  return n != 0 ? std::sqrt(s / n) : 0;
}

Watchdog LiveAxis::watchdog(double seconds, double oscSmax, double bandRmsDegs, double dmodMinLim, std::optional<double> since) const {
  Watchdog w;
  const double cut = since ? std::max(lastTs - seconds, *since) : lastTs - seconds;
  for (std::size_t k = t.size(); k-- > 0 && t[k] >= cut;) {
    if (srate[k] > w.srateMax) w.srateMax = srate[k];
    if (dmod[k] < w.dmodMin) w.dmodMin = dmod[k];
    w.samples++;
  }
  if (w.srateMax > oscSmax) w.reasons.push_back("slew rate " + core::jsToFixed(w.srateMax, 1) + " > " + core::jsToString(oscSmax));
  if (w.dmodMin < dmodMinLim) w.reasons.push_back("Dmod " + core::jsToFixed(w.dmodMin, 2) + " (slew limiter cutting gains)");
  // with `since`, the spectral check waits for a full window after it: shorter windows mistake stick taps for oscillation
  const double span = std::max(seconds, 3.0);
  std::optional<engine::AxisData> win;
  if (!since || lastTs - *since >= span) win = window(span);
  if (win && win->n >= 32) {
    const std::size_t ws = std::min<std::size_t>(core::nextPow2(static_cast<double>(win->n)), 256);
    if (win->n >= ws) {
      Vec err(win->n);
      for (std::size_t i = 0; i < win->n; i++) err[i] = win->actual[i] - win->target[i];
      const core::Psd sp = core::psd(err, win->fs, ws, 0.5);
      w.bandRms = core::bandRms(sp, 1.5, win->fs * 0.49);
      double pk = 0;
      for (std::size_t i = 1; i < sp.f.size(); i++)
        if (sp.f[i] >= 1.5 && sp.psd[i] > pk) {
          pk = sp.psd[i];
          w.peakHz = sp.f[i];
        }
      if (w.bandRms > bandRmsDegs && (!baselineOsc || w.bandRms > 3 * *baselineOsc))
        w.reasons.push_back("rate error oscillation " + core::jsToFixed(w.bandRms, 1) + " deg/s RMS at ~" + core::jsToFixed(w.peakHz, 1) + " Hz");
    }
  }
  w.oscillating = !w.reasons.empty();
  return w;
}

void LiveAxis::setBaseline() {
  const Watchdog w = watchdog(5, 1e9, 1e9, -1);
  baselineOsc = (w.bandRms != 0 && !std::isnan(w.bandRms)) ? w.bandRms : 0.5;   // JS: w.bandRms || 0.5
}

}  // namespace aptuner::app
