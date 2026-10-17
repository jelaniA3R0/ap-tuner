#include "aptuner/engine/analysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aptuner/core/fft.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/core/stepresp.hpp"
#include "aptuner/core/synth.hpp"
#include "aptuner/core/timeid.hpp"
#include "aptuner/engine/vehicle.hpp"

namespace aptuner::engine {

using namespace aptuner::core;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDegPerRad = 57.2957795;

double fx(double v, int d) { return jsRoundFixed(v, d); }
double px(double v, int p) { return jsRoundPrecision(v, p); }
std::string tf(double v, int d) { return jsToFixed(v, d); }
std::string ts(double v) { return jsToString(v); }

Json numOrNull(double v) { return std::isnan(v) ? Json() : Json(v); }
Json optNum(const std::optional<double>& v) { return v ? Json(*v) : Json(); }

Json metricsJson(const std::optional<StepMetrics>& m) {
  if (!m) return Json();
  return Json::obj({{"finalValue", m->finalValue},
                    {"peak", m->peak},
                    {"overshootPct", m->overshootPct},
                    {"riseTime", m->riseTime},
                    {"settlingTime", m->settlingTime},
                    {"oscillations", m->oscillations}});
}

Json marginsJson(const Margins& m) {
  return Json::obj({{"gmDb", m.gmDb}, {"pmDeg", m.pmDeg}, {"wcHz", m.wcHz}, {"w180Hz", m.w180Hz}, {"ms", m.ms}});
}

/** PID gains from a JSON gain set with the JavaScript coercions used by the analysis (missing P/I/D -> NaN) */
PidGains pidOf(const Json& g) {
  PidGains p;
  p.P = jv(g, "P");
  p.I = jv(g, "I");
  p.D = jv(g, "D");
  p.FF = jor(g, "FF", 0);
  p.DFF = jor(g, "DFF", 0);
  p.FLTT = jv(g, "FLTT");
  p.FLTE = jv(g, "FLTE");
  p.FLTD = jv(g, "FLTD");
  return p;
}

Json plantFitJson(const PlantFit& f) {
  return Json::obj({{"K", f.K},
                    {"a", f.a},
                    {"tau", f.tau},
                    {"Td", f.Td},
                    {"cost", f.cost},
                    {"nBins", f.nBins},
                    {"fitBand", Json(Json::Array{f.fitBandLo, f.fitBandHi})},
                    {"fitQuality", f.fitQuality},
                    {"dcGain", f.dcGain},
                    {"bandwidthHz", f.bandwidthHz}});
}

double clampTo(double v, const double r[2]) { return std::min(r[1], std::max(r[0], v)); }

Vec everyOther(const Vec& v) {
  Vec o;
  for (std::size_t i = 0; i < v.size(); i += 2) o.push_back(v[i]);
  return o;
}
Json normEveryOther(const Vec& y, double stepMag) {
  Json a = Json::array();
  for (std::size_t i = 0; i < y.size(); i += 2) a.push(fx(y[i] / stepMag, 4));
  return a;
}
Vec scaled(const Vec& y, double k) {
  Vec o(y.size());
  for (std::size_t i = 0; i < y.size(); i++) o[i] = y[i] / k;
  return o;
}

struct IdResult {
  bool ok = false;
  std::string reason;
  PlantFit fit;
  Vec f, coh;
  CVec Hmeas, Hbare, Hcl;
  Vec cohCl, sxx;
  int nWin = 0;
  double fCoh = 0;
  std::size_t win = 0;
};

/** identify the bare plant for one axis (frequency domain) */
IdResult identify(const AxisData& data, const GyroChain& gyro, Family fam, double scaler2) {
  IdResult r;
  const bool plane = fam == Family::Plane;
  const double windowS = plane ? 10.24 : 2.56;
  const double fs = data.fs;
  const std::size_t n = std::min({data.target.size(), data.actual.size(), data.u.size()});
  if (static_cast<double>(n) < fs * 8) {
    r.reason = "not enough data (" + tf(static_cast<double>(n) / fs, 1) + " s)";
    return r;
  }
  std::size_t win = nextPow2(jsRound(windowS * fs));
  win = std::max<std::size_t>(32, std::min(win, nextPow2(static_cast<double>(n) / 4)));
  const double minIn = 2;   // deg/s
  const FreqResponse ry = freqResponse(data.target.data(), data.actual.data(), n, fs, win, 0.75, minIn);
  const FreqResponse ru = freqResponse(data.target.data(), data.u.data(), n, fs, win, 0.75, minIn);
  if (ry.nWin < 8) {
    r.reason = "not enough pilot/autopilot excitation (" + std::to_string(ry.nWin) + " windows)";
    return r;
  }
  const CVec Hmeas = cx::div(ry.H, ru.H);
  Vec coh(ry.f.size());
  for (std::size_t i = 0; i < coh.size(); i++) coh[i] = std::min(ry.coh[i], ru.coh[i]);
  const CVec F = gyroChain(ry.f, gyro);
  const CVec Hbare = cx::scale(cx::div(Hmeas, F), scaler2);
  const double fMax0 = std::min({60.0, fs * 0.4, (gyro.gyroFilterHz > 0 ? gyro.gyroFilterHz * 0.9 : 60.0)});
  // excitation band: highest frequency of the contiguous band (from the lowest bin) with coherence >= 0.3
  double fCoh = 0;
  for (std::size_t i = 1; i < coh.size(); i++) {
    if (ry.f[i] > fMax0) break;
    if (coh[i] >= 0.3)
      fCoh = ry.f[i];
    else if (ry.f[i] > 1.0)
      break;
  }
  const double fMax = std::min(fMax0, std::max(fCoh * 1.1, ry.f[1] * 6));
  FitPlantOptions fo;
  fo.kind = plane ? "plane" : "copter";
  fo.nWin = ry.nWin;
  fo.fMin = std::max(plane ? 0.1 : 0.3, ry.f[1]);
  fo.fMax = fMax;
  const auto fit = fitPlant(ry.f, Hbare, coh, fo);
  r.f = ry.f;
  r.coh = coh;
  r.Hbare = Hbare;
  if (!fit) {
    r.reason = "coherence too low to fit a model: fly some sharp stick inputs";
    return r;
  }
  r.ok = true;
  r.fit = *fit;
  r.Hmeas = Hmeas;
  r.nWin = ry.nWin;
  r.fCoh = fCoh;
  r.win = win;
  r.sxx = ry.Sxx;
  r.Hcl = ry.H;
  r.cohCl = ry.coh;
  return r;
}

struct Calibration {
  double Td = 0, tau = 0;
  bool calibrated = false;
  std::optional<std::pair<double, double>> mismatch;   // magPct, phaseDeg
};

/** closed-loop match of the identified model (diagnostic; the fit's own lag is used for the design) */
Calibration calibrateLag(const IdResult& id, double fsLoop, const GyroChain& gyro, const Json& cur, double us,
                         double tdFloor, double tauFloor) {
  const Vec& f = id.f;
  const CVec& H = id.Hcl;
  const Vec& coh = id.cohCl;
  std::vector<std::size_t> idx;
  for (std::size_t i = 1; i < f.size(); i++)
    if (f[i] <= id.fCoh && coh[i] >= 0.3) idx.push_back(i);
  const PidGains g = pidOf(cur);
  Calibration c;
  if (idx.size() < 4 || !(g.P > 0)) {
    c.Td = std::max(id.fit.Td, tdFloor);
    c.tau = std::max(id.fit.tau, tauFloor);
    return c;
  }
  Vec fsub(idx.size());
  for (std::size_t k = 0; k < idx.size(); k++) fsub[k] = f[idx[k]];
  struct Cost {
    double c, magPct, phaseDeg;
  };
  auto cost = [&](double Td, double tau) {
    LoopCtx ctx;
    ctx.fsLoop = fsLoop;
    ctx.gyro = gyro;
    ctx.plant = Plant{id.fit.K / us, id.fit.a, tau, Td};
    const ClosedLoop cl = closedLoop(fsub, ctx, g);
    double cc = 0, cm = 0, cp = 0, w = 0;
    for (std::size_t k = 0; k < idx.size(); k++) {
      const std::size_t i = idx[k];
      const double mm = std::hypot(H.re[i], H.im[i]), mp = std::atan2(H.im[i], H.re[i]);
      const double tm = std::hypot(cl.Tcl.re[k], cl.Tcl.im[k]), tp = std::atan2(cl.Tcl.im[k], cl.Tcl.re[k]);
      const double em = std::log((tm + 1e-9) / (mm + 1e-9));
      double ep = tp - mp;
      ep = std::atan2(std::sin(ep), std::cos(ep));
      cc += coh[i] * (em * em + 2 * ep * ep);
      cm += coh[i] * std::fabs(em);
      cp += coh[i] * std::fabs(ep);
      w += coh[i];
    }
    return Cost{cc / w, 100 * (std::exp(cm / w) - 1), cp / w * 180 / kPi};
  };
  const Cost fitCost = cost(id.fit.Td, id.fit.tau);
  c.Td = id.fit.Td;
  c.tau = id.fit.tau;
  c.calibrated = true;
  c.mismatch = std::make_pair(fitCost.magPct, fitCost.phaseDeg);
  return c;
}


Json slimTd(const TimeIdFit& f) {
  auto r = [](double v) { return px(v, 4); };
  return Json::obj({{"bDegs2", r(f.K)},
                    {"bRads2", r(f.K / kDegPerRad)},
                    {"a", r(f.a)},
                    {"tau", r(f.tau)},
                    {"Td", r(f.Td)},
                    {"uLimit", optNum(f.uLimit)},
                    {"limitPct", r(f.limitPct)},
                    {"trimMean", r(f.trimMean)},
                    {"trimMin", r(f.trimMin)},
                    {"trimMax", r(f.trimMax)},
                    {"r2", r(f.r2)},
                    {"vaf", r(f.vaf)},
                    {"bRelErr", r(f.bRelErr)},
                    {"method", f.method},
                    {"airborneS", r(f.airborneS)},
                    {"segments", f.segments},
                    {"fitQuality", r(f.fitQuality)}});
}

Json gainsJson(const PidGains& g) {
  return Json::obj({{"FLTT", g.FLTT}, {"FLTE", g.FLTE}, {"FLTD", g.FLTD}, {"FF", g.FF}, {"DFF", g.DFF},
                    {"P", g.P}, {"I", g.I}, {"D", g.D}});
}

}  // namespace

// ------------------------------------------------------------------------------------------------ helpers

GyroChain gyroChainFromParams(const log::Params& params, const NotchOp& op) {
  GyroChain g;
  const int rate = params.intOf("INS_GYRO_RATE");
  g.backendHz = 1000 * std::pow(2.0, std::max(0, rate));
  for (const char* pre : {"INS_HNTCH_", "INS_HNTC2_"}) {
    const std::string p = pre;
    if (!(params.getOr(p + "ENABLE", kNaN) > 0)) continue;
    NotchParams np;
    np.mode = params.intOf(p + "MODE");
    np.freq = params.truthyOr(p + "FREQ", 0);
    np.ref = params.getOr(p + "REF", 0);
    np.fmRat = params.getOr(p + "FM_RAT", 1);
    NotchDef n;
    n.enable = true;
    n.freq = notchBaseFreq(np, op);
    n.bw = params.truthyOr(p + "BW", 0);
    n.att = params.getOr(p + "ATT", 40);
    n.hmncs = static_cast<int>(params.getOr(p + "HMNCS", 3));
    n.opts = params.intOf(p + "OPTS");
    g.notches.push_back(n);
  }
  g.gyroFilterHz = params.getOr("INS_GYRO_FILTER", 20);
  return g;
}

Json gyroChainJson(const GyroChain& g) {
  Json notches = Json::array();
  for (const auto& n : g.notches)
    notches.push(Json::obj({{"enable", 1}, {"freq", n.freq}, {"bw", n.bw}, {"att", n.att}, {"hmncs", n.hmncs}, {"opts", n.opts}}));
  return Json::obj({{"backendHz", g.backendHz}, {"gyroFilterHz", g.gyroFilterHz}, {"notches", notches}});
}

std::optional<Resampled> resample(const Vec& t, const std::vector<const Vec*>& cols, double fs) {
  const std::size_t n = t.size();
  if (n < 4) return std::nullopt;
  const double t0 = t[0], t1 = t[n - 1];
  const std::size_t m = static_cast<std::size_t>(std::floor((t1 - t0) * fs)) + 1;
  Resampled r;
  r.fs = fs;
  r.t0 = t0;
  r.n = m;
  r.cols.assign(cols.size(), Vec(m));
  std::size_t j = 0;
  for (std::size_t i = 0; i < m; i++) {
    const double tt = t0 + static_cast<double>(i) / fs;
    while (j < n - 2 && t[j + 1] < tt) j++;
    const double dt = t[j + 1] - t[j];
    const double a = dt > 0 ? std::min(1.0, std::max(0.0, (tt - t[j]) / dt)) : 0;
    for (std::size_t k = 0; k < cols.size(); k++) {
      const Vec& c = *cols[k];
      r.cols[k][i] = c[j] + a * (c[j + 1] - c[j]);
    }
  }
  return r;
}

double estimateRate(const double* t, std::size_t n) {
  if (n < 3) return 0;
  const double span = t[n - 1] - t[0];
  return span > 0 ? static_cast<double>(n - 1) / span : 0;
}

Json currentGains(Family family, Axis axis, const log::Params& params) {
  Json g = axisGains(family, axis, params);
  const FamilyDef& f = familyDef(family);
  const int a = static_cast<int>(axis);
  auto put = [&](const char* key, const char* name) {
    if (!name) return;
    auto v = params.get(name);
    if (v) g.set(key, *v);
  };
  put("angleP", f.angleP[a]);
  put("accMax", f.accMax[a]);
  put("tconst", f.tconst[a]);
  return g;
}

Json describeChanges(const Json& cur, const Json& nw) {
  Json out = Json::array();
  for (const char* k : {"P", "I", "D", "FF", "FLTT", "FLTE", "FLTD", "SMAX", "IMAX", "angleP", "tconst"}) {
    if (!nw[k].isNumber() || !cur[k].isNumber()) continue;
    const double a = cur[k].num(), b = nw[k].num();
    if (std::fabs(a - b) <= 1e-9) continue;
    out.push(Json::obj({{"key", k}, {"from", a}, {"to", b}, {"pct", a != 0 ? Json((b / a - 1) * 100) : Json()}}));
  }
  return out;
}

// ------------------------------------------------------------------------------------------------ diagnostics

Json diagnostics(const AxisData& data, const Json& cur, Family family, Axis axis) {
  (void)axis;
  const double fs = data.fs;
  const std::size_t n = data.target.size();
  Vec err(n);
  double eSq = 0, tSq = 0;
  for (std::size_t i = 0; i < n; i++) {
    err[i] = data.target[i] - data.actual[i];
    eSq += err[i] * err[i];
    tSq += data.target[i] * data.target[i];
  }
  const double rmsErr = std::sqrt(eSq / static_cast<double>(n)), rmsTarget = std::sqrt(tSq / static_cast<double>(n));
  Json out = Json::obj({{"rmsErrorDegs", rmsErr},
                        {"rmsTargetDegs", rmsTarget},
                        {"trackingPct", rmsTarget > 1 ? Json(100 * std::max(0.0, 1 - rmsErr / rmsTarget)) : Json()}});
  // spectral content of the actual rate: oscillation band vs control band
  const std::size_t win = std::max<std::size_t>(64, std::min<std::size_t>(1024, nextPow2(fs)));
  if (n >= win * 2) {
    const Psd spA = psd(data.actual.data(), n, fs, win, 0.5), spE = psd(err.data(), n, fs, win, 0.5),
              spT = psd(data.target.data(), n, fs, win, 0.5);
    const double fHi = std::min(30.0, fs * 0.45);
    const double oscRms = bandRms(spE, 2, fHi);   // error energy in the oscillation band
    out.set("oscRmsDegs", oscRms);
    out.set("lowRmsDegs", bandRms(spA, 0.05, 2));
    // dominant peak in the error spectrum above 1 Hz
    double pk = 0, pf = 0;
    std::size_t pi = 0;
    for (std::size_t i = 1; i < spE.f.size(); i++)
      if (spE.f[i] > 1 && spE.f[i] <= fHi && spE.psd[i] > pk) {
        pk = spE.psd[i];
        pf = spE.f[i];
        pi = i;
      }
    Vec sorted(spE.psd.begin() + 1, spE.psd.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t mi = spE.psd.size() / 2;
    double med = mi < sorted.size() ? sorted[mi] : kNaN;
    if (!(med != 0) || std::isnan(med)) med = 1e-12;
    out.set("peakHz", pf);
    const double prominence = pk / med;
    out.set("peakProminence", prominence);
    const double targetAtPeak = spT.psd[pi] != 0 && !std::isnan(spT.psd[pi]) ? spT.psd[pi] : 0;
    const double peakVsTarget = targetAtPeak > 0 ? pk / targetAtPeak : std::numeric_limits<double>::infinity();
    out.set("peakVsTarget", peakVsTarget);
    out.set("oscillating", prominence > 40 && oscRms > 3 && peakVsTarget > 4);
    out.set("noiseRmsDegs", bandRms(spA, std::min(20.0, fs * 0.3), fs * 0.49));
  }
  // PID term shares and saturation
  if (data.P && data.D && data.I) {
    double p2 = 0, d2 = 0, iSum = 0, sat = 0;
    const double uMax = family == Family::Plane ? 44 : 0.98;
    for (std::size_t i = 0; i < n; i++) {
      p2 += (*data.P)[i] * (*data.P)[i];
      d2 += (*data.D)[i] * (*data.D)[i];
      iSum += (*data.I)[i];
      if (std::fabs(data.u[i]) >= uMax) sat++;
    }
    out.set("dOverP", p2 > 0 ? std::sqrt(d2 / p2) : 0.0);
    out.set("iMean", iSum / static_cast<double>(n));
    out.set("saturationPct", 100 * sat / static_cast<double>(n));
  }
  if (data.srate) {
    double m = 0, c = 0;
    for (std::size_t i = 0; i < n; i++) {
      if ((*data.srate)[i] > m) m = (*data.srate)[i];
      if ((*data.srate)[i] > 3) c++;
    }
    out.set("srateMax", m);
    out.set("sratePctAbove3", 100 * c / static_cast<double>(n));
  }
  if (data.dmod) {
    double m = 1;
    for (std::size_t i = 0; i < n; i++)
      if ((*data.dmod)[i] < m) m = (*data.dmod)[i];
    out.set("dmodMin", m);
  }
  out.set("saturation", saturationCheck(data, cur, family, out));
  out.set("grade", gradeAxis(out));
  return out;
}

Json saturationCheck(const AxisData& data, const Json& cur, Family family, const Json& d) {
  if (family == Family::Plane) return Json();
  const std::size_t n = data.target.size();
  const bool hasAir = !data.airborne.empty();
  double sT = 0, sA = 0, sU = 0, c = 0;
  for (std::size_t i = 0; i < n; i++) {
    if (hasAir && !data.airborne[i]) continue;
    sT += data.target[i];
    sA += data.actual[i];
    sU += data.u[i];
    c++;
  }
  if (c < 10) return Json();
  const double imax = jor(cur, "IMAX", kNaN);
  Json r = Json::obj({{"meanDemandDegs", sT / c},
                      {"meanActualDegs", sA / c},
                      {"steadyErrorDegs", (sT - sA) / c},
                      {"uMean", sU / c},
                      {"imax", numOrNull(imax)}});
  double iMean = kNaN, iP95 = kNaN;
  bool haveI = false;
  // I term: from the PID columns when present, else from the (slower) PID log summary
  if (data.I) {
    Vec v;
    double m = 0;
    for (std::size_t i = 0; i < n; i++) {
      if (hasAir && !data.airborne[i]) continue;
      v.push_back(std::fabs((*data.I)[i]));
      m += (*data.I)[i];
    }
    std::sort(v.begin(), v.end());
    iMean = m / static_cast<double>(v.size());
    const std::size_t k = static_cast<std::size_t>(std::floor(static_cast<double>(v.size()) * 0.95));
    iP95 = k < v.size() && v[k] != 0 ? v[k] : 0;
    haveI = true;
  } else if (data.iStats.isObject()) {
    iMean = jv(data.iStats, "mean");
    iP95 = jv(data.iStats, "p95");
    haveI = true;
  }
  if (haveI) {
    r.set("iMean", numOrNull(iMean));
    r.set("iP95", numOrNull(iP95));
  }
  const double steady = (sT - sA) / c;
  const double rmsTarget = jor(d, "rmsTargetDegs", 0);
  const bool persistent = std::fabs(steady) > 8 && std::fabs(steady) > 0.4 * rmsTarget;
  const bool iNearMax = imax > 0 && haveI && (std::fabs(iMean) > 0.7 * imax || iP95 > 0.85 * imax);
  r.set("persistent", persistent);
  r.set("iNearMax", iNearMax);
  const Json& m = data.motors;
  const double pinnedTopPct = m.isObject() ? jv(m, "pinnedTopPct") : kNaN;
  r.set("pinnedPct", m.isObject() ? numOrNull(pinnedTopPct) : Json());
  const bool pinned = m.isObject() && pinnedTopPct > 1;
  r.set("pinned", pinned);
  if (m.isObject() && jor(m, "ccwMean", 0) != 0) {
    r.set("ccwMean", m["ccwMean"]);
    r.set("cwMean", m["cwMean"]);
    r.set("maxPwm", m["maxPwm"]);
  }
  const bool saturated = (persistent && (iNearMax || pinned)) || (iNearMax && pinned);
  const double iMeanOr0 = (iMean != 0 && !std::isnan(iMean)) ? iMean : 0;
  const double imaxOr1 = (imax != 0 && !std::isnan(imax)) ? imax : 1;
  r.set("saturated", saturated);
  r.set("trimHeavy", !saturated && (iNearMax || std::fabs(iMeanOr0) > 0.6 * imaxOr1));
  return r;
}

Json gradeAxis(const Json& d, const GradeCtx& ctx) {
  std::vector<std::string> notes;
  double score = 100;
  const double peakHz = jv(d, "peakHz"), oscRms = jv(d, "oscRmsDegs");
  const Json& tr = d["trackingPct"];
  if (d["oscillating"].asBool()) {
    score -= 40;
    notes.push_back(peakHz > 15 ? "Strong " + tf(peakHz, 1) + " Hz content in the rate error (" + tf(oscRms, 1) +
                                      " deg/s RMS): at this frequency it is almost certainly motor/vibration noise "
                                      "reaching the controller, not a tuning oscillation. Set up the harmonic notch "
                                      "(Log tab, noise section) before tuning."
                                : "Oscillation at " + tf(peakHz, 1) + " Hz (" + tf(oscRms, 1) + " deg/s RMS)");
  } else if (oscRms > 4) {
    score -= 15;
    notes.push_back("Noticeable high-frequency error energy");
  }
  if (!tr.isNull()) {
    const double t = jv(d, "trackingPct");
    if (t < 50) {
      score -= 25;
      notes.push_back("Poor rate tracking (" + tf(t, 0) + "%)");
    } else if (t < 70) {
      score -= 10;
      notes.push_back("Soft rate tracking (" + tf(t, 0) + "%)");
    }
  }
  const double satPct = jv(d, "saturationPct");
  if (satPct > 2) {
    score -= 15;
    notes.push_back("Actuator saturated " + tf(satPct, 1) + "% of the time");
  }
  if (jv(d, "dOverP") > 1.5) {
    score -= 10;
    notes.push_back("D term dominates P (noisy D / too much D)");
  }
  const double srateMax = jv(d, "srateMax");
  if (srateMax > 5) {
    score -= 10;
    notes.push_back("Slew-rate limiter active (max " + tf(srateMax, 1) + ")");
  }
  if (d.has("dmodMin") && jv(d, "dmodMin") < 0.9) {
    score -= 10;
    notes.push_back("Gains were reduced by the slew limiter (Dmod < 0.9)");
  }
  if (std::fabs(jor(d, "iMean", 0)) > 0.15) notes.push_back("Large steady I term: trim / CG / motor imbalance");
  const Json& sat = d["saturation"];
  auto dropTracking = [&]() {
    notes.erase(std::remove_if(notes.begin(), notes.end(),
                               [](const std::string& s) { return s.find("rate tracking") != std::string::npos; }),
                notes.end());
  };
  auto notesJson = [&]() {
    Json a = Json::array();
    for (const auto& s : notes) a.push(s);
    return a;
  };
  if (sat.isObject() && sat["saturated"].asBool()) {
    dropTracking();
    score = std::min(score, 25.0);
    std::string s = "Saturated: the airframe does not follow a steady " + tf(jv(sat, "steadyErrorDegs"), 0) + " deg/s demand";
    if (sat["iNearMax"].asBool())
      s += ", the I term sits at " + tf(std::fabs(jv(sat, "iMean")), 2) + " of its " + ts(jv(sat, "imax")) + " limit";
    if (sat["pinned"].asBool()) s += ", motors at their limit " + tf(jv(sat, "pinnedPct"), 1) + "% of the time";
    s += ". Mechanical / torque imbalance: tuning cannot fix this.";
    notes.insert(notes.begin(), s);
  } else if (ctx.idOk.has_value() && (tr.isNull() || jv(d, "trackingPct") < 50)) {
    // little stick input: the demanded rate is small next to what gusts / trim add, so the tracking says nothing
    if (!*ctx.idOk || jor(d, "rmsTargetDegs", 0) < 20) {
      dropTracking();
      notes.insert(notes.begin(), "Not gradable: too little stick input. The demanded rate (" +
                                      tf(jor(d, "rmsTargetDegs", 0), 1) +
                                      " deg/s RMS) is smaller than what gusts and trim add (" +
                                      tf(jor(d, "rmsErrorDegs", 0), 1) +
                                      " deg/s RMS error), so the tracking cannot be judged. Fly sharp stick taps on "
                                      "this axis.");
      return Json::obj({{"score", Json()}, {"letter", "\xE2\x80\x93"}, {"gradable", false}, {"notes", notesJson()}});
    }
    score = std::min(score, 50.0);
  }
  if (ctx.idOk.has_value() && !*ctx.idOk && score >= 70) {
    score = 69;
    notes.push_back("No airframe model could be measured, so the grade is capped at C");
  }
  if (ctx.lowConf && score >= 70) {
    score = 69;
    notes.push_back("Only a lower-confidence model was measured, so the grade is capped at C");
  }
  const char* letter = score >= 85 ? "A" : score >= 70 ? "B" : score >= 55 ? "C" : score >= 40 ? "D" : "F";
  return Json::obj({{"score", std::max(0.0, score)}, {"letter", letter}, {"gradable", true}, {"notes", notesJson()}});
}

Json planeFFEvents(const AxisData& data) {
  if (!data.P || !data.FF) return Json();
  const std::size_t n = data.target.size();
  const double fs = data.fs;
  struct Lp {
    double al, s = 0;
    double operator()(double v) {
      s += al * (v - s);
      return s;
    }
  };
  auto lp = [&](double fc) { return Lp{1 / (1 + fs / (2 * kPi * fc))}; };
  Lp fa = lp(0.75), fr = lp(0.75), ftg = lp(4);
  Vec absT(n);
  for (std::size_t i = 0; i < n; i++) absT[i] = std::fabs(data.target[i]);
  std::sort(absT.begin(), absT.end());
  const std::size_t k95 = static_cast<std::size_t>(std::floor(static_cast<double>(n) * 0.95));
  const double p95 = k95 < n && absT[k95] != 0 ? absT[k95] : 0;
  if (p95 < 10) return Json::obj({{"ff", Json()}, {"events", 0}, {"reason", "too little rate demand"}});
  const double thr1 = 0.4 * p95, thr2 = 0.25 * thr1;
  int state = 0;
  double maxA = 0, minA = 0, maxR = 0, minR = 0;
  Vec ffs;
  for (std::size_t i = 0; i < n; i++) {
    const double Ii = data.I ? (*data.I)[i] : 0;
    const double u = (*data.P)[i] + (data.D ? (*data.D)[i] : 0) + (*data.FF)[i] + Ii;
    const double clipped = std::max(-45.0, std::min(45.0, u)) - Ii;
    const double act = fa(clipped), rat = fr(data.actual[i]), r = ftg(data.target[i]);
    const double ss = data.ss ? (*data.ss)[i] : std::sqrt(data.scaler2 != 0 ? data.scaler2 : 1);
    if (state == 0) {
      if (r > thr1) {
        state = 1;
        maxA = minA = maxR = minR = 0;
      } else if (r < -thr1) {
        state = -1;
        maxA = minA = maxR = minR = 0;
      }
    } else {
      maxA = std::max(maxA, act);
      minA = std::min(minA, act);
      maxR = std::max(maxR, rat);
      minR = std::min(minR, rat);
      if ((state == 1 && r < thr2) || (state == -1 && r > -thr2)) {
        const double ff = state == 1 ? maxA / (maxR * ss) : minA / (minR * ss);
        if (std::isfinite(ff) && ff > 0 && ff < 5) ffs.push_back(ff);
        state = 0;
      }
    }
  }
  std::sort(ffs.begin(), ffs.end());
  const double m = static_cast<double>(ffs.size());
  if (ffs.size() < 6)
    return Json::obj({{"ff", Json()}, {"events", ffs.size()}, {"reason", "fewer than 6 usable stick events"}});
  return Json::obj({{"ff", ffs[static_cast<std::size_t>(std::floor(m / 2))]},
                    {"p25", ffs[static_cast<std::size_t>(std::floor(m * 0.25))]},
                    {"p75", ffs[static_cast<std::size_t>(std::floor(m * 0.75))]},
                    {"events", ffs.size()}});
}

// ------------------------------------------------------------------------------------------------ analyzeAxis

Json analyzeAxis(const AnalyzeArgs& A) {
  const AxisData& data = *A.data;
  const log::Params& params = *A.params;
  const Family family = A.family;
  const Family fam = family == Family::Heli ? Family::Copter : family;
  const bool plane = fam == Family::Plane;
  const Axis axis = A.axis;
  const std::string axisN = axisName(axis);
  const Preset& pre = preset(A.preset);
  const double loopHz = A.fsLoop != 0 ? A.fsLoop : params.truthyOr("SCHED_LOOP_RATE", plane ? 50 : 400);
  const GyroChain gyro = gyroChainFromParams(params, A.gyroOp);
  const Json cur = currentGains(family, axis, params);
  Json res = Json::obj({{"family", familyName(family)},
                        {"axis", axisN},
                        {"preset", A.preset},
                        {"loopHz", loopHz},
                        {"gyro", gyroChainJson(gyro)},
                        {"current", cur},
                        {"warnings", Json::array()}});
  auto warn = [&](const std::string& s) { res.at("warnings").push(s); };
  // measured closed-loop step response (data-driven, no model)
  StepResponseOptions so;
  so.windowSize = std::max<std::size_t>(64, std::min<std::size_t>(1024, nextPow2(data.fs * (plane ? 5.12 : 1.28))));
  so.durationS = plane ? 2.0 : 0.5;
  so.minInput = 15;
  const StepResponse sr = stepResponse(data.target, data.actual, data.fs, so);
  if (sr.count > 0)
    res.set("measuredStep", Json::obj({{"t", Json(sr.t)},
                                       {"y", Json(sr.mean)},
                                       {"count", sr.count},
                                       {"metrics", metricsJson(stepMetrics(sr.t, sr.mean))}}));
  // diagnostics on the raw data
  res.set("diagnostics", diagnostics(data, cur, fam, axis));
  if (plane) res.set("ffEvents", planeFFEvents(data));
  // identification
  const IdResult id = identify(data, gyro, fam, A.scaler2);
  if (id.ok)
    res.set("identification", Json::obj({{"fit", plantFitJson(id.fit)}, {"nWin", id.nWin}, {"fCoh", id.fCoh}, {"win", id.win}}));
  else
    res.set("identification", Json::obj({{"ok", false}, {"reason", id.reason}}));
  if (id.ok) {
    std::vector<std::size_t> keep;
    for (std::size_t i = 1; i < id.f.size(); i++)
      if (id.f[i] <= 100) keep.push_back(i);
    Vec fk(keep.size());
    for (std::size_t k = 0; k < keep.size(); k++) fk[k] = id.f[keep[k]];
    const CVec Gfit = evalPlant(fk, id.fit);
    Json bf = Json::array(), magM = Json::array(), phM = Json::array(), cohJ = Json::array(), magF = Json::array(),
         phF = Json::array();
    for (std::size_t k = 0; k < keep.size(); k++) {
      const std::size_t i = keep[k];
      bf.push(fx(id.f[i], 3));
      magM.push(px(std::hypot(id.Hbare.re[i], id.Hbare.im[i]), 4));
      phM.push(fx(std::atan2(id.Hbare.im[i], id.Hbare.re[i]) * 180 / kPi, 1));
      cohJ.push(fx(id.coh[i], 3));
      magF.push(px(std::hypot(Gfit.re[k], Gfit.im[k]), 4));
      phF.push(fx(std::atan2(Gfit.im[k], Gfit.re[k]) * 180 / kPi, 1));
    }
    Json& idn = res.at("identification");
    idn.set("bode", Json::obj({{"f", bf}, {"magMeas", magM}, {"phMeas", phM}, {"coh", cohJ}, {"magFit", magF}, {"phFit", phF}}));
    bool ok = id.fit.fitQuality >= 0.4 && id.fCoh >= (plane ? 0.8 : 1.5);
    if (id.fit.Td >= 0.075) {
      ok = false;
      warn("The delay estimate hit its limit (" + tf(id.fit.Td * 1000, 0) + " ms): the data rate (" + ts(data.fs) +
           " Hz) is too low to resolve the loop dynamics. Log/stream the PID data faster (copter: LOG_BITMASK fast "
           "attitude; live: a faster link) before trusting P and D.");
    }
    idn.set("ok", ok);
    if (!ok)
      warn("Model fit is weak (quality " + tf(id.fit.fitQuality, 2) + ", trusted to " + tf(id.fCoh, 1) +
           " Hz). Fly sharper stick inputs or use SystemID mode.");
  }
  // ArduPilot copter / VTOL / heli rate PIDs work in rad/s, plane in deg/s: the plant is expressed per gain unit
  const double us = plane ? 1 : kDegPerRad;
  std::optional<Plant> plant;
  bool lowConf = false;
  std::optional<TimeIdFit> tdFit;
  if (res["identification"]["ok"].asBool()) {
    const double tdFloor = 1 / loopHz + (plane ? 0.015 : 0.003), tauFloor = plane ? 0.03 : 0.005;
    const Calibration cal = calibrateLag(id, loopHz, gyro, cur, us, tdFloor, tauFloor);
    plant = Plant{id.fit.K / us, id.fit.a, cal.tau, cal.Td};
    Json& idn = res.at("identification");
    idn.set("method", "frequency response");
    Json mismatch = cal.mismatch ? Json::obj({{"magPct", cal.mismatch->first}, {"phaseDeg", cal.mismatch->second}}) : Json();
    idn.set("assumed", Json::obj({{"Td", plant->Td},
                                  {"tau", plant->tau},
                                  {"floors", id.fit.Td < tdFloor || id.fit.tau < tauFloor},
                                  {"closedLoopMismatch", mismatch},
                                  {"calibrated", cal.calibrated}}));
    if (cal.mismatch && cal.mismatch->first > 40)
      warn("Model vs measured closed-loop mismatch " + tf(cal.mismatch->first, 0) + "% / " +
           tf(cal.mismatch->second, 0) + " deg: treat the proposal with care.");
  } else if (A.timeDomain && !plane) {
    // fallback for ordinary hovers with little stick input: the time-domain (hover-data) fit, lower confidence
    TimeIdInput tin;
    tin.fs = data.fs;
    tin.actual = data.actual;
    tin.u = data.u;
    tin.airborne = data.airborne;
    TimeIdOptions to;
    to.axis = axis;
    const TimeIdResult td = identifyTimeDomain(tin, to);
    if (td.fit) {
      Json tdj = Json::obj({{"ok", td.ok}, {"reason", td.reason.empty() ? Json() : Json(td.reason)}});
      const Json slim = slimTd(*td.fit);
      for (const auto& [k, v] : slim.items()) tdj.set(k, v);
      res.set("timeDomain", tdj);
    } else {
      res.set("timeDomain", Json::obj({{"ok", false}, {"reason", td.reason}}));
    }
    if (td.ok) {
      tdFit = *td.fit;
      lowConf = true;
      // a delay at either end of the search grid was not resolved: use a cautious value instead
      const bool tdUnresolved = tdFit->Td <= 0.005 || tdFit->Td >= 0.095;
      plant = Plant{tdFit->K / us, tdFit->a, std::max(tdFit->tau, 0.02),
                    tdUnresolved ? 0.08 : std::max(tdFit->Td, 1 / loopHz + 0.003)};
      const std::string idReason = id.reason;
      const std::string resReason = res["identification"]["reason"].strOr("");
      Json reasonFreq = !idReason.empty() ? Json(idReason) : !resReason.empty() ? Json(resReason) : Json();
      res.set("identification",
              Json::obj({{"ok", true},
                         {"method", "time domain (hover data)"},
                         {"confidence", "lower"},
                         {"reasonFreq", reasonFreq},
                         {"fit", Json::obj({{"K", tdFit->K},
                                            {"a", tdFit->a},
                                            {"tau", tdFit->tau},
                                            {"Td", tdFit->Td},
                                            {"fitQuality", tdFit->fitQuality},
                                            {"uLimit", optNum(tdFit->uLimit)},
                                            {"trimMean", tdFit->trimMean}})},
                         {"assumed", Json::obj({{"Td", plant->Td}, {"tau", plant->tau}, {"floors", tdUnresolved}, {"calibrated", false}})}}));
      warn("Lower-confidence model: the flight had too little stick input for the normal measurement, so the "
           "airframe was fitted from the ordinary hover data (it replays " +
           tf(std::max(tdFit->vaf, tdFit->r2) * 100, 0) + "% of the measured rate, control power +-" +
           tf(tdFit->bRelErr * 100, 0) +
           "%). The proposal keeps 30% of the oscillation gain. Confirm with a flight with sharp stick taps.");
    } else {
      Json& idn = res.at("identification");
      const std::string prev = idn["reason"].strOr("");
      idn.set("reason", (prev.empty() ? "" : prev + "; ") + "hover-data fit: " + td.reason);
    }
  }
  if (plant) {
    const double stepMag = 30 / us;
    LoopCtx ctx;
    ctx.fsLoop = loopHz;
    ctx.gyro = gyro;
    ctx.plant = *plant;
    const GainLimits lim = gainLimits(fam, axis);
    AxisGainsIn ag{jv(cur, "FLTT"), jv(cur, "FLTE"), jv(cur, "FLTD")};
    const Filters filters = filterRules(fam, axis, ag, gyro.gyroFilterHz, jor(cur, "tconst", 0.5));
    Psd noise = psd(data.actual, data.fs, std::max<std::size_t>(64, std::min<std::size_t>(1024, nextPow2(data.fs * 1))), 0.5);
    for (auto& v : noise.psd) v = v / (us * us);
    // a yaw axis that runs near its mixer limit only has the headroom above the trim left for control
    const double uMax = plane ? 45
                              : (tdFit && tdFit->uLimit ? std::max(0.1, *tdFit->uLimit - std::fabs(tdFit->trimMean)) : 1);
    const bool hasFF = plane || family == Family::Heli || jv(cur, "FF") > 0;
    // FF from the identified DC gain
    double FF = 0;
    if (hasFF) FF = std::min(lim.FF[1], std::max(lim.FF[0], std::sqrt(A.scaler2) / std::max(plant->K / plant->a, 1e-6)));
    const Json& ffe = res["ffEvents"];
    if (plane && ffe.isObject() && jor(ffe, "ff", 0) != 0) {
      res.set("ffModel", FF);
      FF = std::min(lim.FF[1], std::max(lim.FF[0], jv(ffe, "ff")));
    }
    const double iRatio = plane ? 0 : (hasFF ? 0 : (axis == Axis::Yaw ? 0.1 : 1.0));
    // lower-confidence model: conservative margins and 30% of the oscillation gain; the search may go past the
    // vehicle's normal limit so the full model design can be shown (the proposal is clamped)
    const double stepDurS = lowConf && plant->a < 1 ? 6 : 1.0;
    const Preset& cons = kPresets[0];
    SynthOptions sopt;
    if (lowConf) {
      sopt.gmMinDb = cons.gmMinDb;
      sopt.pmMinDeg = cons.pmMinDeg;
      sopt.msMax = cons.msMax;
      sopt.osMaxPct = cons.osMaxPct;
      sopt.backoff = std::min(pre.backoff, cons.backoff);
    } else {
      sopt.gmMinDb = pre.gmMinDb;
      sopt.pmMinDeg = pre.pmMinDeg;
      sopt.msMax = pre.msMax;
      sopt.osMaxPct = pre.osMaxPct;
      sopt.backoff = pre.backoff;
    }
    sopt.pRange[0] = lim.P[0];
    sopt.pRange[1] = lowConf ? lim.P[1] * 40 : lim.P[1];
    sopt.stepDurS = stepDurS;
    sopt.dRange[0] = lim.D[0];
    sopt.dRange[1] = lim.D[1];
    sopt.iRatio = iRatio != 0 ? iRatio : 1e-6;
    sopt.filters.FLTT = filters.FLTT;
    sopt.filters.FLTE = filters.FLTE;
    sopt.filters.FLTD = filters.FLTD;
    sopt.uMax = uMax;
    sopt.dNoiseCap = plane ? 1.5 : 0.04;
    sopt.noisePsd = NoisePsd{noise.f, noise.psd};
    sopt.noisePsdIsFiltered = true;
    sopt.rMax = plane ? 0.12 : 0.08;
    sopt.stepMag = stepMag;
    const double dfP = A.designFrom.isObject() ? jv(A.designFrom, "P") : kNaN;
    sopt.currentP = dfP > 0 ? dfP : jv(cur, "P");
    sopt.currentD = A.designFrom.isObject() ? jv(A.designFrom, "D") : jv(cur, "D");
    if (std::isnan(sopt.currentP)) sopt.currentP = 0;
    if (std::isnan(sopt.currentD)) sopt.currentD = 0;
    std::optional<SynthResult> syn = synthesizeQuik(ctx, sopt);
    if (!syn || !(syn->gains.P > 0)) syn = synthesize(ctx, sopt);
    if (!syn) {
      warn("No stable gain set found within limits; check the model fit.");
    } else {
      PidGains g = syn->gains;
      double modelP = g.P, modelD = g.D;
      Json headroomNote;
      Margins designMargins = syn->margins;
      // a weak axis can be stable with a huge P on the model: size P by the mixer range instead
      if (lowConf) {
        const double uRange = tdFit->uLimit ? *tdFit->uLimit : std::min(1.0, tdFit->uMaxAbs != 0 ? tdFit->uMaxAbs : 1.0);
        const double pHead = uRange / (10 / kDegPerRad);
        const Vec fq = logspace(0.005, std::min(loopHz / 2, 400.0), 500);
        if (modelP > pHead) {
          headroomNote = "limited by the mixer range (" + ts(fx(uRange, 2)) +
                         " of output for a 10 deg/s error), the model alone would allow " +
                         (modelP >= sopt.pRange[1] * 0.999 ? "more than " : "") + jsToPrecision(modelP, 3);
          modelP = pHead;
          modelD = std::min(modelD, lim.D[1]);
          PidGains gd = g;
          gd.P = modelP;
          gd.D = modelD;
          gd.I = iRule(fam, axis, modelP, FF, hasFF);
          designMargins = margins(fq, openLoop(fq, ctx, gd).L);
        } else {
          PidGains gd = g;
          gd.I = iRule(fam, axis, g.P, FF, hasFF);
          designMargins = margins(fq, openLoop(fq, ctx, gd).L);
        }
        g.P = modelP;
        g.D = modelD;
      }
      g.P = clampTo(g.P, lim.P);
      g.D = clampTo(g.D, lim.D);
      g.FF = FF;
      g.I = iRule(fam, axis, g.P, FF, hasFF);
      Json gj = gainsJson(g);
      if (plane) gj.set("IMAX", std::min(0.9, std::max(0.4, jor(cur, "IMAX", 0.666))));
      const double wc = 2 * kPi * designMargins.wcHz;
      const AngleRule ang = angleRule(fam, axis, wc, lowConf ? 0.5 : 2);
      Json proposed = gj;
      if (ang.angleP) proposed.set("angleP", *ang.angleP);
      if (ang.tconst) proposed.set("tconst", *ang.tconst);
      if (!plane) proposed.set("SMAX", jv(cur, "SMAX") > 0 ? jv(cur, "SMAX") : 50.0);
      // predicted responses: current vs proposed on the identified plant
      PidGains gCur = pidOf(cur);
      const SimResult stepCur = simulateRateStep(ctx, gCur, stepMag, stepDurS, uMax);
      PidGains gNew = g;
      gNew.DFF = 0;
      const SimResult stepNew = simulateRateStep(ctx, gNew, stepMag, stepDurS, uMax);
      const Vec freqs = logspace(lowConf ? 0.005 : 0.2, loopHz / 2, lowConf ? 500 : 300);
      const Margins mCur = margins(freqs, openLoop(freqs, ctx, gCur).L);
      Json osc;
      if (syn->oscillationPoints) {
        const auto& o = *syn->oscillationPoints;
        osc = Json::obj({{"D", optNum(o.D)}, {"P", optNum(o.P)}, {"backoff", o.backoff}, {"startP", o.startP}});
      }
      Json cands = Json::array();
      for (const auto& c : syn->candidates) {
        Json cj = Json::obj({{"r", c.r}});
        if (!(c.fail && std::string(c.fail) == "fail")) {
          cj.set("P", c.P);
          cj.set("D", c.D);
        }
        cj.set("fail", c.fail ? (std::string(c.fail) == "fail" ? Json(true) : Json(c.fail)) : Json());
        if (c.m) cj.set("wc", c.m->wcHz);
        if (!c.fail) cj.set("os", c.overshootPct);
        cands.push(cj);
      }
      Json predicted = Json::obj({{"t", Json(everyOther(stepCur.t))},
                                  {"current", normEveryOther(stepCur.y, stepMag)},
                                  {"proposed", normEveryOther(stepNew.y, stepMag)},
                                  {"currentMetrics", metricsJson(stepMetrics(stepCur.t, scaled(stepCur.y, stepMag)))},
                                  {"proposedMetrics", metricsJson(stepMetrics(stepNew.t, scaled(stepNew.y, stepMag)))}});
      Json proposal = Json::obj({{"gains", Json()},
                                 {"margins", marginsJson(syn->margins)},
                                 {"bandwidthHz", syn->bandwidthHz},
                                 {"overshootPct", syn->overshootPct},
                                 {"dNoise", syn->dNoise},
                                 {"oscillationPoints", osc},
                                 {"predicted", predicted},
                                 {"currentMargins", marginsJson(mCur)},
                                 {"candidates", cands}});
      // safety: one round never moves P / I / FF more than x2 (up) or /2 (down)
      std::vector<std::string> capNote;
      auto capTo = [&](const char* k, double v) {
        proposed.set(k, v);
        capNote.push_back(k);
      };
      const double pFrom = jor(cur, "P", 0), pTo = jv(proposed, "P");
      const double dRatio = jv(proposed, "P") > 0 ? jv(proposed, "D") / jv(proposed, "P") : 0;
      if (pFrom > 0 && pTo > 2 * pFrom) {
        capTo("P", 2 * pFrom);
        proposed.set("D", std::min(jv(proposed, "D"), dRatio * jv(proposed, "P")));
        proposed.set("I", iRule(fam, axis, jv(proposed, "P"), FF, hasFF));
      }
      if (pFrom > 0 && pTo < 0.5 * pFrom) {
        capTo("P", 0.5 * pFrom);
        proposed.set("D", std::max(jv(proposed, "D"), dRatio * jv(proposed, "P")));
        proposed.set("I", iRule(fam, axis, jv(proposed, "P"), FF, hasFF));
      }
      const double dFrom = jor(cur, "D", 0);
      if (dFrom > 0 && jv(proposed, "D") > 3 * dFrom) capTo("D", 3 * dFrom);
      if (dFrom > 0 && jv(proposed, "D") < 0.33 * dFrom) capTo("D", 0.33 * dFrom);
      const double fFrom = jor(cur, "FF", 0);
      if (fFrom > 0 && jv(proposed, "FF") > 2 * fFrom) {
        capTo("FF", 2 * fFrom);
        proposed.set("I", iRule(fam, axis, jv(proposed, "P"), jv(proposed, "FF"), hasFF));
      }
      if (fFrom > 0 && jv(proposed, "FF") < 0.5 * fFrom) {
        capTo("FF", 0.5 * fFrom);
        proposed.set("I", iRule(fam, axis, jv(proposed, "P"), jv(proposed, "FF"), hasFF));
      }
      // lower-confidence model: D moves at most 1.5x per round and is not introduced where it is 0 today
      if (lowConf) {
        if (!(dFrom > 0)) {
          if (jv(proposed, "D") > 0) {
            proposed.set("D", 0.0);
            capNote.push_back("D");
          }
        } else if (jv(proposed, "D") > 1.5 * dFrom) {
          capTo("D", 1.5 * dFrom);
        } else if (jv(proposed, "D") < dFrom / 1.5) {
          capTo("D", dFrom / 1.5);
        }
      }
      // angle gain / time constant: at most x1.5 per round
      const double curAng = jv(cur, "angleP"), curTc = jv(cur, "tconst");
      if (proposed.has("angleP") && curAng > 0)
        proposed.set("angleP", fx(std::min(curAng * 1.5, std::max(curAng / 1.5, jv(proposed, "angleP"))), 2));
      if (proposed.has("tconst") && curTc > 0)
        proposed.set("tconst", fx(std::min(curTc * 1.5, std::max(curTc / 1.5, jv(proposed, "tconst"))), 2));
      // I term winding against a steady torque (trim): give it room (IMAX ~1.5x the trim it has to hold)
      const Json& sat = res["diagnostics"]["saturation"];
      const double curImax = jv(cur, "IMAX");
      if (!plane && sat.isObject() && sat["iNearMax"].asBool() && curImax > 0) {
        const double iP95 = jor(sat, "iP95", 0);
        const double need = std::min(0.9, std::ceil(std::max(iP95, std::fabs(tdFit ? tdFit->trimMean : 0)) * 1.5 / 0.05) * 0.05);
        if (need > curImax + 1e-6) proposed.set("IMAX", fx(need, 2));
      }
      if (lowConf) {
        proposal.set("lowConfidence", true);
        proposal.set("modelDesign", Json::obj({{"P", modelP},
                                               {"I", iRule(fam, axis, modelP, FF, hasFF)},
                                               {"D", modelD},
                                               {"angleP", optNum(ang.angleP)},
                                               {"limitedTo", modelP > lim.P[1] ? Json(lim.P[1]) : Json()},
                                               {"crossoverHz", designMargins.wcHz},
                                               {"margins", marginsJson(designMargins)},
                                               {"headroomNote", headroomNote}}));
        if (modelP > lim.P[1])
          warn("The model asks for rate P " + tf(modelP, 2) +
               (headroomNote.isString() ? " (" + headroomNote.str() + ")" : std::string()) + ", above the usual " +
               axisN + " limit " + ts(lim.P[1]) + " (QuikTune maximum): this airframe has very little " + axisN +
               " authority. The proposal stops at " + ts(lim.P[1]) +
               "; go further only in small steps after the mechanical cause is fixed.");
      }
      if (!capNote.empty()) {
        Json cn = Json::array();
        std::string list;
        for (std::size_t i = 0; i < capNote.size(); i++) {
          cn.push(capNote[i]);
          list += (i ? ", " : "") + capNote[i];
        }
        proposal.set("capped", cn);
        warn("Change limited for " + list + " (one round moves P/I at most 2x" +
             (lowConf ? ", D at most 1.5x and never from 0 on a lower-confidence model" : "") +
             "): fly again with the new gains and re-analyse to continue.");
      }
      proposal.set("changes", describeChanges(cur, proposed));
      if (!capNote.empty() || lowConf) {
        PidGains gp = pidOf(proposed);
        gp.DFF = 0;
        const SimResult stepP = simulateRateStep(ctx, gp, stepMag, stepDurS, uMax);
        proposal.set("designMargins", proposal["margins"]);
        proposal.set("margins", marginsJson(margins(freqs, openLoop(freqs, ctx, gp).L)));
        Json& pr = proposal.at("predicted");
        pr.set("proposed", normEveryOther(stepP.y, stepMag));
        pr.set("proposedMetrics", metricsJson(stepMetrics(stepP.t, scaled(stepP.y, stepMag))));
      }
      proposal.set("gains", proposed);
      res.set("proposal", proposal);
    }
  }
  GradeCtx gc;
  gc.idOk = res["identification"]["ok"].asBool() || (res.has("proposal") && res["proposal"]["partial"].asBool());
  gc.lowConf = lowConf;
  res.at("diagnostics").set("grade", gradeAxis(res["diagnostics"], gc));
  // the measured step response of an ungradable / saturated axis is noise: keep it out of the report
  const Json& gr = res["diagnostics"]["grade"];
  const Json& satj = res["diagnostics"]["saturation"];
  if (res.has("measuredStep") && ((gr["gradable"].isBool() && !gr["gradable"].asBool()) ||
                                  (satj.isObject() && satj["saturated"].asBool()))) {
    res.set("measuredStepUnreliable", res["measuredStep"]["metrics"]);
    res.erase("measuredStep");
  }
  const Json& ffe = res["ffEvents"];
  if (!res.has("proposal") && plane && ffe.isObject() && jor(ffe, "ff", 0) != 0) {
    // no trustworthy model: still propose the feed-forward and I (ArduPilot rule I = FF), keep P / D
    const GainLimits lim = gainLimits(fam, axis);
    double FF = std::min(lim.FF[1], std::max(lim.FF[0], jv(ffe, "ff")));
    const double cff = jv(cur, "FF");
    if (cff > 0) FF = std::min(2 * cff, std::max(0.5 * cff, FF));   // x2 per round
    Json g = cur;
    g.set("FF", FF);
    g.set("I", iRule(fam, axis, jv(cur, "P"), FF, true));
    res.set("proposal", Json::obj({{"gains", g},
                                   {"partial", true},
                                   {"note", "Feed-forward from " + ts(jv(ffe, "events")) +
                                                " stick events (ArduPilot AUTOTUNE method). P and D unchanged: the "
                                                "rate-loop model could not be identified from this flight."},
                                   {"changes", describeChanges(cur, g)}}));
  }
  return res;
}

}  // namespace aptuner::engine
