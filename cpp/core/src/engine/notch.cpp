#include "aptuner/engine/notch.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/fft.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/engine/analysis.hpp"

namespace aptuner::engine {

using namespace aptuner::core;

namespace {

constexpr double kGyroDeg = 57.2958;   // the JavaScript version scales gyro rad/s by this constant

bool inWin(const Window& win, double tUs) {
  if (!win.tStart || *win.tStart == 0) return true;
  const double t = tUs / 1e6;
  return t >= *win.tStart && t <= win.tEnd.value_or(std::nan(""));
}

double estimateFs(const Vec& t) {
  Vec d;
  for (std::size_t i = 1; i < std::min<std::size_t>(t.size(), 5000); i++) {
    const double x = t[i] - t[i - 1];
    if (x > 0) d.push_back(x);
  }
  std::sort(d.begin(), d.end());
  return d.empty() ? 0 : 1 / d[d.size() / 2];
}

std::optional<double> hoverThrottle(const log::LogData& log, Family family) {
  const log::MsgTable* q = log.msg("QTUN");
  const log::MsgTable* ct = family == Family::Vtol && q ? q : log.msg("CTUN");
  if (!ct || !ct->col("ThO")) return std::nullopt;
  Vec v;
  for (double x : *ct->col("ThO"))
    if (x > 0.15) v.push_back(x);
  std::sort(v.begin(), v.end());
  if (v.empty()) return std::nullopt;
  return v[v.size() / 2];
}

std::optional<double> escMeanRpm(const log::LogData& log) {
  const log::MsgTable* e = log.msg("ESC");
  if (!e || !e->col("RPM")) return std::nullopt;
  double s = 0, c = 0;
  for (double r : *e->col("RPM"))
    if (r > 100) {
      s += r;
      c++;
    }
  if (c == 0) return std::nullopt;
  return s / c;
}

/** RMS over a band of an amplitude spectrum (amp^2 / 2 per bin), rounded to 3 decimals as the JavaScript did */
double ampBandRms(const Vec& f, const Vec& amp, double f1, double f2) {
  double e = 0;
  for (std::size_t i = 0; i < f.size(); i++)
    if (f[i] >= f1 && f[i] <= f2) e += amp[i] * amp[i] / 2;
  return jsRoundFixed(std::sqrt(e), 3);
}

double phaseLagAt(const GyroChain& chain, double fHz) {
  const CVec H = gyroChain(Vec{fHz}, chain);
  return jsRoundFixed(-std::atan2(H.im[0], H.re[0]) * 180 / kPi, 1);
}

Vec combinedAmp(const Psd& sx, const Psd& sy, const Psd& sz) {
  Vec a(sx.f.size());
  for (std::size_t i = 0; i < a.size(); i++)
    a[i] = std::sqrt((sx.amp[i] * sx.amp[i] + sy.amp[i] * sy.amp[i] + sz.amp[i] * sz.amp[i]) / 3);
  return a;
}

struct Consistency {
  double f = 0;
  int hits = 0, windows = 0;
  double share = 0;
};

std::vector<Consistency> peakConsistency(const GyroSamples& gs, const log::LogData& log, Family family,
                                         const std::vector<Peak>& peaks) {
  std::vector<Consistency> out;
  if (peaks.empty()) return out;
  const double fs = gs.fs;
  const std::size_t segN = static_cast<std::size_t>(jsRound(10 * fs));
  const std::size_t winSize = std::min<std::size_t>(1024, nextPow2(fs * 1));
  const log::MsgTable* q = log.msg("QTUN");
  const log::MsgTable* ct = family == Family::Vtol && q ? q : log.msg("CTUN");
  const auto hov = hoverThrottle(log, family);
  const bool useThr = !gs.t.empty() && ct && ct->col("ThO") && hov && *hov != 0;
  std::size_t j = 0;
  auto thrAt = [&](double tSec) {
    const Vec& tUs = *ct->col("TimeUS");
    const Vec& thO = *ct->col("ThO");
    const double tu = tSec * 1e6;
    while (j + 1 < ct->n && tUs[j + 1] <= tu) j++;
    return thO[j];
  };
  std::vector<std::vector<double>> parts;
  for (std::size_t s0 = 0; segN > 0 && s0 + segN <= gs.x.size(); s0 += segN) {
    if (useThr && thrAt(gs.t[s0 + (segN >> 1)]) < 0.6 * *hov) continue;   // on the ground / spooling
    const Psd sx = psd(gs.x.data() + s0, segN, fs, winSize), sy = psd(gs.y.data() + s0, segN, fs, winSize),
              sz = psd(gs.z.data() + s0, segN, fs, winSize);
    std::vector<double> fl;
    for (const Peak& p : findPeaks(sx.f, combinedAmp(sx, sy, sz), 8, fs * 0.48, 8)) fl.push_back(p.f);
    parts.push_back(std::move(fl));
  }
  for (const Peak& p : peaks) {
    const double tol = std::max(2.0, 0.08 * p.f);
    int hits = 0;
    for (const auto& list : parts)
      if (std::any_of(list.begin(), list.end(), [&](double f) { return std::fabs(f - p.f) <= tol; })) hits++;
    const int w = static_cast<int>(parts.size());
    out.push_back({p.f, hits, w, w ? static_cast<double>(hits) / w : 0});
  }
  return out;
}

struct NotchRec {
  Json params;
  double gyroFilterHz = 0;
  std::string why;
  NotchDef def;
  std::vector<std::string> notes;
};

NotchRec recommendNotch(const HarmonicFamily& fam, const std::vector<Peak>& peaks, const log::Params& params,
                        double hoverThr, std::optional<double> escRpm, Family family) {
  (void)family;
  const double f0 = fam.f0;
  int hmncs = 0;
  double maxAmp = -1e300;
  for (const auto& m : fam.members) maxAmp = std::max(maxAmp, m.amp);
  for (const auto& m : fam.members)
    if (m.amp > 0.15 * maxAmp && m.k <= 4) hmncs |= 1 << (m.k - 1);
  if (!(hmncs & 1)) hmncs |= 1;
  const bool broad = std::any_of(peaks.begin(), peaks.end(),
                                 [&](const Peak& p) { return std::fabs(p.f - f0) < 2 && p.widthHz > f0 * 0.12; });
  const int opts = broad ? 16 : 0;
  const double bw = jsRoundFixed(f0 / 2, 1);
  int mode = 1;
  double ref = jsRoundFixed(hoverThr != 0 && !std::isnan(hoverThr) ? hoverThr : 0.3, 2);
  std::string why = "throttle-based tracking (mode 1): FREQ = noise peak at hover, REF = hover throttle";
  if (escRpm && *escRpm != 0 && std::fabs(*escRpm / 60 - f0) / f0 < 0.25) {
    mode = 3;
    ref = 1;
    why = "ESC telemetry RPM tracking (mode 3): the ESC RPM matches the noise peak";
  } else if (escRpm && *escRpm != 0 && std::fabs(*escRpm / 60 * 2 - f0) / f0 < 0.25) {
    mode = 3;
    ref = 2;
    why = "ESC telemetry RPM tracking (mode 3): peak is twice the motor RPM (two-blade passing), REF = 2";
  }
  const double gyroCur = params.getOr("INS_GYRO_FILTER", 20);
  // with a good notch the gyro LPF can usually be raised: cap at f0/3 and 80 Hz
  const double gyroFilterHz = std::max(gyroCur, std::min(80.0, jsRound(f0 / 3 / 5) * 5));
  NotchRec r;
  r.params = Json::obj({{"INS_HNTCH_ENABLE", 1},
                        {"INS_HNTCH_MODE", mode},
                        {"INS_HNTCH_FREQ", jsRoundFixed(f0, 1)},
                        {"INS_HNTCH_BW", bw},
                        {"INS_HNTCH_ATT", 40},
                        {"INS_HNTCH_REF", ref},
                        {"INS_HNTCH_HMNCS", hmncs},
                        {"INS_HNTCH_OPTS", opts},
                        {"INS_HNTCH_FM_RAT", 1}});
  r.gyroFilterHz = gyroFilterHz;
  r.why = why;
  r.def.enable = true;
  r.def.freq = f0;
  r.def.bw = bw;
  r.def.att = 40;
  r.def.hmncs = hmncs;
  r.def.opts = opts;
  std::string ks;
  for (std::size_t i = 0; i < fam.members.size(); i++) ks += (i ? ", " : "") + std::to_string(fam.members[i].k);
  r.notes = {"Fundamental " + jsToFixed(f0, 1) + " Hz (" + jsToFixed(f0 * 60, 0) + " RPM-equivalent); harmonics with energy: " + ks,
             broad ? "Peak is broad: triple notch (OPTS bit 4) recommended" : "Peak is narrow: single notch per harmonic",
             "INS_GYRO_FILTER can be raised to " + jsToString(gyroFilterHz) + " Hz once the notch is in place (currently " +
                 jsToString(gyroCur) + ")",
             "After uploading, fly again and re-check: the post-filter spectrum should match the prediction."};
  return r;
}

Json roundedArray(const Vec& v, int precision) {
  Json a = Json::array();
  for (double x : v) a.push(jsRoundPrecision(x, precision));
  return a;
}

}  // namespace

std::optional<GyroSamples> gyroSamples(const log::LogData& log, const Window& win) {
  // 1) raw gyro messages
  const log::MsgTable* gyr = log.msg("GYR");
  if (gyr && gyr->n > 2000) {
    auto split = log::splitByInstance(*gyr, "I");
    const log::MsgTable* g = split.count(0) ? &split.at(0) : (split.empty() ? nullptr : &split.begin()->second);
    if (g && g->col("TimeUS")) {
      std::vector<std::size_t> idx;
      const Vec& tUs = *g->col("TimeUS");
      for (std::size_t i = 0; i < g->n; i++)
        if (inWin(win, tUs[i])) idx.push_back(i);
      if (idx.size() > 2000) {
        const Vec* sUs = g->col("SampleUS");
        GyroSamples gs;
        for (std::size_t i : idx) gs.t.push_back(sUs ? (*sUs)[i] / 1e6 : tUs[i] / 1e6);
        gs.fs = estimateFs(gs.t);
        for (std::size_t i : idx) {
          gs.x.push_back((*g->col("GyrX"))[i] * kGyroDeg);
          gs.y.push_back((*g->col("GyrY"))[i] * kGyroDeg);
          gs.z.push_back((*g->col("GyrZ"))[i] * kGyroDeg);
        }
        gs.source = "GYR raw gyro (" + jsToFixed(gs.fs, 0) + " Hz)";
        gs.prefilter = true;
        return gs;
      }
    }
  }
  // 2) batch sampler
  const log::MsgTable* h = log.msg("ISBH");
  const log::MsgTable* d = log.msg("ISBD");
  if (h && d && h->n > 2 && h->col("type") && h->col("instance") && h->col("N") && d->col("N")) {
    struct Hdr {
      double mul, fs, t;
    };
    std::map<double, Hdr> hdr;
    const Vec &ht = *h->col("type"), &hi = *h->col("instance"), &hn = *h->col("N"), &hm = *h->col("mul"),
              &hr = *h->col("smp_rate"), &hT = *h->col("TimeUS");
    for (std::size_t i = 0; i < h->n; i++)
      if (ht[i] == 1 && hi[i] == 0) hdr[hn[i]] = {hm[i], hr[i], hT[i]};
    GyroSamples gs;
    double fs = 0;
    const Vec& dn = *d->col("N");
    auto xs = d->i16.find("x"), ys = d->i16.find("y"), zs = d->i16.find("z");
    if (xs != d->i16.end() && ys != d->i16.end() && zs != d->i16.end()) {
      for (std::size_t i = 0; i < d->n; i++) {
        auto it = hdr.find(dn[i]);
        if (it == hdr.end() || !inWin(win, it->second.t)) continue;
        fs = it->second.fs;
        const double mul = it->second.mul != 0 ? it->second.mul : 1;
        for (int k = 0; k < 32; k++) {
          gs.x.push_back(xs->second[i][static_cast<std::size_t>(k)] / mul * kGyroDeg);
          gs.y.push_back(ys->second[i][static_cast<std::size_t>(k)] / mul * kGyroDeg);
          gs.z.push_back(zs->second[i][static_cast<std::size_t>(k)] / mul * kGyroDeg);
        }
      }
    }
    if (gs.x.size() > 2000) {
      const int opt = log.params.intOf("INS_LOG_BAT_OPT");
      const bool pre = !(opt & 2);
      gs.fs = fs;
      gs.source = "IMU batch sampler (" + jsToFixed(fs, 0) + " Hz, " + (pre ? "pre" : "post") + "-filter)";
      gs.prefilter = pre;
      gs.batched = true;
      return gs;
    }
  }
  // 3) IMU message (filtered gyro at the logging rate)
  const log::MsgTable* imu = log.msg("IMU");
  if (imu && imu->n > 2000) {
    auto split = log::splitByInstance(*imu, "I");
    if (split.count(0)) {
      const log::MsgTable& im = split.at(0);
      const Vec& tUs = *im.col("TimeUS");
      std::vector<std::size_t> idx;
      for (std::size_t i = 0; i < im.n; i++)
        if (inWin(win, tUs[i])) idx.push_back(i);
      GyroSamples gs;
      for (std::size_t i : idx) gs.t.push_back(tUs[i] / 1e6);
      gs.fs = estimateFs(gs.t);
      if (idx.size() > 2000) {
        for (std::size_t i : idx) {
          gs.x.push_back((*im.col("GyrX"))[i] * kGyroDeg);
          gs.y.push_back((*im.col("GyrY"))[i] * kGyroDeg);
          gs.z.push_back((*im.col("GyrZ"))[i] * kGyroDeg);
        }
        gs.source = "IMU message (" + jsToFixed(gs.fs, 0) + " Hz, post-filter)";
        gs.prefilter = false;
        return gs;
      }
    }
  }
  return std::nullopt;
}

std::vector<Peak> findPeaks(const Vec& f, const Vec& amp, double fMin, double fMax, std::size_t maxPeaks) {
  std::vector<Peak> peaks;
  const std::size_t n = f.size();
  Vec sorted = amp;
  std::sort(sorted.begin(), sorted.end());
  double med = n ? sorted[n / 2] : 0;
  if (!(med != 0) || std::isnan(med)) med = 1e-9;
  for (std::size_t i = 2; i + 2 < n; i++) {
    if (f[i] < fMin || f[i] > fMax) continue;
    if (amp[i] > amp[i - 1] && amp[i] >= amp[i + 1] && amp[i] > amp[i - 2] && amp[i] >= amp[i + 2] && amp[i] > 4 * med) {
      std::size_t lo = i, hi = i;
      while (lo > 0 && amp[lo] > amp[i] * 0.707) lo--;
      while (hi < n - 1 && amp[hi] > amp[i] * 0.707) hi++;
      peaks.push_back({f[i], amp[i], amp[i] / med, f[hi] - f[lo]});
    }
  }
  std::stable_sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return b.amp - a.amp < 0; });
  if (peaks.size() > maxPeaks) peaks.resize(maxPeaks);
  std::stable_sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return a.f - b.f < 0; });
  return peaks;
}

std::optional<HarmonicFamily> harmonicFamily(const std::vector<Peak>& peaks) {
  if (peaks.empty()) return std::nullopt;
  std::optional<HarmonicFamily> best;
  for (const auto& cand : peaks) {
    HarmonicFamily fam;
    fam.f0 = cand.f;
    for (const auto& p : peaks) {
      const double r = p.f / fam.f0;
      const double k = jsRound(r);
      if (k >= 1 && k <= 8 && std::fabs(r - k) < 0.08) {
        fam.score += p.amp;
        fam.members.push_back({static_cast<int>(k), p.f, p.amp});
      }
    }
    if (!best || fam.score > best->score) best = fam;
  }
  return best;
}

Json analyzeNoise(const log::LogData& log, const log::Params& params, Family family, const Window& win) {
  const auto gsOpt = gyroSamples(log, win);
  if (!gsOpt)
    return Json::obj({{"available", false},
                      {"reason", "No gyro data at a usable rate. For notch setup log raw gyro: set INS_RAW_LOG_OPT = 9 "
                                 "(H7 boards) or INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 4, then fly 1-2 minutes."}});
  const GyroSamples& gs = *gsOpt;
  const double fs = gs.fs;
  const std::size_t winSize = std::min<std::size_t>(2048, nextPow2(fs * 1));
  const Psd spX = psd(gs.x, fs, winSize), spY = psd(gs.y, fs, winSize), spZ = psd(gs.z, fs, winSize);
  const Vec& f = spX.f;
  const Vec amp = combinedAmp(spX, spY, spZ);
  NotchOp op;
  const auto hovThr = hoverThrottle(log, family);
  if (hovThr) op.throttle = *hovThr;
  const GyroChain chain = gyroChainFromParams(params, op);
  Json fJ = Json::array();
  for (double v : f) fJ.push(jsRoundFixed(v, 2));
  Json res = Json::obj({{"available", true},
                        {"source", gs.source},
                        {"fs", fs},
                        {"prefilter", gs.prefilter},
                        {"f", fJ},
                        {"amp", roundedArray(amp, 4)},
                        {"ampX", roundedArray(spX.amp, 4)},
                        {"ampY", roundedArray(spY.amp, 4)},
                        {"ampZ", roundedArray(spZ.amp, 4)},
                        {"chain", gyroChainJson(chain)}});
  const Vec Hcur = cx::abs(gyroChain(f, chain));
  res.set("currentFilterMag", roundedArray(Hcur, 4));
  Vec postCurrent;
  if (gs.prefilter) {
    for (std::size_t i = 0; i < amp.size(); i++) postCurrent.push_back(jsRoundPrecision(amp[i] * Hcur[i], 4));
    res.set("postCurrent", Json(postCurrent));
  } else {
    res.set("note",
            "Gyro data is post-filter (already filtered). Peaks you see here are what the controller sees. For a "
            "proper notch design, log raw gyro (INS_RAW_LOG_OPT = 9).");
  }
  // peaks on the analysed spectrum (pre-filter if available)
  const std::vector<Peak> peaks = findPeaks(f, amp, 8, fs * 0.48);
  Json pk = Json::array();
  for (const auto& p : peaks)
    pk.push(Json::obj({{"f", jsRoundFixed(p.f, 1)},
                       {"amp", jsRoundPrecision(p.amp, 3)},
                       {"prominence", jsRoundFixed(p.prominence, 1)},
                       {"widthHz", jsRoundFixed(p.widthHz, 1)}}));
  res.set("peaks", pk);
  const auto fam = harmonicFamily(peaks);
  if (fam) {
    Json mem = Json::array();
    for (const auto& m : fam->members)
      mem.push(Json::obj({{"k", m.k}, {"f", jsRoundFixed(m.f, 1)}, {"amp", jsRoundPrecision(m.amp, 3)}}));
    res.set("harmonics", Json::obj({{"f0", jsRoundFixed(fam->f0, 1)}, {"members", mem}}));
  } else {
    res.set("harmonics", Json());
  }
  // control-band noise (what matters for the D term): RMS in 5-40 Hz pre and post filter
  const double fTop = std::min(40.0, fs / 2);
  res.set("rmsBand", Json::obj({{"pre", ampBandRms(f, amp, 5, fTop)},
                                {"postCurrent", gs.prefilter ? Json(ampBandRms(f, postCurrent, 5, fTop)) : Json()}}));
  // consistency: does the peak show up in most parts of the flight?
  const std::vector<Consistency> cons = peakConsistency(gs, log, family, peaks);
  Json consJ = Json::array();
  for (const auto& c : cons)
    consJ.push(Json::obj({{"f", c.f}, {"hits", c.hits}, {"windows", c.windows}, {"share", c.share}}));
  res.set("consistency", consJ);
  const Consistency* cf = nullptr;
  if (fam)
    for (const auto& c : cons)
      if (std::fabs(c.f - fam->f0) < 0.01) {
        cf = &c;
        break;
      }
  const bool unconfirmed = !gs.prefilter && fam && fam->f0 > 15 && (!cf || cf->windows < 5 || cf->share < 0.8);
  if (unconfirmed) {
    std::string list;
    for (std::size_t i = 0; i < cons.size(); i++)
      list += (i ? ", " : "") + jsToFixed(cons[i].f, 1) + " Hz in " + std::to_string(cons[i].hits) + "/" +
              std::to_string(cons[i].windows) + " parts";
    const std::string why =
        cf && cf->windows < 5
            ? "the flight is too short to check that the peaks repeat (" + std::to_string(cf->windows) +
                  " airborne part(s) of 10 s; 5 needed)"
            : "the peaks do not repeat through the flight (" + (list.empty() ? std::string("no repeatable peak") : list) + ")";
    res.set("recommendation",
            "No notch recommended: the log only holds filtered gyro data (" + gs.source + ") and " + why +
                ". A notch at the wrong frequency only adds delay. To find the motor noise properly, enable the batch "
                "sampler (INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0: pre-filter gyro) or raw gyro logging "
                "(INS_RAW_LOG_OPT = 9 on H7 boards such as the Cube Orange+), or use ESC telemetry RPM (the notch then "
                "follows the motors directly), and fly 1-2 minutes of hover.");
    res.set("notchUnconfirmed", Json::obj({{"f0", jsRoundFixed(fam->f0, 1)}}));
  }
  if (fam && fam->f0 > 15 && !unconfirmed) {
    const double thr = hovThr ? *hovThr : 0;
    const NotchRec rec = recommendNotch(*fam, peaks, params, thr, escMeanRpm(log), family);
    GyroChain chainNew = chain;
    chainNew.notches = {rec.def};
    chainNew.gyroFilterHz = rec.gyroFilterHz;
    const Vec Hnew = cx::abs(gyroChain(f, chainNew));
    Vec post(amp.size());
    for (std::size_t i = 0; i < amp.size(); i++)
      post[i] = jsRoundPrecision(amp[i] * Hnew[i] / (gs.prefilter ? 1 : std::max(Hcur[i], 1e-3)), 4);
    std::vector<std::string> notes = rec.notes;
    if (!gs.prefilter)
      notes.push_back("Prediction is approximate: the log holds filtered gyro data, so the true (larger) pre-filter noise is unknown.");
    if (cf && cf->windows)
      notes.push_back("The " + jsToFixed(fam->f0, 1) + " Hz peak appears in " + std::to_string(cf->hits) + " of " +
                      std::to_string(cf->windows) + " parts of the flight.");
    if (!gs.prefilter)
      notes.push_back("Confirm with pre-filter data before relying on it: INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0 (batch sampler) or ESC telemetry RPM.");
    Json notesJ = Json::array();
    for (const auto& s : notes) notesJ.push(s);
    res.set("recommended",
            Json::obj({{"params", rec.params},
                       {"gyroFilterHz", rec.gyroFilterHz},
                       {"why", rec.why},
                       {"notchDef", Json::obj({{"enable", 1}, {"freq", rec.def.freq}, {"bw", rec.def.bw}, {"att", 40}, {"hmncs", rec.def.hmncs}, {"opts", rec.def.opts}})},
                       {"notes", notesJ},
                       {"postPredicted", Json(post)},
                       {"rmsBandPost", ampBandRms(f, post, 5, fTop)},
                       {"phaseLag10Hz", Json::obj({{"current", phaseLagAt(chain, 10)}, {"proposed", phaseLagAt(chainNew, 10)}})}}));
  } else if (!fam) {
    res.set("recommendation", "No clear motor noise peak above 15 Hz: vibration is low or the data is too short.");
  }
  return res;
}

}  // namespace aptuner::engine
