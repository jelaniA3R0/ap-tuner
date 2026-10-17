#include "aptuner/engine/notch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/fft.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/log/integrity.hpp"

namespace aptuner::engine {

using namespace aptuner::core;

namespace {

constexpr double kGyroDeg = 57.2958;   // the JavaScript version scales gyro rad/s by this constant

/** unbroken blocks [i0, i1) of a sample sequence: split where a sample was left out (index jump) or the time steps over a gap */
Blocks blocksOfSamples(const Vec& t, const std::vector<std::size_t>& idx) {
  Blocks blocks;
  if (idx.empty()) return blocks;
  const double dt = log::medianStep(t);
  std::size_t s = 0;
  for (std::size_t k = 1; k < idx.size(); k++)
    if (idx[k] != idx[k - 1] + 1 || (dt > 0 && t[k] - t[k - 1] > log::kGapFactor * dt)) {
      blocks.emplace_back(s, k);
      s = k;
    }
  blocks.emplace_back(s, idx.size());
  return blocks;
}

/** mean sample rate inside unbroken blocks (gaps excluded) */
double blockRate(const Vec& t, const Blocks& blocks) {
  double num = 0, den = 0;
  for (const auto& [a, b] : blocks)
    if (b - a >= 2) {
      num += static_cast<double>(b - a - 1);
      den += t[b - 1] - t[a];
    }
  return den > 0 ? jsRoundFixed(num / den, 3) : 0;
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

/**
 * Cut the (airborne) gyro data into 10 s parts inside unbroken blocks, find the peaks in each and count, for every
 * peak of the whole-flight spectrum, in how many parts it shows up (within 8% / 2 Hz).
 */
std::vector<Consistency> peakConsistency(const GyroSamples& gs, const std::vector<Peak>& peaks) {
  std::vector<Consistency> out;
  if (peaks.empty()) return out;
  const double fs = gs.fs;
  const std::size_t segN = static_cast<std::size_t>(jsRound(10 * fs));
  const std::size_t winSize = std::min<std::size_t>(1024, nextPow2(fs * 1));
  std::vector<std::vector<double>> parts;
  for (const auto& [b0, b1] : gs.blocks)
    for (std::size_t s0 = b0; segN > 0 && s0 + segN <= b1; s0 += segN) {
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

std::optional<GyroSamples> gyroSamples(const log::LogData& log, const Window& win, const std::optional<Airborne>& air) {
  const auto inWin = [&](double tSec) { return !win.tStart || (tSec >= *win.tStart && tSec <= win.tEnd.value_or(0)); };
  const auto flying = [&](double tSec) { return !air || inIntervals(air->intervals, tSec); };
  // samples of a gyro table in the window, airborne and finite, cut into unbroken blocks (tUs: its time column)
  const auto fromTable = [&](const log::MsgTable& g, const Vec& tUs) -> std::optional<GyroSamples> {
    const Vec* gx = g.col("GyrX");
    const Vec* gy = g.col("GyrY");
    const Vec* gz = g.col("GyrZ");
    if (!gx || !gy || !gz) return std::nullopt;
    std::vector<std::size_t> idx;
    GyroSamples gs;
    for (std::size_t i = 0; i < g.n; i++) {
      const double ts = tUs[i] / 1e6;
      if (inWin(ts) && flying(ts) && std::isfinite((*gx)[i]) && std::isfinite((*gy)[i]) && std::isfinite((*gz)[i])) {
        idx.push_back(i);
        gs.t.push_back(ts);
      }
    }
    gs.blocks = blocksOfSamples(gs.t, idx);
    gs.fs = blockRate(gs.t, gs.blocks);
    for (std::size_t i : idx) {
      gs.x.push_back((*gx)[i] * kGyroDeg);
      gs.y.push_back((*gy)[i] * kGyroDeg);
      gs.z.push_back((*gz)[i] * kGyroDeg);
    }
    return gs;
  };
  // 1) raw gyro messages
  const log::MsgTable* gyr = log.msg("GYR");
  if (gyr && gyr->n > 2000) {
    const auto split = log::splitByInstance(*gyr, "I");
    const log::MsgTable* g = split.count(0) ? &split.at(0) : (split.empty() ? nullptr : &split.begin()->second);
    // INS_RAW_LOG_OPT bit 2: GYR holds the filtered gyro; bit 3 (pre and post filter): the first instance holds the
    // raw gyro and the filtered copy is logged under instance + number of gyros
    const int opt = log.params.intOf("INS_RAW_LOG_OPT");
    const bool post = (opt & 4) && !(opt & 8);
    const Vec* tcol = g ? (g->col("SampleUS") ? g->col("SampleUS") : g->col("TimeUS")) : nullptr;
    if (g && tcol) {
      auto r = fromTable(*g, *tcol);
      if (r && r->x.size() > 2000 && r->fs > 0) {
        r->source = std::string("GYR ") + (post ? "filtered" : "raw") + " gyro (" + jsToFixed(r->fs, 0) + " Hz" +
                    (post ? ", post-filter: INS_RAW_LOG_OPT bit 2" : "") + ")";
        r->prefilter = !post;
        return r;
      }
    }
  }
  // 2) batch sampler: every batch is its own block (the sampler pauses and switches sensors between batches)
  const log::MsgTable* h = log.msg("ISBH");
  const log::MsgTable* d = log.msg("ISBD");
  if (h && d && h->n > 2 && h->col("type") && h->col("instance") && h->col("N") && h->col("mul") && h->col("smp_rate") &&
      h->col("TimeUS") && d->col("N")) {
    struct Hdr {
      double mul, fs, t;
    };
    // gyro batches: type 0 = accel, 1 = gyro
    std::map<double, Hdr> hdr;
    const Vec &ht = *h->col("type"), &hi = *h->col("instance"), &hn = *h->col("N"), &hm = *h->col("mul"),
              &hr = *h->col("smp_rate"), &hT = *h->col("TimeUS");
    for (std::size_t i = 0; i < h->n; i++)
      if (ht[i] == 1 && hi[i] == 0) hdr[hn[i]] = {hm[i], hr[i], hT[i] / 1e6};
    const auto xs = d->i16.find("x"), ys = d->i16.find("y"), zs = d->i16.find("z");
    if (xs != d->i16.end() && ys != d->i16.end() && zs != d->i16.end()) {
      GyroSamples gs;
      double fs = 0;
      std::optional<double> cur, lastSeq;
      std::size_t start = 0;
      const Vec& dn = *d->col("N");
      const Vec* seqno = d->col("seqno");
      for (std::size_t i = 0; i < d->n; i++) {
        const auto it = hdr.find(dn[i]);
        if (it == hdr.end()) continue;
        const Hdr& H = it->second;
        if (!inWin(H.t) || !flying(H.t)) continue;
        if (fs == 0 || std::isnan(fs)) fs = H.fs;
        if (H.fs != fs) continue;
        const std::optional<double> seq = seqno ? std::optional<double>((*seqno)[i]) : std::nullopt;
        // a new batch, or a missing record inside one, starts a new block
        if (!cur || *cur != dn[i] || (seq && lastSeq && *seq != *lastSeq + 1)) {
          if (gs.x.size() > start) gs.blocks.emplace_back(start, gs.x.size());
          start = gs.x.size();
          cur = dn[i];
        }
        lastSeq = seq;
        const double mul = H.mul != 0 && !std::isnan(H.mul) ? H.mul : 1;
        for (std::size_t k = 0; k < 32; k++) {
          gs.x.push_back(xs->second[i][k] / mul * kGyroDeg);
          gs.y.push_back(ys->second[i][k] / mul * kGyroDeg);
          gs.z.push_back(zs->second[i][k] / mul * kGyroDeg);
        }
      }
      if (gs.x.size() > start) gs.blocks.emplace_back(start, gs.x.size());
      if (gs.x.size() > 2000 && fs > 0) {
        const int opt = log.params.intOf("INS_LOG_BAT_OPT");
        const bool pre = !(opt & 2);
        gs.fs = fs;
        gs.source = "IMU batch sampler (" + jsToFixed(fs, 0) + " Hz, " + (pre ? "pre" : "post") + "-filter, " +
                    jsToString(static_cast<double>(gs.blocks.size())) + " batches)";
        gs.prefilter = pre;
        gs.batched = true;
        return gs;
      }
    }
  }
  // 3) IMU message (filtered gyro at the logging rate)
  const log::MsgTable* imu = log.msg("IMU");
  if (imu && imu->n > 2000) {
    const auto split = log::splitByInstance(*imu, "I");
    if (split.count(0) && split.at(0).col("TimeUS")) {
      const log::MsgTable& im = split.at(0);
      auto r = fromTable(im, *im.col("TimeUS"));
      if (r && r->x.size() > 2000 && r->fs > 0) {
        r->source = "IMU message (" + jsToFixed(r->fs, 0) + " Hz, post-filter)";
        r->prefilter = false;
        return r;
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

Json analyzeNoise(const log::LogData& log, const log::Params& params, Family family, const Window& win, const NoiseOpts* opts) {
  std::optional<Airborne> air;
  NotchOp op;
  if (opts) {
    air = opts->air;
    op = opts->op;
  } else {
    const bool hasWin = win.tStart.has_value();
    const auto thr = throttleStream(log, family);
    const double inf = std::numeric_limits<double>::infinity();
    if (family != Family::Plane && thr)
      air = airborneIntervals(*thr, hasWin ? *win.tStart : -inf, hasWin ? win.tEnd.value_or(0) : inf);
    op = notchOperatingPoint(log, family, hasWin ? win : Window{}, air);
  }
  const auto gsOpt = gyroSamples(log, win, air);
  if (!gsOpt)
    return Json::obj({{"available", false},
                      {"reason", std::string("No gyro data at a usable rate") + (air ? " while flying" : "") +
                                     ". For notch setup log raw gyro: set INS_RAW_LOG_OPT = 9 (H7 boards) or "
                                     "INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 4, then fly 1-2 minutes."}});
  const GyroSamples& gs = *gsOpt;
  const double fs = gs.fs;
  std::size_t longest = 0;
  for (const auto& [a, b] : gs.blocks) longest = std::max(longest, b - a);
  const std::size_t winSize = std::min({std::size_t{2048}, nextPow2(fs * 1), prevPow2(static_cast<double>(longest))});
  if (winSize < 64)
    return Json::obj({{"available", false},
                      {"reason", "The gyro data have no unbroken stretch long enough for a spectrum (longest " +
                                     jsToFixed(static_cast<double>(longest) / fs, 2) + " s): the log has gaps or dropouts."}});
  const Psd spX = psd(gs.x, fs, winSize, 0.5, &gs.blocks), spY = psd(gs.y, fs, winSize, 0.5, &gs.blocks),
            spZ = psd(gs.z, fs, winSize, 0.5, &gs.blocks);
  const Vec& f = spX.f;
  const Vec amp = combinedAmp(spX, spY, spZ);
  const GyroChain chain = gyroChainFromParams(params, op);
  std::size_t nUse = 0;
  for (const auto& [a, b] : gs.blocks) nUse = nUse + b - a;
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
                        {"chain", gyroChainJson(chain)},
                        {"dataInfo", Json::obj({{"seconds", static_cast<double>(nUse) / fs},
                                                {"blocks", gs.blocks.size()},
                                                {"longestBlockS", static_cast<double>(longest) / fs},
                                                {"windows", spX.nWin},
                                                {"airborneOnly", air.has_value()}})}});
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
  const double gyroHz = params.getOr("INS_GYRO_FILTER", 20);
  const double fTop = std::min(40.0, fs / 2);
  // aliasing: filtered gyro sampled at the logging rate still holds what the gyro low-pass lets through above fs/2
  // (it is only 19 dB down at 3x its cut-off), and that folds to wrong, lower frequencies
  if (!gs.prefilter && fs / 2 < 3 * (gyroHz > 0 ? gyroHz : 20)) {
    res.set("aliasRisk", true);
    res.set("peaks", Json::array());
    res.set("harmonics", Json());
    res.set("consistency", Json::array());
    res.set("rmsBand", Json::obj({{"pre", ampBandRms(f, amp, 5, fTop)}, {"postCurrent", Json()}}));
    res.set("recommendation",
            "No notch recommended: the gyro data in this log are logged at only " + jsToFixed(fs, 0) + " Hz after the " +
                jsToString(gyroHz) + " Hz gyro filter, which still lets through vibration above " + jsToFixed(fs / 2, 0) +
                " Hz. That vibration folds down to wrong, lower frequencies (aliasing), so the peaks of this spectrum "
                "cannot be trusted. Log the gyro faster or before the filters (batch sampler INS_LOG_BAT_MASK = 1, or raw "
                "gyro logging INS_RAW_LOG_OPT = 9 on H7 boards), or use ESC telemetry RPM.");
    return res;
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
  res.set("rmsBand", Json::obj({{"pre", ampBandRms(f, amp, 5, fTop)},
                                {"postCurrent", gs.prefilter ? Json(ampBandRms(f, postCurrent, 5, fTop)) : Json()}}));
  // consistency: does the peak show up in most parts of the flight? On post-filter data a single strong part (a
  // gust, a landing bump) or filter-shaped bumps can masquerade as a motor peak.
  const std::vector<Consistency> cons = peakConsistency(gs, peaks);
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
    std::string why;
    if (cf && cf->windows < 5)
      why = cf->windows == 0 && gs.blocks.size() > 1
                ? "the gyro data have no unbroken airborne stretch of 10 s to check that the peaks repeat (the log has gaps "
                  "or dropouts)"
                : "the flight is too short to check that the peaks repeat (" + std::to_string(cf->windows) +
                      " airborne part(s) of 10 s; 5 needed)";
    else
      why = "the peaks do not repeat through the flight (" + (list.empty() ? std::string("no repeatable peak") : list) + ")";
    res.set("recommendation",
            "No notch recommended: the log only holds filtered gyro data (" + gs.source + ") and " + why +
                ". A notch at the wrong frequency only adds delay. To find the motor noise properly, enable the batch "
                "sampler (INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 0: pre-filter gyro) or raw gyro logging "
                "(INS_RAW_LOG_OPT = 9 on H7 boards such as the Cube Orange+), or use ESC telemetry RPM (the notch then "
                "follows the motors directly), and fly 1-2 minutes of hover.");
    res.set("notchUnconfirmed", Json::obj({{"f0", jsRoundFixed(fam->f0, 1)}}));
  }
  if (fam && fam->f0 > 15 && !unconfirmed) {
    // operating point of the analysed airborne time: op.throttle || 0, op.escRpm || null
    const double thr = op.throttle && *op.throttle != 0 && !std::isnan(*op.throttle) ? *op.throttle : 0;
    const std::optional<double> esc = op.escRpm != 0 && !std::isnan(op.escRpm) ? std::optional<double>(op.escRpm) : std::nullopt;
    const NotchRec rec = recommendNotch(*fam, peaks, params, thr, esc, family);
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
