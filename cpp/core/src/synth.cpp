#include "aptuner/core/synth.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

namespace aptuner::core {

namespace {
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kSqrt1_2 = std::sqrt(0.5);
}  // namespace

Vec logspace(double f1, double f2, std::size_t n) {
  Vec o(n);
  for (std::size_t i = 0; i < n; i++)
    o[i] = f1 * std::pow(f2 / f1, static_cast<double>(i) / static_cast<double>(n - 1));
  return o;
}

Margins margins(const Vec& freqs, const CVec& L) {
  const Vec mag = cx::abs(L), ph = cx::phaseDeg(L);
  const std::size_t n = ph.size();
  Vec up(n);
  if (n) up[0] = ph[0];
  for (std::size_t i = 1; i < n; i++) {
    double d = ph[i] - ph[i - 1];
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    up[i] = up[i - 1] + d;
  }
  double wc = kNaN, pm = kNaN, w180 = kNaN, gm = kNaN;
  for (std::size_t i = 1; i < mag.size(); i++) {
    if (std::isnan(wc) && mag[i - 1] >= 1 && mag[i] < 1) {
      const double t = (mag[i - 1] - 1) / (mag[i - 1] - mag[i]);
      wc = freqs[i - 1] + t * (freqs[i] - freqs[i - 1]);
      const double p = up[i - 1] + t * (up[i] - up[i - 1]);
      pm = 180 + p;
      while (pm > 360) pm -= 360;
    }
    if (std::isnan(w180) && up[i - 1] > -180 && up[i] <= -180) {
      const double t = (up[i - 1] + 180) / (up[i - 1] - up[i]);
      w180 = freqs[i - 1] + t * (freqs[i] - freqs[i - 1]);
      const double m = mag[i - 1] + t * (mag[i] - mag[i - 1]);
      gm = -20 * std::log10(m);
    }
  }
  // a loop that is always below 1 (very low gain): pm = 180
  if (std::isnan(wc)) {
    pm = 180;
    wc = 0;
  }
  double ms = 0;
  for (std::size_t i = 0; i < mag.size(); i++) {
    const double r = 1 + L.re[i], im = L.im[i];
    double h = std::hypot(r, im);
    if (h == 0 || std::isnan(h)) h = 1e-9;
    const double s = 1 / h;
    if (s > ms) ms = s;
  }
  Margins mg;
  mg.gmDb = gm;
  mg.pmDeg = pm;
  mg.wcHz = wc;
  mg.w180Hz = w180;
  mg.ms = ms;
  return mg;
}

OpenLoop openLoop(const Vec& freqs, const LoopCtx& ctx, const PidGains& g) {
  OpenLoop o;
  o.G = evalPlant(freqs, ctx.plant);
  o.F = gyroChain(freqs, ctx.gyro);
  o.pid = acPid(freqs, ctx.fsLoop, g);
  // the identified Td already contains the loop's computation / output latency (loop-synchronous data)
  o.L = cx::mul(cx::mul(o.G, o.F), o.pid.Cfb);
  return o;
}

ClosedLoop closedLoop(const Vec& freqs, const LoopCtx& ctx, const PidGains& g) {
  OpenLoop ol = openLoop(freqs, ctx, g);
  ClosedLoop c;
  const CVec onePlusL = cx::addReal(ol.L, 1);
  // y = G (Cfb (T r - F y) + Cff T r)  ->  y/r = G T (Cfb + Cff) / (1 + G F Cfb)
  const CVec num = cx::mul(cx::mul(ol.G, ol.pid.T), cx::add(ol.pid.Cfb, ol.pid.Cff));
  c.Tcl = cx::div(num, onePlusL);
  c.S = cx::div(cx::one(freqs.size()), onePlusL);
  c.L = std::move(ol.L);
  c.G = std::move(ol.G);
  c.F = std::move(ol.F);
  c.pid = std::move(ol.pid);
  return c;
}

double sensitivityEnergy(const Vec& freqs, const CVec& S, double fMax) {
  double e = 0;
  for (std::size_t i = 1; i < freqs.size(); i++) {
    if (freqs[i] > fMax) break;
    const double m = std::hypot(S.re[i], S.im[i]);
    e += m * m * (freqs[i] - freqs[i - 1]);
  }
  return e;
}

double sensitivityBandwidth(const Vec& freqs, const CVec& S) {
  const Vec mag = cx::abs(S);
  for (std::size_t i = 0; i < mag.size(); i++)
    if (mag[i] > kSqrt1_2) return freqs[i];
  return freqs.back();
}

double bandwidth3db(const Vec& freqs, const CVec& T) {
  const Vec mag = cx::abs(T);
  const double ref = mag[0] != 0 && !std::isnan(mag[0]) ? mag[0] : 1;
  for (std::size_t i = 0; i < mag.size(); i++)
    if (mag[i] < ref * kSqrt1_2) return freqs[i];
  return freqs.back();
}

SimResult simulateRateStep(const LoopCtx& ctx, const PidGains& g, double stepDegs, double durationS, double uMax) {
  const std::size_t n = static_cast<std::size_t>(jsRound(durationS * ctx.fsLoop));
  const Vec ref(n, stepDegs);
  SimOptions o;
  o.uMax = uMax;
  return simulateRate(ctx, g, ref, o);
}

SimResult simulateRate(const LoopCtx& ctx, const PidGains& g, const Vec& ref, const SimOptions& opt) {
  const double fs = ctx.fsLoop, dt = 1 / fs;
  std::uint32_t rnd = opt.seed;
  auto rand = [&]() {
    rnd = rnd * 1664525u + 1013904223u;
    return static_cast<double>(rnd) / 4294967296.0;
  };
  auto gauss = [&]() {
    double u1 = rand();
    if (u1 == 0) u1 = 1e-9;
    const double u2 = rand();
    return std::sqrt(-2 * std::log(u1)) * std::cos(2 * kPi * u2);
  };
  const int sub = static_cast<int>(std::max(1.0, std::ceil(ctx.gyro.backendHz / fs)));
  const double dts = dt / sub;
  const Plant& p = ctx.plant;
  // plant state: rate (deg/s) with pole a; motor state with lag tau
  double rate = 0, motor = 0;
  const std::size_t delayN = static_cast<std::size_t>(std::max(0.0, jsRound(p.Td / dts)));
  Vec dq(delayN + 1, 0.0);
  // gyro 2-pole LPF at the backend rate (same coefficients as DigitalBiquad)
  const double fc = ctx.gyro.gyroFilterHz, bfs = ctx.gyro.backendHz;
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  const bool useLpf = fc > 0 && fc < bfs / 2;
  if (useLpf) {
    const double fr = bfs / fc, ohm = std::tan(kPi / fr), c = 1 + 2 * std::cos(kPi / 4) * ohm + ohm * ohm;
    b0 = ohm * ohm / c;
    b1 = 2 * b0;
    b2 = b0;
    a1 = 2 * (ohm * ohm - 1) / c;
    a2 = (1 - 2 * std::cos(kPi / 4) * ohm + ohm * ohm) / c;
  }
  double d1 = 0, d2 = 0, gyro = 0;
  const double aLag = p.tau > 0 ? 1 - std::exp(-dts / p.tau) : 1;
  const double eA = std::exp(-p.a * dts), gA = p.a > 1e-9 ? (1 - eA) / p.a : dts;
  // PID state
  double tgt = 0, err = 0, der = 0, integ = 0;
  bool first = true;
  const double aT = g.FLTT > 0 ? dt / (dt + 1 / (2 * kPi * g.FLTT)) : 1;
  const double aE = g.FLTE > 0 ? dt / (dt + 1 / (2 * kPi * g.FLTE)) : 1;
  const double aD = g.FLTD > 0 ? dt / (dt + 1 / (2 * kPi * g.FLTD)) : 1;
  const std::size_t n = ref.size();
  SimResult r;
  r.t.resize(n);
  r.y.resize(n);
  r.u.resize(n);
  r.ytrue.resize(n);
  std::size_t di = 0;
  for (std::size_t k = 0; k < n; k++) {
    const double target = ref[k];
    r.y[k] = gyro;
    r.ytrue[k] = rate;
    // AC_PID::update_all
    if (first) {
      tgt = target;
      err = tgt - gyro;
      der = 0;
      first = false;
    } else {
      tgt += aT * (target - tgt);
      const double e = tgt - gyro;
      const double eLast = err;
      err += aE * (e - err);
      der += aD * ((err - eLast) / dt - der);
    }
    integ += err * g.I * dt;
    double out = g.P * err + g.D * der + integ + g.FF * tgt;
    if (out > opt.uMax)
      out = opt.uMax;
    else if (out < -opt.uMax)
      out = -opt.uMax;
    r.u[k] = out;
    r.t[k] = static_cast<double>(k) * dt;
    // plant + sensor at the backend rate
    for (int s = 0; s < sub; s++) {
      dq[di] = out;
      di = (di + 1) % (delayN + 1);
      const double ud = dq[di];                                  // delayed input
      motor += (ud - motor) * aLag;                              // first-order lag (tau), exact step
      const double acc = p.K * motor + (opt.dist ? (*opt.dist)[k] : 0);   // (s + a), plus torque disturbance
      rate = rate * eA + acc * gA;
      const double xin = rate + (opt.noiseRms != 0 ? gauss() * opt.noiseRms : 0);
      const double yv = b0 * xin + d1;
      d1 = b1 * xin - a1 * yv + d2;
      d2 = b2 * xin - a2 * yv;
      gyro = useLpf ? yv : xin;
    }
  }
  return r;
}

namespace {

PidGains withGains(const PidGains& filters, double P, double I, double D) {
  PidGains g;
  g.FLTT = filters.FLTT;
  g.FLTE = filters.FLTE;
  g.FLTD = filters.FLTD;
  g.FF = 0;
  g.DFF = 0;
  g.P = P;
  g.I = I;
  g.D = D;
  return g;
}

double peakOf(const Vec& y) {
  double peak = 0;
  for (double v : y)
    if (v > peak) peak = v;
  return peak;
}

/** RMS of the D-term output caused by gyro noise */
double dNoiseRms(const LoopCtx& ctx, const SynthOptions& opt, double D) {
  if (!opt.noisePsd) return 0;
  const PidGains g = withGains(opt.filters, 0, 0, D);
  const NoisePsd& np = *opt.noisePsd;
  const PidResponse pid = acPid(np.f, ctx.fsLoop, g);
  const Vec H = opt.noisePsdIsFiltered ? cx::abs(pid.D) : cx::abs(cx::mul(pid.D, gyroChain(np.f, ctx.gyro)));
  double e = 0;
  const double df = np.f.size() > 1 ? np.f[1] - np.f[0] : 0;
  for (std::size_t i = 0; i < H.size(); i++) e += H[i] * H[i] * np.psd[i] * df;
  return std::sqrt(e);
}

}  // namespace

std::optional<SynthResult> synthesize(const LoopCtx& ctx, const SynthOptions& opt) {
  const Vec freqs = logspace(0.1, std::min(ctx.fsLoop / 2, 400.0), 400);
  const double iRatio = opt.iRatio;   // I = P * iRatio
  const double pMax = opt.pRange[1], dMin = opt.dRange[0], dMax = opt.dRange[1];
  const double pMin = std::min(opt.pRange[0], 1e-4);   // search below the vehicle's nominal minimum
  auto ok = [&](const Margins& m) {
    return std::isfinite(m.pmDeg) && m.pmDeg >= opt.pmMinDeg && (std::isnan(m.gmDb) || m.gmDb >= opt.gmMinDb) &&
           m.ms <= opt.msMax;
  };
  // candidate derivative-time ratios D/P (seconds): 0 and a geometric sweep
  Vec ratios{0};
  for (double r = 0.004; r <= opt.rMax; r *= 1.3) ratios.push_back(r);
  std::vector<SynthCandidate> candidates;
  struct Test {
    Margins m;
    PidGains g;
    double os;
    bool pass;
  };
  for (const double r : ratios) {
    double lo = std::log(pMin), hi = std::log(pMax);
    auto test = [&](double P) {
      const double D = std::min(dMax, P * r);
      const PidGains g = withGains(opt.filters, P, P * iRatio, D);
      const Margins m = margins(freqs, openLoop(freqs, ctx, g).L);
      double os = kNaN;
      if (ok(m)) {   // overshoot check by time simulation on the identified plant (rate step, actuator limited)
        const SimResult st = simulateRateStep(ctx, g, opt.stepMag, opt.stepDurS, opt.uMax);
        os = std::max(0.0, (peakOf(st.y) / opt.stepMag - 1) * 100);
      }
      return Test{m, g, os, ok(m) && os <= opt.osMaxPct};
    };
    std::optional<Test> best;
    const Test t0 = test(std::exp(lo));
    if (!t0.pass) {
      SynthCandidate c;
      c.r = r;
      c.fail = "fail";
      candidates.push_back(c);
      continue;
    }
    for (int it = 0; it < 24; it++) {
      const double mid = 0.5 * (lo + hi);
      Test tm = test(std::exp(mid));
      if (tm.pass) {
        lo = mid;
        best = tm;
      } else {
        hi = mid;
      }
    }
    if (!best) best = t0;
    const double D = best->g.D;
    const double dn = dNoiseRms(ctx, opt, D);
    if (opt.dNoiseCap != 0 && dn > opt.dNoiseCap) {
      SynthCandidate c;
      c.r = r;
      c.P = best->g.P;
      c.D = D;
      c.noise = dn;
      c.fail = "noise";
      candidates.push_back(c);
      continue;
    }
    if (D < dMin && r > 0) {
      SynthCandidate c;
      c.r = r;
      c.P = best->g.P;
      c.D = D;
      c.fail = "dmin";
      candidates.push_back(c);
      continue;
    }
    const ClosedLoop cl = closedLoop(freqs, ctx, best->g);
    SynthCandidate c;
    c.r = r;
    c.P = best->g.P;
    c.D = D;
    c.I = best->g.I;
    c.m = best->m;
    c.bwHz = bandwidth3db(freqs, cl.Tcl);
    c.sBwHz = sensitivityBandwidth(freqs, cl.S);
    c.sEnergy = sensitivityEnergy(freqs, cl.S, std::min(10.0, ctx.fsLoop / 20));
    c.noise = dn;
    c.overshootPct = best->os;
    candidates.push_back(c);
  }
  std::vector<SynthCandidate> good;
  for (const auto& c : candidates)
    if (!c.fail) good.push_back(c);
  if (good.empty()) return std::nullopt;
  // largest P that meets the margins; ties go to the lower sensitivity peak (stable, as Array.sort)
  std::stable_sort(good.begin(), good.end(), [](const SynthCandidate& a, const SynthCandidate& b) {
    const double d = (b.P - a.P);
    const double k = d != 0 && !std::isnan(d) ? d : (a.m->ms - b.m->ms);
    return k < 0;
  });
  const SynthCandidate& pick = good[0];
  SynthResult res;
  res.gains = withGains(opt.filters, pick.P, pick.I, pick.D);
  res.margins = *pick.m;
  res.bandwidthHz = pick.bwHz;
  res.sBwHz = pick.sBwHz;
  res.overshootPct = pick.overshootPct;
  res.dNoise = pick.noise;
  res.candidates = std::move(candidates);
  res.freqs = freqs;
  return res;
}

std::optional<SynthResult> synthesizeQuik(const LoopCtx& ctx, const SynthOptions& opt) {
  const Vec freqs = logspace(0.1, std::min(ctx.fsLoop / 2, 400.0), 400);
  const double iRatio = opt.iRatio, backoff = opt.backoff;
  const double pMax = opt.pRange[1], dMin = opt.dRange[0], dMax = opt.dRange[1];
  const double pMin = std::min(opt.pRange[0], 1e-4);
  const double stepMag = opt.stepMag, osMax = opt.osMaxPct;
  auto gainsOf = [&](double P, double D) { return withGains(opt.filters, P, P * iRatio, D); };
  auto stable = [&](double P, double D) {
    const Margins m = margins(freqs, openLoop(freqs, ctx, gainsOf(P, D)).L);
    return (std::isnan(m.gmDb) || m.gmDb > 0) && (std::isnan(m.pmDeg) || m.pmDeg > 0);
  };
  // largest x in [lo, hi] (log scale) for which pred(x) holds; assumes pred true at lo
  auto bisect = [](double lo, double hi, const std::function<bool(double)>& pred) -> std::optional<double> {
    if (!pred(lo)) return std::nullopt;
    if (pred(hi)) return hi;
    double a = std::log(lo), b = std::log(hi);
    for (int i = 0; i < 26; i++) {
      const double m = 0.5 * (a + b);
      if (pred(std::exp(m)))
        a = m;
      else
        b = m;
    }
    return std::exp(a);
  };
  auto dNoise = [&](double D) { return dNoiseRms(ctx, opt, D); };
  // JS: opts.currentP || 0.1 (0 and NaN fall back to 0.1)
  double P0 = std::min(pMax, std::max(pMin, opt.currentP != 0 && !std::isnan(opt.currentP) ? opt.currentP : 0.1));
  double D0 = std::min(dMax, std::max(0.0, opt.currentD));
  // as in the manual guide: if the current gains would oscillate, halve P (and D) until they do not
  for (int i = 0; i < 12 && !stable(P0, D0) && P0 > pMin; i++) {
    P0 = std::max(pMin, P0 * 0.5);
    D0 = D0 * 0.5 < 1e-6 ? 0 : D0 * 0.5;
  }
  // 1) D up to the oscillation point with that P (D may legitimately be 0 on some airframes)
  const double dLo = 1e-6;
  const std::optional<double> dOsc =
      stable(P0, dLo) ? bisect(dLo, dMax, [&](double D) { return stable(P0, D); }) : std::nullopt;
  double D = !dOsc ? 0 : (*dOsc >= dMax ? dMax : *dOsc * backoff);
  if (opt.dNoiseCap != 0 && D > 0) {
    const double n = dNoise(D);
    if (n > opt.dNoiseCap) {
      const auto dCap = bisect(dLo, D, [&](double x) { return dNoise(x) <= opt.dNoiseCap; });
      D = dCap ? *dCap : 0;
    }
  }
  if (D > 0 && D < dMin) D = stable(P0, dMin) ? dMin : 0;
  D = std::min(dMax, D);
  // 2) P up to the oscillation point with that D
  const std::optional<double> pOsc = bisect(pMin, pMax, [&](double P) { return stable(P, D); });
  double P = !pOsc ? pMin : (*pOsc >= pMax ? pMax : *pOsc * backoff);
  // 3) phase margin / sensitivity / overshoot limits (lower P until they hold)
  auto okAll = [&](double Pv) {
    const PidGains g = gainsOf(Pv, D);
    const Margins m = margins(freqs, openLoop(freqs, ctx, g).L);
    if (!(std::isfinite(m.pmDeg) && m.pmDeg >= opt.pmMinDeg && (std::isnan(m.gmDb) || m.gmDb >= opt.gmMinDb) &&
          m.ms <= opt.msMax))
      return false;
    const SimResult st = simulateRateStep(ctx, g, stepMag, opt.stepDurS, opt.uMax);
    return (peakOf(st.y) / stepMag - 1) * 100 <= osMax;
  };
  if (!okAll(P)) {
    const auto Pc = bisect(pMin, P, okAll);
    if (Pc) P = *Pc;
  }
  const PidGains g = gainsOf(P, D);
  SynthResult res;
  res.gains = g;
  res.margins = margins(freqs, openLoop(freqs, ctx, g).L);
  const ClosedLoop cl = closedLoop(freqs, ctx, g);
  res.bandwidthHz = bandwidth3db(freqs, cl.Tcl);
  res.sBwHz = sensitivityBandwidth(freqs, cl.S);
  const SimResult st = simulateRateStep(ctx, g, stepMag, opt.stepDurS, opt.uMax);
  res.overshootPct = std::max(0.0, (peakOf(st.y) / stepMag - 1) * 100);
  res.dNoise = dNoise(D);
  OscillationPoints op;
  op.D = dOsc;
  op.P = pOsc;
  op.backoff = backoff;
  op.startP = P0;
  res.oscillationPoints = op;
  res.freqs = freqs;
  return res;
}

}  // namespace aptuner::core
