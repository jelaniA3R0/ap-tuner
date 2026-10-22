// Model of one ArduPilot attitude axis: linear margins and the nonlinear time simulation (see cascade.hpp).
#include "aptuner/core/cascade.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace aptuner::core::cascade {

namespace {

constexpr double kPi = 3.14159265358979323846;
const Cx J{0, 1};

/** first-order low-pass 1 / (1 + s / (2 pi fc)); fc <= 0 -> 1 */
Cx lp1(double fc, Cx s) { return fc > 0 ? 1.0 / (1.0 + s / (2 * kPi * fc)) : Cx{1, 0}; }
/** 2-pole Butterworth low-pass at fc */
Cx butter2(double fc, Cx s) {
  if (!(fc > 0)) return {1, 0};
  const double w0 = 2 * kPi * fc;
  return 1.0 / (1.0 + std::sqrt(2.0) * s / w0 + s * s / (w0 * w0));
}
/** rate PID without the error filter: P + I / s + D s FD */
Cx ratePid(const Gains& g, Cx s) { return g.P + g.I / s + g.D * s * lp1(g.FLTD, s); }

/** ArduPilot calc_lowpass_alpha_dt */
double lpAlpha(double dt, double fc) {
  if (!(fc > 0) || !(dt > 0)) return 1;
  const double rc = 1 / (2 * kPi * fc);
  return dt / (dt + rc);
}

double clampD(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

/** the plant G(jw) */
Cx plantAt(const Plant& p, double w) {
  const Cx s = J * w;
  return p.a * std::exp(-s * p.T) / ((s + p.c) * (1.0 + p.tau * s));
}

}  // namespace

double wrap180(double deg) {
  double x = std::fmod(deg + 180.0, 360.0);
  if (x <= 0) x += 360.0;
  return x - 180.0;
}

// ------------------------------------------------------------------------------------------------- linear analysis

Cx controllerAt(const Gains& g, double w) {
  const Cx s = J * w;
  return lp1(g.FLTE, s) * ratePid(g, s) * (lp1(g.FLTT, s) * g.angleP / s + butter2(g.gyroHz, s));
}

Cx loopAt(const Gains& g, const Plant& p, double w) { return plantAt(p, w) * controllerAt(g, w); }

double noiseGain(const Gains& g, double fHz) {
  const Cx s = J * (2 * kPi * fHz);
  return std::abs(lp1(g.FLTE, s) * ratePid(g, s) * butter2(g.gyroHz, s));
}

Margins margins(const Gains& g, const Plant& p) {
  // log grid 0.03..500 rad/s, the phase unwrapped along it
  constexpr double kStep = 0.003;
  const double l0 = std::log(0.03), l1 = std::log(500.0);
  const std::size_t n = static_cast<std::size_t>((l1 - l0) / kStep) + 1;
  Vec w(n), mag(n), ph(n);
  double off = 0, prev = 0;
  Margins m;
  for (std::size_t i = 0; i < n; ++i) {
    w[i] = std::exp(l0 + kStep * static_cast<double>(i));
    const Cx L = loopAt(g, p, w[i]);
    mag[i] = std::abs(L);
    double a = std::arg(L) * 180 / kPi;
    if (i) {
      while (a + off - prev > 180) off -= 360;
      while (a + off - prev < -180) off += 360;
    }
    ph[i] = a + off;
    prev = ph[i];
    m.ms = std::max(m.ms, 1 / std::abs(1.0 + L));
  }
  std::size_t ic = 0;
  for (std::size_t i = 1; i < n; ++i)
    if (mag[i - 1] >= 1 && mag[i] < 1) {
      m.wcRad = w[i];
      double pm = 180 + ph[i];
      while (pm > 180) pm -= 360;
      while (pm <= -180) pm += 360;
      m.pmDeg = pm;
      ic = i;
    }
  // gain margin: the first odd multiple of 180 deg crossed at or above the last gain crossover
  for (std::size_t i = std::max<std::size_t>(1, ic); i < n; ++i) {
    const double k = std::floor((ph[i] + 180) / 360), k0 = std::floor((ph[i - 1] + 180) / 360);
    if (k != k0) {
      m.gmDb = -20 * std::log10(mag[i]);
      m.w180Rad = w[i];
      break;
    }
  }
  return m;
}

WorstMargins worstMargins(const Gains& g, const std::vector<Plant>& set, double stopBelowPm) {
  WorstMargins wm;
  for (std::size_t i = 0; i < set.size(); ++i) {
    const Margins m = margins(g, set[i]);
    const double pm = m.pmDeg ? *m.pmDeg : -99;
    if (pm < wm.pmDeg) {
      wm.pmDeg = pm;
      wm.wcAtWorstPmRad = m.wcRad;
      wm.worstPlant = i;
    }
    wm.gmDb = std::min(wm.gmDb, m.gmDb);
    wm.ms = std::max(wm.ms, m.ms);
    wm.slowestWcRad = std::min(wm.slowestWcRad, m.wcRad);
    if (wm.pmDeg < stopBelowPm) break;
  }
  return wm;
}

Poles slowAxisPoles(double a, double c, double P, double I, double K) {
  const double b2 = a * P + c, b1 = a * (P * K + I), b0 = a * I * K;
  auto f = [&](Cx s) { return s * s * s + b2 * s * s + b1 * s + b0; };
  std::vector<Cx> z{{-0.4, 0.9}, {-0.4, -0.9}, {-1, 0.1}};
  for (int it = 0; it < 500; ++it)
    for (std::size_t i = 0; i < z.size(); ++i) {
      Cx den{1, 0};
      for (std::size_t j = 0; j < z.size(); ++j)
        if (j != i) den *= z[i] - z[j];
      z[i] -= f(z[i]) / den;
    }
  Poles out;
  out.roots = z;
  for (const Cx& r : z) {
    if (r.real() >= 0) out.stable = false;
    if (std::fabs(r.imag()) > 1e-6 && r.imag() > 0) {
      const double wn = std::abs(r), zeta = -r.real() / wn;
      if (zeta < out.zeta) {
        out.zeta = zeta;
        out.wnRad = wn;
        out.periodS = 2 * kPi / r.imag();
      }
    }
  }
  return out;
}

std::optional<MarginalFit> marginalModeFit(const Gains& flown, double w0, double T, double c) {
  const Cx C = controllerAt(flown, w0);
  if (std::abs(C) < 1e-12) return std::nullopt;
  const Cx G = -1.0 / C;
  // G = a e^(-jwT) / ((jw + c)(1 + j w tau)): phase = -atan2(w, c) - atan(w tau) - w T
  double ph = std::arg(G);   // (-pi, pi]
  double lag = -ph - std::atan2(w0, c);   // = atan(w tau) + w T, in (0, pi) for this plant form
  while (lag < 0) lag += 2 * kPi;
  while (lag >= 2 * kPi) lag -= 2 * kPi;
  const double rest = lag - w0 * T;
  if (!(rest > 0 && rest < kPi / 2)) return std::nullopt;
  MarginalFit m;
  m.tau = std::tan(rest) / w0;
  m.T = T;
  m.lagDeg = lag * 180 / kPi;
  m.a = std::abs(G) * std::abs(Cx{c, w0}) * std::sqrt(1 + m.tau * m.tau * w0 * w0);
  return m;
}

// ------------------------------------------------------------------------------------------------- time domain

double sqrtController(double error, double p, double secondOrdLim, double dt) {
  double c;
  if (!(secondOrdLim > 0)) {
    c = error * p;
  } else if (p == 0) {
    c = error > 0 ? std::sqrt(2 * secondOrdLim * error) : error < 0 ? -std::sqrt(-2 * secondOrdLim * error) : 0;
  } else {
    const double ld = secondOrdLim / (p * p);
    if (error > ld) c = std::sqrt(2 * secondOrdLim * (error - ld / 2));
    else if (error < -ld) c = -std::sqrt(2 * secondOrdLim * (-error - ld / 2));
    else c = error * p;
  }
  if (dt > 0) c = clampD(c, -std::fabs(error) / dt, std::fabs(error) / dt);
  return c;
}

double invSqrtController(double output, double p, double dMax) {
  if (dMax > 0 && p == 0) return output * output / (2 * dMax);
  if (!(dMax > 0) && p != 0) return output / p;
  if (!(dMax > 0) && p == 0) return 0;
  const double linearVelocity = dMax / p;
  if (std::fabs(output) < linearVelocity) return output / p;
  const double linearDist = dMax / (p * p);
  const double stop = linearDist * 0.5 + output * output / (2 * dMax);
  return output > 0 ? stop : -stop;
}

double angleControllerAccel(const Gains& g) {
  if (!(g.accMax > 0)) return 0;
  const double half = g.accMax * kD2R / 2;
  return g.kind == Kind::Yaw ? clampD(half, 10 * kD2R, 120 * kD2R) : clampD(half, 40 * kD2R, 720 * kD2R);
}

double yawHeadingErrorMax(const Gains& g) {
  const double cap = 45 * kD2R;
  if (!(g.P > 0)) return cap;
  // AC_AttitudeControl::thrust_heading_rotation_angles uses clamp(ACC_Y/2, 10..120) even when ACC_Y_MAX is 0
  const double acc = clampD(g.accMax * kD2R / 2, 10 * kD2R, 120 * kD2R);
  return std::min(invSqrtController(1 / g.P, g.angleP, acc), cap);
}

double yawStickToRate(double pwm, const Gains& g) {
  double x = 0;
  if (pwm > g.rcTrim + g.rcDz) x = (pwm - g.rcTrim - g.rcDz) / (g.rcMax - g.rcTrim - g.rcDz);
  else if (pwm < g.rcTrim - g.rcDz) x = -(g.rcTrim - g.rcDz - pwm) / (g.rcTrim - g.rcDz - g.rcMin);
  x = clampD(x, -1, 1);
  const double e = g.pltExpo;
  x = (1 - e) * x + e * x * x * x;
  return x * g.pltRate * kD2R;
}

GustSequence::GustSequence(double seconds, double hz, std::uint32_t seed) : hz_(hz) {
  const std::size_t n = static_cast<std::size_t>(std::ceil(seconds * hz)) + 2;
  v_.resize(n);
  std::uint64_t s = seed;
  double gs = 0;
  for (std::size_t k = 0; k < n; ++k) {
    s = (s * 16807ULL) % 2147483647ULL;
    const double rnd = static_cast<double>(s) / 2147483647.0 - 0.5;
    gs += 0.02 * ((rnd * 2) * 1.2 - gs);
    v_[k] = gs / 5.8;   // rad/s/s on a = 5.8 -> output units
  }
}

double GustSequence::at(double t) const {
  if (v_.empty()) return 0;
  const auto k = static_cast<std::size_t>(std::max(0.0, std::floor(t * hz_)));
  return v_[std::min(k, v_.size() - 1)];
}

SimOutput simulate(const Gains& g, const Plant& p, const SimInput& in) {
  const double dt = 1 / (g.loopHz > 0 ? g.loopHz : 300);
  const auto N = static_cast<std::size_t>(std::llround(in.seconds / dt));
  const bool yaw = g.kind == Kind::Yaw;
  const double aT = lpAlpha(dt, g.FLTT), aE = lpAlpha(dt, g.FLTE), aD = lpAlpha(dt, g.FLTD);
  const double aM = p.tau > 0 ? 1 - std::exp(-dt / p.tau) : 1;
  const double accShape = g.accMax > 0 ? g.accMax * kD2R : 0;   // input shaping uses the full ACC limit
  const double accCtl = angleControllerAccel(g);
  const double rmax = g.rateMax > 0 ? g.rateMax * kD2R : 0;
  const double headingMax = yaw ? yawHeadingErrorMax(g) : 0;
  // gyro 2-pole Butterworth at the loop rate
  bool useGyro = g.gyroHz > 0 && g.gyroHz < 0.5 / dt;
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  if (useGyro) {
    const double K = std::tan(kPi * g.gyroHz * dt), norm = 1 / (1 + std::sqrt(2.0) * K + K * K);
    b0 = K * K * norm;
    b1 = 2 * b0;
    b2 = b0;
    a1 = 2 * (K * K - 1) * norm;
    a2 = (1 - std::sqrt(2.0) * K + K * K) * norm;
  }
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  const auto nDelay = static_cast<std::size_t>(std::llround(std::max(0.0, p.T + in.extraDelay) / dt));
  std::deque<double> buf(nDelay + 1, 0.0);
  std::uint64_t seed = in.seed ? in.seed : 1;
  auto rnd = [&seed]() {
    seed = (seed * 1103515245ULL + 12345ULL) % 2147483648ULL;
    return static_cast<double>(seed) / 2147483648.0 - 0.5;
  };

  double att = in.initialAngleDeg * kD2R, rate = 0, um = 0, umPrev = 0;
  double tgt = att, ffr = 0;   // attitude target and its feed-forward rate
  double tF = 0, eF = 0, dF = 0, I = in.initialI;
  bool first = true, limit = false;
  std::size_t limCount = 0, nJ = 0;
  double du2 = 0, uPrev = 0;
  const std::size_t recEvery = std::max<std::size_t>(1, static_cast<std::size_t>(std::llround(1 / (dt * in.recordHz))));
  SimOutput out;
  for (std::size_t k = 0; k < N; ++k) {
    const double t = static_cast<double>(k) * dt;
    // ---- pilot input and its shaping
    if (yaw) {
      const double des = in.stickPwm ? yawStickToRate(in.stickPwm(t), g) : 0;
      double dv = des;
      if (g.pltRateTC > 0) dv = ffr + sqrtController(des - ffr, 1 / std::max(g.pltRateTC, 0.01), 0, dt) * dt;
      if (accShape > 0) dv = clampD(dv, ffr - accShape * dt, ffr + accShape * dt);
      ffr = dv;
      tgt += ffr * dt;
    } else {
      const double raw = in.angleDeg ? in.angleDeg(t) * kD2R : 0;
      if (in.shaped) {
        ffr = k ? (raw - tgt) / dt : 0;
        tgt = raw;
      } else {
        double dv = sqrtController(raw - tgt, 1 / std::max(g.inputTC, 0.01), accShape, dt);
        if (accShape > 0) dv = clampD(dv, ffr - accShape * dt, ffr + accShape * dt);
        ffr = dv;
        tgt += ffr * dt;
      }
    }
    if (rmax > 0) ffr = clampD(ffr, -rmax, rmax);
    // ---- angle controller
    double e = yaw ? wrap180((tgt - att) / kD2R) * kD2R : tgt - att;
    if (yaw && std::fabs(e) > headingMax) {
      e = e > 0 ? headingMax : -headingMax;
      tgt = att + e;   // the target is dragged along
    }
    double corr = sqrtController(e, g.angleP, accCtl, dt);
    if (rmax > 0) corr = clampD(corr, -rmax, rmax);
    const double rt = corr + ffr;
    // ---- gyro
    const double meas0 = rate + (in.gyroNoiseDegs > 0 ? in.gyroNoiseDegs * kD2R * rnd() * std::sqrt(12.0) : 0);
    double meas = meas0;
    if (useGyro) {
      meas = b0 * meas0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
      x2 = x1;
      x1 = meas0;
      y2 = y1;
      y1 = meas;
    }
    // ---- rate PID (AC_PID::update_all)
    if (first) {
      tF = rt;
      eF = tF - meas;
      dF = 0;
      first = false;
    } else {
      tF += aT * (rt - tF);
      const double eLast = eF;
      eF += aE * ((tF - meas) - eF);
      dF += aD * ((eF - eLast) / dt - dF);
    }
    if (g.I > 0) {
      if (!limit || (I > 0 && eF < 0) || (I < 0 && eF > 0)) I = clampD(I + eF * g.I * dt, -g.IMAX, g.IMAX);
    } else {
      I = 0;
    }
    double u = g.P * eF + I + g.D * dF + g.FF * tF;
    limit = std::fabs(u) > p.umax;
    if (limit) {
      u = u > 0 ? p.umax : -p.umax;
      ++limCount;
    }
    if (k >= N / 2) {
      du2 += (u - uPrev) * (u - uPrev);
      ++nJ;
    }
    uPrev = u;
    // ---- delay, motor lag, vehicle
    buf.push_back(u);
    const double ud = buf.front();
    buf.pop_front();
    umPrev = um;
    um += aM * (ud - um);
    const double lead = in.lead > 0 ? in.lead * (um - umPrev) / dt : 0;
    const double d = in.dist ? in.dist(t) : 0;
    rate += (p.a * (um + lead - p.trim - d) - p.c * rate) * dt;
    att += rate * dt;
    if (k % recEvery == 0) {
      out.t.push_back(t);
      out.att.push_back(yaw ? wrap180(att / kD2R) : att / kD2R);
      out.rate.push_back(rate / kD2R);
      out.u.push_back(u);
      out.I.push_back(I);
      out.target.push_back(yaw ? wrap180(tgt / kD2R) : tgt / kD2R);
    }
  }
  out.limitedFrac = N ? static_cast<double>(limCount) / static_cast<double>(N) : 0;
  out.outputJitter = nJ ? std::sqrt(du2 / static_cast<double>(nJ)) / dt : 0;
  return out;
}

}  // namespace aptuner::core::cascade
