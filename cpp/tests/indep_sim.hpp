// Independent closed-loop simulator of one multirotor rate axis (port of test/indep_sim.js, written for the audit
// from the ArduPilot formulas: AC_PID::update_all / update_i, calc_lowpass_alpha_dt, DigitalBiquadFilter, linear
// angle P + input-shaping feed-forward). It uses no AP Tuner code.
//   plant (continuous, 8 kHz sub-steps):  w' = K * m(t) - a * w + d(t),  m = first-order lag (tau) of u(t - Td)
//   gyro: sampled at 1 kHz, + white noise, 2-pole Butterworth at INS_GYRO_FILTER
//   rate PID at 400 Hz reads the latest filtered gyro sample; output held (ZOH) until the next loop
//   units: rad/s, rad/s^2, mixer units (as ArduPilot Copter)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace aptuner::test {

/** the simulator's random numbers (mulberry32 uniform, polar Box-Muller normal) */
struct SimRng {
  std::uint32_t s;
  std::optional<double> spare;
  explicit SimRng(std::uint32_t seed) : s(seed) {}
  double u() {
    s += 0x6D2B79F5u;
    std::uint32_t t = s;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return static_cast<double>(t ^ (t >> 14)) / 4294967296.0;
  }
  double n() {
    if (spare) {
      const double v = *spare;
      spare.reset();
      return v;
    }
    double a, b, r;
    do {
      a = 2 * u() - 1;
      b = 2 * u() - 1;
      r = a * a + b * b;
    } while (r >= 1 || r == 0);
    const double m = std::sqrt(-2 * std::log(r) / r);
    spare = b * m;
    return a * m;
  }
};

struct SimPlant {
  double K = 0, a = 0, tau = 0, Td = 0;
};
struct SimGains {
  double P = 0, I = 0, D = 0, FF = 0, FLTT = 0, FLTE = 0, FLTD = 0, IMAX = 0;
};
struct SimPilot {
  std::uint32_t seed = 99;
  std::array<double, 2> angAmp{0, 0}, rateAmp{0, 0}, hold{0, 0}, gap{0, 0};
};
struct SimOpts {
  SimPlant plant;
  SimGains gains;
  double fsLoop = 400, backendHz = 1000, subHz = 8000, gyroHz = 0, dur = 60;
  std::string mode = "acro";   // "acro" | "angle" | "none"
  double angleP = 0, inputTc = 0;
  std::optional<SimPilot> pilot;
  double gyroNoise = 0;   // rad/s RMS at the backend rate
  double kick = 0;        // rad/s initial rate
  std::uint32_t seed = 1;
};
struct SimOut {
  double fs = 0;
  std::vector<double> target, actual, P, I, D, FF, wTrue, theta;
};

inline SimOut simulate(const SimOpts& o) {
  constexpr double kPi = 3.141592653589793;
  const double fsLoop = o.fsLoop, backend = o.backendHz, sub = o.subHz;
  const double h = 1 / sub, dt = 1 / fsLoop;
  const long nPerGyro = std::lround(sub / backend), nPerLoop = std::lround(sub / fsLoop);
  const SimPlant& p = o.plant;
  const SimGains& g = o.gains;
  SimRng rng(o.seed), prng(o.pilot ? o.pilot->seed : 99);
  const long N = std::lround(o.dur * sub), nLoop = N / nPerLoop;
  // DigitalBiquadFilter coefficients
  bool bq = o.gyroHz > 0;
  double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  if (bq) {
    const double fr = backend / o.gyroHz, ohm = std::tan(kPi / fr), c = 1 + 2 * std::cos(kPi / 4) * ohm + ohm * ohm;
    b0 = ohm * ohm / c;
    b1 = 2 * b0;
    b2 = b0;
    a1 = 2 * (ohm * ohm - 1) / c;
    a2 = (1 - 2 * std::cos(kPi / 4) * ohm + ohm * ohm) / c;
  }
  double gx1 = 0, gx2 = 0, gy1 = 0, gy2 = 0, gyroF = 0;
  const auto alpha = [&](double fc) { return fc > 0 ? dt / (dt + 1 / (2 * kPi * fc)) : 1.0; };
  const double aT = alpha(g.FLTT), aE = alpha(g.FLTE), aD = alpha(g.FLTD);
  double tgtF = 0, errF = 0, der = 0, integ = 0;
  bool first = true;
  const long nd = std::max(0L, std::lround(p.Td / h));
  std::vector<double> dq(static_cast<std::size_t>(nd + 1), 0.0);
  long di = 0;
  const double lagA = p.tau > 0 ? 1 - std::exp(-h / p.tau) : 1, eA = std::exp(-p.a * h), gA = p.a > 1e-12 ? (1 - eA) / p.a : h;
  double w = o.kick, th = 0, m = 0, d = 0, u = 0;
  const double uLim = std::numeric_limits<double>::infinity();
  const SimPilot pil = o.pilot ? *o.pilot : SimPilot{};
  double cmd = 0, thT = 0, rateShaped = 0;
  long cmdLeft = 0;
  bool inHold = false;
  const auto nextCmd = [&]() {
    if (inHold) {
      inHold = false;
      cmd = 0;
      cmdLeft = std::lround((pil.gap[0] + (pil.gap[1] - pil.gap[0]) * prng.u()) * fsLoop);
    } else {
      inHold = true;
      const auto& amp = o.mode == "angle" ? pil.angAmp : pil.rateAmp;
      const double sgn = prng.u() < 0.5 ? -1 : 1;
      const double mag = amp[0] + (amp[1] - amp[0]) * prng.u();
      cmd = sgn * mag * kPi / 180;
      cmdLeft = std::lround((pil.hold[0] + (pil.hold[1] - pil.hold[0]) * prng.u()) * fsLoop);
    }
  };
  SimOut out;
  out.fs = fsLoop;
  for (auto* v : {&out.target, &out.actual, &out.P, &out.I, &out.D, &out.FF, &out.wTrue, &out.theta})
    v->assign(static_cast<std::size_t>(nLoop), 0.0);
  long k = 0;
  for (long s = 0; s < N; s++) {
    if (s % nPerGyro == 0) {
      const double x = w + (o.gyroNoise != 0 ? o.gyroNoise * rng.n() : 0);
      if (bq) {
        const double y = b0 * x + b1 * gx1 + b2 * gx2 - a1 * gy1 - a2 * gy2;
        gx2 = gx1;
        gx1 = x;
        gy2 = gy1;
        gy1 = y;
        gyroF = y;
      } else {
        gyroF = x;
      }
    }
    if (s % nPerLoop == 0 && k < nLoop) {
      // pilot / outer loop -> rate target (rad/s)
      double rt = 0;
      if (o.mode != "none" && o.pilot) {
        if (cmdLeft <= 0) nextCmd();
        cmdLeft--;
      }
      if (o.mode == "acro") {
        if (o.inputTc > 0) {
          rateShaped += (cmd - rateShaped) * (dt / o.inputTc);
          rt = rateShaped;
        } else {
          rt = o.pilot ? cmd : 0;
        }
      } else if (o.mode == "angle") {
        const double ff = (cmd - thT) / o.inputTc;
        thT += ff * dt;
        rt = ff + o.angleP * (thT - th);
      }
      // AC_PID::update_all (measurement = latest filtered gyro), then update_i, out = P + I + D (+ FF)
      if (first) {
        tgtF = rt;
        errF = tgtF - gyroF;
        der = 0;
        first = false;
      } else {
        const double eLast = errF;
        tgtF += aT * (rt - tgtF);
        errF += aE * ((tgtF - gyroF) - errF);
        der += aD * ((errF - eLast) / dt - der);
      }
      const bool limited = std::fabs(u) >= uLim;
      if (g.I > 0) {
        if (!limited || (integ > 0 && errF < 0) || (integ < 0 && errF > 0)) {
          integ += errF * g.I * dt;
          if (g.IMAX > 0) integ = std::max(-g.IMAX, std::min(g.IMAX, integ));
        }
      } else {
        integ = 0;
      }
      const double Pout = g.P * errF, Dout = g.D * der, FFout = g.FF * tgtF;
      u = Pout + Dout + integ + FFout;
      if (u > uLim)
        u = uLim;
      else if (u < -uLim)
        u = -uLim;
      const std::size_t kk = static_cast<std::size_t>(k);
      out.target[kk] = tgtF;
      out.actual[kk] = gyroF;
      out.P[kk] = Pout;
      out.I[kk] = integ;
      out.D[kk] = Dout;
      out.FF[kk] = FFout;
      out.wTrue[kk] = w;
      out.theta[kk] = th;
      k++;
    }
    dq[static_cast<std::size_t>(di)] = u;
    di = (di + 1) % (nd + 1);
    const double ud = nd ? dq[static_cast<std::size_t>(di)] : u;
    m += (ud - m) * lagA;
    w = w * eA + (p.K * m + d) * gA;
    th += w * h;
  }
  return out;
}

}  // namespace aptuner::test
