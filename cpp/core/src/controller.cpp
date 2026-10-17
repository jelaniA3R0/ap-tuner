#include "aptuner/core/controller.hpp"

#include <algorithm>
#include <cmath>

namespace aptuner::core {

CVec lpf1(const Vec& freqs, double fs, double fc) {
  const std::size_t n = freqs.size();
  if (!(fc > 0)) return cx::one(n);
  const double dt = 1 / fs, rc = 1 / (2 * kPi * fc), alpha = dt / (dt + rc);
  // H(z) = alpha / (1 - (1-alpha) z^-1)
  const CVec z1 = cx::zinv(freqs, fs);
  const CVec den = cx::addReal(cx::scale(z1, -(1 - alpha)), 1);
  CVec num(n);
  for (auto& v : num.re) v = alpha;
  return cx::div(num, den);
}

CVec biquadLpf(const Vec& freqs, double fs, double fc) {
  const std::size_t n = freqs.size();
  if (!(fc > 0) || fc >= fs / 2) return cx::one(n);
  const double fr = fs / fc, ohm = std::tan(kPi / fr), c = 1 + 2 * std::cos(kPi / 4) * ohm + ohm * ohm;
  const double b0 = ohm * ohm / c, b1 = 2 * b0, b2 = b0, a1 = 2 * (ohm * ohm - 1) / c,
               a2 = (1 - 2 * std::cos(kPi / 4) * ohm + ohm * ohm) / c;
  return biquad(freqs, fs, b0, b1, b2, 1, a1, a2);
}

CVec biquad(const Vec& freqs, double fs, double b0, double b1, double b2, double a0, double a1, double a2) {
  const CVec z1 = cx::zinv(freqs, fs, 1), z2 = cx::zinv(freqs, fs, 2);
  const CVec num = cx::addReal(cx::add(cx::scale(z1, b1), cx::scale(z2, b2)), b0);
  const CVec den = cx::addReal(cx::add(cx::scale(z1, a1), cx::scale(z2, a2)), a0);
  return cx::div(num, den);
}

CVec notch(const Vec& freqs, double fs, double center, double bw, double attDb) {
  const std::size_t n = freqs.size();
  if (!(center > 0.5 * bw) || !(center < 0.5 * fs)) return cx::one(n);
  const double A = std::pow(10.0, -attDb / 40);
  const double octaves = std::log2(center / (center - bw / 2)) * 2;
  const double Q = std::sqrt(std::pow(2.0, octaves)) / (std::pow(2.0, octaves) - 1);
  const double omega = 2 * kPi * center / fs, alpha = std::sin(omega) / (2 * Q);
  const double b0 = 1 + alpha * A * A, b1 = -2 * std::cos(omega), b2 = 1 - alpha * A * A;
  const double a0 = 1 + alpha, a1 = b1, a2 = 1 - alpha;
  return biquad(freqs, fs, b0, b1, b2, a0, a1, a2);
}

CVec harmonicNotch(const Vec& freqs, double fs, const NotchDef& nd) {
  CVec H = cx::one(freqs.size());
  if (!nd.enable || !(nd.freq > 0)) return H;
  const int composite = (nd.opts & 1) ? 2 : (nd.opts & 16) ? 3 : 1;
  for (int h = 0; h < 8; h++) {
    if (!(nd.hmncs & (1 << h))) continue;
    const double mul = h + 1;
    double center = nd.freq * mul;
    const double bwh = nd.bw * mul;
    const double nyq = fs * 0.48, bwl = bwh * 0.52;
    const double spread = bwh / (32 * center);
    center = std::min(std::max(center, bwl), nyq);
    for (int c = 0; c < nd.chained; c++) {
      if (composite != 2 && center < nyq) H = cx::mul(H, notch(freqs, fs, center, bwh / composite, nd.att));
      if (composite > 1) {
        const double lo = center * (1 - spread), hi = center * (1 + spread);
        if (lo < nyq) H = cx::mul(H, notch(freqs, fs, lo, bwh / composite, nd.att));
        if (hi < nyq) H = cx::mul(H, notch(freqs, fs, hi, bwh / composite, nd.att));
      }
    }
  }
  return H;
}

double notchBaseFreq(const NotchParams& p, const NotchOp& op) {
  const int mode = p.mode;
  const double refOr1 = p.ref != 0 && !std::isnan(p.ref) ? p.ref : 1;   // JS: (p.ref || 1)
  if (mode == 0) return p.freq;
  if (mode == 1) {
    const double thr = std::max(0.0, op.throttle.has_value() ? *op.throttle : p.ref);
    return p.freq * std::max(p.fmRat.value_or(1.0), std::sqrt(thr / refOr1));
  }
  if (mode == 2 || mode == 5) return std::max(op.rpm / 60, p.freq) * refOr1;
  if (mode == 3) return std::max(op.escRpm / 60, p.freq) * refOr1;
  if (mode == 4) return std::max(op.fftHz != 0 ? op.fftHz : p.freq, p.freq) * refOr1;
  return p.freq;
}

CVec gyroChain(const Vec& freqs, const GyroChain& g) {
  CVec H = biquadLpf(freqs, g.backendHz, g.gyroFilterHz);
  for (const auto& n : g.notches) H = cx::mul(H, harmonicNotch(freqs, g.backendHz, n));
  return H;
}

PidResponse acPid(const Vec& freqs, double fs, const PidGains& g) {
  const std::size_t n = freqs.size();
  const double dt = 1 / fs;
  PidResponse r;
  r.T = lpf1(freqs, fs, g.FLTT);
  const CVec E = lpf1(freqs, fs, g.FLTE);
  const CVec Dlp = lpf1(freqs, fs, g.FLTD);
  const CVec z1 = cx::zinv(freqs, fs);
  const CVec oneLessZ1 = cx::addReal(cx::scale(z1, -1), 1);   // (1 - z^-1)
  // integrator: I*dt * 1/(1 - z^-1)   (AC_PID integrates the filtered error each loop)
  const CVec Iz = cx::div(cx::one(n), oneLessZ1);
  r.P = cx::scale(E, g.P);
  r.I = cx::scale(cx::mul(E, Iz), g.I * dt);
  r.D = cx::scale(cx::mul(cx::mul(E, oneLessZ1), Dlp), g.D / dt);
  r.Cfb = cx::add(cx::add(r.P, r.I), r.D);
  const CVec DFF = cx::scale(oneLessZ1, g.DFF / dt);
  r.Cff = cx::addReal(DFF, g.FF);
  return r;
}

}  // namespace aptuner::core
