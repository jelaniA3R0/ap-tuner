#include "aptuner/core/cx.hpp"

namespace aptuner::core::cx {

CVec of(std::size_t n) { return CVec(n); }

CVec one(std::size_t n) {
  CVec o(n);
  for (auto& v : o.re) v = 1.0;
  return o;
}

CVec mul(const CVec& a, const CVec& b) {
  const std::size_t n = a.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    o.re[i] = a.re[i] * b.re[i] - a.im[i] * b.im[i];
    o.im[i] = a.re[i] * b.im[i] + a.im[i] * b.re[i];
  }
  return o;
}

CVec div(const CVec& a, const CVec& b) {
  const std::size_t n = a.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    double d = b.re[i] * b.re[i] + b.im[i] * b.im[i];
    if (d == 0 || std::isnan(d)) d = 1e-300;   // JS: (...) || 1e-300
    o.re[i] = (a.re[i] * b.re[i] + a.im[i] * b.im[i]) / d;
    o.im[i] = (a.im[i] * b.re[i] - a.re[i] * b.im[i]) / d;
  }
  return o;
}

CVec add(const CVec& a, const CVec& b) {
  const std::size_t n = a.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    o.re[i] = a.re[i] + b.re[i];
    o.im[i] = a.im[i] + b.im[i];
  }
  return o;
}

CVec scale(const CVec& a, double k) {
  const std::size_t n = a.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    o.re[i] = a.re[i] * k;
    o.im[i] = a.im[i] * k;
  }
  return o;
}

CVec addReal(const CVec& a, double k) {
  const std::size_t n = a.size();
  CVec o(n);
  for (std::size_t i = 0; i < n; i++) {
    o.re[i] = a.re[i] + k;
    o.im[i] = a.im[i];
  }
  return o;
}

Vec abs(const CVec& a) {
  Vec o(a.size());
  for (std::size_t i = 0; i < a.size(); i++) o[i] = std::hypot(a.re[i], a.im[i]);
  return o;
}

Vec phaseDeg(const CVec& a) {
  Vec o(a.size());
  for (std::size_t i = 0; i < a.size(); i++) o[i] = std::atan2(a.im[i], a.re[i]) * 180 / kPi;
  return o;
}

CVec zinv(const Vec& freqs, double fs, double k) {
  CVec o(freqs.size());
  for (std::size_t i = 0; i < freqs.size(); i++) {
    const double th = -2 * kPi * freqs[i] / fs * k;
    o.re[i] = std::cos(th);
    o.im[i] = std::sin(th);
  }
  return o;
}

CVec delay(const Vec& freqs, double Td) {
  CVec o(freqs.size());
  for (std::size_t i = 0; i < freqs.size(); i++) {
    const double th = -2 * kPi * freqs[i] * Td;
    o.re[i] = std::cos(th);
    o.im[i] = std::sin(th);
  }
  return o;
}

}  // namespace aptuner::core::cx
