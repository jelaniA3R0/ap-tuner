// Complex-array helpers (port of the C object in src/model/controller.js). Frequency responses are kept as
// separate real / imaginary arrays, one value per frequency of a grid.
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace aptuner::core {

using Vec = std::vector<double>;

inline constexpr double kPi = 3.14159265358979323846;

struct CVec {
  Vec re, im;
  CVec() = default;
  explicit CVec(std::size_t n) : re(n, 0.0), im(n, 0.0) {}
  std::size_t size() const { return re.size(); }
};

/** JavaScript Math.round: nearest integer, halves towards +infinity */
inline double jsRound(double x) {
  const double r = std::floor(x);
  return (x - r >= 0.5) ? r + 1.0 : r;
}

/** JavaScript Math.sign (NaN stays NaN) */
inline double jsSign(double x) { return x > 0 ? 1.0 : x < 0 ? -1.0 : x; }

namespace cx {

CVec of(std::size_t n);
CVec one(std::size_t n);
CVec mul(const CVec& a, const CVec& b);
CVec div(const CVec& a, const CVec& b);
CVec add(const CVec& a, const CVec& b);
CVec scale(const CVec& a, double k);
CVec addReal(const CVec& a, double k);
Vec abs(const CVec& a);
Vec phaseDeg(const CVec& a);
/** z^-k at every frequency for sample rate fs */
CVec zinv(const Vec& freqs, double fs, double k = 1);
/** pure delay e^(-j w Td) */
CVec delay(const Vec& freqs, double Td);

}  // namespace cx
}  // namespace aptuner::core
