#include "aptuner/core/fft.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace aptuner::core {

FFT::FFT(std::size_t n) : n_(n), rev_(n), cos_(n / 2), sin_(n / 2) {
  if (n == 0 || (n & (n - 1)) != 0) throw std::invalid_argument("FFT size must be a power of two");
  std::size_t bits = 0;
  while ((std::size_t{1} << bits) < n) bits++;
  for (std::size_t i = 0; i < n; i++) {
    std::size_t r = 0, x = i;
    for (std::size_t b = 0; b < bits; b++) {
      r = (r << 1) | (x & 1);
      x >>= 1;
    }
    rev_[i] = static_cast<std::uint32_t>(r);
  }
  for (std::size_t i = 0; i < n / 2; i++) {
    const double a = -2 * kPi * static_cast<double>(i) / static_cast<double>(n);
    cos_[i] = std::cos(a);
    sin_[i] = std::sin(a);
  }
}

void FFT::transform(double* re, double* im, bool inverse) const {
  const std::size_t n = n_;
  for (std::size_t i = 0; i < n; i++) {
    const std::size_t j = rev_[i];
    if (j > i) {
      std::swap(re[i], re[j]);
      std::swap(im[i], im[j]);
    }
  }
  for (std::size_t size = 2; size <= n; size <<= 1) {
    const std::size_t half = size >> 1, step = n / size;
    for (std::size_t i = 0; i < n; i += size) {
      for (std::size_t j = 0, k = 0; j < half; j++, k += step) {
        const double c = cos_[k], s = inverse ? -sin_[k] : sin_[k];
        const std::size_t a = i + j, b = a + half;
        const double tr = re[b] * c - im[b] * s, ti = re[b] * s + im[b] * c;
        re[b] = re[a] - tr;
        im[b] = im[a] - ti;
        re[a] += tr;
        im[a] += ti;
      }
    }
  }
  if (inverse) {
    const double dn = static_cast<double>(n);
    for (std::size_t i = 0; i < n; i++) {
      re[i] /= dn;
      im[i] /= dn;
    }
  }
}

Vec hann(std::size_t n) {
  Vec w(n);
  for (std::size_t i = 0; i < n; i++) w[i] = 0.5 - 0.5 * std::cos(2 * kPi * static_cast<double>(i) / static_cast<double>(n));
  return w;
}

std::size_t nextPow2(double n) {
  std::size_t p = 1;
  while (static_cast<double>(p) < n) p <<= 1;
  return p;
}

}  // namespace aptuner::core
