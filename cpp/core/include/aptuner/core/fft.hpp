// Radix-2 complex FFT (in place, iterative) and helpers. Port of src/dsp/fft.js.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "aptuner/core/cx.hpp"

namespace aptuner::core {

class FFT {
 public:
  /** n must be a power of two (throws std::invalid_argument otherwise) */
  explicit FFT(std::size_t n);
  std::size_t size() const { return n_; }
  /** transform (re, im) of length n in place; inverse divides by n */
  void transform(double* re, double* im, bool inverse = false) const;
  void transform(Vec& re, Vec& im, bool inverse = false) const { transform(re.data(), im.data(), inverse); }

 private:
  std::size_t n_;
  std::vector<std::uint32_t> rev_;
  Vec cos_, sin_;
};

/** periodic Hann window: 0.5 - 0.5 cos(2 pi i / n) */
Vec hann(std::size_t n);

/** smallest power of two >= n (n may be fractional, as in the JavaScript version) */
std::size_t nextPow2(double n);

}  // namespace aptuner::core
