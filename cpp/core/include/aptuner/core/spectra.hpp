// Welch spectra, cross-spectra, coherence and frequency-response estimation. Port of src/dsp/spectra.js
// (the method of ArduPilot's WebTools: windowed, overlapped FFT windows averaged into Sxx, Syy, Sxy;
// H = Sxy / Sxx; coherence = |Sxy|^2 / (Sxx Syy)).
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "aptuner/core/cx.hpp"

namespace aptuner::core {

/** unbroken stretches [i0, i1) of a sample array: Welch windows never straddle a block edge (gaps, ground time) */
using Blocks = std::vector<std::pair<std::size_t, std::size_t>>;

struct FreqResponse {
  Vec f;
  CVec H;   // y / x
  Vec coh;  // 0..1
  Vec Sxx, Syy;
  int nWin = 0;
  std::size_t windowSize = 0;
  double fs = 0;
};

/** x -> y frequency response; windows whose input RMS is below minRms are skipped; blocks: null = the whole array */
FreqResponse freqResponse(const double* x, const double* y, std::size_t n, double fs, std::size_t windowSize = 512,
                          double overlap = 0.5, double minRms = 0, const Blocks* blocks = nullptr);
inline FreqResponse freqResponse(const Vec& x, const Vec& y, double fs, std::size_t windowSize = 512,
                                 double overlap = 0.5, double minRms = 0, const Blocks* blocks = nullptr) {
  return freqResponse(x.data(), y.data(), x.size() < y.size() ? x.size() : y.size(), fs, windowSize, overlap, minRms, blocks);
}

struct Psd {
  Vec f;
  Vec psd;  // amplitude^2 / Hz
  Vec amp;  // linear peak amplitude of a sinusoid at that bin
  int nWin = 0;
  std::size_t windowSize = 0;
  double fs = 0;
};

Psd psd(const double* x, std::size_t n, double fs, std::size_t windowSize = 1024, double overlap = 0.5,
        const Blocks* blocks = nullptr);
inline Psd psd(const Vec& x, double fs, std::size_t windowSize = 1024, double overlap = 0.5, const Blocks* blocks = nullptr) {
  return psd(x.data(), x.size(), fs, windowSize, overlap, blocks);
}

/** number of Welch windows of windowSize (overlap fraction) that fit inside the blocks */
std::size_t countWindows(const Blocks& blocks, std::size_t windowSize, double overlap = 0.5);

/** integrate the PSD between f1 and f2 -> RMS */
double bandRms(const Psd& sp, double f1, double f2);

}  // namespace aptuner::core
