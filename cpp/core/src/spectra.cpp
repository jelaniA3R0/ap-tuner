#include "aptuner/core/spectra.hpp"

#include <algorithm>
#include <cmath>

#include "aptuner/core/fft.hpp"

namespace aptuner::core {

FreqResponse freqResponse(const double* x, const double* y, std::size_t N, double fs, std::size_t windowSize,
                          double overlap, double minRms, const Blocks* blocks) {
  const Vec w = hann(windowSize);
  const FFT fft(windowSize);
  const std::size_t nb = windowSize / 2 + 1;
  Vec Sxx(nb, 0.0), Syy(nb, 0.0), Sxyr(nb, 0.0), Sxyi(nb, 0.0);
  const std::size_t step =
      static_cast<std::size_t>(std::max(1.0, jsRound(static_cast<double>(windowSize) * (1 - overlap))));
  int nWin = 0;
  Vec xr(windowSize), xi(windowSize), yr(windowSize), yi(windowSize);
  const double ws = static_cast<double>(windowSize);
  const Blocks all{{0, N}};
  for (const auto& [b0, b1] : blocks ? *blocks : all)
  for (std::size_t s = b0; s + windowSize <= std::min(b1, N); s += step) {
    // detrend (mean removal) + window
    double mx = 0, my = 0;
    for (std::size_t i = 0; i < windowSize; i++) {
      mx += x[s + i];
      my += y[s + i];
    }
    mx /= ws;
    my /= ws;
    double rms = 0;
    for (std::size_t i = 0; i < windowSize; i++) {
      const double xv = (x[s + i] - mx);
      rms += xv * xv;
      xr[i] = xv * w[i];
      xi[i] = 0;
      yr[i] = (y[s + i] - my) * w[i];
      yi[i] = 0;
    }
    rms = std::sqrt(rms / ws);
    if (rms < minRms) continue;
    fft.transform(xr, xi);
    fft.transform(yr, yi);
    for (std::size_t k = 0; k < nb; k++) {
      Sxx[k] += xr[k] * xr[k] + xi[k] * xi[k];
      Syy[k] += yr[k] * yr[k] + yi[k] * yi[k];
      // Sxy = conj(X) * Y
      Sxyr[k] += xr[k] * yr[k] + xi[k] * yi[k];
      Sxyi[k] += xr[k] * yi[k] - xi[k] * yr[k];
    }
    nWin++;
  }
  FreqResponse r;
  r.f.resize(nb);
  for (std::size_t k = 0; k < nb; k++) r.f[k] = static_cast<double>(k) * fs / ws;
  r.H = CVec(nb);
  r.coh.assign(nb, 0.0);
  for (std::size_t k = 0; k < nb; k++) {
    if (Sxx[k] > 0) {
      r.H.re[k] = Sxyr[k] / Sxx[k];
      r.H.im[k] = Sxyi[k] / Sxx[k];
    }
    const double den = Sxx[k] * Syy[k];
    r.coh[k] = den > 0 ? (Sxyr[k] * Sxyr[k] + Sxyi[k] * Sxyi[k]) / den : 0;
  }
  // scale PSDs to amplitude^2/Hz (Welch, Hann window power correction)
  double wp = 0;
  for (std::size_t i = 0; i < windowSize; i++) wp += w[i] * w[i];
  const double scale = nWin > 0 ? 1 / (nWin * wp * fs) : 0;
  for (std::size_t k = 0; k < nb; k++) {
    Sxx[k] *= scale * 2;
    Syy[k] *= scale * 2;
  }
  r.Sxx = std::move(Sxx);
  r.Syy = std::move(Syy);
  r.nWin = nWin;
  r.windowSize = windowSize;
  r.fs = fs;
  return r;
}

Psd psd(const double* x, std::size_t n, double fs, std::size_t windowSize, double overlap, const Blocks* blocks) {
  const Vec w = hann(windowSize);
  const FFT fft(windowSize);
  const std::size_t nb = windowSize / 2 + 1;
  Vec S(nb, 0.0);
  const std::size_t step =
      static_cast<std::size_t>(std::max(1.0, jsRound(static_cast<double>(windowSize) * (1 - overlap))));
  int nWin = 0;
  Vec xr(windowSize), xi(windowSize);
  const double wsz = static_cast<double>(windowSize);
  const Blocks all{{0, n}};
  for (const auto& [b0, b1] : blocks ? *blocks : all)
  for (std::size_t s = b0; s + windowSize <= std::min(b1, n); s += step) {
    double m = 0;
    for (std::size_t i = 0; i < windowSize; i++) m += x[s + i];
    m /= wsz;
    for (std::size_t i = 0; i < windowSize; i++) {
      xr[i] = (x[s + i] - m) * w[i];
      xi[i] = 0;
    }
    fft.transform(xr, xi);
    for (std::size_t k = 0; k < nb; k++) S[k] += xr[k] * xr[k] + xi[k] * xi[k];
    nWin++;
  }
  double wp = 0, ws = 0;
  for (std::size_t i = 0; i < windowSize; i++) {
    wp += w[i] * w[i];
    ws += w[i];
  }
  Psd r;
  r.f.resize(nb);
  r.amp.resize(nb);
  for (std::size_t k = 0; k < nb; k++) {
    r.f[k] = static_cast<double>(k) * fs / wsz;
    const double mean = nWin ? S[k] / nWin : 0;
    // linear amplitude (peak) of a sinusoid at that bin
    r.amp[k] = std::sqrt(mean) * (k == 0 || k == nb - 1 ? 1 : std::sqrt(2.0)) / ws;
    S[k] = nWin ? 2 * mean / (wp * fs) : 0;
  }
  r.psd = std::move(S);
  r.nWin = nWin;
  r.windowSize = windowSize;
  r.fs = fs;
  return r;
}

std::size_t countWindows(const Blocks& blocks, std::size_t windowSize, double overlap) {
  const std::size_t step =
      static_cast<std::size_t>(std::max(1.0, jsRound(static_cast<double>(windowSize) * (1 - overlap))));
  std::size_t k = 0;
  for (const auto& [b0, b1] : blocks)
    if (b1 - b0 >= windowSize) k += (b1 - b0 - windowSize) / step + 1;
  return k;
}

double bandRms(const Psd& sp, double f1, double f2) {
  double e = 0;
  const double df = sp.f.size() > 1 ? sp.f[1] - sp.f[0] : 0;
  for (std::size_t k = 0; k < sp.f.size(); k++)
    if (sp.f[k] >= f1 && sp.f[k] <= f2) e += sp.psd[k] * df;
  return std::sqrt(e);
}

}  // namespace aptuner::core
