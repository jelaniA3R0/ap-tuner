#include "aptuner/core/stepresp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aptuner/core/fft.hpp"

namespace aptuner::core {

StepResponse stepResponse(const double* target, const double* actual, std::size_t N, double fs,
                          const StepResponseOptions& opt) {
  StepResponse out;
  const std::size_t windowSize = opt.windowSize;
  if (N < windowSize) return out;
  const FFT fft(windowSize);
  const Vec w = hann(windowSize);
  const std::size_t nb = windowSize;
  const std::size_t stepLen =
      std::min(static_cast<std::size_t>(std::ceil(opt.durationS * fs)), windowSize);
  out.t.resize(stepLen);
  for (std::size_t i = 0; i < stepLen; i++) out.t[i] = static_cast<double>(i) / fs;
  // noise regularisation: smoothed unit step as in PIDReview (gaussian with 25 Hz cutoff)
  const double cutHz = 25;
  long long lenLpf = std::max<long long>(2, static_cast<long long>(std::floor(cutHz * static_cast<double>(windowSize) / fs)));
  lenLpf += lenLpf - 2;
  const double radius = std::ceil(static_cast<double>(lenLpf) * 0.5), sigma = static_cast<double>(lenLpf) / 6;
  const std::size_t half = windowSize / 2 + 1;
  Vec sn(half, 1.0);
  double last = 0;
  const std::size_t m = std::min<std::size_t>(static_cast<std::size_t>(lenLpf), half);
  for (std::size_t j = 0; j < m; j++) {
    const double d = static_cast<double>(j) - radius;
    sn[j] = last + std::exp((-0.5 / (sigma * sigma)) * d * d);
    last = sn[j];
  }
  for (std::size_t j = 0; j < m; j++) sn[j] /= last;
  Vec full(windowSize, 0.0);
  for (std::size_t j = 0; j < half; j++) full[j] = sn[j];
  for (std::size_t j = 1; j + 1 < half; j++) full[windowSize - j] = sn[j];
  for (std::size_t j = 0; j < windowSize; j++) full[j] = 1 / ((1 - full[j] + 1e-9) * 10);
  const std::size_t step = static_cast<std::size_t>(
      std::max(1.0, jsRound(static_cast<double>(windowSize) / static_cast<double>(opt.overlapDiv))));
  Vec mean(stepLen, 0.0);
  int count = 0;
  Vec xr(nb), xi(nb), yr(nb), yi(nb), hr(nb), hi(nb);
  const Blocks all{{0, N}};
  for (const auto& [b0, b1] : opt.blocks ? *opt.blocks : all)
  for (std::size_t s = b0; s + windowSize <= std::min(b1, N) && count < opt.maxWindows; s += step) {
    double peak = 0;
    for (std::size_t i = 0; i < windowSize; i++) {
      const double a = std::fabs(target[s + i]);
      if (a > peak) peak = a;
    }
    if (peak < opt.minInput) continue;
    for (std::size_t i = 0; i < windowSize; i++) {
      xr[i] = target[s + i] * w[i];
      xi[i] = 0;
      yr[i] = actual[s + i] * w[i];
      yi[i] = 0;
    }
    fft.transform(xr, xi);
    fft.transform(yr, yi);
    for (std::size_t k = 0; k < nb; k++) {
      // Pyx = Y * conj(X); Pxx = X conj(X) + sn
      const double pr = yr[k] * xr[k] + yi[k] * xi[k], pi = yi[k] * xr[k] - yr[k] * xi[k];
      const double pxx = xr[k] * xr[k] + xi[k] * xi[k] + full[k];
      hr[k] = pr / pxx;
      hi[k] = pi / pxx;
    }
    fft.transform(hr, hi, true);
    StepWindow win;
    win.t0 = static_cast<double>(s) / fs;
    win.resp.resize(stepLen);
    double acc = 0;
    for (std::size_t l = 0; l < stepLen; l++) {
      acc += hr[l];
      win.resp[l] = acc;
    }
    for (std::size_t l = 0; l < stepLen; l++) mean[l] += win.resp[l];
    out.windows.push_back(std::move(win));
    count++;
  }
  if (count)
    for (std::size_t l = 0; l < stepLen; l++) mean[l] /= count;
  out.mean = std::move(mean);
  out.count = count;
  return out;
}

std::optional<StepMetrics> stepMetrics(const Vec& t, const Vec& y) {
  if (y.empty()) return std::nullopt;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::size_t n = y.size();
  const std::size_t k0 = static_cast<std::size_t>(std::floor(static_cast<double>(n) * 0.7));
  auto mean = [&](std::size_t a, std::size_t b) {
    double s = 0;
    for (std::size_t i = a; i < b; i++) s += y[i];
    return s / static_cast<double>(b - a);
  };
  const std::size_t tl = n - k0;
  const double finalV = mean(k0, n);
  double peak = -std::numeric_limits<double>::infinity();
  for (double v : y) peak = v > peak || std::isnan(v) ? v : peak;
  const std::size_t third = std::max<std::size_t>(1, tl / 3);
  const double drift = std::fabs(mean(n - third, n) - mean(k0, k0 + third));
  double spread = 0;
  for (std::size_t i = k0; i < n; i++) spread = std::max(spread, std::fabs(y[i] - finalV));
  const bool settled = std::isfinite(finalV) && std::fabs(finalV) > 0.05 && drift <= 0.02 * std::fabs(finalV) &&
                       spread <= 0.05 * std::fabs(finalV);
  StepMetrics m;
  m.peak = peak;
  if (!settled) {
    m.finalValue = std::isfinite(finalV) ? finalV : nan;
    m.overshootPct = nan;
    m.riseTime = nan;
    m.settlingTime = nan;
    m.oscillations = 0;
    m.settled = false;
    return m;
  }
  m.finalValue = finalV;
  m.overshootPct = finalV > 0 ? std::max(0.0, (peak - finalV) / finalV * 100) : 0;
  auto timeAt = [&](double frac) {
    for (std::size_t i = 0; i < n; i++)
      if (y[i] >= frac * finalV) return i < t.size() ? t[i] : nan;
    return nan;
  };
  const double t10 = timeAt(0.1), t90 = timeAt(0.9);
  m.riseTime = std::isfinite(t90 - t10) ? t90 - t10 : nan;
  m.settlingTime = nan;
  for (std::size_t k = n; k-- > 0;) {
    if (std::fabs(y[k] - finalV) > 0.05 * std::fabs(finalV)) {
      m.settlingTime = t[std::min(k + 1, n - 1)];
      break;
    }
  }
  // oscillation count: sign changes of (y - final) after the peak
  std::size_t ip = n;
  for (std::size_t i = 0; i < n; i++)
    if (y[i] == peak) {
      ip = i;
      break;
    }
  int osc = 0;
  if (ip < n)
    for (std::size_t i = ip + 1; i < n; i++)
      if (jsSign(y[i] - finalV) != jsSign(y[i - 1] - finalV) && std::fabs(y[i] - finalV) > 0.02) osc++;
  m.oscillations = osc;
  return m;
}

}  // namespace aptuner::core
