// Bare-airframe rate plant and its identification from a measured frequency response. Port of src/model/plant.js.
//   G(s) = K e^(-s Td) / ((s + a)(tau s + 1))
// a ~ 0 for multirotors (torque -> rate is nearly an integrator), a = -Lp for planes, tau = motor / servo lag,
// Td = transport delay. u = rate-PID output (copter: mixer -1..1; plane: surface degrees), y = rate (deg/s).
#pragma once

#include <functional>
#include <optional>
#include <string>

#include "aptuner/core/cx.hpp"

namespace aptuner::core {

struct Plant {
  double K = 0, a = 0, tau = 0, Td = 0;
};

CVec evalPlant(const Vec& freqs, const Plant& p);

struct NelderMeadResult {
  Vec x;
  double f = 0;
};
struct NelderMeadOptions {
  double step = 0.3;
  int maxIter = 600;
  double tol = 1e-7;
};
/** Nelder-Mead simplex minimiser (small problems), identical to the JavaScript version */
NelderMeadResult nelderMead(const std::function<double(const Vec&)>& f, const Vec& x0,
                            const NelderMeadOptions& opt = {});

struct PlantFit : Plant {
  double cost = 0;
  int nBins = 0;
  double fitBandLo = 0, fitBandHi = 0;
  double misfit = 0;       // weighted mean misfit (divided by the sum of the weights)
  double fitQuality = 0;   // 1 / (1 + misfit / 0.3): 1 at zero misfit, ~0.5 at 0.3
  double dcGain = 0;       // K / a
  double bandwidthHz = 0;  // min(a, 1/tau) / 2 pi
};

struct FitPlantOptions {
  std::optional<double> cohMin;   // default: 0.3 with >= 40 windows, else 0.5
  int nWin = 0;
  double fMin = 0.3, fMax = 40;
  std::string kind = "copter";    // "copter" | "plane": initial guesses only
  double fixedTdMax = 0.08;
};
/** fit K, a, tau, Td to H (coherence-weighted); nullopt when fewer than 6 usable bins */
std::optional<PlantFit> fitPlant(const Vec& freqs, const CVec& H, const Vec& coh, const FitPlantOptions& opt = {});

}  // namespace aptuner::core
