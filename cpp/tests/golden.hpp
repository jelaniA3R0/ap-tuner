// Helpers for golden tests: load the JavaScript reference results (tests/golden/*.json, written by
// tools/make-golden.js) and compare arrays with a relative tolerance.
#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "aptuner/core/cx.hpp"
#include "aptuner/core/json.hpp"
#include "check.hpp"

#ifndef APTUNER_GOLDEN_DIR
#error "APTUNER_GOLDEN_DIR must point at tests/golden"
#endif

namespace aptuner::test {

inline core::Json loadGolden(const std::string& name) {
  const std::string path = std::string(APTUNER_GOLDEN_DIR) + "/" + name + ".json";
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    fail(__FILE__, __LINE__, "cannot open " + path);
    return {};
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return core::Json::parse(ss.str());
}

/** element-wise |got - exp| <= rel * max(|exp|, scale) where scale = absFloor * max|exp| over the array */
inline bool vecNear(const char* file, int line, const std::string& what, const core::Vec& got, const core::Vec& exp,
                    double rel, double absFloor = 1e-9) {
  if (got.size() != exp.size()) {
    fail(file, line, what + ": size " + std::to_string(got.size()) + " vs " + std::to_string(exp.size()));
    return false;
  }
  double maxAbs = 0;
  for (double v : exp)
    if (std::isfinite(v)) maxAbs = std::fmax(maxAbs, std::fabs(v));
  const double floorAbs = absFloor * (maxAbs > 0 ? maxAbs : 1);
  double worst = 0;
  std::size_t worstI = 0;
  for (std::size_t i = 0; i < exp.size(); i++) {
    const double a = got[i], b = exp[i];
    if (std::isnan(b) && std::isnan(a)) continue;
    const double err = std::fabs(a - b) / std::fmax(std::fabs(b), floorAbs / rel);
    if (!(err <= worst)) {
      worst = err;
      worstI = i;
    }
  }
  if (!(worst <= rel)) {
    char buf[256];
    std::snprintf(buf, sizeof buf, "%s: element %zu got %.17g expected %.17g (rel err %.3g > %.3g)", what.c_str(),
                  worstI, got[worstI], exp[worstI], worst, rel);
    fail(file, line, buf);
    return false;
  }
  return true;
}

inline core::CVec cvecOf(const core::Json& j) {
  core::CVec c;
  c.re = j["re"].toVec();
  c.im = j["im"].toVec();
  return c;
}

}  // namespace aptuner::test

#define CHECK_VEC(got, exp, rel) ::aptuner::test::vecNear(__FILE__, __LINE__, #got, (got), (exp), (rel))
#define CHECK_VEC_ABS(got, exp, rel, absFloor) \
  ::aptuner::test::vecNear(__FILE__, __LINE__, #got, (got), (exp), (rel), (absFloor))
#define CHECK_CVEC(got, exp, rel)                                                         \
  do {                                                                                    \
    const auto& g_ = (got);                                                               \
    const auto e_ = ::aptuner::test::cvecOf(exp);                                         \
    ::aptuner::test::vecNear(__FILE__, __LINE__, std::string(#got) + ".re", g_.re, e_.re, (rel)); \
    ::aptuner::test::vecNear(__FILE__, __LINE__, std::string(#got) + ".im", g_.im, e_.im, (rel)); \
  } while (0)
