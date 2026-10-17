// Minimal self-registering test framework (no external dependencies).
//   TEST(name) { CHECK(cond); CHECK_NEAR(a, b, tol); CHECK_REL(a, b, relTol); }
// Every failed check prints file:line and the values; the runner returns the number of failed tests.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace aptuner::test {

struct Case {
  std::string name;
  std::function<void()> fn;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int n = 0;
  return n;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void fail(const char* file, int line, const std::string& what) {
  ++failures();
  std::fprintf(stderr, "  FAILED %s:%d: %s\n", file, line, what.c_str());
}

inline bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol || (std::isnan(a) && std::isnan(b)); }

inline bool relNear(double a, double b, double rel) {
  const double scale = std::fmax(std::fabs(a), std::fabs(b));
  return near(a, b, rel * (scale > 0 ? scale : 1.0));
}

}  // namespace aptuner::test

#define APT_CONCAT_(a, b) a##b
#define APT_CONCAT(a, b) APT_CONCAT_(a, b)
#define TEST(name)                                                                                   \
  static void name();                                                                                \
  static ::aptuner::test::Registrar APT_CONCAT(reg_, name)(#name, name);                             \
  static void name()

#define CHECK(cond)                                                                                  \
  do {                                                                                               \
    if (!(cond)) ::aptuner::test::fail(__FILE__, __LINE__, "CHECK(" #cond ")");                     \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                        \
  do {                                                                                               \
    const double va_ = (a), vb_ = (b);                                                               \
    if (!::aptuner::test::near(va_, vb_, (tol)))                                                     \
      ::aptuner::test::fail(__FILE__, __LINE__,                                                      \
                            std::string(#a " = ") + std::to_string(va_) + ", " #b " = " + std::to_string(vb_)); \
  } while (0)

#define CHECK_REL(a, b, rel)                                                                         \
  do {                                                                                               \
    const double va_ = (a), vb_ = (b);                                                               \
    if (!::aptuner::test::relNear(va_, vb_, (rel)))                                                  \
      ::aptuner::test::fail(__FILE__, __LINE__,                                                      \
                            std::string(#a " = ") + std::to_string(va_) + ", " #b " = " + std::to_string(vb_)); \
  } while (0)
