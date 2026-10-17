#include "aptuner/core/jsnum.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace aptuner::core {

namespace {

/** shortest round-trip significant digits and decimal exponent: x = 0.d1d2...dk * 10^n */
void shortestDigits(double x, std::string& digits, int& n) {
  char buf[64];
#if defined(APTUNER_HAVE_FLOAT_TO_CHARS) || (defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L)
  auto res = std::to_chars(buf, buf + sizeof buf, x, std::chars_format::scientific);
  *res.ptr = 0;
#else
  for (int p = 1; p <= 17; p++) {
    std::snprintf(buf, sizeof buf, "%.*e", p - 1, x);
    if (std::strtod(buf, nullptr) == x) break;
  }
#endif
  // buf = "d.ddde+XX" or "de+XX"
  const char* e = std::strchr(buf, 'e');
  digits.clear();
  for (const char* c = buf; c < e; c++)
    if (*c >= '0' && *c <= '9') digits.push_back(*c);
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  n = std::atoi(e + 1) + 1;
}

/** exact decimal expansion check: is |x| exactly a tie at the digit after `digitsAfterPoint` in %f form? */
bool isFixedTie(double ax, int d) {
  char buf[512];
  std::snprintf(buf, sizeof buf, "%.*f", d + 60, ax);
  const char* dot = std::strchr(buf, '.');
  if (!dot) return false;
  const char* p = dot + 1 + d;
  if (*p != '5') return false;
  for (const char* q = p + 1; *q; q++)
    if (*q != '0') return false;
  return true;
}

/** increment a decimal string of digits (and an optional '.') by one unit in the last place */
std::string incrementDecimal(std::string s) {
  int i = static_cast<int>(s.size()) - 1;
  while (i >= 0) {
    if (s[i] == '.') {
      i--;
      continue;
    }
    if (s[i] == '9') {
      s[i] = '0';
      i--;
    } else {
      s[i]++;
      return s;
    }
  }
  return "1" + s;
}

}  // namespace

std::string jsToString(double x) {
  if (std::isnan(x)) return "NaN";
  if (std::isinf(x)) return x > 0 ? "Infinity" : "-Infinity";
  if (x == 0) return "0";
  std::string sign = x < 0 ? "-" : "";
  std::string d;
  int n = 0;
  shortestDigits(std::fabs(x), d, n);
  const int k = static_cast<int>(d.size());
  std::string out;
  if (k <= n && n <= 21) {
    out = d + std::string(static_cast<std::size_t>(n - k), '0');
  } else if (0 < n && n <= 21) {
    out = d.substr(0, static_cast<std::size_t>(n)) + "." + d.substr(static_cast<std::size_t>(n));
  } else if (-6 < n && n <= 0) {
    out = "0." + std::string(static_cast<std::size_t>(-n), '0') + d;
  } else {
    const int e = n - 1;
    out = d.substr(0, 1);
    if (k > 1) out += "." + d.substr(1);
    out += std::string("e") + (e >= 0 ? "+" : "-") + std::to_string(e >= 0 ? e : -e);
  }
  return sign + out;
}

std::string jsToFixed(double x, int digits) {
  if (std::isnan(x)) return "NaN";
  if (std::fabs(x) >= 1e21 || std::isinf(x)) return jsToString(x);
  if (x == 0) x = 0;   // -0 prints as 0
  const bool neg = x < 0;
  const double ax = std::fabs(x);
  char buf[512];
  std::string s;
  if (isFixedTie(ax, digits)) {
    // exact tie: the spec picks the larger n (round half away from zero)
    std::snprintf(buf, sizeof buf, "%.*f", digits + 1, ax);
    s = buf;
    s.pop_back();   // drop the '5'
    if (!s.empty() && s.back() == '.') s.pop_back();
    s = incrementDecimal(s);
  } else {
    std::snprintf(buf, sizeof buf, "%.*f", digits, ax);
    s = buf;
  }
  return (neg ? "-" : "") + s;
}

std::string jsToPrecision(double x, int p) {
  if (std::isnan(x) || std::isinf(x)) return jsToString(x);
  std::string sign;
  if (x < 0) {
    sign = "-";
    x = -x;
  }
  std::string m;   // p significant digits
  int e = 0;
  if (x == 0) {
    m = std::string(static_cast<std::size_t>(p), '0');
    e = 0;
  } else {
    char buf[512];
    std::snprintf(buf, sizeof buf, "%.*e", p - 1 + 60, x);
    // tie check on the exact expansion: digit p+1 is 5 and the rest zeros
    std::string all;
    const char* ep = std::strchr(buf, 'e');
    for (const char* c = buf; c < ep; c++)
      if (*c >= '0' && *c <= '9') all.push_back(*c);
    int e0 = std::atoi(ep + 1);
    bool tie = all.size() > static_cast<std::size_t>(p) && all[static_cast<std::size_t>(p)] == '5';
    for (std::size_t i = static_cast<std::size_t>(p) + 1; tie && i < all.size(); i++)
      if (all[i] != '0') tie = false;
    if (tie) {
      m = incrementDecimal(all.substr(0, static_cast<std::size_t>(p)));
      if (m.size() > static_cast<std::size_t>(p)) {   // 99.9 -> 100
        m.pop_back();
        e0++;
      }
      e = e0;
    } else {
      std::snprintf(buf, sizeof buf, "%.*e", p - 1, x);
      ep = std::strchr(buf, 'e');
      m.clear();
      for (const char* c = buf; c < ep; c++)
        if (*c >= '0' && *c <= '9') m.push_back(*c);
      e = std::atoi(ep + 1);
    }
  }
  std::string out;
  if (e < -6 || e >= p) {
    out = m.substr(0, 1);
    if (p > 1) out += "." + m.substr(1);
    out += std::string("e") + (e >= 0 ? "+" : "-") + std::to_string(e >= 0 ? e : -e);
  } else if (e >= 0) {
    out = m.substr(0, static_cast<std::size_t>(e) + 1);
    if (p > e + 1) out += "." + m.substr(static_cast<std::size_t>(e) + 1);
  } else {
    out = "0." + std::string(static_cast<std::size_t>(-(e + 1)), '0') + m;
  }
  return sign + out;
}

double jsRoundFixed(double x, int digits) { return std::strtod(jsToFixed(x, digits).c_str(), nullptr); }
double jsRoundPrecision(double x, int precision) {
  return std::strtod(jsToPrecision(x, precision).c_str(), nullptr);
}

}  // namespace aptuner::core
