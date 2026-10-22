#include "aptuner/core/parammeta.hpp"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <regex>
#include <utility>

#include "aptuner/core/jsnum.hpp"

namespace aptuner::core {

namespace {

struct Rule {
  bool isInt = false;
  std::optional<std::pair<double, double>> doc, hard;
  std::vector<double> values;   // empty: no enumeration
  int bits = -1;                // -1: not a bitmask
  bool reboot = false;
};

Rule R(std::optional<std::pair<double, double>> doc, std::optional<std::pair<double, double>> hard) {
  Rule r;
  r.doc = doc;
  r.hard = hard;
  return r;
}
Rule Rint(std::optional<std::pair<double, double>> doc, std::optional<std::pair<double, double>> hard) {
  Rule r = R(doc, hard);
  r.isInt = true;
  return r;
}
Rule Renum(std::vector<double> values, bool reboot = false) {
  Rule r;
  r.isInt = true;
  r.values = std::move(values);
  r.reboot = reboot;
  return r;
}
Rule Rbits(int bits, bool reboot) {
  Rule r;
  r.isInt = true;
  r.bits = bits;
  r.reboot = reboot;
  return r;
}
Rule Rreboot(Rule r) {
  r.reboot = true;
  return r;
}

using Range = std::pair<double, double>;
const std::string kRate = "(?:ATC_RAT|Q_A_RAT)";

/** [pattern, rule]; the first match wins (the table of src/model/parammeta.js) */
const std::vector<std::pair<std::regex, Rule>>& rules() {
  static const std::vector<std::pair<std::regex, Rule>> t = [] {
    std::vector<std::pair<std::string, Rule>> src{
        {"^" + kRate + "_(?:RLL|PIT)_P$", R(Range{0.01, 0.5}, Range{0, 5})},
        {"^" + kRate + "_(?:RLL|PIT)_I$", R(Range{0.01, 2}, Range{0, 10})},
        {"^" + kRate + "_(?:RLL|PIT)_D$", R(Range{0, 0.05}, Range{0, 1})},
        {"^" + kRate + "_YAW_P$", R(Range{0.1, 2.5}, Range{0, 5})},
        {"^" + kRate + "_YAW_I$", R(Range{0.01, 1}, Range{0, 10})},
        {"^" + kRate + "_YAW_D$", R(Range{0, 0.02}, Range{0, 1})},
        {"^" + kRate + "_(?:RLL|PIT|YAW)_FF$", R(Range{0, 0.5}, Range{0, 5})},
        {"^" + kRate + "_(?:RLL|PIT|YAW)_IMAX$", R(Range{0, 1}, Range{0, 1})},
        {"^" + kRate + "_(?:RLL|PIT|YAW)_FLT[TED]$", R(Range{0, 100}, Range{0, 1000})},   // [AP-assumed] range
        {"^" + kRate + "_(?:RLL|PIT|YAW)_SMAX$", R(Range{0, 200}, Range{0, 1000})},
        {"^" + kRate + "_(?:RLL|PIT|YAW)_D_FF$", R(Range{0, 0.02}, Range{0, 1})},
        {"^(?:ATC_ANG|Q_A_ANG)_(?:RLL|PIT|YAW)_P$", R(Range{3, 12}, Range{0.1, 30})},
        // the limits the model tuner proposes for heavy / slow vehicles (ArduPilot 4.7 deg/s/s names; older centi-degree names)
        {"^(?:ATC|Q_A)_ACC_(?:R|P)_MAX$", R(Range{0, 1800}, Range{0, 10000})},
        {"^(?:ATC|Q_A)_ACC_Y_MAX$", R(Range{0, 720}, Range{0, 10000})},
        {"^(?:ATC|Q_A)_ACCEL_(?:R|P|Y)_MAX$", R(Range{0, 180000}, Range{0, 1000000})},
        {"^(?:ATC|Q_A)_RATE_(?:R|P|Y)_MAX$", R(Range{0, 1080}, Range{0, 2000})},
        {"^(?:ATC|Q_A)_RATE_WPY_MAX$", R(Range{0, 1080}, Range{0, 2000})},
        {"^(?:ATC|Q_A)_INPUT_TC$", R(Range{0, 1}, Range{0, 5})},
        {"^(?:Q_PLT_Y_RATE|PILOT_Y_RATE)$", R(Range{1, 500}, Range{0, 2000})},
        {"^(?:RLL|PTCH)_RATE_P$", R(Range{0.08, 0.35}, Range{0, 5})},
        {"^(?:RLL|PTCH)_RATE_I$", R(Range{0.01, 0.6}, Range{0, 5})},
        {"^(?:RLL|PTCH)_RATE_D$", R(Range{0.001, 0.03}, Range{0, 1})},
        {"^(?:RLL|PTCH)_RATE_FF$", R(Range{0.05, 2}, Range{0, 5})},
        {"^YAW_RATE_(?:P|I|FF)$", R(std::nullopt, Range{0, 5})},   // [AP-assumed] no documented range used
        {"^YAW_RATE_D$", R(std::nullopt, Range{0, 1})},
        {"^(?:RLL|PTCH|YAW)_RATE_IMAX$", R(Range{0, 1}, Range{0, 1})},
        {"^(?:RLL|PTCH|YAW)_RATE_FLT[TED]$", R(Range{0, 100}, Range{0, 1000})},   // [AP-assumed] range
        {"^(?:RLL|PTCH|YAW)_RATE_SMAX$", R(Range{0, 200}, Range{0, 1000})},
        {"^(?:RLL2SRV|PTCH2SRV)_TCONST$", R(Range{0.4, 1}, Range{0.05, 5})},
        {"^INS_GYRO_FILTER$", Rint(Range{0, 256}, Range{0, 1000})},
        {"^INS_HNTC[H2]_ENABLE$", Renum({0, 1})},
        {"^INS_HNTC[H2]_MODE$", Renum({0, 1, 2, 3, 4, 5})},
        {"^INS_HNTC[H2]_FREQ$", R(Range{10, 495}, Range{0, 2000})},
        {"^INS_HNTC[H2]_BW$", R(Range{5, 250}, Range{0, 1000})},
        {"^INS_HNTC[H2]_ATT$", R(Range{5, 50}, Range{0, 100})},
        {"^INS_HNTC[H2]_REF$", Rreboot(R(Range{0, 1}, Range{0, 10}))},
        {"^INS_HNTC[H2]_HMNCS$", Rbits(15, true)},   // [AP-assumed] up to 16 harmonics
        {"^INS_HNTC[H2]_OPTS$", Rbits(5, true)},     // [AP-assumed] bits 0-5 (4 = triple notch)
        {"^INS_HNTC[H2]_FM_RAT$", R(Range{0.1, 1}, Range{0, 2})},
        {"^FFT_ENABLE$", Renum({0, 1}, true)},
        {"^GCS_PID_MASK$", Rbits(4, false)},
    };
    std::vector<std::pair<std::regex, Rule>> out;
    for (auto& [re, rule] : src) out.emplace_back(std::regex(re, std::regex::ECMAScript), rule);
    return out;
  }();
  return t;
}

const Rule* paramRule(const std::string& name) {
  for (const auto& [re, rule] : rules())
    if (std::regex_search(name, re)) return &rule;
  return nullptr;
}

std::string num(double v) { return jsToString(v); }

/** decimal exponent of |x| > 0 by exact steps (the same in both editions; log10 may differ in the last bit) */
int decExp(double a) {
  int e = 0;
  while (a >= 10) {
    a /= 10;
    e++;
  }
  while (a < 1) {
    a *= 10;
    e--;
  }
  return e;
}
std::string stripZeros(std::string s) {
  if (s.find('.') == std::string::npos) return s;
  while (!s.empty() && s.back() == '0') s.pop_back();
  if (!s.empty() && s.back() == '.') s.pop_back();
  return s;
}
double f32(double v) { return static_cast<double>(static_cast<float>(v)); }

}  // namespace

Json ParamCheck::toJson() const {
  Json e = Json::array(), w = Json::array();
  for (const auto& s : errors) e.push(s);
  for (const auto& s : warnings) w.push(s);
  return Json::obj({{"ok", ok}, {"errors", e}, {"warnings", w}, {"reboot", reboot}, {"int", isInt}});
}

bool isRebootRequired(const std::string& name) {
  const Rule* r = paramRule(name);
  return r && r->reboot;
}

ParamCheck validateParam(const std::string& name, const Json& value) {
  const Rule* rule = paramRule(name);
  ParamCheck c;
  if (!value.isNumber() || !std::isfinite(value.num())) {
    c.errors.push_back("is not a number");
  } else if (!rule) {
    c.errors.push_back("is not a parameter AP Tuner can check");
  } else {
    const double v = value.num();
    const bool whole = std::floor(v) == v;
    if (rule->isInt && !whole) c.errors.push_back("must be a whole number");
    if (!rule->values.empty()) {
      bool in = false;
      std::string list;
      for (std::size_t i = 0; i < rule->values.size(); i++) {
        if (rule->values[i] == v) in = true;
        list += (i ? ", " : "") + num(rule->values[i]);
      }
      if (!in) c.errors.push_back("must be one of " + list);
    }
    if (rule->bits >= 0 && whole && (v < 0 || v >= std::pow(2.0, rule->bits + 1)))
      c.errors.push_back("sets bits ArduPilot does not define (bits 0 to " + std::to_string(rule->bits) + " only)");
    if (rule->hard && (v < rule->hard->first || v > rule->hard->second))
      c.errors.push_back("is outside " + num(rule->hard->first) + " to " + num(rule->hard->second));
    if (c.errors.empty() && rule->doc && (v < rule->doc->first || v > rule->doc->second))
      c.warnings.push_back("is outside ArduPilot's documented range " + num(rule->doc->first) + " to " + num(rule->doc->second));
  }
  c.ok = c.errors.empty();
  c.reboot = rule && rule->reboot;
  c.isInt = rule && rule->isInt;
  return c;
}

std::string formatParamValue(const std::string& name, double value) {
  const Rule* rule = paramRule(name);
  if (rule && rule->isInt && std::isfinite(value) && std::floor(value) == value) return jsToFixed(value, 0);
  const double f = f32(value);
  if (!std::isfinite(f)) return jsToString(f);   // never in a checked file (refused); an UNDO file keeps what it read
  if (f == 0) return "0";
  const int e = decExp(std::fabs(f));
  for (int p = 1; p <= 9; p++) {
    const std::string s = jsToFixed(f, std::max(0, std::min(20, p - 1 - e)));
    if (f32(std::strtod(s.c_str(), nullptr)) == f) return stripZeros(s);
  }
  return stripZeros(jsToFixed(f, std::max(0, std::min(20, 9 - e))));
}

}  // namespace aptuner::core
