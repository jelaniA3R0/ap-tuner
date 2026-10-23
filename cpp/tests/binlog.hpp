// Builds small DataFlash (.bin) logs in memory for the tests: FMT definitions, records, and a ready-made hover
// flight (RATE at a set rate, throttle, mode, parameters) with optional gaps and bad samples. Port of
// test/binlog.js: the same bytes for the same options.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aptuner::test {

/** Math.PI */
inline constexpr double kPiT = 3.141592653589793;

/** a record value: a number or a string (n / N / Z fields) */
struct V {
  double num = 0;
  std::string s;
  bool isStr = false;
  V(double d) : num(d) {}
  V(int d) : num(d) {}
  V(const char* c) : s(c), isStr(true) {}
  V(std::string c) : s(std::move(c)), isStr(true) {}
};

inline int fmtSize(char ch) {
  switch (ch) {
    case 'b': case 'B': case 'M': return 1;
    case 'h': case 'H': case 'g': case 'c': case 'C': return 2;
    case 'i': case 'I': case 'e': case 'E': case 'L': case 'f': case 'n': return 4;
    case 'N': return 16;
    case 'Z': case 'a': return 64;
    case 'd': case 'q': case 'Q': return 8;
    default: throw std::runtime_error(std::string("format char ") + ch + " not supported by the test builder");
  }
}

class LogBuilder {
 public:
  struct Def {
    int type = 0;
    std::string format;
    int size = 0;
  };
  /** define a message type: name (<= 4 chars), format characters, comma-separated labels; len < 0: from the format */
  LogBuilder& fmt(int type, const std::string& name, const std::string& format, const std::string& labels, int len = -1) {
    int size = 3;
    for (char ch : format) size += ch == 'x' ? 0 : fmtSize(ch);
    if (len >= 0) size = len;
    std::uint8_t b[89] = {};
    b[0] = 0xa3;
    b[1] = 0x95;
    b[2] = 128;
    b[3] = static_cast<std::uint8_t>(type);
    b[4] = static_cast<std::uint8_t>(size);
    std::memcpy(b + 5, name.data(), std::min<std::size_t>(4, name.size()));
    std::memcpy(b + 9, format.data(), std::min<std::size_t>(16, format.size()));
    std::memcpy(b + 25, labels.data(), std::min<std::size_t>(64, labels.size()));
    bytes.insert(bytes.end(), b, b + 89);
    defs[name] = {type, format, size};
    return *this;
  }
  /** append a record: values in field order (TimeUS first, in microseconds) */
  LogBuilder& rec(const std::string& name, const std::vector<V>& values) {
    const Def& d = defs.at(name);
    std::vector<std::uint8_t> b(static_cast<std::size_t>(d.size), 0);
    b[0] = 0xa3;
    b[1] = 0x95;
    b[2] = static_cast<std::uint8_t>(d.type);
    std::size_t off = 3;
    for (std::size_t k = 0; k < d.format.size(); k++) {
      const char ch = d.format[k];
      const V& v = values.at(k);
      const auto put = [&](const void* p, std::size_t n) { std::memcpy(b.data() + off, p, n); };
      switch (ch) {
        case 'Q': { const std::uint64_t x = static_cast<std::uint64_t>(jsRound(v.num)); put(&x, 8); break; }
        case 'q': { const std::int64_t x = static_cast<std::int64_t>(jsRound(v.num)); put(&x, 8); break; }
        case 'f': { const float x = static_cast<float>(v.num); put(&x, 4); break; }
        case 'd': { const double x = v.num; put(&x, 8); break; }
        case 'B': case 'M': b[off] = static_cast<std::uint8_t>(v.num); break;
        case 'b': { const std::int8_t x = static_cast<std::int8_t>(v.num); put(&x, 1); break; }
        case 'H': case 'g': { const std::uint16_t x = static_cast<std::uint16_t>(v.num); put(&x, 2); break; }
        case 'h': { const std::int16_t x = static_cast<std::int16_t>(v.num); put(&x, 2); break; }
        case 'I': { const std::uint32_t x = static_cast<std::uint32_t>(v.num); put(&x, 4); break; }
        case 'i': { const std::int32_t x = static_cast<std::int32_t>(v.num); put(&x, 4); break; }
        case 'n': case 'N': case 'Z': {
          const std::size_t n = static_cast<std::size_t>(fmtSize(ch));
          std::memcpy(b.data() + off, v.s.data(), std::min(n, v.s.size()));
          break;
        }
        default: throw std::runtime_error(std::string("format char ") + ch + " not supported by the test builder");
      }
      off += static_cast<std::size_t>(fmtSize(ch));
    }
    bytes.insert(bytes.end(), b.begin(), b.end());
    return *this;
  }
  LogBuilder& raw(const std::vector<std::uint8_t>& r) {
    bytes.insert(bytes.end(), r.begin(), r.end());
    return *this;
  }
  const std::vector<std::uint8_t>& buffer() const { return bytes; }

  std::vector<std::uint8_t> bytes;
  std::map<std::string, Def> defs;

 private:
  static double jsRound(double x) {
    const double r = std::floor(x);
    return (x - r >= 0.5) ? r + 1.0 : r;
  }
};

/** the definitions used by the ready-made flights */
inline LogBuilder& standardFormats(LogBuilder& L) {
  return L.fmt(64, "PARM", "QNf", "TimeUS,Name,Value")
      .fmt(65, "MSG", "QZ", "TimeUS,Message")
      .fmt(66, "MODE", "QBB", "TimeUS,Mode,ModeNum")
      .fmt(67, "RATE", "Qfffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut")
      .fmt(68, "QTUN", "Qf", "TimeUS,ThO")
      .fmt(69, "CTUN", "Qf", "TimeUS,ThO")
      .fmt(70, "IMU", "QBfffH", "TimeUS,I,GyrX,GyrY,GyrZ,GHz");
}

struct RateSample {
  double des, act, out;
};
struct HoverOpts {
  std::string firmware = "ArduCopter V4.6.0";
  double rate = 300, seconds = 60, flyFrom = 5, flyTo = 55;
  std::string thrMsg = "CTUN";
  double thrScale = 1;
  std::optional<std::pair<double, double>> gap;   // no records at all in [t0, t1]
  double jitterUs = 0;
  std::vector<std::pair<std::string, double>> params;   // added to / replacing the defaults
  struct Change {
    double t;
    std::string name;
    double value;
  };
  std::vector<Change> changes;
  std::vector<std::pair<double, int>> modes;   // {t, mode}
  std::optional<double> nanAt, hugeAt;
  std::function<RateSample(double t, bool flying)> signal;   // roll; pitch / yaw get scaled copies
  std::optional<std::pair<double, double>> imu;              // {rate, ghz}
};

/** a hover flight in time order (see test/binlog.js hoverLog) */
inline std::vector<std::uint8_t> hoverLog(const HoverOpts& o = {}) {
  std::function<RateSample(double, bool)> sig = o.signal;
  if (!sig)
    sig = [](double t, bool fly) {
      const double des = 10 * std::sin(2 * kPiT * 0.5 * t);
      const double err = fly ? 2 : 10;
      return RateSample{des, des - err, 0.01 * err};
    };
  LogBuilder L;
  standardFormats(L);
  struct Ev {
    double t;
    std::string name;
    std::vector<V> values;
  };
  std::vector<Ev> events;
  const auto add = [&](double t, const std::string& name, std::vector<V> values) {
    if (o.gap && t >= o.gap->first && t <= o.gap->second) return;
    events.push_back({t, name, std::move(values)});
  };
  std::vector<std::pair<std::string, double>> params{{"INS_GYRO_FILTER", 20}, {"INS_GYRO_RATE", 0}, {"SCHED_LOOP_RATE", 400},
                                                     {"ATC_RAT_RLL_P", 0.1},  {"ATC_RAT_PIT_P", 0.1}, {"ATC_RAT_YAW_P", 0.2}};
  for (const auto& [k, v] : o.params) {
    bool found = false;
    for (auto& p : params)
      if (p.first == k) {
        p.second = v;
        found = true;
      }
    if (!found) params.emplace_back(k, v);
  }
  int k = 0;
  for (const auto& [name, v] : params) events.push_back({0.001 + 1e-6 * k++, "PARM", {0, name, v}});
  events.push_back({0.002, "MSG", {0, o.firmware}});
  for (const auto& [t, mode] : o.modes) add(t, "MODE", {0, mode, mode});
  for (const auto& c : o.changes) add(c.t, "PARM", {0, c.name, c.value});
  const long n = std::lround(o.seconds * o.rate);
  for (long i = 1; i <= n; i++) {
    const double t = static_cast<double>(i) / o.rate + (o.jitterUs != 0 ? o.jitterUs * 1e-6 * std::sin(1.7 * static_cast<double>(i)) : 0);
    const bool fly = t >= o.flyFrom && t < o.flyTo;
    const RateSample s = sig(t, fly);
    double des = s.des, act = s.act;
    if (o.nanAt && std::fabs(t - *o.nanAt) < 0.5 / o.rate) act = std::nan("");
    if (o.hugeAt && std::fabs(t - *o.hugeAt) < 0.5 / o.rate) des = 1e6;
    add(t, "RATE", {0, des, act, s.out, 0.5 * des, 0.5 * act, 0.5 * s.out, 0.2 * des, 0.2 * act, 0.2 * s.out});
  }
  for (int i = 0; i <= o.seconds * 10; i++) {
    const double t = i / 10.0 + 0.0003;
    add(t, o.thrMsg, {0, (t >= o.flyFrom && t < o.flyTo ? 0.5 : 0) * o.thrScale});
  }
  if (o.imu)
    for (long i = 1; static_cast<double>(i) <= o.seconds * o.imu->first; i++) {
      const double t = static_cast<double>(i) / o.imu->first + 0.00011;
      add(t, "IMU", {0, 0, 0.01 * std::sin(2 * kPiT * 40 * t), 0, 0, o.imu->second});
    }
  std::stable_sort(events.begin(), events.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
  for (auto& e : events) {
    e.values[0] = V(std::floor(e.t * 1e6 + 0.5));
    L.rec(e.name, e.values);
  }
  return L.buffer();
}

/** the audit scripts' deterministic Gaussian noise (LCG + Box-Muller), as test/logintegrity.test.js */
struct Noise {
  std::uint32_t s;
  explicit Noise(std::uint32_t seed) : s(seed) {}
  double rnd() {
    s = s * 1664525u + 1013904223u;
    return s / 4294967296.0;
  }
  double operator()() {
    double u = 0;
    while (u == 0) u = rnd();
    return std::sqrt(-2 * std::log(u)) * std::cos(2 * kPiT * rnd());
  }
};

}  // namespace aptuner::test
