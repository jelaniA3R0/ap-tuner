// DataFlash reader on a synthetic log built in memory: FMT definitions, numeric / string / scaled fields,
// parameters with in-flight changes, text messages, instances and the partial-log helpers.
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "aptuner/log/dfreader.hpp"
#include "check.hpp"

using namespace aptuner::log;

namespace {

struct LogBuilder {
  std::vector<std::uint8_t> b;
  void head(int type) {
    b.push_back(0xa3);
    b.push_back(0x95);
    b.push_back(static_cast<std::uint8_t>(type));
  }
  void u8(int v) { b.push_back(static_cast<std::uint8_t>(v)); }
  void raw(const void* p, std::size_t n) {
    const auto* c = static_cast<const std::uint8_t*>(p);
    b.insert(b.end(), c, c + n);
  }
  void str(const std::string& s, std::size_t n) {
    for (std::size_t i = 0; i < n; i++) b.push_back(i < s.size() ? static_cast<std::uint8_t>(s[i]) : 0);
  }
  void u64(std::uint64_t v) { raw(&v, 8); }
  void f32(float v) { raw(&v, 4); }
  void i16(std::int16_t v) { raw(&v, 2); }
  void i32(std::int32_t v) { raw(&v, 4); }
  void fmt(int type, int len, const std::string& name, const std::string& format, const std::string& labels) {
    head(128);
    u8(type);
    u8(len);
    str(name, 4);
    str(format, 16);
    str(labels, 64);
  }
};

LogBuilder sampleLog() {
  LogBuilder L;
  L.fmt(129, 3 + 8 + 16 + 4, "PARM", "QNf", "TimeUS,Name,Value");
  L.fmt(130, 3 + 8 + 64, "MSG", "QZ", "TimeUS,Message");
  L.fmt(131, 3 + 8 + 1 + 4 + 2 + 4 + 4, "TST", "QBfcLi", "TimeUS,I,Val,Scaled,Lat,Raw");
  auto parm = [&](std::uint64_t t, const char* n, float v) {
    L.head(129);
    L.u64(t);
    L.str(n, 16);
    L.f32(v);
  };
  parm(1000, "ATC_RAT_RLL_P", 0.135f);
  parm(2000, "INS_GYRO_FILTER", 20.0f);
  L.head(130);
  L.u64(3000);
  L.str("ArduCopter V4.6.0 (abc)", 64);
  for (int k = 0; k < 10; k++) {
    L.head(131);
    L.u64(static_cast<std::uint64_t>(10000 + 1000 * k));
    L.u8(k % 2);
    L.f32(static_cast<float>(k) * 0.5f);
    L.i16(static_cast<std::int16_t>(-150 + k));
    L.i32(static_cast<std::int32_t>(515000000 + k));
    L.i32(-7 * k);
    if (k == 4) {
      L.u8(0x55);   // garbage byte: the reader must resync
    }
  }
  parm(30000, "ATC_RAT_RLL_P", 0.2f);   // in-flight change
  return L;
}

}  // namespace

TEST(dfreader_parses_synthetic_log) {
  const LogBuilder L = sampleLog();
  const LogData log = parseBin(L.b.data(), L.b.size());
  CHECK(log.params.size() == 2);
  CHECK_NEAR(*log.params.get("ATC_RAT_RLL_P"), 0.135f, 0);   // boot value kept
  CHECK(log.changes.size() == 1);
  CHECK(log.changes.size() == 1 && log.changes[0].name == "ATC_RAT_RLL_P" && log.changes[0].t == 0.03);
  CHECK(log.text.size() == 1 && log.text[0].msg == "ArduCopter V4.6.0 (abc)" && log.text[0].t == 0.003);
  const MsgTable* t = log.msg("TST");
  CHECK(t && t->n == 10);
  if (!t) return;
  CHECK((*t->col("TimeUS"))[3] == 13000);
  CHECK((*t->col("Val"))[3] == 1.5);
  CHECK_NEAR((*t->col("Scaled"))[0], -1.5, 1e-15);
  CHECK_NEAR((*t->col("Lat"))[2], 51.5000002, 1e-12);
  CHECK((*t->col("Raw"))[9] == -63);
  CHECK(log.counts.at("TST") == 10 && log.counts.at("FMT") == 3 && log.counts.at("PARM") == 3);
  const auto inst = splitByInstance(*t, "I");
  CHECK(inst.size() == 2 && inst.at(0).n == 5 && inst.at(1).n == 5);
  CHECK((*inst.at(1).col("Val"))[0] == 0.5);
  // only the wanted messages become columns
  const LogData only = parseBin(L.b.data(), L.b.size(), {"PARM"});
  CHECK(only.msg("TST") == nullptr && only.counts.at("TST") == 10);
}

TEST(dfreader_partial_log_helpers) {
  const LogBuilder L = sampleLog();
  const auto fmts = readFormats(L.b.data(), L.b.size());
  CHECK(fmts.count(131) && fmts.at(131).name == "TST" && fmts.at(131).labels.size() == 6);
  // a slice from the middle of the log: starts inside a message
  const std::size_t cut = L.b.size() / 2;
  const std::uint8_t* mid = L.b.data() + cut;
  const std::size_t n = L.b.size() - cut;
  const long long s = firstSync(mid, n, fmts);
  CHECK(s > 0);
  const auto t0 = firstTime(mid, n, fmts), t1 = lastTime(mid, n, fmts);
  CHECK(t0.has_value() && t1.has_value());
  if (t0 && t1) CHECK(*t0 <= *t1 && *t1 == 0.03);
  CHECK(completeLength(L.b.data(), L.b.size() - 5, fmts) < L.b.size() - 5);
  Params p;
  p.set("A", 2.7);
  p.set("B", 0);
  p.set("C", -3.9);
  CHECK(p.intOf("A") == 2 && p.intOf("C") == -3 && p.intOf("missing") == 0);
  CHECK(p.truthyOr("B", 5) == 5 && p.getOr("B", 5) == 0 && p.getOr("missing", 7) == 7);
}
