// Log-analysis data integrity and signal processing (audit 2026-10-17, roadmap R1): the C++ counterpart of
// test/logintegrity.test.js. The cases come from the audit's independent checks; every expected value is worked out
// here from the signal definition, a closed form or an independent computation, never taken from the app.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/fft.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/core/plant.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/core/stepresp.hpp"
#include "aptuner/engine/analysis.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/notch.hpp"
#include "aptuner/log/dfreader.hpp"
#include "aptuner/log/integrity.hpp"
#include "binlog.hpp"
#include "check.hpp"
#include "indep_sim.hpp"

using namespace aptuner;
using aptuner::core::Json;
using aptuner::core::Vec;
using aptuner::test::HoverOpts;
using aptuner::test::LogBuilder;
using aptuner::test::Noise;
using aptuner::test::kPiT;

namespace {

constexpr double G = 57.2958;
const double kInf = std::numeric_limits<double>::infinity();

double roundT(double x) {
  const double r = std::floor(x);
  return (x - r >= 0.5) ? r + 1.0 : r;
}
std::vector<std::uint8_t> bytesOf(const std::string& s) { return std::vector<std::uint8_t>(s.begin(), s.end()); }
/** a telemetry-log-like buffer: big-endian microsecond stamps (2025-2026) every `step` bytes, a MAVLink start byte after each */
std::vector<std::uint8_t> tlogLike(std::size_t size, std::size_t records, std::size_t step, std::uint8_t magic) {
  std::vector<std::uint8_t> b(size, 0);
  for (std::size_t k = 0; k < records; k++) {
    const std::uint64_t us = 1760000000000000ull + 1000ull * k;
    for (int i = 0; i < 8; i++) b[k * step + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(us >> (56 - 8 * i));
    b[k * step + 8] = magic;
  }
  return b;
}
bool anyContains(const Json& arr, const std::string& what) {
  if (!arr.isArray()) return false;
  for (const auto& s : arr.arr())
    if (s.isString() && s.str().find(what) != std::string::npos) return true;
  return false;
}
Json analyze(const std::vector<std::uint8_t>& buf, const engine::AnalyzeLogOptions& opt = {}) {
  log::LogData lg = log::parseBin(buf.data(), buf.size(), engine::logWant());
  return engine::analyzeLog(lg, "test.bin", opt);
}
Json analyzeHover(const HoverOpts& o, const engine::AnalyzeLogOptions& opt = {}) { return analyze(test::hoverLog(o), opt); }
log::Params paramsOf(const std::vector<std::pair<std::string, double>>& kv) {
  log::Params p;
  for (const auto& [k, v] : kv) p.set(k, v);
  return p;
}

/** a gyro-only log as the audit's tests build it (GYR or IMU table + CTUN throttle), see logintegrity.test.js */
struct GyroLogOpts {
  std::string table = "GYR";
  double fs = 1000, seconds = 10;
  std::function<double(double)> x, y, z;
  std::vector<std::pair<std::string, double>> params;
  double thr = 0.4;
  bool percent = false;
  std::function<bool(double)> keep;
};
log::LogData gyroLog(const GyroLogOpts& o) {
  std::vector<double> ts;
  for (long i = 0; static_cast<double>(i) < o.fs * o.seconds; i++) {
    const double t = static_cast<double>(i) / o.fs;
    if (!o.keep || o.keep(t)) ts.push_back(t);
  }
  const std::size_t n = ts.size();
  log::MsgTable tab;
  tab.n = n;
  Vec TimeUS(n), I(n, 0.0), X(n), Y(n), Z(n);
  for (std::size_t k = 0; k < n; k++) {
    const double t = ts[k];
    TimeUS[k] = roundT((10 + t) * 1e6);
    X[k] = (o.x ? o.x(t) : 0) / G;
    Y[k] = (o.y ? o.y(t) : 0) / G;
    Z[k] = (o.z ? o.z(t) : 0) / G;
  }
  tab.labels = {"TimeUS", "I", "GyrX", "GyrY", "GyrZ"};
  tab.num["TimeUS"] = TimeUS;
  tab.num["I"] = I;
  tab.num["GyrX"] = X;
  tab.num["GyrY"] = Y;
  tab.num["GyrZ"] = Z;
  if (o.table == "GYR") {
    tab.labels.push_back("SampleUS");
    tab.num["SampleUS"] = TimeUS;
  }
  const std::size_t m = static_cast<std::size_t>(roundT(o.seconds * 10));
  log::MsgTable ct;
  ct.n = m;
  ct.labels = {"TimeUS", "ThO"};
  Vec cT(m), cTh(m);
  for (std::size_t i = 0; i < m; i++) {
    cT[i] = roundT((10 + static_cast<double>(i) / 10) * 1e6);
    cTh[i] = o.percent ? o.thr * 100 : o.thr;
  }
  ct.num["TimeUS"] = cT;
  ct.num["ThO"] = cTh;
  log::LogData lg;
  lg.msgs[o.table] = std::move(tab);
  lg.msgs["CTUN"] = std::move(ct);
  lg.params = paramsOf(o.params);
  lg.integrity.format.kind = "dataflash";
  lg.integrity.format.supported = true;
  return lg;
}
Json noiseOf(const log::LogData& lg, const log::Params& params, core::Family fam) {
  return engine::analyzeNoise(lg, params, fam, engine::Window{});
}
engine::AxisData rateData(double fs, const Vec& actual) {
  engine::AxisData d;
  d.fs = fs;
  d.n = actual.size();
  d.target.assign(actual.size(), 0.0);
  d.actual = actual;
  d.u.assign(actual.size(), 0.0);
  return d;
}

}  // namespace

// ------------------------------------------------------------------------------------------- DataFlash reader (F-79)

TEST(integrity_file_type_detection) {
  const std::vector<std::uint8_t> empty;
  CHECK(log::sniffFormat(empty.data(), 0).kind == "empty");
  const std::vector<std::uint8_t> tl = tlogLike(40, 1, 40, 0xfd);
  CHECK(log::sniffFormat(tl.data(), tl.size()).kind == "tlog");
  const auto txt = bytesOf("FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n");
  CHECK(log::sniffFormat(txt.data(), txt.size()).kind == "dataflash-text");
  const auto csv = bytesOf("TimeUS,Tar,Act\n1000,0.1,0.09\n");
  CHECK(log::sniffFormat(csv.data(), csv.size()).kind == "text");
  const std::vector<std::uint8_t> junk(600, 0x95);
  CHECK(log::sniffFormat(junk.data(), junk.size()).kind == "unknown");
  LogBuilder L;
  L.fmt(70, "TST", "Qf", "TimeUS,V");
  CHECK(log::sniffFormat(L.buffer().data(), L.buffer().size()).kind == "dataflash");
  for (const auto& b : {empty, tl, bytesOf("a,b\n1,2\n")}) {
    const log::LogData p = log::parseBin(b.data(), b.size());
    CHECK(!p.integrity.format.supported && !p.integrity.format.message.empty());
  }
}

TEST(integrity_float16) {
  CHECK(log::f16(0x3e00) == 1.5);
  CHECK(log::f16(0xc000) == -2);
  CHECK(log::f16(0x7bff) == 65504);
  CHECK(log::f16(0x7c00) == kInf);
  CHECK(log::f16(0xfc00) == -kInf);
  CHECK(std::isnan(log::f16(0x7e00)));
  CHECK(log::f16(0x0001) == std::pow(2.0, -24));   // smallest subnormal
  CHECK(log::f16(0x3555) == 0.333251953125);       // 2^-2 * (1 + 341/1024)
  LogBuilder L;
  L.fmt(200, "TST", "QgfB", "TimeUS,G,F,B");
  for (int k = 0; k < 3; k++) L.rec("TST", {1e6 + k, 0x3e00, 2.25, 7});
  const log::LogData p = log::parseBin(L.buffer().data(), L.buffer().size());
  const log::MsgTable* m = p.msg("TST");
  CHECK(m && m->n == 3);
  if (m) {
    CHECK((*m->col("G"))[0] == 1.5);
    CHECK((*m->col("F"))[0] == 2.25);
    CHECK((*m->col("B"))[2] == 7);
  }
}

TEST(integrity_fmt_validation) {
  LogBuilder L;
  L.fmt(70, "TST", "Qf", "TimeUS,V").fmt(201, "BAD", "Qffff", "TimeUS,A,B,C,D", 15).fmt(202, "UNK", "Qx", "TimeUS,X", 12);
  for (int k = 0; k < 5; k++) L.rec("TST", {1000 * k, k});
  const log::LogData p = log::parseBin(L.buffer().data(), L.buffer().size());
  CHECK(p.integrity.badFmtCount == 2);
  CHECK(p.integrity.badFmts.size() == 2);
  if (p.integrity.badFmts.size() == 2) {
    CHECK(p.integrity.badFmts[0].reason.find("length 15 does not match its format (27 bytes)") != std::string::npos);
    CHECK(p.integrity.badFmts[1].reason.find("unknown format character 'x'") != std::string::npos);
  }
  CHECK((!p.msg("BAD") || p.msg("BAD")->n == 0) && p.msg("TST") && p.msg("TST")->n == 5);
}

TEST(integrity_records_must_chain) {
  LogBuilder L;
  L.fmt(70, "TST", "Qf", "TimeUS,V");
  for (int k = 0; k < 10; k++) {
    L.rec("TST", {1000 * (k + 1), k});
    if (k == 4) L.raw({0x55});
  }
  const log::LogData p = log::parseBin(L.buffer().data(), L.buffer().size());
  const log::MsgTable* t = p.msg("TST");
  CHECK(t && t->n == 9);
  if (t) CHECK(*t->col("V") == (Vec{0, 1, 2, 3, 5, 6, 7, 8, 9}));
  CHECK(p.integrity.skippedBytes == 15 + 1);   // the unverifiable record and the junk byte
  CHECK(p.integrity.resyncs == 1);
}

TEST(integrity_corrupted_type_byte) {
  LogBuilder L;
  L.fmt(70, "TST", "Qf", "TimeUS,V").fmt(71, "TS2", "Qff", "TimeUS,A,B");
  for (int k = 0; k < 10; k++) L.rec("TST", {1000 * (k + 1), k});
  std::vector<std::uint8_t> b = L.buffer();
  b[89 * 2 + 15 * 5 + 2] = 71;   // record 5 now claims to be a 19-byte TS2
  const log::LogData p = log::parseBin(b.data(), b.size());
  CHECK(p.msg("TST") && p.msg("TST")->n == 9);
  CHECK(!p.msg("TS2") || p.msg("TS2")->n == 0);
}

TEST(integrity_truncated_padding_zero_tail) {
  LogBuilder L;
  L.fmt(70, "TST", "Qf", "TimeUS,V");
  for (int k = 0; k < 10; k++) L.rec("TST", {1000 * (k + 1), k + 1});
  const std::vector<std::uint8_t> b = L.buffer();
  const log::LogData cut = log::parseBin(b.data(), b.size() - 5);
  CHECK(cut.integrity.truncatedBytes == 10);
  CHECK(cut.msg("TST") && cut.msg("TST")->n == 9);
  std::vector<std::uint8_t> padded = b;
  padded.resize(b.size() + 1000, 0);
  const log::LogData pad = log::parseBin(padded.data(), padded.size());
  CHECK(pad.integrity.paddingBytes == 1000 && pad.integrity.truncatedBytes == 0 && pad.integrity.skippedBytes == 0);
  CHECK(pad.msg("TST") && pad.msg("TST")->n == 10);
  LogBuilder L2;
  L2.fmt(70, "TST", "Qf", "TimeUS,V");
  L2.rec("TST", {1000, 1}).rec("TST", {2000, 0});   // 0.0 = four zero bytes at the end
  const log::LogData z = log::parseBin(L2.buffer().data(), L2.buffer().size());
  CHECK(z.msg("TST") && z.msg("TST")->n == 2);
  CHECK(z.integrity.truncatedBytes == 0 && z.integrity.paddingBytes == 0);
}

TEST(integrity_parameter_change_back_counts) {
  LogBuilder L;
  L.fmt(64, "PARM", "QNf", "TimeUS,Name,Value");
  L.rec("PARM", {1000, "ATC_RAT_RLL_P", 0.1})
      .rec("PARM", {10e6, "ATC_RAT_RLL_P", 0.2})
      .rec("PARM", {20e6, "ATC_RAT_RLL_P", 0.1})
      .rec("PARM", {25e6, "ATC_RAT_RLL_P", 0.1});
  const log::LogData p = log::parseBin(L.buffer().data(), L.buffer().size());
  CHECK(p.params.get("ATC_RAT_RLL_P") == static_cast<double>(0.1f));
  CHECK(p.changes.size() == 2);
  if (p.changes.size() == 2) {
    CHECK(p.changes[0].t == 10 && p.changes[0].val == static_cast<double>(0.2f));
    CHECK(p.changes[1].t == 20 && p.changes[1].val == static_cast<double>(0.1f));
  }
}

// ------------------------------------------------------------------------------------------- time stamps and gaps (F-07)

TEST(integrity_clean_times) {
  using K = std::vector<std::size_t>;
  log::TimeClean c = log::cleanTimes({0, 1, 2, 2, 3});
  CHECK(c.keep == (K{0, 1, 2, 4}) && c.duplicates == 1);
  c = log::cleanTimes({0, 1, 2, -5, 3, 4});
  CHECK(c.keep == (K{0, 1, 2, 4, 5}) && c.outOfOrder == 1);
  c = log::cleanTimes({0, 1, 2, 1e9, 3, 4, 5});
  CHECK(c.keep == (K{0, 1, 2, 4, 5, 6}) && c.outOfOrder == 1);
  c = log::cleanTimes({0, 1, 2, 1e9, 2e9, 3, 4, 5, 6});
  CHECK(c.keep == (K{0, 1, 2, 5, 6, 7, 8}) && c.outOfOrder == 2);
  Vec reset;
  for (int i = 0; i <= 10; i++) reset.push_back(100 + i);
  for (int i = 0; i <= 20; i++) reset.push_back(i);
  c = log::cleanTimes(reset);
  CHECK(c.keep.size() == 11 && c.reset && c.reset->first == 110 && c.reset->second == 0 && c.droppedAfterReset == 21);
  c = log::cleanTimes({0, 1, 2, 3, -1});
  CHECK(c.outOfOrder == 1 && !c.reset);
}

TEST(integrity_split_at_gaps) {
  Vec t;
  for (int i = 0; i <= 1000; i++) t.push_back(i / 100.0);
  for (int i = 2000; i <= 3000; i++) t.push_back(i / 100.0);
  const log::GapSplit s = log::splitAtGaps(t);
  CHECK(s.segments == (std::vector<std::pair<std::size_t, std::size_t>>{{0, 1001}, {1001, 2002}}));
  CHECK(s.gaps == (std::vector<std::pair<double, double>>{{10, 20}}));
  Vec t2;
  for (int i = 0; i <= 100; i++)
    if (i != 50) t2.push_back(i / 100.0);
  CHECK(log::splitAtGaps(t2).segments.size() == 1);
}

TEST(integrity_clean_log_times_per_instance) {
  log::LogData lg;
  log::MsgTable rate;
  rate.n = 6;
  rate.labels = {"TimeUS", "R"};
  rate.num["TimeUS"] = {1e6, 2e6, 900, 4e6, 5e6, 6e6};
  rate.num["R"] = {1, 2, 3, 4, 5, 6};
  log::MsgTable imu;
  imu.n = 6;
  imu.labels = {"TimeUS", "I", "GyrX"};
  imu.num["TimeUS"] = {1e6, 1e6, 2e6, 2e6, 3e6, 3e6};
  imu.num["I"] = {0, 1, 0, 1, 0, 1};
  imu.num["GyrX"] = Vec(6, 0.0);
  lg.msgs["RATE"] = rate;
  lg.msgs["IMU"] = imu;
  const auto rep = log::cleanLogTimes(lg, {"RATE", "IMU"});
  CHECK(rep.size() == 1 && rep.count("RATE") && rep.at("RATE").outOfOrder == 1);
  CHECK(*lg.msg("RATE")->col("R") == (Vec{1, 2, 4, 5, 6}));
  CHECK(lg.msg("IMU")->n == 6);
}

TEST(integrity_logging_gap_in_flight) {
  HoverOpts o;
  o.gap = std::make_pair(20.0, 25.0);
  o.jitterUs = 800;
  o.nanAt = 30;
  o.hugeAt = 40;
  const Json r = analyzeHover(o);
  const Json& a = r["axes"]["roll"];
  const Json& di = a["dataInfo"];
  CHECK(di["segments"].numOr(-1) == 2 && di["gapCount"].numOr(-1) == 1);
  CHECK_NEAR(di["gaps"][0]["t0"].numOr(0), 20, 0.01);
  CHECK_NEAR(di["gaps"][0]["t1"].numOr(0), 25, 0.01);
  CHECK_NEAR(di["fs"].numOr(0), 300, 0.01);
  CHECK(di["excluded"]["nonFinite"].numOr(-1) == 1 && di["excluded"]["implausible"].numOr(-1) == 1);
  CHECK(anyContains(a["warnings"], "Logging stopped for 5.0 s"));
  bool outside = true;
  for (const auto& t : a["series"]["t"].arr()) outside = outside && (t.num() < 20 || t.num() > 25);
  CHECK(outside);
  // the error is 2 deg/s while flying and 10 deg/s on the ground: only the flown samples count (F-75)
  CHECK_NEAR(a["diagnostics"]["rmsErrorDegs"].numOr(0), 2, 1e-5);
  CHECK_NEAR(a["diagnostics"]["saturation"]["steadyErrorDegs"].numOr(0), 2, 1e-5);
  // usable samples = RATE records inside the airborne stretches, counted from the record times: throttle 0.5 from
  // 5.0003 s, a gap 20-25 s in the throttle log too, 0 from 55.0003 s; 3 s / 2 s ground margins, none at the gap
  const std::pair<double, double> air[2] = {{5.0003 + 3, 19.9003 + 0.5}, {25.0003, 55.0003 - 2}};
  long n = 0;
  for (int i = 1; i <= 60 * 300; i++) {
    const double tt = i / 300.0 + 800e-6 * std::sin(1.7 * i);
    if (tt >= 20 && tt <= 25) continue;
    const double ts = roundT(tt * 1e6) / 1e6;
    if (std::fabs(tt - 30) < 0.5 / 300 || std::fabs(tt - 40) < 0.5 / 300) continue;
    for (const auto& [p0, p1] : air)
      if (ts >= p0 && ts < p1) n++;
  }
  CHECK_NEAR(di["usableSeconds"].numOr(0), static_cast<double>(n) / di["fs"].numOr(1), 1e-9);
}

TEST(integrity_clock_reset_uses_first_part) {
  HoverOpts a, b;
  a.seconds = 40;
  a.flyTo = 35;
  b.seconds = 20;
  b.flyTo = 15;
  std::vector<std::uint8_t> buf = test::hoverLog(a);
  const std::vector<std::uint8_t> second = test::hoverLog(b);
  buf.insert(buf.end(), second.begin(), second.end());
  const Json r = analyze(buf);
  const Json& ti = r["integrity"]["timeIssues"]["RATE"];
  CHECK(ti.isObject() && ti["reset"].isObject());
  CHECK_NEAR(ti["reset"]["fromS"].numOr(0), 40, 0.01);
  CHECK_NEAR(ti["reset"]["toS"].numOr(0), 1 / 300.0, 1e-4);
  CHECK(anyContains(r["integrity"]["notes"], "clock jumps back"));
  CHECK_NEAR(r["flightWindow"]["tEnd"].numOr(0), 34.9003, 1e-6);
}

// ------------------------------------------------------------------------------------------- window, parameters, modes, throttle

TEST(integrity_gain_change_splits_window) {
  HoverOpts o;
  o.params = {{"STAT_RUNTIME", 1}};
  o.changes = {{20, "ATC_RAT_RLL_P", 0.2}, {35, "STAT_RUNTIME", 7}};
  const Json r = analyzeHover(o);
  CHECK(r["paramChanges"].numOr(-1) == 1 && r["paramChangesAll"].numOr(-1) == 2);
  CHECK_NEAR(r["flightWindow"]["tStart"].numOr(0), 20.0003, 1e-6);
  CHECK_NEAR(r["flightWindow"]["tEnd"].numOr(0), 54.9003, 1e-6);
  CHECK(r["params"]["ATC_RAT_RLL_P"].numOr(0) == static_cast<double>(0.2f));
  CHECK(r["axes"]["roll"]["current"]["P"].numOr(0) == static_cast<double>(0.2f));
  const Json& tc = r["tuningChanges"];
  CHECK(tc.size() == 1 && tc[0]["t"].numOr(0) == 20 && tc[0]["name"].strOr("") == "ATC_RAT_RLL_P" &&
        tc[0]["from"].numOr(0) == static_cast<double>(0.1f) && tc[0]["to"].numOr(0) == static_cast<double>(0.2f));
  CHECK(anyContains(r["integrity"]["notes"], "Tuning parameters changed during the flight (ATC_RAT_RLL_P 0.1 -> 0.2 at 20.0 s)"));
  HoverOpts e;
  e.changes = {{40, "ATC_RAT_RLL_P", 0.2}};
  const Json early = analyzeHover(e);
  CHECK_NEAR(early["flightWindow"]["tEnd"].numOr(0), 39.9003, 1e-6);   // the longest piece is before the change
  CHECK(early["params"]["ATC_RAT_RLL_P"].numOr(0) == static_cast<double>(0.1f));
  CHECK(engine::isTuningParam("INS_HNTCH_FREQ") && engine::isTuningParam("SERVO5_FUNCTION") &&
        !engine::isTuningParam("STAT_RUNTIME") && !engine::isTuningParam("INS_GYROFFS_X") &&
        !engine::isTuningParam("Q_M_THST_HOVER"));
}

TEST(integrity_vtol_modes_and_transition) {
  for (int m : {17, 18, 19, 20, 21, 22, 23, 25}) CHECK(engine::isQMode(m));
  for (int m : {5, 24, 26}) CHECK(!engine::isQMode(m));
  engine::AnalyzeLogOptions vtol;
  vtol.family = core::Family::Vtol;
  HoverOpts o;
  o.firmware = "ArduPlane V4.7.1";
  o.thrMsg = "QTUN";
  o.modes = {{1, 5}, {10, 18}};
  const Json r = analyzeHover(o, vtol);
  CHECK_NEAR(r["flightWindow"]["tStart"].numOr(0), 15.0003, 1e-6);   // 5 s after FBWA -> QHOVER
  CHECK(anyContains(r["integrity"]["notes"], "first 5 s after the switch from FBWA to QHOVER"));
  HoverOpts t = o;
  t.modes = {{1, 24}};
  const Json none = analyzeHover(t, vtol);
  CHECK(none["axes"]["roll"]["error"].strOr("").find("no flight in a VTOL (Q) mode") != std::string::npos);
}

TEST(integrity_plane_throttle_percent) {
  GyroLogOpts g;
  g.fs = 1000;
  g.seconds = 60;
  g.x = [](double t) { return 5 * std::sin(2 * kPiT * 60.06 * t) + 2 * std::sin(2 * kPiT * 120.12 * t); };
  g.params = {{"INS_GYRO_FILTER", 20}};
  g.thr = 0.45;
  g.percent = true;
  log::LogData lg = gyroLog(g);
  lg.text = {{0, "ArduPlane V4.7.1"}};
  const Json r = noiseOf(lg, lg.params, core::Family::Plane);
  CHECK(r["recommended"].isObject());
  CHECK(r["recommended"]["params"]["INS_HNTCH_REF"].numOr(0) == 0.45);
}

// ------------------------------------------------------------------------------------------- filter model (F-74, F-78, F-89, F-77)

TEST(integrity_notch_operating_point) {
  struct Case {
    int mode;
    double freq, ref, fmRat;
    core::NotchOp op;
    double want;
  };
  const auto mkOp = [](double throttle, double rpm, double rpm2, double esc, double fft) {
    core::NotchOp o;
    if (throttle >= 0) o.throttle = throttle;
    o.rpm = rpm;
    o.rpm2 = rpm2;
    o.escRpm = esc;
    o.fftHz = fft;
    return o;
  };
  const Case cases[] = {
      {3, 100, 2, 1, mkOp(-1, 0, 0, 3000, 0), 100},   // motors 50 Hz x REF 2
      {3, 40, 1, 1, mkOp(-1, 0, 0, 6000, 0), 100},
      {3, 50, 1, 0.8, mkOp(-1, 0, 0, 1800, 0), 40},   // 30 Hz is below the 40 Hz floor
      {3, 50, 1, 0.8, mkOp(-1, 0, 0, 0, 0), 40},      // no ESC data: at the floor
      {2, 30, 1, 1, mkOp(-1, 3000, 0, 0, 0), 50},
      {5, 30, 1, 1, mkOp(-1, 3000, 4200, 0, 0), 70},
      {4, 60, 1, 0.75, mkOp(-1, 0, 0, 0, 40), 45},
      {3, 80, 0, 1, mkOp(-1, 0, 0, 9000, 0), 80},     // REF 0: no dynamic update
      {1, 80, 0.25, 0.9, mkOp(0.1, 0, 0, 0, 0), 72},
  };
  for (const Case& c : cases) {
    core::NotchParams p;
    p.mode = c.mode;
    p.freq = c.freq;
    p.ref = c.ref;
    p.fmRat = c.fmRat;
    CHECK_NEAR(core::notchBaseFreq(p, c.op), c.want, 1e-9);
  }
  core::NotchParams thr;
  thr.mode = 1;
  thr.freq = 80;
  thr.ref = 0.25;
  thr.fmRat = 1;
  CHECK_NEAR(core::notchBaseFreq(thr, mkOp(0.39, 0, 0, 0, 0)), 80 * std::sqrt(0.39 / 0.25), 1e-9);
  const core::GyroChain ch = engine::gyroChainFromParams(
      paramsOf({{"INS_HNTCH_ENABLE", 1}, {"INS_HNTCH_MODE", 3}, {"INS_HNTCH_FREQ", 40}, {"INS_HNTCH_BW", 20}, {"INS_HNTCH_REF", 1}, {"INS_HNTCH_FM_RAT", 1}}),
      mkOp(-1, 0, 0, 6000, 0));
  CHECK(!ch.notches.empty());
  if (!ch.notches.empty()) CHECK(ch.notches[0].freq == 100 && ch.notches[0].refFreq == 40 && ch.notches[0].refBw == 20);
}

namespace {
/** independent ArduPilot NotchFilter magnitude at f (Hz) for centre c, quality Q, attenuation A (linear) */
double fwNotch(double f, double fs, double c, double Q, double A) {
  const double w = 2 * kPiT * c / fs, al = std::sin(w) / (2 * Q);
  const double b0 = 1 + al * A * A, b1 = -2 * std::cos(w), b2 = 1 - al * A * A, a0 = 1 + al, a1 = b1, a2 = 1 - al, z = 2 * kPiT * f / fs;
  return std::hypot(b0 + b1 * std::cos(z) + b2 * std::cos(2 * z), b1 * std::sin(z) + b2 * std::sin(2 * z)) /
         std::hypot(a0 + a1 * std::cos(z) + a2 * std::cos(2 * z), a1 * std::sin(z) + a2 * std::sin(2 * z));
}
double Qof(double c, double bw) {
  const double o = std::log2(c / (c - bw / 2)) * 2;
  return std::sqrt(std::pow(2, o)) / (std::pow(2, o) - 1);
}
core::NotchDef movedNotch(double c, int opts) {
  core::NotchDef n;
  n.enable = true;
  n.freq = c;
  n.bw = 40;
  n.att = 40;
  n.hmncs = 1;
  n.opts = opts;
  n.refFreq = 80;
  n.refBw = 40;
  return n;
}
}  // namespace

TEST(integrity_moved_notch_keeps_configured_q) {
  const double c = 80 * std::sqrt(0.39 / 0.25), Q = Qof(80, 40);
  CHECK_NEAR(Q, 1.7142857, 1e-6);
  for (double f : {80.0, 90.0, 95.0, 105.0, 120.0}) {
    const double app = core::cx::abs(core::harmonicNotch(Vec{f}, 1000, movedNotch(c, 0)))[0];
    CHECK_NEAR(app, fwNotch(f, 1000, c, Q, 0.1), 1e-12);
  }
  CHECK_NEAR(fwNotch(90, 1000, c, Q, 0.1), 0.357, 5e-4);   // the audit's value at 90 Hz
  // double notch: both halves use the Q of BW/2 and the spread BW / (32 FREQ), fixed from the configured values
  const double sp = 40.0 / (32 * 80), Q2 = Qof(80, 20);
  for (double f : {85.0, 100.0, 110.0}) {
    const double app = core::cx::abs(core::harmonicNotch(Vec{f}, 1000, movedNotch(c, 1)))[0];
    CHECK_NEAR(app, fwNotch(f, 1000, c * (1 - sp), Q2, 0.1) * fwNotch(f, 1000, c * (1 + sp), Q2, 0.1), 1e-12);
  }
}

TEST(integrity_filters_at_logged_gyro_rate) {
  const log::Params p = paramsOf({{"INS_GYRO_RATE", 1}, {"INS_GYRO_FILTER", 20}});
  const core::GyroChain a = engine::gyroChainFromParams(p, core::NotchOp{});
  CHECK(a.backendHz == 2000 && a.backendSource == "INS_GYRO_RATE");
  core::NotchOp op;
  op.backendHz = 3276;
  const core::GyroChain b = engine::gyroChainFromParams(p, op);
  CHECK(b.backendHz == 3276 && b.backendSource == "IMU.GHz");
  HoverOpts o;
  o.imu = std::make_pair(300.0, 3276.0);
  const Json r = analyzeHover(o);
  CHECK(r["axes"]["roll"]["gyro"]["backendHz"].numOr(0) == 3276);
}

TEST(integrity_gyr_post_filter_option) {
  const auto mk = [](double opt) {
    GyroLogOpts g;
    g.fs = 1000;
    g.seconds = 30;
    g.x = [](double t) { return 0.3 * std::sin(2 * kPiT * 60.06 * t); };
    g.params = {{"INS_RAW_LOG_OPT", opt}, {"INS_GYRO_FILTER", 80}, {"INS_HNTCH_ENABLE", 1}, {"INS_HNTCH_MODE", 0},
                {"INS_HNTCH_FREQ", 60}, {"INS_HNTCH_BW", 30}, {"INS_HNTCH_ATT", 40}, {"INS_HNTCH_HMNCS", 1}};
    return gyroLog(g);
  };
  const log::LogData l5 = mk(5), l9 = mk(9), l0 = mk(0);
  const Json post = noiseOf(l5, l5.params, core::Family::Copter);
  CHECK(post["prefilter"].isBool() && !post["prefilter"].asBool());
  CHECK(!post.has("postCurrent"));   // not filtered a second time
  CHECK(noiseOf(l9, l9.params, core::Family::Copter)["prefilter"].asBool());   // pre and post: instance 0 is raw
  CHECK(noiseOf(l0, l0.params, core::Family::Copter)["prefilter"].asBool());
}

// ------------------------------------------------------------------------------------------- spectra (F-83, F-62, F-76, F-81)

TEST(integrity_batches_are_separate_welch_blocks) {
  double floors[2] = {0, 0};
  for (int mode = 0; mode < 2; mode++) {   // 0 continuous, 1 batched
    const bool batched = mode == 1;
    const double fs = 1000, mul = 1000;
    const int B = 1024, nb = 40;
    Noise g(14);
    std::uint32_t rs = 14;
    const auto rnd = [&]() {
      rs = rs * 1664525u + 1013904223u;
      return rs / 4294967296.0;
    };
    log::MsgTable H, D;
    Vec hT, hN, hType, hInst, hMul, hRate, dT, dN;
    std::vector<std::array<std::int16_t, 32>> dx, dy, dz;
    long tc = 0;
    for (int b = 0; b < nb; b++) {
      const double ph = batched ? 2 * kPiT * rnd() : 0, tb = 10 + b * 3;
      hT.push_back(tb * 1e6);
      hN.push_back(b);
      hType.push_back(1);
      hInst.push_back(0);
      hMul.push_back(mul);
      hRate.push_back(fs);
      for (int r = 0; r < B / 32; r++) {
        std::array<std::int16_t, 32> x{}, y{}, z{};
        for (int k = 0; k < 32; k++) {
          const double t = batched ? (r * 32 + k) / fs : static_cast<double>(tc) / fs;
          tc++;
          x[static_cast<std::size_t>(k)] = static_cast<std::int16_t>(roundT((10 * std::sin(2 * kPiT * 40 * t + ph) + 0.1 * g()) / G * mul));
          y[static_cast<std::size_t>(k)] = static_cast<std::int16_t>(roundT(0.1 * g() / G * mul));
          z[static_cast<std::size_t>(k)] = static_cast<std::int16_t>(roundT(0.1 * g() / G * mul));
        }
        dT.push_back(tb * 1e6 + r * 1000);
        dN.push_back(b);
        dx.push_back(x);
        dy.push_back(y);
        dz.push_back(z);
      }
    }
    H.n = hT.size();
    H.labels = {"TimeUS", "N", "type", "instance", "mul", "smp_rate"};
    H.num["TimeUS"] = hT;
    H.num["N"] = hN;
    H.num["type"] = hType;
    H.num["instance"] = hInst;
    H.num["mul"] = hMul;
    H.num["smp_rate"] = hRate;
    D.n = dT.size();
    D.labels = {"TimeUS", "N", "x", "y", "z"};
    D.num["TimeUS"] = dT;
    D.num["N"] = dN;
    D.i16["x"] = dx;
    D.i16["y"] = dy;
    D.i16["z"] = dz;
    log::LogData lg;
    lg.msgs["ISBH"] = H;
    lg.msgs["ISBD"] = D;
    lg.params = paramsOf({{"INS_LOG_BAT_OPT", 0}, {"INS_GYRO_FILTER", 20}});
    const Json r = noiseOf(lg, lg.params, core::Family::Copter);
    const Vec f = r["f"].toVec(), amp = r["amp"].toVec();
    Vec fl;
    for (std::size_t k = 0; k < f.size(); k++)
      if (f[k] > 100 && f[k] < 400) fl.push_back(amp[k]);
    std::sort(fl.begin(), fl.end());
    CHECK(!fl.empty());
    if (!fl.empty()) floors[mode] = fl[fl.size() / 2];
  }
  // the noise floor of joined-up batches equals that of a continuous record (the audit before the fix: x1.81)
  const double ratio = floors[1] / floors[0];
  CHECK(ratio > 0.9 && ratio < 1.1);
}

TEST(integrity_dropouts_make_no_spurious_peaks) {
  Noise g(5);
  GyroLogOpts o;
  o.fs = 1000;
  o.seconds = 60;
  o.x = [&g](double t) { return 10 * std::sin(2 * kPiT * 40 * t) + 0.05 * g(); };
  o.keep = [](double t) { return std::fmod(t, 0.5) >= 0.037; };
  const log::LogData lg = gyroLog(o);
  const Json r = noiseOf(lg, lg.params, core::Family::Copter);
  const double df = r["f"][1].numOr(0) - r["f"][0].numOr(0);
  CHECK(r["peaks"].size() == 1);
  if (r["peaks"].size() >= 1) CHECK(std::fabs(r["peaks"][0]["f"].numOr(0) - 40) <= df);
}

TEST(integrity_aliasing_rate_data_not_called_oscillation) {
  Noise g(4);
  const double fs = 25;
  const std::size_t n = 25 * 60;
  Vec actual(n);
  for (std::size_t i = 0; i < n; i++) actual[i] = 10 * std::sin(2 * kPiT * 30 * static_cast<double>(i) / fs + 0.3) + 0.2 * g();
  const Json d = engine::diagnostics(rateData(fs, actual), Json::object(), core::Family::Copter, core::Axis::Roll);
  CHECK(d["oscillating"].isBool() && !d["oscillating"].asBool());
  CHECK(d["aliasRisk"].asBool() && d["peakAmbiguous"].asBool());
  CHECK_NEAR(d["peakHz"].numOr(0), 5, 0.5);   // the folded peak sits at |30 - 25| = 5 Hz
  CHECK(anyContains(d["grade"]["notes"], "aliasing"));
}

TEST(integrity_aliasing_no_notch_from_slow_post_filter_gyro) {
  Noise g(12);
  GyroLogOpts a;
  a.table = "IMU";
  a.fs = 50;
  a.seconds = 120;
  a.x = [&g](double t) { return 2 * std::sin(2 * kPiT * 130 * t + 0.1) + 0.2 * g(); };
  a.y = [&g](double t) { return 2 * std::sin(2 * kPiT * 130 * t + 1.1) + 0.2 * g(); };
  a.params = {{"INS_GYRO_FILTER", 20}};
  const log::LogData t12 = gyroLog(a);
  const Json r12 = noiseOf(t12, t12.params, core::Family::Copter);
  CHECK(!r12.has("recommended") && r12["aliasRisk"].asBool() && r12["recommendation"].strOr("").find("aliasing") != std::string::npos);
  GyroLogOpts b;
  b.table = "IMU";
  b.fs = 400;
  b.seconds = 120;
  b.x = [&g](double t) { return 0.8 * std::sin(2 * kPiT * 250 * t) + 0.4 * std::sin(2 * kPiT * 500 * t + 0.7) + 0.05 * g(); };
  b.params = {{"INS_GYRO_FILTER", 80}};
  const log::LogData t12b = gyroLog(b);
  const Json r12b = noiseOf(t12b, t12b.params, core::Family::Copter);
  CHECK(!r12b.has("recommended") && r12b["aliasRisk"].asBool());
  GyroLogOpts c;
  c.table = "IMU";
  c.fs = 300;
  c.seconds = 60;
  c.x = [](double t) { return std::sin(2 * kPiT * 41 * t); };
  c.params = {{"INS_GYRO_FILTER", 20}};
  const log::LogData ok = gyroLog(c);
  CHECK(!noiseOf(ok, ok.params, core::Family::Copter)["aliasRisk"].asBool());   // 300 Hz after a 20 Hz filter is alias-free
}

TEST(integrity_peak_is_a_true_local_maximum) {
  Noise g(16);
  const double fs = 200;
  const std::size_t n = 200 * 120;
  Vec red(n);
  double s = 0;
  const double a = std::exp(-2 * kPiT * 0.3 / fs);
  for (std::size_t i = 0; i < n; i++) {
    s = a * s + (1 - a) * 400 * g();
    red[i] = s;
  }
  const Json d = engine::diagnostics(rateData(fs, red), Json::object(), core::Family::Copter, core::Axis::Roll);
  CHECK(d["oscillating"].isBool() && !d["oscillating"].asBool());
  CHECK(d["peakProminence"].numOr(1e9) < 10);   // red noise has no prominent peak
  Vec osc(n);
  for (std::size_t i = 0; i < n; i++) osc[i] = red[i] + 8 * std::sin(2 * kPiT * 6 * static_cast<double>(i) / fs);
  const Json d2 = engine::diagnostics(rateData(fs, osc), Json::object(), core::Family::Copter, core::Axis::Roll);
  CHECK(d2["oscillating"].asBool());
  CHECK_NEAR(d2["peakHz"].numOr(0), 6, fs / 256);
  core::Psd falling;
  falling.f = {0, 1, 2, 3, 4, 5};
  falling.psd = {6, 5, 4, 3, 2, 1};
  CHECK(!engine::localPeak(falling, 0.5, 5));   // a falling spectrum has no peak
}

TEST(integrity_native_rate_no_content_above_nyquist) {
  Noise g(18);
  const double sigma = 10;
  HoverOpts o;
  o.seconds = 60;
  o.flyFrom = 1;
  o.flyTo = 59;
  o.signal = [&g, sigma](double, bool) { return test::RateSample{0, sigma * g(), 0}; };
  const std::vector<std::uint8_t> buf = test::hoverLog(o);
  log::LogData lg = log::parseBin(buf.data(), buf.size(), engine::logWant());
  log::cleanLogTimes(lg, engine::logTimed());
  engine::Window win;
  win.tStart = 2;
  win.tEnd = 58;
  const auto d = engine::axisData(lg, core::Family::Copter, core::Axis::Roll, win);
  CHECK(d.has_value());
  if (!d) return;
  CHECK_NEAR(d->fs, 300, 0.01);   // grid at the logged 300 Hz, not 400 Hz
  const core::Blocks blocks = engine::blocksOf(engine::usableMask(*d), d->n);
  const core::Psd sp = core::psd(d->actual, d->fs, 512, 0.5, &blocks);
  CHECK_NEAR(sp.f.back(), d->fs / 2, 1e-9);   // the top bin is the Nyquist frequency of the data
  double m = 0, c = 0;
  for (std::size_t k = 0; k < sp.f.size(); k++)
    if (sp.f[k] >= 10 && sp.f[k] <= 140) {
      m += sp.psd[k];
      c++;
    }
  const double white = 2 * sigma * sigma / d->fs;
  CHECK_NEAR(m / c, white, 0.1 * white);   // white-noise level 2 sigma^2 / fs
  // band RMS 20 Hz .. 0.49 fs as the D-noise model measures it (the audit before the fix: 7.33 for 9.31)
  std::size_t longest = 0;
  for (const auto& [b0, b1] : blocks) longest = std::max(longest, b1 - b0);
  std::size_t nw = std::max<std::size_t>(64, std::min<std::size_t>(1024, core::nextPow2(d->fs)));
  nw = std::min(nw, std::max<std::size_t>(64, core::prevPow2(static_cast<double>(longest))));
  const core::Psd spN = core::psd(d->actual, d->fs, nw, 0.5, &blocks);
  CHECK_NEAR(core::bandRms(spN, std::min(20.0, d->fs * 0.25), d->fs * 0.49), sigma * std::sqrt((0.49 * d->fs - 20) / (d->fs / 2)),
             0.05 * sigma);
}

// ------------------------------------------------------------------------------------------- fit quality, step metrics, invalid samples (F-59, F-63)

TEST(integrity_fit_quality_independent_of_coherence) {
  const core::Plant truth{2000, 1, 0.05, 0.012};
  Vec f(64);
  for (std::size_t i = 0; i < 64; i++) f[i] = static_cast<double>(i + 1) * 0.390625;
  const core::CVec Gt = core::evalPlant(f, truth);
  core::CVec H(64);
  for (std::size_t i = 0; i < 64; i++) {
    const double m = i % 2 ? 1.25 : 1 / 1.25, p = (i % 3 == 0 ? 15 : -15) * kPiT / 180;
    H.re[i] = (Gt.re[i] * std::cos(p) - Gt.im[i] * std::sin(p)) * m;
    H.im[i] = (Gt.re[i] * std::sin(p) + Gt.im[i] * std::cos(p)) * m;
  }
  core::FitPlantOptions fo;
  fo.kind = "copter";
  fo.nWin = 90;
  fo.fMin = 0.3;
  fo.fMax = 25;
  std::vector<core::PlantFit> q;
  for (double c : {1.0, 0.8, 0.6, 0.5}) {
    const auto fit = core::fitPlant(f, H, Vec(64, c), fo);
    CHECK(fit.has_value());
    if (fit) q.push_back(*fit);
  }
  if (q.size() != 4) return;
  for (const auto& fit : q) {
    CHECK_NEAR(fit.fitQuality, q[0].fitQuality, 1e-9);                  // quality independent of coherence
    CHECK_NEAR(fit.fitQuality, 1 / (1 + fit.misfit / 0.3), 1e-12);      // quality from the weighted mean misfit
  }
  CHECK_NEAR(q[0].fitQuality, 0.632, 0.002);   // the audit's value at coherence 1
}

TEST(integrity_step_metrics_not_settled_and_5pct_band) {
  const auto first = [](double tau, double dur, double fs) {
    std::pair<Vec, Vec> r;
    for (long i = 0; i < static_cast<long>(std::ceil(dur * fs)); i++) {
      r.first.push_back(static_cast<double>(i) / fs);
      r.second.push_back(1 - std::exp(-static_cast<double>(i) / fs / tau));
    }
    return r;
  };
  auto r = first(0.03, 0.5, 400);
  auto m = core::stepMetrics(r.first, r.second);
  CHECK(m && m->settled);
  if (m) {
    CHECK_NEAR(m->overshootPct, 0, 0.01);
    CHECK_NEAR(m->riseTime, 0.03 * std::log(9), 0.0025);       // rise 10-90 % = tau ln 9
    CHECK_NEAR(m->settlingTime, 0.03 * std::log(20), 0.0025);  // 5 % settling = tau ln 20
  }
  for (double tau : {0.15, 0.25}) {
    r = first(tau, 0.5, 400);
    m = core::stepMetrics(r.first, r.second);
    CHECK(m && !m->settled);   // does not settle in 0.5 s
    if (m) CHECK(std::isnan(m->overshootPct) && std::isnan(m->riseTime));
  }
  const double z = 0.5, wn = 2 * kPiT * 4, wd = wn * std::sqrt(1 - z * z);
  Vec t, y;
  for (int i = 0; i < 400; i++) {
    const double tt = i / 400.0;
    t.push_back(tt);
    y.push_back(1 - std::exp(-z * wn * tt) * std::sin(wd * tt + std::acos(z)) / std::sqrt(1 - z * z));
  }
  m = core::stepMetrics(t, y);
  CHECK(m && m->settled);
  if (m) CHECK_NEAR(m->overshootPct, 100 * std::exp(-z * kPiT / std::sqrt(1 - z * z)), 0.05);   // second-order overshoot
  m = core::stepMetrics({0, 0.1, 0.2, 0.3}, {0, 0, 0, 0});
  CHECK(m && !m->settled && m->finalValue == 0);
}

TEST(integrity_nan_never_reaches_a_result) {
  Noise g(8);
  const double fs = 200;
  const std::size_t n = 200 * 60;
  Vec target(n), actual(n);
  for (std::size_t i = 0; i < n; i++) {
    target[i] = 20 * std::sin(2 * kPiT * 0.5 * static_cast<double>(i) / fs);
    actual[i] = target[i] + g();
  }
  engine::AxisData cd = rateData(fs, actual);
  cd.target = target;
  const Json clean = engine::diagnostics(cd, Json::object(), core::Family::Copter, core::Axis::Roll);
  cd.actual[5000] = std::nan("");
  const Json d = engine::diagnostics(cd, Json::object(), core::Family::Copter, core::Axis::Roll);
  CHECK_NEAR(d["rmsErrorDegs"].numOr(0), clean["rmsErrorDegs"].numOr(1e9), 0.01 * clean["rmsErrorDegs"].numOr(0));
  CHECK(d["grade"]["letter"].strOr("?") == clean["grade"]["letter"].strOr("!"));
  CHECK(std::isfinite(d["oscRmsDegs"].numOr(std::nan(""))) && std::isfinite(d["noiseRmsDegs"].numOr(std::nan(""))));
  // full analysis on the independent simulator's data (check14 3b): one NaN must not change the proposal
  const double DEG = 180 / kPiT;
  test::SimOpts so;
  so.plant = {280, 2, 0.025, 0.003};
  so.gains = {0.135, 0.135, 0.0036, 0, 20, 0, 20, 0.5};
  so.gyroHz = 40;
  so.fsLoop = 400;
  so.dur = 60;
  so.mode = "angle";
  so.angleP = 4.5;
  so.inputTc = 0.15;
  test::SimPilot pil;
  pil.seed = 11;
  pil.angAmp = {4, 12};
  pil.rateAmp = {20, 60};
  pil.hold = {0.3, 1.0};
  pil.gap = {0.3, 1.2};
  so.pilot = pil;
  so.gyroNoise = 1 / DEG;
  so.seed = 3;
  const test::SimOut s = test::simulate(so);
  const std::size_t m = s.target.size();
  const auto mk = [&]() {
    engine::AxisData a;
    a.fs = 400;
    a.t0 = 0;
    a.n = m;
    a.target.resize(m);
    a.actual.resize(m);
    a.u.resize(m);
    for (std::size_t i = 0; i < m; i++) {
      a.target[i] = s.target[i] * DEG;
      a.actual[i] = s.actual[i] * DEG;
      a.u[i] = s.P[i] + s.I[i] + s.D[i] + s.FF[i];
    }
    a.P = s.P;
    a.I = s.I;
    a.D = s.D;
    a.FF = s.FF;
    a.srate = Vec(m, 0.0);
    a.dmod = Vec(m, 1.0);
    return a;
  };
  const log::Params params = paramsOf({{"SCHED_LOOP_RATE", 400}, {"INS_GYRO_RATE", 0}, {"INS_GYRO_FILTER", 40}, {"ATC_RAT_RLL_P", 0.135},
                                       {"ATC_RAT_RLL_I", 0.135}, {"ATC_RAT_RLL_D", 0.0036}, {"ATC_RAT_RLL_FF", 0}, {"ATC_RAT_RLL_FLTT", 20},
                                       {"ATC_RAT_RLL_FLTE", 0}, {"ATC_RAT_RLL_FLTD", 20}, {"ATC_RAT_RLL_SMAX", 50}, {"ATC_RAT_RLL_IMAX", 0.5},
                                       {"ATC_RAT_RLL_D_FF", 0}, {"ATC_ANG_RLL_P", 4.5}});
  const engine::AxisData good = mk();
  engine::AxisData bad = mk();
  bad.actual[5000] = std::nan("");
  bad.u[7000] = kInf;
  engine::AnalyzeArgs a0;
  a0.family = core::Family::Copter;
  a0.axis = core::Axis::Roll;
  a0.data = &good;
  a0.params = &params;
  engine::AnalyzeArgs a1 = a0;
  a1.data = &bad;
  const Json r0 = engine::analyzeAxis(a0), r1 = engine::analyzeAxis(a1);
  CHECK(r0["identification"]["ok"].asBool() && r1["identification"]["ok"].asBool());
  const double p0 = r0["proposal"]["gains"]["P"].numOr(0), p1 = r1["proposal"]["gains"]["P"].numOr(-1);
  CHECK(p0 > 0);
  CHECK_NEAR(p1, p0, 0.01 * p0);
  CHECK(std::isfinite(r1["diagnostics"]["rmsErrorDegs"].numOr(std::nan(""))));
  // a gyro spectrum with one NaN sample still finds the 80 Hz tone
  GyroLogOpts go;
  go.fs = 1000;
  go.seconds = 60;
  go.x = [&g](double t) { return 5 * std::sin(2 * kPiT * 80 * t) + 0.05 * g(); };
  log::LogData lg = gyroLog(go);
  lg.msgs["GYR"].num["GyrX"][30000] = std::nan("");
  const Json r = noiseOf(lg, log::Params{}, core::Family::Copter);
  bool tone = false;
  for (const auto& p : r["peaks"].arr()) tone = tone || std::fabs(p["f"].numOr(0) - 80) < 1.5;
  CHECK(tone);
}

// ------------------------------------------------------------------------------------------- refusals (F-79)

TEST(integrity_not_a_dataflash_log_is_refused) {
  const Json e = analyze({});
  CHECK(e["error"].strOr("") == "The file is empty.");
  CHECK(e["axes"]["roll"]["error"].isString() && e["axes"]["pitch"]["error"].isString() && e["axes"]["yaw"]["error"].isString());
  CHECK(e["noise"]["available"].isBool() && !e["noise"]["available"].asBool());
  CHECK(e["family"].isString() && core::parseFamily(e["family"].str()).has_value());   // the app reads the family of every result
  CHECK(analyze(tlogLike(4000, 1, 4000, 0xfe))["format"]["kind"].strOr("") == "tlog");
  // the command-line exit code (3) is checked by the ctest analyze_refuses_non_log
}
