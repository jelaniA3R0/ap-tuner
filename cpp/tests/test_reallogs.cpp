// The owner's tandem QuadPlane hover logs (ArduPlane 4.7.1, 2026-10-15) and the audit's refusal cases made from
// them. Skipped unless APTUNER_LOG_DIR (environment, or the CMake option of the same name) names the folder that
// holds them (the logs are not published with the source).
// Expected values are computed here from the raw records, independently of the analysis pipeline.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/log/dfreader.hpp"
#include "aptuner/log/integrity.hpp"
#include "check.hpp"

using namespace aptuner;
using aptuner::core::Json;
using aptuner::core::Vec;

namespace {

std::string logPath(const std::string& name) {
  // the folder with the tandem QuadPlane logs: APTUNER_LOG_DIR (CMake option APTUNER_LOG_DIR); unset: the tests skip
  const char* d = std::getenv("APTUNER_LOG_DIR");
  if (!d || !*d) return "";
  const std::string dir = d;
  return dir + "/2026-10-15 " + name + ".bin";
}
bool present(const std::string& p) {
  if (p.empty()) {
    std::printf("        (skipped: set APTUNER_LOG_DIR to the folder with the tandem logs)\n");
    return false;
  }
  std::ifstream f(p, std::ios::binary);
  if (f.good()) return true;
  std::printf("        (skipped: %s not found)\n", p.c_str());
  return false;
}
bool anyContains(const Json& arr, const std::string& what) {
  if (!arr.isArray()) return false;
  for (const auto& s : arr.arr())
    if (s.isString() && s.str().find(what) != std::string::npos) return true;
  return false;
}
Json analyzeBuf(const std::vector<std::uint8_t>& buf, std::size_t n) {
  log::LogData lg = log::parseBin(buf.data(), n, engine::logWant());
  return engine::analyzeLog(lg, "test.bin");
}
Json analyzeBuf(const std::vector<std::uint8_t>& buf) { return analyzeBuf(buf, buf.size()); }
const char* const kAxisNames[] = {"roll", "pitch", "yaw"};

/** independent airborne stretches from QTUN.ThO: above 60% of the median hover throttle in the window (>= 0.15), 3 s
 * after lift-off and 2 s before touch-down left out (not at a gap), a sample held for 0.5 s at most */
std::vector<std::pair<double, double>> airborne(const log::MsgTable& q, double tA, double tB) {
  const std::size_t n = q.n;
  Vec t(n);
  for (std::size_t i = 0; i < n; i++) t[i] = (*q.col("TimeUS"))[i] / 1e6;
  const Vec& v = *q.col("ThO");
  Vec hov;
  for (std::size_t i = 0; i < n; i++)
    if (t[i] >= tA && t[i] <= tB && v[i] > 0.15) hov.push_back(v[i]);
  std::sort(hov.begin(), hov.end());
  const double lim = std::max(0.15, 0.6 * hov[hov.size() / 2]);
  Vec d;
  for (std::size_t i = 1; i < n; i++) d.push_back(t[i] - t[i - 1]);
  std::sort(d.begin(), d.end());
  const double hold = std::max(0.5, 3 * d[d.size() / 2]);
  std::vector<std::pair<double, double>> out;
  std::size_t i = 0;
  bool afterGap = false;
  while (i < n) {
    if (!(v[i] > lim)) {
      i++;
      afterGap = false;
      continue;
    }
    std::size_t j = i;
    while (j + 1 < n && v[j + 1] > lim && t[j + 1] - t[j] <= hold) j++;
    const bool atGap = j + 1 < n && v[j + 1] > lim;
    const double a = t[i] + (afterGap ? 0 : 3);
    const double b = (j + 1 < n ? std::min(t[j + 1], t[j] + hold) : t[j] + hold) - (atGap ? 0 : 2);
    if (b > a) out.emplace_back(a, b);
    afterGap = atGap;
    i = j + 1;
  }
  return out;
}

}  // namespace

TEST(reallog_0420_gap_reported_and_left_out) {
  const std::string p = logPath("04-20-29");
  if (!present(p)) return;
  const Json r = engine::analyzeLogFile(p);
  const log::LogData raw = log::parseBinFile(p, {"RATE", "QTUN"});
  const double tS = r["flightWindow"]["tStart"].numOr(0), tE = r["flightWindow"]["tEnd"].numOr(0);
  CHECK_NEAR(tS, 157.5, 0.1);
  CHECK_NEAR(tE, 320.2, 0.1);
  const log::MsgTable* R = raw.msg("RATE");
  const log::MsgTable* Q = raw.msg("QTUN");
  CHECK(R && Q);
  if (!R || !Q) return;
  // the largest step between RATE records inside the window, from the raw records
  Vec t;
  for (double v : *R->col("TimeUS"))
    if (v / 1e6 >= tS && v / 1e6 <= tE) t.push_back(v / 1e6);
  double g0 = 0, g1 = 0;
  for (std::size_t i = 1; i < t.size(); i++)
    if (t[i] - t[i - 1] > g1 - g0) {
      g0 = t[i - 1];
      g1 = t[i];
    }
  CHECK_NEAR(g1 - g0, 51.775, 0.001);
  for (const char* axis : kAxisNames) {
    const Json& a = r["axes"][axis];
    const Json& di = a["dataInfo"];
    CHECK(di["gapCount"].numOr(-1) == 1);
    CHECK_NEAR(di["gaps"][0]["t0"].numOr(0), g0, 1e-9);
    CHECK_NEAR(di["gaps"][0]["t1"].numOr(0), g1, 1e-9);
    CHECK(anyContains(a["warnings"], "Logging stopped for 51.8 s (230.5-282.3 s)"));
    bool outside = true;
    for (const auto& v : a["series"]["t"].arr()) outside = outside && (v.num() <= g0 || v.num() >= g1);
    CHECK(outside);
    CHECK_NEAR(di["fs"].numOr(0), 300.03, 0.01);   // native 300 Hz (the old code used a 200 Hz grid)
  }
  // roll RMS error over the airborne RATE records, computed from the raw records
  const auto iv = airborne(*Q, tS, tE);
  const Vec &tu = *R->col("TimeUS"), &des = *R->col("RDes"), &act = *R->col("R");
  double e2 = 0, c = 0;
  for (std::size_t i = 0; i < R->n; i++) {
    const double ts = tu[i] / 1e6;
    if (ts < tS || ts > tE) continue;
    if (!std::any_of(iv.begin(), iv.end(), [&](const std::pair<double, double>& x) { return ts >= x.first && ts < x.second; })) continue;
    const double e = des[i] - act[i];
    e2 += e * e;
    c++;
  }
  const Json& roll = r["axes"]["roll"];
  CHECK_NEAR(roll["diagnostics"]["rmsErrorDegs"].numOr(0), std::sqrt(e2 / c), 1e-9);
  CHECK_NEAR(roll["dataInfo"]["usableSeconds"].numOr(0), c / roll["dataInfo"]["fs"].numOr(1), 1e-9);
  CHECK(roll["dataInfo"]["usableSeconds"].numOr(1e9) < 0.3 * (tE - tS));   // only 27% of the window flown
  CHECK(r["paramChanges"].numOr(-1) == 0);   // no tuning parameter changed (the 65 changes are STAT_* and baro values)
}

TEST(reallog_0433_dropout_cut_in_memory) {
  const std::string p = logPath("04-33-02");
  if (!present(p)) return;
  const std::vector<std::uint8_t> buf = log::readFile(p);
  const log::LogData head = log::parseBin(buf.data(), buf.size(), {"__none__"});
  // the records stamped 160-166 s are cut out: a 6 s logging dropout
  std::vector<std::uint8_t> cut;
  std::size_t i = 0, from = 0;
  while (i + 3 <= buf.size()) {
    const log::Fmt* fm = nullptr;
    if (buf[i] == 0xa3 && buf[i + 1] == 0x95) {
      const auto it = head.fmts.find(buf[i + 2]);
      if (it != head.fmts.end() && it->second.len > 0) fm = &it->second;
    }
    if (!fm) {
      i++;
      continue;
    }
    std::optional<double> t;
    if (!fm->labels.empty() && fm->labels[0] == "TimeUS" && !fm->format.empty() && fm->format[0] == 'Q' && i + 11 <= buf.size()) {
      std::uint64_t us = 0;
      std::memcpy(&us, buf.data() + i + 3, 8);
      t = static_cast<double>(us) / 1e6;
    }
    if (t && *t >= 160 && *t <= 166) {
      cut.insert(cut.end(), buf.begin() + static_cast<std::ptrdiff_t>(from), buf.begin() + static_cast<std::ptrdiff_t>(i));
      from = i + static_cast<std::size_t>(fm->len);
    }
    i += static_cast<std::size_t>(fm->len);
  }
  cut.insert(cut.end(), buf.begin() + static_cast<std::ptrdiff_t>(std::min(from, buf.size())), buf.end());
  const Json a = analyzeBuf(buf), b = analyzeBuf(cut);
  const Json &pa = a["axes"]["pitch"], &pb = b["axes"]["pitch"];
  CHECK(pa["dataInfo"]["gapCount"].numOr(-1) == 0);
  CHECK(pb["dataInfo"]["gapCount"].numOr(-1) == 1);
  CHECK_NEAR(pb["dataInfo"]["gaps"][0]["s"].numOr(0), 6, 0.01);
  CHECK(anyContains(pb["warnings"], "Logging stopped for 6.0 s"));
  CHECK_NEAR(pa["dataInfo"]["usableSeconds"].numOr(0) - pb["dataInfo"]["usableSeconds"].numOr(0), 6, 0.2);   // only the 6 s are lost
  // the same proposal (the audit before the fix: D 0.0054 -> 0.00492)
  const Json &ca = pa["proposal"]["changes"], &cb = pb["proposal"]["changes"];
  CHECK(ca.isArray() && cb.isArray() && ca.size() == cb.size());
  if (ca.isArray() && cb.isArray() && ca.size() == cb.size())
    for (std::size_t k = 0; k < ca.size(); k++) {
      CHECK(ca[k]["key"].strOr("a") == cb[k]["key"].strOr("b"));
      CHECK(core::jsRoundPrecision(ca[k]["to"].numOr(0), 4) == core::jsRoundPrecision(cb[k]["to"].numOr(1), 4));
    }
}

TEST(reallog_0443_random_byte_errors) {
  const std::string p = logPath("04-43-00");
  if (!present(p)) return;
  std::vector<std::uint8_t> buf = log::readFile(p);
  std::uint32_t s = 5;
  const auto rnd = [&]() {
    s = s * 1664525u + 1013904223u;
    return s / 4294967296.0;
  };
  for (int k = 0; k < 2000; k++) {
    const std::size_t pos = 200000 + static_cast<std::size_t>(std::floor(rnd() * static_cast<double>(buf.size() - 200000)));
    buf[pos] = static_cast<std::uint8_t>(std::floor(rnd() * 256));
  }
  const Json r = analyzeBuf(buf);
  CHECK(r["integrity"]["skippedBytes"].numOr(0) > 0 && r["integrity"]["resyncs"].numOr(0) > 0);   // damage counted
  CHECK(anyContains(r["integrity"]["notes"], "could not be read"));                                // and reported
  log::LogData lg = log::parseBin(buf.data(), buf.size(), {"RATE"});
  log::cleanLogTimes(lg, {"RATE"});
  const Vec& t = *lg.msg("RATE")->col("TimeUS");
  bool increasing = true;
  for (std::size_t i = 1; i < t.size(); i++) increasing = increasing && t[i] > t[i - 1];
  CHECK(increasing);   // RATE stamps increase after cleaning
  for (const char* axis : kAxisNames) CHECK(std::isfinite(r["axes"][axis]["diagnostics"]["rmsErrorDegs"].numOr(std::nan(""))));
}

TEST(reallog_refusal_cases) {
  const std::string p = logPath("04-43-00");
  if (!present(p)) return;
  const std::vector<std::uint8_t> src = log::readFile(p);
  // 4 MB zeroed in the middle
  std::vector<std::uint8_t> z = src;
  std::fill(z.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(z.size(), 4 * 1048576)),
            z.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(z.size(), 8 * 1048576)), 0);
  const Json rz = analyzeBuf(z);
  CHECK(rz["integrity"]["skippedBytes"].numOr(0) >= 4 * 1048576 - 200);   // the zeroed bytes are counted
  CHECK(anyContains(rz["integrity"]["notes"], "4.0 MB of the file could not be read"));
  // the blanked bytes are a hole in time; it must be a reported gap or lie outside the analysed window
  const log::LogData zr = log::parseBin(z.data(), z.size(), {"RATE"});
  const Vec& t = *zr.msg("RATE")->col("TimeUS");
  double h0 = 0, h1 = 0;
  for (std::size_t i = 1; i < t.size(); i++)
    if (t[i] - t[i - 1] > (h1 - h0) * 1e6) {
      h0 = t[i - 1] / 1e6;
      h1 = t[i] / 1e6;
    }
  CHECK(h1 - h0 > 10);
  const bool overlaps = rz["flightWindow"]["tStart"].numOr(0) < h1 && rz["flightWindow"]["tEnd"].numOr(0) > h0;
  for (const char* axis : kAxisNames) {
    const Json& a = rz["axes"][axis];
    bool ok = a.has("error");
    if (!ok && overlaps)
      for (const auto& g : a["dataInfo"]["gaps"].arr()) ok = ok || std::fabs(g["t0"].numOr(0) - h0) < 1e-6;
    if (!ok && !overlaps) ok = a["dataInfo"]["gapCount"].numOr(-1) == 0;
    CHECK(ok);   // the hole is a gap or outside the window, never filled in
  }
  // truncated mid-record
  const Json rt = analyzeBuf(src, std::min<std::size_t>(src.size(), 6700001));
  CHECK(rt["integrity"]["truncatedBytes"].numOr(0) > 0 && anyContains(rt["integrity"]["notes"], "ends in the middle of a record"));
  // formats and parameters only, no flight
  const Json rn = analyzeBuf(src, std::min<std::size_t>(src.size(), 65536));
  for (const char* axis : kAxisNames) CHECK(rn["axes"][axis].has("error"));
  // a telemetry log and an empty file
  std::vector<std::uint8_t> tl(200000, 0);
  for (std::size_t k = 0; k < 2000; k++) {
    const std::uint64_t us = 1760000000000000ull + 1000ull * k;
    for (int i = 0; i < 8; i++) tl[k * 100 + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(us >> (56 - 8 * i));
    tl[k * 100 + 8] = 0xfd;
  }
  CHECK(analyzeBuf(tl)["format"]["kind"].strOr("") == "tlog");
  CHECK(analyzeBuf({})["error"].strOr("") == "The file is empty.");
}
