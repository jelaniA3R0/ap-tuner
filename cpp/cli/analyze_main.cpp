// aptuner-analyze: command-line log analysis (C++ counterpart of tools/analyze-log.js).
//   aptuner-analyze <log.bin> [conservative|normal|aggressive] [copter|vtol|plane|heli] [--json out.json] [--param out.param]
// Exit code 3 when the file is not a DataFlash log the analysis can read (the --json result is still written).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "aptuner/core/build_info.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/report.hpp"

using aptuner::core::Json;
using aptuner::core::jsToFixed;
using aptuner::core::jsToString;

namespace {

void usage() {
  std::puts("usage: aptuner-analyze <log.bin> [conservative|normal|aggressive] [copter|vtol|plane|heli]");
  std::puts("                       [--json result.json] [--param proposal.param]");
  std::puts("       aptuner-analyze --version");
}

/** JS template text of a value: numbers as JS prints them, missing -> "undefined" */
std::string s(const Json& v) {
  if (v.isNumber()) return jsToString(v.num());
  if (v.isString()) return v.str();
  if (v.isBool()) return v.asBool() ? "true" : "false";
  if (v.isNull()) return "null";
  return v.dump();
}
/** o.key in a JS template: a missing key prints "undefined", null "null" */
std::string sk(const Json& o, const char* key) { return o.has(key) ? s(o[key]) : "undefined"; }
/** v?.toFixed(d) */
std::string f(const Json& v, int d) { return v.isNumber() ? jsToFixed(v.num(), d) : "undefined"; }
std::string fk(const Json& v, double k, int d) { return v.isNumber() ? jsToFixed(v.num() * k, d) : "NaN"; }
/** v ?? alt */
std::string nn(const Json& v, const std::string& alt) { return v.isNull() ? alt : s(v); }
std::string fmt4(const Json& v) { return v.isNumber() ? jsToString(aptuner::core::jsRoundPrecision(v.num(), 4)) : s(v); }
/** JavaScript truthiness of a value */
bool truthy(const Json& v) {
  if (v.isNumber()) return v.num() != 0 && !std::isnan(v.num());
  if (v.isString()) return !v.str().empty();
  if (v.isBool()) return v.asBool();
  return v.isArray() || v.isObject();
}
/** a step-metrics object that says the response has not settled (settled === false) */
bool notSettled(const Json& m) { return m["settled"].isBool() && !m["settled"].asBool(); }
std::string stepTxt(const Json& m) {
  return notSettled(m) ? "not settled" : "rise " + fk(m["riseTime"], 1000, 0) + " ms / overshoot " + f(m["overshootPct"], 0) + "%";
}

void printResult(const Json& r, long long ms) {
  const Json& fw = r["flightWindow"];
  std::printf("%s\n%s | family %s | flight window %s-%s s | tuning-parameter changes in log: %s (all parameter changes: %s) | %lld ms\n",
              s(r["file"]).c_str(), r.has("firmware") ? s(r["firmware"]).c_str() : "undefined", s(r["family"]).c_str(),
              f(fw["tStart"], 0).c_str(), f(fw["tEnd"], 0).c_str(), s(r["paramChanges"]).c_str(), s(r["paramChangesAll"]).c_str(), ms);
  const Json& ig = r["integrity"];
  std::string ti;
  for (const auto& [k, v] : ig["timeIssues"].items()) {
    if (!ti.empty()) ti += "; ";
    ti += k + " out-of-order " + s(v["outOfOrder"]) + " repeated " + s(v["duplicates"]);
    if (v["reset"].isObject()) ti += " reset " + f(v["reset"]["fromS"], 1) + "->" + f(v["reset"]["toS"], 1) + " s";
  }
  std::printf("file: %s bytes, skipped %s (re-synchronised %sx), truncated tail %s, padding %s, invalid FMT %s, FMT conflicts %s; "
              "time-stamp problems: %s\n",
              s(ig["bytes"]).c_str(), s(ig["skippedBytes"]).c_str(), s(ig["resyncs"]).c_str(), s(ig["truncatedBytes"]).c_str(),
              s(ig["paddingBytes"]).c_str(), s(ig["badFmtCount"]).c_str(), s(ig["fmtConflicts"]).c_str(),
              ti.empty() ? "none" : ti.c_str());
  for (const auto& n : ig["notes"].arr()) std::printf("   * %s\n", s(n).c_str());
  for (const auto& [axis, a] : r["axes"].items()) {
    if (a.has("error")) {
      std::printf("\n== %s: %s\n", axis.c_str(), s(a["error"]).c_str());
      continue;
    }
    const Json &d = a["diagnostics"], &c = a["current"], &di = a["dataInfo"];
    std::printf("\n== %s  data %s s (%s s usable) of %s @ %s Hz (median step %s Hz) in %s unbroken piece(s), longest %s s, %s gap(s); "
                "masked: %s non-finite, %s impossible; scaler2=%s thr=%s\n",
                axis.c_str(), f(di["seconds"], 0).c_str(), f(di["usableSeconds"], 0).c_str(), s(di["source"]).c_str(),
                s(di["fs"]).c_str(), f(di["rawRateHz"], 1).c_str(), s(di["segments"]).c_str(), f(di["longestSegmentS"], 1).c_str(),
                s(di["gapCount"]).c_str(), s(di["excluded"]["nonFinite"]).c_str(), s(di["excluded"]["implausible"]).c_str(),
                f(di["scaler2"], 2).c_str(), jsToFixed(di["throttle"].numOr(0), 2).c_str());
    std::printf("   current: P=%s I=%s D=%s FF=%s FLTT=%s FLTE=%s FLTD=%s SMAX=%s angleP=%s tconst=%s\n", s(c["P"]).c_str(),
                s(c["I"]).c_str(), s(c["D"]).c_str(), s(c["FF"]).c_str(), s(c["FLTT"]).c_str(), s(c["FLTE"]).c_str(),
                s(c["FLTD"]).c_str(), s(c["SMAX"]).c_str(), nn(c["angleP"], "-").c_str(), nn(c["tconst"], "-").c_str());
    const Json& g = d["grade"];
    const std::string peak = d.has("peakHz") && d["peakHz"].isNull() ? std::string("none")
                                                                      : f(d["peakHz"], 1) + "Hz x" + f(d["peakProminence"], 0);
    std::printf("   diagnostics: grade %s (%s) tracking %s%% rmsErr %s osc %s peak %s%s noise %s D/P %s sat %s%% srateMax %s dmodMin %s\n",
                s(g["letter"]).c_str(), nn(g["score"], "not gradable").c_str(), f(d["trackingPct"], 0).c_str(),
                f(d["rmsErrorDegs"], 2).c_str(), f(d["oscRmsDegs"], 2).c_str(), peak.c_str(),
                truthy(d["aliasRisk"]) ? " (aliasing possible)" : "", f(d["noiseRmsDegs"], 2).c_str(), f(d["dOverP"], 2).c_str(),
                f(d["saturationPct"], 1).c_str(), f(d["srateMax"], 1).c_str(), f(d["dmodMin"], 2).c_str());
    for (const auto& n : g["notes"].arr()) std::printf("     - %s\n", s(n).c_str());
    if (a.has("measuredStep")) {
      const Json& m = a["measuredStep"]["metrics"];
      if (notSettled(m))
        std::printf("   measured step (%s windows): not settled within the window (final %s)\n", s(a["measuredStep"]["count"]).c_str(),
                    m["finalValue"].isNull() ? "-" : f(m["finalValue"], 2).c_str());
      else
        std::printf("   measured step (%s windows): rise %s ms, overshoot %s%%, settle %s ms, final %s\n",
                    s(a["measuredStep"]["count"]).c_str(), fk(m["riseTime"], 1000, 0).c_str(), f(m["overshootPct"], 0).c_str(),
                    fk(m["settlingTime"], 1000, 0).c_str(), f(m["finalValue"], 2).c_str());
    }
    if (a.has("timeDomain")) {
      const Json& t = a["timeDomain"];
      std::printf("   hover-data fit (%s): b=%s rad/s2 per unit (%s deg/s2) a=%s tau=%s ms Td=%s ms mixer limit %s (%s%% of time) "
                  "trim %s..%s (mean %s) r2 %s replay %s b +-%s%% [%s, %s s airborne in %s part(s)]\n",
                  t["ok"].asBool() ? "OK" : ("FAILED: " + sk(t, "reason")).c_str(), sk(t, "bRads2").c_str(),
                  sk(t, "bDegs2").c_str(), sk(t, "a").c_str(), fk(t["tau"], 1000, 0).c_str(), fk(t["Td"], 1000, 0).c_str(),
                  nn(t["uLimit"], "-").c_str(), sk(t, "limitPct").c_str(), sk(t, "trimMin").c_str(), sk(t, "trimMax").c_str(),
                  sk(t, "trimMean").c_str(), sk(t, "r2").c_str(), sk(t, "vaf").c_str(), fk(t["bRelErr"], 100, 0).c_str(),
                  sk(t, "method").c_str(), sk(t, "airborneS").c_str(), sk(t, "segments").c_str());
    }
    const Json& sat = d["saturation"];
    if (sat.isObject())
      std::printf("   saturation: %s | steady error %s deg/s (demand %s, actual %s) | I mean %s p95 %s / IMAX %s | motors pinned %s%%%s\n",
                  sat["saturated"].asBool() ? "SATURATED" : sat["trimHeavy"].asBool() ? "trim-heavy" : "no",
                  f(sat["steadyErrorDegs"], 1).c_str(), f(sat["meanDemandDegs"], 1).c_str(), f(sat["meanActualDegs"], 1).c_str(),
                  f(sat["iMean"], 3).c_str(), f(sat["iP95"], 3).c_str(), s(sat["imax"]).c_str(), f(sat["pinnedPct"], 1).c_str(),
                  sat["ccwMean"].isNumber() ? (" | CCW " + f(sat["ccwMean"], 0) + " us vs CW " + f(sat["cwMean"], 0) + " us").c_str() : "");
    const Json& id = a["identification"];
    if (!id["ok"].asBool())
      std::printf("   identification: FAILED - %s\n", truthy(id["reason"]) ? s(id["reason"]).c_str() : "weak fit");
    else if (id["method"].strOr("").rfind("time", 0) == 0)
      std::printf("   model (%s, LOWER CONFIDENCE): K=%s a=%s design lag %s ms delay %s ms quality %s\n", s(id["method"]).c_str(),
                  f(id["fit"]["K"], 1).c_str(), f(id["fit"]["a"], 3).c_str(), fk(id["assumed"]["tau"], 1000, 0).c_str(),
                  fk(id["assumed"]["Td"], 1000, 0).c_str(), f(id["fit"]["fitQuality"], 2).c_str());
    else
      std::printf("   model: K=%s a=%s tau=%s ms Td=%s ms quality %s trusted to %s Hz (%s windows)\n", f(id["fit"]["K"], 1).c_str(),
                  f(id["fit"]["a"], 2).c_str(), fk(id["fit"]["tau"], 1000, 0).c_str(), fk(id["fit"]["Td"], 1000, 0).c_str(),
                  f(id["fit"]["fitQuality"], 2).c_str(), f(id["fCoh"], 1).c_str(), s(id["nWin"]).c_str());
    for (const auto& w : a["warnings"].arr()) std::printf("   ! %s\n", s(w).c_str());
    if (a["proposal"].isObject()) {
      const Json& p = a["proposal"];
      const Json& gg = p["gains"];
      std::printf("   PROPOSED: P=%s I=%s D=%s FF=%s FLTT=%s FLTE=%s FLTD=%s %s%s\n", f(gg["P"], 4).c_str(), f(gg["I"], 4).c_str(),
                  f(gg["D"], 5).c_str(), f(gg["FF"], 3).c_str(), s(gg["FLTT"]).c_str(), s(gg["FLTE"]).c_str(),
                  s(gg["FLTD"]).c_str(), truthy(gg["angleP"]) ? ("angleP=" + s(gg["angleP"])).c_str() : "",
                  truthy(gg["tconst"]) ? ("tconst=" + s(gg["tconst"])).c_str() : "");
      if (p["margins"].isObject())
        std::printf("   margins: GM %s dB, PM %s deg, crossover %s Hz, Ms %s | current: GM %s PM %s wc %s\n",
                    f(p["margins"]["gmDb"], 1).c_str(), f(p["margins"]["pmDeg"], 0).c_str(), f(p["margins"]["wcHz"], 1).c_str(),
                    f(p["margins"]["ms"], 2).c_str(), f(p["currentMargins"]["gmDb"], 1).c_str(),
                    f(p["currentMargins"]["pmDeg"], 0).c_str(), f(p["currentMargins"]["wcHz"], 1).c_str());
      if (p["predicted"].isObject()) {
        const Json &pm = p["predicted"]["proposedMetrics"], &cm = p["predicted"]["currentMetrics"];
        std::printf("   predicted step: current %s  ->  proposed %s\n", stepTxt(cm).c_str(), stepTxt(pm).c_str());
      }
      if (p["modelDesign"].isObject()) {
        const Json& md = p["modelDesign"];
        std::printf("   full model design (uncapped): P=%s I=%s D=%s angleP=%s crossover %s Hz\n", f(md["P"], 3).c_str(),
                    f(md["I"], 3).c_str(), f(md["D"], 4).c_str(), s(md["angleP"]).c_str(), f(md["crossoverHz"], 2).c_str());
      }
      std::string ch;
      for (const auto& c2 : p["changes"].arr()) {
        if (!ch.empty()) ch += ", ";
        ch += s(c2["key"]) + " " + fmt4(c2["from"]) + " -> " + fmt4(c2["to"]);
        if (c2["pct"].isNumber()) ch += std::string(" (") + (c2["pct"].num() > 0 ? "+" : "") + f(c2["pct"], 0) + "%)";
      }
      std::printf("   changes: %s\n", ch.c_str());
    }
  }
  const Json& nz = r["noise"];
  std::printf("\n== gyro noise / notch\n");
  if (!nz["available"].asBool()) {
    std::printf("   %s\n", s(nz["reason"]).c_str());
  } else {
    std::string pk;
    for (const auto& p : nz["peaks"].arr()) pk += (pk.empty() ? "" : " ") + s(p["f"]) + "Hz(x" + s(p["prominence"]) + ")";
    const Json& ni = nz["dataInfo"];
    std::printf("   source: %s; %s s in %s unbroken block(s)%s; peaks: %s\n", s(nz["source"]).c_str(), f(ni["seconds"], 0).c_str(),
                s(ni["blocks"]).c_str(), truthy(ni["airborneOnly"]) ? ", airborne only" : "", pk.c_str());
    if (nz["harmonics"].isObject()) {
      std::string m;
      for (const auto& x : nz["harmonics"]["members"].arr()) m += (m.empty() ? "" : " ") + s(x["k"]) + "x@" + s(x["f"]);
      std::printf("   harmonic family f0=%s Hz: %s\n", s(nz["harmonics"]["f0"]).c_str(), m.c_str());
    }
    if (nz.has("note")) std::printf("   %s\n", s(nz["note"]).c_str());
    std::printf("   control-band (5-40 Hz) noise RMS: pre %s deg/s, with current filters %s\n", s(nz["rmsBand"]["pre"]).c_str(),
                nn(nz["rmsBand"]["postCurrent"], "-").c_str());
    if (nz["recommended"].isObject()) {
      const Json& rc = nz["recommended"];
      std::printf("   RECOMMENDED: %s INS_GYRO_FILTER=%s\n", rc["params"].dump().c_str(), s(rc["gyroFilterHz"]).c_str());
      std::printf("   %s; predicted control-band noise %s deg/s; phase lag @10 Hz current %s deg -> proposed %s deg\n",
                  s(rc["why"]).c_str(), s(rc["rmsBandPost"]).c_str(), s(rc["phaseLag10Hz"]["current"]).c_str(),
                  s(rc["phaseLag10Hz"]["proposed"]).c_str());
      for (const auto& n : rc["notes"].arr()) std::printf("     - %s\n", s(n).c_str());
    }
    if (nz["consistency"].size()) {
      std::string cs;
      for (const auto& c : nz["consistency"].arr())
        cs += (cs.empty() ? "" : ", ") + f(c["f"], 1) + " Hz " + s(c["hits"]) + "/" + s(c["windows"]);
      std::printf("   peak repeatability: %s\n", cs.c_str());
    }
    if (nz.has("recommendation")) std::printf("   %s\n", s(nz["recommendation"]).c_str());
  }
  for (const auto& [axis, a] : r["axes"].items()) {
    if (!a["issues"].size()) continue;
    std::printf("\n== issues: %s\n", axis.c_str());
    for (const auto& i : a["issues"].arr())
      std::printf("   [%s] %s%s\n", s(i["level"]).c_str(), s(i["text"]).c_str(), i.has("fix") ? ("  -> " + s(i["fix"])).c_str() : "");
  }
  if (r["motors"].isObject())
    std::printf("\n== motors (flying): mean PWM %s, at upper limit (%s us) %s%% of the time, max %s\n",
                r["motors"]["meanPwm"].dump().c_str(), s(r["motors"]["topPwm"]).c_str(),
                f(r["motors"]["pinnedTopPct"], 1).c_str(), s(r["motors"]["maxPwm"]).c_str());
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> pos;
  std::string jsonOut, paramOut;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--version" || a == "-v") {
      std::puts(aptuner::core::buildInfoLine().c_str());
      return 0;
    }
    if (a == "--help" || a == "-h") {
      usage();
      return 0;
    }
    if (a == "--json" && i + 1 < argc) {
      jsonOut = argv[++i];
      continue;
    }
    if (a == "--param" && i + 1 < argc) {
      paramOut = argv[++i];
      continue;
    }
    pos.push_back(a);
  }
  if (pos.empty()) {
    usage();
    return 2;
  }
  aptuner::engine::AnalyzeLogOptions opt;
  if (pos.size() >= 2) opt.preset = pos[1];
  if (pos.size() >= 3) opt.family = aptuner::core::parseFamily(pos[2]);
  try {
    const auto t0 = std::chrono::steady_clock::now();
    const Json r = aptuner::engine::analyzeLogFile(pos[0], opt);
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    if (!jsonOut.empty()) {
      std::ofstream o(jsonOut, std::ios::binary);
      o << r.dump();
    }
    if (r.has("error")) {
      // not a DataFlash log the analysis can read: say why and stop
      std::printf("%s\nNOT ANALYSED (%s): %s\n", s(r["file"]).c_str(), s(r["format"]["kind"]).c_str(), s(r["error"]).c_str());
      return 3;
    }
    printResult(r, ms);
    if (!paramOut.empty()) {
      const auto fam = aptuner::core::parseFamily(r["family"].str());
      // checked, fixed point, the log's vehicle in the header (tools/compare-cpp.js compares it with the JavaScript text)
      const Json rows = aptuner::engine::proposalRows(*fam, r["axes"], r["noise"], r["params"]);
      std::ofstream o(paramOut, std::ios::binary);
      o << aptuner::engine::paramFile(rows, {"from " + pos[0]}, Json(aptuner::engine::logIdentityText(r)));
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "aptuner-analyze: %s\n", e.what());
    return 1;
  }
  return 0;
}
