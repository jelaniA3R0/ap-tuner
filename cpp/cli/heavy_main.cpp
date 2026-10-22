// aptuner-heavy: model-based tuning of heavy / slow vehicles from one or more logs of the same airframe.
//   aptuner-heavy <log.bin> [more.bin ...] [--plan plan.json] [--out folder] [--spacing metres] [--json result.json]
// Writes the step files (Step N.param, Step N UNDO.param) and plan.json (for "check my next flight") into --out.
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "aptuner/core/build_info.hpp"
#include "aptuner/engine/heavytune.hpp"

using aptuner::core::Json;
namespace heavy = aptuner::engine::heavy;
namespace fs = std::filesystem;

namespace {

std::string s(const Json& v) {
  if (v.isString()) return v.str();
  if (v.isNull()) return "-";
  return v.dump();
}

const Json::Array& arr(const Json& j) {
  static const Json::Array empty;
  return j.isArray() ? j.arr() : empty;
}

void writeText(const fs::path& p, const std::string& t) {
  std::ofstream o(p, std::ios::binary);
  o << t;
}

void print(const Json& r) {
  const Json& v = r["vehicle"];
  std::printf("vehicle: system id %s, %s, %s\n", s(v["sysid"]).c_str(), s(v["family"]).c_str(), s(v["firmware"]).c_str());
  for (const auto& f : arr(r["flights"])) std::printf("  flight %s (%s, rate data %s Hz)\n", s(f["name"]).c_str(), s(f["source"]).c_str(), s(f["rateHz"]).c_str());
  if (r.has("error")) std::printf("\nNOT ANALYSED: %s\n", r["error"].str().c_str());
  std::printf("\n== hovers\n");
  for (const auto& h : arr(r["hovers"]))
    std::printf("  %s #%s  lift-off %s s, %s s long, throttle %s, mode %s\n", s(h["flight"]).c_str(), s(h["index"]).c_str(), s(h["tLift"]).c_str(),
                s(h["seconds"]).c_str(), s(h["throttle"]).c_str(), s(h["mode"]).c_str());
  std::printf("\n== in plain words\n");
  for (const auto& x : arr(r["summary"])) std::printf("  %s\n", s(x).c_str());
  std::printf("\n== issues\n");
  for (const auto& i : arr(r["issues"])) {
    std::printf("  [%s]%s %s%s: %s\n", s(i["level"]).c_str(), i["cantTune"].isBool() && i["cantTune"].asBool() ? " (tuning can't fix)" : "",
                i["axis"].strOr("").empty() ? "" : (i["axis"].str() + " ").c_str(), s(i["title"]).c_str(), s(i["text"]).c_str());
    if (!i["fix"].strOr("").empty()) std::printf("        -> %s\n", s(i["fix"]).c_str());
  }
  std::printf("\n== identification\n");
  for (const char* a : {"yaw", "pitch", "roll"}) std::printf("  %s: %s\n", a, r["identification"][a].dump().c_str());
  std::printf("\n== replay (the model against the flights)\n");
  for (const char* a : {"yaw", "pitch", "roll"}) {
    const Json& x = r["replay"][a];
    if (x.isNull()) continue;
    std::printf("  %s: %s - %s\n", a, x["ok"].isBool() && x["ok"].asBool() ? "OK" : "NOT CONFIRMED", s(x["text"]).c_str());
    if (x["hovers"].isArray())
      for (const auto& h : arr(x["hovers"]))
        std::printf("     %s: swing log %s / model %s deg, heading RMS %s deg, output RMS %s %s\n", s(h["hover"]).c_str(), s(h["swingLogDeg"]).c_str(),
                    s(h["swingModelDeg"]).c_str(), s(h["headingRmsDeg"]).c_str(), s(h["outputRms"]).c_str(), h["pass"].asBool() ? "ok" : "MISS");
  }
  std::printf("\n== axes\n");
  for (const char* a : {"yaw", "pitch", "roll"}) {
    const Json& x = r["axes"][a];
    std::printf("  %s flown %s\n", a, x["flown"].dump().c_str());
    if (x.has("predictedFlown")) std::printf("     model on the flown gains: %s\n", x["predictedFlown"].dump().c_str());
    if (x.has("polesFlown")) std::printf("     poles: %s\n", x["polesFlown"].dump().c_str());
    if (x.has("design")) std::printf("     design: %s\n", x["design"].dump().c_str());
  }
  std::printf("\n== steps\n");
  for (const auto& st : arr(r["steps"])) {
    std::printf("  %s%s\n", s(st["title"]).c_str(), st["jump"].asBool() ? "" : " (limited to x2: model not validated)");
    for (const auto& row : arr(st["rows"]))
      std::printf("     %-22s %10s -> %-10s %s\n", s(row["name"]).c_str(), s(row["from"]).c_str(), s(row["to"]).c_str(), s(row["reason"]).c_str());
    std::printf("     predicted: %s\n", st["predicted"].dump().c_str());
  }
  if (r.has("check")) std::printf("\n== next-flight check\n  %s\n", r["check"].dump().c_str());
  std::printf("\n== notes\n");
  for (const auto& n : arr(r["notes"])) std::printf("  %s\n", s(n).c_str());
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> files;
  std::string planPath, outDir, jsonPath;
  heavy::Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--version") {
      std::printf("%s\n", aptuner::core::buildInfoLine().c_str());
      return 0;
    } else if (a == "--plan" && i + 1 < argc) planPath = argv[++i];
    else if (a == "--out" && i + 1 < argc) outDir = argv[++i];
    else if (a == "--json" && i + 1 < argc) jsonPath = argv[++i];
    else if (a == "--spacing" && i + 1 < argc) opt.motorSpacingM = std::stod(argv[++i]);
    else files.push_back(a);
  }
  if (files.empty()) {
    std::puts("usage: aptuner-heavy <log.bin> [more.bin ...] [--plan plan.json] [--out folder] [--spacing metres] [--json result.json]");
    return 2;
  }
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<heavy::FlightRecord> flights;
  for (const auto& f : files) {
    try {
      flights.push_back(heavy::recordFromLogFile(f));
    } catch (const std::exception& e) {
      std::printf("%s: NOT READ: %s\n", f.c_str(), e.what());
      return 3;
    }
  }
  if (!planPath.empty()) {
    std::ifstream in(planPath, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    opt.plan = Json::parse(ss.str());
  }
  Json r;
  try {
    r = heavy::analyzeFlights(flights, opt);
  } catch (const std::exception& e) {
    std::printf("ANALYSIS FAILED: %s\n", e.what());
    return 4;
  }
  print(r);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
  std::printf("\n(%lld ms)\n", static_cast<long long>(ms));
  if (!jsonPath.empty()) writeText(jsonPath, r.dump());
  if (!outDir.empty()) {
    fs::create_directories(outDir);
    for (const auto& st : arr(r["steps"])) {
      const std::string n = "Step " + s(st["n"]);
      writeText(fs::path(outDir) / (n + ".param"), st["paramText"].strOr(""));
      writeText(fs::path(outDir) / (n + " UNDO.param"), st["undoText"].strOr(""));
    }
    writeText(fs::path(outDir) / "plan.json", r["plan"].dump());
    std::printf("files written to %s\n", outDir.c_str());
  }
  return r.has("error") ? 1 : 0;
}
