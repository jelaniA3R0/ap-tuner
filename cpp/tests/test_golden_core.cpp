// Phase 1 golden tests: every core math function against the JavaScript reference results
// (tests/golden/*.json, written by tools/make-golden.js from the same inputs).
#include <cmath>
#include <string>

#include "aptuner/core/controller.hpp"
#include "aptuner/core/fft.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/core/json.hpp"
#include "aptuner/core/plant.hpp"
#include "aptuner/core/rules.hpp"
#include "aptuner/core/spectra.hpp"
#include "aptuner/core/stepresp.hpp"
#include "aptuner/core/synth.hpp"
#include "aptuner/core/timeid.hpp"
#include "golden.hpp"

using namespace aptuner::core;
using aptuner::test::loadGolden;

namespace {

PidGains gainsOf(const Json& j) {
  PidGains g;
  g.P = j["P"].numOr(0);
  g.I = j["I"].numOr(0);
  g.D = j["D"].numOr(0);
  g.FF = j["FF"].numOr(0);
  g.DFF = j["DFF"].numOr(0);
  g.FLTT = j["FLTT"].numOr(0);
  g.FLTE = j["FLTE"].numOr(0);
  g.FLTD = j["FLTD"].numOr(0);
  return g;
}

NotchDef notchOf(const Json& j) {
  NotchDef n;
  n.enable = j["enable"].numOr(0) != 0;
  n.freq = j["freq"].numOr(0);
  n.bw = j["bw"].numOr(0);
  n.att = j["att"].numOr(40);
  n.hmncs = static_cast<int>(j["hmncs"].numOr(3));
  n.opts = static_cast<int>(j["opts"].numOr(0));
  n.chained = static_cast<int>(j["chained"].numOr(1));
  return n;
}

GyroChain gyroOf(const Json& j) {
  GyroChain g;
  g.backendHz = j["backendHz"].numOr(1000);
  g.gyroFilterHz = j["gyroFilterHz"].numOr(20);
  for (const auto& n : j["notches"].arr()) g.notches.push_back(notchOf(n));
  return g;
}

LoopCtx ctxOf(const Json& j) {
  LoopCtx c;
  c.fsLoop = j["fsLoop"].num();
  c.gyro = gyroOf(j["gyro"]);
  c.plant = Plant{j["plant"]["K"].num(), j["plant"]["a"].num(), j["plant"]["tau"].num(), j["plant"]["Td"].num()};
  return c;
}

SynthOptions synthOptsOf(const Json& j) {
  SynthOptions o;
  o.gmMinDb = j["gmMinDb"].num();
  o.pmMinDeg = j["pmMinDeg"].num();
  o.msMax = j["msMax"].num();
  o.osMaxPct = j["osMaxPct"].numOr(10);
  o.backoff = j["backoff"].numOr(0.4);
  o.pRange[0] = j["pRange"][0].num();
  o.pRange[1] = j["pRange"][1].num();
  o.dRange[0] = j["dRange"][0].num();
  o.dRange[1] = j["dRange"][1].num();
  o.iRatio = j["iRatio"].num();
  o.filters.FLTT = j["filters"]["FLTT"].num();
  o.filters.FLTE = j["filters"]["FLTE"].num();
  o.filters.FLTD = j["filters"]["FLTD"].num();
  o.uMax = j["uMax"].numOr(INFINITY);
  o.dNoiseCap = j["dNoiseCap"].numOr(0);
  if (j.has("noisePsd")) o.noisePsd = NoisePsd{j["noisePsd"]["f"].toVec(), j["noisePsd"]["psd"].toVec()};
  o.noisePsdIsFiltered = j["noisePsdIsFiltered"].asBool();
  o.rMax = j["rMax"].numOr(0.08);
  o.stepMag = j["stepMag"].numOr(30);
  o.stepDurS = j["stepDurS"].numOr(1.0);
  o.currentP = j["currentP"].numOr(0);
  o.currentD = j["currentD"].numOr(0);
  return o;
}

void checkSynth(const char* what, const std::optional<SynthResult>& got, const Json& exp, double rel) {
  if (exp.isNull()) {
    if (got) aptuner::test::fail(__FILE__, __LINE__, std::string(what) + ": expected no result");
    return;
  }
  if (!got) {
    aptuner::test::fail(__FILE__, __LINE__, std::string(what) + ": no result");
    return;
  }
  const Json& g = exp["gains"];
  CHECK_REL(got->gains.P, g["P"].num(), rel);
  CHECK_REL(got->gains.I, g["I"].num(), rel);
  CHECK_NEAR(got->gains.D, g["D"].num(), rel * std::fmax(1e-6, std::fabs(g["D"].num())));
  CHECK_NEAR(got->margins.pmDeg, exp["margins"]["pmDeg"].num(), 1e-3 + rel * 100);
  if (exp["margins"]["gmDb"].isNumber()) CHECK_NEAR(got->margins.gmDb, exp["margins"]["gmDb"].num(), 1e-3 + rel * 30);
  CHECK_REL(got->margins.ms, exp["margins"]["ms"].num(), rel * 10);
  CHECK_NEAR(got->overshootPct, exp["overshootPct"].num(), 1e-3 + rel * 100);
  CHECK_REL(got->dNoise, exp["dNoise"].num(), rel * 10);
  CHECK(static_cast<double>(got->candidates.size()) == exp["nCand"].num());
  if (!exp["osc"].isNull()) {
    CHECK(got->oscillationPoints.has_value());
    if (got->oscillationPoints) {
      const auto& o = *got->oscillationPoints;
      CHECK(o.D.has_value() == exp["osc"]["D"].isNumber());
      if (o.D && exp["osc"]["D"].isNumber()) CHECK_REL(*o.D, exp["osc"]["D"].num(), rel);
      CHECK(o.P.has_value() == exp["osc"]["P"].isNumber());
      if (o.P && exp["osc"]["P"].isNumber()) CHECK_REL(*o.P, exp["osc"]["P"].num(), rel);
      CHECK_REL(o.startP, exp["osc"]["startP"].num(), 1e-12);
    }
  }
}

}  // namespace

TEST(golden_fft) {
  const Json j = loadGolden("fft");
  const std::size_t n = static_cast<std::size_t>(j["n"].num());
  Vec re = j["inRe"].toVec(), im = j["inIm"].toVec();
  FFT f(n);
  f.transform(re, im);
  CHECK_VEC(re, j["fwd"]["re"].toVec(), 1e-12);
  CHECK_VEC(im, j["fwd"]["im"].toVec(), 1e-12);
  f.transform(re, im, true);
  CHECK_VEC(re, j["back"]["re"].toVec(), 1e-12);
  CHECK_VEC(im, j["back"]["im"].toVec(), 1e-12);
  CHECK_VEC(hann(16), j["hann16"].toVec(), 1e-15);
  for (const auto& p : j["nextPow2"].arr()) CHECK(static_cast<double>(nextPow2(p[0].num())) == p[1].num());
}

TEST(golden_spectra) {
  const Json j = loadGolden("spectra");
  const double fs = j["fs"].num();
  const Vec x = j["x"].toVec(), y = j["y"].toVec();
  const FreqResponse fr = freqResponse(x, y, fs, 512, 0.75, 0);
  CHECK(static_cast<double>(fr.nWin) == j["fr"]["nWin"].num());
  CHECK_VEC(fr.f, j["fr"]["f"].toVec(), 1e-15);
  CHECK_CVEC(fr.H, j["fr"]["H"], 1e-9);
  CHECK_VEC(fr.coh, j["fr"]["coh"].toVec(), 1e-9);
  CHECK_VEC(fr.Sxx, j["fr"]["Sxx"].toVec(), 1e-9);
  CHECK_VEC(fr.Syy, j["fr"]["Syy"].toVec(), 1e-9);
  const FreqResponse fr2 = freqResponse(x, y, fs, 256, 0.5, 6);
  CHECK(static_cast<double>(fr2.nWin) == j["frSkip"]["nWin"].num());
  CHECK_VEC(fr2.coh, j["frSkip"]["coh"].toVec(), 1e-9);
  const Psd sp = psd(x, fs, 256, 0.5);
  CHECK(static_cast<double>(sp.nWin) == j["psd"]["nWin"].num());
  CHECK_VEC(sp.psd, j["psd"]["psd"].toVec(), 1e-9);
  CHECK_VEC(sp.amp, j["psd"]["amp"].toVec(), 1e-9);
  CHECK_REL(bandRms(sp, 5, 40), j["bandRms"].num(), 1e-10);
}

TEST(golden_stepresp) {
  const Json j = loadGolden("stepresp");
  StepResponseOptions o;
  o.windowSize = 512;
  o.durationS = 0.5;
  o.minInput = 15;
  const StepResponse sr = stepResponse(j["target"].toVec(), j["actual"].toVec(), j["fs"].num(), o);
  CHECK(static_cast<double>(sr.count) == j["count"].num());
  CHECK_VEC(sr.t, j["t"].toVec(), 1e-15);
  CHECK_VEC(sr.mean, j["mean"].toVec(), 1e-8);
  const auto m = stepMetrics(sr.t, sr.mean);
  CHECK(m.has_value());
  if (m) {
    CHECK_REL(m->finalValue, j["metrics"]["finalValue"].num(), 1e-8);
    CHECK_REL(m->overshootPct, j["metrics"]["overshootPct"].num(), 1e-6);
    CHECK_NEAR(m->riseTime, j["metrics"]["riseTime"].num(), 1e-12);
    CHECK_NEAR(m->settlingTime, j["metrics"]["settlingTime"].num(), 1e-12);
    CHECK(static_cast<double>(m->oscillations) == j["metrics"]["oscillations"].num());
  }
  const auto h = stepMetrics(j["handT"].toVec(), j["handY"].toVec());
  CHECK(h.has_value());
  if (h) {
    CHECK_REL(h->overshootPct, j["handMetrics"]["overshootPct"].num(), 1e-10);
    CHECK_NEAR(h->riseTime, j["handMetrics"]["riseTime"].num(), 1e-12);
    CHECK_NEAR(h->settlingTime, j["handMetrics"]["settlingTime"].num(), 1e-12);
  }
  CHECK(!stepMetrics({}, {}).has_value());
}

TEST(golden_controller) {
  const Json j = loadGolden("controller");
  const Vec f = j["freqs"].toVec();
  CHECK_CVEC(lpf1(f, 400, 20), j["lpf1"], 1e-11);
  CHECK_CVEC(lpf1(f, 400, 0), j["lpf1Off"], 1e-15);
  CHECK_CVEC(biquadLpf(f, 1000, 20), j["biquadLpf"], 1e-9);
  CHECK_CVEC(notch(f, 1000, 80, 40, 40), j["notch"], 1e-9);
  for (const auto& h : j["harmonic"].arr()) CHECK_CVEC(harmonicNotch(f, 1000, notchOf(h["n"])), h["H"], 1e-9);
  CHECK_CVEC(gyroChain(f, gyroOf(j["gyro"])), j["gyroChain"], 1e-9);
  const PidResponse pid = acPid(f, 400, gainsOf(j["gains"]));
  CHECK_CVEC(pid.T, j["pid"]["T"], 1e-11);
  CHECK_CVEC(pid.Cfb, j["pid"]["Cfb"], 1e-9);
  CHECK_CVEC(pid.Cff, j["pid"]["Cff"], 1e-9);
  CHECK_CVEC(pid.P, j["pid"]["P"], 1e-11);
  CHECK_CVEC(pid.I, j["pid"]["I"], 1e-9);
  CHECK_CVEC(pid.D, j["pid"]["D"], 1e-9);
  for (const auto& c : j["notchBase"].arr()) {
    NotchParams p;
    p.mode = static_cast<int>(c["p"]["mode"].num());
    p.freq = c["p"]["freq"].num();
    p.ref = c["p"]["ref"].num();
    p.fmRat = c["p"]["fm_rat"].num();
    NotchOp op;
    if (c["op"].has("throttle")) op.throttle = c["op"]["throttle"].num();
    op.rpm = c["op"]["rpm"].numOr(0);
    op.escRpm = c["op"]["escRpm"].numOr(0);
    op.fftHz = c["op"]["fftHz"].numOr(0);
    CHECK_REL(notchBaseFreq(p, op), c["f"].num(), 1e-14);
  }
}

TEST(golden_plant) {
  const Json j = loadGolden("plant");
  const Vec f = j["freqs"].toVec();
  const Json& t = j["truth"];
  CHECK_CVEC(evalPlant(f, Plant{t["K"].num(), t["a"].num(), t["tau"].num(), t["Td"].num()}), j["G"], 1e-11);
  FitPlantOptions o;
  o.kind = "copter";
  o.nWin = 60;
  o.fMin = 0.3;
  o.fMax = 30;
  const auto fit = fitPlant(f, aptuner::test::cvecOf(j["H"]), j["coh"].toVec(), o);
  CHECK(fit.has_value());
  if (fit) {
    const Json& e = j["fit"];
    CHECK_REL(fit->K, e["K"].num(), 1e-5);
    CHECK_REL(fit->a, e["a"].num(), 1e-5);
    CHECK_REL(fit->tau, e["tau"].num(), 1e-5);
    CHECK_REL(fit->Td, e["Td"].num(), 1e-5);
    CHECK_REL(fit->cost, e["cost"].num(), 1e-5);
    CHECK(static_cast<double>(fit->nBins) == e["nBins"].num());
    CHECK_REL(fit->fitQuality, e["fitQuality"].num(), 1e-8);
  }
  const auto ros = nelderMead(
      [](const Vec& x) { return 100 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]) + (1 - x[0]) * (1 - x[0]); },
      {-1.2, 1}, {0.3, 2000, 1e-12});
  CHECK_NEAR(ros.x[0], j["rosenbrock"]["x"][0].num(), 1e-9);
  CHECK_NEAR(ros.x[1], j["rosenbrock"]["x"][1].num(), 1e-9);
}

TEST(golden_synth_loops) {
  const Json j = loadGolden("synth");
  const LoopCtx ctx = ctxOf(j["ctx"]);
  const PidGains g = gainsOf(j["g"]);
  const Vec f = j["freqs"].toVec();
  CHECK_VEC(logspace(0.1, 200, 7), j["logspace"].toVec(), 1e-14);
  const OpenLoop ol = openLoop(f, ctx, g);
  CHECK_CVEC(ol.L, j["L"], 1e-9);
  const ClosedLoop cl = closedLoop(f, ctx, g);
  CHECK_CVEC(cl.Tcl, j["Tcl"], 1e-9);
  CHECK_CVEC(cl.S, j["S"], 1e-9);
  const Margins m = margins(f, ol.L);
  CHECK_REL(m.pmDeg, j["margins"]["pmDeg"].num(), 1e-9);
  CHECK_REL(m.gmDb, j["margins"]["gmDb"].num(), 1e-9);
  CHECK_REL(m.wcHz, j["margins"]["wcHz"].num(), 1e-9);
  CHECK_REL(m.w180Hz, j["margins"]["w180Hz"].num(), 1e-9);
  CHECK_REL(m.ms, j["margins"]["ms"].num(), 1e-9);
  CHECK_REL(bandwidth3db(f, cl.Tcl), j["bw3db"].num(), 1e-12);
  CHECK_REL(sensitivityBandwidth(f, cl.S), j["sBw"].num(), 1e-12);
  CHECK_REL(sensitivityEnergy(f, cl.S, 10), j["sEnergy"].num(), 1e-9);
  const SimResult st = simulateRateStep(ctx, g, 30 / 57.2957795, 1.0, 1);
  CHECK_VEC(st.y, j["step"]["y"].toVec(), 1e-9);
  CHECK_VEC(st.u, j["step"]["u"].toVec(), 1e-9);
  const LoopCtx ctxN = ctxOf(j["ctxNotch"]);
  const Vec dist = j["dist"].toVec();
  SimOptions so;
  so.noiseRms = 0.02;
  so.seed = 9;
  so.dist = &dist;
  so.uMax = 0.8;
  const SimResult ns = simulateRate(ctxN, g, j["refs"].toVec(), so);
  CHECK_VEC(ns.y, j["noisy"]["y"].toVec(), 1e-8);
  CHECK_VEC(ns.u, j["noisy"]["u"].toVec(), 1e-8);
  CHECK_VEC(ns.ytrue, j["noisy"]["ytrue"].toVec(), 1e-8);
}

TEST(golden_synth_design) {
  const Json j = loadGolden("synth");
  const LoopCtx ctx = ctxOf(j["ctx"]), ctxN = ctxOf(j["ctxNotch"]), ctxY = ctxOf(j["ctxYaw"]);
  SynthOptions noiseOpts = synthOptsOf(j["optsNoise"]);
  checkSynth("quik", synthesizeQuik(ctx, synthOptsOf(j["opts"])), j["quik"], 1e-6);
  checkSynth("quikNoise", synthesizeQuik(ctxN, noiseOpts), j["quikNoise"], 1e-6);
  checkSynth("quikCons", synthesizeQuik(ctx, synthOptsOf(j["optsCons"])), j["quikCons"], 1e-6);
  checkSynth("quikYaw", synthesizeQuik(ctxY, synthOptsOf(j["yawOpts"])), j["quikYaw"], 1e-6);
  checkSynth("shaped", synthesize(ctx, synthOptsOf(j["opts"])), j["shaped"], 1e-6);
  checkSynth("shapedNoise", synthesize(ctxN, noiseOpts), j["shapedNoise"], 1e-6);
}

TEST(golden_rules) {
  const Json j = loadGolden("rules");
  auto fam = [](const Json& s) { return *parseFamily(s.str()); };
  auto ax = [](const Json& s) { return *parseAxis(s.str()); };
  for (const auto& c : j["filters"].arr()) {
    AxisGainsIn cur{c["c"]["FLTT"].num(), c["c"]["FLTE"].num(), c["c"]["FLTD"].num()};
    const Filters r = filterRules(fam(c["f"]), ax(c["a"]), cur, c["gh"].num(), c["tau"].num());
    CHECK(r.FLTT == c["r"]["FLTT"].num());
    CHECK(r.FLTE == c["r"]["FLTE"].num());
    CHECK(r.FLTD == c["r"]["FLTD"].num());
  }
  for (const auto& c : j["irule"].arr())
    CHECK(iRule(fam(c["f"]), ax(c["a"]), c["P"].num(), c["FF"].num(), c["hasFF"].asBool()) == c["r"].num());
  for (const auto& c : j["angle"].arr()) {
    const AngleRule r = angleRule(fam(c["f"]), ax(c["a"]), c["wc"].num(), c["yawMin"].num());
    if (c["r"].has("angleP")) CHECK(r.angleP && *r.angleP == c["r"]["angleP"].num());
    if (c["r"].has("tconst")) CHECK(r.tconst && *r.tconst == c["r"]["tconst"].num());
  }
  for (const auto& c : j["limits"].arr()) {
    const GainLimits l = gainLimits(fam(c["f"]), ax(c["a"]));
    CHECK(l.P[0] == c["r"]["P"][0].num() && l.P[1] == c["r"]["P"][1].num());
    CHECK(l.D[0] == c["r"]["D"][0].num() && l.D[1] == c["r"]["D"][1].num());
    CHECK(l.FF[0] == c["r"]["FF"][0].num() && l.FF[1] == c["r"]["FF"][1].num());
  }
  for (const char* f : {"copter", "vtol", "plane"}) {
    const auto names = neededParams(*parseFamily(f));
    const auto& exp = j["needed"][f].arr();
    CHECK(names.size() == exp.size());
    for (std::size_t i = 0; i < names.size() && i < exp.size(); i++) CHECK(names[i] == exp[i].str());
  }
  for (const auto& c : j["osc"].arr()) CHECK(oscSmaxFor(c[0].num()) == c[1].num());
  for (const auto& p : kPresets) {
    const Json& e = j["presets"][p.name];
    CHECK(p.gmMinDb == e["gmMinDb"].num() && p.pmMinDeg == e["pmMinDeg"].num() && p.msMax == e["msMax"].num());
    CHECK(p.osMaxPct == e["osMaxPct"].num() && p.backoff == e["backoff"].num() && std::string(p.label) == e["label"].str());
  }
}

TEST(golden_timeid) {
  const Json j = loadGolden("timeid");
  const Vec sig = j["sig"].toVec();
  CHECK_VEC(lpf2(sig, 100, 2), j["lpf2"].toVec(), 1e-11);
  CHECK_VEC(lag1(sig, 100, 0.05), j["lag1"].toVec(), 1e-12);
  for (const auto& c : j["cases"].arr()) {
    TimeIdInput in;
    in.fs = c["fs"].num();
    in.actual = c["actual"].toVec();
    in.u = c["u"].toVec();
    for (double v : c["airborne"].toVec()) in.airborne.push_back(v != 0 ? 1 : 0);
    TimeIdOptions o;
    o.axis = *parseAxis(c["axis"].str());
    const TimeIdResult r = identifyTimeDomain(in, o);
    const std::string label = c["axis"].str();
    CHECK(r.ok == c["ok"].asBool());
    CHECK(r.reason == c["reason"].strOr(""));
    if (!c["fit"].isNull()) {
      CHECK(r.fit.has_value());
      if (!r.fit) continue;
      const Json& e = c["fit"];
      const TimeIdFit& f = *r.fit;
      CHECK(f.method == e["method"].str());
      CHECK(f.tau == e["tau"].num());
      CHECK_REL(f.Td, e["Td"].num(), 1e-12);
      CHECK(f.uLimit.has_value() == e["uLimit"].isNumber());
      if (f.uLimit && e["uLimit"].isNumber()) CHECK(*f.uLimit == e["uLimit"].num());
      CHECK_REL(f.K, e["K"].num(), 1e-5);
      CHECK_REL(f.a, e["a"].num(), 1e-5);
      CHECK_REL(f.r2, e["r2"].num(), 1e-8);
      CHECK_NEAR(f.vaf, e["vaf"].num(), 1e-5);
      CHECK_NEAR(f.trimMean, e["trimMean"].num(), 1e-5);
      CHECK_REL(f.bRelErr, e["bRelErr"].num(), 1e-6);
      CHECK_NEAR(f.limitPct, e["limitPct"].num(), 1e-9);
      CHECK(static_cast<double>(f.segments) == e["segments"].num());
      CHECK_NEAR(f.fitQuality, e["fitQuality"].num(), 1e-5);
    }
  }
}

TEST(golden_jsnum) {
  const Json j = loadGolden("jsnum");
  for (const auto& c : j["toString"].arr()) CHECK(jsToString(c[0].num()) == c[1].str());
  for (const auto& c : j["toFixed"].arr()) {
    const std::string got = jsToFixed(c[0].num(), static_cast<int>(c[1].num()));
    if (got != c[2].str())
      aptuner::test::fail(__FILE__, __LINE__, "toFixed(" + jsToString(c[0].num()) + ", " + jsToString(c[1].num()) +
                                                  ") = " + got + ", expected " + c[2].str());
  }
  for (const auto& c : j["toPrecision"].arr()) {
    const std::string got = jsToPrecision(c[0].num(), static_cast<int>(c[1].num()));
    if (got != c[2].str())
      aptuner::test::fail(__FILE__, __LINE__, "toPrecision(" + jsToString(c[0].num()) + ", " +
                                                  jsToString(c[1].num()) + ") = " + got + ", expected " + c[2].str());
  }
  // JSON round trip writes exactly what JSON.stringify wrote
  CHECK(Json::parse(j["json"].str()).dump() == j["json"].str());
}
