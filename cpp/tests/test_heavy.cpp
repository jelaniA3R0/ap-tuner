// Model-based tuning of heavy / slow vehicles (engine/heavytune): the live-telemetry path on a simulated flight, the
// marginal-mode identification on a simulated oscillating loop, and the tandem QuadPlane logs as regression cases
// (skipped when the logs are not on this PC; the acceptance criteria of the hand tune, docs/validation/tandem-hover-logs.md).
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "../core/src/engine/heavy_internal.hpp"
#include "aptuner/core/cascade.hpp"
#include "aptuner/engine/flightrec.hpp"
#include "aptuner/engine/heavytune.hpp"
#include "check.hpp"

using aptuner::core::Json;
namespace cc = aptuner::core::cascade;
namespace heavy = aptuner::engine::heavy;

namespace {

Json quadplaneParams(double yawP, double yawI, double yawK) {
  Json p = Json::obj({{"Q_ENABLE", 1}, {"Q_FRAME_CLASS", 1}, {"Q_FRAME_TYPE", 1}, {"SYSID_THISMAV", 7}, {"SCHED_LOOP_RATE", 300}, {"INS_GYRO_FILTER", 20},
                      {"Q_A_RAT_YAW_P", yawP}, {"Q_A_RAT_YAW_I", yawI}, {"Q_A_RAT_YAW_IMAX", 0.5}, {"Q_A_RAT_YAW_FLTE", 2.5}, {"Q_A_RAT_YAW_FLTT", 20},
                      {"Q_A_ANG_YAW_P", yawK}, {"Q_A_ACC_Y_MAX", 100}, {"Q_A_RATE_Y_MAX", 75}, {"Q_PLT_Y_RATE", 100}, {"Q_PLT_Y_EXPO", 0.25},
                      {"Q_PLT_Y_RATE_TC", 0.25}, {"RCMAP_YAW", 4}, {"RC4_MIN", 1000}, {"RC4_TRIM", 1500}, {"RC4_MAX", 2000}, {"RC4_DZ", 30},
                      {"Q_M_PWM_MIN", 1000}, {"Q_M_PWM_MAX", 2000}, {"Q_M_SPIN_MIN", 0.13}, {"Q_M_SPIN_MAX", 0.95}, {"Q_M_THST_EXPO", 0.65},
                      {"Q_M_THST_HOVER", 0.5}, {"GCS_PID_MASK", 7}});
  for (const char* ax : {"RLL", "PIT"}) {
    const std::string a = std::string("Q_A_RAT_") + ax + "_";
    p.set(a + "P", 0.25);
    p.set(a + "I", 0.25);
    p.set(a + "D", 0.0036);
    p.set(a + "FLTT", 20);
    p.set(a + "FLTD", 10);
    p.set(a + "IMAX", 0.5);
    p.set(std::string("Q_A_ANG_") + ax + "_P", 4.5);
  }
  p.set("Q_A_ACC_R_MAX", 400);
  p.set("Q_A_ACC_P_MAX", 400);
  p.set("Q_A_INPUT_TC", 0.15);
  for (int i = 5; i <= 8; ++i) p.set("SERVO" + std::to_string(i) + "_FUNCTION", 28 + i);   // motors 1..4
  return p;
}

std::string tandemLog(const char* name) {
  const char* home = std::getenv("USERPROFILE");
  if (!home) return "";
  return std::string(home) + "/OneDrive/Documents/Mission Planner/logs/FIXED_WING/1/" + name;
}
bool exists(const std::string& p) { return !p.empty() && std::filesystem::exists(p); }

const Json* issueWith(const Json& r, const std::string& title) {
  for (const auto& i : r["issues"].arr())
    if (i["title"].strOr("").find(title) != std::string::npos) return &i;
  return nullptr;
}

}  // namespace

TEST(heavy_live_telemetry_pipeline) {
  // a slow yaw axis (a 0.25, c 0.1, motor lag 0.2 s, torque trim 0.4) flown three times with stock-like gains; the
  // telemetry the app would see through Mission Planner (25 Hz) goes through the recorder and the full analysis
  cc::Gains g;
  g.kind = cc::Kind::Yaw;
  g.P = 0.3;
  g.I = 0.03;
  g.IMAX = 0.5;
  g.angleP = 3;
  g.FLTE = 2.5;
  g.accMax = 100;
  g.rateMax = 75;
  g.pltRate = 100;
  cc::Plant p;
  p.a = 0.25;
  p.c = 0.1;
  p.tau = 0.2;
  p.T = 0.01;
  p.trim = 0.4;
  p.umax = 0.65;
  heavy::TelemetryRecorder rec;
  rec.setFamily(aptuner::core::Family::Vtol);
  rec.setParams(quadplaneParams(g.P, g.I, g.angleP));
  double t0 = 100;
  for (int hover = 0; hover < 3; ++hover) {
    cc::SimInput in;
    in.seconds = 40;
    in.recordHz = 25;
    const cc::SimOutput s = cc::simulate(g, p, in);
    auto hb = [&](double t, bool armed) { rec.onMessage("HEARTBEAT", Json::obj({{"type", 20}, {"base_mode", armed ? 128 + 1 : 1}, {"custom_mode", 17}}), t); };
    // 3 s on the ground at idle, then the hover, then 3 s landed
    for (int k = 0; k < 75; ++k) {
      const double t = t0 + k / 25.0;
      hb(t, true);
      rec.onMessage("ATTITUDE", Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", 0}}), t);
      rec.onMessage("SERVO_OUTPUT_RAW", Json::obj({{"port", 0}, {"servo5_raw", 1150}, {"servo6_raw", 1150}, {"servo7_raw", 1150}, {"servo8_raw", 1150}}), t);
    }
    t0 += 3;
    for (std::size_t k = 0; k < s.t.size(); ++k) {
      const double t = t0 + s.t[k];
      hb(t, true);
      const double r = s.rate[k] * cc::kD2R;
      rec.onMessage("PID_TUNING", Json::obj({{"axis", 3}, {"desired", r}, {"achieved", r}, {"P", s.u[k] - s.I[k]}, {"I", s.I[k]}, {"D", 0}, {"FF", 0}}), t);
      for (int ax : {1, 2}) rec.onMessage("PID_TUNING", Json::obj({{"axis", ax}, {"desired", 0}, {"achieved", 0}, {"P", 0}, {"I", 0}, {"D", 0}, {"FF", 0}}), t);
      rec.onMessage("ATTITUDE", Json::obj({{"roll", 0}, {"pitch", 0}, {"yaw", s.att[k] * cc::kD2R}}), t);
      rec.onMessage("RC_CHANNELS", Json::obj({{"chan4_raw", 1500}}), t);
      // CCW motors (1, 2) carry the yaw output
      const double y = 250 * s.u[k];
      rec.onMessage("SERVO_OUTPUT_RAW", Json::obj({{"port", 0}, {"servo5_raw", 1500 + y}, {"servo6_raw", 1500 + y}, {"servo7_raw", 1500 - y}, {"servo8_raw", 1500 - y}}), t);
    }
    t0 += 40;
    for (int k = 0; k < 75; ++k) hb(t0 + k / 25.0, false);
    t0 += 10;
  }
  const heavy::FlightRecord fr = rec.finish("telemetry test");
  const Json r = heavy::analyzeFlights({fr});
  CHECK(!r.has("error"));
  CHECK(r["hovers"].size() == 3);
  const Json& y = r["identification"]["yaw"];
  CHECK(y["ok"].asBool());
  CHECK(std::fabs(y["a"].numOr(0) - 0.25) < 0.06);
  for (const auto& t : y["trims"].arr()) CHECK(std::fabs(t.numOr(0) - 0.4) < 0.08);
  CHECK(r["replay"]["yaw"]["ok"].asBool());
  if (!r["replay"]["yaw"]["ok"].asBool()) std::printf("    yaw ident %s\n", y.dump().c_str());
  if (!r["replay"]["yaw"]["ok"].asBool())
    for (const auto& h : r["replay"]["yaw"]["hovers"].arr())
      std::printf("    %s: swing log %g model %g heading RMS %g output RMS %g\n", h["hover"].strOr("").c_str(), h["swingLogDeg"].numOr(0),
                  h["swingModelDeg"].numOr(0), h["headingRmsDeg"].numOr(0), h["outputRms"].numOr(0));
  CHECK(r["heavyClass"]["heavy"].asBool());
  // a yaw step is proposed, with an UNDO file holding the flown values
  bool yawStep = false;
  for (const auto& s : r["steps"].arr())
    if (s["title"].strOr("").find("yaw") != std::string::npos) {
      yawStep = true;
      CHECK(s["undoText"].strOr("").find("Q_A_RAT_YAW_P,0.3") != std::string::npos);
    }
  CHECK(yawStep);
}

TEST(heavy_marginal_mode_on_simulated_oscillation) {
  // stock pitch gains on a = 5.8, tau 0.08, T 0.02: the loop sits at its stability edge and gusts keep it oscillating;
  // the app must find the oscillation and give back the control power from it
  cc::Gains g;
  g.P = 0.25;
  g.I = 0.25;
  g.D = 0.0036;
  g.angleP = 4.5;
  g.FLTT = 20;
  g.FLTD = 10;
  g.gyroHz = 20;
  g.accMax = 400;
  g.rateMax = 75;
  cc::Plant p;
  p.a = 5.8;
  p.tau = 0.08;
  p.T = 0.02;
  p.umax = 0.45;
  const cc::GustSequence gust(61);
  cc::SimInput in;
  in.seconds = 60;
  in.shaped = true;
  in.angleDeg = [](double) { return 0.0; };
  in.dist = [&](double t) { return -gust.at(t); };
  in.recordHz = 100;
  const cc::SimOutput s = cc::simulate(g, p, in);
  heavy::FlightRecord fr;
  fr.family = aptuner::core::Family::Vtol;
  for (std::size_t k = 0; k < s.t.size(); ++k) {
    fr.axis[1].actual.push(s.t[k], s.rate[k]);
    fr.axis[1].target.push(s.t[k], s.rate[k]);
    fr.axis[1].out.push(s.t[k], s.u[k]);
    fr.axis[1].I.push(s.t[k], s.I[k]);
  }
  heavy::Hover h;
  h.t0 = 0;
  h.tLift = 5;
  h.t1 = 60;
  const Json qp = quadplaneParams(0.18, 0.018, 4.5);
  for (const auto& [k, v] : qp.items()) h.params.set(k, v.num());
  const heavy::Oscillation o = heavy::findOscillation({fr}, {h}, 1);
  CHECK(o.found);
  CHECK(o.fHz > 0.36 && o.fHz < 0.5);
  const heavy::RpIdent id = heavy::identifyMarginal(o, {h}, aptuner::core::Family::Vtol, 1);
  if (!id.ok || std::fabs(id.a - 5.8) >= 0.2 * 5.8) {
    std::printf("    oscillation %.3f Hz (+-%.1f deg), fit: %s a %.2f\n", o.fHz, o.angleAmpDeg, id.reason.c_str(), id.a);
    const cc::Gains fg = heavy::gainsFromParams(h.params, aptuner::core::Family::Vtol, 1);
    for (double T : {0.01, 0.025}) {
      const cc::Cx C = cc::controllerAt(fg, 2 * 3.14159265 * o.fHz);
      std::printf("    T %.3f: arg(-1/C) %.1f deg |1/C| %.3f\n", T, std::arg(-1.0 / C) * 57.3, 1 / std::abs(C));
    }
  }
  CHECK(id.ok);
  CHECK(std::fabs(id.a - 5.8) < 0.2 * 5.8);
}

TEST(heavylog_tandem_acceptance) {
  const std::string a = tandemLog("2026-10-15 04-20-29.bin"), b = tandemLog("2026-10-15 04-33-02.bin"), c = tandemLog("2026-10-15 04-43-00.bin");
  if (!exists(a) || !exists(b) || !exists(c)) {
    std::printf("    skipped: the tandem logs are not on this PC\n");
    return;
  }
  const Json r = heavy::analyzeFlights({heavy::recordFromLogFile(a), heavy::recordFromLogFile(b), heavy::recordFromLogFile(c)});
  // yaw: control power, trims, mixer limit, torque imbalance flagged as mechanical, replay of 04-33-02 hover 1
  const Json& y = r["identification"]["yaw"];
  CHECK(y["a"].numOr(0) >= 0.13 && y["a"].numOr(0) <= 0.35);
  for (const auto& t : y["trims"].arr()) CHECK(t.numOr(0) >= 0.33 && t.numOr(0) <= 0.5);
  CHECK(y["umax"].numOr(0) >= 0.58 && y["umax"].numOr(0) <= 0.68);
  const Json* ti = issueWith(r, "Torque imbalance");
  CHECK(ti && (*ti)["cantTune"].asBool());
  bool sawHover = false;
  for (const auto& h : r["replay"]["yaw"]["hovers"].arr())
    if (h["flight"].strOr("") == "2026-10-15 04-33-02.bin" && h["hover"].strOr("").find("hover 1") != std::string::npos) {
      sawHover = true;
      CHECK(std::fabs(h["swingModelDeg"].numOr(0) - 84) <= 15);
    }
  CHECK(sawHover);
  // pitch: a 0.43-0.48 Hz cascade with the flown gains near PM 0
  CHECK(issueWith(r, "Cascade oscillation"));
  const Json& pi = r["identification"]["pitch"];
  CHECK(pi["frequencyHz"].numOr(0) >= 0.43 && pi["frequencyHz"].numOr(0) <= 0.48);
  const double pm = r["replay"]["pitch"]["pmWorstDeg"].numOr(99);
  CHECK(pm >= -20 && pm <= 10);
  // roll: marginal, flagged
  CHECK(r["identification"]["roll"]["ok"].asBool());
  // CG aft by 3-6% of the motor spacing
  const double cg = r["cg"]["aftFractionOfSpacing"].numOr(0);
  CHECK(cg >= 0.03 && cg <= 0.06);
  // proposals
  const Json& yd = r["axes"]["yaw"]["design"]["gains"];
  CHECK(yd["P"].numOr(0) >= 1.0);
  CHECK(yd["angleP"].numOr(9) <= 1.5);
  CHECK(yd["IMAX"].numOr(0) >= 0.6);
  CHECK(yd["rateMax"].numOr(99) < 75 && yd["accMax"].numOr(999) < 100);
  for (const char* ax : {"pitch", "roll"}) {
    const Json& d = r["axes"][ax]["design"];
    CHECK(d["ok"].asBool());
    CHECK(d["gains"]["angleP"].numOr(9) <= 1.5);
    CHECK(d["gains"]["I"].numOr(9) <= 0.5 * d["gains"]["P"].numOr(0) + 1e-9);
    CHECK(d["pmWorstDeg"].numOr(0) >= 40);
    CHECK(d["noiseVsNow"].numOr(9) <= 1.15);
  }
  CHECK(r["steps"].size() >= 2);
  for (const auto& s : r["steps"].arr()) CHECK(!s["undoText"].strOr("").empty());

  // 21 Oct, Step 1 flown: recognised, the yaw prediction confirmed, the pitch wobble still reported
  const std::string s1 = tandemLog("2026-10-21 00-34-01.bin"), s2 = tandemLog("2026-10-21 01-11-37.bin");
  if (!exists(s1) || !exists(s2)) return;
  heavy::Options o1;
  o1.plan = r["plan"];
  const Json r1 = heavy::analyzeFlights({heavy::recordFromLogFile(s1)}, o1);
  CHECK(r1["check"]["stepFlown"].numOr(0) == 1);
  CHECK(r1["check"]["axes"]["yaw"]["confirmed"].asBool());
  CHECK(r1["check"]["axes"]["yaw"]["flownSwingDeg"].numOr(0) >= 10 && r1["check"]["axes"]["yaw"]["flownSwingDeg"].numOr(0) <= 16);
  CHECK(issueWith(r1, "Cascade oscillation"));
  // Step 2 flown: the yaw prediction confirmed, the growing pitch oscillation detected, a pitch / roll proposal
  heavy::Options o2;
  o2.plan = r1["plan"];
  const Json r2 = heavy::analyzeFlights({heavy::recordFromLogFile(s2)}, o2);
  CHECK(r2["check"]["axes"]["yaw"]["confirmed"].asBool());
  const Json* osc = issueWith(r2, "Cascade oscillation");
  CHECK(osc && (*osc)["axis"].strOr("") == "pitch" ? true : issueWith(r2, "Cascade oscillation") != nullptr);
  bool growing = false;
  for (const auto& i : r2["issues"].arr())
    if (i["axis"].strOr("") == "pitch" && i["text"].strOr("").find("growing") != std::string::npos) growing = true;
  CHECK(growing);
  bool rpStep = false;
  for (const auto& s : r2["steps"].arr())
    if (s["title"].strOr("").find("roll and pitch") != std::string::npos) rpStep = true;
  CHECK(rpStep);
}
