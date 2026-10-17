#include "notchlive.hpp"

#include <cmath>

#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/vehicle.hpp"

namespace aptuner::app {

using core::jsToFixed;
using engine::jv;
using rt::Loop;

const std::vector<std::string> kNotchParams{"INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_HNTCH_FREQ", "INS_HNTCH_BW",
                                            "INS_HNTCH_ATT",    "INS_HNTCH_REF",  "INS_HNTCH_HMNCS", "INS_HNTCH_OPTS",
                                            "INS_HNTCH_FM_RAT", "INS_GYRO_FILTER", "FFT_ENABLE",     "FFT_MINHZ",
                                            "FFT_MAXHZ"};
constexpr int kAuxFftNotchTune = 162;

namespace {
Json lastN(const Json& a, std::size_t n) {
  Json o = a;
  if (o.size() > n) o.arr().erase(o.arr().begin(), o.arr().end() - static_cast<std::ptrdiff_t>(n));
  return o;
}
}  // namespace

NotchLive::NotchLive(io::Link& link, Tuner& tuner) : link_(link), tuner_(tuner) {
  link_.on("ESC_TELEMETRY_1_TO_4", [this](const Json& f) { onEsc(f); });
  link_.on("ESC_TELEMETRY_5_TO_8", [this](const Json& f) { onEsc(f); });
  link_.on("STATUSTEXT", [this](const Json& f) {
    const std::string t = f["text"].strOr("");
    if (t.rfind("FFT", 0) == 0) {
      fftMessages_.push(Json::obj({{"t", Loop::now()}, {"text", t}}));
      if (fftMessages_.size() > 30) fftMessages_.arr().erase(fftMessages_.arr().begin());
    }
  });
}

void NotchLive::onEsc(const Json& f) {
  double s = 0;
  int n = 0;
  for (const auto& v : f["rpm"].arr())
    if (v.numOr(0) > 0) {
      s += v.num();
      n++;
    }
  if (!n) return;
  escSeen_ = true;
  escLastTs_ = Loop::now();
  rpm_.push_back({escLastTs_, s / n, n});
  if (rpm_.size() > 600) rpm_.erase(rpm_.begin());
}

rt::Task<Json> NotchLive::status() {
  const Json got = co_await link_.requestParams(kNotchParams, 3);
  for (const auto& [k, v] : got.items()) tuner_.params.set(k, v);
  const Json& p = tuner_.params;
  const bool fftCapable = got["FFT_ENABLE"].isNumber();
  const double now = Loop::now();
  double sum = 0;
  int cnt = 0;
  for (const auto& r : rpm_)
    if (now - r.t < 20000) {
      sum += r.mean;
      cnt++;
    }
  const Json escRpm = cnt ? Json(sum / cnt) : Json();
  Json current = Json::object();
  for (const auto& k : kNotchParams) current.set(k, p[k]);
  const bool escTel = escSeen_ && now - escLastTs_ < 20000;
  Json out = Json::obj({{"current", current},
                        {"fftCapable", fftCapable},
                        {"fftEnabled", fftCapable && jv(p, "FFT_ENABLE") > 0},
                        {"escTelemetry", escTel},
                        {"escRpm", escRpm},
                        {"notchActive", jv(p, "INS_HNTCH_ENABLE") > 0},
                        {"mode", static_cast<int>(p["INS_HNTCH_MODE"].numOr(0))},
                        {"armed", link_.vehicle()["armed"].asBool()},
                        {"messages", lastN(fftMessages_, 8)},
                        {"options", Json::array()}});
  if (escTel) {
    const double f0 = escRpm.num() / 60;
    out.set("escProposal", Json::obj({{"INS_HNTCH_ENABLE", 1}, {"INS_HNTCH_MODE", 3}, {"INS_HNTCH_FREQ", core::jsRoundFixed(std::max(10.0, f0 * 0.5), 1)},
                                      {"INS_HNTCH_BW", core::jsRoundFixed(std::max(5.0, f0 * 0.25), 1)}, {"INS_HNTCH_ATT", 40}, {"INS_HNTCH_REF", 1},
                                      {"INS_HNTCH_HMNCS", 3}, {"INS_HNTCH_OPTS", 0}, {"INS_HNTCH_FM_RAT", 1}}));
    out.at("options").push(Json::obj({{"id", "esc"}, {"title", "Track the ESC RPM telemetry (recommended)"},
                                       {"detail", "Your ESCs report RPM (about " + core::jsToString(core::jsRound(escRpm.num())) + " rpm now, " +
                                                      jsToFixed(f0, 1) + " Hz). The notch follows the motors exactly, no measurement flight needed."}}));
  }
  if (fftCapable)
    out.at("options").push(Json::obj(
        {{"id", "fft"},
         {"title", out["fftEnabled"].asBool() ? "Measure the motor noise with the autopilot's own FFT" : "Enable the autopilot's FFT (needs one reboot), then measure"},
         {"detail", "In a steady hover the autopilot runs an FFT on its raw gyro for 30 s and writes the notch frequency, bandwidth, reference throttle and harmonics itself (ArduPilot RC option 162)."}}));
  out.at("options").push(Json::obj({{"id", "log"}, {"title", "Design from a log file"},
                                     {"detail", "Log raw gyro (INS_RAW_LOG_OPT = 9) on a 1-2 minute hover, then use the Log tab. Most precise, and it shows you the noise spectrum."}}));
  if (!fftCapable && !escTel) out.set("note", "This autopilot reports neither an FFT nor ESC RPM telemetry: use the log route.");
  co_return out;
}

rt::Task<Json> NotchLive::applyEsc() {
  const Json st = co_await status();
  if (!st["escProposal"].isObject()) throw rt::Error("no ESC RPM telemetry seen in the last 20 s");
  // in flight: watched on every axis and reverted if the vehicle starts to oscillate
  const bool held = co_await tuner_.applyFilterChange(st["escProposal"], "notch: ESC RPM tracking");
  if (!held) throw rt::Error("the notch was not kept: " + (tuner_.stateInfo.empty() ? std::string("see the messages") : tuner_.stateInfo), "reverted");
  co_return st["escProposal"];
}

rt::Task<Json> NotchLive::enableFft() {
  const bool held = co_await tuner_.applyFilterChange(Json::obj({{"FFT_ENABLE", 1}}), "notch: enable in-flight FFT");
  if (!held) throw rt::Error("FFT_ENABLE was not kept: " + (tuner_.stateInfo.empty() ? std::string("see the messages") : tuner_.stateInfo), "reverted");
  co_return Json::obj({{"FFT_ENABLE", tuner_.params["FFT_ENABLE"]}, {"rebootNeeded", true}});
}

rt::Task<Json> NotchLive::reboot() {
  if (!link_.alive()) throw rt::Error("no vehicle heartbeat: connect first");
  if (link_.vehicle()["armed"].asBool()) throw rt::Error("refusing to reboot an armed vehicle");
  Json r;
  try {
    r = co_await link_.commandAck(246, {1});
  } catch (const std::exception&) {
    r = Json::obj({{"result", -1}});
  }
  co_return Json::obj({{"sent", true}, {"result", r["result"]}});
}

rt::Task<Json> NotchLive::startFftTune() {
  if (!link_.vehicle()["armed"].asBool()) throw rt::Error("the vehicle must be flying (armed, steady hover)");
  const Json st = co_await status();
  if (!st["fftEnabled"].asBool()) throw rt::Error("FFT_ENABLE is 0: enable it and reboot first");
  Json want = Json::object();
  if (!(jv(tuner_.params, "INS_HNTCH_ENABLE") > 0)) want.set("INS_HNTCH_ENABLE", 1);
  if (st["mode"].numOr(0) != 1) want.set("INS_HNTCH_MODE", 1);
  if (want.size() && !(co_await tuner_.applyFilterChange(want, "notch: throttle tracking for the FFT tune")))
    throw rt::Error("the notch was not kept: " + (tuner_.stateInfo.empty() ? std::string("see the messages") : tuner_.stateInfo), "reverted");
  fftMessages_ = Json::array();
  Json r;
  try {
    r = co_await link_.commandAck(218, {static_cast<double>(kAuxFftNotchTune), 2});
  } catch (const std::exception&) {
    r = Json::obj({{"result", -1}});
  }
  if (r["result"].numOr(-1) != 0)
    throw rt::Error("the autopilot did not accept the FFT tune command (result " + core::jsToString(r["result"].numOr(-1)) +
                    "); firmware may lack the GyroFFT feature");
  tune_ = Json::obj({{"startedAt", Loop::now()}, {"done", false}});
  co_return Json::obj({{"started", true}});
}

rt::Task<Json> NotchLive::stopFftTune() {
  Json r;
  try {
    r = co_await link_.commandAck(218, {static_cast<double>(kAuxFftNotchTune), 0});
  } catch (const std::exception&) {
    r = Json::obj({{"result", -1}});
  }
  co_await rt::sleep(2500);
  const Json before = tuner_.params;
  const Json got = co_await link_.requestParams(kNotchParams, 3);
  for (const auto& [k, v] : got.items()) tuner_.params.set(k, v);
  Json changed = Json::object();
  for (const auto& k : kNotchParams)
    if (got[k].isNumber() && !(got[k] == before[k])) changed.set(k, Json::obj({{"from", before[k]}, {"to", got[k]}}));
  for (const auto& [k, c] : changed.items())
    tuner_.journal.push(Json::obj({{"t", Loop::now()}, {"name", k}, {"from", c["from"]}, {"to", c["to"]}, {"reason", "written by the autopilot FFT notch tune"},
                                   {"armed", link_.vehicle()["armed"].asBool()}}));
  tuner_.saveSession();
  const Json msgs = lastN(fftMessages_, 8);
  tune_ = Json::obj({{"done", true}, {"result", r["result"]}, {"changed", changed}, {"messages", msgs}});
  // what the autopilot wrote in flight is watched like the app's own changes (reverted on a trip)
  const bool held = co_await tuner_.watchExternalChange(changed, "written by the autopilot FFT notch tune");
  bool selected = false;
  for (const auto& m : fftMessages_.arr())
    if (m["text"].strOr("").find("selected") != std::string::npos) selected = true;
  Json current = Json::object();
  for (const auto& k : kNotchParams) current.set(k, tuner_.params[k]);
  co_return Json::obj({{"ok", held && (changed.size() > 0 || selected)}, {"held", held}, {"changed", changed}, {"messages", msgs}, {"current", current}});
}

}  // namespace aptuner::app
