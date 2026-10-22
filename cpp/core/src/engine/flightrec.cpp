// One flight for the model-based tuner, from a .bin log or from live telemetry (see flightrec.hpp).
#include "aptuner/engine/flightrec.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <regex>

#include "aptuner/engine/flightdata.hpp"
#include "aptuner/log/integrity.hpp"

namespace aptuner::engine::heavy {

namespace {

constexpr double kR2D = 57.29577951308232;

const log::Vec* col(const log::MsgTable* m, const char* f) { return m ? m->col(f) : nullptr; }

/** a column of a table as a series (time from TimeUS), optionally only the rows where `sel` holds */
Series series(const log::MsgTable* m, const char* field, double scale = 1, const std::vector<char>* sel = nullptr) {
  Series s;
  const log::Vec* t = col(m, "TimeUS");
  const log::Vec* v = col(m, field);
  if (!t || !v) return s;
  s.t.reserve(t->size());
  s.v.reserve(t->size());
  for (std::size_t i = 0; i < t->size() && i < v->size(); ++i) {
    if (sel && !(*sel)[i]) continue;
    const double x = (*v)[i];
    if (!std::isfinite(x)) continue;
    s.push((*t)[i] / 1e6, x * scale);
  }
  return s;
}

/** rows of a multi-instance table that belong to instance 0 (all rows when there is no instance field) */
std::vector<char> instance0(const log::MsgTable* m) {
  std::vector<char> sel(m ? m->n : 0, 1);
  if (!m) return sel;
  const log::Vec* in = m->col("I");
  if (!in) in = m->col("IMU");
  if (!in) in = m->col("Instance");
  if (in)
    for (std::size_t i = 0; i < m->n && i < in->size(); ++i) sel[i] = (*in)[i] == 0;
  return sel;
}

}  // namespace

// ------------------------------------------------------------------------------------------------- Series

std::size_t Series::indexAt(double tq) const {
  if (t.empty() || tq <= t.front()) return 0;
  const auto it = std::upper_bound(t.begin(), t.end(), tq);
  return static_cast<std::size_t>(it - t.begin()) - 1;
}

double Series::at(double tq) const { return t.empty() ? std::nan("") : v[indexAt(tq)]; }

double Series::rateHz() const {
  if (t.size() < 3) return 0;
  Vec d;
  d.reserve(t.size() - 1);
  for (std::size_t i = 1; i < t.size(); ++i)
    if (t[i] > t[i - 1]) d.push_back(t[i] - t[i - 1]);
  if (d.empty()) return 0;
  std::nth_element(d.begin(), d.begin() + static_cast<long>(d.size() / 2), d.end());
  return 1 / d[d.size() / 2];
}

log::Params FlightRecord::paramsAt(double t) const {
  log::Params p = params;
  for (const auto& c : changes)
    if (c.t <= t) p.set(c.name, c.to);
  return p;
}

std::map<int, int> liftMotorChannels(const log::Params& p) {
  std::map<int, int> m;
  for (int ch = 1; ch <= 32; ++ch) {
    const auto f = p.get("SERVO" + std::to_string(ch) + "_FUNCTION");
    if (!f) continue;
    const int fn = static_cast<int>(*f);
    if (fn >= 33 && fn <= 40 && !m.count(fn - 32)) m[fn - 32] = ch;
  }
  // a copter without SERVO functions logged: motors on channels 1..4
  if (m.empty() && !p.has("Q_ENABLE"))
    for (int i = 1; i <= 4; ++i) m[i] = i;
  return m;
}

// ------------------------------------------------------------------------------------------------- from a log

FlightRecord recordFromLog(log::LogData& lg, const std::string& name, std::optional<core::Family> family) {
  const std::vector<std::string> timed{"RATE", "PIDR", "PIDP", "PIDY", "PIQR", "PIQP", "PIQY", "ATT", "RCIN", "RCOU",
                                       "QTUN", "CTUN", "MODE", "MAG", "VIBE"};
  log::cleanLogTimes(lg, timed);
  FlightRecord r;
  r.source = "log";
  r.name = name;
  r.family = family ? *family : detectLogFamily(lg);
  r.params = lg.params;
  r.changes = parameterChanges(lg).all;
  r.text = lg.text;
  for (const auto& m : lg.text)
    if (std::regex_search(m.msg, std::regex("Ardu(Copter|Plane)", std::regex::icase))) {
      r.firmware = m.msg;
      break;
    }
  const auto sysid = r.params.get("SYSID_THISMAV") ? r.params.get("SYSID_THISMAV") : r.params.get("MAV_SYSID");
  r.identity = Json::obj({{"sysid", sysid ? Json(*sysid) : Json()}, {"firmwareText", r.firmware},
                          {"frameClass", Json(r.params.getOr(r.family == core::Family::Vtol ? "Q_FRAME_CLASS" : "FRAME_CLASS", 0))},
                          {"frameType", Json(r.params.getOr(r.family == core::Family::Vtol ? "Q_FRAME_TYPE" : "FRAME_TYPE", 0))}});

  const auto& fd = core::familyDef(r.family == core::Family::Heli ? core::Family::Copter : r.family);
  const log::MsgTable* rate = lg.msg("RATE");
  static const std::array<std::array<const char*, 3>, 3> rf{{{"RDes", "R", "ROut"}, {"PDes", "P", "POut"}, {"YDes", "Y", "YOut"}}};
  const log::MsgTable* att = lg.msg("ATT");
  static const std::array<std::array<const char*, 2>, 3> af{{{"DesRoll", "Roll"}, {"DesPitch", "Pitch"}, {"DesYaw", "Yaw"}}};
  for (int a = 0; a < 3; ++a) {
    AxisRec& x = r.axis[a];
    x.target = series(rate, rf[a][0]);
    x.actual = series(rate, rf[a][1]);
    x.out = series(rate, rf[a][2]);
    const log::MsgTable* pid = lg.msg(fd.logPid[a]);
    x.P = series(pid, "P");
    x.I = series(pid, "I");
    x.D = series(pid, "D");
    x.FF = series(pid, "FF");
    x.flags = series(pid, "Flags");
    x.dmod = series(pid, "Dmod");
    x.srate = series(pid, "SRate");
    // no RATE message: the PID log carries target / actual (deg/s) and the output is the sum of its terms
    if (x.actual.empty() && pid) {
      x.target = series(pid, "Tar");
      x.actual = series(pid, "Act");
      for (std::size_t i = 0; i < x.P.size(); ++i) {
        const double tt = x.P.t[i];
        x.out.push(tt, x.P.v[i] + x.I.at(tt) + x.D.at(tt) + (x.FF.empty() ? 0 : x.FF.at(tt)));
      }
    }
    x.angDes = series(att, af[a][0]);
    x.ang = series(att, af[a][1]);
  }
  // lift throttle: RATE.AOut (0..1) when logged, else the throttle stream of the analysis
  r.throttle = series(rate, "AOut");
  if (r.throttle.empty()) {
    if (const auto thr = throttleStream(lg, r.family)) {
      r.throttle.t = thr->t;
      r.throttle.v = thr->v;
    }
  }
  const int yawCh = static_cast<int>(r.params.getOr("RCMAP_YAW", 4));
  r.stickYaw = series(lg.msg("RCIN"), ("C" + std::to_string(yawCh)).c_str());
  for (const auto& [motor, ch] : liftMotorChannels(r.params)) {
    Series s = series(lg.msg("RCOU"), ("C" + std::to_string(ch)).c_str());
    if (!s.empty()) r.motors[motor] = std::move(s);
  }
  const log::MsgTable* qt = lg.msg(r.family == core::Family::Vtol ? "QTUN" : "CTUN");
  r.altDes = series(qt, "DAlt");
  r.alt = series(qt, "Alt");
  r.mode = series(lg.msg("MODE"), "ModeNum");
  if (r.mode.empty()) r.mode = series(lg.msg("MODE"), "Mode");
  {
    const log::MsgTable* mag = lg.msg("MAG");
    const auto sel = instance0(mag);
    const Series mx = series(mag, "MagX", 1, &sel), my = series(mag, "MagY", 1, &sel), mz = series(mag, "MagZ", 1, &sel);
    for (std::size_t i = 0; i < mx.size() && i < my.size() && i < mz.size(); ++i)
      r.magField.push(mx.t[i], std::sqrt(mx.v[i] * mx.v[i] + my.v[i] * my.v[i] + mz.v[i] * mz.v[i]));
  }
  {
    const log::MsgTable* vb = lg.msg("VIBE");
    const auto sel = instance0(vb);
    const Series vx = series(vb, "VibeX", 1, &sel), vy = series(vb, "VibeY", 1, &sel), vz = series(vb, "VibeZ", 1, &sel);
    for (std::size_t i = 0; i < vx.size() && i < vy.size() && i < vz.size(); ++i)
      r.vibe.push(vx.t[i], std::max({vx.v[i], vy.v[i], vz.v[i]}));
  }
  return r;
}

FlightRecord recordFromLogFile(const std::string& path, std::optional<core::Family> family) {
  log::LogData lg = log::parseBinFile(path, {"RATE", "PIDR", "PIDP", "PIDY", "PIQR", "PIQP", "PIQY", "ATT", "RCIN", "RCOU", "QTUN",
                                             "CTUN", "MODE", "MAG", "VIBE", "PARM", "MSG"});
  if (!lg.integrity.format.supported) throw std::runtime_error(lg.integrity.format.message.empty() ? "not a DataFlash log" : lg.integrity.format.message);
  std::string base = path;
  const auto cut = base.find_last_of("\\/");
  if (cut != std::string::npos) base = base.substr(cut + 1);
  return recordFromLog(lg, base, family);
}

// ------------------------------------------------------------------------------------------------- live telemetry

void TelemetryRecorder::setParams(const Json& params) {
  params_ = params;
  log::Params p;
  for (const auto& [k, v] : params.items())
    if (v.isNumber()) p.set(k, v.num());
  // a parameter that changes while recording is a change during the flight
  if (rec_.params.size() && t0_ >= 0) {
    for (const auto& [k, v] : p.items()) {
      const auto old = rec_.paramsAt(tLast_).get(k);
      if (!old || *old != v) rec_.changes.push_back({tLast_, k, old, v});
    }
  } else {
    rec_.params = p;
  }
  const auto sysid = p.get("SYSID_THISMAV") ? p.get("SYSID_THISMAV") : p.get("MAV_SYSID");
  rec_.identity.set("sysid", sysid ? Json(*sysid) : Json());
}

void TelemetryRecorder::clear() {
  const core::Family f = rec_.family;
  const log::Params p = rec_.paramsAt(tLast_);
  const Json id = rec_.identity;
  rec_ = FlightRecord();
  rec_.family = f;
  rec_.params = p;
  rec_.identity = id;
  t0_ = -1;
  tLast_ = 0;
}

double TelemetryRecorder::pidSeconds(int axis) const {
  const Series& s = rec_.axis[static_cast<std::size_t>(axis)].actual;
  return s.size() < 2 ? 0 : s.t.back() - s.t.front();
}

void TelemetryRecorder::onMessage(const std::string& type, const Json& f, double tS) {
  if (t0_ < 0) t0_ = tS;
  tLast_ = std::max(tLast_, tS);
  if (type == "PID_TUNING") {
    const int ax = static_cast<int>(f["axis"].numOr(0)) - 1;
    if (ax < 0 || ax > 2) return;
    // copter / VTOL rate PIDs report rad/s (planes deg/s)
    const double k = rec_.family == core::Family::Plane ? 1 : kR2D;
    AxisRec& x = rec_.axis[static_cast<std::size_t>(ax)];
    x.target.push(tS, f["desired"].numOr(0) * k);
    x.actual.push(tS, f["achieved"].numOr(0) * k);
    const double P = f["P"].numOr(0), I = f["I"].numOr(0), D = f["D"].numOr(0), FF = f["FF"].numOr(0);
    x.P.push(tS, P);
    x.I.push(tS, I);
    x.D.push(tS, D);
    x.FF.push(tS, FF);
    x.out.push(tS, P + I + D + FF);
    if (f.has("SRate")) x.srate.push(tS, f["SRate"].numOr(0));
    if (f.has("PDmod")) x.dmod.push(tS, f["PDmod"].numOr(1));
  } else if (type == "ATTITUDE") {
    rec_.axis[0].ang.push(tS, f["roll"].numOr(0) * kR2D);
    rec_.axis[1].ang.push(tS, f["pitch"].numOr(0) * kR2D);
    double y = f["yaw"].numOr(0) * kR2D;
    if (y < 0) y += 360;
    rec_.axis[2].ang.push(tS, y);
  } else if (type == "NAV_CONTROLLER_OUTPUT") {
    rec_.axis[0].angDes.push(tS, f["nav_roll"].numOr(0));
    rec_.axis[1].angDes.push(tS, f["nav_pitch"].numOr(0));
  } else if (type == "RC_CHANNELS") {
    const int ch = static_cast<int>(rec_.params.getOr("RCMAP_YAW", 4));
    const Json& v = f["chan" + std::to_string(ch) + "_raw"];
    if (v.isNumber() && v.num() > 0 && v.num() < 65535) rec_.stickYaw.push(tS, v.num());
  } else if (type == "SERVO_OUTPUT_RAW") {
    const int port = static_cast<int>(f["port"].numOr(0));
    for (const auto& [motor, ch] : liftMotorChannels(rec_.params)) {
      const int local = ch - port * 8;
      if (local < 1 || local > 16) continue;
      const Json& v = f["servo" + std::to_string(local) + "_raw"];
      if (v.isNumber() && v.num() > 0) rec_.motors[motor].push(tS, v.num());
    }
    // lift throttle from the mean motor output (PWM range Q_M_PWM_MIN / MAX or MOT_PWM_MIN / MAX)
    const bool q = rec_.family == core::Family::Vtol;
    const double lo = rec_.params.getOr(q ? "Q_M_PWM_MIN" : "MOT_PWM_MIN", 0) > 0 ? rec_.params.getOr(q ? "Q_M_PWM_MIN" : "MOT_PWM_MIN", 1000) : 1000;
    const double hi = rec_.params.getOr(q ? "Q_M_PWM_MAX" : "MOT_PWM_MAX", 0) > 0 ? rec_.params.getOr(q ? "Q_M_PWM_MAX" : "MOT_PWM_MAX", 2000) : 2000;
    double sum = 0;
    int n = 0;
    for (auto& [m, s] : rec_.motors)
      if (!s.empty() && s.t.back() == tS) {
        sum += s.v.back();
        ++n;
      }
    if (n) rec_.throttle.push(tS, std::clamp((sum / n - lo) / (hi - lo), 0.0, 1.0));
  } else if (type == "HEARTBEAT" || type == "heartbeat") {
    // only the autopilot's heartbeat (type 1..: not a GCS = 6)
    if (f["type"].numOr(0) == 6) return;
    armed_ = (static_cast<int>(f["base_mode"].numOr(0)) & 128) != 0 || f["armed"].asBool();
    rec_.armed.push(tS, armed_ ? 1 : 0);
    rec_.mode.push(tS, f["custom_mode"].numOr(-1));
  } else if (type == "VFR_HUD") {
    rec_.alt.push(tS, f["alt"].numOr(0));
  } else if (type == "STATUSTEXT") {
    rec_.text.push_back({tS, f["text"].strOr("")});
  } else if (type == "VIBRATION") {
    rec_.vibe.push(tS, std::max({f["vibration_x"].numOr(0), f["vibration_y"].numOr(0), f["vibration_z"].numOr(0)}));
  }
}

FlightRecord TelemetryRecorder::finish(const std::string& name) const {
  FlightRecord r = rec_;
  r.source = "telemetry";
  r.name = name;
  return r;
}

}  // namespace aptuner::engine::heavy
