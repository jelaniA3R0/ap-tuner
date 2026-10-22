#include "server.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <regex>
#include <set>
#include <thread>

#include "aptuner/core/build_info.hpp"
#include "aptuner/core/jsnum.hpp"
#include "aptuner/engine/identity.hpp"
#include "aptuner/engine/loganalysis.hpp"
#include "aptuner/engine/report.hpp"
#include "aptuner/engine/vehicle.hpp"
#include "aptuner/io/autoconnect.hpp"
#include "aptuner/io/link.hpp"
#include "aptuner/io/sysinfo.hpp"
#include "aptuner/rt/loop.hpp"
#include "common.hpp"
#include "flight.hpp"
#include "groundtune.hpp"
#include "heavylive.hpp"
#include "paramwrite.hpp"
#include "aptuner/core/parammeta.hpp"
#include "aptuner/engine/heavytune.hpp"
#include "http.hpp"
#include "journal.hpp"
#include "notchlive.hpp"
#include "tuner.hpp"

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

namespace aptuner::app {

using core::Json;
using rt::Loop;

namespace fs = std::filesystem;

// ------------------------------------------------------------------------------------------------ helpers

std::string findWebDir() {
  fs::path d = fsPath(io::exeDir());
  for (int i = 0; i < 6 && !d.empty(); i++) {
    if (fs::exists(d / "web" / "index.html")) return pathStr(d / "web");
    if (d == d.parent_path()) break;
    d = d.parent_path();
  }
  return "";
}

std::intptr_t openAppWindow(const std::string& url, bool appWindow) {
#ifdef _WIN32
  if (appWindow) {
    std::string edge;
    for (const char* p : {"C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe", "C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe"})
      if (fs::exists(p)) edge = p;
    if (!edge.empty()) {
      const char* la = std::getenv("LOCALAPPDATA");
      const std::string profile = std::string(la ? la : ".") + "\\AP Tuner\\edge-profile";
      std::string cmd = "\"" + edge + "\" --app=" + url + " --window-size=1400,920 --user-data-dir=\"" + profile + "\"";
      STARTUPINFOA si{};
      si.cb = sizeof si;
      PROCESS_INFORMATION pi{};
      if (CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &pi)) {
        CloseHandle(pi.hThread);
        return reinterpret_cast<std::intptr_t>(pi.hProcess);
      }
    }
  }
  ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
  return 0;
#else
  (void)appWindow;
#ifdef __APPLE__
  std::system(("open '" + url + "'").c_str());
#else
  std::system(("xdg-open '" + url + "' >/dev/null 2>&1 &").c_str());
#endif
  return 0;
#endif
}

bool processAlive(std::intptr_t handle) {
#ifdef _WIN32
  if (!handle) return false;
  return WaitForSingleObject(reinterpret_cast<HANDLE>(handle), 0) == WAIT_TIMEOUT;
#else
  (void)handle;
  return false;
#endif
}

const char* const kPageCsp =
    "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; "
    "base-uri 'none'; form-action 'none'; frame-ancestors 'none'";

namespace {

const std::map<std::string, std::string> kMime{{".html", "text/html; charset=utf-8"}, {".js", "text/javascript"}, {".css", "text/css"},
                                                {".png", "image/png"}, {".svg", "image/svg+xml"}, {".ico", "image/x-icon"},
                                                {".woff2", "font/woff2"}, {".json", "application/json"}};

HttpResponse jsonResp(int code, const Json& j) {
  HttpResponse r;
  r.status = code;
  r.body = j.dump();
  r.headers.push_back({"X-Content-Type-Options", "nosniff"});
  return r;
}

// API routes that only read: GET, no token. Everything else under /api/ changes state: POST + token.
const std::set<std::string> kReadRoutes{"/api/events",   "/api/ports",         "/api/presets",       "/api/notch/status", "/api/live/rows", "/api/live/param",
                                        "/api/log/list", "/api/log/param",     "/api/ground/result", "/api/ground/state", "/api/ground/file", "/api/journal",
                                        "/api/heavy/state", "/api/heavy/result", "/api/heavy/file"};

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

/** constant-time comparison (the time does not tell how much of a guessed token was right) */
bool sameToken(const std::string& a, const std::string& b) {
  if (a.size() != b.size() || b.empty()) return false;
  unsigned char d = 0;
  for (std::size_t i = 0; i < a.size(); i++) d = static_cast<unsigned char>(d | (static_cast<unsigned char>(a[i]) ^ static_cast<unsigned char>(b[i])));
  return d == 0;
}
}  // namespace

std::optional<HttpResponse> gateRequest(const HttpRequest& req, int port, const std::string& token) {
  const std::string p = std::to_string(port);
  const std::string host = lower(req.header("host"));
  if (host != "127.0.0.1:" + p && host != "localhost:" + p) return jsonResp(403, Json::obj({{"error", "forbidden: unknown Host"}}));
  if (req.headers.count("origin")) {
    const std::string origin = lower(req.header("origin"));
    if (origin != "http://127.0.0.1:" + p && origin != "http://localhost:" + p) return jsonResp(403, Json::obj({{"error", "forbidden: request from another site"}}));
  }
  if (req.header("sec-fetch-site") == "cross-site") return jsonResp(403, Json::obj({{"error", "forbidden: request from another site"}}));
  if (req.path.rfind("/api/", 0) != 0) return std::nullopt;
  const bool write = !kReadRoutes.count(req.path) || (req.path == "/api/log/param" && req.param("save") == "1");
  if (!write) {
    if (req.method == "GET" || req.method == "HEAD") return std::nullopt;
    HttpResponse r = jsonResp(405, Json::obj({{"error", "use GET"}}));
    r.headers.push_back({"Allow", "GET"});
    return r;
  }
  if (req.method != "POST") {
    HttpResponse r = jsonResp(405, Json::obj({{"error", "this request changes something: it must be a POST"}}));
    r.headers.push_back({"Allow", "POST"});
    return r;
  }
  if (!sameToken(req.header("x-aptuner-token"), token))
    return jsonResp(403, Json::obj({{"error", "missing or wrong app token: reload the page (the app may have been restarted)"}}));
  return std::nullopt;
}

namespace {

Json listLogs(const std::string& sessions) {
  std::vector<fs::path> dirs{fs::path("D:\\")};
  const char* home = std::getenv("USERPROFILE");
  if (!home) home = std::getenv("HOME");
  if (home) {
    dirs.push_back(fsPath(home) / "Documents" / "Mission Planner" / "logs");
    dirs.push_back(fsPath(home) / "Desktop");
    dirs.push_back(fsPath(home) / "Downloads");
  }
  dirs.push_back(fsPath(sessions) / "uploads");
  struct Item {
    std::string path;
    double size, mtime;
  };
  std::vector<Item> out;
  std::function<void(const fs::path&, int)> walk = [&](const fs::path& d, int depth) {
    std::error_code ec;
    fs::directory_iterator it(d, fs::directory_options::skip_permission_denied, ec);
    if (ec) return;
    for (const auto& e : it) {
      std::error_code e2;
      const std::string name = pathStr(e.path().filename());
      if (e.is_directory(e2)) {
        if (depth > 0 && !name.empty() && name[0] != '$' && name != "System Volume Information") walk(e.path(), depth - 1);
      } else if (name.size() > 4) {
        std::string ext = name.substr(name.size() - 4);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (ext != ".bin") continue;
        const auto sz = e.file_size(e2);
        const auto mt = e.last_write_time(e2);
        if (e2) continue;
        const auto sys = std::chrono::time_point_cast<std::chrono::system_clock::duration>(mt - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
        const double ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(sys.time_since_epoch()).count());
        out.push_back({pathStr(e.path()), static_cast<double>(sz), ms});
      }
    }
  };
  for (const auto& d : dirs) walk(d, 2);
  std::set<std::string> seen;
  std::vector<Item> uniq;
  for (const auto& x : out) {
    std::string k = x.path;
    std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (seen.insert(k).second) uniq.push_back(x);
  }
  std::stable_sort(uniq.begin(), uniq.end(), [](const Item& a, const Item& b) { return b.mtime < a.mtime; });
  if (uniq.size() > 60) uniq.resize(60);
  Json arr = Json::array();
  for (const auto& x : uniq) arr.push(Json::obj({{"path", x.path}, {"size", x.size}, {"mtime", x.mtime}}));
  return arr;
}

Json sanitizeOpts(const Json& b) {
  Json o = Json::object();
  const std::string pr = b["preset"].strOr("");
  if (pr == "conservative" || pr == "normal" || pr == "aggressive") o.set("preset", pr);
  if (b.has("autoApply")) o.set("autoApply", b["autoApply"].asBool());
  if (b["axes"].isArray() && b["axes"].size()) {
    Json a = Json::array();
    for (const auto& x : b["axes"].arr())
      if (x.isString() && core::parseAxis(x.str())) a.push(x);
    o.set("axes", a);
  }
  if (b["observeSeconds"].numOr(0) != 0) o.set("observeSeconds", std::max(15.0, std::min(120.0, b["observeSeconds"].numOr(30))));
  const std::string size = b["size"].strOr("");
  const std::map<std::string, double> sizeOsc{{"small", 5}, {"medium", 5}, {"large", 3}, {"heavy", 1.5}};
  if (sizeOsc.count(size)) {
    o.set("size", size);
    o.set("oscSmax", sizeOsc.at(size));
  }
  if (b["oscSmax"].numOr(0) != 0) o.set("oscSmax", std::max(1.0, std::min(20.0, b["oscSmax"].numOr(5))));
  if (b.has("allowRamp")) o.set("allowRamp", b["allowRamp"].asBool());
  if (b["stepMax"].numOr(0) != 0) o.set("stepMax", std::max(1.1, std::min(2.0, b["stepMax"].numOr(1.5))));
  if (b["watchSeconds"].numOr(0) != 0) o.set("watchSeconds", std::max(2.0, std::min(15.0, b["watchSeconds"].numOr(4))));
  if (b.has("inFlightWrites")) o.set("inFlightWrites", b["inFlightWrites"].isBool() && b["inFlightWrites"].asBool());
  return o;
}

/** the running application (all members are used on the loop thread unless noted) */
class App {
 public:
  explicit App(const AppOptions& o)
      : opts_(o),
        token_(randomHex(24)),
        journal(pathStr(fsPath(o.sessionsDir) / "journal")),   // one append-only write journal for this run
        tuner(link, pathStr(fsPath(o.sessionsDir)), &journal),
        notch(link, tuner),
        flight(link, tuner, notch),
        ground(link, o.sessionsDir, &journal),
        heavy(link, o.sessionsDir) {
    heavy.on("state", [this](const Json& s) { hub.publish("heavy", s.dump()); });
    heavy.on("progress", [this](const Json& p) { hub.publish("heavyProgress", p.dump()); });
    heavy.onResult = [this](const Json& r) {
      lastHeavy = r;
      hub.publish("heavyResult", Json::obj({{"ok", !r.has("error")}}).dump());
    };
    link.on("status", [this](const Json& s) { tuner.log("link", s.strOr("")); });
    tuner.on("log", [this](const Json& e) { hub.publish("log", e.dump()); });
    tuner.on("state", [this](const Json& s) { hub.publish("state", s.dump()); });
    tuner.on("analysis", [this](const Json& r) { hub.publish("analysis", r.dump()); });
    tuner.on("param", [this](const Json& p) { hub.publish("param", p.dump()); });
    tuner.on("progress", [this](const Json& p) { hub.publish("progress", p.dump()); });
    flight.on("phase", [this](const Json& p) { hub.publish("flight", p.dump()); });
    ground.on("phase", [this](const Json& p) { hub.publish("ground", p.dump()); });
  }

  void startTimers() {
    Loop::main().setInterval(250, [this] { publishSnapshot(); });
    tuner.startMonitor();
  }

  /** the tuner options from a request; turning the in-flight opt-in on or off is logged */
  void setOpts(const Json& body) {
    const Json o = sanitizeOpts(body);
    if (o.has("inFlightWrites") && o["inFlightWrites"].asBool() != tuner.opts["inFlightWrites"].asBool())
      tuner.log("tuner", o["inFlightWrites"].asBool() ? "changes in flight turned ON (experimental, tested only in the simulator)" : "changes in flight turned off");
    for (const auto& [k, v] : o.items()) tuner.opts.set(k, v);
  }

  void publishSnapshot() {
    if (!hub.count()) return;
    Json snap = tuner.snapshot();
    Json series = Json::object();
    for (const auto& ax : kAxisNames) {
      const LiveAxis& la = tuner.axes[ax];
      const std::size_t n = la.t.size(), keep = 600;
      const std::size_t step = std::max<std::size_t>(1, std::min<std::size_t>(n, 2500) / keep);
      const std::size_t i0 = n > keep * step ? n - keep * step : 0;
      Json t = Json::array(), tar = Json::array(), act = Json::array(), P = Json::array(), I = Json::array(), D = Json::array(),
           FF = Json::array(), sr = Json::array(), dm = Json::array();
      auto r = [](double v, int d) { return core::jsRoundFixed(v, d); };
      for (std::size_t i = i0; i < n; i += step) {
        t.push(r(la.t[i], 3));
        tar.push(r(la.target[i], 2));
        act.push(r(la.actual[i], 2));
        P.push(r(la.P[i], 4));
        I.push(r(la.I[i], 4));
        D.push(r(la.D[i], 4));
        FF.push(r(la.FF[i], 4));
        sr.push(r(la.srate[i], 2));
        dm.push(r(la.dmod[i], 3));
      }
      series.set(ax, Json::obj({{"t", t}, {"target", tar}, {"actual", act}, {"P", P}, {"I", I}, {"D", D}, {"FF", FF}, {"srate", sr}, {"dmod", dm}}));
    }
    snap.set("series", series);
    snap.set("connected", link.connected());
    snap.set("alive", link.alive());
    snap.set("linkDesc", link.description());
    snap.set("auto", autoJson());
    snap.set("escTelemetry", notch.escSeen() && Loop::now() - notch.escLastTs() < 20000);
    snap.set("flight", flight.snapshot());
    snap.set("ground", ground.snapshot());
    hub.publish("snapshot", snap.dump());
  }

  Json autoJson() const {
    Json j = Json::obj({{"running", autoState.running}, {"tried", autoState.tried}, {"found", autoState.found}});
    if (!autoState.running && (autoState.tried.size() || !autoState.found.isNull())) {
      j.set("missionPlanner", autoState.missionPlanner);
      j.set("busyPorts", autoState.busyPorts);
    }
    return j;
  }

  /** the live session's values: every tuned gain and the notch block, with the value read at the start (changed: differs) */
  Json liveRows() {
    Json rows = Json::array();
    const Json snap = tuner.snapshot();
    if (!tuner.family || !snap["gains"].isObject()) return rows;
    for (const auto& ax : kAxisNames) {
      const Json n = engine::axisParamNames(*tuner.family, *core::parseAxis(ax));
      const Json& g = snap["gains"][ax];
      const Json& o = tuner.originals[ax];
      for (const char* k : {"P", "I", "D", "FF", "FLTT", "FLTE", "FLTD", "SMAX", "IMAX", "angleP", "tconst"}) {
        if (!n[k].isString() || !g[k].isNumber()) continue;
        rows.push(Json::obj({{"group", ax}, {"name", n[k]}, {"value", g[k]}, {"from", o[k]}, {"label", k}, {"changed", o[k].isNumber() && !engine::sameF32(o[k], g[k])}}));
      }
    }
    const Json& at = tuner.paramsAtPrepare;
    for (const char* k : {"INS_HNTCH_ENABLE", "INS_HNTCH_MODE", "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTCH_ATT", "INS_HNTCH_REF", "INS_HNTCH_HMNCS",
                          "INS_HNTCH_OPTS", "INS_HNTCH_FM_RAT", "INS_GYRO_FILTER"})
      if (tuner.params[k].isNumber())
        rows.push(Json::obj({{"group", "notch"}, {"name", k}, {"value", tuner.params[k]}, {"from", at[k]}, {"label", k},
                             {"changed", at[k].isNumber() && !engine::sameF32(at[k], tuner.params[k])}}));
    return rows;
  }
  /** the .param text of the values a session changed (checked, fixed point, the vehicle named in the header) */
  engine::ParamFileOut liveParamFile() {
    Json rows = Json::array();
    const Json all = liveRows();
    for (const auto& r : all.arr())
      if (r["changed"].asBool()) rows.push(r);
    std::vector<std::string> head{"Live session values read from the vehicle on " + localNow() + ": the " + core::jsToString(static_cast<double>(rows.size())) +
                                  " value(s) this session changed"};
    if (!rows.size()) head.push_back("Nothing was changed in this session.");
    engine::ParamFileOut out = engine::paramFileChecked(rows, head, tuner.identity.isObject() ? tuner.identity : link.identity());
    Json list = Json::array();
    for (const auto& r : rows.arr()) list.push(Json::obj({{"name", r["name"]}, {"value", r["value"]}}));
    journal.append("export", Json::obj({{"what", "live .param"}, {"rows", list}, {"errors", out.errors}, {"warnings", out.warnings},
                                        {"identity", tuner.identity.isObject() ? tuner.identity["key"] : Json()}}));
    return out;
  }
  /** the .param text of a log analysis (checked; the vehicle is the one in the log) */
  engine::ParamFileOut logParamFile(const Json& r) {
    const std::string base = pathStr(fsPath(r["file"].strOr("log.bin")).filename());
    const std::string who = engine::logIdentityText(r);
    std::string presetName = "normal";
    for (const auto& [k, a] : r["axes"].items())
      if (a["preset"].isString()) {
        presetName = a["preset"].str();
        break;
      }
    engine::ParamFileOut out = engine::paramFileChecked(
        r["rows"],
        {"From log " + base + " (" + r["firmware"].strOr("") + "), " + r["family"].strOr("") + ", preset " + presetName,
         "Fly carefully after loading: one axis at a time, ready to switch back to your previous parameters."},
        Json(who));
    Json list = Json::array();
    for (const auto& x : r["rows"].arr()) list.push(Json::obj({{"name", x["name"]}, {"value", x["value"]}}));
    journal.append("export", Json::obj({{"what", "log .param"}, {"file", r["file"]}, {"rows", list}, {"errors", out.errors}, {"warnings", out.warnings}}));
    return out;
  }

  /** API handlers that run on the loop thread */
  rt::Task<HttpResponse> api(std::string path, Json body, HttpRequest req) {
    auto ok = [](Json extra = Json::object()) {
      Json j = Json::obj({{"ok", true}});
      for (const auto& [k, v] : extra.items()) j.set(k, v);
      return jsonResp(200, j);
    };
    if (path == "/api/autoconnect") {
      if (autoState.running) co_return jsonResp(200, Json::obj({{"ok", false}, {"running", true}}));
      const Json found = co_await io::autoconnect(link, autoState, [this](const Json& p) { hub.publish("connect", p.dump()); });
      if (found.isObject()) co_return jsonResp(200, Json::obj({{"ok", true}, {"found", found}}));
      co_return jsonResp(200, Json::obj({{"ok", false}, {"tried", autoState.tried}, {"missionPlanner", autoState.missionPlanner}, {"busyPorts", autoState.busyPorts}}));
    }
    if (path == "/api/connect") {
      const std::string kind = body["kind"].strOr("tcp");
      Json o = Json::obj({{"kind", kind}, {"host", body["host"].strOr("127.0.0.1")}});
      if (kind == "serial")
        o.set("port", body["port"].strOr(""));
      else
        o.set("port", std::floor(body["port"].numOr(14550)));
      o.set("baud", std::floor(body["baud"].numOr(115200)));
      link.connect(o);
      co_return ok();
    }
    if (path == "/api/disconnect") {
      link.disconnect();
      co_return ok();
    }
    if (path == "/api/ports") {
      Json a = Json::array();
      for (const auto& p : io::listSerialPorts()) a.push(Json::obj({{"port", p.port}, {"name", p.name}}));
      co_return jsonResp(200, a);
    }
    if (path == "/api/prepare") {
      const Json r = co_await tuner.prepare(body["confirm"].asBool());
      Json j = Json::obj({{"ok", true}});
      for (const auto& [k, v] : r.items()) j.set(k, v);
      co_return jsonResp(200, j);
    }
    if (path == "/api/start" || path == "/api/options") {
      setOpts(body);
      if (path == "/api/start") tuner.start();
      co_return ok(Json::obj({{"opts", tuner.opts}}));
    }
    if (path == "/api/pause") {
      tuner.pause("paused by user");
      co_return ok();
    }
    if (path == "/api/resume") {
      tuner.resume();
      co_return ok();
    }
    if (path == "/api/stop") {
      co_await tuner.stop();
      co_return ok();
    }
    if (path == "/api/apply") {
      // the approval names the proposal it showed (id + hash): another proposal is not written (F-33)
      const bool applied = co_await tuner.applyProposal(body["axis"].strOr(""), Json::obj({{"id", body["id"]}, {"hash", body["hash"]}}));
      if (applied && tuner.state != "paused") tuner.resume();
      co_return jsonResp(200, Json::obj({{"ok", applied}}));
    }
    if (path == "/api/undo") {
      co_await tuner.undoAxis(body["axis"].strOr(""));
      co_return ok();
    }
    if (path == "/api/restore") {
      co_await tuner.restoreOriginals();
      co_return ok();
    }
    if (path == "/api/analyze") {
      const std::string ax = body["axis"].isString() ? body["axis"].str() : tuner.currentAxis();
      co_return jsonResp(200, tuner.analyze(ax, body["seconds"].numOr(0)));
    }
    // (no /api/setparam: arbitrary parameter writes are not offered, F-05)
    if (path == "/api/presets") {
      Json p = Json::object();
      for (const auto& pr : core::kPresets)
        p.set(pr.name, Json::obj({{"gmMinDb", pr.gmMinDb}, {"pmMinDeg", pr.pmMinDeg}, {"msMax", pr.msMax}, {"osMaxPct", pr.osMaxPct},
                                  {"backoff", pr.backoff}, {"label", pr.label}}));
      co_return jsonResp(200, p);
    }
    if (path == "/api/notch/status") co_return jsonResp(200, co_await notch.status());
    if (path == "/api/notch/esc") {
      const Json applied = co_await notch.applyEsc();
      co_return ok(Json::obj({{"applied", applied}, {"rebootNeeded", notch.rebootNeeded()}}));
    }
    if (path == "/api/notch/fft/enable") co_return ok(co_await notch.enableFft());
    if (path == "/api/notch/reboot") co_return jsonResp(200, co_await notch.reboot());
    if (path == "/api/notch/fft/start") co_return ok(co_await notch.startFftTune());
    if (path == "/api/notch/fft/stop") co_return ok(co_await notch.stopFftTune());
    if (path == "/api/flight/start") {
      setOpts(body);
      flight.start(Json::obj({{"axes", tuner.opts["axes"]}, {"preset", tuner.opts["preset"]}, {"auto", body["auto"].asBool()},
                              {"observeSeconds", tuner.opts["observeSeconds"]}}));
      co_return ok(Json::obj({{"flight", flight.snapshot()}}));
    }
    if (path == "/api/flight/continue") {
      flight.cont();
      co_return ok();
    }
    if (path == "/api/flight/repeat") {
      flight.repeatCheckup();
      co_return ok();
    }
    if (path == "/api/flight/skip") {
      flight.skipFilters();
      co_return ok();
    }
    if (path == "/api/flight/resume") {
      flight.resume();
      co_return ok();
    }
    if (path == "/api/flight/pause") {
      tuner.pause("paused by you");
      co_return ok();
    }
    if (path == "/api/flight/stop") {
      flight.stop();
      co_return ok();
    }
    if (path == "/api/ground/start") {
      // developer hook: APT_TEST_USB=5763 makes a SITL TCP port stand in for the USB cable
      Json testUsb = body["testUsb"];
      const char* envUsb = std::getenv("APT_TEST_USB");
      if (!testUsb.isObject() && envUsb) testUsb = Json::obj({{"kind", "tcp"}, {"port", envUsb}});
      Json axes;
      if (body["axes"].isArray()) {
        axes = Json::array();
        for (const auto& a : body["axes"].arr())
          if (a.isString() && core::parseAxis(a.str())) axes.push(a);
        if (!axes.size()) axes = Json();
      }
      Json usb;
      if (testUsb.isObject() && testUsb["kind"].strOr("") == "tcp") {
        const double port = std::floor(testUsb["port"].numOr(0));
        usb = Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", port}, {"label", "test TCP " + core::jsToString(port)}});
      }
      const double secs = std::max(15.0, std::min(90.0, std::floor(body["seconds"].numOr(30))));
      ground.start(Json::obj({{"axes", axes}, {"preset", body["preset"].strOr("normal")}, {"seconds", secs}, {"usb", usb}}));
      co_return ok();
    }
    if (path == "/api/ground/again") {
      ground.captureAgain();
      co_return ok();
    }
    if (path == "/api/ground/usb") {
      const Json& t = body["testUsb"];
      if (t.isObject() && t["kind"].strOr("") == "tcp") {
        const double port = std::floor(t["port"].numOr(0));
        ground.opts.set("usb", Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", port}, {"label", "test TCP " + core::jsToString(port)}}));
      }
      co_await ground.goUsb(body["useSaved"].asBool());
      co_return ok();
    }
    if (path == "/api/ground/write") {
      std::vector<std::string> names;
      for (const auto& n : body["names"].arr())
        if (n.isString()) names.push_back(n.str());
      co_return ok(Json::obj({{"results", co_await ground.write(names, Json::obj({{"id", body["id"]}, {"hash", body["hash"]}}))}}));
    }
    if (path == "/api/ground/undo") {
      co_await ground.undo();
      co_return ok();
    }
    if (path == "/api/ground/recheck") {
      co_await ground.recheck();
      co_return ok();
    }
    if (path == "/api/ground/reboot") co_return ok(co_await ground.rebootNow());
    if (path == "/api/journal") {
      Json entries = Json::array();
      const auto& rec = journal.recent();
      for (std::size_t i = rec.size() > 200 ? rec.size() - 200 : 0; i < rec.size(); i++) entries.push(rec[i]);
      Json j = Json::obj({{"entries", entries}});
      const Json st = journal.status();
      for (const auto& [k, v] : st.items()) j.set(k, v);
      co_return jsonResp(200, j);
    }
    if (path == "/api/ground/stop") {
      ground.stop();
      co_return ok();
    }
    if (path == "/api/ground/result") co_return jsonResp(200, ground.result.isObject() ? ground.result : Json::object());
    if (path == "/api/ground/state") co_return jsonResp(200, ground.snapshot());
    if (path == "/api/ground/file") {
      const Json& fp = ground.files[req.param("which")];
      if (!fp.isString() || !fs::exists(fsPath(fp.str()))) co_return jsonResp(404, Json::obj({{"error", "no file"}}));
      HttpResponse r;
      r.contentType = "application/octet-stream";
      r.filePath = fp.str();
      r.headers.push_back({"Content-Disposition", "attachment; filename=\"" + pathStr(fsPath(fp.str()).filename()) + "\""});
      co_return r;
    }
    if (path == "/api/live/rows") co_return jsonResp(200, Json::obj({{"family", familyJson(tuner.family)}, {"rows", liveRows()}, {"journal", tuner.journal}}));
    if (path == "/api/live/param") {
      HttpResponse r;
      r.contentType = "text/plain";
      r.body = liveParamFile().text;
      r.headers.push_back({"Content-Disposition", "attachment; filename=\"AP Tuner live gains.param\""});
      co_return r;
    }
    if (path == "/api/log/param") {
      if (!lastLogResult.isObject()) co_return jsonResp(400, Json::obj({{"error", "analyse a log first"}}));
      std::string presetName = "normal";
      for (const auto& [k, a] : lastLogResult["axes"].items())
        if (a["preset"].isString()) {
          presetName = a["preset"].str();
          break;
        }
      const std::string file = lastLogResult["file"].strOr("log.bin");
      const std::string base = pathStr(fsPath(file).filename());
      const engine::ParamFileOut pf = logParamFile(lastLogResult);
      const std::string txt = pf.text;
      static const std::regex binRe("\\.bin$", std::regex::icase);
      if (req.param("save") == "1") {
        const std::string out = std::regex_replace(file, binRe, "") + " - AP Tuner.param";
        writeText(out, txt);
        co_return jsonResp(200, Json::obj({{"ok", true}, {"file", out}, {"errors", pf.errors}, {"warnings", pf.warnings}}));
      }
      HttpResponse r;
      r.contentType = "text/plain";
      r.body = txt;
      r.headers.push_back({"Content-Disposition", "attachment; filename=\"" + std::regex_replace(base, binRe, "") + " - AP Tuner.param\""});
      co_return r;
    }
    // ---- model tuning: check edited values, exact files, write a step to the drone (two stages, verified)
    if (path == "/api/heavy/paramfile") {
      if (!lastHeavy.isObject()) co_return jsonResp(400, Json::obj({{"error", "analyse a flight first"}}));
      const Json ev = engine::heavy::evaluateSettings(lastHeavy, body["rows"]);
      Json rows = Json::array();
      for (const auto& r : ev["rows"].arr())
        if (r["ok"].asBool()) {
          const Json fp = lastHeavy["flownParams"][r["name"].strOr("")];
          Json row = Json::obj({{"group", body["title"].strOr("settings")}, {"name", r["name"]}, {"value", r["value"]}, {"label", ""}});
          if (fp.isNumber()) row.set("from", fp);
          rows.push(row);
        }
      const std::string title = body["title"].strOr("Model tuning step");
      const Json who = Json(engine::logIdentityText(Json::obj({{"file", lastHeavy["flights"][0]["name"]},
                                                               {"params", Json::obj({{"SYSID_THISMAV", lastHeavy["vehicle"]["sysid"]}})},
                                                               {"firmware", lastHeavy["vehicle"]["firmware"]}})));
      const engine::ParamFileOut pf = engine::paramFileChecked(rows, {title + " - values as shown in AP Tuner", "Proposals only: check the values before writing."}, who, true);
      journal.append("export", Json::obj({{"what", "model-tuning .param (as shown)"}, {"rows", rows}, {"errors", pf.errors}}));
      co_return jsonResp(200, Json::obj({{"text", pf.text}, {"errors", pf.errors}, {"warnings", pf.warnings}, {"check", ev}}));
    }
    if (path == "/api/heavy/writeplan") {
      if (!lastHeavy.isObject()) throw rt::Error("analyse a flight first");
      if (!link.alive()) throw rt::Error("the drone is not connected: press Connect (Mission Planner is found by itself)");
      if (link.vehicle()["armed"].asBool()) throw rt::Error("the drone is armed: land and disarm first. Values are only written on the ground");
      const Json id = link.identity();
      const Json want = lastHeavy["vehicle"]["sysid"];
      if (!want.isNumber() || !(id["sysid"] == want))
        throw rt::Error("the connected autopilot is system id " + id["sysid"].dump() + ", the analysed flights come from system id " + want.dump() +
                        ": write only to the drone the flights came from");
      // only the parameters of the step (values may be edited, names may not)
      const int n = static_cast<int>(body["step"].numOr(0));
      const Json* st = nullptr;
      if (lastHeavy["steps"].isArray())
        for (const auto& s : lastHeavy["steps"].arr())
        if (static_cast<int>(s["n"].numOr(0)) == n) st = &s;
      if (!st) throw rt::Error("no such step");
      std::set<std::string> allowed;
      for (const auto& r : (*st)["rows"].arr()) allowed.insert(r["name"].strOr(""));
      for (const auto& r : body["rows"].isArray() ? body["rows"].arr() : Json::Array{})
        if (!allowed.count(r["name"].strOr(""))) throw rt::Error(r["name"].strOr("?") + " is not part of step " + std::to_string(n));
      const Json ev = engine::heavy::evaluateSettings(lastHeavy, body["rows"]);
      for (const auto& r : ev["rows"].arr())
        if (!r["ok"].asBool()) throw rt::Error(r["name"].strOr("?") + " " + (r["errors"].size() ? r["errors"][0].strOr("") : std::string("is not valid")));
      // the values on the drone right now
      std::vector<std::string> names;
      for (const auto& r : ev["rows"].arr()) names.push_back(r["name"].str());
      const Json now = co_await link.requestParams(names, 4);
      Json rows = Json::array();
      std::string canon;
      for (const auto& r : ev["rows"].arr()) {
        const std::string nm = r["name"].str();
        if (!now[nm].isNumber()) throw rt::Error(nm + " could not be read from the drone");
        const std::string vt = core::formatParamValue(nm, now[nm].num());
        rows.push(Json::obj({{"name", nm}, {"vehicle", now[nm]}, {"vehicleText", vt}, {"to", r["value"]}, {"text", r["text"]}, {"same", vt == r["text"].str()}}));
        canon += nm + "=" + r["text"].str() + ";";
      }
      std::uint64_t h = 1469598103934665603ULL;
      for (unsigned char ch : canon) h = (h ^ ch) * 1099511628211ULL;
      char hx[20];
      std::snprintf(hx, sizeof hx, "%016llx", static_cast<unsigned long long>(h));
      heavyPending = Json::obj({{"id", randomHex(8)}, {"hash", std::string(hx)}, {"step", n}, {"rows", rows}, {"gen", link.generation()}, {"sysid", id["sysid"]},
                                {"compid", id["compid"]}, {"check", ev["axes"]}});
      co_return jsonResp(200, heavyPending);
    }
    if (path == "/api/heavy/write") {
      const Json p = heavyPending;
      if (!p.isObject() || body["id"].strOr("") != p["id"].strOr("x") || body["hash"].strOr("") != p["hash"].strOr("x"))
        throw rt::Error("the list changed since it was shown: look at it again, then press Write");
      if (!link.alive()) throw rt::Error("the drone is not connected");
      const Json id = link.identity();
      if (p["gen"].numOr(-1) != link.generation() || !(p["sysid"] == id["sysid"]) || !(p["compid"] == id["compid"]))
        throw rt::Error("the connection changed since the list was shown: show it again, then Write");
      if (link.vehicle()["armed"].asBool()) throw rt::Error("the drone is armed: land and disarm first");
      if (!journal.ok()) throw rt::Error("the write journal cannot be written (" + journal.error() + "): nothing is written");
      heavyPending = Json();
      journal.append("approval", Json::obj({{"what", "model-tuning step"}, {"step", p["step"]}, {"hash", p["hash"]}, {"by", "user"}, {"rows", p["rows"]}}));
      Json items = Json::array();
      for (const auto& r : p["rows"].arr())
        if (!r["same"].asBool()) items.push(Json::obj({{"name", r["name"]}, {"to", r["to"]}, {"expect", r["vehicle"]}, {"reason", "model-tuning step"}}));
      const std::string undo = pathStr(fsPath(opts_.sessionsDir) / "heavy-writes" /
                                       (isoStampForFile().substr(0, 19) + " UNDO step " + core::jsToString(p["step"].numOr(0)) + ".param"));
      TxnOpts to;
      to.journal = [this](const std::string& k, const Json& d) { journal.append(k, d); };
      to.ctx = Json::obj({{"via", "model tuning"}, {"step", p["step"]}, {"hash", p["hash"]}});
      to.afterPreRead = [undo](const Json& before) {
        Json urows = Json::array();
        for (const auto& b : before.arr()) urows.push(Json::obj({{"group", "before AP Tuner"}, {"name", b["name"]}, {"value", b["before"]}}));
        std::error_code ec;
        fs::create_directories(fsPath(undo).parent_path(), ec);
        if (!writeText(undo, engine::paramFile(urows, {"Values on the drone before AP Tuner wrote, " + localNow(), "Load this file to go back."}, Json(), false)))
          throw std::runtime_error("the UNDO file could not be saved (" + undo + "): nothing was written");
      };
      const Json txn = items.size() ? co_await runTransaction(link, items, to) : Json::obj({{"ok", true}, {"rows", Json::array()}});
      bool reboot = false;
      for (const auto& r : p["rows"].arr())
        if (!r["same"].asBool()) reboot = reboot || core::isRebootRequired(r["name"].strOr("")) || r["name"].strOr("") == "INS_GYRO_FILTER";
      heavyLastWrite = Json::obj({{"txn", txn}, {"undoFile", items.size() ? Json(undo) : Json()}, {"reboot", reboot && txn["ok"].asBool()}, {"rows", p["rows"]},
                                  {"text", items.size() ? txnText(txn) : std::string("every value was already on the drone: nothing written")}});
      co_return jsonResp(200, heavyLastWrite);
    }
    if (path == "/api/heavy/reboot") {
      if (!heavyLastWrite.isObject() || !heavyLastWrite["txn"]["ok"].asBool()) throw rt::Error("write a step first");
      if (link.vehicle()["armed"].asBool()) throw rt::Error("the drone is armed: land and disarm first");
      Json expect = Json::array();
      for (const auto& r : heavyLastWrite["rows"].arr()) expect.push(Json::obj({{"name", r["name"]}, {"value", r["to"]}}));
      RebootOpts ro;
      ro.journal = [this](const std::string& k, const Json& d) { journal.append(k, d); };
      ro.ctx = Json::obj({{"via", "model tuning"}});
      co_return jsonResp(200, co_await rebootAndVerify(link, expect, ro));
    }
    // ---- model tuning (heavy / slow vehicles): live recording, result, files
    if (path == "/api/heavy/live/start") co_return jsonResp(200, co_await heavy.start());
    if (path == "/api/heavy/live/stop") {
      heavy.stop();
      co_return ok();
    }
    if (path == "/api/heavy/live/analyse") {
      heavy.analyseNow();
      co_return ok();
    }
    if (path == "/api/heavy/state")
      co_return jsonResp(200, Json::obj({{"live", heavy.state()}, {"hasResult", lastHeavy.isObject()}, {"connected", link.connected()}, {"alive", link.alive()},
                                         {"vehicle", link.alive() ? link.vehicle() : Json()}, {"identity", link.alive() ? link.identity() : Json()},
                                         {"lastWrite", heavyLastWrite}}));
    if (path == "/api/heavy/result") co_return jsonResp(200, lastHeavy.isObject() ? lastHeavy : Json::object());
    if (path == "/api/heavy/file") {
      if (!lastHeavy.isObject()) co_return jsonResp(400, Json::obj({{"error", "analyse a flight first"}}));
      const std::string kind = req.param("kind");
      const int n = std::atoi(req.param("step").c_str());
      const Json& v = lastHeavy["vehicle"];
      const std::string who = "AP Tuner sysid " + (v["sysid"].isNumber() ? core::jsToString(v["sysid"].num()) : std::string("unknown"));
      HttpResponse r;
      r.contentType = "text/plain";
      std::string fname;
      if (kind == "plan") {
        r.contentType = "application/json";
        r.body = lastHeavy["plan"].dump();
        fname = who + " plan.json";
      } else {
        const Json* st = nullptr;
        if (lastHeavy["steps"].isArray())
        for (const auto& s : lastHeavy["steps"].arr())
          if (static_cast<int>(s["n"].numOr(0)) == n) st = &s;
        if (!st) co_return jsonResp(404, Json::obj({{"error", "no such step"}}));
        r.body = kind == "undo" ? (*st)["undoText"].strOr("") : (*st)["paramText"].strOr("");
        fname = who + " Step " + std::to_string(n) + (kind == "undo" ? " UNDO" : "") + ".param";
        journal.append("export", Json::obj({{"what", "model-tuning .param"}, {"step", n}, {"kind", kind.empty() ? "param" : kind}}));
      }
      r.headers.push_back({"Content-Disposition", "attachment; filename=\"" + fname + "\""});
      co_return r;
    }
    co_return jsonResp(404, Json::obj({{"error", "unknown api"}}));
  }

  /** run an API call on the loop and wait for its response (connection thread) */
  HttpResponse callOnLoop(const std::string& path, const Json& body, const HttpRequest& req) {
    std::promise<HttpResponse> pr;
    auto fut = pr.get_future();
    Loop::main().post([this, path, body, req, &pr] {
      rt::Future<HttpResponse> f;
      try {
        f = api(path, body, req);
      } catch (const std::exception& e) {
        pr.set_value(jsonResp(500, Json::obj({{"error", e.what()}})));
        return;
      }
      f.then([f, &pr] {
        try {
          pr.set_value(f.get());
        } catch (const std::exception& e) {
          pr.set_value(jsonResp(500, Json::obj({{"error", e.what()}})));
        }
      });
    });
    return fut.get();
  }

  /** log analysis (pure: runs on the connection thread, like a worker), then stored on the loop */
  HttpResponse analyzeFile(const std::string& file, const std::string& preset, const std::string& family) {
    try {
      engine::AnalyzeLogOptions ao;
      ao.preset = preset.empty() ? "normal" : preset;
      if (!family.empty()) ao.family = core::parseFamily(family);
      Json r = engine::analyzeLogFile(file, ao);
      const auto fam = core::parseFamily(r["family"].str());
      // the proposed rows, with the value in the log and what the file checks found
      r.set("rows", engine::proposalRows(*fam, r["axes"], r["noise"], r["params"]));
      const engine::ParamFileOut pf = engine::paramFileChecked(r["rows"]);
      r.set("paramChecks", Json::obj({{"errors", pf.errors}, {"warnings", pf.warnings}}));
      std::string body = r.dump();
      std::promise<void> done;
      Loop::main().post([this, r, &done] {
        lastLogResult = r;
        done.set_value();
      });
      done.get_future().get();
      HttpResponse resp;
      resp.body = std::move(body);
      return resp;
    } catch (const std::exception& e) {
      return jsonResp(500, Json::obj({{"error", e.what()}}));
    }
  }

  void handle(const HttpRequest& req, HttpConnection& c) {
    if (auto refused = gateRequest(req, opts_.port, token_)) {
      c.send(*refused);
      return;
    }
    if (req.path == "/api/events") {
      // replay the latest analyses, as the JavaScript server did
      std::promise<std::vector<std::string>> pr;
      auto fut = pr.get_future();
      Loop::main().post([this, &pr] {
        std::vector<std::string> init;
        for (const auto& [ax, r] : tuner.results.items()) init.push_back("event: analysis\ndata: " + r.dump() + "\n\n");
        pr.set_value(std::move(init));
      });
      hub.serve(c, fut.get());
      return;
    }
    if (req.path.rfind("/api/", 0) == 0) {
      if (req.path == "/api/log/list") {
        c.send(jsonResp(200, listLogs(opts_.sessionsDir)));
        return;
      }
      if (req.path == "/api/log/upload") {
        std::string name = req.hasParam("name") ? req.param("name") : "upload.bin";
        static const std::regex bad("[^\\w .()-]");
        name = std::regex_replace(name, bad, "_");
        const std::string file = pathStr(fsPath(opts_.sessionsDir) / "uploads" / name);
        writeText(file, req.body);
        c.send(analyzeFile(file, req.param("preset"), req.param("family")));
        return;
      }
      Json body = Json::object();
      if (req.method == "POST" && req.header("content-type").find("application/json") != std::string::npos && !req.body.empty()) {
        try {
          body = Json::parse(req.body);
        } catch (const std::exception&) {
          body = Json::object();
        }
      }
      // model tuning of one or more logs of the same vehicle (pure work: on the connection thread, like the log analysis)
      if (req.path == "/api/heavy/upload") {
        std::string name = req.hasParam("name") ? req.param("name") : "upload.bin";
        static const std::regex bad("[^\\w .()-]");
        name = std::regex_replace(name, bad, "_");
        const std::string file = pathStr(fsPath(opts_.sessionsDir) / "uploads" / name);
        writeText(file, req.body);
        c.send(jsonResp(200, Json::obj({{"ok", true}, {"path", file}})));
        return;
      }
      if (req.path == "/api/heavy/evaluate") {
        Json b = Json::object();
        try {
          b = Json::parse(req.body);
        } catch (const std::exception&) {
        }
        std::promise<Json> got;
        Loop::main().post([this, &got] { got.set_value(lastHeavy); });
        const Json r = got.get_future().get();
        if (!r.isObject()) {
          c.send(jsonResp(400, Json::obj({{"error", "analyse a flight first"}})));
          return;
        }
        try {
          c.send(jsonResp(200, engine::heavy::evaluateSettings(r, b["rows"])));
        } catch (const std::exception& e) {
          c.send(jsonResp(500, Json::obj({{"error", e.what()}})));
        }
        return;
      }
      if (req.path == "/api/heavy/analyze") {
        Json b = Json::object();
        try {
          b = Json::parse(req.body);
        } catch (const std::exception&) {
        }
        std::vector<engine::heavy::FlightRecord> flights;
        try {
          for (const auto& p : b["paths"].isArray() ? b["paths"].arr() : Json::Array{}) {
            const std::string fp = p.strOr("");
            if (fp.empty() || !fs::exists(fsPath(fp))) {
              c.send(jsonResp(400, Json::obj({{"error", "file not found: " + fp}})));
              return;
            }
            flights.push_back(engine::heavy::recordFromLogFile(fp));
          }
          if (flights.empty()) {
            c.send(jsonResp(400, Json::obj({{"error", "choose one or more .bin logs of the same vehicle"}})));
            return;
          }
          Json r = analyseAndSave(flights, opts_.sessionsDir, b["spacingM"].numOr(0));
          std::promise<void> done;
          Loop::main().post([this, r, &done] {
            lastHeavy = r;
            done.set_value();
          });
          done.get_future().get();
          c.send(jsonResp(200, r));
        } catch (const std::exception& e) {
          c.send(jsonResp(500, Json::obj({{"error", e.what()}})));
        }
        return;
      }
      if (req.path == "/api/log/analyze") {
        const std::string p = body["path"].strOr("");
        if (p.empty() || !fs::exists(fsPath(p))) {
          c.send(jsonResp(400, Json::obj({{"error", "file not found: " + p}})));
          return;
        }
        c.send(analyzeFile(p, body["preset"].strOr("normal"), body["family"].strOr("")));
        return;
      }
      c.send(callOnLoop(req.path, body, req));
      return;
    }
    // static files from web/
    std::string p = req.path == "/" ? "/index.html" : req.path;
    if (p.find("..") != std::string::npos) {
      c.send(HttpResponse{404, "text/plain", "not found", {}, ""});
      return;
    }
    const fs::path file = fsPath(opts_.webDir) / fsPath(p.substr(1));
    std::error_code ec;
    if (!fs::is_regular_file(file, ec)) {
      c.send(HttpResponse{404, "text/plain", "not found", {}, ""});
      return;
    }
    HttpResponse r;
    const std::string ext = pathStr(file.extension());
    r.contentType = kMime.count(ext) ? kMime.at(ext) : "application/octet-stream";
    if (ext == ".html") {
      // the page carries this run's token (only a page served from here can read it) and a strict CSP
      std::string html = readText(pathStr(file));
      const std::string mark = "__APTUNER_TOKEN__";
      const auto at = html.find(mark);
      if (at != std::string::npos) html.replace(at, mark.size(), token_);
      r.body = std::move(html);
      r.headers.push_back({"Cache-Control", "no-store"});
      r.headers.push_back({"Content-Security-Policy", kPageCsp});
      r.headers.push_back({"X-Frame-Options", "DENY"});
      r.headers.push_back({"Referrer-Policy", "no-referrer"});
    } else {
      r.filePath = pathStr(file);
      r.headers.push_back({"Cache-Control", "no-cache"});
    }
    r.headers.push_back({"X-Content-Type-Options", "nosniff"});
    c.send(r);
  }

  AppOptions opts_;
  std::string token_;   // per run; the app page gets it, other pages cannot read it
  io::Link link;
  Journal journal;      // before the tuner and the ground session: they write to it
  Tuner tuner;
  NotchLive notch;
  FlightSession flight;
  GroundTune ground;
  HeavyLive heavy;
  Json lastHeavy;   // the latest model-tuning result (loop thread)
  Json heavyPending, heavyLastWrite;   // the list shown before a write, and the last write
  io::AutoConnectState autoState;
  Json lastLogResult;
  SseHub hub;
};

}  // namespace

int runApp(const AppOptions& o) {
  HttpServer server;
  std::string err;
  const std::string url = "http://127.0.0.1:" + std::to_string(o.port);
  if (!server.listen(o.port, err)) {
    std::printf("AP Tuner is already running on port %d\n", o.port);
    if (o.appMode || o.openBrowser) openAppWindow(url, o.appMode);
    return 0;
  }
  auto app = std::make_unique<App>(o);
  server.start([&app](const HttpRequest& req, HttpConnection& c) { app->handle(req, c); });
  std::printf("%s running at %s%s\n", core::buildInfoLine().c_str(), url.c_str(), o.appMode ? " (app mode)" : "");
  std::fflush(stdout);
  auto& loop = Loop::main();
  std::intptr_t window = 0;
  loop.post([&] {
    app->startTimers();
    if (o.appMode || o.openBrowser) window = openAppWindow(url, o.appMode);
  });
  if (o.appMode) {
    // exit when the app window has really been closed: no event-stream client for 20 s and our Edge window gone
    loop.post([&] {
      loop.setInterval(5000, [&] {
        const bool idle = app->hub.everHadClient() && app->hub.count() == 0 && nowMs() - app->hub.lastClientAt() > 20000;
        if (idle && !processAlive(window)) {
          std::printf("app window closed, exiting\n");
          loop.stop();
        }
      });
    });
  }
  loop.run();
  server.stop();
  app->link.disconnect();
  return 0;
}

}  // namespace aptuner::app
