#include "aptuner/io/autoconnect.hpp"

#include <regex>
#include <set>

#include "aptuner/io/sysinfo.hpp"

namespace aptuner::io {

using core::Json;
using rt::Loop;

namespace {
// telemetry radios: SiK / RFD900 default 57600; USB and many radios 115200; RFD900x / ELRS / long-range links faster
const int kSerialBauds[] = {57600, 115200, 921600, 460800, 230400};

bool contains(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }
}  // namespace

rt::Task<Json> autoconnect(Link& link, AutoConnectState& state, std::function<void(const Json&)> onProgress) {
  std::vector<Json> cands{
      Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", 14550}, {"label", "Mission Planner mirror (TCP 14550)"}, {"wait", 3000}}),
      Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", 5762}, {"label", "Simulator (SITL, port 5762)"}, {"wait", 2500}}),
      Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", 5763}, {"label", "Simulator (SITL, port 5763)"}, {"wait", 2500}}),
      Json::obj({{"kind", "tcp"}, {"host", "127.0.0.1"}, {"port", 5760}, {"label", "Simulator (SITL, port 5760)"}, {"wait", 2500}}),
      Json::obj({{"kind", "udpin"}, {"port", 14550}, {"label", "UDP 14550 (Mission Planner UDP mirror or a Wi-Fi link)"}, {"wait", 3500}}),
  };
  const bool mpOpen = processRunning("MissionPlanner.exe");
  // Bluetooth COM ports are skipped (opening one with no paired device hangs); use the manual connection for them
  std::regex bt("bluetooth", std::regex::icase);
  for (const auto& p : listSerialPorts()) {
    if (std::regex_search(p.name, bt)) continue;
    for (const int baud : kSerialBauds)
      cands.push_back(Json::obj({{"kind", "serial"}, {"port", p.port}, {"baud", baud},
                                 {"label", p.name + " at " + std::to_string(baud)}, {"wait", 3500}}));
  }
  state = AutoConnectState();
  state.running = true;
  state.missionPlanner = mpOpen;
  std::set<std::string> busyPorts, silentPorts;
  for (const Json& c : cands) {
    const std::string kind = c["kind"].str();
    const std::string port = c["port"].isString() ? c["port"].str() : std::string();
    if (kind == "serial" && (busyPorts.count(port) || silentPorts.count(port))) continue;   // held by another program, or silent
    state.current = c["label"].str();
    if (onProgress) onProgress(Json::obj({{"trying", c["label"]}}));
    auto lastStatus = std::make_shared<std::string>();
    const std::uint64_t bytes0 = link.stats().bytes;
    const std::uint64_t sub = link.on("status", [lastStatus](const Json& s) { *lastStatus = s.strOr(""); });
    link.connect(c);
    double t0 = Loop::now();
    bool ok = false, opened = kind != "serial";
    const double wait = c["wait"].numOr(3000);
    // serial: the listening time starts when the port is really open
    if (kind == "serial") {
      while (Loop::now() - t0 < 10000 && !link.serialOpened() && !contains(*lastStatus, "cannot open") &&
             !contains(*lastStatus, "serial closed") && !link.alive())
        co_await rt::sleep(100);
      opened = link.serialOpened() || link.alive();
      t0 = Loop::now();
    }
    while (opened && Loop::now() - t0 < wait) {
      co_await rt::sleep(200);
      if (link.alive()) {
        ok = true;
        break;
      }
      if (kind == "serial" && contains(*lastStatus, "cannot open")) break;
      if (!link.connected() && Loop::now() - t0 > 1200 && kind != "serial") break;
    }
    link.off(sub);
    if (ok) {
      state.running = false;
      state.found = c;
      if (onProgress) onProgress(Json::obj({{"found", c["label"]}}));
      co_return c;
    }
    std::string result = "no heartbeat";
    const bool cannotOpen = contains(*lastStatus, "cannot open");
    if (kind == "serial" && opened && link.stats().bytes == bytes0 && !cannotOpen) {
      silentPorts.insert(port);
      result = "no data (port open, nothing received: is the aircraft powered and the radio link LED solid?)";
    } else if (kind == "serial" && !opened && !cannotOpen) {
      busyPorts.insert(port);
      result = "port did not open within 10 s";
    } else if (kind == "serial" && link.stats().bytes > bytes0 && !ok) {
      result = "data but no MAVLink heartbeat at this speed";
    }
    if (kind == "serial" && cannotOpen) {
      busyPorts.insert(port);
      static const std::regex inUse("denied|in use|access", std::regex::icase);
      std::string s = *lastStatus;
      if (s.rfind("serial: ", 0) == 0) s = s.substr(8);
      result = std::regex_search(*lastStatus, inUse) ? "port in use by another program" : s;
    }
    std::string label = c["label"].str();
    if (kind == "serial" && busyPorts.count(port)) {
      static const std::regex atBaud(" at \\d+$");
      label = port + " (" + std::regex_replace(label, atBaud, "") + ")";
    }
    state.tried.push(Json::obj({{"label", label}, {"result", result}, {"busy", kind == "serial" && busyPorts.count(port) > 0}}));
    link.disconnect();
    co_await rt::sleep(kind == "serial" ? 600 : 150);   // let Windows release the COM port before the next try
  }
  state.running = false;
  state.found = Json();
  for (const auto& p : busyPorts) state.busyPorts.push(p);
  co_return Json();
}

}  // namespace aptuner::io
