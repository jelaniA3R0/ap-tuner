// The AP Tuner application: the HTTP + Server-Sent Events API that the web UI (web/) talks to, the vehicle link,
// the live tuner, guided flight, Telemetry + USB session and live notch. Port of src/server.js.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "http.hpp"

namespace aptuner::app {

struct AppOptions {
  int port = 8090;
  bool appMode = false;     // open the app window and exit when it closes
  bool openBrowser = false; // open the default browser once
  std::string webDir, sessionsDir;
};

/** runs the server until the process exits; returns the exit code */
int runApp(const AppOptions& o);

/** The request gate of the local API (2026-10-17 audit F-05): the Host (and Origin, when sent) must be 127.0.0.1 /
    localhost on this port, no cross-site browser request, and every state-changing /api/ request must be a POST that
    carries the per-run token. Nothing when the request may go on, else the refusal to send. */
std::optional<HttpResponse> gateRequest(const HttpRequest& req, int port, const std::string& token);
/** the Content-Security-Policy of the app page */
extern const char* const kPageCsp;

/** directory that holds web/index.html (searched upwards from the executable), "" if none */
std::string findWebDir();
/** open the app window (Edge app mode on Windows, else the default browser); returns a process handle or 0 */
std::intptr_t openAppWindow(const std::string& url, bool appWindow);
bool processAlive(std::intptr_t handle);

}  // namespace aptuner::app
