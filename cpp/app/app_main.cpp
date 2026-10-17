// aptuner: the application. Starts the local server for the web UI and opens it as its own window.
//   aptuner                 app mode: server + app window, exits when the window is closed
//   aptuner --server        server only (no window, no auto-exit)
//   aptuner --open          server + the default browser
//   aptuner --port 8091     another port (also the PORT environment variable)
//   aptuner --version
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "aptuner/core/build_info.hpp"
#include "aptuner/io/sysinfo.hpp"
#include "common.hpp"
#include "server.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char** argv) {
  using namespace aptuner;
  app::AppOptions o;
  if (const char* p = std::getenv("PORT")) o.port = std::atoi(p);
  bool server = false;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--version" || a == "-v") {
      std::puts(core::buildInfoLine().c_str());
      return 0;
    }
    if (a == "--server") server = true;
    else if (a == "--open") o.openBrowser = true;
    else if (a == "--app") o.appMode = true;
    else if (a == "--port" && i + 1 < argc) o.port = std::atoi(argv[++i]);
    else if (a == "--web" && i + 1 < argc) o.webDir = argv[++i];
    else if (a == "--sessions" && i + 1 < argc) o.sessionsDir = argv[++i];
  }
  if (!server && !o.openBrowser) o.appMode = true;
  if (o.webDir.empty()) o.webDir = app::findWebDir();
  if (o.webDir.empty()) {
#ifdef _WIN32
    MessageBoxA(nullptr, "The web folder (web\\index.html) was not found next to aptuner.exe.", "AP Tuner", MB_ICONERROR);
#endif
    std::fprintf(stderr, "web/index.html not found next to the program\n");
    return 1;
  }
  if (o.sessionsDir.empty()) o.sessionsDir = app::pathStr(app::fsPath(o.webDir).parent_path() / "sessions");
  std::error_code ec;
  std::filesystem::create_directories(app::fsPath(o.sessionsDir), ec);
  if (o.appMode) {
    // no console in app mode: keep a log next to the session files
    const std::string logFile = app::pathStr(app::fsPath(o.sessionsDir) / "aptuner.log");
    std::FILE* f = std::freopen(logFile.c_str(), "a", stdout);
    (void)f;
  }
  return app::runApp(o);
}

#if defined(_WIN32) && defined(_MSC_VER)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) { return main(__argc, __argv); }
#endif
