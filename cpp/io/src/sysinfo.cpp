#include "aptuner/io/sysinfo.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <regex>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <tlhelp32.h>
#else
#include <dirent.h>
#include <unistd.h>

#include <fstream>
#endif

namespace aptuner::io {

namespace {
std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

#ifdef _WIN32
std::string narrow(const wchar_t* w) {
  if (!w) return {};
  const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<std::size_t>(n > 0 ? n - 1 : 0), '\0');
  if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
  return s;
}
#endif
}  // namespace

std::vector<SerialPortInfo> listSerialPorts() {
  std::vector<SerialPortInfo> out;
#ifdef _WIN32
  // 1) port names in registry order (what SerialPort.GetPortNames returns)
  std::vector<std::string> ports;
  HKEY key;
  if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) == ERROR_SUCCESS) {
    for (DWORD i = 0;; i++) {
      wchar_t name[256], data[256];
      DWORD nl = 256, dl = sizeof data, type = 0;
      if (RegEnumValueW(key, i, name, &nl, nullptr, &type, reinterpret_cast<BYTE*>(data), &dl) != ERROR_SUCCESS) break;
      if (type == REG_SZ) ports.push_back(narrow(data));
    }
    RegCloseKey(key);
  }
  // 2) friendly names from the Ports device class
  static const GUID kPorts = {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
  std::vector<std::pair<std::string, std::string>> friendly;   // port -> name
  HDEVINFO set = SetupDiGetClassDevsW(&kPorts, nullptr, nullptr, DIGCF_PRESENT);
  if (set != INVALID_HANDLE_VALUE) {
    SP_DEVINFO_DATA dev{};
    dev.cbSize = sizeof dev;
    static const std::regex comRe("\\((COM\\d+)\\)");
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dev); i++) {
      wchar_t buf[512];
      if (!SetupDiGetDeviceRegistryPropertyW(set, &dev, SPDRP_FRIENDLYNAME, nullptr, reinterpret_cast<BYTE*>(buf), sizeof buf, nullptr)) continue;
      const std::string n = narrow(buf);
      std::smatch m;
      if (std::regex_search(n, m, comRe)) friendly.emplace_back(m[1].str(), n);
    }
    SetupDiDestroyDeviceInfoList(set);
  }
  for (const auto& p : ports) {
    std::string name = p;
    for (const auto& [fp, fn] : friendly)
      if (fp == p) {
        name = fn;
        break;
      }
    out.push_back({p, name});
  }
#else
  namespace fs = std::filesystem;
  std::error_code ec;
  // by-id names are the friendly ones on Linux
  std::vector<std::pair<std::string, std::string>> byId;
  for (const auto& e : fs::directory_iterator("/dev/serial/by-id", ec)) {
    std::error_code e2;
    const auto target = fs::canonical(e.path(), e2);
    if (!e2) byId.emplace_back(target.string(), e.path().filename().string());
  }
  for (const auto& e : fs::directory_iterator("/dev", ec)) {
    const std::string n = e.path().filename().string();
    if (n.rfind("ttyUSB", 0) == 0 || n.rfind("ttyACM", 0) == 0 || n.rfind("cu.usb", 0) == 0 || n.rfind("cu.SLAB", 0) == 0 ||
        n.rfind("tty.usb", 0) == 0) {
      const std::string path = e.path().string();
      std::string name = path;
      for (const auto& [t, id] : byId)
        if (t == path) name = id + " (" + path + ")";
      out.push_back({path, name});
    }
  }
  std::sort(out.begin(), out.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) { return a.port < b.port; });
#endif
  return out;
}

std::vector<SerialMapEntry> serialMap() {
  std::vector<SerialMapEntry> out;
#ifdef _WIN32
  HKEY key;
  if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) == ERROR_SUCCESS) {
    for (DWORD i = 0;; i++) {
      wchar_t name[256], data[256];
      DWORD nl = 256, dl = sizeof data, type = 0;
      if (RegEnumValueW(key, i, name, &nl, nullptr, &type, reinterpret_cast<BYTE*>(data), &dl) != ERROR_SUCCESS) break;
      if (type == REG_SZ) out.push_back({narrow(data), narrow(name)});
    }
    RegCloseKey(key);
  }
#else
  for (const auto& p : listSerialPorts()) out.push_back({p.port, p.port});
#endif
  return out;
}

bool processRunning(const std::string& exeName) {
  const std::string want = lower(exeName);
#ifdef _WIN32
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return false;
  PROCESSENTRY32W pe{};
  pe.dwSize = sizeof pe;
  bool found = false;
  for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe))
    if (lower(narrow(pe.szExeFile)) == want) {
      found = true;
      break;
    }
  CloseHandle(snap);
  return found;
#else
  DIR* d = opendir("/proc");
  if (!d) return false;
  bool found = false;
  while (dirent* e = readdir(d)) {
    if (!std::isdigit(static_cast<unsigned char>(e->d_name[0]))) continue;
    std::ifstream f(std::string("/proc/") + e->d_name + "/comm");
    std::string comm;
    if (std::getline(f, comm) && lower(comm) == want) {
      found = true;
      break;
    }
  }
  closedir(d);
  return found;
#endif
}

int countProcessesWithCommandLine(const std::string& exeName, const std::string& needle) {
  (void)exeName;
  (void)needle;
  return 0;   // only used for the app window watchdog; the C++ app tracks its own window instead
}

std::string exeDir() {
#ifdef _WIN32
  wchar_t buf[MAX_PATH];
  const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
  return std::filesystem::path(std::wstring(buf, n)).parent_path().string();
#else
  std::error_code ec;
  const auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
  return ec ? std::filesystem::current_path().string() : p.parent_path().string();
#endif
}

}  // namespace aptuner::io
