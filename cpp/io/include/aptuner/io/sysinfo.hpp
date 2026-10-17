// System queries the app needs: serial ports with friendly names, whether a program (Mission Planner) is running,
// opening a URL / app window. Native APIs (SetupAPI / Toolhelp on Windows, /dev and /proc elsewhere).
#pragma once

#include <string>
#include <vector>

namespace aptuner::io {

struct SerialPortInfo {
  std::string port;   // "COM53" or "/dev/ttyUSB0"
  std::string name;   // "Silicon Labs CP210x USB to UART Bridge (COM53)"
};
std::vector<SerialPortInfo> listSerialPorts();

/** active serial ports from the registry (Windows): {port, dev} where dev is e.g. \Device\USBSER000 */
struct SerialMapEntry {
  std::string port, dev;
};
std::vector<SerialMapEntry> serialMap();

/** true when a process with this executable name is running (case-insensitive, e.g. "MissionPlanner.exe") */
bool processRunning(const std::string& exeName);
/** number of running processes whose command line contains `needle` (Windows; 0 elsewhere) */
int countProcessesWithCommandLine(const std::string& exeName, const std::string& needle);

/** directory of the running executable */
std::string exeDir();

}  // namespace aptuner::io
