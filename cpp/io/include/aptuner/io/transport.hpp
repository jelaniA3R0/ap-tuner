// Byte transports for the MAVLink link: TCP client, UDP client / listener and serial port (native: Win32 COM
// API on Windows, termios elsewhere; replaces the JavaScript version's PowerShell serial bridge).
// A reader thread per transport hands everything (data, status lines, open / close) to the event loop, so
// the callbacks always run on the loop thread. send() and close() are called from the loop thread.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace aptuner::io {

struct TransportCallbacks {
  std::function<void(const std::vector<std::uint8_t>&)> onData;
  std::function<void(const std::string&)> onStatus;   // human-readable status line (JS: link 'status' events)
  std::function<void()> onOpen;                       // connected / port opened
  std::function<void(int code)> onClose;              // closed (code 0 = normal)
};

class Transport {
 public:
  virtual ~Transport() = default;
  virtual void send(const std::vector<std::uint8_t>& bytes) = 0;
  virtual void close() = 0;
};

std::unique_ptr<Transport> openTcp(const std::string& host, int port, TransportCallbacks cb);
/** listen = bind host:port and answer whoever sends (Mission Planner UDP mirror, Wi-Fi links);
 *  otherwise bind an ephemeral port and send to host:port */
std::unique_ptr<Transport> openUdp(const std::string& host, int port, bool listen, TransportCallbacks cb);
std::unique_ptr<Transport> openSerial(const std::string& port, int baud, TransportCallbacks cb);

}  // namespace aptuner::io
