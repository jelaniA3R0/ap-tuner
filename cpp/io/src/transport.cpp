#include "aptuner/io/transport.hpp"

#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>

#include "aptuner/rt/loop.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
using SockT = SOCKET;
static constexpr SockT kBadSock = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
using SockT = int;
static constexpr SockT kBadSock = -1;
#endif

namespace aptuner::io {

namespace {

std::string lastErrorText() {
#ifdef _WIN32
  const DWORD e = GetLastError();
  char buf[512] = {0};
  const DWORD n = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, e,
                                 MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), buf, sizeof buf, nullptr);
  std::string s(buf, n);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
  return s.empty() ? "error " + std::to_string(e) : s;
#else
  return std::strerror(errno);
#endif
}

std::string sockErrorText() {
#ifdef _WIN32
  const int e = WSAGetLastError();
  SetLastError(static_cast<DWORD>(e));
  return lastErrorText();
#else
  return std::strerror(errno);
#endif
}

void closeSock(SockT s) {
  if (s == kBadSock) return;
#ifdef _WIN32
  shutdown(s, SD_BOTH);
  closesocket(s);
#else
  shutdown(s, SHUT_RDWR);
  ::close(s);
#endif
}

void netInit() {
#ifdef _WIN32
  static std::once_flag once;
  std::call_once(once, [] {
    WSADATA d;
    WSAStartup(MAKEWORD(2, 2), &d);
  });
#endif
}

/** state shared between the owner (loop thread) and the reader thread */
struct Shared : std::enable_shared_from_this<Shared> {
  std::atomic<bool> closed{false};
  TransportCallbacks cb;
  std::mutex sendMu;
  void post(std::function<void(TransportCallbacks&)> fn) {
    if (closed) return;
    auto self = shared_from_this();
    rt::Loop::main().post([self, fn] {
      if (!self->closed) fn(self->cb);
    });
  }
  void data(const std::uint8_t* p, std::size_t n) {
    std::vector<std::uint8_t> v(p, p + n);
    post([v](TransportCallbacks& c) {
      if (c.onData) c.onData(v);
    });
  }
  void status(const std::string& s) {
    post([s](TransportCallbacks& c) {
      if (c.onStatus) c.onStatus(s);
    });
  }
  void opened() {
    post([](TransportCallbacks& c) {
      if (c.onOpen) c.onOpen();
    });
  }
  void ended(int code) {
    post([code](TransportCallbacks& c) {
      if (c.onClose) c.onClose(code);
    });
  }
};

// ------------------------------------------------------------------------------------------------ TCP

class TcpTransport : public Transport {
 public:
  TcpTransport(const std::string& host, int port, TransportCallbacks cb) : st_(std::make_shared<St>()) {
    st_->cb = std::move(cb);
    netInit();
    auto st = st_;
    std::thread([st, host, port] {
      addrinfo hints{};
      hints.ai_family = AF_UNSPEC;
      hints.ai_socktype = SOCK_STREAM;
      addrinfo* res = nullptr;
      if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res) {
        st->status("tcp error: cannot resolve " + host);
        st->ended(1);
        return;
      }
      SockT s = kBadSock;
      std::string err = "connection refused";
      for (addrinfo* a = res; a && !st->closed; a = a->ai_next) {
        s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (s == kBadSock) continue;
        st->sock = s;
        if (::connect(s, a->ai_addr, static_cast<int>(a->ai_addrlen)) == 0) break;
        err = sockErrorText();
        closeSock(s);
        st->sock = kBadSock;
        s = kBadSock;
      }
      freeaddrinfo(res);
      if (s == kBadSock || st->closed) {
        st->status("tcp error: " + err);
        st->ended(1);
        return;
      }
      int one = 1;
      setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
      st->status("tcp connected");
      st->opened();
      std::uint8_t buf[8192];
      while (!st->closed) {
        const auto n = ::recv(s, reinterpret_cast<char*>(buf), sizeof buf, 0);
        if (n <= 0) break;
        st->data(buf, static_cast<std::size_t>(n));
      }
      st->status("tcp closed");
      st->ended(0);
    }).detach();
  }
  ~TcpTransport() override { close(); }
  void send(const std::vector<std::uint8_t>& b) override {
    const SockT s = st_->sock;
    if (st_->closed || s == kBadSock) return;
    std::lock_guard<std::mutex> lk(st_->sendMu);
    std::size_t off = 0;
    while (off < b.size()) {
      const auto n = ::send(s, reinterpret_cast<const char*>(b.data() + off), static_cast<int>(b.size() - off), 0);
      if (n <= 0) return;
      off += static_cast<std::size_t>(n);
    }
  }
  void close() override {
    if (st_->closed.exchange(true)) return;
    closeSock(st_->sock.exchange(kBadSock));
  }

 private:
  struct St : Shared {
    std::atomic<SockT> sock{kBadSock};
  };
  std::shared_ptr<St> st_;
};

// ------------------------------------------------------------------------------------------------ UDP

class UdpTransport : public Transport {
 public:
  UdpTransport(const std::string& host, int port, bool listen, TransportCallbacks cb) : st_(std::make_shared<St>()) {
    st_->cb = std::move(cb);
    netInit();
    SockT s = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<std::uint16_t>(listen ? port : 0));
    inet_pton(AF_INET, listen ? (host.empty() ? "0.0.0.0" : host.c_str()) : "0.0.0.0", &local.sin_addr);
    if (s == kBadSock || ::bind(s, reinterpret_cast<sockaddr*>(&local), sizeof local) != 0) {
      const std::string e = sockErrorText();
      closeSock(s);
      st_->status("udp error: " + e);
      st_->ended(1);
      return;
    }
    st_->sock = s;
    st_->listen = listen;
    if (!listen) {
      st_->remote.sin_family = AF_INET;
      st_->remote.sin_port = htons(static_cast<std::uint16_t>(port));
      inet_pton(AF_INET, host.empty() ? "127.0.0.1" : host.c_str(), &st_->remote.sin_addr);
      st_->haveRemote = true;
      st_->status("udp client -> " + (host.empty() ? std::string("127.0.0.1") : host) + ":" + std::to_string(port));
    } else {
      st_->status("udp listening on " + std::to_string(port));
    }
    st_->opened();
    auto st = st_;
    std::thread([st] {
      std::uint8_t buf[65536];
      while (!st->closed) {
        sockaddr_in from{};
#ifdef _WIN32
        int fl = sizeof from;
#else
        socklen_t fl = sizeof from;
#endif
        const auto n = ::recvfrom(st->sock, reinterpret_cast<char*>(buf), sizeof buf, 0, reinterpret_cast<sockaddr*>(&from), &fl);
        if (n < 0) {
          if (st->closed) break;
#ifdef _WIN32
          if (WSAGetLastError() == WSAECONNRESET) continue;   // ICMP port unreachable from an earlier send
#endif
          break;
        }
        {
          std::lock_guard<std::mutex> lk(st->sendMu);
          if (!st->haveRemote || st->listen) {
            st->remote = from;
            st->haveRemote = true;
          }
        }
        st->data(buf, static_cast<std::size_t>(n));
      }
    }).detach();
  }
  ~UdpTransport() override { close(); }
  void send(const std::vector<std::uint8_t>& b) override {
    if (st_->closed || st_->sock == kBadSock) return;
    std::lock_guard<std::mutex> lk(st_->sendMu);
    if (!st_->haveRemote) return;
    ::sendto(st_->sock, reinterpret_cast<const char*>(b.data()), static_cast<int>(b.size()), 0,
             reinterpret_cast<const sockaddr*>(&st_->remote), sizeof st_->remote);
  }
  void close() override {
    if (st_->closed.exchange(true)) return;
    closeSock(st_->sock.exchange(kBadSock));
  }

 private:
  struct St : Shared {
    std::atomic<SockT> sock{kBadSock};
    sockaddr_in remote{};
    bool haveRemote = false;
    bool listen = false;
  };
  std::shared_ptr<St> st_;
};

// ------------------------------------------------------------------------------------------------ serial

#ifdef _WIN32
class SerialTransport : public Transport {
 public:
  SerialTransport(const std::string& port, int baud, TransportCallbacks cb) : st_(std::make_shared<St>()) {
    st_->cb = std::move(cb);
    const std::string desc = port + " @ " + std::to_string(baud);
    auto st = st_;
    std::thread([st, port, baud, desc] {
      HANDLE h = CreateFileA(("\\\\.\\" + port).c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
      if (h == INVALID_HANDLE_VALUE) {
        st->status("serial: cannot open " + port + ": " + lastErrorText());
        st->status("serial closed (code 2)");
        st->ended(2);
        return;
      }
      SetupComm(h, 65536, 65536);
      DCB dcb{};
      dcb.DCBlength = sizeof dcb;
      GetCommState(h, &dcb);
      dcb.BaudRate = static_cast<DWORD>(baud);
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
      dcb.fBinary = TRUE;
      dcb.fParity = FALSE;
      dcb.fOutxCtsFlow = FALSE;
      dcb.fOutxDsrFlow = FALSE;
      dcb.fDsrSensitivity = FALSE;
      dcb.fDtrControl = DTR_CONTROL_ENABLE;
      dcb.fRtsControl = RTS_CONTROL_ENABLE;
      dcb.fOutX = FALSE;
      dcb.fInX = FALSE;
      dcb.fAbortOnError = FALSE;
      if (!SetCommState(h, &dcb)) {
        st->status("serial: cannot open " + port + ": " + lastErrorText());
        CloseHandle(h);
        st->status("serial closed (code 2)");
        st->ended(2);
        return;
      }
      COMMTIMEOUTS to{};
      to.ReadIntervalTimeout = MAXDWORD;
      to.ReadTotalTimeoutMultiplier = MAXDWORD;
      to.ReadTotalTimeoutConstant = 50;   // return as soon as bytes arrive, or after 50 ms
      to.WriteTotalTimeoutMultiplier = 0;
      to.WriteTotalTimeoutConstant = 500;
      SetCommTimeouts(h, &to);
      PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
      st->handle = h;
      st->status("serial port opened " + desc);
      st->opened();
      std::uint8_t buf[4096];
      int code = 0;
      while (!st->closed) {
        DWORD n = 0;
        const BOOL ok = ReadFile(h, buf, sizeof buf, &n, nullptr);
        if (!ok) {
          if (!st->closed) st->status("serial: " + lastErrorText());
          code = 1;
          break;
        }
        if (n) st->data(buf, n);
        std::this_thread::yield();
      }
      {
        std::lock_guard<std::mutex> lk(st->ioMu);
        st->handle = INVALID_HANDLE_VALUE;
        CloseHandle(h);
      }
      st->status(code ? "serial closed (code 1)" : "serial closed");
      st->ended(code);
    }).detach();
  }
  ~SerialTransport() override { close(); }
  void send(const std::vector<std::uint8_t>& b) override {
    if (st_->closed) return;
    std::lock_guard<std::mutex> lk(st_->ioMu);
    if (st_->handle == INVALID_HANDLE_VALUE) return;
    DWORD n = 0;
    // a write that times out (radio buffer full) drops this packet instead of closing the port
    if (!WriteFile(st_->handle, b.data(), static_cast<DWORD>(b.size()), &n, nullptr) || n != b.size())
      st_->status("serial: write dropped: " + lastErrorText());
  }
  void close() override { st_->closed = true; }   // the reader thread closes the handle within 50 ms

 private:
  struct St : Shared {
    HANDLE handle = INVALID_HANDLE_VALUE;
    std::mutex ioMu;
  };
  std::shared_ptr<St> st_;
};
#else
speed_t baudFlag(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

class SerialTransport : public Transport {
 public:
  SerialTransport(const std::string& port, int baud, TransportCallbacks cb) : st_(std::make_shared<St>()) {
    st_->cb = std::move(cb);
    const std::string desc = port + " @ " + std::to_string(baud);
    auto st = st_;
    std::thread([st, port, baud, desc] {
      const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      const speed_t sp = baudFlag(baud);
      termios tio{};
      if (fd < 0 || !sp || tcgetattr(fd, &tio) != 0) {
        st->status("serial: cannot open " + port + ": " + (fd < 0 ? std::string(std::strerror(errno)) : "unsupported baud rate"));
        if (fd >= 0) ::close(fd);
        st->status("serial closed (code 2)");
        st->ended(2);
        return;
      }
      cfmakeraw(&tio);
      cfsetispeed(&tio, sp);
      cfsetospeed(&tio, sp);
      tio.c_cflag |= CLOCAL | CREAD;
      tcsetattr(fd, TCSANOW, &tio);
      tcflush(fd, TCIOFLUSH);
      st->fd = fd;
      st->status("serial port opened " + desc);
      st->opened();
      std::uint8_t buf[4096];
      int code = 0;
      while (!st->closed) {
        pollfd p{fd, POLLIN, 0};
        const int r = ::poll(&p, 1, 50);
        if (r < 0 && errno != EINTR) {
          code = 1;
          break;
        }
        if (r > 0) {
          const auto n = ::read(fd, buf, sizeof buf);
          if (n > 0) st->data(buf, static_cast<std::size_t>(n));
          else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
            if (!st->closed) st->status("serial: " + std::string(std::strerror(errno)));
            code = 1;
            break;
          }
        }
      }
      {
        std::lock_guard<std::mutex> lk(st->sendMu);
        st->fd = -1;
        ::close(fd);
      }
      st->status(code ? "serial closed (code 1)" : "serial closed");
      st->ended(code);
    }).detach();
  }
  ~SerialTransport() override { close(); }
  void send(const std::vector<std::uint8_t>& b) override {
    if (st_->closed) return;
    std::lock_guard<std::mutex> lk(st_->sendMu);
    if (st_->fd < 0) return;
    std::size_t off = 0;
    int waited = 0;
    while (off < b.size()) {
      const auto n = ::write(st_->fd, b.data() + off, b.size() - off);
      if (n > 0) {
        off += static_cast<std::size_t>(n);
        continue;
      }
      if (errno != EAGAIN || waited >= 500) {
        st_->status("serial: write dropped: " + std::string(std::strerror(errno)));
        return;
      }
      pollfd p{st_->fd, POLLOUT, 0};
      ::poll(&p, 1, 50);
      waited += 50;
    }
  }
  void close() override { st_->closed = true; }

 private:
  struct St : Shared {
    int fd = -1;
  };
  std::shared_ptr<St> st_;
};
#endif

}  // namespace

std::unique_ptr<Transport> openTcp(const std::string& host, int port, TransportCallbacks cb) {
  return std::make_unique<TcpTransport>(host, port, std::move(cb));
}
std::unique_ptr<Transport> openUdp(const std::string& host, int port, bool listen, TransportCallbacks cb) {
  return std::make_unique<UdpTransport>(host, port, listen, std::move(cb));
}
std::unique_ptr<Transport> openSerial(const std::string& port, int baud, TransportCallbacks cb) {
  return std::make_unique<SerialTransport>(port, baud, std::move(cb));
}

}  // namespace aptuner::io
