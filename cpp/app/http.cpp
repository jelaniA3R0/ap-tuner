#include "http.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <fstream>
#include <thread>

#include "common.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
using SockT = SOCKET;
static const SockT kBad = INVALID_SOCKET;
static void closeS(SockT s) {
  shutdown(s, SD_BOTH);
  closesocket(s);
}
#else
#include <arpa/inet.h>
#include <csignal>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
using SockT = int;
static const SockT kBad = -1;
static void closeS(SockT s) {
  shutdown(s, SHUT_RDWR);
  ::close(s);
}
#endif

namespace aptuner::app {

double nowMs() {
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string statusText(int code) {
  switch (code) {
    case 200: return "OK";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    default: return "Status";
  }
}

std::string urlDecode(const std::string& s) {
  std::string o;
  for (std::size_t i = 0; i < s.size(); i++) {
    if (s[i] == '+') {
      o.push_back(' ');
    } else if (s[i] == '%' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) &&
               std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
      o.push_back(static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)));
      i += 2;
    } else {
      o.push_back(s[i]);
    }
  }
  return o;
}

namespace {
std::map<std::string, std::string> parseQuery(const std::string& q) {
  std::map<std::string, std::string> m;
  std::size_t a = 0;
  while (a < q.size()) {
    std::size_t b = q.find('&', a);
    if (b == std::string::npos) b = q.size();
    const std::string kv = q.substr(a, b - a);
    const std::size_t e = kv.find('=');
    m[urlDecode(kv.substr(0, e))] = e == std::string::npos ? "" : urlDecode(kv.substr(e + 1));
    a = b + 1;
  }
  return m;
}
}  // namespace

std::string HttpRequest::param(const std::string& name) const {
  const auto m = parseQuery(query);
  auto it = m.find(name);
  return it == m.end() ? "" : it->second;
}
bool HttpRequest::hasParam(const std::string& name) const { return parseQuery(query).count(name) != 0; }
std::string HttpRequest::header(const std::string& n) const {
  auto it = headers.find(n);
  return it == headers.end() ? "" : it->second;
}

// ------------------------------------------------------------------------------------------------ connection

namespace {

class Conn : public HttpConnection {
 public:
  explicit Conn(SockT s) : s_(s) {}
  bool sendRaw(const std::string& data) override {
    std::size_t off = 0;
    while (off < data.size()) {
      const auto n = ::send(s_, data.data() + off, static_cast<int>(std::min<std::size_t>(data.size() - off, 1 << 20)), 0);
      if (n <= 0) return false;
      off += static_cast<std::size_t>(n);
    }
    return true;
  }
  void send(const HttpResponse& r) override {
    std::string body = r.body;
    if (!r.filePath.empty()) body = readText(r.filePath);
    std::string h = "HTTP/1.1 " + std::to_string(r.status) + " " + statusText(r.status) + "\r\n";
    h += "Content-Type: " + r.contentType + "\r\n";
    h += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    for (const auto& [k, v] : r.headers) h += k + ": " + v + "\r\n";
    h += "Connection: keep-alive\r\n\r\n";
    sendRaw(h);
    sendRaw(body);
  }
  /** read one request; false when the client closed or sent garbage */
  bool read(HttpRequest& req) {
    std::size_t hdrEnd;
    while ((hdrEnd = buf_.find("\r\n\r\n")) == std::string::npos) {
      if (buf_.size() > 65536) return false;
      if (!fill()) return false;
    }
    const std::string head = buf_.substr(0, hdrEnd);
    buf_.erase(0, hdrEnd + 4);
    std::size_t lineEnd = head.find("\r\n");
    const std::string first = head.substr(0, lineEnd);
    const std::size_t s1 = first.find(' '), s2 = first.rfind(' ');
    if (s1 == std::string::npos || s2 == s1) return false;
    req.method = first.substr(0, s1);
    std::string target = first.substr(s1 + 1, s2 - s1 - 1);
    const std::size_t q = target.find('?');
    req.path = urlDecode(target.substr(0, q));
    req.query = q == std::string::npos ? "" : target.substr(q + 1);
    req.headers.clear();
    std::size_t pos = lineEnd == std::string::npos ? head.size() : lineEnd + 2;
    while (pos < head.size()) {
      std::size_t e = head.find("\r\n", pos);
      if (e == std::string::npos) e = head.size();
      const std::string line = head.substr(pos, e - pos);
      const std::size_t c = line.find(':');
      if (c != std::string::npos) {
        std::string k = line.substr(0, c), v = line.substr(c + 1);
        std::transform(k.begin(), k.end(), k.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        v.erase(0, v.find_first_not_of(" \t"));
        req.headers[k] = v;
      }
      pos = e + 2;
    }
    const std::size_t len = req.headers.count("content-length") ? std::stoull(req.headers["content-length"]) : 0;
    if (len > (512u << 20)) return false;
    while (buf_.size() < len)
      if (!fill()) return false;
    req.body = buf_.substr(0, len);
    buf_.erase(0, len);
    return true;
  }

 private:
  bool fill() {
    char tmp[65536];
    const auto n = ::recv(s_, tmp, sizeof tmp, 0);
    if (n <= 0) return false;
    buf_.append(tmp, static_cast<std::size_t>(n));
    return true;
  }
  SockT s_;
  std::string buf_;
};

}  // namespace

// ------------------------------------------------------------------------------------------------ SSE

void SseHub::serve(HttpConnection& c, const std::vector<std::string>& initial) {
  auto cl = std::make_shared<Client>();
  std::uint64_t id;
  {
    std::lock_guard<std::mutex> lk(mu_);
    id = next_++;
    clients_[id] = cl;
  }
  everHad_ = true;
  bool ok = c.sendRaw("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n");
  ok = ok && c.sendRaw("retry: 2000\n\n");
  for (const auto& s : initial)
    if (ok) ok = c.sendRaw(s);
  while (ok) {
    std::string msg;
    {
      std::unique_lock<std::mutex> lk(cl->mu);
      cl->cv.wait_for(lk, std::chrono::seconds(15), [&] { return !cl->q.empty() || cl->closed; });
      if (cl->closed) break;
      if (cl->q.empty()) {
        msg = ": ping\n\n";   // detects a client that went away
      } else {
        while (!cl->q.empty()) {
          msg += cl->q.front();
          cl->q.pop_front();
        }
      }
    }
    ok = c.sendRaw(msg);
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    clients_.erase(id);
  }
  lastClientAt_ = nowMs();
}

void SseHub::publish(const std::string& event, const std::string& jsonData) {
  const std::string s = "event: " + event + "\ndata: " + jsonData + "\n\n";
  std::vector<std::shared_ptr<Client>> list;
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& [id, c] : clients_) list.push_back(c);
  }
  for (auto& c : list) {
    {
      std::lock_guard<std::mutex> lk(c->mu);
      if (c->q.size() > 200) c->q.pop_front();   // a stalled client must not grow memory without bound
      c->q.push_back(s);
    }
    c->cv.notify_one();
  }
}

std::size_t SseHub::count() const {
  std::lock_guard<std::mutex> lk(mu_);
  return clients_.size();
}

// ------------------------------------------------------------------------------------------------ server

bool HttpServer::listen(int port, std::string& error) {
#ifdef _WIN32
  WSADATA d;
  WSAStartup(MAKEWORD(2, 2), &d);
#else
  std::signal(SIGPIPE, SIG_IGN);   // a browser that goes away must not stop the server
#endif
  SockT s = ::socket(AF_INET, SOCK_STREAM, 0);
  if (s == kBad) {
    error = "cannot create a socket";
    return false;
  }
#ifdef _WIN32
  BOOL excl = TRUE;
  setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&excl), sizeof excl);
#else
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
#endif
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<std::uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  if (::bind(s, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || ::listen(s, 32) != 0) {
    closeS(s);
    error = "port " + std::to_string(port) + " is in use";
    return false;
  }
  sock_ = static_cast<std::intptr_t>(s);
  return true;
}

void HttpServer::start(Handler h) {
  handler_ = std::move(h);
  const SockT ls = static_cast<SockT>(sock_);
  std::thread([this, ls] {
    while (!stop_) {
      sockaddr_in from{};
#ifdef _WIN32
      int fl = sizeof from;
#else
      socklen_t fl = sizeof from;
#endif
      SockT c = ::accept(ls, reinterpret_cast<sockaddr*>(&from), &fl);
      if (c == kBad) {
        if (stop_) break;
        continue;
      }
      int one = 1;
      setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
      Handler handler = handler_;
      std::thread([c, handler] {
        Conn conn(c);
        HttpRequest req;
        while (conn.read(req)) {
          try {
            handler(req, conn);
          } catch (...) {
            break;
          }
          const std::string connHdr = req.header("connection");
          if (connHdr == "close" || req.path == "/api/events") break;
        }
        closeS(c);
      }).detach();
    }
  }).detach();
}

void HttpServer::stop() {
  stop_ = true;
  if (sock_ != -1) closeS(static_cast<SockT>(sock_));
  sock_ = -1;
}

}  // namespace aptuner::app
