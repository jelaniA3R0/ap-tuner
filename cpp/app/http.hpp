// Minimal HTTP/1.1 server (no dependencies): one thread per connection, keep-alive, Server-Sent Events hub.
// Only binds to 127.0.0.1: the app serves its own window on this PC.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aptuner::app {

struct HttpRequest {
  std::string method, path, query;
  std::map<std::string, std::string> headers;   // lower-case names
  std::string body;
  /** decoded query parameter ("" when missing) */
  std::string param(const std::string& name) const;
  bool hasParam(const std::string& name) const;
  std::string header(const std::string& lowerName) const;
};

struct HttpResponse {
  int status = 200;
  std::string contentType = "application/json";
  std::string body;
  std::vector<std::pair<std::string, std::string>> headers;
  std::string filePath;   // when set, the file is streamed as the body
};

/** connection handed to a handler: for responses, or kept open for an event stream */
class HttpConnection {
 public:
  virtual ~HttpConnection() = default;
  virtual bool sendRaw(const std::string& data) = 0;
  virtual void send(const HttpResponse& r) = 0;
};

class SseHub {
 public:
  /** serve an event stream on this connection until the client goes away (blocks the connection thread) */
  void serve(HttpConnection& c, const std::vector<std::string>& initial);
  void publish(const std::string& event, const std::string& jsonData);
  std::size_t count() const;
  bool everHadClient() const { return everHad_; }
  double lastClientAt() const { return lastClientAt_; }

 private:
  struct Client {
    std::mutex mu;
    std::condition_variable cv;
    std::deque<std::string> q;
    bool closed = false;
  };
  mutable std::mutex mu_;
  std::map<std::uint64_t, std::shared_ptr<Client>> clients_;
  std::uint64_t next_ = 1;
  std::atomic<bool> everHad_{false};
  std::atomic<double> lastClientAt_{0};
};

class HttpServer {
 public:
  using Handler = std::function<void(const HttpRequest&, HttpConnection&)>;
  /** bind 127.0.0.1:port; false (with a reason) when the port is taken */
  bool listen(int port, std::string& error);
  /** accept connections on a background thread */
  void start(Handler h);
  void stop();

 private:
  std::intptr_t sock_ = -1;
  Handler handler_;
  std::atomic<bool> stop_{false};
};

std::string statusText(int code);
std::string urlDecode(const std::string& s);
double nowMs();

}  // namespace aptuner::app
