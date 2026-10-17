// The request gate of the local API (2026-10-17 audit F-05), checked directly: Host / Origin allow-list, no cross-site
// browser requests, GET only for reading, POST + the per-run token for anything that changes state. The whole server
// (both editions) is also checked from outside by test/server-security.test.js.
#include <map>
#include <string>

#include "check.hpp"
#include "common.hpp"
#include "server.hpp"

using aptuner::app::gateRequest;
using aptuner::app::HttpRequest;

namespace {
HttpRequest req(const std::string& method, const std::string& path, std::map<std::string, std::string> headers, const std::string& query = "") {
  HttpRequest r;
  r.method = method;
  r.path = path;
  r.query = query;
  r.headers = std::move(headers);
  return r;
}
int status(const std::optional<aptuner::app::HttpResponse>& r) { return r ? r->status : 0; }
}  // namespace

TEST(server_gate_host_origin_method_token) {
  const std::string tok = aptuner::app::randomHex(24);
  const std::string H = "127.0.0.1:8090";
  CHECK(tok.size() == 48 && tok != aptuner::app::randomHex(24));   // random, 192 bits
  CHECK(status(gateRequest(req("GET", "/", {{"host", H}}), 8090, tok)) == 0);
  CHECK(status(gateRequest(req("GET", "/", {{"host", "evil.example:8090"}}), 8090, tok)) == 403);   // DNS rebinding
  CHECK(status(gateRequest(req("GET", "/", {}), 8090, tok)) == 403);                                 // no Host
  CHECK(status(gateRequest(req("GET", "/api/presets", {{"host", "localhost:8090"}}), 8090, tok)) == 0);
  CHECK(status(gateRequest(req("GET", "/api/presets", {{"host", "127.0.0.1:8091"}}), 8090, tok)) == 403);   // another port
  CHECK(status(gateRequest(req("GET", "/api/pause", {{"host", H}}), 8090, tok)) == 405);
  CHECK(status(gateRequest(req("GET", "/api/flight/continue", {{"host", H}}), 8090, tok)) == 405);
  CHECK(status(gateRequest(req("POST", "/api/presets", {{"host", H}}), 8090, tok)) == 405);   // a read route is GET only
  CHECK(status(gateRequest(req("GET", "/api/log/param", {{"host", H}}, "save=1"), 8090, tok)) == 405);
  CHECK(status(gateRequest(req("GET", "/api/log/param", {{"host", H}}), 8090, tok)) == 0);
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}}), 8090, tok)) == 403);   // no token
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}, {"x-aptuner-token", tok.substr(1) + "0"}}), 8090, tok)) == 403);
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}, {"x-aptuner-token", tok}}), 8090, tok)) == 0);
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}, {"x-aptuner-token", tok}, {"origin", "http://evil.example"}}), 8090, tok)) == 403);
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}, {"x-aptuner-token", tok}, {"origin", "null"}}), 8090, tok)) == 403);
  CHECK(status(gateRequest(req("POST", "/api/pause", {{"host", H}, {"x-aptuner-token", tok}, {"origin", "http://127.0.0.1:8090"}}), 8090, tok)) == 0);
  CHECK(status(gateRequest(req("GET", "/api/events", {{"host", H}, {"sec-fetch-site", "cross-site"}}), 8090, tok)) == 403);
  CHECK(status(gateRequest(req("POST", "/api/setparam", {{"host", H}, {"x-aptuner-token", tok}}), 8090, tok)) == 0);   // passes the gate, then 404
}
