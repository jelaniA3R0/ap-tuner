#include "aptuner/core/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "aptuner/core/jsnum.hpp"

namespace aptuner::core {

namespace {
const Json kNull;
}

Json::Json(const std::vector<double>& v) : type_(Type::Array) {
  a_.reserve(v.size());
  for (double d : v) a_.emplace_back(d);
}

Json Json::obj(std::initializer_list<std::pair<std::string, Json>> kv) {
  Json j = object();
  for (const auto& p : kv) j.set(p.first, p.second);
  return j;
}

bool Json::asBool() const {
  switch (type_) {
    case Type::Bool: return b_;
    case Type::Number: return d_ != 0 && !std::isnan(d_);
    case Type::String: return !s_.empty();
    case Type::Null: return false;
    default: return true;
  }
}

double Json::num() const {
  if (type_ != Type::Number) throw std::runtime_error("json: not a number");
  return d_;
}

double Json::numOr(double fallback) const {
  if (type_ == Type::Number) return d_;
  if (type_ == Type::Bool) return b_ ? 1 : 0;
  if (type_ == Type::String && !s_.empty()) {
    char* end = nullptr;
    const double v = std::strtod(s_.c_str(), &end);
    if (end && *end == 0) return v;
  }
  return fallback;
}

const std::string& Json::str() const {
  if (type_ != Type::String) throw std::runtime_error("json: not a string");
  return s_;
}

std::string Json::strOr(const std::string& fallback) const { return type_ == Type::String ? s_ : fallback; }

const Json::Array& Json::arr() const {
  if (type_ != Type::Array) throw std::runtime_error("json: not an array");
  return a_;
}
Json::Array& Json::arr() {
  if (type_ == Type::Null) type_ = Type::Array;
  if (type_ != Type::Array) throw std::runtime_error("json: not an array");
  return a_;
}
const Json::Object& Json::items() const {
  if (type_ != Type::Object) throw std::runtime_error("json: not an object");
  return o_;
}
Json::Object& Json::items() {
  if (type_ == Type::Null) type_ = Type::Object;
  if (type_ != Type::Object) throw std::runtime_error("json: not an object");
  return o_;
}

std::size_t Json::size() const {
  if (type_ == Type::Array) return a_.size();
  if (type_ == Type::Object) return o_.size();
  return 0;
}

const Json& Json::operator[](const std::string& key) const {
  if (type_ != Type::Object) return kNull;
  for (const auto& kv : o_)
    if (kv.first == key) return kv.second;
  return kNull;
}

const Json& Json::operator[](std::size_t i) const {
  if (type_ != Type::Array || i >= a_.size()) return kNull;
  return a_[i];
}

bool Json::has(const std::string& key) const {
  if (type_ != Type::Object) return false;
  for (const auto& kv : o_)
    if (kv.first == key) return true;
  return false;
}

Json& Json::set(const std::string& key, Json v) {
  Object& o = items();
  for (auto& kv : o)
    if (kv.first == key) {
      kv.second = std::move(v);
      return kv.second;
    }
  o.emplace_back(key, std::move(v));
  return o.back().second;
}

Json& Json::at(const std::string& key) {
  Object& o = items();
  for (auto& kv : o)
    if (kv.first == key) return kv.second;
  o.emplace_back(key, Json());
  return o.back().second;
}

Json& Json::push(Json v) {
  Array& a = arr();
  a.push_back(std::move(v));
  return a.back();
}

bool Json::erase(const std::string& key) {
  if (type_ != Type::Object) return false;
  for (auto it = o_.begin(); it != o_.end(); ++it)
    if (it->first == key) {
      o_.erase(it);
      return true;
    }
  return false;
}

std::vector<double> Json::toVec() const {
  std::vector<double> v;
  if (type_ != Type::Array) return v;
  v.reserve(a_.size());
  for (const auto& x : a_) v.push_back(x.isNumber() ? x.d_ : std::numeric_limits<double>::quiet_NaN());
  return v;
}

bool operator==(const Json& a, const Json& b) {
  if (a.type_ != b.type_) return false;
  switch (a.type_) {
    case Json::Type::Null: return true;
    case Json::Type::Bool: return a.b_ == b.b_;
    case Json::Type::Number: return a.d_ == b.d_;
    case Json::Type::String: return a.s_ == b.s_;
    case Json::Type::Array: return a.a_ == b.a_;
    case Json::Type::Object: return a.o_ == b.o_;
  }
  return false;
}

std::string jsonQuote(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (const unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out.push_back('"');
  return out;
}

void Json::dumpTo(std::string& out) const {
  switch (type_) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += b_ ? "true" : "false"; break;
    case Type::Number:
      if (std::isfinite(d_))
        out += jsToString(d_);
      else
        out += "null";
      break;
    case Type::String: out += jsonQuote(s_); break;
    case Type::Array:
      out.push_back('[');
      for (std::size_t i = 0; i < a_.size(); i++) {
        if (i) out.push_back(',');
        a_[i].dumpTo(out);
      }
      out.push_back(']');
      break;
    case Type::Object:
      out.push_back('{');
      for (std::size_t i = 0; i < o_.size(); i++) {
        if (i) out.push_back(',');
        out += jsonQuote(o_[i].first);
        out.push_back(':');
        o_[i].second.dumpTo(out);
      }
      out.push_back('}');
      break;
  }
}

std::string Json::dump() const {
  std::string out;
  dumpTo(out);
  return out;
}

namespace {

struct Parser {
  const char* p;
  const char* end;
  const char* begin;

  [[noreturn]] void fail(const char* what) const {
    throw std::runtime_error(std::string("json parse error at ") + std::to_string(p - begin) + ": " + what);
  }
  void ws() {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
  }
  bool lit(const char* s) {
    const std::size_t n = std::strlen(s);
    if (static_cast<std::size_t>(end - p) >= n && std::memcmp(p, s, n) == 0) {
      p += n;
      return true;
    }
    return false;
  }
  static void putUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80) {
      out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
      out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
      out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }
  unsigned hex4() {
    if (end - p < 4) fail("bad \\u escape");
    unsigned v = 0;
    for (int i = 0; i < 4; i++) {
      const char c = *p++;
      v <<= 4;
      if (c >= '0' && c <= '9')
        v |= static_cast<unsigned>(c - '0');
      else if (c >= 'a' && c <= 'f')
        v |= static_cast<unsigned>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F')
        v |= static_cast<unsigned>(c - 'A' + 10);
      else
        fail("bad hex digit");
    }
    return v;
  }
  std::string string() {
    if (*p != '"') fail("expected string");
    p++;
    std::string out;
    while (true) {
      if (p >= end) fail("unterminated string");
      const char c = *p++;
      if (c == '"') break;
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (p >= end) fail("bad escape");
      const char e = *p++;
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned cp = hex4();
          if (cp >= 0xD800 && cp < 0xDC00 && end - p >= 6 && p[0] == '\\' && p[1] == 'u') {
            p += 2;
            const unsigned lo = hex4();
            if (lo >= 0xDC00 && lo < 0xE000) cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          }
          putUtf8(out, cp);
          break;
        }
        default: fail("bad escape");
      }
    }
    return out;
  }
  Json value(int depth) {
    if (depth > 512) fail("nesting too deep");
    ws();
    if (p >= end) fail("unexpected end");
    const char c = *p;
    if (c == '{') {
      p++;
      Json o = Json::object();
      ws();
      if (p < end && *p == '}') {
        p++;
        return o;
      }
      while (true) {
        ws();
        std::string k = string();
        ws();
        if (p >= end || *p != ':') fail("expected ':'");
        p++;
        Json v = value(depth + 1);
        o.items().emplace_back(std::move(k), std::move(v));   // duplicates kept in order (last wins on lookup is not needed)
        ws();
        if (p < end && *p == ',') {
          p++;
          continue;
        }
        if (p < end && *p == '}') {
          p++;
          return o;
        }
        fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      p++;
      Json a = Json::array();
      ws();
      if (p < end && *p == ']') {
        p++;
        return a;
      }
      while (true) {
        a.arr().push_back(value(depth + 1));
        ws();
        if (p < end && *p == ',') {
          p++;
          continue;
        }
        if (p < end && *p == ']') {
          p++;
          return a;
        }
        fail("expected ',' or ']'");
      }
    }
    if (c == '"') return Json(string());
    if (lit("true")) return Json(true);
    if (lit("false")) return Json(false);
    if (lit("null")) return Json();
    if (c == '-' || (c >= '0' && c <= '9')) {
      const char* s = p;
      if (*p == '-') p++;
      while (p < end && ((*p >= '0' && *p <= '9') || *p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-'))
        p++;
      const std::string tok(s, p);
      char* e = nullptr;
      const double v = std::strtod(tok.c_str(), &e);
      if (!e || *e != 0) fail("bad number");
      return Json(v);
    }
    fail("unexpected character");
  }
};

}  // namespace

Json Json::parse(const std::string& text) {
  Parser ps{text.data(), text.data() + text.size(), text.data()};
  Json v = ps.value(0);
  ps.ws();
  if (ps.p != ps.end) ps.fail("trailing characters");
  return v;
}

}  // namespace aptuner::core
