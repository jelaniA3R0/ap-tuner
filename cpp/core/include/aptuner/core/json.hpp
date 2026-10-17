// Minimal JSON value, parser and writer (no dependencies). Value semantics; objects keep their insertion order,
// like JavaScript objects, so documents round-trip in the same order. Numbers are doubles, written the way
// JavaScript's JSON.stringify writes them.
#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace aptuner::core {

class Json {
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };
  using Array = std::vector<Json>;
  using Object = std::vector<std::pair<std::string, Json>>;

  Json() = default;
  Json(std::nullptr_t) {}
  Json(bool b) : type_(Type::Bool), b_(b) {}
  Json(double d) : type_(Type::Number), d_(d) {}
  Json(int v) : type_(Type::Number), d_(v) {}
  Json(long v) : type_(Type::Number), d_(static_cast<double>(v)) {}
  Json(long long v) : type_(Type::Number), d_(static_cast<double>(v)) {}
  Json(unsigned v) : type_(Type::Number), d_(v) {}
  Json(unsigned long v) : type_(Type::Number), d_(static_cast<double>(v)) {}
  Json(unsigned long long v) : type_(Type::Number), d_(static_cast<double>(v)) {}
  Json(const char* s) : type_(Type::String), s_(s) {}
  Json(std::string s) : type_(Type::String), s_(std::move(s)) {}
  Json(Array a) : type_(Type::Array), a_(std::move(a)) {}
  Json(Object o) : type_(Type::Object), o_(std::move(o)) {}
  Json(const std::vector<double>& v);

  static Json array() { return Json(Array{}); }
  static Json object() { return Json(Object{}); }
  /** build an object from {key, value} pairs */
  static Json obj(std::initializer_list<std::pair<std::string, Json>> kv);

  Type type() const { return type_; }
  bool isNull() const { return type_ == Type::Null; }
  bool isBool() const { return type_ == Type::Bool; }
  bool isNumber() const { return type_ == Type::Number; }
  bool isString() const { return type_ == Type::String; }
  bool isArray() const { return type_ == Type::Array; }
  bool isObject() const { return type_ == Type::Object; }

  bool asBool() const;
  double num() const;   // number (throws for other types)
  /** number, bool -> 0/1, numeric string -> value; anything else -> fallback */
  double numOr(double fallback) const;
  const std::string& str() const;
  std::string strOr(const std::string& fallback) const;
  const Array& arr() const;
  Array& arr();
  const Object& items() const;
  Object& items();
  std::size_t size() const;

  /** object member (a shared null Json when missing / not an object) */
  const Json& operator[](const std::string& key) const;
  const Json& operator[](const char* key) const { return (*this)[std::string(key)]; }
  /** array element (null Json when out of range) */
  const Json& operator[](std::size_t i) const;
  const Json& operator[](int i) const { return (*this)[static_cast<std::size_t>(i)]; }
  bool has(const std::string& key) const;
  /** set or replace an object member (turns null into an object) */
  Json& set(const std::string& key, Json v);
  /** mutable member, created as null when missing (turns null into an object) */
  Json& at(const std::string& key);
  /** append to an array (turns null into an array) */
  Json& push(Json v);
  bool erase(const std::string& key);

  std::vector<double> toVec() const;   // array of numbers (null -> NaN)

  /** compact JSON text (numbers as JavaScript prints them; NaN / Infinity -> null, as JSON.stringify) */
  std::string dump() const;
  static Json parse(const std::string& text);   // throws std::runtime_error with the position

  friend bool operator==(const Json& a, const Json& b);

 private:
  void dumpTo(std::string& out) const;
  Type type_ = Type::Null;
  bool b_ = false;
  double d_ = 0;
  std::string s_;
  Array a_;
  Object o_;
};

/** JSON string escaping (quotes included) */
std::string jsonQuote(const std::string& s);

}  // namespace aptuner::core
