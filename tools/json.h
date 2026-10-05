// A minimal JSON reader for the request files of systolique_dump (objects, arrays, strings,
// numbers, true/false/null; no \u escapes beyond ASCII). Standard library only.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace systolique::json {

struct Value {
  enum class Type { Null, Bool, Number, String, Array, Object } type = Type::Null;
  bool b = false;
  double num = 0;
  std::string str;
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  bool has(const std::string &k) const { return type == Type::Object && obj.count(k); }
  const Value &operator[](const std::string &k) const {
    static const Value null;
    auto it = obj.find(k);
    return it == obj.end() ? null : it->second;
  }
  bool is_string() const { return type == Type::String; }
  int64_t as_int() const {
    if (type != Type::Number) throw std::runtime_error("JSON: a number is expected");
    return int64_t(num);
  }
  bool as_bool() const {
    if (type == Type::Bool) return b;
    if (type == Type::Number) return num != 0;
    throw std::runtime_error("JSON: true/false is expected");
  }
  const std::string &as_string() const {
    if (type != Type::String) throw std::runtime_error("JSON: a string is expected");
    return str;
  }
};

class Parser {
 public:
  explicit Parser(const std::string &text) : s_(text) {}
  Value parse() {
    Value v = value();
    ws();
    if (i_ != s_.size()) fail("trailing characters");
    return v;
  }

 private:
  [[noreturn]] void fail(const std::string &what) const {
    throw std::runtime_error("JSON: " + what + " at offset " + std::to_string(i_));
  }
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\r' || s_[i_] == '\t'))
      ++i_;
  }
  bool eat(char c) {
    ws();
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  void expect(char c) {
    if (!eat(c)) fail(std::string("'") + c + "' expected");
  }
  std::string string() {
    expect('"');
    std::string o;
    while (i_ < s_.size() && s_[i_] != '"') {
      char c = s_[i_++];
      if (c == '\\' && i_ < s_.size()) {
        const char e = s_[i_++];
        c = e == 'n' ? '\n' : e == 't' ? '\t' : e;
      }
      o += c;
    }
    if (i_ >= s_.size()) fail("unterminated string");
    ++i_;
    return o;
  }
  Value value() {
    ws();
    if (i_ >= s_.size()) fail("value expected");
    Value v;
    const char c = s_[i_];
    if (c == '{') {
      ++i_;
      v.type = Value::Type::Object;
      if (eat('}')) return v;
      do {
        ws();
        const std::string k = string();
        expect(':');
        v.obj[k] = value();
      } while (eat(','));
      expect('}');
    } else if (c == '[') {
      ++i_;
      v.type = Value::Type::Array;
      if (eat(']')) return v;
      do v.arr.push_back(value());
      while (eat(','));
      expect(']');
    } else if (c == '"') {
      v.type = Value::Type::String;
      v.str = string();
    } else if (s_.compare(i_, 4, "true") == 0 || s_.compare(i_, 5, "false") == 0) {
      v.type = Value::Type::Bool;
      v.b = s_[i_] == 't';
      i_ += v.b ? 4 : 5;
    } else if (s_.compare(i_, 4, "null") == 0) {
      i_ += 4;
    } else {
      size_t n = 0;
      try {
        v.num = std::stod(s_.substr(i_), &n);
      } catch (const std::exception &) {
        fail("bad value");
      }
      v.type = Value::Type::Number;
      i_ += n;
    }
    return v;
  }

  const std::string &s_;
  size_t i_ = 0;
};

inline Value parse(const std::string &text) { return Parser(text).parse(); }

}  // namespace systolique::json
