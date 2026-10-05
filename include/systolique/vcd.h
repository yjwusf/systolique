// VcdWriter: a minimal Value Change Dump (IEEE 1364 section 18) writer, standard library only.
// SystolicArray uses it when ArrayOptions::vcd_path is set (off by default) to dump the ports and
// every PE's state each cycle; it only reads state, so timing does not change (test
// systolique_vcd compares every output of runs with and without it).
//
// Usage: add() every variable (before the first commit), then per time step set() the values and
// commit(time); only changed values are written. Scopes are one level below the root module.
#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace systolique {

class VcdWriter {
 public:
  // Opens `path` for writing (std::runtime_error if it cannot); `root` names the top scope.
  VcdWriter(const std::string &path, const std::string &root, const std::string &timescale = "1ns");
  ~VcdWriter();  // flushes and closes the file
  VcdWriter(const VcdWriter &) = delete;
  VcdWriter &operator=(const VcdWriter &) = delete;

  // A variable of `width` bits (1..64) in `scope` ("" = the root); returns its handle.
  int add(const std::string &scope, const std::string &name, unsigned width);
  // The value of a variable at the current time step (the low `width` bits are kept).
  void set(int id, uint64_t value);
  void set_signed(int id, int64_t value) { set(id, uint64_t(value)); }
  // Ends the time step `time` (non-decreasing): writes the header on the first call, then every
  // variable whose value changed since the previous step.
  void commit(uint64_t time);

 private:
  struct Var {
    std::string scope, name, code;
    unsigned width;
    uint64_t value = 0;
    bool dirty = true;
  };
  void header();
  void write_value(const Var &v);

  std::ofstream out_;
  std::string root_, timescale_;
  std::vector<Var> vars_;
  bool started_ = false;
};

}  // namespace systolique
