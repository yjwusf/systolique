// Helpers shared by the test programs.
#pragma once

#include "systolique/arith.h"
#include "systolique/systolic_array.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace systolique::test {

struct Checks {
  unsigned long long done = 0, failed = 0;
  std::string first;
  void fail(int64_t cycle, const std::string &what) {
    if (!failed++) first = "cycle " + std::to_string(cycle) + ": " + what;
  }
};

inline std::string pe_name(unsigned r, unsigned c, unsigned reg) {
  return "PE " + std::to_string(r) + "," + std::to_string(c) + " c" + std::to_string(reg + 1);
}

// A register's value against the d value its provenance names: WS uses the low inputType bits
// (c.asTypeOf(inputType), PE.scala:121, 127); a value that went through d unchanged has them.
inline bool raw_matches(const ArrayConfig &cfg, int64_t reg, const RegSource &s) {
  return arith::sext(reg, cfg.in_bits) == arith::sext(s.value, cfg.in_bits);
}

// Every register whose value came through d must still hold it (WS-only behaviour: a WS PE
// never changes the stationary register, PE.scala:119-131).
inline void check_raw(const SystolicArray &a, Checks &ck) {
  const ArrayConfig &cfg = a.config();
  for (unsigned r = 0; r < cfg.rows(); ++r)
    for (unsigned c = 0; c < cfg.cols(); ++c) {
      const PeView v = a.pe(r, c);
      for (unsigned k = 0; k < 2; ++k) {
        const RegSource &s = v.src[k];
        if (s.d_row < 0) continue;
        ++ck.done;
        const int64_t reg = k ? v.regs.c2 : v.regs.c1;
        if (!raw_matches(cfg, reg, s))
          ck.fail(a.cycle(), pe_name(r, c, k) + " = " + std::to_string(reg) + ", d row " +
                                 std::to_string(s.d_row) + " lane " + std::to_string(s.lane) +
                                 " was " + std::to_string(s.value));
      }
    }
}

template <class M>
M reverse_rows(M m) {
  std::reverse(m.begin(), m.end());
  return m;
}

}  // namespace systolique::test
