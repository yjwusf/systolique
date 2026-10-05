// PE: one processing element of Gemmini's systolic array (PE.scala:31-147, Gemmini v0.7.2).
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Two-phase update (docs/systolic_array.md, "Classes and the clock"):
//   set_inputs(in)  the values on in_a, in_b, in_d, in_control, in_valid in this cycle
//   eval()          phase 1, combinational: out_b/out_c and the registers' next values from the
//                   inputs and the current registers; changes no register
//   tick()          phase 2, the rising clock edge: c1, c2, last_s take their next values
// The split is needed because a PE's outputs feed the PE below it in the same cycle (a Tile is
// combinational, Tile.scala:56-107): every PE of a Tile must be evaluated, in column order, from
// the pre-edge registers before any of them commits. tick() evaluates first if eval() has not run
// since the inputs were set or the last edge.
//
// One MacUnit (PE.scala:14-24, 64-65): at most one multiply-accumulate per cycle.
#pragma once

#include "systolique/config.h"
#include "systolique/types.h"

#include <cstdint>

namespace systolique {

// What a PE receives in one cycle. in_id and in_last pass through the PE unchanged
// (PE.scala:79-85); the Tile carries them.
struct PeIn {
  int64_t a = 0, b = 0, d = 0;
  PeControl control;
  unsigned valid = 0;
};

// What a PE drives in one cycle besides the pass-through ports (out_a = in_a, out_control,
// out_valid, out_id, out_last; PE.scala:79-85).
struct PeOut {
  int64_t b = 0;  // WS: in_b + a * weight (MacUnit); OS: in_b
  int64_t c = 0;  // the register being propagated (OS: rounded, shifted and saturated)
};

class PE {
 public:
  // Takes the widths and the dataflow from the configuration; registers start at 0 (power-on,
  // as Verilator's --x-initial 0 makes them).
  explicit PE(const ArrayConfig &cfg);
  ~PE();

  void set_inputs(const PeIn &in);
  const PeIn &inputs() const { return in_; }
  void eval();
  void tick();

  const PeOut &out() const { return out_; }
  const PeRegs &regs() const { return regs_; }

 private:
  Dataflow dataflow_;
  unsigned in_bits_, out_bits_, c_bits_, shift_bits_;
  PeIn in_;
  PeOut out_;
  PeRegs regs_, next_;
  bool evaluated_ = false;
};

}  // namespace systolique
