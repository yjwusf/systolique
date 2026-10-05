// Derived from Gemmini's PE.scala (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/pe.h"

#include "systolique/arith.h"

namespace systolique {

using namespace arith;

PE::PE(const ArrayConfig &cfg)
    : dataflow_(cfg.dataflow),
      in_bits_(cfg.in_bits),
      out_bits_(cfg.out_bits),
      c_bits_(cfg.c_bits()),
      shift_bits_(cfg.shift_bits()) {}

// A PE owns no resources beyond its member registers.
PE::~PE() = default;

void PE::set_inputs(const PeIn &in) {
  in_ = in;
  evaluated_ = false;
}

void PE::eval() {
  const PeRegs &r = regs_;
  PeRegs &n = next_;
  const unsigned prop = in_.control.propagate;
  // val flip = last_s =/= prop; shift_offset = Mux(flip, shift, 0.U)   (PE.scala:89-91)
  const unsigned shift_offset = r.last_s != prop ? in_.control.shift : 0;
  // Dataflow selection (PE.scala:102, 118); a 1-bit dataflow is never "bad" for BOTH.
  const bool os = dataflow_ == Dataflow::OS ||
                  (dataflow_ == Dataflow::BOTH && in_.control.dataflow == 0);
  n = r;
  if (os) {
    // mac_unit.io.in_b := b.asTypeOf(inputType) (PE.scala:106, 113)
    const int64_t mac_b = sext(in_.b, in_bits_);
    out_.b = in_.b;                                                           // PE.scala:105, 112
    if (prop) {
      out_.c = clip(round_shift(r.c1, shift_offset, c_bits_, shift_bits_), out_bits_);  // :104
      n.c2 = sext(mac(r.c2, in_.a, mac_b, out_bits_), c_bits_);              // PE.scala:107-108
      n.c1 = sext(in_.d, c_bits_);                                           // PE.scala:109
    } else {
      out_.c = clip(round_shift(r.c2, shift_offset, c_bits_, shift_bits_), out_bits_);  // :111
      n.c1 = sext(mac(r.c1, in_.a, mac_b, out_bits_), c_bits_);              // PE.scala:114-115
      n.c2 = sext(in_.d, c_bits_);                                           // PE.scala:116
    }
  } else {
    // Weight-stationary: the stationary register is the multiplicand, b the addend, the
    // product-sum leaves through out_b (PE.scala:119-131).
    if (prop) {
      out_.c = sext(r.c1, out_bits_);
      out_.b = mac(in_.b, in_.a, sext(r.c2, in_bits_), out_bits_);
      n.c1 = sext(in_.d, c_bits_);
    } else {
      out_.c = sext(r.c2, out_bits_);
      out_.b = mac(in_.b, in_.a, sext(r.c1, in_bits_), out_bits_);
      n.c2 = sext(in_.d, c_bits_);
    }
  }
  // when (!valid) { c1 := c1; c2 := c2 } (PE.scala:141-146). The DontCare on the MacUnit inputs
  // there is resolved by the FIRRTL compiler to the valid-path values, so out_b is unaffected.
  if (!in_.valid) {
    n.c1 = r.c1;
    n.c2 = r.c2;
  }
  if (in_.valid) n.last_s = prop;  // RegEnable(prop, valid) (PE.scala:89)
  evaluated_ = true;
}

void PE::tick() {
  if (!evaluated_) eval();
  regs_ = next_;
  evaluated_ = false;  // the outputs belong to the old registers until the next eval()
}

}  // namespace systolique
