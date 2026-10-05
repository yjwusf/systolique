// Bit-exact SInt/UInt arithmetic of Gemmini's Arithmetic typeclass for SInt
// (Arithmetic.scala:90-128) and of the Chisel operators the array uses, on values held
// sign-extended in int64_t (all widths here are <= 32 bits, products <= 33 bits).
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Chisel width rules that matter: SInt `+`/`-` keep max(w1, w2) bits (wrap); connecting a wider
// SInt to a narrower one truncates, a narrower to a wider one sign-extends; asTypeOf(SInt(w))
// reinterprets the low w bits.
//
// Status: validated through the components that use them (every stored RTL trace replays
// exactly, systolique_trace_<config>; live lockstep rtl_<config>).
#pragma once

#include <cstdint>

namespace systolique::arith {

// The low w bits of v as a signed value.
inline int64_t sext(int64_t v, unsigned w) {
  const unsigned s = 64 - w;
  return int64_t(uint64_t(v) << s) >> s;
}
// The low w bits of v as an unsigned value.
inline uint64_t zext(uint64_t v, unsigned w) {
  return w >= 64 ? v : (v & ((uint64_t(1) << w) - 1));
}
// Bit k of a sign-extended value (k beyond the width reads the sign, like `>>>` then [0]).
inline unsigned bit(int64_t v, unsigned k) { return unsigned((v >> (k > 63 ? 63 : k)) & 1); }

// MacUnit: io.out_d := io.in_c.mac(io.in_a, io.in_b) = in_a * in_b + in_c, out_d is out_w bits
// (PE.scala:14-24, Arithmetic.scala:93). The sum wraps at max(product, in_c) bits, which is
// >= out_w, so the low out_w bits are those of the exact sum.
inline int64_t mac(int64_t c, int64_t a, int64_t b, unsigned out_w) { return sext(a * b + c, out_w); }

// self >> u for SInt (Arithmetic.scala:97-108): an arithmetic shift that rounds to nearest, ties
// to even (RISC-V vxrm rnu/rne). self is w bits, u is shift_w bits. The mask
// `(1.U << (u - 1.U)) - 1.U` is 2^shift_w bits wide (a 1-bit literal shifted by a shift_w-bit
// amount), as in the generated Verilog.
inline int64_t round_shift(int64_t self, unsigned u, unsigned w, unsigned shift_w) {
  const unsigned point_five = u == 0 ? 0 : bit(self, u - 1);
  unsigned zeros = 0;
  if (u > 1) {
    const unsigned mask_w = 1u << shift_w;
    const uint64_t mask = zext(zext(uint64_t(1) << (u - 1), mask_w) - 1, mask_w);
    zeros = (zext(uint64_t(self), w) & mask) != 0;
  }
  const unsigned ones_digit = bit(self, u);
  const unsigned r = point_five & (zeros | ones_digit);
  return sext((self >> (u > 63 ? 63 : u)) + r, w);
}

// clippedToWidthOf(SInt(w)) (Arithmetic.scala:122-126): saturate, keep the low w bits.
inline int64_t clip(int64_t v, unsigned w) {
  const int64_t maxsat = (int64_t(1) << (w - 1)) - 1;
  const int64_t minsat = -(int64_t(1) << (w - 1));
  return sext(v > maxsat ? maxsat : (v < minsat ? minsat : v), w);
}

// Util.wrappingAdd(u, n, max_plus_one: Int) (Util.scala:7-15) on w-bit unsigned values:
// Mux(u >= max - n + 1 && n =/= 0, n - (max - u) - 1, u + n), every operation wrapping at w bits
// (w covers the literal max and n here).
inline uint64_t wrapping_add_int(uint64_t u, uint64_t n, uint64_t max_plus_one, unsigned w) {
  const uint64_t max = max_plus_one - 1;
  if (max == 0) return 0;
  const uint64_t lim = zext(zext(max - n, w) + 1, w);
  if (u >= lim && n != 0) return zext(zext(n - zext(max - u, w), w) - 1, w);
  return zext(u + n, w);
}

}  // namespace systolique::arith
