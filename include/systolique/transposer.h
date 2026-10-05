// Transposer: Gemmini's AlwaysOutTransposer (Transposer.scala:94-151, Gemmini v0.7.2), the one
// MeshWithDelays instantiates (MeshWithDelays.scala:155). Derived from Gemmini (BSD-3-Clause, see
// LICENSE.gemmini and NOTICE).
//
// A DIM x DIM array of registers. While dir is LEFT, every accepted row shifts the array one
// column left with the new row entering the right column, and the left column is the output;
// while dir is UP the same happens vertically. dir flips after every DIM accepted rows, so the
// matrix shifted in during one pass comes out transposed during the next. counter and dir are
// RegInit (reset), the array registers are not. Output: out_col (outCol.bits; outCol.valid and
// inRow.ready are always 1, Transposer.scala:136-137), from registers only.
//
// Two-phase: set_inputs, eval() (next registers), tick() (commit; out_col of the new cycle).
#pragma once

#include <cstdint>
#include <vector>

namespace systolique {

struct TransposerIn {
  bool in_valid = false;         // inRow.valid; inRow.fire = inRow.valid (ready is always 1)
  std::vector<int64_t> in_row;   // inRow.bits, DIM values
  bool reset = false;
};

class Transposer {
 public:
  explicit Transposer(unsigned dim);  // registers 0 (power-on)
  ~Transposer();

  void set_inputs(const TransposerIn &in);
  void eval();
  void tick();
  // outCol.bits = Mux(dir === LEFT_DIR, left_out, up_out) (Transposer.scala:139-142).
  const std::vector<int64_t> &out_col() const { return out_; }

  unsigned dim() const { return dim_; }
  int64_t reg(unsigned y, unsigned x) const { return regs_[y * dim_ + x]; }  // pes_y_x.reg
  unsigned counter() const { return counter_; }
  unsigned dir() const { return dir_; }

 private:
  void settle();
  unsigned dim_;
  std::vector<int64_t> regs_, next_regs_;  // [row * dim + col]
  unsigned counter_ = 0, dir_ = 0, next_counter_ = 0, next_dir_ = 0;
  TransposerIn in_;
  std::vector<int64_t> out_;
  bool evaluated_ = false;
};

}  // namespace systolique
