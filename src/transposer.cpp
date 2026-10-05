// Derived from Gemmini's Transposer.scala (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/transposer.h"

#include "systolique/arith.h"
#include "systolique/config.h"

#include <algorithm>

namespace systolique {

Transposer::Transposer(unsigned dim)
    : dim_(dim), regs_(size_t(dim) * dim, 0), next_regs_(regs_), out_(dim, 0) {
  in_.in_row.assign(dim, 0);
  settle();
}

// Registers are values; nothing else is owned.
Transposer::~Transposer() = default;

void Transposer::set_inputs(const TransposerIn &in) {
  in_ = in;
  evaluated_ = false;
}

void Transposer::settle() {
  // left_out = column 0 (outL of pes(row)(0)), up_out = row 0 (Transposer.scala:136-142)
  for (unsigned k = 0; k < dim_; ++k) out_[k] = dir_ == 0 ? regs_[k * dim_] : regs_[k];
}

void Transposer::eval() {
  next_regs_ = regs_;
  next_counter_ = counter_;
  next_dir_ = dir_;
  if (in_.in_valid) {  // every PE's reg is RegEnable(.., inRow.fire) (Transposer.scala:110, 133)
    const std::vector<int64_t> &row = in_.in_row;
    for (unsigned y = 0; y < dim_; ++y)
      for (unsigned x = 0; x < dim_; ++x)
        // Mux(dir === LEFT_DIR, inR, inD): from the right neighbour (or inRow(y) at the right
        // edge), or from below (or inRow(x) at the bottom) (Transposer.scala:110, 120-129)
        next_regs_[y * dim_ + x] =
            dir_ == 0 ? (x == dim_ - 1 ? row[y] : regs_[y * dim_ + x + 1])
                      : (y == dim_ - 1 ? row[x] : regs_[(y + 1) * dim_ + x]);
    // counter := wrappingAdd(counter, 1.U, dim); dir flips at counter == dim-1 (:144-150)
    if (counter_ == dim_ - 1) next_dir_ = dir_ ^ 1;
    next_counter_ = unsigned(
        arith::wrapping_add_int(counter_, 1, dim_, std::max(1u, log2_ceil(dim_))));
  }
  if (in_.reset) {  // RegInit (Transposer.scala:117-118)
    next_counter_ = 0;
    next_dir_ = 0;
  }
  evaluated_ = true;
}

void Transposer::tick() {
  if (!evaluated_) eval();
  regs_.swap(next_regs_);
  counter_ = next_counter_;
  dir_ = next_dir_;
  settle();
  evaluated_ = false;
}

}  // namespace systolique
