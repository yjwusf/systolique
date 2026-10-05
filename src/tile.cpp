// Derived from Gemmini's Tile.scala (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/tile.h"

#include "systolique/arith.h"

namespace systolique {

Tile::Tile(const ArrayConfig &cfg)
    : rows_(cfg.tile_rows), cols_(cfg.tile_cols), out_bits_(cfg.out_bits),
      tree_(cfg.tree_reduction()) {
  pes_.reserve(size_t(rows_) * cols_);
  for (unsigned k = 0; k < rows_ * cols_; ++k) pes_.push_back(std::make_unique<PE>(cfg));
  in_.a.assign(rows_, 0);
  in_.b.assign(cols_, 0);
  in_.d.assign(cols_, 0);
  in_.control.assign(cols_, PeControl{});
  in_.valid.assign(cols_, 0);
  in_.id.assign(cols_, 0);
  in_.last.assign(cols_, 0);
  out_.b.assign(cols_, 0);
  out_.c.assign(cols_, 0);
  out_.control.assign(cols_, PeControl{});
  out_.valid.assign(cols_, 0);
  out_.id.assign(cols_, 0);
  out_.last.assign(cols_, 0);
}

// The PEs are owned through unique_ptr; releasing them here makes the order explicit.
Tile::~Tile() { pes_.clear(); }

void Tile::set_inputs(const TileIn &in) {
  in_ = in;
  evaluated_ = false;
}

void Tile::eval() {
  for (unsigned j = 0; j < cols_; ++j) {
    int64_t b = in_.b[j], d = in_.d[j], sum = 0;
    PeIn pin;
    pin.control = in_.control[j];
    pin.valid = in_.valid[j];
    for (unsigned i = 0; i < rows_; ++i) {
      PE &pe = *pes_[i * cols_ + j];
      pin.a = in_.a[i];
      pin.b = tree_ ? 0 : b;  // Tile.scala:59
      pin.d = d;
      pe.set_inputs(pin);
      pe.eval();
      b = pe.out().b;
      d = pe.out().c;
      sum += pe.out().b;
    }
    out_.c[j] = d;
    // tree_reduction: accumulateTree(prods :+ io.in_b(c)) (Tile.scala:117-121, Util.scala:112)
    out_.b[j] = tree_ ? arith::sext(sum + in_.b[j], out_bits_) : b;
    out_.control[j] = in_.control[j];
    out_.valid[j] = in_.valid[j];
    out_.id[j] = in_.id[j];
    out_.last[j] = in_.last[j];
  }
  evaluated_ = true;
}

void Tile::tick() {
  if (!evaluated_) eval();
  for (auto &pe : pes_) pe->tick();
  evaluated_ = false;
}

}  // namespace systolique
