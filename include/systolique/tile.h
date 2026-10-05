// Tile: a combinational tileRows x tileColumns grid of PEs (Tile.scala:16-131, Gemmini v0.7.2).
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// A Tile has no registers of its own: in_a is broadcast right along each PE row (Tile.scala:
// 47-53), b, d (through out_c), control, valid, id and last go down each PE column (:56-107),
// the outputs are the bottom row's (:110-125). With tree reduction (a WS-only array with
// tileRows > 1) each PE gets b = 0 and the Tile sums the column's products and in_b
// (accumulateTree, Tile.scala:117-121, Util.scala:112-126).
//
// Two-phase update as PE (pe.h): eval() evaluates every PE column top to bottom from the
// current registers; tick() commits every PE.
#pragma once

#include "systolique/config.h"
#include "systolique/pe.h"
#include "systolique/types.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace systolique {

struct TileIn {
  std::vector<int64_t> a;             // tileRows
  std::vector<int64_t> b, d;          // tileColumns
  std::vector<PeControl> control;     // tileColumns
  std::vector<unsigned> valid, id, last;
};

struct TileOut {
  std::vector<int64_t> b, c;          // tileColumns: the bottom PEs' out_b (or the tree sum), out_c
  std::vector<PeControl> control;     // passed through (Tile.scala:110-125)
  std::vector<unsigned> valid, id, last;
};

class Tile {
 public:
  // Allocates tileRows x tileColumns PEs.
  explicit Tile(const ArrayConfig &cfg);
  ~Tile();
  Tile(const Tile &) = delete;
  Tile &operator=(const Tile &) = delete;

  void set_inputs(const TileIn &in);
  const TileIn &inputs() const { return in_; }
  void eval();
  void tick();
  const TileOut &out() const { return out_; }

  // PE (i, j) of this tile, i = row within the tile, j = column.
  const PE &pe(unsigned i, unsigned j) const { return *pes_[i * cols_ + j]; }

 private:
  unsigned rows_, cols_, out_bits_;
  bool tree_;
  std::vector<std::unique_ptr<PE>> pes_;  // [i * cols + j]
  TileIn in_;
  TileOut out_;
  bool evaluated_ = false;
};

}  // namespace systolique
