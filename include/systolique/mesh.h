// Mesh: meshRows x meshColumns Tiles with tile_latency + 1 registers in front of every Tile
// (Mesh.scala:17-129, Gemmini v0.7.2). Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini
// and NOTICE).
//
// Registers (Mesh.scala:50-128): in front of tile (r, c) a ShiftRegister of depth L+1 for a (from
// the left) and for in_valid, id and last (from above), a Pipe(valid, x, L+1) for b and d (enabled
// by lane 0's valid) and for the control fields (each by its own lane's valid); the bottom
// tiles' outputs go through output_delay more registers. No reset (no RegInit in Mesh.scala).
//
// Register for register except for one documented merge (docs/microarchitecture.md, "Cycle
// correspondence"): every Pipe has its own chain of valid flops (RegNext(valid) per stage,
// Mesh.scala:42-46) that holds exactly the values of the in_valid ShiftRegister of the same lane
// (Mesh.scala:91-97); the model keeps that one chain (TileRegs::valid) and enables every Pipe
// stage from it. The live lockstep bench checks the RTL's Pipe valid flops against it (pipev).
//
// Clock (two-phase, docs/systolic_array.md): every Tile input comes from a register, so the
// Tiles' combinational values depend on registers only; they are settled at construction and
// after every tick(), which is when out() becomes the outputs of the new cycle.
//   set_inputs(in)  the Mesh inputs of this cycle
//   eval()          phase 1: next values of the inter-tile and output registers from the inputs
//                   and the settled Tile outputs (no register changes)
//   tick()          phase 2: the edge; the registers and every Tile's PEs commit, then the Tiles
//                   settle for the new cycle. tick() runs eval() first if it has not run since
//                   the inputs were set.
#pragma once

#include "systolique/config.h"
#include "systolique/tile.h"
#include "systolique/types.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace systolique {

class Mesh {
 public:
  // Validates the configuration (std::invalid_argument), allocates the Tiles and the registers
  // (all 0) and settles the outputs.
  explicit Mesh(const ArrayConfig &cfg);
  ~Mesh();
  Mesh(const Mesh &) = delete;
  Mesh &operator=(const Mesh &) = delete;

  const ArrayConfig &config() const { return cfg_; }
  void set_inputs(const MeshIn &in);
  const MeshIn &inputs() const { return in_; }
  void eval();
  void tick();
  // The outputs of the current cycle (registers only).
  const MeshOut &out() const { return out_; }

  // State, for SystolicArray, the correspondence checks and the docs.
  const Tile &tile(unsigned r, unsigned c) const { return *tiles_[r * cfg_.mesh_cols + c]; }
  const PeRegs &pe(unsigned row, unsigned col) const {
    return tile(row / cfg_.tile_rows, col / cfg_.tile_cols)
        .pe(row % cfg_.tile_rows, col % cfg_.tile_cols)
        .regs();
  }
  // The in_valid ShiftRegister of tile (r, c), lane j, stage k (0 = first after the input).
  unsigned valid_stage(unsigned r, unsigned c, unsigned j, unsigned k) const {
    return regs_[r * cfg_.mesh_cols + c].valid[k * cfg_.tile_cols + j];
  }
  // The matmul id travelling with that valid (the in_id ShiftRegister, Mesh.scala:100-106).
  unsigned id_stage(unsigned r, unsigned c, unsigned j, unsigned k) const {
    return regs_[r * cfg_.mesh_cols + c].id[k * cfg_.tile_cols + j];
  }
  // What PE (row, col) sees this cycle besides a/b/d: control, valid, id and last of its column,
  // stage tile_latency of its tile's chains (a Tile passes them down unchanged, Tile.scala:56-107;
  // PE.scala:79-85). The PE updates its registers at the end of this cycle iff valid
  // (PE.scala:141-146).
  struct PeInputs {
    PeControl control;
    unsigned valid = 0, id = 0, last = 0;
  };
  PeInputs pe_inputs(unsigned row, unsigned col) const;
  // The PE registers by their name in the generated Verilog of Mesh (tile.pe.register).
  void registers(std::vector<std::pair<std::string, int64_t>> &out) const;

 private:
  // Registers in front of tile (r, c): the ShiftRegister/Pipe chains of depth tile_latency + 1
  // (Mesh.scala:50-115); index [stage * lanes + lane], stage 0 is fed from the input side.
  struct TileRegs {
    std::vector<int64_t> a;                 // ShiftRegister(in_a, L+1)        tileRows lanes
    std::vector<int64_t> b, d;              // Pipe(valid.head, in_b/in_d)     tileColumns lanes
    std::vector<unsigned> df, prop, shift;  // Pipe(valid(j), control) per lane
    std::vector<unsigned> valid, id, last;  // ShiftRegister(in_valid/id/last)
  };

  void settle();                          // Tiles and out_ from the registers
  void bottom_outputs(MeshOut &o) const;  // the bottom Tiles' outputs (before output_delay)

  ArrayConfig cfg_;
  unsigned depth_;  // tile_latency + 1
  std::vector<std::unique_ptr<Tile>> tiles_;   // [r * meshColumns + c]
  std::vector<TileRegs> regs_, next_;
  std::vector<MeshOut> outq_, outq_next_;      // ShiftRegister(.., output_delay) (Mesh.scala:117-128)
  MeshIn in_;
  MeshOut out_;
  TileIn tin_;  // scratch for settle()
  bool evaluated_ = false;
};

}  // namespace systolique
