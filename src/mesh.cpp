// Derived from Gemmini's Mesh.scala (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/mesh.h"

#include <stdexcept>

namespace systolique {

Mesh::Mesh(const ArrayConfig &cfg) : cfg_(cfg), depth_(cfg.tile_latency + 1) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("configuration " + cfg.name + ": " + e);
  const unsigned ntiles = cfg.mesh_rows * cfg.mesh_cols;
  tiles_.reserve(ntiles);
  for (unsigned k = 0; k < ntiles; ++k) tiles_.push_back(std::make_unique<Tile>(cfg));
  regs_.resize(ntiles);
  for (auto &t : regs_) {
    t.a.assign(depth_ * cfg.tile_rows, 0);
    t.b.assign(depth_ * cfg.tile_cols, 0);
    t.d = t.b;
    t.df.assign(depth_ * cfg.tile_cols, 0);
    t.prop = t.shift = t.valid = t.id = t.last = t.df;
  }
  next_ = regs_;
  outq_.resize(cfg.output_delay);
  for (auto &o : outq_) o.resize(cfg);
  outq_next_ = outq_;
  in_.resize(cfg);
  out_.resize(cfg);
  tin_.a.assign(cfg.tile_rows, 0);
  tin_.b.assign(cfg.tile_cols, 0);
  tin_.d.assign(cfg.tile_cols, 0);
  tin_.control.assign(cfg.tile_cols, PeControl{});
  tin_.valid.assign(cfg.tile_cols, 0);
  tin_.id.assign(cfg.tile_cols, 0);
  tin_.last.assign(cfg.tile_cols, 0);
  settle();
}

// The Tiles (and through them the PEs) are owned through unique_ptr; the registers are values.
Mesh::~Mesh() { tiles_.clear(); }

void Mesh::set_inputs(const MeshIn &in) {
  in_ = in;
  evaluated_ = false;
}

// Every Tile input is stage tile_latency of the registers in front of it (Mesh.scala:50-115).
void Mesh::settle() {
  const unsigned tr = cfg_.tile_rows, tc = cfg_.tile_cols, L = depth_ - 1;
  for (size_t t = 0; t < tiles_.size(); ++t) {
    const TileRegs &g = regs_[t];
    for (unsigned i = 0; i < tr; ++i) tin_.a[i] = g.a[L * tr + i];
    for (unsigned j = 0; j < tc; ++j) {
      const unsigned lane = L * tc + j;
      tin_.b[j] = g.b[lane];
      tin_.d[j] = g.d[lane];
      tin_.control[j] = PeControl{g.df[lane], g.prop[lane], g.shift[lane]};
      tin_.valid[j] = g.valid[lane];
      tin_.id[j] = g.id[lane];
      tin_.last[j] = g.last[lane];
    }
    tiles_[t]->set_inputs(tin_);
    tiles_[t]->eval();
  }
  if (outq_.empty())
    bottom_outputs(out_);
  else
    out_ = outq_.back();
}

void Mesh::bottom_outputs(MeshOut &o) const {
  const unsigned tc = cfg_.tile_cols, r = cfg_.mesh_rows - 1;
  for (unsigned c = 0; c < cfg_.mesh_cols; ++c) {
    const TileOut &x = tiles_[r * cfg_.mesh_cols + c]->out();
    for (unsigned j = 0; j < tc; ++j) {
      const unsigned k = c * tc + j;
      o.b[k] = x.b[j];
      o.c[k] = x.c[j];
      o.valid[k] = x.valid[j];
      o.id[k] = x.id[j];
      o.last[k] = x.last[j];
      o.control[k] = x.control[j];
    }
  }
}

// Mesh.scala:39-128: next values of the registers between the tiles and of the output delay.
void Mesh::eval() {
  const unsigned tr = cfg_.tile_rows, tc = cfg_.tile_cols, L = depth_ - 1;
  // Output registers: ShiftRegister(bottom tile outputs, output_delay) (Mesh.scala:117-128).
  if (!outq_.empty()) {
    for (size_t k = outq_.size() - 1; k > 0; --k) outq_next_[k] = outq_[k - 1];
    bottom_outputs(outq_next_[0]);
  }
  for (unsigned r = 0; r < cfg_.mesh_rows; ++r)
    for (unsigned c = 0; c < cfg_.mesh_cols; ++c) {
      TileRegs &t = next_[r * cfg_.mesh_cols + c];
      const TileRegs &o = regs_[r * cfg_.mesh_cols + c];
      t = o;  // registers whose enable is low keep their value
      // a: ShiftRegister(in_a, L+1) from the left (Mesh.scala:50-56); out_a = in_a, so the tile
      // on the left passes its own input on (Tile.scala:47-53).
      const TileIn *left = c == 0 ? nullptr : &tiles_[r * cfg_.mesh_cols + c - 1]->inputs();
      for (unsigned i = 0; i < tr; ++i) {
        for (unsigned k = L; k > 0; --k) t.a[k * tr + i] = o.a[(k - 1) * tr + i];
        t.a[i] = left ? left->a[i] : in_.a[r * tr + i];
      }
      // Everything else comes from above: the Mesh inputs or the outputs of the tile above
      // (out_b/out_c computed, control/valid/id/last passed through its PEs).
      const TileOut *up = r == 0 ? nullptr : &tiles_[(r - 1) * cfg_.mesh_cols + c]->out();
      auto upv = [&](unsigned j) { return up ? up->valid[j] : in_.valid[c * tc + j]; };
      for (unsigned j = 0; j < tc; ++j) {
        const unsigned k = c * tc + j;
        const int64_t ub = up ? up->b[j] : in_.b[k];
        const int64_t ud = up ? up->c[j] : in_.d[k];
        const PeControl uctl = up ? up->control[j] : in_.control[k];
        const unsigned uid = up ? up->id[j] : in_.id[k];
        const unsigned ulast = up ? up->last[j] : in_.last[k];
        // Pipe(valid, bits, L+1): stage s is RegEnable(stage s-1, valid delayed s); the valid
        // delayed s is valid stage s-1 of the same lane (b/d use lane 0: valid.head,
        // Mesh.scala:62, 71; control uses its own lane, Mesh.scala:81-85).
        for (unsigned s = L; s > 0; --s) {
          const unsigned cur = s * tc + j, prev = (s - 1) * tc + j;
          if (o.valid[(s - 1) * tc]) {
            t.b[cur] = o.b[prev];
            t.d[cur] = o.d[prev];
          }
          if (o.valid[prev]) {
            t.df[cur] = o.df[prev];
            t.prop[cur] = o.prop[prev];
            t.shift[cur] = o.shift[prev];
          }
          t.valid[cur] = o.valid[prev];
          t.id[cur] = o.id[prev];
          t.last[cur] = o.last[prev];
        }
        if (upv(0)) {
          t.b[j] = ub;
          t.d[j] = ud;
        }
        if (upv(j)) {
          t.df[j] = uctl.dataflow;
          t.prop[j] = uctl.propagate;
          t.shift[j] = uctl.shift;
        }
        t.valid[j] = upv(j);  // ShiftRegister(in_valid, L+1) (Mesh.scala:91-97)
        t.id[j] = uid;        // Mesh.scala:100-106
        t.last[j] = ulast;    // Mesh.scala:109-115
      }
    }
  evaluated_ = true;
}

void Mesh::tick() {
  if (!evaluated_) eval();
  regs_.swap(next_);
  outq_.swap(outq_next_);
  for (auto &t : tiles_) t->tick();  // the PEs take the values evaluated from the old registers
  settle();
  evaluated_ = false;
}

Mesh::PeInputs Mesh::pe_inputs(unsigned row, unsigned col) const {
  const TileIn &t = tile(row / cfg_.tile_rows, col / cfg_.tile_cols).inputs();
  const unsigned j = col % cfg_.tile_cols;
  PeInputs p;
  p.control = t.control[j];
  p.valid = t.valid[j];
  p.id = t.id[j];
  p.last = t.last[j];
  return p;
}

void Mesh::registers(std::vector<std::pair<std::string, int64_t>> &out) const {
  for (unsigned R = 0; R < cfg_.rows(); ++R)
    for (unsigned C = 0; C < cfg_.cols(); ++C) {
      const std::string p = "mesh_" + std::to_string(R / cfg_.tile_rows) + "_" +
                            std::to_string(C / cfg_.tile_cols) + ".tile_" +
                            std::to_string(R % cfg_.tile_rows) + "_" +
                            std::to_string(C % cfg_.tile_cols) + ".";
      const PeRegs &pe = this->pe(R, C);
      out.emplace_back(p + "c1", pe.c1);
      out.emplace_back(p + "c2", pe.c2);
      out.emplace_back(p + "last_s", pe.last_s);
    }
}

}  // namespace systolique
