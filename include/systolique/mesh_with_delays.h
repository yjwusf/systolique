// MeshWithDelays: the request handshake, input buffers, transposer, input skew, Mesh, output
// de-skew and tag queues around the systolic array (MeshWithDelays.scala:32-256, Gemmini v0.7.2;
// Transposer.scala AlwaysOutTransposer, TagQueue.scala, chisel3.util.Queue). Derived from Gemmini
// (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Owns, as the RTL instantiates them: a Mesh (MeshWithDelays.scala:167), a Transposer (:155), a
// TagQueue (:222) and the total_rows Queue (:237).
//
// Register for register except for the skew shift registers (docs/microarchitecture.md, "Cycle
// correspondence"): shifted() (MeshWithDelays.scala:70-91) delays lane group i by
// i*(tile_latency+1) cycles with a separate ShiftRegister per lane, a triangle of flops; the
// model keeps one history of the un-skewed vectors (feed_history, resp_history), and stage k of
// lane i's ShiftRegister in cycle t is history entry k+1 (feed) or k (resp) in cycle t. The live
// lockstep bench checks every such stage (feed, resp).
//
// Interface: ready/valid, non-blocking. out() holds this cycle's outputs (ready signals, the
// response, tags_in_progress); they depend on registers only, so a driver reads them, decides
// its valid bits and data, calls set_inputs() and tick(). A handshake happens in the cycle in
// which valid and ready are both 1.
//
// Clock (two-phase, docs/systolic_array.md):
//   set_inputs(in)  this cycle's inputs, reset included
//   eval()          phase 1: next values of every register (own, Transposer, queues, Mesh) from
//                   the inputs and the current registers; no register changes
//   tick()          phase 2: the edge; all registers of the hierarchy commit together, then the
//                   combinational values (skewed feed, outputs) settle for the new cycle.
//                   tick() runs eval() first if it has not run since the inputs were set.
// The split matters here: the Mesh is fed from this module's registers and this module reads the
// Mesh's outputs, so both must compute their next state from the pre-edge values.
#pragma once

#include "systolique/config.h"
#include "systolique/mesh.h"
#include "systolique/tag_queue.h"
#include "systolique/transposer.h"
#include "systolique/types.h"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace systolique {

class MeshWithDelays {
 public:
  // Validates the configuration (std::invalid_argument), allocates the Mesh, Transposer and
  // queues; every register 0 (power-on, as Verilator's --x-initial 0).
  explicit MeshWithDelays(const ArrayConfig &cfg);
  virtual ~MeshWithDelays();
  MeshWithDelays(const MeshWithDelays &) = delete;
  MeshWithDelays &operator=(const MeshWithDelays &) = delete;

  const ArrayConfig &config() const { return cfg_; }
  void set_inputs(const MwdIn &in);
  const MwdIn &inputs() const { return in_; }
  void eval();
  void tick();
  const MwdOut &out() const { return out_; }

  // Children.
  const Mesh &mesh() const { return *mesh_; }
  const Transposer &transposer() const { return *transposer_; }
  const TagQueue &tag_queue() const { return *tagq_; }
  const RowsQueue &rows_queue() const { return *rowsq_; }

  // State of the current cycle.
  unsigned matmul_id() const { return regs_.matmul_id; }
  unsigned fire_counter() const { return regs_.fire_counter; }
  unsigned in_prop() const { return regs_.in_prop; }
  bool req_valid() const { return regs_.req_valid; }
  const MwdReq &req() const { return regs_.req; }
  // The matmul id of this cycle's response rows (out_matmul_id, MeshWithDelays.scala:232).
  unsigned output_id() const;
  // The Mesh inputs of this cycle: the skewed feed, from registers only.
  const MeshIn &mesh_in() const { return mesh_in_; }
  // Entry k of the feed history: the un-skewed Mesh feed k cycles ago (k = 0: this cycle,
  // combinational from the registers).
  const MeshIn &feed_history(unsigned k) const { return feed_hist_[k]; }
  unsigned feed_history_size() const { return unsigned(feed_hist_.size()); }
  // Entry k of the response history: what entered the output de-skew k+1 cycles ago.
  struct RespSample {
    std::vector<int64_t> data;
    unsigned valid = 0, last = 0, id = 0;
  };
  const RespSample &resp_history(unsigned k) const { return resp_hist_[k]; }
  unsigned resp_history_size() const { return unsigned(resp_hist_.size()); }
  // The registers kept one for one, by their name in the generated Verilog of MeshWithDelays
  // (instance.register for the Transposer and the queues).
  void registers(std::vector<std::pair<std::string, int64_t>> &out) const;

 protected:
  // The Mesh receives mesh_in() through this function. In the hardware it is the identity; the
  // provenance twin of SystolicArray (systolic_array.cpp) overrides it. It is called in eval().
  virtual void adjust_mesh_feed(MeshIn &feed) const;

 private:
  // MeshWithDelays.scala:93-108, 183
  struct Regs {
    bool req_valid = false;
    MwdReq req;
    unsigned matmul_id = 0, fire_counter = 0;
    std::vector<int64_t> a_buf, b_buf, d_buf;
    bool a_written = false, b_written = false, d_written = false;
    unsigned in_prop = 0;
    unsigned result_shift = 0;  // RegNext(req.bits.pe_control.shift) (MeshWithDelays.scala:183)
  };
  struct Comb {  // combinational values of the current registers
    bool input_next = false, last_fire = false, pause = false;
    bool req_ready = false, a_ready = false, b_ready = false, d_ready = false;
  };
  Comb comb() const;
  void make_feed(MeshIn &f) const;  // the un-skewed feed from the registers
  void skew();                       // mesh_in_ from feed_hist_
  void resp_sample(RespSample &s) const;
  void settle();                     // comb_, feed, mesh_in_, out_ from the registers

  ArrayConfig cfg_;
  unsigned depth_;  // tile_latency + 1
  std::unique_ptr<Mesh> mesh_;
  std::unique_ptr<Transposer> transposer_;
  std::unique_ptr<TagQueue> tagq_;
  std::unique_ptr<RowsQueue> rowsq_;
  Regs regs_, next_;
  std::deque<MeshIn> feed_hist_;      // feed_hist_[k]: feed of k cycles ago
  std::deque<RespSample> resp_hist_;  // resp_hist_[k]: resp sample of k+1 cycles ago
  RespSample resp_next_;
  MwdIn in_;
  Comb comb_;
  MeshIn mesh_in_, mesh_feed_;
  MwdOut out_;
  bool evaluated_ = false;
};

}  // namespace systolique
