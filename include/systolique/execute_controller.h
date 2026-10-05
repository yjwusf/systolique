// ExecuteController (ExecuteController.scala, Gemmini v0.7.2), the splitter: it turns the
// execute commands the ReservationStation issues (config, preload, compute_preloaded,
// compute_accumulated) into the micro-ops Gemmini executes, cycle by cycle:
//   * the TransposePreloadUnroller in front (TransposePreloadUnroller.scala:30-81) and the
//     3-headed command queue (ExecuteController.scala:58-80);
//   * config_ex (:541-582);
//   * the pairing of commands into array passes: a preload alone, a compute overlapped with the
//     next preload, a compute alone, and the flushes it inserts by itself (:532-693);
//   * the A / B / D operand row reads from the scratchpad banks with their fire counters and
//     bank-conflict rules (:244-502), RAW hazards against the tags in the array (:210-230);
//   * the control-signal queue (spad_read_delay + 1 entries, :178-181) that pairs each row's
//     operands with the bank responses and feeds MeshWithDelays (:811-899);
//   * the write-back of the result rows to the scratchpad or the accumulator (:903-951) and the
//     completions to the ReservationStation (:579, 643-679, 970-990).
// It owns the array as a SystolicArray (MeshWithDelays with its Mesh, plus the accounting and
// provenance), and tells it which operation and micro-op every request belongs to
// (RequestNote). Im2col is not modelled: hasIm2Col is false in v0.7.2 (GemminiConfigs.scala:173).
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Clock (two-phase, docs/systolic_array.md). cmd_ready() depends on registers only. The rest of
// its outputs depend on this cycle's inputs through the RTL's combinational paths: the
// write-back and the response pops on the scratchpad's responses (registers), the read requests
// on the banks' readiness, which in turn depends on the write-back. So the owner calls
// writeback(sp_resp) (const) first, asks the banks for their readiness, then set_inputs() with
// everything, eval() (every next register value, no register changes) and tick() (commit;
// runs eval() if needed). out() is valid after eval().
//
// Micro-ops: with a MicroOpTable (set_micro_ops), tick() records the cycles of every command it
// executes (MicroOp::issued .. completed), creates a Request micro-op per array request, an
// OperandRead per operand row read and a ResultRow per result row, and passes (op, micro-op) to
// the array with each request. Recording only reads state.
#pragma once

#include "systolique/fe_util.h"
#include "systolique/frontend_config.h"
#include "systolique/isa.h"
#include "systolique/micro_ops.h"
#include "systolique/scratchpad.h"
#include "systolique/systolic_array.h"

#include <array>
#include <memory>
#include <vector>

namespace systolique {

class ExecuteController {
 public:
  struct In {
    bool cmd_valid = false;          // io.cmd (the ReservationStation's execute issue)
    GemminiCmd cmd;
    std::vector<SpReadResp> sp_resp;  // io.srams.read(i).resp (Scratchpad::resp())
    std::vector<bool> sp_req_ready;   // io.srams.read(i).req.ready (Scratchpad::read_ready())
    std::vector<AccumulatorBank::ReadyView> acc_ready;  // io.acc.read_req(i).ready, by address
  };
  // What depends on the scratchpad's responses only (ExecuteController.scala:811-951).
  struct Writeback {
    std::vector<bool> sp_resp_ready;  // io.srams.read(i).resp.ready
    std::vector<SpWrite> sp_write;    // io.srams.write(i)
    std::vector<AccWrite> acc_write;  // io.acc.write(i)
  };
  struct Out {
    bool cmd_ready = false;
    bool completed_valid = false;  // io.completed
    unsigned completed_id = 0;
    bool busy = false;             // io.busy (:232)
    std::vector<SpReadReq> sp_read;       // io.srams.read(i).req
    std::vector<bool> sp_resp_ready;
    std::vector<SpWrite> sp_write;
    std::vector<AccReadReq> acc_read;     // io.acc.read_req(i)
    std::vector<bool> acc_read_ready;     // the bank's ready for the address asked for
    std::vector<AccWrite> acc_write;
  };

  // Power-on (every register as after the RTL's reset); the array runs `opt` (provenance on by
  // default; ArrayOptions::vcd_path for its own waveform).
  explicit ExecuteController(const FrontendConfig &cfg, ArrayOptions opt = {});
  ~ExecuteController();
  ExecuteController(const ExecuteController &) = delete;
  ExecuteController &operator=(const ExecuteController &) = delete;

  void set_micro_ops(MicroOpTable *t) { uops_ = t; }
  const FrontendConfig &config() const { return cfg_; }

  bool cmd_ready() const { return tpu_q_.enq_ready(); }  // io.cmd.ready (registers only)
  Writeback writeback(const std::vector<SpReadResp> &sp_resp) const;
  void set_inputs(const In &in);
  void eval();
  void tick();
  const Out &out() const { return out_; }
  int64_t cycle() const { return array_->cycle(); }

  // State
  const SystolicArray &array() const { return *array_; }
  unsigned state() const { return state_; }  // 0 waiting_for_cmd, 1 compute, 2 flush, 3 flushing (:73-74)
  unsigned queued() const { return cmd_q_.len(); }
  bool hazard_stall() const { return hazard_stall_; }  // a RAW hazard holds the head this cycle
  bool idle() const;  // no command, nothing pending, no request in flight, the array drained
  uint64_t mesh_requests() const { return mesh_requests_; }
  uint64_t compute_requests() const { return compute_requests_; }
  // The micro-ops of this cycle (after eval): the command at the queue head, the pass being
  // formed, the request whose row enters the Mesh, the result row leaving.
  int64_t head_uop() const;
  int64_t pass_uop() const { return pass_uop_; }
  // the Request micro-op whose row the control queue feeds this cycle, and its operation
  int64_t feed_uop() const { return q_.cntl_valid ? q_.head.pass_uop : -1; }
  int feed_op() const {
    return q_.cntl_valid ? (q_.head.compute_op >= 0 ? q_.head.compute_op : q_.head.preload_op) : -1;
  }
  // the preload micro-op whose result row leaves the array this cycle
  int64_t result_uop() const { return q_.mo.resp_valid && q_.tag.rob_valid ? q_.tag.uop : -1; }

 private:
  struct Tag {  // mesh_tag (:52-62) and the micro-op of the preload that owns it
    bool rob_valid = false;
    unsigned rob_id = 0;
    LocalAddr addr;
    unsigned rows = 0, cols = 0;
    int op = -1;
    int64_t uop = -1;
  };
  struct Cntl {  // ComputeCntlSignals (:698-748)
    bool perform_mul_pre = false, perform_single_mul = false, perform_single_preload = false;
    unsigned a_bank = 0, b_bank = 0, d_bank = 0, a_bank_acc = 0, b_bank_acc = 0, d_bank_acc = 0;
    bool a_read_from_acc = false, b_read_from_acc = false, d_read_from_acc = false;
    bool a_garbage = false, b_garbage = false, d_garbage = false;
    bool accumulate_zeros = false, preload_zeros = false;
    bool a_fire = false, b_fire = false, d_fire = false;
    unsigned a_unpadded_cols = 0, b_unpadded_cols = 0, d_unpadded_cols = 0;
    LocalAddr c_addr;
    unsigned c_rows = 0, c_cols = 0;
    bool a_transpose = false, bd_transpose = false;
    unsigned total_rows = 0;
    bool rob_valid = false;
    unsigned rob_id = 0;
    unsigned dataflow = 0, prop = 0, shift = 0;
    bool first = false;
    // bookkeeping: the commands of this pass and its Request micro-op
    int compute_op = -1, preload_op = -1;
    int64_t compute_uop = -1, preload_uop = -1, pass_uop = -1;
    bool computes = false;  // the compute's A is not the garbage address
  };
  struct Pending {
    bool valid = false;
    unsigned bits = 0;
  };
  // Phase 1a: the mesh feed, response pops, write-back and the mesh completion (:811-990).
  struct Deq {
    MwdOut mo;
    bool cntl_valid = false, cntl_deq_ready = false, cntl_deq_fire = false;
    Cntl head;
    MwdIn mi;
    bool mesh_a_fire = false, mesh_b_fire = false, mesh_d_fire = false, mesh_req_fire = false;
    Writeback wb;
    Tag tag;
    bool resp_fire_rob = false, mesh_completed_fire = false;
    bool busy = false, in_progress = false;
    std::vector<bool> wb_written;  // per bank: written (sp first, then acc)
  };
  // Phase 1b: the FSM, the operand reads, the next control entry (:244-801).
  struct Decision {
    bool start_a = false, start_b = false, start_d = false;
    bool performing_single_preload = false, performing_single_mul = false, performing_mul_pre = false;
    unsigned next_state = 0;
    bool set_perform_single_preload = false, set_perform_mul_pre = false, set_perform_single_mul = false;
    bool clear_performs = false;
    unsigned pop = 0;
    bool do_config = false;
    bool set_pending[2] = {false, false};
    Pending new_pending[2];
    bool set_in_prop_flush = false, in_prop_flush = false;
    bool a_fire = false, b_fire = false, d_fire = false, firing = false, cntl_ready = false;
    bool about_to_fire_all_rows = false;
    unsigned total_rows = 0;
    bool cntl_enq = false;
    Cntl cntl;
    bool pop_pending[2] = {false, false};
    bool hazard_stall = false;
    // operand reads that fire, per bank: which operand ('A', 'B', 'D' or 0)
    std::vector<char> sp_read_operand;
    // TransposePreloadUnroller
    bool tpu_out_fire = false;
    GemminiCmd tpu_out;
    unsigned tpu_pop = 0, tpu_next_state = 0;
    bool b_transposed_and_ws_next = false;
  };
  Tag tag_of(unsigned valid, unsigned id) const;
  Deq deq(const std::vector<SpReadResp> &sp_resp) const;
  void issue(const Deq &q, Decision &d, Out &o) const;
  void record();  // micro-ops of the cycle being committed (tick)
  AddrMap am() const { return amap_; }

  FrontendConfig cfg_;
  AddrMap amap_;
  unsigned block_, rows_bits_;
  // TransposePreloadUnroller (TransposePreloadUnroller.scala)
  MultiHeadedQueue<GemminiCmd> tpu_q_{2, 2};
  unsigned tpu_state_ = 0;  // idle, first_compute, second_preload
  bool b_transposed_and_ws_ = false;
  // ExecuteController registers
  MultiHeadedQueue<GemminiCmd> cmd_q_;
  unsigned state_ = 0;
  unsigned current_dataflow_ = 0;
  bool in_prop_flush_ = false;
  unsigned in_shift_ = 0;
  uint32_t acc_scale_ = 0;
  unsigned activation_ = 0;
  bool a_transpose_ = false, bd_transpose_ = false;
  unsigned a_fire_counter_ = 0, b_fire_counter_ = 0, d_fire_counter_ = 0;
  bool a_fire_started_ = false, b_fire_started_ = false, d_fire_started_ = false;
  uint64_t a_addr_offset_ = 0, a_addr_stride_ = 0, c_addr_stride_ = 0;
  bool perform_single_preload_ = false, perform_single_mul_ = false, perform_mul_pre_ = false;
  Pending pending_[2];
  Queue<Cntl> cntl_q_;
  unsigned output_counter_ = 0;
  std::unique_ptr<SystolicArray> array_;
  // The mesh tags: the bench tag id the array carries indexes this table (the array stores and
  // returns the tag only, MeshWithDelays.scala:206-249).
  std::array<Tag, 256> tags_{};
  unsigned next_tag_ = 0;

  // this cycle
  In in_;
  Deq q_;
  Decision d_;
  Out out_;
  bool evaluated_ = false;
  bool hazard_stall_ = false;
  uint64_t mesh_requests_ = 0, compute_requests_ = 0;

  // bookkeeping (micro-ops)
  MicroOpTable *uops_ = nullptr;
  std::array<int64_t, 64> uop_of_rob_{};  // rob id -> the command micro-op issued with it
  int64_t pass_uop_ = -1;     // the Request micro-op of the pass being formed
  int last_op_ = -1;          // operation of the last command popped (flushes belong to it)
  int64_t last_uop_ = -1;
  int64_t flush_uop_ = -1;
};

}  // namespace systolique
