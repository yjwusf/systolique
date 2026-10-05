// LoopMatmul (LoopMatmul.scala, Gemmini v0.7.2): unrolls gemmini_loop_ws (LOOP_WS and its five
// configuration commands) into mvin / preload / compute / mvout commands with its five sub-FSMs
// (LdA, LdB, LdD, Execute, StC), two loops in flight, the A/B load arbiter (WeightedArbiter) and
// the reservation-station occupancy counters that throttle it. Other commands pass through
// while no loop is configured. The LAYERNORM / SOFTMAX mvout sequences of StC are not modelled
// (unsupported() reports them). Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and
// NOTICE).
//
// Clock (two-phase): out() (the unrolled command, busy, in_ready) depends on registers only;
// set_inputs() gives the cycle's enqueue, io.out.ready and the ReservationStation's completion
// counts; eval() computes the next registers without changing any; tick() commits.
//
// Bookkeeping: every command it emits carries the operation of the LOOP_WS that configured its
// loop and, as parent micro-op, that LOOP_WS command's micro-op (GemminiCmd::op / parent_uop).
#pragma once

#include "systolique/isa.h"

#include <array>
#include <string>

namespace systolique {

class LoopMatmul {
 public:
  struct In {
    bool in_valid = false;  // enqueue into the input queue (Queue(io.in), LoopMatmul.scala:808)
    GemminiCmd in_bits;
    bool out_ready = false;  // io.out.ready
    unsigned ld_completed = 0, st_completed = 0, ex_completed = 0;  // the ReservationStation's counts
  };
  struct Out {
    bool out_valid = false;
    GemminiCmd out_bits;
    bool busy = false;            // io.busy (:810)
    bool in_ready = false;        // io.in.ready (the input queue)
  };

  explicit LoopMatmul(const FrontendConfig &cfg);
  ~LoopMatmul();
  LoopMatmul(const LoopMatmul &) = delete;
  LoopMatmul &operator=(const LoopMatmul &) = delete;

  const Out &out() const { return out_; }
  void set_inputs(const In &in);
  void eval();
  void tick();
  bool in_fire() const { return in_.in_valid && out_.in_ready; }
  bool cmd_deq() const { return deq_; }  // after eval(): the input queue's head left this cycle
  const GemminiCmd &cmd_head() const { return r_.cmdq.deq_bits(); }
  // Empty, or why the configured loops cannot be modelled.
  std::string unsupported() const;
  bool idle() const;

 private:
  struct LoopState {  // LoopMatmulState (LoopMatmul.scala:693-764)
    uint64_t max_k = 0, max_j = 0, max_i = 0, pad_k = 0, pad_j = 0, pad_i = 0;
    uint64_t a_dram_addr = 0, b_dram_addr = 0, d_dram_addr = 0, c_dram_addr = 0;
    uint64_t a_dram_stride = 0, b_dram_stride = 0, d_dram_stride = 0, c_dram_stride = 0;
    bool a_transpose = false, b_transpose = false;
    unsigned act = 0;
    bool low_d = false, full_c = false, ex_accumulate = false;
    unsigned a_ex_spad_id = 0, b_ex_spad_id = 0;
    bool configured = false, running = false;
    bool lda_started = false, ldb_started = false, ex_started = false, ldd_started = false, st_started = false;
    bool lda_completed = false, ldb_completed = false, ex_completed = false, ldd_completed = false,
         st_completed = false;
    uint64_t a_addr_start = 0, b_addr_end = 0, resadd_addr_start = 0;
    int op = -1;          // bookkeeping: the LOOP_WS command's operation and micro-op
    int64_t uop = -1;
    bool all_completed() const {
      return lda_completed && ldb_completed && ldd_completed && ex_completed && st_completed;
    }
    void reset() {
      configured = running = false;
      lda_started = ldb_started = ex_started = ldd_started = st_started = false;
      lda_completed = ldb_completed = ex_completed = ldd_completed = st_completed = false;
    }
  };
  struct LdReq {  // LoopMatmulLdAReq / LdBReq / LdDReq
    uint64_t max_i = 0, max_k = 0, max_j = 0, pad_i = 0, pad_k = 0, pad_j = 0;
    uint64_t dram_addr = 0, dram_stride = 0;
    bool transpose = false, low_d = false, is_resadd = false;
    uint64_t addr_start = 0, addr_end = 0;
    unsigned loop_id = 0;
  };
  struct ExReq {  // LoopMatmulExecuteReq
    uint64_t max_j = 0, max_k = 0, max_i = 0, pad_j = 0, pad_k = 0, pad_i = 0;
    bool a_tranpose = false, b_tranpose = false, accumulate = false;
    uint64_t a_addr_start = 0, b_addr_end = 0, c_addr_start = 0;
    unsigned loop_id = 0;
    bool skip = false;
  };
  struct StReq {  // LoopMatmulStCReq
    uint64_t max_k = 0, max_j = 0, max_i = 0, pad_j = 0, pad_i = 0;
    uint64_t dram_addr = 0, dram_stride = 0;
    bool full_c = false;
    unsigned act = 0;
    uint64_t addr_start = 0;
    unsigned loop_id = 0;
    bool is_resadd = false;
  };
  struct Sub {  // state and iterators of one sub-FSM
    unsigned state = 0;  // 0 = idle; LdX: 1 = ld; Ex: 1 = pre, 2 = comp; StC: 1 = st
    uint64_t i = 0, j = 0, k = 0;
    bool idle() const { return state == 0; }
  };
  struct Regs {
    Queue<GemminiCmd> cmdq{2};
    std::array<LoopState, 2> loops;
    unsigned head_loop_id = 0;
    bool is_resadd = false;
    uint64_t ld_d_addr_start = 0, ex_c_addr_start = 0, st_c_addr_start = 0;
    unsigned ld_util = 0, st_util = 0, ex_util = 0;
    LdReq lda_req, ldb_req, ldd_req;
    ExReq ex_req;
    StReq st_req;
    Sub lda, ldb, ldd, ex, stc;
  };
  struct SubCmd {
    bool valid = false;
    GemminiCmd cmd;
  };
  // Register-only values of the current state.
  struct Comb {
    SubCmd lda, ldb, ldd, ex, stc;
    bool loop_configured = false;
    unsigned loop_being_configured_id = 0;
    bool is_loop_run_cmd = false, is_loop_config_cmd = false, is_loop_cmd = false;
    int arb_choice = -1;  // 0 stC, 1 ex, 2 ldD, 3 ldA/B
    int ab_choice = -1;   // 0 = A, 1 = B
  };
  SubCmd lda_cmd(const Regs &r) const;
  SubCmd ldb_cmd(const Regs &r) const;
  SubCmd ldd_cmd(const Regs &r) const;
  SubCmd ex_cmd(const Regs &r) const;
  SubCmd stc_cmd(const Regs &r) const;
  void settle();  // c_ and out_ from r_
  uint64_t dram_add(uint64_t base, uint64_t off) const {
    return (base + (off & 0xffffffffull)) & mask_bits(cfg_.core_max_addr_bits);  // castDramOffset
  }

  FrontendConfig cfg_;
  AddrMap amap_;
  unsigned block_, max_block_len_, max_block_len_acc_, max_addr_, max_acc_addr_;
  unsigned mvin_rows_bits_, mvin_cols_bits_;
  Regs r_, n_;
  Comb c_;
  Out out_;
  In in_;
  bool deq_ = false;
  bool evaluated_ = false;
};

}  // namespace systolique
