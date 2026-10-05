// Gemmini's command path between the core and the controllers, wired as Controller.scala wires
// it (Gemmini v0.7.2): raw_cmd_q (Queue of 2, Controller.scala:131-136), LoopConv's input queue
// (LoopConv passes every command through while no conv loop is configured; loop_conv_ws itself is
// not modelled and is dropped), LoopMatmul, the unrolled-command queue (Queue of 2, :167), and
// the ReservationStation with its three issue ports (:233-246). Flush, counter and clock-gate
// commands are taken off the unrolled queue without entering the ReservationStation (:358-401).
// The RTL top rtl/ex/src/CmdTop.scala has the same ports. Derived from Gemmini (BSD-3-Clause, see
// LICENSE.gemmini and NOTICE).
//
// Clock (two-phase): out() (cmd_ready, busy, the issue ports) depends on registers only;
// set_inputs() gives the core's command, the controllers' ready bits and the completion;
// eval() computes the next registers and the matmul completion counts (the
// ReservationStation's combinational outputs); tick() commits.
//
// Micro-ops: with a MicroOpTable, tick() records when each command entered the
// ReservationStation (MicroOp::alloc) and was issued, and creates the command micro-op of every
// command LoopMatmul emits (its parent: the loop_ws command's micro-op).
#pragma once

#include "systolique/fe_util.h"
#include "systolique/isa.h"
#include "systolique/loop_matmul.h"
#include "systolique/micro_ops.h"
#include "systolique/reservation_station.h"

#include <memory>

namespace systolique {

class CommandPath {
 public:
  struct In {
    bool cmd_valid = false;  // io.cmd (the core)
    GemminiCmd cmd;
    bool issue_ready[3] = {false, false, false};  // ld, ex, st controllers' cmd.ready
    bool completed_valid = false;                 // the completion arbiter's output
    unsigned completed_id = 0;
  };
  struct IssuePort {
    bool valid = false;
    unsigned funct = 0, rob_id = 0;
    uint64_t rs1 = 0, rs2 = 0;
    GemminiCmd cmd;  // the whole command (with its bookkeeping)
  };
  struct Out {
    bool cmd_ready = false;     // io.cmd.ready
    bool busy = false;          // io.busy without the scratchpad's (Controller.scala:330)
    bool loop_matmul_busy = false;
    IssuePort issue[3];         // ld, ex, st
    unsigned matmul_completed[3] = {0, 0, 0};  // after eval(): ld, ex, st
  };

  explicit CommandPath(const FrontendConfig &cfg);
  ~CommandPath();
  CommandPath(const CommandPath &) = delete;
  CommandPath &operator=(const CommandPath &) = delete;

  void set_micro_ops(MicroOpTable *t) { uops_ = t; }
  const Out &out() const { return out_; }
  void set_inputs(const In &in);
  void eval();
  void tick();
  int64_t cycle() const { return cycle_; }
  bool idle() const;
  std::string unsupported() const;  // LoopMatmul features the model does not have, else ""
  const ReservationStation &reservation_station() const { return *rs_; }
  const LoopMatmul &loop_matmul() const { return *lm_; }

 private:
  void settle();
  FrontendConfig cfg_;
  std::unique_ptr<ReservationStation> rs_;
  std::unique_ptr<LoopMatmul> lm_;
  Queue<GemminiCmd> raw_q_{2}, conv_q_{2}, unrolled_q_{2};
  In in_;
  Out out_;
  // this cycle
  bool alloc_valid_ = false, un_deq_ = false, conv_deq_ = false, lm_in_fire_ = false, raw_deq_ = false;
  bool evaluated_ = false;
  int64_t cycle_ = 0;
  MicroOpTable *uops_ = nullptr;
};

}  // namespace systolique
