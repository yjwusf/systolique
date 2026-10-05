// Gemmini's command path (Controller.scala:124-170, 233-246, 312-401, Gemmini v0.7.2), as
// rtl/ex/src/CmdTop.scala wires it. Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and
// NOTICE).
#include "systolique/command_path.h"

#include <stdexcept>

namespace systolique {

namespace {
bool is_loop_conv(unsigned f) { return f >= LOOP_CONV_WS && f <= LOOP_CONV_WS_CONFIG_6; }
bool is_loop_matmul(unsigned f) { return f >= LOOP_WS && f <= LOOP_WS_CONFIG_STRIDES_DC; }
bool bypasses_rs(unsigned f) { return f == FLUSH_CMD || f == CLKGATE_EN || f == COUNTER_OP; }

UopKind kind_of(unsigned funct) {
  switch (funct) {
    case CONFIG_CMD: return UopKind::Config;
    case PRELOAD_CMD: return UopKind::Preload;
    case COMPUTE_AND_FLIP_CMD: return UopKind::ComputePreloaded;
    case COMPUTE_AND_STAY_CMD: return UopKind::ComputeAccumulated;
    case LOAD_CMD:
    case LOAD2_CMD:
    case LOAD3_CMD: return UopKind::Mvin;
    case STORE_CMD: return UopKind::Mvout;
    default: return is_loop_matmul(funct) ? UopKind::LoopCmd : UopKind::OtherCmd;
  }
}
}  // namespace

CommandPath::CommandPath(const FrontendConfig &cfg)
    : cfg_(cfg), rs_(std::make_unique<ReservationStation>(cfg)), lm_(std::make_unique<LoopMatmul>(cfg)) {
  settle();
}

CommandPath::~CommandPath() = default;

void CommandPath::settle() {
  out_ = Out();
  out_.cmd_ready = raw_q_.enq_ready();
  const LoopMatmul::Out &lm = lm_->out();
  out_.busy = raw_q_.deq_valid() || conv_q_.deq_valid() || lm.busy || rs_->busy() || unrolled_q_.deq_valid() ||
              lm.out_valid;  // Controller.scala:330 without the scratchpad's busy
  out_.loop_matmul_busy = lm.busy;
  for (unsigned q = 0; q < 3; ++q) {
    const ReservationStation::Issue &is = rs_->issue(q);
    IssuePort &p = out_.issue[q];
    p.valid = is.valid;
    p.funct = is.cmd.funct;
    p.rs1 = is.cmd.rs1;
    p.rs2 = is.cmd.rs2;
    p.rob_id = is.rob_id;
    p.cmd = is.cmd;
  }
}

bool CommandPath::idle() const { return !out_.busy; }

std::string CommandPath::unsupported() const { return lm_->unsupported(); }

void CommandPath::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void CommandPath::eval() {
  // unrolled commands into the ReservationStation (Controller.scala:358-401)
  const bool un_valid = unrolled_q_.deq_valid();
  const GemminiCmd &un = unrolled_q_.deq_bits();
  alloc_valid_ = un_deq_ = false;
  if (un_valid) {
    if (bypasses_rs(un.funct)) {
      un_deq_ = true;
    } else {
      alloc_valid_ = true;
      un_deq_ = rs_->alloc_ready(un);
    }
  }
  ReservationStation::In ri;
  ri.alloc_valid = alloc_valid_;
  ri.alloc = un;
  for (unsigned q = 0; q < 3; ++q)
    ri.issue_ready[q] = in_.issue_ready[q];
  ri.completed_valid = in_.completed_valid;
  ri.completed_id = in_.completed_id;
  rs_->set_inputs(ri);
  rs_->eval();
  const ReservationStation::Completed &done = rs_->completed();
  out_.matmul_completed[0] = done.matmul_ld;
  out_.matmul_completed[1] = done.matmul_ex;
  out_.matmul_completed[2] = done.matmul_st;

  // LoopConv's queue: loop_conv_ws commands are dropped, everything else goes to LoopMatmul
  const bool conv_valid = conv_q_.deq_valid();
  const GemminiCmd &conv = conv_q_.deq_bits();
  const bool conv_is_loop = is_loop_conv(conv.funct);
  const bool lm_in_ready = lm_->out().in_ready;
  conv_deq_ = conv_valid && (conv_is_loop || lm_in_ready);
  lm_in_fire_ = conv_valid && !conv_is_loop && lm_in_ready;
  raw_deq_ = raw_q_.deq_valid() && conv_q_.enq_ready();

  LoopMatmul::In li;
  li.in_valid = lm_in_fire_;
  li.in_bits = conv;
  li.out_ready = unrolled_q_.enq_ready();
  li.ld_completed = done.matmul_ld;
  li.st_completed = done.matmul_st;
  li.ex_completed = done.matmul_ex;
  lm_->set_inputs(li);
  lm_->eval();
  evaluated_ = true;
}

void CommandPath::tick() {
  if (!evaluated_)
    eval();
  const int64_t t = cycle_;
  const LoopMatmul::Out &lm = lm_->out();
  GemminiCmd unrolled = lm.out_bits;
  const bool unrolled_fire = lm.out_valid && unrolled_q_.enq_ready();
  const GemminiCmd un = unrolled_q_.deq_bits();
  if (uops_) {
    MicroOpTable &T = *uops_;
    auto with = [&](int64_t id, auto fn) {
      if (T.has(id))
        fn(T.at(id));
    };
    // a command LoopMatmul emits becomes a command micro-op
    if (unrolled_fire && unrolled.uop < 0 && unrolled.from_matmul_fsm) {
      MicroOp u;
      u.kind = kind_of(unrolled.funct);
      u.op = unrolled.op;
      u.parent = unrolled.parent_uop;
      u.funct = unrolled.funct;
      u.rs1 = unrolled.rs1;
      u.rs2 = unrolled.rs2;
      u.sent = t;
      unrolled.uop = T.add(u);
    }
    if (rs_->alloc_fire())
      with(un.uop, [&](MicroOp &u) { first_cycle(u.alloc, t); });
    if (un_deq_ && bypasses_rs(un.funct))
      with(un.uop, [&](MicroOp &u) {  // flush / counter / clock gate: taken here (:358-401)
        first_cycle(u.alloc, t);
        first_cycle(u.completed, t);
      });
    if (lm_->cmd_deq() && is_loop_matmul(lm_->cmd_head().funct))
      with(lm_->cmd_head().uop, [&](MicroOp &u) {  // consumed by LoopMatmul (:901-951)
        first_cycle(u.started, t);
        first_cycle(u.completed, t);
      });
    for (unsigned q = 0; q < 3; ++q)
      if (out_.issue[q].valid && in_.issue_ready[q])
        with(out_.issue[q].cmd.uop, [&](MicroOp &u) {
          first_cycle(u.issued, t);
          u.rob_id = int(out_.issue[q].rob_id);
        });
  }
  rs_->tick();
  lm_->tick();
  unrolled_q_.tick(unrolled_fire, unrolled, un_deq_);
  const GemminiCmd raw = raw_q_.deq_bits();
  conv_q_.tick(raw_deq_, raw, conv_deq_);
  raw_q_.tick(in_.cmd_valid && raw_q_.enq_ready(), in_.cmd, raw_deq_);
  ++cycle_;
  settle();
  evaluated_ = false;
}

}  // namespace systolique
