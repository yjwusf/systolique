// ExecuteUnit and Controller: the frontend blocks wired as rtl/ex/src/ExecuteTop.scala and
// CtrlTop.scala wire Gemmini v0.7.2's modules. Derived from Gemmini (BSD-3-Clause, see
// LICENSE.gemmini and NOTICE).
#include "systolique/controller.h"

#include <stdexcept>

namespace systolique {

// ------------------------------------------------------------------ ExecuteUnit
ExecuteUnit::ExecuteUnit(const FrontendConfig &cfg, ArrayOptions opt)
    : cfg_(cfg), ec_(std::make_unique<ExecuteController>(cfg, opt)), sp_(std::make_unique<Scratchpad>(cfg)),
      acc_(std::make_unique<Accumulator>(cfg)) {}

ExecuteUnit::~ExecuteUnit() = default;

bool ExecuteUnit::idle() const { return ec_->idle() && sp_->idle() && acc_->idle(); }

void ExecuteUnit::set_inputs(const ExecuteTopIn &in) {
  in_ = in;
  evaluated_ = false;
}

void ExecuteUnit::eval() {
  // 1. registers only: the scratchpad's responses; from them the write-back and response pops
  const std::vector<SpReadResp> sp_resp = sp_->resp();
  const ExecuteController::Writeback wb = ec_->writeback(sp_resp);
  // 2. the banks' readiness for those writes and pops (Scratchpad.scala:166, AccumulatorMem.scala:326)
  ExecuteController::In ei;
  ei.cmd_valid = in_.cmd_valid;
  ei.cmd = in_.cmd_bookkeeping;
  ei.cmd.funct = in_.cmd_funct;
  ei.cmd.rs1 = in_.cmd_rs1;
  ei.cmd.rs2 = in_.cmd_rs2;
  ei.cmd.rob_valid = true;  // rob_id.valid := true.B (ExecuteTop.scala)
  ei.cmd.rob_id = in_.cmd_rob_id;
  ei.sp_resp = sp_resp;
  ei.sp_req_ready = sp_->read_ready(wb.sp_write, wb.sp_resp_ready, in_.dma_sp);
  ei.acc_ready = acc_->ready_view(wb.acc_write, in_.dma_acc);
  // 3. the ExecuteController with everything
  ec_->set_inputs(ei);
  ec_->eval();
  const ExecuteController::Out &eo = ec_->out();
  // 4. the banks with the ExecuteController's reads, writes and pops
  Scratchpad::In si;
  si.read = eo.sp_read;
  si.resp_ready = eo.sp_resp_ready;
  si.write = eo.sp_write;
  si.dma = in_.dma_sp;
  sp_->set_inputs(si);
  sp_->eval();
  Accumulator::In ai;
  ai.read = eo.acc_read;
  ai.write = eo.acc_write;
  ai.dma = in_.dma_acc;
  acc_->set_inputs(ai);
  acc_->eval();

  ExecuteTopOut &o = out_;
  o.cmd_ready = eo.cmd_ready;
  o.completed_valid = eo.completed_valid;
  o.completed_bits = eo.completed_id;
  o.busy = eo.busy;
  o.dma_sp_taken = Scratchpad::dma_taken(eo.sp_write, in_.dma_sp);
  o.dma_acc_taken = acc_->dma_taken(eo.acc_write, in_.dma_acc);
  o.sp_read = eo.sp_read;
  o.sp_read_ready = ei.sp_req_ready;
  o.sp_resp_ready = eo.sp_resp_ready;
  o.sp_write = eo.sp_write;
  o.acc_read = eo.acc_read;
  o.acc_read_ready = eo.acc_read_ready;
  o.acc_write = eo.acc_write;
  evaluated_ = true;
}

void ExecuteUnit::tick() {
  if (!evaluated_)
    eval();
  ec_->tick();
  sp_->tick();
  acc_->tick();
  evaluated_ = false;
}

// ------------------------------------------------------------------ Controller
Controller::Controller(const FrontendConfig &cfg, ArrayOptions opt)
    : cfg_(cfg), cp_(std::make_unique<CommandPath>(cfg)), ex_(std::make_unique<ExecuteUnit>(cfg, opt)) {}

Controller::~Controller() = default;

void Controller::set_micro_ops(MicroOpTable *t) {
  cp_->set_micro_ops(t);
  ex_->set_micro_ops(t);
}

bool Controller::idle() const { return cp_->idle() && ex_->idle(); }

CtrlTopOut Controller::out_regs() const {
  CtrlTopOut o;
  const CommandPath::Out &c = cp_->out();
  o.cmd_ready = c.cmd_ready;
  o.busy = c.busy;
  o.loop_matmul_busy = c.loop_matmul_busy;
  o.ld = c.issue[ReservationStation::LDQ];
  o.ex = c.issue[ReservationStation::EXQ];
  o.st = c.issue[ReservationStation::STQ];
  o.ex_ready = ex_->cmd_ready();
  return o;
}

void Controller::set_inputs(const CtrlTopIn &in) {
  in_ = in;
  evaluated_ = false;
}

void Controller::eval() {
  CtrlTopOut o = out_regs();
  // the execute issue (Controller.scala:243-246)
  ExecuteTopIn ei;
  ei.cmd_valid = o.ex.valid;
  ei.cmd_funct = o.ex.funct;
  ei.cmd_rs1 = o.ex.rs1;
  ei.cmd_rs2 = o.ex.rs2;
  ei.cmd_rob_id = o.ex.rob_id;
  ei.cmd_bookkeeping = o.ex.cmd;
  ei.dma_sp = in_.dma_sp;
  ei.dma_acc = in_.dma_acc;
  ex_->set_inputs(ei);
  ex_->eval();
  const ExecuteTopOut &eo = ex_->out();
  o.bank = eo;
  o.ex_completed_valid = eo.completed_valid;
  o.ex_completed_bits = eo.completed_bits;
  o.ex_busy = eo.busy;
  // the completion Arbiter (Controller.scala:312-327): execute, load, store
  CommandPath::In ci;
  ci.cmd_valid = in_.cmd_valid;
  ci.cmd = in_.cmd;
  ci.issue_ready[ReservationStation::LDQ] = in_.ld_ready;
  ci.issue_ready[ReservationStation::EXQ] = o.ex_ready;
  ci.issue_ready[ReservationStation::STQ] = in_.st_ready;
  o.ld_completed_ready = !eo.completed_valid;
  o.st_completed_ready = !eo.completed_valid && !in_.ld_completed_valid;
  if (eo.completed_valid) {
    ci.completed_valid = true;
    ci.completed_id = eo.completed_bits;
  } else if (in_.ld_completed_valid) {
    ci.completed_valid = true;
    ci.completed_id = in_.ld_completed_bits;
  } else if (in_.st_completed_valid) {
    ci.completed_valid = true;
    ci.completed_id = in_.st_completed_bits;
  }
  cp_->set_inputs(ci);
  cp_->eval();
  out_ = o;
  evaluated_ = true;
}

void Controller::tick() {
  if (!evaluated_)
    eval();
  ex_->tick();
  cp_->tick();
  evaluated_ = false;
}

}  // namespace systolique
