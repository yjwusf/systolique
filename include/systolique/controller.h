// The frontend blocks wired together, with the ports of the RTL tops of rtl/ex/src/ (Gemmini
// v0.7.2; derived from Gemmini, BSD-3-Clause, see LICENSE.gemmini and NOTICE):
//
//   ExecuteUnit  ExecuteController + Scratchpad + Accumulator, wired as Scratchpad.scala:446-823
//                and Controller.scala:271-302 wire them; ports of ExecuteTop.scala: the
//                ReservationStation's execute issue in, the completion out, the DMA's row writes
//                in, the ExecuteController's bank ports observed.
//   Controller   CommandPath + ExecuteUnit + the completion Arbiter (execute, then load, then
//                store, Controller.scala:312-327); ports of CtrlTop.scala: the core's commands
//                in, the load / store controllers' issue and completion ports, the DMA's row
//                writes.
//
// Clock (two-phase): out_regs() is what depends on registers only (the ready bits and issue
// ports a driver reads before choosing its inputs); set_inputs() gives the cycle's inputs;
// eval() settles every combinational path in the RTL's order (scratchpad responses, the
// ExecuteController's write-back, the banks' readiness, the operand reads, the issue and
// completion handshakes) and computes every next register value without changing one, after
// which out() holds all of the cycle's outputs; tick() commits every block.
#pragma once

#include "systolique/command_path.h"
#include "systolique/execute_controller.h"
#include "systolique/micro_ops.h"
#include "systolique/scratchpad.h"

#include <memory>

namespace systolique {

// Every port of ExecuteTop for one cycle.
struct ExecuteTopIn {
  bool cmd_valid = false;
  unsigned cmd_funct = 0, cmd_rob_id = 0;
  uint64_t cmd_rs1 = 0, cmd_rs2 = 0;
  GemminiCmd cmd_bookkeeping;  // op / uop of the command (not a port)
  DmaSpWrite dma_sp;
  DmaAccWrite dma_acc;
};
struct ExecuteTopOut {
  bool cmd_ready = false, completed_valid = false, busy = false;
  unsigned completed_bits = 0;
  bool dma_sp_taken = false, dma_acc_taken = false;
  std::vector<SpReadReq> sp_read;
  std::vector<bool> sp_read_ready, sp_resp_ready;
  std::vector<SpWrite> sp_write;
  std::vector<AccReadReq> acc_read;
  std::vector<bool> acc_read_ready;
  std::vector<AccWrite> acc_write;
};

class ExecuteUnit {
 public:
  explicit ExecuteUnit(const FrontendConfig &cfg, ArrayOptions opt = {});
  ~ExecuteUnit();
  ExecuteUnit(const ExecuteUnit &) = delete;
  ExecuteUnit &operator=(const ExecuteUnit &) = delete;

  void set_micro_ops(MicroOpTable *t) { ec_->set_micro_ops(t); }
  bool cmd_ready() const { return ec_->cmd_ready(); }  // registers only
  void set_inputs(const ExecuteTopIn &in);
  void eval();
  void tick();
  const ExecuteTopOut &out() const { return out_; }  // after eval()
  int64_t cycle() const { return ec_->cycle(); }
  bool idle() const;

  const ExecuteController &ex() const { return *ec_; }
  const Scratchpad &scratchpad() const { return *sp_; }
  const Accumulator &accumulator() const { return *acc_; }
  Scratchpad &scratchpad() { return *sp_; }
  Accumulator &accumulator() { return *acc_; }

 private:
  FrontendConfig cfg_;
  std::unique_ptr<ExecuteController> ec_;
  std::unique_ptr<Scratchpad> sp_;
  std::unique_ptr<Accumulator> acc_;
  ExecuteTopIn in_;
  ExecuteTopOut out_;
  bool evaluated_ = false;
};

// Every port of CtrlTop for one cycle.
struct CtrlTopIn {
  bool cmd_valid = false;  // the core
  GemminiCmd cmd;          // funct, rs1, rs2 (+ op / uop bookkeeping)
  bool ld_ready = false, st_ready = false;
  bool ld_completed_valid = false, st_completed_valid = false;
  unsigned ld_completed_bits = 0, st_completed_bits = 0;
  DmaSpWrite dma_sp;
  DmaAccWrite dma_acc;
};
struct CtrlTopOut {
  bool cmd_ready = false, busy = false, loop_matmul_busy = false;
  CommandPath::IssuePort ld, st, ex;
  bool ex_ready = false;
  bool ld_completed_ready = false, st_completed_ready = false;
  bool ex_completed_valid = false;
  unsigned ex_completed_bits = 0;
  bool ex_busy = false;
  ExecuteTopOut bank;  // the ExecuteController's bank ports and the DMA's taken bits
};

class Controller {
 public:
  explicit Controller(const FrontendConfig &cfg, ArrayOptions opt = {});
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;

  void set_micro_ops(MicroOpTable *t);
  // The outputs that depend on registers only: cmd_ready, busy, loop_matmul_busy, the ld / st /
  // ex issue ports, ex_ready.
  CtrlTopOut out_regs() const;
  void set_inputs(const CtrlTopIn &in);
  void eval();
  void tick();
  const CtrlTopOut &out() const { return out_; }  // after eval()
  int64_t cycle() const { return ex_->cycle(); }
  bool idle() const;  // nothing queued, issued, in flight or in the array

  const CommandPath &command_path() const { return *cp_; }
  const ExecuteUnit &execute_unit() const { return *ex_; }
  ExecuteUnit &execute_unit() { return *ex_; }

 private:
  FrontendConfig cfg_;
  std::unique_ptr<CommandPath> cp_;
  std::unique_ptr<ExecuteUnit> ex_;
  CtrlTopIn in_;
  CtrlTopOut out_;
  bool evaluated_ = false;
};

}  // namespace systolique
