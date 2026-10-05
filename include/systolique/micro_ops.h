// Micro-ops: what Gemmini executes for an operation, cycle by cycle, and where every array
// PE-cycle and every scratchpad / accumulator access goes. Docs: docs/micro_ops.md.
//
// An Operation is what a user submits (a Gemmini command stream, or a whole matmul that the
// Engine turns into gemmini.h's command stream). Its micro-ops:
//
//   commands   one per Gemmini command, as the controllers execute it: Config, Preload,
//              ComputePreloaded, ComputeAccumulated (ExecuteController), Mvin, Mvout (the
//              modelled DMA), LoopCmd (loop_ws and its configuration commands, consumed by
//              LoopMatmul), Fence (the core's), Flush (the array flush the ExecuteController
//              inserts by itself, ExecuteController.scala:620-627, 682-693)
//   Request    one MeshWithDelays request (an array pass): a preload alone, a compute
//              overlapped with the next preload, a compute alone, or a flush
//              (ExecuteController.scala:532-693)
//   rows       OperandRead (one A / B / D row read from a scratchpad bank), ResultRow (one row
//              leaving the array with a tag, and its write-back to the scratchpad or the
//              accumulator), DmaRow (one row an mvin writes into a bank / an mvout reads)
//
// Every micro-op has a unique id (its index in MicroOpTable), the id of its operation and of its
// parent micro-op. Cycles follow docs/systolic_array.md: cycle 0 is the first cycle with reset
// low; an event "in cycle t" is a handshake (valid and ready both 1) in t, its effect is in the
// registers from t + 1. -1 = has not happened.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace systolique {

enum class UopKind : uint8_t {
  Config,
  Preload,
  ComputePreloaded,
  ComputeAccumulated,
  Mvin,
  Mvout,
  Flush,
  LoopCmd,
  Fence,
  OtherCmd,
  Request,
  OperandRead,
  ResultRow,
  DmaRow,
};
const char *to_string(UopKind k);
bool is_command(UopKind k);

// The pass a Request micro-op makes (ExecuteController.scala:281-283).
enum class PassKind : uint8_t { SinglePreload = 0, MulPre = 1, SingleMul = 2, Flush = 3 };
const char *to_string(PassKind k);

struct MicroOp {
  int64_t id = -1;
  int op = -1;           // the operation
  int64_t parent = -1;   // rows: their command; Request: its compute command (else its preload);
                         // commands from LoopMatmul: the loop_ws command; else -1
  UopKind kind = UopKind::OtherCmd;

  // ---- commands
  unsigned funct = 0;
  uint64_t rs1 = 0, rs2 = 0;
  int rob_id = -1;        // the ReservationStation entry it was issued from
  int request = -1;       // the array request that executes it (SystolicArray::requests())
  int64_t pass = -1;      // its Request micro-op
  // Cycles:
  //   sent       the core's command entered raw_cmd_q (io.cmd.fire)
  //   alloc      it entered the ReservationStation (alloc.fire)
  //   issued     the ReservationStation issued it to its controller (issue.fire)
  //   started    first cycle of the ExecuteController pass that executes it (the FSM decides
  //              to perform it); DMA: the first row moved
  //   popped     it left the ExecuteController's command queue
  //   first_read / last_read   first / last of its operand rows read from a bank (read.req.fire)
  //   accept     its request accepted by MeshWithDelays (req.fire)
  //   first_in / last_in       its request's first / last row at the Mesh input (lane 0)
  //   first_out / last_out     its request's first / last row leaving the array (resp.valid)
  //   result_first / result_last  rows carrying its tag (the preload's: its C)
  //   wb_first / wb_last       those rows written back (scratchpad / accumulator write fire)
  //   wb_done    the last write-back is in the memory (scratchpad: wb_last; accumulator: two
  //              cycles later, AccumulatorMem.scala:110-125)
  //   completed  completion reported to the ReservationStation (io.completed)
  int64_t sent = -1, alloc = -1, issued = -1, started = -1, popped = -1;
  int64_t first_read = -1, last_read = -1, accept = -1, first_in = -1, last_in = -1;
  int64_t first_out = -1, last_out = -1, result_first = -1, result_last = -1;
  int64_t wb_first = -1, wb_last = -1, wb_done = -1, completed = -1;
  unsigned reads = 0, rows_in = 0, result_rows = 0, wb_rows = 0;

  // ---- Request
  PassKind pass_kind = PassKind::Flush;
  int64_t compute_uop = -1, preload_uop = -1;
  unsigned total_rows = 0;

  // ---- rows: the event cycle and when its effect is in memory
  int64_t cycle = -1, done = -1;
  char operand = 0;       // OperandRead: 'A', 'B', 'D'; DmaRow: 'I' (mvin) / 'O' (mvout)
  unsigned row = 0;       // row of the operand / result / transfer
  unsigned bank = 0;
  uint32_t addr = 0;      // bank row
  bool to_acc = false;    // the accumulator (else the scratchpad)
  bool written = false;   // ResultRow: written back (false: garbage address or padding row)
  bool accumulate = false;
};

struct Operation {
  int id = -1;
  std::string label;
  bool is_matmul = false;
  uint64_t workload_macs = 0;  // M*N*K of a matmul (else 0)
  unsigned commands = 0;       // commands the core sends for it (fences included)
  // first / last command the core sent (fences and LoopMatmul's commands not counted), last cycle any of its micro-ops did
  // something (a fence: the cycle it found Gemmini idle)
  int64_t first_sent = -1, last_sent = -1, end = -1;
};

// The micro-op table of a run. Owned by the Engine (or a bench); the classes that execute
// micro-ops get a non-owning pointer and only append and update records: recording never
// steers a decision (systolique_micro_ops checks runs with and without it).
class MicroOpTable {
 public:
  MicroOpTable() = default;
  ~MicroOpTable() = default;
  int64_t add(MicroOp u) {
    u.id = int64_t(uops_.size());
    uops_.push_back(u);
    return u.id;
  }
  MicroOp &at(int64_t id) { return uops_.at(size_t(id)); }
  const MicroOp &at(int64_t id) const { return uops_.at(size_t(id)); }
  bool has(int64_t id) const { return id >= 0 && size_t(id) < uops_.size(); }
  const std::vector<MicroOp> &all() const { return uops_; }
  std::vector<MicroOp> &all_mut() { return uops_; }
  size_t size() const { return uops_.size(); }
  int add_op(Operation o) {
    o.id = int(ops_.size());
    ops_.push_back(o);
    return o.id;
  }
  Operation &op(int id) { return ops_.at(size_t(id)); }
  const std::vector<Operation> &ops() const { return ops_; }
  std::vector<Operation> &ops_mut() { return ops_; }
  void clear() {
    uops_.clear();
    ops_.clear();
  }

 private:
  std::vector<MicroOp> uops_;
  std::vector<Operation> ops_;
};

// Updates a cycle field to the earliest / latest event.
inline void first_cycle(int64_t &f, int64_t t) {
  if (f < 0 || t < f)
    f = t;
}
inline void last_cycle(int64_t &f, int64_t t) {
  if (t > f)
    f = t;
}

}  // namespace systolique
