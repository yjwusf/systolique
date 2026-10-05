// Engine: the front door. Submit operations (a Gemmini command stream, or a whole matmul that it
// turns into gemmini.h's command stream), tick until done, and read the cycle-by-cycle state,
// the micro-op table and the accounting. Docs: docs/micro_ops.md.
//
//   Engine e;                                   // Gemmini's defaultConfig, DmaParams{}
//   int op = e.submit(Matmul{64, 64, 64, kWS});  // tiled_matmul_auto's commands, data in DRAM
//   e.run();                                    // tick() until every operation is done
//   const MicroOpTable &t = e.micro_ops();      // per micro-op cycles, parents, operations
//   Accounting a = e.accounting();              // PE-cycles per state, per micro-op, per op
//   bool ok = e.result(op) == e.reference(op);  // C as the mvouts stored it
//
// What is cycle-exact and what is modelled: the Controller (command path, ExecuteController,
// scratchpad and accumulator banks, the systolic array) replays the Verilated RTL's ports every
// cycle (stored traces: ctest systolique_fe_trace_<top>; live: rtl_fe_*); the Host around it (the core's
// command issue and the load / store side with the DMA and memory) is a model whose parameters
// are not validated (host.h, DmaParams). An Engine run is therefore exact given the Host's
// behaviour; how long mvins and mvouts take is the model's.
//
// Cycles: cycle 0 is the first cycle after the four reset cycles (docs/systolic_array.md);
// tick() is the clock edge that ends cycle(): the Host chooses the cycle's inputs from the
// Controller's register outputs, the Controller evaluates, the Host sees the handshakes, every
// block commits.
#pragma once

#include "systolique/controller.h"
#include "systolique/host.h"
#include "systolique/micro_ops.h"
#include "systolique/vcd.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace systolique {

// A matmul C = act(A B + D), A: M x K, B: K x N, D: M x N (int32, optional), C: M x N int8 (or
// int32 with full_c: the accumulator's raw rows, without the activation), all row-major in DRAM. The Engine issues it the way gemmini.h's
// tiled_matmul_auto does (gemmini-rocc-tests 1a1a1c6, include/gemmini.h:692-848, 1231-1330):
// WS through gemmini_loop_ws (LoopMatmul), OS through sp_tiled_matmul_os's explicit mvin /
// preload / compute / mvout commands.
struct Matmul {
  unsigned M = 64, K = 64, N = 64;
  unsigned dataflow = kWS;  // kOS or kWS
  bool bias = false;
  bool full_c = false;      // C as int32 (the accumulator's full rows)
  unsigned act = ACT_NONE;  // ACT_NONE or ACT_RELU
  uint32_t seed = 1;        // A, B in [-range, range), D in [-1000, 1000]
  int range = 8;
  std::string label;
};

struct EngineOptions {
  FrontendConfig cfg = FrontendConfig::gemmini_default();
  ArrayOptions array;      // the SystolicArray inside (provenance on; its own VCD if asked)
  DmaParams dma;
  bool micro_ops = true;   // record the micro-op table (bookkeeping only)
  // Non-empty: a VCD of the Controller's ports and the micro-ops in flight (off by default; it
  // only reads state: systolique_micro_ops checks that runs with and without it are identical).
  std::string vcd_path;
};

class Engine {
 public:
  explicit Engine(EngineOptions opt = {});
  ~Engine();
  Engine(const Engine &) = delete;
  Engine &operator=(const Engine &) = delete;

  // Operations, sent in submission order. Returns the operation id.
  int submit(const std::vector<Command> &cmds, const std::string &label);
  int submit(const Matmul &m);
  // The command stream the Engine issues for a matmul (data addresses as submit() lays them out).
  static std::vector<Command> matmul_commands(const Matmul &m, const FrontendConfig &cfg, uint64_t a,
                                              uint64_t b, uint64_t d, uint64_t c);

  void tick();  // step(host_inputs())
  // A cycle in two halves, for benches that drive another copy of the Controller (the RTL) with
  // the same inputs: the Host's inputs of this cycle (from the Controller's register outputs),
  // then the Controller's evaluation, the Host's observation and the edge. step() returns the
  // Controller's outputs of the cycle.
  CtrlTopIn host_inputs() const;
  CtrlTopOut step(const CtrlTopIn &in);
  bool done() const;
  // tick() until done(); returns the cycles of the run (cycle()). Throws std::runtime_error after
  // max_cycles or when the Host reports an unsupported command.
  int64_t run(int64_t max_cycles = 20000000);
  int64_t cycle() const { return ctrl_->cycle(); }

  // Called in every cycle after the Controller has evaluated and before the edge.
  using Hook = std::function<void(const Engine &, const CtrlTopIn &, const CtrlTopOut &)>;
  void set_hook(Hook h) { hook_ = std::move(h); }

  const Controller &controller() const { return *ctrl_; }
  const SystolicArray &array() const { return ctrl_->execute_unit().ex().array(); }
  const MicroOpTable &micro_ops() const { return uops_; }
  const Host &host() const { return *host_; }
  // The array's accounting; per_uop / per_op attribute every occupied PE-cycle.
  Accounting accounting() const { return array().accounting(); }
  // Copies what the array knows (first_in .. last_out) into the micro-ops and closes every
  // operation's window (Operation::end). run() calls it.
  void finalize();
  // Conservation of the micro-op attribution: per-operation and per-micro-op PE-cycles sum to
  // the totals, every MAC PE-cycle belongs to a compute micro-op, every matmul's MAC PE-cycles
  // equal its MACs. "" if all hold.
  std::string check() const;

  // A matmul's C as stored in DRAM, and C computed by plain C++ from the same A, B, D.
  std::vector<int32_t> result(int op) const;
  std::vector<int32_t> reference(int op) const;
  const Dram &dram() const { return dram_; }

 private:
  struct MatmulData {
    int op = -1;
    Matmul m;
    uint64_t a = 0, b = 0, d = 0, c = 0;
  };
  void declare_vcd();
  void sample_vcd(const CtrlTopIn &in, const CtrlTopOut &o);

  EngineOptions opt_;
  Dram dram_;
  MicroOpTable uops_;
  std::unique_ptr<Controller> ctrl_;
  std::unique_ptr<Host> host_;
  std::vector<MatmulData> matmuls_;
  uint64_t next_base_ = 0x10000000ull;
  Hook hook_;
  struct Vcd;
  std::unique_ptr<Vcd> vcd_;
  bool finalized_ = false;
};

}  // namespace systolique
