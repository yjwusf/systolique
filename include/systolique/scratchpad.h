// The scratchpad and accumulator banks as Gemmini's ExecuteController sees them (Gemmini v0.7.2):
//
//   ScratchpadBank   ScratchpadBank (Scratchpad.scala:97-169): a single-ported SyncReadMem whose
//                    read request is taken only when no write uses the bank, a 1-entry
//                    pipe+flow response queue, and the ExecuteController's response Pipeline of
//                    spad_read_delay stages (Scratchpad.scala:489-497).
//   Scratchpad       sp_banks ScratchpadBanks; per bank the ExecuteController's write has
//                    priority over the DMA's (Scratchpad.scala:513-555).
//   AccumulatorBank  AccumulatorMem (AccumulatorMem.scala:92-343), two-ported (acc_singleported
//                    false): the acc_latency-stage write pipeline, read-modify-write through the
//                    shared adder, the 1-entry response queue.
//   Accumulator      acc_banks AccumulatorBanks and AccPipeShared (AccumulatorMem.scala:74-90),
//                    the adder the banks share; the ExecuteController's write has priority over
//                    the DMA's (Scratchpad.scala:718-823).
//
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Scope: the timing-relevant behaviour plus the data. The DMA side is a row-write port per
// memory (DmaSpWrite / DmaAccWrite), taken in the DMA's priority slot, exactly as the test ports
// of the RTL top rtl/ex/src/ExecuteTop.scala; mvout reads, the normalizer and AccumulatorScale
// are not modelled (the accumulator's read responses have no consumer, as in ExecuteTop, so the
// ExecuteController cannot take operands from the accumulator: docs/micro_ops.md).
//
// Clock (two-phase, docs/systolic_array.md): resp() depends on registers only; read_ready() and
// ready_view() are const functions of the registers and this cycle's writes and response pops
// (the RTL's combinational paths from the write port and resp.ready to read.req.ready,
// Scratchpad.scala:166, AccumulatorMem.scala:326-332); set_inputs() gives the cycle's inputs,
// eval() computes every next register value without changing one, tick() commits them (it runs
// eval() first if needed). The ExecuteController's reads depend on read_ready(), so the owner
// evaluates in this order: resp(), the ExecuteController's write-back, read_ready() /
// ready_view(), the ExecuteController's reads, set_inputs(), tick().
#pragma once

#include "systolique/fe_util.h"
#include "systolique/frontend_config.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace systolique {

constexpr unsigned kFeDim = 16;  // DIM of the configuration the frontend classes support
using SpRow = std::array<int8_t, kFeDim>;    // a scratchpad row: DIM inputType elements
using AccRow = std::array<int32_t, kFeDim>;  // an accumulator row: DIM accType elements

// ScratchpadReadIO / ScratchpadWriteIO (Scratchpad.scala:75-95), the ExecuteController's side.
struct SpReadReq {
  bool valid = false;
  uint32_t addr = 0;  // bank row
};
struct SpReadResp {
  bool valid = false;
  SpRow data{};
};
struct SpWrite {
  bool en = false;
  uint32_t addr = 0;
  SpRow data{};
  uint64_t mask = 0;  // one bit per byte (aligned_to = 1)
};
// AccumulatorReadReq / AccumulatorWriteReq (AccumulatorMem.scala:8-44).
struct AccReadReq {
  bool valid = false;
  uint32_t addr = 0;
  uint32_t scale = 0;  // acc_scale_t bits, carried into the response
  unsigned act = 0;
};
struct AccWrite {
  bool valid = false;
  uint32_t addr = 0;
  AccRow data{};
  bool acc = false;   // accumulate: read-modify-write through the adder
  uint64_t mask = 0;  // one bit per byte (DIM elements x 4 bytes)
};
// The DMA's row writes (mvin): the test ports of rtl/ex/src/ExecuteTop.scala.
struct DmaSpWrite {
  bool valid = false;
  unsigned bank = 0;
  uint32_t addr = 0;
  SpRow data{};
  uint64_t mask = 0;
};
struct DmaAccWrite {
  bool valid = false;
  unsigned bank = 0;
  uint32_t addr = 0;
  AccRow data{};
  uint64_t mask = 0;
  bool acc = false;
};

class ScratchpadBank {
 public:
  struct In {
    bool write_en = false;
    SpWrite write;          // the write the bank takes (the Scratchpad's mux)
    SpReadReq req;          // the ExecuteController's read request
    bool resp_ready = false;  // the ExecuteController pops the response pipeline's head
  };
  ScratchpadBank(unsigned entries, unsigned read_delay);
  ~ScratchpadBank();
  ScratchpadBank(const ScratchpadBank &) = delete;
  ScratchpadBank &operator=(const ScratchpadBank &) = delete;

  // The response at the ExecuteController (the Pipeline's output), registers only.
  SpReadResp resp() const;
  // read.req.ready for this cycle's write enable and response pop (Scratchpad.scala:166).
  bool read_ready(bool write_en, bool resp_ready) const;
  void set_inputs(const In &in);
  void eval();
  void tick();
  bool req_ready() const { return next_.req_ready; }  // after eval()
  const SpRow &row(unsigned r) const { return mem_[r]; }
  void poke(unsigned r, const SpRow &v) { mem_[r] = v; }  // functional (tests, preloads)
  bool idle() const;

 private:
  struct Comb {
    bool q_enq_valid = false, resp_valid = false, resp_fire = false, enq_fire = false;
    bool pipe_in_ready = false, req_ready = false, req_fire = false;
    SpReadResp q_enq_bits, resp;
  };
  Comb comb(bool write_en, bool resp_ready) const;
  std::vector<SpRow> mem_;
  bool ren_reg_ = false;      // RegNext(read.req.fire)
  uint32_t raddr_reg_ = 0;    // the SyncReadMem's registered read address
  Queue<SpReadResp> q_{1, true, true};
  Pipeline<SpReadResp> pipe_;
  In in_;
  Comb next_;
  bool evaluated_ = false;
};

class Scratchpad {
 public:
  struct In {
    std::vector<SpReadReq> read;      // per bank: the ExecuteController's read request
    std::vector<bool> resp_ready;     // per bank: response pops
    std::vector<SpWrite> write;       // per bank: the ExecuteController's write
    DmaSpWrite dma;                   // the DMA's row write
  };
  explicit Scratchpad(const FrontendConfig &cfg);
  ~Scratchpad();
  Scratchpad(const Scratchpad &) = delete;
  Scratchpad &operator=(const Scratchpad &) = delete;

  unsigned banks() const { return unsigned(banks_.size()); }
  std::vector<SpReadResp> resp() const;  // registers only
  // Per bank read.req.ready for this cycle's writes and pops.
  std::vector<bool> read_ready(const std::vector<SpWrite> &ex_write, const std::vector<bool> &resp_ready,
                               const DmaSpWrite &dma) const;
  // The DMA's write is taken (its bank has no ExecuteController write this cycle).
  static bool dma_taken(const std::vector<SpWrite> &ex_write, const DmaSpWrite &dma);
  void set_inputs(const In &in);
  void eval();
  void tick();
  bool idle() const;
  const ScratchpadBank &bank(unsigned b) const { return *banks_[b]; }
  const SpRow &row(unsigned bank, unsigned r) const { return banks_[bank]->row(r); }
  void poke(unsigned bank, unsigned r, const SpRow &v) { banks_[bank]->poke(r, v); }

 private:
  std::vector<std::unique_ptr<ScratchpadBank>> banks_;
  In in_;
  bool evaluated_ = false;
};

class AccumulatorBank {
 public:
  struct In {
    bool write_valid = false;
    AccWrite write;    // the write offered to the bank (the Accumulator's mux)
    AccReadReq req;    // the ExecuteController's read request
  };
  // What read.req.ready depends on (AccumulatorMem.scala:326-332): the response queue will be
  // empty, no accumulate write this cycle, and the read address is not in the write pipeline.
  struct ReadyView {
    bool will_be_empty = true;
    bool write_acc = false;
    std::array<bool, 2> pw_valid{};
    std::array<uint32_t, 2> pw_addr{};
    bool ready(uint32_t addr) const {
      if (!will_be_empty || write_acc)
        return false;
      for (unsigned k = 0; k < 2; ++k)
        if (pw_valid[k] && pw_addr[k] == addr)
          return false;
      return true;
    }
  };
  explicit AccumulatorBank(unsigned entries);
  ~AccumulatorBank();
  AccumulatorBank(const AccumulatorBank &) = delete;
  AccumulatorBank &operator=(const AccumulatorBank &) = delete;

  ReadyView ready_view(bool write_valid, const AccWrite &write) const;
  // io.write.ready for a write (AccumulatorMem.scala:334-335; block_write_req is false when
  // two-ported).
  bool write_ready(const AccWrite &write) const;
  void set_inputs(const In &in);
  void eval();
  // The edge. `adder_sum`: AccPipeShared's output this cycle (Accumulator).
  void tick(const AccRow &adder_sum);
  bool write_fire() const { return next_.write_fire; }  // after eval()
  bool req_fire() const { return next_.req_fire; }
  // The adder's inputs of this cycle (io.adder: valid, op1 = the read data, op2 = stage 0's data)
  bool adder_valid() const { return pw_[0].valid && pw_[0].acc; }
  const AccRow &adder_op1() const { return mem_[raddr_reg_]; }
  const AccRow &adder_op2() const { return pw_[0].data; }
  const AccRow &row(unsigned r) const { return mem_[r]; }
  void poke(unsigned r, const AccRow &v) { mem_[r] = v; }
  bool idle() const;

 private:
  struct Stage {  // pipelined_writes(i) (AccumulatorMem.scala:110-117)
    bool valid = false;
    uint32_t addr = 0;
    AccRow data{};
    bool acc = false;
    uint64_t mask = 0;
  };
  struct Meta {  // RegNext(read.req.bits.*) (AccumulatorMem.scala:316-322)
    uint32_t scale = 0;
    unsigned act = 0;
  };
  struct RespEntry {
    AccRow data{};
    Meta meta;
  };
  struct Comb {
    bool write_fire = false, req_fire = false, q_enq_valid = false;
    RespEntry q_enq_bits;
  };
  Comb comb() const;
  std::vector<AccRow> mem_;
  std::array<Stage, 2> pw_;
  bool rd_reg_ = false;       // RegNext(read.req.fire)
  uint32_t raddr_reg_ = 0;    // TwoPortSyncMem's registered read address
  Meta meta_reg_;
  Queue<RespEntry> q_{1, true, true};
  In in_;
  Comb next_;
  bool evaluated_ = false;
};

class Accumulator {
 public:
  struct In {
    std::vector<AccReadReq> read;   // per bank
    std::vector<AccWrite> write;    // per bank: the ExecuteController's write
    DmaAccWrite dma;
  };
  explicit Accumulator(const FrontendConfig &cfg);
  ~Accumulator();
  Accumulator(const Accumulator &) = delete;
  Accumulator &operator=(const Accumulator &) = delete;

  unsigned banks() const { return unsigned(banks_.size()); }
  // Per bank, what read.req.ready depends on, for this cycle's writes.
  std::vector<AccumulatorBank::ReadyView> ready_view(const std::vector<AccWrite> &ex_write,
                                                     const DmaAccWrite &dma) const;
  // The DMA's write is taken: no ExecuteController write to its bank and the bank takes it.
  bool dma_taken(const std::vector<AccWrite> &ex_write, const DmaAccWrite &dma) const;
  void set_inputs(const In &in);
  void eval();
  void tick();
  bool idle() const;
  // An ExecuteController write the bank did not take (Scratchpad.scala:724 asserts against it;
  // ExecuteTop leaves the assertion out): counted, never expected.
  uint64_t ex_writes_dropped() const { return dropped_; }
  const AccumulatorBank &bank(unsigned b) const { return *banks_[b]; }
  const AccRow &row(unsigned bank, unsigned r) const { return banks_[bank]->row(r); }
  void poke(unsigned bank, unsigned r, const AccRow &v) { banks_[bank]->poke(r, v); }

 private:
  void bank_inputs(const In &in, unsigned b, AccumulatorBank::In &bi) const;
  std::vector<std::unique_ptr<AccumulatorBank>> banks_;
  AccRow adder_sum_{};  // AccPipeShared: ShiftRegister(op1 + op2, acc_latency - 1 = 1)
  In in_;
  bool evaluated_ = false;
  uint64_t dropped_ = 0;
};

}  // namespace systolique
