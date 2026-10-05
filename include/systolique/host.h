// What lies around Gemmini's Controller in the Engine: the core that sends the commands, and the
// load / store side (LoadController, StoreController, the DMA and memory) as a MODELLED
// latency / bandwidth port. Nothing here is checked against the RTL: the classes it drives
// (Controller) are; this is their environment. Docs: docs/micro_ops.md, "Data movement".
//
//   Dram     main memory: a sparse byte array.
//   Host     the core: sends the commands of the operations in order, one per cycle when
//            io.cmd.ready (Controller.scala:131-136); a fence waits until Gemmini is not busy
//            (the RoCC busy, Controller.scala:330, plus this model's DMA). The load / store
//            controllers: take the ReservationStation's ld / st issues, move the rows, report
//            the completions through the completion Arbiter.
//
// DMA model (DmaParams; status: unvalidated, not fitted on any data):
//   * one load channel and one store channel, each serving its commands in issue order and
//     holding at most `max_cmds` (accepted, not completed); config_ld / config_st are applied
//     when accepted (the ReservationStation completes them on issue, ReservationStation.scala:343)
//   * an mvin of R rows of `cols` elements: DRAM row r of the command arrives at
//     accept + latency + ceil((r + 1) * row_bytes / bytes_per_cycle), row_bytes = cols * element
//     size; each arrived DRAM row is split into its blocks of DIM elements and every block row
//     is written into its bank through the DMA's row-write port (DmaSpWrite / DmaAccWrite: one
//     scratchpad and one accumulator row per cycle, in the DMA's lower-priority slot, taken when
//     the ExecuteController does not write that bank: these bank effects are the RTL's); the mvin
//     completes in the cycle after its last row was taken
//   * an mvout completes at accept + latency + ceil(rows * row_bytes / bytes_per_cycle); its
//     rows are read from the banks (functionally, no bank port) and written to DRAM then; the
//     ReservationStation's dependencies keep it behind the commands that write its rows
//   * scales: identity only (mvin scale and acc scale 1.0); activation of mvout from the
//     accumulator: NONE or RELU (config_st)
//
// Clock: the Host is the Controller's environment, so its cycle has the Controller's two halves:
// inputs() is const (it chooses the cycle's inputs from its state and the Controller's register
// outputs), observe() is its edge (it takes the cycle's handshakes, seen in the Controller's
// evaluated outputs, and advances its state).
#pragma once

#include "systolique/controller.h"
#include "systolique/isa.h"
#include "systolique/micro_ops.h"

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace systolique {

class Dram {
 public:
  Dram() = default;
  ~Dram() = default;
  uint8_t read8(uint64_t addr) const;
  void write8(uint64_t addr, uint8_t v);
  void read(uint64_t addr, uint8_t *dst, size_t n) const;
  void write(uint64_t addr, const uint8_t *src, size_t n);

 private:
  static constexpr unsigned kPageBits = 12;
  std::unordered_map<uint64_t, std::vector<uint8_t>> pages_;
};

struct DmaParams {
  unsigned latency = 40;          // cycles from accepting an mvin / mvout to its first bytes (unvalidated)
  unsigned bytes_per_cycle = 16;  // the bus: dma_buswidth 128 bits (Configs.scala:66); one beat per
                                  // cycle is an assumption (unvalidated)
  unsigned max_cmds = 2;          // mvin / mvout commands each channel holds (unvalidated)
};

// A command the core sends, with its bookkeeping and the cycles to wait after the previous one.
struct HostCmd {
  Command cmd;
  int op = -1;
  int64_t uop = -1;
  unsigned gap = 0;
};

class Host {
 public:
  Host(const FrontendConfig &cfg, DmaParams p, Dram *dram);
  ~Host();
  Host(const Host &) = delete;
  Host &operator=(const Host &) = delete;

  void set_micro_ops(MicroOpTable *t) { uops_ = t; }
  void push(const HostCmd &c) { cmds_.push_back(c); }
  // This cycle's inputs of the Controller, from the outputs that depend on its registers only.
  CtrlTopIn inputs(const CtrlTopOut &regs, int64_t cycle) const;
  // The cycle's handshakes, seen in the Controller's outputs; `banks` for the mvout reads.
  void observe(const CtrlTopIn &in, const CtrlTopOut &out, int64_t cycle, const ExecuteUnit &banks);
  bool commands_left() const { return !cmds_.empty(); }
  bool dma_idle() const;
  const std::string &error() const { return error_; }
  uint64_t mvin_rows() const { return mvin_rows_; }

 private:
  struct Row {  // one bank row an mvin writes
    int64_t ready = 0;  // the cycle from which it can be offered
    bool acc = false;
    DmaSpWrite sp;
    DmaAccWrite accw;
    int64_t uop = -1;
    uint64_t xfer = 0;  // the Xfer it belongs to
    unsigned index = 0;
  };
  struct Xfer {  // an accepted mvin / mvout
    GemminiCmd cmd;
    uint64_t seq = 0;
    unsigned rob_id = 0;
    int64_t accepted = 0, done = -1;  // done: the cycle its completion can be offered
    unsigned rows_left = 0;
    bool rows_made = false;
  };
  struct Channel {
    std::deque<Xfer> cmds;
    int64_t bus_free = 0;  // the cycle the channel's bus is free again
  };
  void start_mvin(Xfer &x, int64_t cycle);
  void finish_mvout(const Xfer &x, const ExecuteUnit &banks, int64_t cycle);
  void config_ld(const GemminiCmd &c);

  FrontendConfig cfg_;
  AddrMap amap_;
  DmaParams p_;
  Dram *dram_;
  std::deque<HostCmd> cmds_;
  int64_t last_sent_ = 0;
  uint64_t next_xfer_ = 0;
  Channel ld_, st_;
  std::deque<Row> sp_rows_, acc_rows_;
  uint64_t ld_stride_[3] = {0, 0, 0}, ld_block_stride_[3] = {16, 16, 16};
  bool ld_shrunk_[3] = {false, false, false};
  uint64_t st_stride_ = 0;
  unsigned st_act_ = 0;
  MicroOpTable *uops_ = nullptr;
  std::string error_;
  uint64_t mvin_rows_ = 0;
};

}  // namespace systolique
