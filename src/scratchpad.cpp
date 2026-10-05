// Scratchpad and accumulator banks (Scratchpad.scala, AccumulatorMem.scala, SyncMem.scala,
// Gemmini v0.7.2), as wired for the ExecuteController in rtl/ex/src/ExecuteTop.scala.
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/scratchpad.h"

#include <stdexcept>

namespace systolique {

// ------------------------------------------------------------------ ScratchpadBank
ScratchpadBank::ScratchpadBank(unsigned entries, unsigned read_delay)
    : mem_(entries, SpRow{}), pipe_(read_delay) {}

ScratchpadBank::~ScratchpadBank() = default;

SpReadResp ScratchpadBank::resp() const {
  SpReadResp r = pipe_.out_bits();
  r.valid = pipe_.out_valid();
  return r;
}

ScratchpadBank::Comb ScratchpadBank::comb(bool write_en, bool resp_ready) const {
  Comb c;
  // The SyncReadMem's data for the address registered last cycle enters the 1-entry pipe+flow
  // queue (Scratchpad.scala:141-161).
  c.q_enq_valid = ren_reg_;
  c.q_enq_bits.valid = true;
  c.q_enq_bits.data = mem_[raddr_reg_];
  c.resp_valid = q_.deq_valid(c.q_enq_valid);
  c.resp = q_.deq_bits(&c.q_enq_bits);
  // the response goes into the ExecuteController's Pipeline(spad_read_delay) (Scratchpad.scala:496)
  c.pipe_in_ready = pipe_.in_ready(resp_ready);
  c.resp_fire = c.resp_valid && c.pipe_in_ready;
  c.enq_fire = c.q_enq_valid && q_.enq_ready(c.pipe_in_ready);
  // q_will_be_empty && !singleport_busy_with_write (Scratchpad.scala:163-166)
  const bool will_be_empty = int(q_.count()) + int(c.enq_fire) - int(c.resp_fire) == 0;
  c.req_ready = will_be_empty && !write_en;
  return c;
}

bool ScratchpadBank::read_ready(bool write_en, bool resp_ready) const {
  return comb(write_en, resp_ready).req_ready;
}

void ScratchpadBank::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void ScratchpadBank::eval() {
  next_ = comb(in_.write_en, in_.resp_ready);
  next_.req_fire = in_.req.valid && next_.req_ready;
  evaluated_ = true;
}

void ScratchpadBank::tick() {
  if (!evaluated_)
    eval();
  if (in_.write_en) {
    SpRow &r = mem_.at(in_.write.addr);
    for (unsigned k = 0; k < kFeDim; ++k)
      if ((in_.write.mask >> k) & 1)
        r[k] = in_.write.data[k];
  }
  pipe_.tick(next_.resp_fire, next_.resp, in_.resp_ready);
  q_.tick(next_.enq_fire, next_.q_enq_bits, next_.resp_fire);
  ren_reg_ = next_.req_fire;
  if (next_.req_fire)
    raddr_reg_ = in_.req.addr;
  evaluated_ = false;
}

bool ScratchpadBank::idle() const { return !ren_reg_ && q_.empty() && !pipe_.any_valid(); }

// ------------------------------------------------------------------ Scratchpad
Scratchpad::Scratchpad(const FrontendConfig &cfg) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("Scratchpad: " + e);
  for (unsigned b = 0; b < cfg.sp_banks; ++b)
    banks_.push_back(std::make_unique<ScratchpadBank>(cfg.sp_bank_entries(), cfg.spad_read_delay));
  in_.read.assign(cfg.sp_banks, SpReadReq());
  in_.resp_ready.assign(cfg.sp_banks, false);
  in_.write.assign(cfg.sp_banks, SpWrite());
}

Scratchpad::~Scratchpad() = default;

std::vector<SpReadResp> Scratchpad::resp() const {
  std::vector<SpReadResp> r;
  for (const auto &b : banks_)
    r.push_back(b->resp());
  return r;
}

std::vector<bool> Scratchpad::read_ready(const std::vector<SpWrite> &ex_write,
                                         const std::vector<bool> &resp_ready, const DmaSpWrite &dma) const {
  std::vector<bool> r(banks_.size());
  for (unsigned b = 0; b < banks_.size(); ++b) {
    const bool write_en = ex_write[b].en || (dma.valid && dma.bank == b);  // Scratchpad.scala:519-521
    r[b] = banks_[b]->read_ready(write_en, resp_ready[b]);
  }
  return r;
}

bool Scratchpad::dma_taken(const std::vector<SpWrite> &ex_write, const DmaSpWrite &dma) {
  // the PixelRepeater's resp.ready: the bank is not written by the ExecuteController
  // (Scratchpad.scala:529-543)
  return dma.valid && dma.bank < ex_write.size() && !ex_write[dma.bank].en;
}

void Scratchpad::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void Scratchpad::eval() {
  for (unsigned b = 0; b < banks_.size(); ++b) {
    ScratchpadBank::In bi;
    const bool exwrite = in_.write[b].en;
    const bool dmawrite = in_.dma.valid && in_.dma.bank == b;
    bi.write_en = exwrite || dmawrite;
    if (exwrite) {  // the ExecuteController first (Scratchpad.scala:522-528)
      bi.write = in_.write[b];
    } else if (dmawrite) {
      bi.write.en = true;
      bi.write.addr = in_.dma.addr;
      bi.write.data = in_.dma.data;
      bi.write.mask = in_.dma.mask;
    }
    bi.req = in_.read[b];
    bi.resp_ready = in_.resp_ready[b];
    banks_[b]->set_inputs(bi);
    banks_[b]->eval();
  }
  evaluated_ = true;
}

void Scratchpad::tick() {
  if (!evaluated_)
    eval();
  for (auto &b : banks_)
    b->tick();
  evaluated_ = false;
}

bool Scratchpad::idle() const {
  for (const auto &b : banks_)
    if (!b->idle())
      return false;
  return true;
}

// ------------------------------------------------------------------ AccumulatorBank
AccumulatorBank::AccumulatorBank(unsigned entries) : mem_(entries, AccRow{}) {}

AccumulatorBank::~AccumulatorBank() = default;

bool AccumulatorBank::write_ready(const AccWrite &w) const {
  for (const auto &p : pw_)
    if (p.valid && p.addr == w.addr && w.acc)
      return false;
  return true;
}

AccumulatorBank::Comb AccumulatorBank::comb() const {
  Comb c;
  c.write_fire = in_.write_valid && write_ready(in_.write);
  c.q_enq_valid = rd_reg_;  // RegNext(io.read.req.fire) (AccumulatorMem.scala:323)
  c.q_enq_bits.data = mem_[raddr_reg_];
  c.q_enq_bits.meta = meta_reg_;
  const ReadyView v = ready_view(in_.write_valid, in_.write);
  c.req_fire = in_.req.valid && v.ready(in_.req.addr);
  return c;
}

AccumulatorBank::ReadyView AccumulatorBank::ready_view(bool write_valid, const AccWrite &write) const {
  ReadyView v;
  // The response queue is never drained here (its consumer, the normalizer and AccumulatorScale,
  // is not part of the model; ExecuteTop ties resp.ready low), so resp.fire is 0.
  const bool enq_fire = rd_reg_ && q_.enq_ready(false);
  v.will_be_empty = int(q_.count()) + int(enq_fire) == 0;
  v.write_acc = write_valid && write.acc;
  for (unsigned k = 0; k < 2; ++k) {
    v.pw_valid[k] = pw_[k].valid;
    v.pw_addr[k] = pw_[k].addr;
  }
  return v;
}

void AccumulatorBank::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void AccumulatorBank::eval() {
  next_ = comb();
  evaluated_ = true;
}

void AccumulatorBank::tick(const AccRow &adder_sum) {
  if (!evaluated_)
    eval();
  // The oldest pipelined write lands (AccumulatorMem.scala:121-125): the adder's sum when it
  // accumulates, else its data, byte-masked.
  const Stage &old = pw_[1];
  if (old.valid) {
    AccRow &row = mem_.at(old.addr);
    for (unsigned k = 0; k < kFeDim; ++k)
      if ((old.mask >> (4 * k)) & 0xf) {
        const uint32_t nv = old.acc ? uint32_t(adder_sum[k]) : uint32_t(old.data[k]);
        uint32_t cur = uint32_t(row[k]);
        for (unsigned byte = 0; byte < 4; ++byte)
          if ((old.mask >> (4 * k + byte)) & 1)
            cur = (cur & ~(0xffu << (8 * byte))) | (nv & (0xffu << (8 * byte)));
        row[k] = int32_t(cur);
      }
  }
  const bool rmw = next_.write_fire && in_.write.acc;
  q_.tick(next_.q_enq_valid && q_.enq_ready(false), next_.q_enq_bits, false);
  pw_[1] = pw_[0];
  pw_[0].valid = next_.write_fire;
  pw_[0].addr = in_.write.addr;
  pw_[0].data = in_.write.data;
  pw_[0].acc = in_.write.acc;
  pw_[0].mask = in_.write.mask;
  rd_reg_ = next_.req_fire;
  // The read port serves the accumulate's read first (AccumulatorMem.scala:126-127).
  if (rmw)
    raddr_reg_ = in_.write.addr;
  else if (next_.req_fire)
    raddr_reg_ = in_.req.addr;
  if (next_.req_fire) {
    meta_reg_.scale = in_.req.scale;
    meta_reg_.act = in_.req.act;
  }
  evaluated_ = false;
}

bool AccumulatorBank::idle() const { return !rd_reg_ && !pw_[0].valid && !pw_[1].valid; }

// ------------------------------------------------------------------ Accumulator
Accumulator::Accumulator(const FrontendConfig &cfg) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("Accumulator: " + e);
  for (unsigned b = 0; b < cfg.acc_banks; ++b)
    banks_.push_back(std::make_unique<AccumulatorBank>(cfg.acc_bank_entries()));
  in_.read.assign(cfg.acc_banks, AccReadReq());
  in_.write.assign(cfg.acc_banks, AccWrite());
}

Accumulator::~Accumulator() = default;

void Accumulator::bank_inputs(const In &in, unsigned b, AccumulatorBank::In &bi) const {
  // rtl/ex/src/ExecuteTop.scala, as Scratchpad.scala:718-823: the ExecuteController's write
  // first, then the DMA's; acc and addr are muxed even when neither writes.
  const bool exwrite = in.write[b].valid;
  const bool dmawrite = in.dma.valid && in.dma.bank == b;
  bi = AccumulatorBank::In();
  if (exwrite) {
    bi.write_valid = true;
    bi.write = in.write[b];
  } else {
    bi.write.acc = in.dma.acc;
    bi.write.addr = in.dma.addr;
    if (dmawrite) {
      bi.write_valid = true;
      bi.write.data = in.dma.data;
      bi.write.mask = in.dma.mask;
    }
  }
  bi.write.valid = bi.write_valid;
  bi.req = in.read[b];
}

std::vector<AccumulatorBank::ReadyView> Accumulator::ready_view(const std::vector<AccWrite> &ex_write,
                                                                 const DmaAccWrite &dma) const {
  In in;
  in.read.assign(banks_.size(), AccReadReq());
  in.write = ex_write;
  in.dma = dma;
  std::vector<AccumulatorBank::ReadyView> v;
  for (unsigned b = 0; b < banks_.size(); ++b) {
    AccumulatorBank::In bi;
    bank_inputs(in, b, bi);
    v.push_back(banks_[b]->ready_view(bi.write_valid, bi.write));
  }
  return v;
}

bool Accumulator::dma_taken(const std::vector<AccWrite> &ex_write, const DmaAccWrite &dma) const {
  if (!dma.valid || dma.bank >= banks_.size() || ex_write[dma.bank].valid)
    return false;
  AccWrite w;
  w.addr = dma.addr;
  w.acc = dma.acc;
  return banks_[dma.bank]->write_ready(w);
}

void Accumulator::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void Accumulator::eval() {
  for (unsigned b = 0; b < banks_.size(); ++b) {
    AccumulatorBank::In bi;
    bank_inputs(in_, b, bi);
    banks_[b]->set_inputs(bi);
    banks_[b]->eval();
  }
  evaluated_ = true;
}

void Accumulator::tick() {
  if (!evaluated_)
    eval();
  // AccPipeShared (AccumulatorMem.scala:74-90): Mux1H over the banks whose adder is valid (an OR
  // of the selected operands), then the sum through ShiftRegister(_, acc_latency - 1).
  AccRow op1{}, op2{};
  for (const auto &b : banks_)
    if (b->adder_valid())
      for (unsigned k = 0; k < kFeDim; ++k) {
        op1[k] = int32_t(uint32_t(op1[k]) | uint32_t(b->adder_op1()[k]));
        op2[k] = int32_t(uint32_t(op2[k]) | uint32_t(b->adder_op2()[k]));
      }
  AccRow next_sum{};
  for (unsigned k = 0; k < kFeDim; ++k)
    next_sum[k] = int32_t(uint32_t(op1[k]) + uint32_t(op2[k]));
  for (unsigned b = 0; b < banks_.size(); ++b) {
    if (in_.write[b].valid && !banks_[b]->write_fire())
      ++dropped_;
    banks_[b]->tick(adder_sum_);
  }
  adder_sum_ = next_sum;
  evaluated_ = false;
}

bool Accumulator::idle() const {
  for (const auto &b : banks_)
    if (!b->idle())
      return false;
  return true;
}

}  // namespace systolique
