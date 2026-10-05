// The core and the modelled load / store side around the Controller (host.h). Not derived
// from Gemmini's sources beyond the command encodings it decodes (GemminiISA.scala) and the
// row layout of mvin / mvout (gemmini.h).
#include "systolique/host.h"

#include <algorithm>
#include <cstring>

namespace systolique {

// ------------------------------------------------------------------ Dram
uint8_t Dram::read8(uint64_t addr) const {
  auto it = pages_.find(addr >> kPageBits);
  return it == pages_.end() ? 0 : it->second[addr & ((1u << kPageBits) - 1)];
}

void Dram::write8(uint64_t addr, uint8_t v) {
  auto &p = pages_[addr >> kPageBits];
  if (p.empty())
    p.assign(size_t(1) << kPageBits, 0);
  p[addr & ((1u << kPageBits) - 1)] = v;
}

void Dram::read(uint64_t addr, uint8_t *dst, size_t n) const {
  for (size_t i = 0; i < n; ++i)
    dst[i] = read8(addr + i);
}

void Dram::write(uint64_t addr, const uint8_t *src, size_t n) {
  for (size_t i = 0; i < n; ++i)
    write8(addr + i, src[i]);
}

// ------------------------------------------------------------------ Host
namespace {
bool is_mvin(unsigned f) { return f == LOAD_CMD || f == LOAD2_CMD || f == LOAD3_CMD; }
unsigned mvin_id(unsigned f) { return f == LOAD2_CMD ? 1 : f == LOAD3_CMD ? 2 : 0; }
int64_t div_up(int64_t a, int64_t b) { return (a + b - 1) / b; }
}  // namespace

Host::Host(const FrontendConfig &cfg, DmaParams p, Dram *dram) : cfg_(cfg), amap_(cfg.addr_map()), p_(p), dram_(dram) {
  if (p_.bytes_per_cycle == 0 || p_.max_cmds == 0)
    throw std::invalid_argument("Host: bytes_per_cycle and max_cmds must be positive");
}

Host::~Host() = default;

bool Host::dma_idle() const { return ld_.cmds.empty() && st_.cmds.empty() && sp_rows_.empty() && acc_rows_.empty(); }

CtrlTopIn Host::inputs(const CtrlTopOut &regs, int64_t cycle) const {
  CtrlTopIn in;
  // the core: fences wait until Gemmini is idle; then the next command, `gap` cycles after the last
  size_t k = 0;
  const bool quiet = !regs.busy && dma_idle();
  while (k < cmds_.size() && cmds_[k].cmd.funct == kFence && quiet)
    ++k;
  if (k < cmds_.size() && cmds_[k].cmd.funct != kFence && cycle >= last_sent_ + int64_t(cmds_[k].gap)) {
    in.cmd_valid = true;
    in.cmd.funct = cmds_[k].cmd.funct;
    in.cmd.rs1 = cmds_[k].cmd.rs1;
    in.cmd.rs2 = cmds_[k].cmd.rs2;
    in.cmd.op = cmds_[k].op;
    in.cmd.uop = cmds_[k].uop;
  }
  // the load and store controllers
  in.ld_ready = ld_.cmds.size() < p_.max_cmds;
  in.st_ready = st_.cmds.size() < p_.max_cmds;
  if (!ld_.cmds.empty() && ld_.cmds.front().done >= 0 && ld_.cmds.front().done <= cycle) {
    in.ld_completed_valid = true;
    in.ld_completed_bits = ld_.cmds.front().rob_id;
  }
  if (!st_.cmds.empty() && st_.cmds.front().done >= 0 && st_.cmds.front().done <= cycle) {
    in.st_completed_valid = true;
    in.st_completed_bits = st_.cmds.front().rob_id;
  }
  // the DMA's row writes, in the order they arrived
  if (!sp_rows_.empty() && sp_rows_.front().ready <= cycle)
    in.dma_sp = sp_rows_.front().sp;
  if (!acc_rows_.empty() && acc_rows_.front().ready <= cycle)
    in.dma_acc = acc_rows_.front().accw;
  return in;
}

void Host::config_ld(const GemminiCmd &c) {
  // ConfigMvinRs1 (GemminiISA.scala:83-92): id [4:3], shrunk [2], block_stride [31:16]; rs2: stride
  const unsigned id = unsigned((c.rs1 >> 3) & 3);
  if (id < 3) {
    ld_stride_[id] = c.rs2;
    ld_shrunk_[id] = (c.rs1 >> 2) & 1;
    ld_block_stride_[id] = (c.rs1 >> 16) & 0xffff;
  }
  if (uint32_t(c.rs1 >> 32) != float_bits(1.0f) && error_.empty())
    error_ = "mvin scales other than 1.0 are not modelled";
}

void Host::start_mvin(Xfer &x, int64_t cycle) {
  const unsigned dim = cfg_.dim();
  const unsigned id = mvin_id(x.cmd.funct);
  const LocalAddr la(x.cmd.rs2, amap_);
  const unsigned rows = rs_rows(x.cmd.rs2, cfg_.mvin_rows_bits());
  const unsigned cols = rs_cols(x.cmd.rs2, cfg_.mvin_cols_bits());
  const bool to_acc = la.is_acc();
  const unsigned elem = to_acc && !ld_shrunk_[id] ? cfg_.array.acc_bits / 8 : cfg_.array.in_bits / 8;
  const int64_t row_bytes = int64_t(cols) * elem;
  const int64_t s = std::max(cycle + int64_t(p_.latency), ld_.bus_free);
  const unsigned blocks = (cols + dim - 1) / dim;
  for (unsigned r = 0; r < rows; ++r) {
    const int64_t arrive = s + div_up(int64_t(r + 1) * row_bytes, p_.bytes_per_cycle);
    for (unsigned b = 0; b < blocks; ++b) {
      Row w;
      w.ready = arrive;
      w.acc = to_acc;
      w.index = r * blocks + b;
      const uint64_t local = uint64_t(la.data()) + uint64_t(b) * ld_block_stride_[id] + r;
      const uint64_t dram = x.cmd.rs1 + uint64_t(r) * ld_stride_[id] + uint64_t(b) * dim * elem;
      for (unsigned k = 0; k < dim; ++k) {
        const unsigned e = b * dim + k;
        if (e >= cols)
          break;
        if (to_acc) {
          int32_t v;
          if (elem == 4) {
            uint8_t bytes[4];
            dram_->read(dram + uint64_t(k) * 4, bytes, 4);
            std::memcpy(&v, bytes, 4);
          } else {
            v = int8_t(dram_->read8(dram + k));
          }
          w.accw.data[k] = v;
          w.accw.mask |= 0xfull << (4 * k);
        } else {
          w.sp.data[k] = int8_t(dram_->read8(dram + k));
          w.sp.mask |= 1ull << k;
        }
      }
      if (to_acc) {
        const uint32_t a = uint32_t(local & mask_bits(amap_.acc_addr_bits));
        w.accw.valid = true;
        w.accw.bank = a >> amap_.acc_bank_row_bits;
        w.accw.addr = a & uint32_t(mask_bits(amap_.acc_bank_row_bits));
        w.accw.acc = la.accumulate();
      } else {
        const uint32_t a = uint32_t(local & mask_bits(amap_.sp_addr_bits));
        w.sp.valid = true;
        w.sp.bank = a >> amap_.sp_bank_row_bits;
        w.sp.addr = a & uint32_t(mask_bits(amap_.sp_bank_row_bits));
      }
      w.uop = x.cmd.uop;
      w.xfer = x.seq;
      (to_acc ? acc_rows_ : sp_rows_).push_back(w);
      ++x.rows_left;
    }
  }
  ld_.bus_free = s + div_up(int64_t(rows) * row_bytes, p_.bytes_per_cycle);
  x.rows_made = true;
  if (x.rows_left == 0)
    x.done = cycle + 1;
  if (uops_ && uops_->has(x.cmd.uop))
    first_cycle(uops_->at(x.cmd.uop).started, s);
}

void Host::finish_mvout(const Xfer &x, const ExecuteUnit &banks, int64_t cycle) {
  const unsigned dim = cfg_.dim();
  const LocalAddr la(x.cmd.rs2, amap_);
  const unsigned rows = rs_rows(x.cmd.rs2, cfg_.mvin_rows_bits());
  const unsigned cols = rs_cols(x.cmd.rs2, cfg_.mvin_cols_bits());
  const bool from_acc = la.is_acc(), full = la.read_full();
  const unsigned elem = from_acc && full ? cfg_.array.acc_bits / 8 : cfg_.array.in_bits / 8;
  if (la.is_garbage())
    return;
  const unsigned blocks = (cols + dim - 1) / dim;
  for (unsigned r = 0; r < rows; ++r)
    for (unsigned b = 0; b < blocks; ++b) {
      const uint64_t local = uint64_t(la.data()) + uint64_t(b) * dim + r;  // block stride DIM (ReservationStation.scala:166)
      const uint64_t dram = x.cmd.rs1 + uint64_t(r) * st_stride_ + uint64_t(b) * dim * elem;
      for (unsigned k = 0; k < dim && b * dim + k < cols; ++k) {
        if (from_acc) {
          const uint32_t a = uint32_t(local & mask_bits(amap_.acc_addr_bits));
          int64_t v = banks.accumulator().row(a >> amap_.acc_bank_row_bits,
                                              a & uint32_t(mask_bits(amap_.acc_bank_row_bits)))[k];
          if (full) {
            const int32_t w = int32_t(v);
            uint8_t bytes[4];
            std::memcpy(bytes, &w, 4);
            dram_->write(dram + uint64_t(k) * 4, bytes, 4);
          } else {
            // ACC_SCALE (identity), clip to inputType, then the activation (gemmini.h scale_and_sat)
            v = std::min<int64_t>(127, std::max<int64_t>(-128, v));
            if (st_act_ == ACT_RELU && v < 0)
              v = 0;
            dram_->write8(dram + k, uint8_t(int8_t(v)));
          }
        } else {
          const uint32_t a = uint32_t(local & mask_bits(amap_.sp_addr_bits));
          dram_->write8(dram + k, uint8_t(banks.scratchpad().row(a >> amap_.sp_bank_row_bits,
                                                                 a & uint32_t(mask_bits(amap_.sp_bank_row_bits)))[k]));
        }
      }
      if (uops_ && uops_->has(x.cmd.uop)) {
        MicroOp u;
        u.kind = UopKind::DmaRow;
        u.op = uops_->at(x.cmd.uop).op;
        u.parent = x.cmd.uop;
        u.operand = 'O';
        u.row = r * blocks + b;
        u.to_acc = from_acc;
        u.addr = uint32_t(local);
        u.cycle = u.done = cycle;
        uops_->add(u);
      }
    }
}

void Host::observe(const CtrlTopIn &in, const CtrlTopOut &out, int64_t cycle, const ExecuteUnit &banks) {
  auto rec = [&](int64_t id) -> MicroOp * { return uops_ && uops_->has(id) ? &uops_->at(id) : nullptr; };
  // the core
  const bool quiet = !out.busy && dma_idle();
  while (!cmds_.empty() && cmds_.front().cmd.funct == kFence && quiet) {
    if (MicroOp *u = rec(cmds_.front().uop)) {
      first_cycle(u->sent, cycle);
      first_cycle(u->completed, cycle);
    }
    cmds_.pop_front();
  }
  if (in.cmd_valid && out.cmd_ready && !cmds_.empty()) {
    if (MicroOp *u = rec(cmds_.front().uop))
      first_cycle(u->sent, cycle);
    cmds_.pop_front();
    last_sent_ = cycle;
  }
  // load issue
  if (out.ld.valid && in.ld_ready) {
    const GemminiCmd &c = out.ld.cmd;
    if (MicroOp *u = rec(c.uop))
      first_cycle(u->issued, cycle);
    if (c.funct == CONFIG_CMD) {
      config_ld(c);
      if (MicroOp *u = rec(c.uop)) {
        first_cycle(u->started, cycle);
        first_cycle(u->completed, cycle);  // completed on issue (ReservationStation.scala:343)
      }
    } else if (is_mvin(c.funct)) {
      Xfer x;
      x.cmd = c;
      x.rob_id = out.ld.rob_id;
      x.accepted = cycle;
      x.seq = next_xfer_++;
      ld_.cmds.push_back(x);
      start_mvin(ld_.cmds.back(), cycle);
    } else if (error_.empty()) {
      error_ = std::string("unexpected command on the load queue: ") + funct_name(c.funct);
    }
  }
  // rows taken by the banks
  auto taken = [&](std::deque<Row> &q) {
    const Row w = q.front();
    q.pop_front();
    ++mvin_rows_;
    for (auto &x : ld_.cmds)
      if (x.seq == w.xfer) {
        if (--x.rows_left == 0)
          x.done = cycle + 1;
        break;
      }
    if (MicroOp *u = rec(w.uop)) {
      MicroOp r;
      r.kind = UopKind::DmaRow;
      r.op = u->op;
      r.parent = w.uop;
      r.operand = 'I';
      r.row = w.index;
      r.to_acc = w.acc;
      r.bank = w.acc ? w.accw.bank : w.sp.bank;
      r.addr = w.acc ? w.accw.addr : w.sp.addr;
      r.cycle = cycle;
      r.done = w.acc ? cycle + 2 : cycle;
      last_cycle(u->wb_last, cycle);
      first_cycle(u->wb_first, cycle);
      uops_->add(r);  // may move the table: u is not used after this
    }
  };
  if (in.dma_sp.valid && out.bank.dma_sp_taken)
    taken(sp_rows_);
  if (in.dma_acc.valid && out.bank.dma_acc_taken)
    taken(acc_rows_);
  if (in.ld_completed_valid && out.ld_completed_ready) {
    if (MicroOp *u = rec(ld_.cmds.front().cmd.uop))
      first_cycle(u->completed, cycle);
    ld_.cmds.pop_front();
  }
  // store issue
  if (out.st.valid && in.st_ready) {
    const GemminiCmd &c = out.st.cmd;
    if (MicroOp *u = rec(c.uop))
      first_cycle(u->issued, cycle);
    if (c.funct == CONFIG_CMD) {
      // ConfigMvoutRs1 / Rs2 (GemminiISA.scala:107-125): activation [3:2], stride rs2[31:0]
      st_stride_ = c.rs2 & 0xffffffffull;
      st_act_ = unsigned((c.rs1 >> 2) & 3);
      if (uint32_t(c.rs2 >> 32) != float_bits(1.0f) && error_.empty())
        error_ = "mvout scales other than 1.0 are not modelled";
      if (MicroOp *u = rec(c.uop)) {
        first_cycle(u->started, cycle);
        first_cycle(u->completed, cycle);
      }
    } else if (c.funct == STORE_CMD) {
      Xfer x;
      x.cmd = c;
      x.rob_id = out.st.rob_id;
      x.accepted = cycle;
      const LocalAddr la(c.rs2, amap_);
      const unsigned rows = rs_rows(c.rs2, cfg_.mvin_rows_bits()), cols = rs_cols(c.rs2, cfg_.mvin_cols_bits());
      const unsigned elem = la.is_acc() && la.read_full() ? cfg_.array.acc_bits / 8 : cfg_.array.in_bits / 8;
      const int64_t s = std::max(cycle + int64_t(p_.latency), st_.bus_free);
      st_.bus_free = s + div_up(int64_t(rows) * cols * elem, p_.bytes_per_cycle);
      x.done = st_.bus_free;
      if (MicroOp *u = rec(c.uop))
        first_cycle(u->started, s);
      st_.cmds.push_back(x);
    } else if (error_.empty()) {
      error_ = std::string("unexpected command on the store queue: ") + funct_name(c.funct);
    }
  }
  if (in.st_completed_valid && out.st_completed_ready) {
    const Xfer &x = st_.cmds.front();
    finish_mvout(x, banks, cycle);
    if (MicroOp *u = rec(x.cmd.uop))
      first_cycle(u->completed, cycle);
    st_.cmds.pop_front();
  }
}

}  // namespace systolique
