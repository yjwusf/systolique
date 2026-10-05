// ReservationStation (ReservationStation.scala, Gemmini v0.7.2). Line numbers are that file's.
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/reservation_station.h"

#include <algorithm>
#include <stdexcept>

namespace systolique {

ReservationStation::ReservationStation(const FrontendConfig &cfg)
    : cfg_(cfg), amap_(cfg.addr_map()), block_(cfg.dim()), type_bits_(cfg.rob_type_bits()) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("ReservationStation: " + e);
  const unsigned n[3] = {cfg.rs_entries_ld, cfg.rs_entries_ex, cfg.rs_entries_st};
  for (unsigned q = 0; q < 3; ++q) {
    e_[q].resize(n[q]);
    for (auto &e : e_[q]) {
      for (unsigned d = 0; d < 3; ++d)
        e.deps[d].assign(n[d], false);
      e.opa.start = e.opa.end = e.opb.start = e.opb.end = LocalAddr(0, amap_);
    }
  }
  settle();
}

ReservationStation::~ReservationStation() = default;

unsigned ReservationStation::queue_of(const GemminiCmd &cmd) const {
  const unsigned funct = cmd.funct;
  const bool is_compute = funct == COMPUTE_AND_STAY_CMD || funct == COMPUTE_AND_FLIP_CMD;
  const unsigned t = unsigned(cmd.rs1 & 3);
  const bool is_load = funct == LOAD_CMD || funct == LOAD2_CMD || funct == LOAD3_CMD ||
                       (funct == CONFIG_CMD && t == CONFIG_LOAD);
  const bool is_ex = funct == PRELOAD_CMD || is_compute || (funct == CONFIG_CMD && t == CONFIG_EX);
  return is_load ? LDQ : is_ex ? EXQ : STQ;  // Mux1H (:300-304)
}

ReservationStation::Entry ReservationStation::make_entry(const GemminiCmd &cmd) const {
  Entry ne;
  const unsigned funct = cmd.funct;
  const bool is_compute = funct == COMPUTE_AND_STAY_CMD || funct == COMPUTE_AND_FLIP_CMD;
  const unsigned rows_bits = log2_up(block_ + 1);
  const unsigned mv_cols_bits = cfg_.mvin_cols_bits();
  ne.issued = false;
  ne.cmd = cmd;
  ne.is_config = funct == CONFIG_CMD;

  // op1 (:216-229)
  Op op1, op2, dst;
  op1.valid = funct == PRELOAD_CMD || is_compute;
  op1.start = LocalAddr(cmd.rs1, amap_);
  if (funct == PRELOAD_CMD) {
    const uint64_t preload_rows = rs_rows(cmd.rs1, rows_bits);
    op1.end = op1.start.plus_overflow(preload_rows, op1.wraps_around);
  } else {
    const uint64_t rows = rs_rows(cmd.rs1, rows_bits), cols = rs_cols(cmd.rs1, rows_bits);
    const uint64_t compute_rows = (conf_.a_transpose ? cols : rows) * conf_.a_stride;
    op1.end = op1.start.plus_overflow(compute_rows, op1.wraps_around);
  }
  // op2 (:231-259)
  op2.valid = is_compute || funct == STORE_CMD;
  op2.start = LocalAddr(cmd.rs2, amap_);
  if (is_compute) {
    op2.end = op2.start.plus_overflow(rs_rows(cmd.rs2, rows_bits), op2.wraps_around);
  } else if (conf_.pooling_is_enabled) {
    const uint32_t acc_bank = op2.start.acc_bank();
    LocalAddr next_bank(0, amap_);
    next_bank.set_flags(true, false, false);
    next_bank.set_data(uint64_t(acc_bank + 1) << amap_.acc_bank_row_bits);
    op2.end = next_bank;
    op2.wraps_around = next_bank.acc_bank() == 0;
  } else {
    const uint64_t mvout_cols = rs_cols(cmd.rs2, mv_cols_bits), mvout_rows = rs_rows(cmd.rs2, rows_bits);
    const uint64_t mats = mvout_cols / block_ + (mvout_cols % block_ != 0);
    const uint64_t total = (mats - 1) * block_ + mvout_rows;
    bool ovf = false;
    op2.end = op2.start.plus_overflow(total, ovf);
    op2.wraps_around = conf_.pooling_is_enabled || ovf;
  }
  // dst (:261-293)
  dst.valid = funct == PRELOAD_CMD || funct == LOAD_CMD || funct == LOAD2_CMD || funct == LOAD3_CMD;
  dst.start = LocalAddr(cmd.rs2 & 0xffffffffull, amap_);
  if (funct == PRELOAD_CMD) {
    const uint64_t preload_rows = rs_rows(cmd.rs2, rows_bits) * conf_.c_stride;
    dst.end = dst.start.plus_overflow(preload_rows, dst.wraps_around);
  } else {
    const unsigned id = funct == LOAD2_CMD ? 1 : funct == LOAD3_CMD ? 2 : 0;
    const uint64_t block_stride = conf_.ld_block_strides[id];
    const uint64_t pixel_repeats = conf_.ld_pixel_repeats[id];
    const uint64_t mvin_cols = rs_cols(cmd.rs2, mv_cols_bits), mvin_rows = rs_rows(cmd.rs2, rows_bits);
    const uint64_t mats = mvin_cols / block_ + (mvin_cols % block_ != 0);
    const uint64_t total = (mats - 1) * block_stride + mvin_rows;
    if (cfg_.has_first_layer_optimizations) {
      const LocalAddr start = dst.start;
      bool uf = false;
      if (!start.is_acc())
        dst.start = start.full_sp_addr() > amap_.sp_rows / 2 ? start.floor_sub(pixel_repeats, amap_.sp_rows / 2, uf)
                                                             : start.floor_sub(pixel_repeats, 0, uf);
    }
    dst.end = dst.start.plus_overflow(total, dst.wraps_around);
  }
  // (:207-214)
  ne.opa_is_dst = dst.valid;
  if (dst.valid) {
    ne.opa = dst;
    ne.opb = op1.valid ? op1 : op2;
  } else {
    ne.opa = op1.valid ? op1 : op2;
    ne.opb = op2;
  }
  ne.q = queue_of(cmd);

  // dependencies on the current entries (:309-339)
  const bool not_config = !ne.is_config;
  for (unsigned q = 0; q < 3; ++q) {
    ne.deps[q].clear();
    for (size_t i = 0; i < e_[q].size(); ++i) {
      const Entry &e = e_[q][i];
      bool d = false;
      if (ne.q == LDQ) {
        if (q == LDQ)
          d = e.valid && !e.issued;
        else if (q == EXQ)
          d = e.valid && !ne.is_config &&
              ((ne.opa.overlaps(e.opa) && e.opa.valid) || (ne.opa.overlaps(e.opb) && e.opb.valid));
        else
          d = e.valid && e.opa.valid && not_config && ne.opa.overlaps(e.opa);
      } else if (ne.q == EXQ) {
        if (q == LDQ)
          d = e.valid && e.opa.valid && not_config && (ne.opa.overlaps(e.opa) || ne.opb.overlaps(e.opa));
        else if (q == EXQ)
          d = e.valid && !e.issued;
        else
          d = e.valid && e.opa.valid && not_config && ne.opa_is_dst && ne.opa.overlaps(e.opa);
      } else {
        if (q == LDQ)
          d = e.valid && e.opa.valid && not_config && ne.opa.overlaps(e.opa);
        else if (q == EXQ)
          d = e.valid && e.opa.valid && not_config && e.opa_is_dst && ne.opa.overlaps(e.opa);
        else
          d = e.valid && !e.issued;
      }
      ne.deps[q].push_back(d);
    }
  }
  ne.complete_on_issue = ne.is_config && ne.q != EXQ;  // :343
  ne.valid = true;
  return ne;
}

bool ReservationStation::alloc_ready(const GemminiCmd &cmd) const {
  // :355-362: the first invalid entry of the command's queue, else the last one
  const auto &es = e_[queue_of(cmd)];
  size_t id = es.size() - 1;
  for (size_t i = 0; i < es.size(); ++i)
    if (!es[i].valid) {
      id = i;
      break;
    }
  return !es[id].valid;
}

void ReservationStation::settle() {
  for (unsigned q = 0; q < 3; ++q) {
    Issue &is = issue_[q];
    is = Issue();
    is.rob_id = q << type_bits_;  // Cat(q, OHToUInt(0)) when nothing is selected
    for (size_t i = 0; i < e_[q].size(); ++i) {
      const Entry &e = e_[q][i];
      if (e.valid && e.ready() && !e.issued) {  // :395-404, PriorityEncoderOH
        is.valid = true;
        is.entry = unsigned(i);
        is.cmd = e.cmd;
        is.rob_id = (q << type_bits_) | unsigned(i);
        is.cmd.rob_valid = true;
        is.cmd.rob_id = is.rob_id;
        break;
      }
    }
  }
}

bool ReservationStation::busy() const {
  unsigned util = 0;
  for (unsigned q = 0; q < 3; ++q)
    for (const auto &e : e_[q])
      util += e.valid;
  return util != 0 && !(util == 1 && conf_.solitary_preload);  // :135
}

unsigned ReservationStation::occupancy(unsigned q) const {
  unsigned n = 0;
  for (const auto &e : e_[q])
    n += e.valid;
  return n;
}

void ReservationStation::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void ReservationStation::eval() {
  done_ = Completed();
  alloc_fire_ = false;
  for (unsigned q = 0; q < 3; ++q)
    next_[q] = e_[q];
  next_conf_ = conf_;

  // allocation (:184-389)
  if (in_.alloc_valid && alloc_ready(in_.alloc)) {
    alloc_fire_ = true;
    Entry ne = make_entry(in_.alloc);
    const auto &es = e_[ne.q];
    size_t id = es.size() - 1;
    for (size_t i = 0; i < es.size(); ++i)
      if (!es[i].valid) {
        id = i;
        break;
      }
    next_[ne.q][id] = ne;
    const GemminiCmd &c = in_.alloc;
    Config &cf = next_conf_;
    if (ne.is_config && ne.q == EXQ) {
      cf.a_stride = (c.rs1 >> 16) & 0xffff;
      cf.c_stride = (c.rs2 >> 48) & 0xffff;
      if (!((c.rs1 >> 7) & 1))
        cf.a_transpose = (c.rs1 >> 8) & 1;
    } else if (ne.is_config && ne.q == LDQ) {
      const unsigned id2 = unsigned((c.rs1 >> 3) & 3);
      const uint64_t block_stride = (c.rs1 >> 16) & 0xffff;
      const unsigned prb = std::min(8u, log2_up(cfg_.array.cols() + 1));  // pixel_repeats_bits
      uint64_t repeat_pixels = (c.rs1 >> 8) & mask_bits(prb);
      if (repeat_pixels < 1)
        repeat_pixels = 1;
      if (id2 < 3) {
        cf.ld_block_strides[id2] = block_stride & mask_bits(log2_up(cfg_.sp_rows()));
        cf.ld_pixel_repeats[id2] = (repeat_pixels - 1) & mask_bits(prb);
      }
    } else if (ne.is_config && ne.q == STQ && (c.rs1 & 3) != CONFIG_NORM) {
      cf.pooling_is_enabled = ((c.rs1 >> 4) & 3) != 0;
    } else if (c.funct == PRELOAD_CMD) {
      cf.solitary_preload = true;
    } else if (c.funct == COMPUTE_AND_FLIP_CMD || c.funct == COMPUTE_AND_STAY_CMD) {
      cf.solitary_preload = false;
    }
  }

  // issue (:392-448)
  for (unsigned q = 0; q < 3; ++q) {
    if (!(issue_[q].valid && in_.issue_ready[q]))
      continue;
    const unsigned id = issue_[q].entry;
    const Entry &ie = e_[q][id];
    next_[q][id].issued = true;
    next_[q][id].valid = !ie.complete_on_issue;
    for (unsigned q2 = 0; q2 < 3; ++q2)
      for (auto &e : next_[q2])
        if (q == q2 || ie.complete_on_issue)
          e.deps[q][id] = false;
    const bool c = ie.complete_on_issue && ie.cmd.from_matmul_fsm;
    if (q == LDQ) done_.matmul_ld += c;
    if (q == EXQ) done_.matmul_ex += c;
    if (q == STQ) done_.matmul_st += c;
  }

  // completion (:451-483)
  if (in_.completed_valid) {
    const unsigned qt = in_.completed_id >> type_bits_, id = in_.completed_id & unsigned(mask_bits(type_bits_));
    if (qt < 3 && id < e_[qt].size()) {
      for (unsigned q2 = 0; q2 < 3; ++q2)
        for (auto &e : next_[q2])
          e.deps[qt][id] = false;
      next_[qt][id].valid = false;
      const bool c = e_[qt][id].cmd.from_matmul_fsm;
      if (qt == LDQ) done_.matmul_ld += c;
      if (qt == EXQ) done_.matmul_ex += c;
      if (qt == STQ) done_.matmul_st += c;
    }
  }

  // ld / st entries never keep an opb (:487-492)
  for (unsigned q : {unsigned(LDQ), unsigned(STQ)})
    for (auto &e : next_[q])
      e.opb.valid = false;
  evaluated_ = true;
}

void ReservationStation::tick() {
  if (!evaluated_)
    eval();
  for (unsigned q = 0; q < 3; ++q)
    e_[q].swap(next_[q]);
  conf_ = next_conf_;
  settle();
  evaluated_ = false;
}

}  // namespace systolique
