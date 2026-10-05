// LoopMatmul (LoopMatmul.scala, Gemmini v0.7.2). Line numbers below are that file's.
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/loop_matmul.h"

#include <algorithm>
#include <stdexcept>

namespace systolique {

namespace {
constexpr unsigned kIter = 16;  // iterator_bitwidth (:771)
}

LoopMatmul::LoopMatmul(const FrontendConfig &cfg) : cfg_(cfg), amap_(cfg.addr_map()) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("LoopMatmul: " + e);
  block_ = cfg.dim();
  max_block_len_ = cfg.max_block_len();          // :772
  max_block_len_acc_ = cfg.max_block_len_acc();  // :773
  max_addr_ = cfg.sp_rows();
  max_acc_addr_ = cfg.acc_rows();
  mvin_rows_bits_ = cfg.mvin_rows_bits();
  mvin_cols_bits_ = cfg.mvin_cols_bits();
  // reset (:1110-1117)
  for (unsigned i = 0; i < 2; ++i) {
    r_.loops[i].reset();
    r_.loops[i].a_addr_start = i * (max_addr_ / 2);
    r_.loops[i].b_addr_end = (i + 1) * (max_addr_ / 2);
    r_.loops[i].resadd_addr_start = i * (max_acc_addr_ / 2);
  }
  settle();
}

LoopMatmul::~LoopMatmul() = default;

std::string LoopMatmul::unsupported() const {
  for (const auto &l : r_.loops)
    if (l.configured && (l.act == ACT_LAYERNORM || l.act == ACT_SOFTMAX))
      return "LoopMatmul's LAYERNORM / SOFTMAX mvout sequences are not modelled";
  return "";
}

bool LoopMatmul::idle() const {
  return r_.cmdq.empty() && !r_.loops[0].configured && !r_.loops[1].configured && r_.lda.idle() &&
         r_.ldb.idle() && r_.ldd.idle() && r_.ex.idle() && r_.stc.idle() && r_.ld_util == 0 &&
         r_.st_util == 0 && r_.ex_util == 0;
}

// LdA (:28-122)
LoopMatmul::SubCmd LoopMatmul::lda_cmd(const Regs &R) const {
  const LdReq &r = R.lda_req;
  const Sub &s = R.lda;
  const uint64_t row_it = r.transpose ? s.k : s.i, col_it = r.transpose ? s.i : s.k;
  const uint64_t max_row = r.transpose ? r.max_k : r.max_i, max_col = r.transpose ? r.max_i : r.max_k;
  const uint64_t row_pad = r.transpose ? r.pad_k : r.pad_i, col_pad = r.transpose ? r.pad_i : r.pad_k;
  const uint64_t max_blocks = std::min<uint64_t>(max_col, max_block_len_);  // :61-62
  const uint64_t dram_offset = (row_it * r.dram_stride + col_it) * block_ * (cfg_.array.in_bits / 8);
  const uint64_t sp_addr = r.addr_start + (row_it * max_col + col_it) * block_;
  const uint64_t blocks = col_it + max_blocks <= max_col ? max_blocks : max_col - col_it;
  const uint64_t cols = blocks * block_ - (col_it + blocks >= max_col ? col_pad : 0);
  const uint64_t rows = block_ - (row_it == max_row - 1 ? row_pad : 0);
  SubCmd c;
  c.cmd.funct = LOAD_CMD;
  c.cmd.rs1 = dram_add(r.dram_addr, dram_offset);
  const LocalAddr la = r.is_resadd ? LocalAddr::cast_acc(sp_addr, false, false, amap_) : LocalAddr::cast_sp(sp_addr, amap_);
  c.cmd.rs2 = pack_rs(unsigned(rows & mask_bits(mvin_rows_bits_)), unsigned(cols & mask_bits(mvin_cols_bits_)), la.raw);
  c.valid = s.state != 0 && !(R.ld_util >= cfg_.rs_entries_ld) && r.dram_addr != 0;  // :93
  return c;
}

// LdB (:139-237)
LoopMatmul::SubCmd LoopMatmul::ldb_cmd(const Regs &R) const {
  const LdReq &r = R.ldb_req;
  const Sub &s = R.ldb;
  const uint64_t row_it = r.transpose ? s.j : s.k, col_it = r.transpose ? s.k : s.j;
  const uint64_t max_col = r.transpose ? r.max_k : r.max_j;
  const uint64_t col_pad = r.transpose ? r.pad_k : r.pad_j;
  const uint64_t max_blocks = std::min<uint64_t>(max_col, max_block_len_);
  const uint64_t sp_addr_start = r.is_resadd ? r.addr_end : r.addr_end - r.max_k * r.max_j * block_;  // :178
  const uint64_t dram_offset = (row_it * r.dram_stride + col_it) * block_ * (cfg_.array.in_bits / 8);
  const uint64_t sp_addr = sp_addr_start + (row_it * max_col + col_it) * block_;
  const uint64_t blocks = col_it + max_blocks <= max_col ? max_blocks : max_col - col_it;
  const uint64_t cols = blocks * block_ - (col_it + blocks >= max_col ? col_pad : 0);
  // :185 compares max_row_iterator with itself minus one: never true, so rows is always DIM
  const uint64_t rows = block_;
  SubCmd c;
  c.cmd.funct = LOAD2_CMD;
  c.cmd.rs1 = dram_add(r.dram_addr, dram_offset);
  const LocalAddr la = r.is_resadd ? LocalAddr::cast_acc(sp_addr, true, false, amap_) : LocalAddr::cast_sp(sp_addr, amap_);
  c.cmd.rs2 = pack_rs(unsigned(rows & mask_bits(mvin_rows_bits_)), unsigned(cols & mask_bits(mvin_cols_bits_)), la.raw);
  c.valid = s.state != 0 && !(R.ld_util >= cfg_.rs_entries_ld) && r.dram_addr != 0;  // :208
  return c;
}

// LdD (:253-332)
LoopMatmul::SubCmd LoopMatmul::ldd_cmd(const Regs &R) const {
  const LdReq &r = R.ldd_req;
  const Sub &s = R.ldd;
  const uint64_t max_blocks = r.low_d ? std::min<uint64_t>(r.max_j, max_block_len_)
                                      : std::min<uint64_t>(r.max_j, max_block_len_acc_);  // :274
  const uint64_t elem = r.low_d ? cfg_.array.in_bits / 8 : cfg_.array.acc_bits / 8;
  const uint64_t dram_offset = (s.i * r.dram_stride + s.j) * block_ * elem;
  const uint64_t sp_addr = r.addr_start + (s.i * r.max_j + s.j) * block_;
  const uint64_t blocks = s.j + max_blocks <= r.max_j ? max_blocks : r.max_j - s.j;
  const uint64_t cols = blocks * block_ - (s.j + blocks >= r.max_j ? r.pad_j : 0);
  const uint64_t rows = block_ - (s.i == r.max_i - 1 ? r.pad_i : 0);
  SubCmd c;
  c.cmd.funct = LOAD3_CMD;
  c.cmd.rs1 = dram_add(r.dram_addr, dram_offset);
  c.cmd.rs2 = pack_rs(unsigned(rows & mask_bits(mvin_rows_bits_)), unsigned(cols & mask_bits(mvin_cols_bits_)),
                      LocalAddr::cast_acc(sp_addr, false, false, amap_).raw);
  c.valid = s.state != 0 && !(R.ld_util >= cfg_.rs_entries_ld) && r.dram_addr != 0;  // :306
  return c;
}

// Execute (:352-495)
LoopMatmul::SubCmd LoopMatmul::ex_cmd(const Regs &R) const {
  const ExReq &r = R.ex_req;
  const Sub &s = R.ex;
  const uint64_t b_addr_start = r.b_addr_end - r.max_k * r.max_j * block_;  // :387
  const uint64_t a_row = r.a_tranpose ? s.k : s.i, a_col = r.a_tranpose ? s.i : s.k;
  const uint64_t b_row = r.b_tranpose ? s.j : s.k, b_col = r.b_tranpose ? s.k : s.j;
  const uint64_t a_max_col = r.a_tranpose ? r.max_i : r.max_k;
  const uint64_t b_max_col = r.b_tranpose ? r.max_k : r.max_j;
  const uint64_t a_addr = r.a_addr_start + (a_row * a_max_col + a_col) * block_;
  const uint64_t b_addr = b_addr_start + (b_row * b_max_col + b_col) * block_;
  const uint64_t c_addr = r.c_addr_start + (s.i * r.max_j + s.j) * block_;
  const uint64_t a_cols = block_ - (s.k == r.max_k - 1 ? r.pad_k : 0);
  const uint64_t a_rows = block_ - (s.i == r.max_i - 1 ? r.pad_i : 0);
  const uint64_t b_cols = block_ - (s.j == r.max_j - 1 ? r.pad_j : 0);
  const uint64_t b_rows = block_ - (s.k == r.max_k - 1 ? r.pad_k : 0);
  const uint64_t c_cols = block_ - (s.j == r.max_j - 1 ? r.pad_j : 0);
  const uint64_t c_rows = block_ - (s.i == r.max_i - 1 ? r.pad_i : 0);
  const unsigned rb = mvin_rows_bits_, cb = mvin_cols_bits_;  // PreloadRs / ComputeRs widths
  SubCmd c;
  if (s.state == 1) {  // pre (:412-430)
    c.cmd.funct = PRELOAD_CMD;
    const LocalAddr pre = s.i == 0 ? LocalAddr::cast_sp(b_addr, amap_) : LocalAddr::garbage(amap_);
    c.cmd.rs1 = pack_rs(unsigned(b_rows & mask_bits(rb)), unsigned(b_cols & mask_bits(cb)), pre.raw);
    c.cmd.rs2 = pack_rs(unsigned(c_rows & mask_bits(rb)), unsigned(c_cols & mask_bits(cb)),
                        LocalAddr::cast_acc(c_addr, r.accumulate || s.k != 0, false, amap_).raw);
  } else {  // comp (:432-449)
    c.cmd.funct = s.i == 0 ? COMPUTE_AND_FLIP_CMD : COMPUTE_AND_STAY_CMD;
    c.cmd.rs1 = pack_rs(unsigned(a_rows & mask_bits(rb)), unsigned(a_cols & mask_bits(cb)),
                        LocalAddr::cast_sp(a_addr, amap_).raw);
    c.cmd.rs2 = pack_rs(block_, block_, LocalAddr::garbage(amap_).raw);
  }
  // :458-463, the loads this loop's command depends on are ahead (:873)
  const bool lda_completed = R.lda_req.loop_id != r.loop_id || R.lda.idle();
  const bool ldb_completed = R.ldb_req.loop_id != r.loop_id || R.ldb.idle();
  const bool ldd_completed = R.ldd_req.loop_id != r.loop_id || R.ldd.idle();
  const bool lda_ahead = lda_completed || R.lda.k > s.k || (R.lda.k == s.k && R.lda.i > s.i);
  // (:461 compares lda's k, not ldb's, with the execute FSM's k)
  const bool ldb_ahead = ldb_completed || R.ldb.k > s.k || (R.lda.k == s.k && R.ldb.j > s.j);
  const bool ld_ahead = lda_ahead && ldb_ahead && ldd_completed;
  c.valid = s.state != 0 && !(R.ex_util >= cfg_.rs_entries_ex) && ld_ahead && !r.skip;
  return c;
}

// StC (:514-690); the LAYERNORM / SOFTMAX states are not modelled
LoopMatmul::SubCmd LoopMatmul::stc_cmd(const Regs &R) const {
  const StReq &r = R.st_req;
  const Sub &s = R.stc;
  const uint64_t max_blocks = r.full_c ? 1 : std::min<uint64_t>(r.max_j, max_block_len_);  // :542
  const uint64_t elem = r.full_c ? cfg_.array.acc_bits / 8 : cfg_.array.in_bits / 8;
  const uint64_t dram_offset = (s.i * r.dram_stride + s.j) * block_ * elem;
  const uint64_t sp_addr = r.addr_start + (s.i * r.max_j + s.j) * block_;
  const uint64_t blocks = s.j + max_blocks <= r.max_j ? max_blocks : r.max_j - s.j;
  const uint64_t cols = blocks * block_ - (s.j + blocks >= r.max_j ? r.pad_j : 0);
  const uint64_t rows = block_ - (s.i == r.max_i - 1 ? r.pad_i : 0);
  SubCmd c;
  c.cmd.funct = STORE_CMD;
  c.cmd.rs1 = dram_add(r.dram_addr, dram_offset);
  c.cmd.rs2 = pack_rs(unsigned(rows & mask_bits(mvin_rows_bits_)), unsigned(cols & mask_bits(mvin_cols_bits_)),
                      LocalAddr::cast_acc(sp_addr, false, r.full_c, amap_).raw);
  // :627-634 (the Execute FSM's iterators; ex_completed :881)
  const bool ex_completed = R.ex_req.loop_id != r.loop_id || R.ex.idle();
  bool ex_ahead;
  if (r.is_resadd) {
    // :889-895: with resadd, StC follows the B loads
    const bool done = (R.lda_req.loop_id != r.loop_id || R.lda.idle()) && (R.ldb_req.loop_id != r.loop_id || R.ldb.idle());
    const uint64_t ex_i = R.ldb.k, ex_j = R.ldb.j;
    ex_ahead = done || (ex_i > s.i || (ex_i == s.i && ex_j >= s.j + blocks));
  } else {
    ex_ahead = ex_completed || (r.act != ACT_LAYERNORM && r.act != ACT_SOFTMAX && R.ex.k == r.max_k - 1 &&
                                (R.ex.j >= s.j + blocks || (R.ex.j == s.j + blocks - 1 && R.ex.i > s.i)));
  }
  c.valid = s.state != 0 && !(R.st_util >= cfg_.rs_entries_st) && ex_ahead && r.dram_addr != 0;  // :636
  return c;
}

void LoopMatmul::settle() {
  Comb &c = c_;
  const Regs &R = r_;
  c = Comb();
  const LoopState &head = R.loops[R.head_loop_id];
  const unsigned tail_id = R.head_loop_id ^ 1;
  c.loop_configured = R.loops[0].configured || R.loops[1].configured;  // :792
  c.loop_being_configured_id = head.configured ? tail_id : R.head_loop_id;  // :794
  const bool cmd_valid = R.cmdq.deq_valid();
  const GemminiCmd &cmd = R.cmdq.deq_bits();
  c.is_loop_run_cmd = cmd.funct == LOOP_WS;  // :851-853
  c.is_loop_config_cmd = cmd.funct >= LOOP_WS_CONFIG_BOUNDS && cmd.funct <= LOOP_WS_CONFIG_STRIDES_DC;
  c.is_loop_cmd = c.is_loop_run_cmd || c.is_loop_config_cmd;
  // :866-870
  c.lda = lda_cmd(R);
  c.ldb = ldb_cmd(R);
  c.ex = ex_cmd(R);
  c.ldd = ldd_cmd(R);
  c.stc = stc_cmd(R);

  // the A/B arbiter (WeightedArbiter.scala:7-75, weightA 0 and static weights: :813-827)
  const bool same_loop = R.lda_req.loop_id == R.ldb_req.loop_id;
  const bool forceA0 = !same_loop && R.lda_req.loop_id == R.head_loop_id;
  const bool forceB0 = !same_loop && R.ldb_req.loop_id == R.head_loop_id;
  const bool forceA = R.is_resadd ? (same_loop && !R.lda.idle()) : forceA0;
  const bool forceB = R.is_resadd ? (forceB0 || R.lda.idle()) : forceB0;
  if (forceA)
    c.ab_choice = 0;
  else if (forceB)
    c.ab_choice = 1;
  else if (R.lda.idle())
    c.ab_choice = 1;
  else if (R.ldb.idle())
    c.ab_choice = 0;
  else if (R.lda.k > R.ldb.k || (R.ldb.k == 0 && R.ldb.j == 0))
    c.ab_choice = 1;
  else
    c.ab_choice = 0;
  const SubCmd &ab = c.ab_choice == 0 ? c.lda : c.ldb;
  // the global Arbiter(4): stC, ex, ldD, ldA/B (:830-835)
  c.arb_choice = arbiter_choice({c.stc.valid, c.ex.valid, c.ldd.valid, ab.valid});
  const bool arb_valid = c.arb_choice >= 0;

  out_ = Out();
  if (c.loop_configured) {  // :855-860
    out_.out_valid = arb_valid;
    if (arb_valid) {
      const SubCmd *src[4] = {&c.stc, &c.ex, &c.ldd, &ab};
      out_.out_bits = src[c.arb_choice]->cmd;
      // bookkeeping: the loop the emitting sub-FSM works for
      const unsigned lid = c.arb_choice == 0   ? R.st_req.loop_id
                           : c.arb_choice == 1 ? R.ex_req.loop_id
                           : c.arb_choice == 2 ? R.ldd_req.loop_id
                           : c.ab_choice == 0  ? R.lda_req.loop_id
                                               : R.ldb_req.loop_id;
      out_.out_bits.op = R.loops[lid].op;
      out_.out_bits.uop = -1;
      out_.out_bits.parent_uop = R.loops[lid].uop;
    }
    out_.out_bits.from_matmul_fsm = true;
    out_.out_bits.from_conv_fsm = false;
  } else {
    out_.out_valid = cmd_valid && !c.is_loop_config_cmd && !c.is_loop_run_cmd;
    out_.out_bits = cmd;
  }
  out_.out_bits.rob_valid = false;
  out_.busy = cmd_valid || c.loop_configured;  // :810
  out_.in_ready = R.cmdq.enq_ready();
}

void LoopMatmul::set_inputs(const In &in) {
  in_ = in;
  evaluated_ = false;
}

void LoopMatmul::eval() {
  const Comb &c = c_;
  const Regs &R = r_;
  n_ = r_;
  Regs &N = n_;
  const unsigned head_id = R.head_loop_id, tail_id = head_id ^ 1;
  const LoopState &head = R.loops[head_id];
  const bool arb_fire = c.arb_choice >= 0 && in_.out_ready;  // arb.io.out.ready := io.out.ready (:864)
  const bool cmd_valid = R.cmdq.deq_valid();
  const GemminiCmd &cmd = R.cmdq.deq_bits();
  // :862
  const bool cmd_ready = c.is_loop_cmd ? !R.loops[c.loop_being_configured_id].configured
                                       : (!c.loop_configured && in_.out_ready);
  deq_ = cmd_valid && cmd_ready;

  // which sub-FSM commands fire (the Arbiters' grants)
  const bool fire_stc = arb_fire && c.arb_choice == 0;
  const bool fire_ex = arb_fire && c.arb_choice == 1;
  const bool fire_ldd = arb_fire && c.arb_choice == 2;
  const bool fire_ab = arb_fire && c.arb_choice == 3;
  const bool fire_lda = fire_ab && c.ab_choice == 0;
  const bool fire_ldb = fire_ab && c.ab_choice == 1;

  // request channels to the sub-FSMs (:958-1082)
  const unsigned lda_id = head.lda_started ? tail_id : head_id;
  const unsigned ldb_id = head.ldb_started ? tail_id : head_id;
  const unsigned ex_id = head.ex_started ? tail_id : head_id;
  const unsigned ldd_id = head.ldd_started ? tail_id : head_id;
  const unsigned st_id = head.st_started ? tail_id : head_id;
  const LoopState &la = R.loops[lda_id], &lb = R.loops[ldb_id], &le = R.loops[ex_id], &ld = R.loops[ldd_id],
                  &ls = R.loops[st_id];
  const uint64_t half = max_addr_ / 2;
  const bool lda_req_valid = !la.lda_started && la.configured;
  const bool ldb_req_valid = !lb.ldb_started && lb.configured;
  const bool ex_req_valid = !le.ex_started && le.lda_started && le.ldb_started && le.ldd_started && le.configured;
  const bool ldd_req_valid = !ld.ldd_started && ld.configured;
  const bool st_req_valid = R.is_resadd ? (!ls.st_started && ls.configured)
                                        : (!ls.st_started && ls.ex_started && ls.configured);
  const bool lda_req_fire = lda_req_valid && R.lda.idle();
  const bool ldb_req_fire = ldb_req_valid && R.ldb.idle();
  const bool ex_req_fire = ex_req_valid && R.ex.idle();
  const bool ldd_req_fire = ldd_req_valid && R.ldd.idle();
  const bool st_req_fire = st_req_valid && R.stc.idle();

  // ---- sub-FSM registers
  {  // LdA :98-121
    Sub &n = N.lda;
    const LdReq &r = R.lda_req;
    if (r.dram_addr == 0) {
      n.state = 0;
    } else if (fire_lda) {
      const uint64_t max_col_dim = r.transpose ? r.max_i : r.max_k;
      const uint64_t max_blocks = std::min<uint64_t>(max_col_dim, max_block_len_);
      const uint64_t i_blocks = r.transpose ? max_blocks : 1, k_blocks = r.transpose ? 1 : max_blocks;
      const uint64_t ni = floor_add(R.lda.i, i_blocks, r.max_i, kIter);
      const uint64_t nk = floor_add(R.lda.k, k_blocks, r.max_k, kIter, ni == 0);
      n.i = ni;
      n.k = nk;
      if (ni == 0 && nk == 0)
        n.state = 0;
    }
    if (lda_req_fire) {
      LdReq q;
      q.max_k = R.is_resadd ? la.max_j : la.max_k;
      q.max_i = la.max_i;
      q.pad_k = R.is_resadd ? la.pad_j : la.pad_k;
      q.pad_i = la.pad_i;
      q.dram_addr = la.a_dram_addr;
      q.dram_stride = la.a_dram_stride;
      q.transpose = la.a_transpose;
      q.addr_start = R.is_resadd ? la.resadd_addr_start
                                 : (la.a_ex_spad_id == 0 ? la.a_addr_start : (la.a_ex_spad_id - 1) * half);
      q.loop_id = lda_id;
      q.is_resadd = R.is_resadd;
      N.lda_req = q;
      n.state = 1;
      n.i = n.k = 0;
    }
  }
  {  // LdB :213-236
    Sub &n = N.ldb;
    const LdReq &r = R.ldb_req;
    if (r.dram_addr == 0) {
      n.state = 0;
    } else if (fire_ldb) {
      const uint64_t max_col_dim = r.transpose ? r.max_k : r.max_j;
      const uint64_t max_blocks = std::min<uint64_t>(max_col_dim, max_block_len_);
      const uint64_t j_blocks = r.transpose ? 1 : max_blocks, k_blocks = r.transpose ? max_blocks : 1;
      const uint64_t nj = floor_add(R.ldb.j, j_blocks, r.max_j, kIter);
      const uint64_t nk = floor_add(R.ldb.k, k_blocks, r.max_k, kIter, nj == 0);
      n.j = nj;
      n.k = nk;
      if (nj == 0 && nk == 0)
        n.state = 0;
    }
    if (ldb_req_fire) {
      LdReq q;
      q.max_j = lb.max_j;
      q.max_k = R.is_resadd ? lb.max_i : lb.max_k;
      q.pad_j = lb.pad_j;
      q.pad_k = R.is_resadd ? lb.pad_i : lb.pad_k;
      q.dram_addr = lb.b_dram_addr;
      q.dram_stride = lb.b_dram_stride;
      q.transpose = lb.b_transpose;
      q.addr_end = R.is_resadd ? lb.resadd_addr_start : (lb.b_ex_spad_id == 0 ? lb.b_addr_end : lb.b_ex_spad_id * half);
      q.loop_id = ldb_id;
      q.is_resadd = R.is_resadd;
      N.ldb_req = q;
      n.state = 1;
      n.j = n.k = 0;
    }
  }
  {  // LdD :311-331
    Sub &n = N.ldd;
    const LdReq &r = R.ldd_req;
    if (r.dram_addr == 0) {
      n.state = 0;
    } else if (fire_ldd) {
      const uint64_t max_blocks = r.low_d ? std::min<uint64_t>(r.max_j, max_block_len_)
                                          : std::min<uint64_t>(r.max_j, max_block_len_acc_);
      const uint64_t ni = floor_add(R.ldd.i, 1, r.max_i, kIter);
      const uint64_t nj = floor_add(R.ldd.j, max_blocks, r.max_j, kIter, ni == 0);
      n.i = ni;
      n.j = nj;
      if (ni == 0 && nj == 0)
        n.state = 0;
    }
    if (ldd_req_fire) {
      LdReq q;
      q.max_j = ld.max_j;
      q.max_i = ld.max_i;
      q.pad_j = ld.pad_j;
      q.pad_i = ld.pad_i;
      q.dram_addr = ld.d_dram_addr;
      q.dram_stride = ld.d_dram_stride;
      q.low_d = ld.low_d;
      q.addr_start = R.ld_d_addr_start;
      q.loop_id = ldd_id;
      N.ldd_req = q;
      n.state = 1;
      n.j = n.i = 0;
    }
  }
  {  // Execute :468-492
    Sub &n = N.ex;
    const ExReq &r = R.ex_req;
    if (r.skip) {
      n.state = 0;
    } else if (fire_ex) {
      if (R.ex.state == 1) {
        n.state = 2;
      } else {
        const uint64_t ni = floor_add(R.ex.i, 1, r.max_i, kIter);
        const uint64_t nj = floor_add(R.ex.j, 1, r.max_j, kIter, ni == 0);
        const uint64_t nk = floor_add(R.ex.k, 1, r.max_k, kIter, nj == 0 && ni == 0);
        n.i = ni;
        n.j = nj;
        n.k = nk;
        n.state = (nk == 0 && nj == 0 && ni == 0) ? 0 : 1;
      }
    }
    if (ex_req_fire) {
      ExReq q;
      q.max_j = le.max_j;
      q.max_k = le.max_k;
      q.max_i = le.max_i;
      q.pad_j = le.pad_j;
      q.pad_k = le.pad_k;
      q.pad_i = le.pad_i;
      q.accumulate = le.ex_accumulate;
      q.a_addr_start = le.a_ex_spad_id == 0 ? le.a_addr_start : (le.a_ex_spad_id - 1) * half;
      q.b_addr_end = le.b_ex_spad_id == 0 ? le.b_addr_end : le.b_ex_spad_id * half;
      q.a_tranpose = le.a_transpose;
      q.b_tranpose = le.b_transpose;
      q.c_addr_start = R.ex_c_addr_start;
      q.loop_id = ex_id;
      q.skip = R.is_resadd;
      N.ex_req = q;
      n.state = 1;
      n.i = n.j = n.k = 0;
    }
  }
  {  // StC :644-689 (the st state only)
    Sub &n = N.stc;
    const StReq &r = R.st_req;
    if (r.dram_addr == 0) {
      n.state = 0;
    } else if (fire_stc && R.stc.state == 1) {
      const uint64_t max_blocks = r.full_c ? 1 : std::min<uint64_t>(r.max_j, max_block_len_);
      const uint64_t ni = floor_add(R.stc.i, 1, r.max_i, kIter);
      const uint64_t nj = floor_add(R.stc.j, max_blocks, r.max_j, kIter, ni == 0);
      n.i = ni;
      n.j = nj;
      if (ni == 0 && nj == 0)
        n.state = 0;
    }
    if (st_req_fire) {
      StReq q;
      q.max_k = R.is_resadd ? 1 : ls.max_k;
      q.max_j = ls.max_j;
      q.max_i = ls.max_i;
      q.pad_j = ls.pad_j;
      q.pad_i = ls.pad_i;
      q.dram_addr = ls.c_dram_addr;
      q.dram_stride = ls.c_dram_stride;
      q.full_c = ls.full_c;
      q.act = ls.act;
      q.addr_start = R.is_resadd ? ls.resadd_addr_start : R.st_c_addr_start;
      q.loop_id = st_id;
      q.is_resadd = R.is_resadd;
      N.st_req = q;
      n.state = 1;  // LAYERNORM / SOFTMAX would start in ln_config (not modelled)
      n.i = n.j = 0;
    }
  }

  // ---- reservation-station occupancy (:842-844)
  N.ld_util = R.ld_util + ((fire_lda || fire_ldb || fire_ldd) ? 1 : 0) - in_.ld_completed;
  N.st_util = R.st_util + (fire_stc ? 1 : 0) - in_.st_completed;
  N.ex_util = R.ex_util + (fire_ex ? 1 : 0) - in_.ex_completed;

  // ---- configuration registers (:901-951)
  LoopState &lbc = N.loops[c.loop_being_configured_id];
  if (cmd_valid && c.is_loop_cmd && !R.loops[c.loop_being_configured_id].configured) {
    const uint64_t m16 = mask_bits(kIter), ma = mask_bits(cfg_.core_max_addr_bits);
    switch (cmd.funct) {
      case LOOP_WS_CONFIG_BOUNDS:
        lbc.max_k = (cmd.rs2 >> 32) & m16;
        lbc.max_j = (cmd.rs2 >> 16) & m16;
        lbc.max_i = cmd.rs2 & m16;
        lbc.pad_k = (cmd.rs1 >> 32) & m16;
        lbc.pad_j = (cmd.rs1 >> 16) & m16;
        lbc.pad_i = cmd.rs1 & m16;
        break;
      case LOOP_WS_CONFIG_ADDRS_AB:
        lbc.a_dram_addr = cmd.rs1 & ma;
        lbc.b_dram_addr = cmd.rs2 & ma;
        break;
      case LOOP_WS_CONFIG_ADDRS_DC:
        lbc.d_dram_addr = cmd.rs1 & ma;
        lbc.c_dram_addr = cmd.rs2 & ma;
        break;
      case LOOP_WS_CONFIG_STRIDES_AB:
        lbc.a_dram_stride = cmd.rs1 & ma;
        lbc.b_dram_stride = cmd.rs2 & ma;
        break;
      case LOOP_WS_CONFIG_STRIDES_DC:
        lbc.d_dram_stride = cmd.rs1 & ma;
        lbc.c_dram_stride = cmd.rs2 & ma;
        break;
      case LOOP_WS:
        lbc.ex_accumulate = cmd.rs1 & 1;
        lbc.full_c = (cmd.rs1 >> 1) & 1;
        lbc.low_d = (cmd.rs1 >> 2) & 1;
        lbc.act = unsigned((cmd.rs1 >> 8) & 7);  // Activation.bitwidth = 3
        lbc.a_ex_spad_id = unsigned((cmd.rs1 >> 18) & 3);
        lbc.b_ex_spad_id = unsigned((cmd.rs1 >> 16) & 3);
        lbc.a_transpose = cmd.rs2 & 1;
        lbc.b_transpose = (cmd.rs2 >> 1) & 1;
        N.is_resadd = (cmd.rs2 >> 2) & 1;
        lbc.configured = true;
        lbc.op = cmd.op;
        lbc.uop = cmd.uop;
        break;
      default:
        break;
    }
  }

  // ---- request handshakes (:973-1075)
  if (lda_req_fire) {
    N.loops[lda_id].running = true;
    N.loops[lda_id].lda_started = true;
  }
  if (ldb_req_fire) {
    N.loops[ldb_id].running = true;
    N.loops[ldb_id].ldb_started = true;
  }
  if (ex_req_fire) {
    N.loops[ex_id].running = true;
    N.loops[ex_id].ex_started = true;
    if (R.loops[ex_id].c_dram_addr != 0)
      N.ex_c_addr_start = floor_add(R.ex_c_addr_start, max_acc_addr_ / 2, max_acc_addr_, 16);
  }
  if (ldd_req_fire) {
    N.loops[ldd_id].running = true;
    N.loops[ldd_id].ldd_started = true;
    if (R.loops[ldd_id].c_dram_addr != 0)
      N.ld_d_addr_start = floor_add(R.ld_d_addr_start, max_acc_addr_ / 2, max_acc_addr_, 16);
  }
  if (st_req_fire) {
    N.loops[st_id].running = true;
    N.loops[st_id].st_started = true;
    if (R.loops[st_id].c_dram_addr != 0)
      N.st_c_addr_start = floor_add(R.st_c_addr_start, max_acc_addr_ / 2, max_acc_addr_, 16);
  }

  // ---- completion (:1084-1102), from the sub-FSMs' state before this edge
  auto done = [&](const Sub &s, unsigned id, bool LoopState::*started, bool LoopState::*completed) {
    if (s.idle() && R.loops[id].running && R.loops[id].*started)
      N.loops[id].*completed = true;
  };
  done(R.lda, R.lda_req.loop_id, &LoopState::lda_started, &LoopState::lda_completed);
  done(R.ldb, R.ldb_req.loop_id, &LoopState::ldb_started, &LoopState::ldb_completed);
  done(R.ex, R.ex_req.loop_id, &LoopState::ex_started, &LoopState::ex_completed);
  done(R.ldd, R.ldd_req.loop_id, &LoopState::ldd_started, &LoopState::ldd_completed);
  done(R.stc, R.st_req.loop_id, &LoopState::st_started, &LoopState::st_completed);

  // ---- retire the head loop (:1104-1107)
  if (head.running && head.all_completed()) {
    N.loops[head_id].reset();
    N.head_loop_id = tail_id;
  }

  // ---- the input queue
  N.cmdq.tick(in_.in_valid && R.cmdq.enq_ready(), in_.in_bits, deq_);
  evaluated_ = true;
}

void LoopMatmul::tick() {
  if (!evaluated_)
    eval();
  std::swap(r_, n_);
  settle();
  evaluated_ = false;
}

}  // namespace systolique
