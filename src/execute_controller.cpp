// ExecuteController (ExecuteController.scala) and TransposePreloadUnroller
// (TransposePreloadUnroller.scala), Gemmini v0.7.2. Line numbers are ExecuteController.scala's
// unless another file is named. Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and
// NOTICE).
#include "systolique/execute_controller.h"

#include <algorithm>
#include <stdexcept>

namespace systolique {

namespace {
enum State : unsigned { WAITING = 0, COMPUTE = 1, FLUSH = 2, FLUSHING = 3 };  // :73-74

int64_t clip(int64_t v, unsigned bits) {
  const int64_t hi = (int64_t(1) << (bits - 1)) - 1, lo = -(int64_t(1) << (bits - 1));
  return v > hi ? hi : v < lo ? lo : v;
}

}  // namespace

ExecuteController::ExecuteController(const FrontendConfig &cfg, ArrayOptions opt)
    : cfg_(cfg), amap_(cfg.addr_map()), block_(cfg.dim()), rows_bits_(log2_up(cfg.dim() + 1)),
      cmd_q_(cfg.ex_queue_length, 3), cntl_q_(cfg.spad_read_delay + 1, /*pipe=*/true) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("ExecuteController: " + e);
  if (opt.interface != Interface::MeshWithDelays)
    throw std::invalid_argument("ExecuteController: the array needs the MeshWithDelays interface");
  array_ = std::make_unique<SystolicArray>(cfg.array, opt);
  // The RTL's reset: MeshWithDelays' registers are 0 after it as before; four cycles with reset
  // high so that cycle 0 is the first cycle after reset, as for the RTL tops.
  array_->reset(4);
  for (auto &t : tags_)
    t.addr = LocalAddr::garbage(amap_);
  uop_of_rob_.fill(-1);
  in_.sp_resp.assign(cfg.sp_banks, SpReadResp());
  in_.sp_req_ready.assign(cfg.sp_banks, false);
  in_.acc_ready.assign(cfg.acc_banks, AccumulatorBank::ReadyView());
}

ExecuteController::~ExecuteController() = default;

ExecuteController::Tag ExecuteController::tag_of(unsigned valid, unsigned id) const {
  if (!valid) {  // make_this_garbage (:58-61); rows / cols are DontCare, 0 in the RTL
    Tag t;
    t.addr = LocalAddr::garbage(amap_);
    return t;
  }
  return tags_[id % tags_.size()];
}

int64_t ExecuteController::head_uop() const { return cmd_q_.valid(0) ? cmd_q_.bits(0).uop : -1; }

bool ExecuteController::idle() const {
  if (tpu_q_.len() || cmd_q_.len() || tpu_state_ || state_ != WAITING || !cntl_q_.empty() ||
      pending_[0].valid || pending_[1].valid || a_fire_started_ || b_fire_started_ || d_fire_started_)
    return false;
  const MwdOut &mo = array_->out();
  for (unsigned k = 0; k < mo.tags_valid.size(); ++k)  // matmul_in_progress (:230)
    if (tag_of(mo.tags_valid[k], mo.tags_id[k]).rob_valid)
      return false;
  if (array_->mesh_with_delays().req_valid())
    return false;
  for (const RequestInfo &r : array_->requests())  // every row that entered has left
    if (r.last_in < 0 || r.rows_out < r.rows_in)
      return false;
  return true;
}

// ------------------------------------------------------------------ mesh feed and write-back
ExecuteController::Deq ExecuteController::deq(const std::vector<SpReadResp> &sp_resp) const {
  Deq q;
  const unsigned dim = block_;
  q.mo = array_->out();
  const MwdOut &mo = q.mo;
  q.wb.sp_resp_ready.assign(cfg_.sp_banks, false);
  q.wb.sp_write.assign(cfg_.sp_banks, SpWrite());
  q.wb.acc_write.assign(cfg_.acc_banks, AccWrite());
  q.mi.resize(cfg_.array);

  // ---- the control-signal queue's head feeds the mesh (:811-899)
  q.cntl_valid = cntl_q_.deq_valid();
  q.head = cntl_q_.deq_bits();
  const Cntl &cn = q.head;
  auto read_valid = [&](unsigned b) { return b < sp_resp.size() && sp_resp[b].valid; };
  // The accumulator's responses never arrive (no normalizer / AccumulatorScale in the model,
  // as in ExecuteTop): an operand from the accumulator is never valid.
  const bool dataA_valid = cn.a_garbage || cn.a_unpadded_cols == 0 ||
                           (cn.a_read_from_acc ? false : read_valid(cn.a_bank));
  const bool dataB_valid = cn.b_garbage || cn.b_unpadded_cols == 0 ||
                           (cn.accumulate_zeros ? false : cn.b_read_from_acc ? false : read_valid(cn.b_bank));
  const bool dataD_valid = cn.d_garbage || cn.d_unpadded_cols == 0 ||
                           (cn.preload_zeros ? false : cn.d_read_from_acc ? false : read_valid(cn.d_bank));
  MwdIn &mi = q.mi;
  mi.a_valid = q.cntl_valid && cn.a_fire && dataA_valid;
  mi.b_valid = q.cntl_valid && cn.b_fire && dataB_valid;
  mi.d_valid = q.cntl_valid && cn.d_fire && dataD_valid;
  q.mesh_a_fire = mi.a_valid && mo.a_ready;
  q.mesh_b_fire = mi.b_valid && mo.b_ready;
  q.mesh_d_fire = mi.d_valid && mo.d_ready;
  q.cntl_deq_ready = (!cn.a_fire || q.mesh_a_fire || !mo.a_ready) &&
                     (!cn.b_fire || q.mesh_b_fire || !mo.b_ready) &&
                     (!cn.d_fire || q.mesh_d_fire || !mo.d_ready) && (!cn.first || mo.req_ready);
  q.cntl_deq_fire = q.cntl_valid && q.cntl_deq_ready;

  // data (:832-838, 879-899)
  static const SpRow zero{};
  auto row_of = [&](unsigned bank) -> const SpRow & { return bank < sp_resp.size() ? sp_resp[bank].data : zero; };
  const SpRow &a_raw = row_of(cn.a_bank);
  const SpRow &b_raw = cn.accumulate_zeros ? zero : row_of(cn.b_bank);
  const SpRow &d_raw = cn.preload_zeros ? zero : row_of(cn.d_bank);
  const bool a_sbfit = current_dataflow_ == kOS ? !a_transpose_ : a_transpose_;  // :123
  const bool b_sbfit = current_dataflow_ == kOS && bd_transpose_;                // :126
  bool a_zero = false, b_zero = false;
  if (q.cntl_valid && cn.perform_single_preload) {  // :890-893
    a_zero = !a_sbfit;
    b_zero = !b_sbfit;
  }
  if (q.cntl_valid && cn.perform_single_mul) {  // :895-899
    a_zero = a_sbfit;
    b_zero = b_sbfit;
  }
  for (unsigned k = 0; k < dim; ++k) {
    mi.a[k] = (a_zero || k >= cn.a_unpadded_cols) ? 0 : a_raw[k];
    mi.b[k] = (b_zero || k >= cn.b_unpadded_cols) ? 0 : b_raw[k];
    mi.d[k] = k >= cn.d_unpadded_cols ? 0 : d_raw[k];
  }
  // the request (:192-207, 883-898)
  mi.req_valid = q.cntl_valid ? (q.cntl_deq_fire && (cn.a_fire || cn.b_fire || cn.d_fire)) : state_ == FLUSH;
  mi.req.flush = (state_ == FLUSH && !q.cntl_valid) ? 1 : 0;
  mi.req.propagate = state_ == FLUSH ? in_prop_flush_ : cn.prop;
  mi.req.dataflow = cn.dataflow;
  mi.req.shift = cn.shift;
  mi.req.a_transpose = cn.a_transpose;
  mi.req.bd_transpose = cn.bd_transpose;
  mi.req.total_rows = q.cntl_valid ? cn.total_rows : block_;
  // The tag travels as an index into tags_ (written at the request's handshake); its valid bit
  // is the TagQueue entry's (make_this_garbage clears it, TagQueue.scala:37, 48).
  mi.req.tag_valid = 1;
  mi.req.tag_id = next_tag_;
  q.mesh_req_fire = mi.req_valid && mo.req_ready;

  // pop the responses that were fed (:841-865)
  if (q.cntl_deq_fire) {
    if (cn.a_fire && q.mesh_a_fire && !cn.a_garbage && cn.a_unpadded_cols > 0 && !cn.a_read_from_acc)
      q.wb.sp_resp_ready[cn.a_bank] = true;
    if (cn.b_fire && q.mesh_b_fire && !cn.b_garbage && !cn.accumulate_zeros && cn.b_unpadded_cols > 0 &&
        !cn.b_read_from_acc)
      q.wb.sp_resp_ready[cn.b_bank] = true;
    if (cn.d_fire && q.mesh_d_fire && !cn.d_garbage && !cn.preload_zeros && cn.d_unpadded_cols > 0 &&
        !cn.d_read_from_acc)
      q.wb.sp_resp_ready[cn.d_bank] = true;
  }

  // ---- write-back of the array's results (:903-951)
  q.tag = tag_of(mo.resp_tag_valid, mo.resp_tag_id);
  const Tag &tag = q.tag;
  const unsigned w_total = mo.resp_total_rows;
  const uint64_t step = uint64_t(output_counter_) * c_addr_stride_;
  const LocalAddr w_address = current_dataflow_ == kWS
                                  ? tag.addr.plus(step)
                                  : tag.addr.plus((uint64_t(w_total) - 1 - step) & mask_bits(20));
  const bool write_to_acc = w_address.is_acc();
  const unsigned w_bank = write_to_acc ? w_address.acc_bank() : w_address.sp_bank();
  const uint32_t w_row = write_to_acc ? w_address.acc_row() : w_address.sp_row();
  const bool is_garbage_addr = tag.addr.is_garbage();
  const bool write_this_row = current_dataflow_ == kWS
                                  ? output_counter_ < tag.rows
                                  : ((uint64_t(w_total) - 1 - output_counter_) & mask_bits(rows_bits_)) < tag.rows;
  q.resp_fire_rob = mo.resp_valid && tag.rob_valid;
  const bool start_array_outputting = q.resp_fire_rob && !is_garbage_addr;
  SpRow sp_data{};
  AccRow acc_data{};
  uint64_t sp_mask = 0, acc_mask = 0;
  for (unsigned k = 0; k < dim; ++k) {
    int64_t e = clip(mo.resp_data[k], cfg_.array.in_bits);  // clippedToWidthOf (:926)
    if (activation_ == ACT_RELU && e < 0)
      e = 0;
    sp_data[k] = int8_t(e);
    acc_data[k] = int32_t(mo.resp_data[k]);  // withWidthOf(accType): sign-extended
    if (k < tag.cols) {
      sp_mask |= 1ull << k;
      acc_mask |= 0xfull << (4 * k);
    }
  }
  for (unsigned i = 0; i < cfg_.sp_banks; ++i) {
    SpWrite &w = q.wb.sp_write[i];
    w.en = start_array_outputting && w_bank == i && !write_to_acc && !is_garbage_addr && write_this_row;
    w.addr = w_row & uint32_t(mask_bits(amap_.sp_bank_row_bits));
    w.data = sp_data;
    w.mask = sp_mask;
  }
  for (unsigned i = 0; i < cfg_.acc_banks; ++i) {
    AccWrite &w = q.wb.acc_write[i];
    w.valid = start_array_outputting && w_bank == i && write_to_acc && !is_garbage_addr && write_this_row;
    w.addr = w_row & uint32_t(mask_bits(amap_.acc_bank_row_bits));
    w.acc = w_address.accumulate();
    w.data = acc_data;
    w.mask = acc_mask;
  }
  q.mesh_completed_fire = q.resp_fire_rob && mo.resp_last;

  bool in_progress = false;  // matmul_in_progress (:230)
  for (unsigned k = 0; k < mo.tags_valid.size(); ++k)
    in_progress |= tag_of(mo.tags_valid[k], mo.tags_id[k]).rob_valid;
  q.in_progress = in_progress;
  q.busy = cmd_q_.valid(0) || in_progress;  // :232
  return q;
}

ExecuteController::Writeback ExecuteController::writeback(const std::vector<SpReadResp> &sp_resp) const {
  return deq(sp_resp).wb;
}

// ------------------------------------------------------------------ command FSM and operand reads
void ExecuteController::issue(const Deq &q, Decision &d, Out &o) const {
  d = Decision();
  const unsigned dim = block_;
  const MwdOut &mo = q.mo;
  auto funct = [&](unsigned i) { return cmd_q_.bits(i).funct; };
  auto rs1 = [&](unsigned i) { return cmd_q_.bits(i).rs1; };
  auto rs2 = [&](unsigned i) { return cmd_q_.bits(i).rs2; };
  const bool valid0 = cmd_q_.valid(0), valid1 = cmd_q_.valid(1), valid2 = cmd_q_.valid(2);
  const bool DoConfig = funct(0) == CONFIG_CMD;  // :83-85
  auto is_compute = [&](unsigned i) { return funct(i) == COMPUTE_AND_FLIP_CMD || funct(i) == COMPUTE_AND_STAY_CMD; };
  auto is_preload = [&](unsigned i) { return funct(i) == PRELOAD_CMD; };
  const unsigned preload_cmd_place = is_preload(0) ? 0 : 1;   // :87
  const unsigned in_prop = funct(0) == COMPUTE_AND_FLIP_CMD;  // :90
  const bool a_sbfit = current_dataflow_ == kOS ? !a_transpose_ : a_transpose_;  // :123
  const unsigned a_address_place = preload_cmd_place == 0 ? 1 : (a_sbfit ? 2 : 0);
  const bool b_sbfit = current_dataflow_ == kOS && bd_transpose_;
  const unsigned b_address_place = preload_cmd_place == 0 ? 1 : (b_sbfit ? 2 : 0);
  const bool d_sbfit = current_dataflow_ == kWS && bd_transpose_;

  const LocalAddr a_address_rs1(rs1(a_address_place), amap_), b_address_rs2(rs2(b_address_place), amap_);
  const LocalAddr d_address_rs1(rs1(preload_cmd_place), amap_), c_address_rs2(rs2(preload_cmd_place), amap_);
  const bool multiply_garbage = a_address_rs1.is_garbage();
  const bool accumulate_zeros = b_address_rs2.is_garbage();
  const bool preload_zeros = d_address_rs1.is_garbage();
  const unsigned rb = rows_bits_;
  const unsigned a_cols_d = rs_cols(rs1(a_address_place), rb), a_rows_d = rs_rows(rs1(a_address_place), rb);
  const unsigned b_cols_d = rs_cols(rs2(b_address_place), rb), b_rows_d = rs_rows(rs2(b_address_place), rb);
  const unsigned d_cols_d = rs_cols(rs1(preload_cmd_place), rb), d_rows_d = rs_rows(rs1(preload_cmd_place), rb);
  const unsigned a_cols = a_transpose_ ? a_rows_d : a_cols_d, a_rows = a_transpose_ ? a_cols_d : a_rows_d;
  const bool bt = current_dataflow_ == kOS && bd_transpose_, dt = current_dataflow_ == kWS && bd_transpose_;
  const unsigned b_cols = bt ? b_rows_d : b_cols_d, b_rows = bt ? b_cols_d : b_rows_d;
  const unsigned d_cols = dt ? d_rows_d : d_cols_d, d_rows = dt ? d_cols_d : d_rows_d;
  const unsigned c_cols = rs_cols(rs2(preload_cmd_place), rb), c_rows = rs_rows(rs2(preload_cmd_place), rb);

  // RAW hazards against the tags in the array (:210-230)
  bool raw_hazard_pre = false, raw_hazard_mulpre = false, matmul_in_progress = false;
  for (unsigned k = 0; k < mo.tags_valid.size(); ++k) {
    const Tag t = tag_of(mo.tags_valid[k], mo.tags_id[k]);
    matmul_in_progress |= t.rob_valid;
    if (t.addr.is_garbage())
      continue;
    auto same = [&](uint64_t v) { return t.addr.is_same_address(LocalAddr(v, amap_)); };
    raw_hazard_pre |= same(rs1(0)) || same(rs1(1)) || same(rs2(1));
    raw_hazard_mulpre |= same(rs1(1)) || same(rs1(2)) || same(rs2(2));
  }
  const bool any_pending = pending_[0].valid || pending_[1].valid;

  // ---- the FSM (:532-693): which operands start inputting this cycle
  d.performing_single_preload = perform_single_preload_ && state_ == COMPUTE;  // :281-283
  d.performing_single_mul = perform_single_mul_ && state_ == COMPUTE;
  d.performing_mul_pre = perform_mul_pre_ && state_ == COMPUTE;
  d.next_state = state_;
  switch (state_) {
    case WAITING:
      d.clear_performs = true;
      if (valid0) {
        if (DoConfig && !matmul_in_progress && !any_pending) {
          d.do_config = true;
          d.pop = 1;
        } else if (is_preload(0) && valid1 && !raw_hazard_pre) {
          d.set_perform_single_preload = d.performing_single_preload = true;
          d.start_a = a_sbfit;
          d.start_b = b_sbfit;
          d.start_d = true;
          d.next_state = COMPUTE;
        } else if (is_compute(0) && valid1 && is_preload(1) && valid2 && !raw_hazard_mulpre) {
          // third_instruction_needed is true: RAW hazards are possible (:210, 228)
          d.set_perform_mul_pre = d.performing_mul_pre = true;
          d.start_a = d.start_b = d.start_d = true;
          d.next_state = COMPUTE;
        } else if (is_compute(0)) {
          d.set_perform_single_mul = d.performing_single_mul = true;
          d.start_a = !a_sbfit;
          d.start_b = !b_sbfit;
          d.next_state = COMPUTE;
        } else if (matmul_in_progress && (current_dataflow_ == kOS || DoConfig)) {
          d.next_state = FLUSH;
        }
        d.hazard_stall = (is_preload(0) && valid1 && raw_hazard_pre) ||
                         (is_compute(0) && valid1 && is_preload(1) && valid2 && raw_hazard_mulpre);
      } else if (matmul_in_progress && current_dataflow_ == kOS) {
        d.next_state = FLUSH;
      }
      break;
    case COMPUTE:
      if (perform_single_preload_) {
        d.start_a = a_sbfit;
        d.start_b = b_sbfit;
        d.start_d = true;
      } else if (perform_mul_pre_) {
        d.start_a = d.start_b = d.start_d = true;
      } else if (perform_single_mul_) {
        d.start_a = !a_sbfit;
        d.start_b = !b_sbfit;
      }
      break;
    case FLUSH:
      if (q.mesh_req_fire)
        d.next_state = FLUSHING;
      break;
    case FLUSHING:
      if (mo.req_ready)
        d.next_state = WAITING;
      break;
  }

  // ---- operands (:244-386)
  const LocalAddr a_address = a_address_rs1.plus(a_addr_offset_);
  const LocalAddr b_address = b_address_rs2.plus(b_fire_counter_);
  const LocalAddr d_address = d_address_rs1.plus(dim - 1 - d_fire_counter_);
  const bool a_read_from_acc = a_address_rs1.is_acc(), b_read_from_acc = b_address_rs2.is_acc(),
             d_read_from_acc = d_address_rs1.is_acc();
  const bool a_garbage = a_address_rs1.is_garbage() || !d.start_a;
  const bool b_garbage = b_address_rs2.is_garbage() || !d.start_b;
  const bool d_garbage = d_address_rs1.is_garbage() || !d.start_d;
  unsigned total_rows = dim;  // :285-302
  if (current_dataflow_ == kWS && d_garbage && !a_sbfit && !b_sbfit && !d_sbfit) {
    const unsigned rows_a = a_garbage ? 1 : a_rows, rows_b = b_garbage ? 1 : b_rows;
    total_rows = std::max(std::max(rows_a, rows_b), 4u);
  }
  d.total_rows = total_rows;
  const bool a_row_nz = a_fire_counter_ < a_rows, b_row_nz = b_fire_counter_ < b_rows;  // :310-312
  const bool d_row_nz = dim - 1 - d_fire_counter_ < d_rows;

  struct Operand {
    LocalAddr addr;
    bool is_garbage, start;
    unsigned counter;
    bool started;
    int priority;
  };
  const Operand ops[3] = {
      {a_address, a_address_rs1.is_garbage(), d.start_a, a_fire_counter_, a_fire_started_, 0},
      {b_address, b_address_rs2.is_garbage(), d.start_b, b_fire_counter_, b_fire_started_, 1},
      {d_address, d_address_rs1.is_garbage(), d.start_d, d_fire_counter_, d_fire_started_, 2}};
  auto same_bank = [](const Operand &x, const Operand &y) {  // :316-327
    const bool g = x.is_garbage || y.is_garbage || !x.start || !y.start;
    const bool ax = x.addr.is_acc(), ay = y.addr.is_acc();
    return !g && ((ax && ay) || (!ax && !ay && x.addr.sp_bank() == y.addr.sp_bank()));
  };
  bool op_valid[3];
  for (int x = 0; x < 3; ++x) {  // :341-357
    bool wait = false;
    for (int y = 0; y < 3; ++y) {
      if (y == x)
        continue;
      const bool sb = same_bank(ops[x], ops[y]);
      const bool sc = ops[x].started == ops[y].started && ops[x].counter == ops[y].counter;
      const bool oa = ops[x].started && ops[x].counter == wrapping_add(ops[y].counter, 1, total_rows, 4);
      const bool hp = ops[y].priority < ops[x].priority;
      wait |= (sb && hp && sc) || oa;
    }
    op_valid[x] = !wait;
  }
  const bool a_valid = op_valid[0], b_valid = op_valid[1], d_valid = op_valid[2];
  bool a_ready = true, b_ready = true, d_ready = true;

  d.cntl_ready = cntl_q_.enq_ready(q.cntl_deq_ready);  // a pipe queue (:178-181)

  // scratchpad bank reads (:423-454)
  const unsigned dataAbank = a_address.sp_bank(), dataBbank = b_address.sp_bank(), dataDbank = d_address.sp_bank();
  o.sp_read.assign(cfg_.sp_banks, SpReadReq());
  d.sp_read_operand.assign(cfg_.sp_banks, 0);
  for (unsigned i = 0; i < cfg_.sp_banks; ++i) {
    const bool read_a = a_valid && !a_read_from_acc && dataAbank == i && d.start_a && !multiply_garbage && a_row_nz;
    const bool read_b = b_valid && !b_read_from_acc && dataBbank == i && d.start_b && !accumulate_zeros && b_row_nz;
    const bool read_d = d_valid && !d_read_from_acc && dataDbank == i && d.start_d && !preload_zeros && d_row_nz;
    const bool ready = in_.sp_req_ready[i];
    if (read_a && !ready) a_ready = false;
    if (read_b && !ready) b_ready = false;
    if (read_d && !ready) d_ready = false;
    SpReadReq &r = o.sp_read[i];
    r.valid = (read_a || read_b || read_d) && d.cntl_ready;
    r.addr = read_b ? b_address.sp_row() : read_d ? d_address.sp_row() : a_address.sp_row();
    if (r.valid && ready)
      d.sp_read_operand[i] = read_b ? 'B' : read_d ? 'D' : 'A';
  }
  // accumulator bank reads (:457-502); not gated by cntl_ready, as in the RTL
  const unsigned aAcc = a_address.acc_bank(), bAcc = b_address.acc_bank(), dAcc = d_address.acc_bank();
  o.acc_read.assign(cfg_.acc_banks, AccReadReq());
  o.acc_read_ready.assign(cfg_.acc_banks, false);
  for (unsigned i = 0; i < cfg_.acc_banks; ++i) {
    const bool ra = a_valid && a_read_from_acc && aAcc == i && d.start_a && !multiply_garbage && a_row_nz;
    const bool rbb = b_valid && b_read_from_acc && bAcc == i && d.start_b && !accumulate_zeros && b_row_nz;
    const bool rd = d_valid && d_read_from_acc && dAcc == i && d.start_d && !preload_zeros && d_row_nz;
    AccReadReq &r = o.acc_read[i];
    r.valid = ra || rbb || rd;
    r.scale = acc_scale_;
    r.act = activation_;
    r.addr = rbb ? b_address.acc_row() : rd ? d_address.acc_row() : a_address.acc_row();
    // the bank's readiness depends on the address it is asked for (AccumulatorMem.scala:327-332)
    const bool ready = i < in_.acc_ready.size() && in_.acc_ready[i].ready(r.addr);
    o.acc_read_ready[i] = ready;
    if (ra && !ready) a_ready = false;
    if (rbb && !ready) b_ready = false;
    if (rd && !ready) d_ready = false;
  }
  d.a_fire = a_valid && a_ready;
  d.b_fire = b_valid && b_ready;
  d.d_fire = d_valid && d_ready;
  d.firing = d.start_a || d.start_b || d.start_d;
  const unsigned last = total_rows - 1;
  d.about_to_fire_all_rows = ((a_fire_counter_ == last && d.a_fire) || a_fire_counter_ == 0) &&
                             ((b_fire_counter_ == last && d.b_fire) || b_fire_counter_ == 0) &&
                             ((d_fire_counter_ == last && d.d_fire) || d_fire_counter_ == 0) &&
                             (a_fire_started_ || b_fire_started_ || d_fire_started_) && d.cntl_ready;

  // ---- the end of a pass (:632-681)
  if (state_ == COMPUTE && d.about_to_fire_all_rows) {
    const GemminiCmd &c0 = cmd_q_.bits(0), &c1 = cmd_q_.bits(1);
    if (perform_single_preload_) {
      d.pop = 1;
      d.next_state = WAITING;
      d.set_pending[0] = true;
      d.new_pending[0] = {c0.rob_valid && c_address_rs2.is_garbage(), c0.rob_id};
      if (current_dataflow_ == kOS) {
        d.set_in_prop_flush = true;
        d.in_prop_flush = !LocalAddr(rs2(0), amap_).is_garbage();
      }
    } else if (perform_mul_pre_) {
      d.pop = 2;
      d.next_state = WAITING;
      d.set_pending[0] = d.set_pending[1] = true;
      d.new_pending[0] = {c0.rob_valid, c0.rob_id};
      d.new_pending[1] = {c1.rob_valid && c_address_rs2.is_garbage(), c1.rob_id};
      if (current_dataflow_ == kOS) {
        d.set_in_prop_flush = true;
        d.in_prop_flush = !LocalAddr(rs2(1), amap_).is_garbage();
      }
    } else if (perform_single_mul_) {
      d.pop = 1;
      d.next_state = WAITING;
      d.set_pending[0] = true;
      d.new_pending[0] = {c0.rob_valid, c0.rob_id};
    }
  }

  // ---- control signals for the mesh (:750-801)
  const bool computing = d.performing_mul_pre || d.performing_single_mul || d.performing_single_preload;
  d.cntl_enq = computing && d.cntl_ready;
  Cntl &cn = d.cntl;
  cn.perform_mul_pre = d.performing_mul_pre;
  cn.perform_single_mul = d.performing_single_mul;
  cn.perform_single_preload = d.performing_single_preload;
  cn.a_bank = dataAbank;
  cn.b_bank = dataBbank;
  cn.d_bank = dataDbank;
  cn.a_bank_acc = aAcc;
  cn.b_bank_acc = bAcc;
  cn.d_bank_acc = dAcc;
  cn.a_garbage = a_garbage;
  cn.b_garbage = b_garbage;
  cn.d_garbage = d_garbage;
  cn.a_read_from_acc = a_read_from_acc;
  cn.b_read_from_acc = b_read_from_acc;
  cn.d_read_from_acc = d_read_from_acc;
  cn.accumulate_zeros = accumulate_zeros;
  cn.preload_zeros = preload_zeros;
  cn.a_unpadded_cols = a_row_nz ? a_cols : 0;
  cn.b_unpadded_cols = b_row_nz ? b_cols : 0;
  cn.d_unpadded_cols = d_row_nz ? d_cols : 0;
  cn.total_rows = total_rows;
  cn.a_fire = d.a_fire;
  cn.b_fire = d.b_fire;
  cn.d_fire = d.d_fire;
  cn.c_addr = c_address_rs2;
  cn.c_rows = c_rows;
  cn.c_cols = c_cols;
  cn.a_transpose = a_transpose_;
  cn.bd_transpose = bd_transpose_;
  cn.rob_valid = !d.performing_single_mul && !c_address_rs2.is_garbage();
  cn.rob_id = cmd_q_.bits(preload_cmd_place).rob_id;
  cn.dataflow = current_dataflow_;
  cn.prop = d.performing_single_preload ? in_prop_flush_ : in_prop;
  cn.shift = in_shift_;
  cn.first = !a_fire_started_ && !b_fire_started_ && !d_fire_started_;
  // bookkeeping: the commands of the pass (the compute is cmd 0 unless the pass is a preload)
  if (d.performing_mul_pre || d.performing_single_mul) {
    cn.compute_op = cmd_q_.bits(0).op;
    cn.compute_uop = cmd_q_.bits(0).uop;
    // the compute's A operand is garbage only for a garbage rs1 (gemmini.h never issues one)
    cn.computes = !LocalAddr(rs1(0), amap_).is_garbage();
  }
  if (d.performing_mul_pre || d.performing_single_preload) {
    cn.preload_op = cmd_q_.bits(preload_cmd_place).op;
    cn.preload_uop = cmd_q_.bits(preload_cmd_place).uop;
  }
  cn.pass_uop = pass_uop_;

  // ---- completions (:579, 958-990): config, then the mesh, then the pending ones
  o.completed_valid = false;
  o.completed_id = 0;
  if (q.mesh_completed_fire) {
    o.completed_valid = true;
    o.completed_id = q.tag.rob_id;
  } else {
    if (d.do_config) {
      o.completed_valid = cmd_q_.bits(0).rob_valid;
      o.completed_id = cmd_q_.bits(0).rob_id;
    }
    if (pending_[0].valid) {
      o.completed_valid = true;
      o.completed_id = pending_[0].bits;
      d.pop_pending[0] = true;
    } else if (pending_[1].valid) {
      o.completed_valid = true;
      o.completed_id = pending_[1].bits;
      d.pop_pending[1] = true;
    }
  }

  // ---- TransposePreloadUnroller (TransposePreloadUnroller.scala:30-81) into the command queue
  {
    const bool v0 = tpu_q_.valid(0), v1 = tpu_q_.valid(1);
    const GemminiCmd &c0 = tpu_q_.bits(0), &c1 = tpu_q_.bits(1);
    const bool first_preload = v0 && c0.funct == PRELOAD_CMD && tpu_state_ == 0;
    const bool unroll_preload = b_transposed_and_ws_ && v1 && c1.funct == COMPUTE_AND_FLIP_CMD;
    bool out_valid = v0;  // MuxCase (:57-60)
    if (first_preload)
      out_valid = !b_transposed_and_ws_ || v1;
    else if (tpu_state_ > 1)
      out_valid = true;
    GemminiCmd out = c0;
    const uint64_t garbage = 0xffffffffull;
    if (first_preload && unroll_preload) {
      out.rs2 = (c0.rs2 & ~garbage) | garbage;
      out.rob_valid = false;
    } else if (tpu_state_ == 1) {
      out = c1;  // first_compute_cmd: only inst.rs1/rs2 (register numbers) are overwritten
      out.funct = COMPUTE_AND_STAY_CMD;
      out.rob_valid = false;
    } else if (tpu_state_ == 2) {
      out.rs1 = (c0.rs1 & ~garbage) | garbage;
    }
    d.tpu_out_fire = out_valid && cmd_q_.enq_ready();
    d.tpu_out = out;
    d.tpu_pop = (d.tpu_out_fire && !(first_preload && unroll_preload) && tpu_state_ != 1) ? 1 : 0;
    d.tpu_next_state = tpu_state_;
    d.b_transposed_and_ws_next = b_transposed_and_ws_;
    if (d.tpu_out_fire) {
      const bool is_config = c0.funct == CONFIG_CMD && (c0.rs1 & 3) == CONFIG_EX;
      if (is_config) {
        if (!((c0.rs1 >> 7) & 1))
          d.b_transposed_and_ws_next = ((c0.rs1 >> 2) & 1) == kWS && ((c0.rs1 >> 9) & 1);
      } else if (first_preload && unroll_preload) {
        d.tpu_next_state = 1;
      } else if (tpu_state_ >= 1) {
        d.tpu_next_state = (tpu_state_ + 1) % 3;  // ChiselEnum .next wraps to idle
      }
    }
  }

  o.cmd_ready = tpu_q_.enq_ready();
  o.busy = q.busy;
  o.sp_resp_ready = q.wb.sp_resp_ready;
  o.sp_write = q.wb.sp_write;
  o.acc_write = q.wb.acc_write;
}

void ExecuteController::set_inputs(const In &in) {
  in_ = in;
  if (in_.sp_resp.size() != cfg_.sp_banks || in_.sp_req_ready.size() != cfg_.sp_banks ||
      in_.acc_ready.size() != cfg_.acc_banks)
    throw std::invalid_argument("ExecuteController::set_inputs: one entry per bank");
  evaluated_ = false;
}

void ExecuteController::eval() {
  q_ = deq(in_.sp_resp);
  issue(q_, d_, out_);
  hazard_stall_ = d_.hazard_stall;
  evaluated_ = true;
}

// ------------------------------------------------------------------ rising edge
void ExecuteController::tick() {
  if (!evaluated_)
    eval();
  const Decision &d = d_;
  if (uops_)
    record();

  // ---- config_ex (:541-582), from the head before it is popped
  unsigned next_dataflow = current_dataflow_;
  if (d.do_config) {
    const GemminiCmd &c = cmd_q_.bits(0);
    if ((c.rs1 & 3) == CONFIG_EX) {  // ConfigExRs1 / ConfigExRs2 (GemminiISA.scala:189-211)
      if (!((c.rs1 >> 7) & 1)) {     // set_only_strides
        if (cfg_.has_nonlinear_activations)
          activation_ = unsigned((c.rs1 >> 3) & 7);
        in_shift_ = unsigned(c.rs2 & mask_bits(log2_up(cfg_.array.acc_bits)));
        acc_scale_ = uint32_t(c.rs1 >> 32);
        a_transpose_ = (c.rs1 >> 8) & 1;
        bd_transpose_ = (c.rs1 >> 9) & 1;
        next_dataflow = unsigned((c.rs1 >> 2) & 1);
      }
      a_addr_stride_ = (c.rs1 >> 16) & 0xffff;
      c_addr_stride_ = (c.rs2 >> 48) & 0xffff;
    }
    // the CONFIG_IM2COL fields (:567-577) only matter with im2col, which v0.7.2 never enables
  }

  // ---- the TransposePreloadUnroller and the command queue
  cmd_q_.tick(d.tpu_out_fire, d.tpu_out, d.pop);
  tpu_q_.tick(in_.cmd_valid && tpu_q_.enq_ready(), in_.cmd, d.tpu_pop);
  tpu_state_ = d.tpu_next_state;
  b_transposed_and_ws_ = d.b_transposed_and_ws_next;

  // ---- controller registers
  if (current_dataflow_ == kWS)
    in_prop_flush_ = false;  // :92-95
  if (d.set_in_prop_flush)
    in_prop_flush_ = d.in_prop_flush;
  current_dataflow_ = next_dataflow;
  if (d.clear_performs)
    perform_single_preload_ = perform_mul_pre_ = perform_single_mul_ = false;
  if (d.set_perform_single_preload) perform_single_preload_ = true;
  if (d.set_perform_mul_pre) perform_mul_pre_ = true;
  if (d.set_perform_single_mul) perform_single_mul_ = true;
  state_ = d.next_state;

  // fire counters (:365-415)
  if (!d.firing) {
    a_fire_counter_ = 0;
    a_addr_offset_ = 0;
  } else if (d.a_fire && d.cntl_ready) {
    a_addr_offset_ = a_fire_counter_ == d.total_rows - 1 ? 0 : (a_addr_offset_ + a_addr_stride_) & mask_bits(20);
    a_fire_counter_ = unsigned(wrapping_add(a_fire_counter_, 1, d.total_rows, 4));
    a_fire_started_ = true;
  }
  if (!d.firing) {
    b_fire_counter_ = 0;
  } else if (d.b_fire && d.cntl_ready) {
    b_fire_counter_ = unsigned(wrapping_add(b_fire_counter_, 1, d.total_rows, 4));
    b_fire_started_ = true;
  }
  if (!d.firing) {
    d_fire_counter_ = 0;
  } else if (d.d_fire && d.cntl_ready) {
    d_fire_counter_ = unsigned(wrapping_add(d_fire_counter_, 1, d.total_rows, 4));
    d_fire_started_ = true;
  }
  if (d.about_to_fire_all_rows)
    a_fire_started_ = b_fire_started_ = d_fire_started_ = false;

  // pending completions: set by the FSM, then popped (:643-679, 982-990)
  for (int i = 0; i < 2; ++i) {
    if (d.set_pending[i])
      pending_[i] = d.new_pending[i];
    if (d.pop_pending[i])
      pending_[i].valid = false;
  }

  // write-back counter (:970-980)
  if (q_.resp_fire_rob)
    output_counter_ = unsigned(wrapping_add(output_counter_, 1, q_.mo.resp_total_rows, 4));

  // ---- the array: the request's tag (the table entry the bench tag points to), then the edge
  RequestNote note;
  const Cntl &h = q_.head;
  if (q_.mesh_req_fire) {
    ++mesh_requests_;
    const bool flush = q_.mi.req.flush != 0;
    if (!flush) {  // the TagQueue takes non-flush requests (MeshWithDelays.scala:224)
      Tag t;
      t.rob_valid = h.rob_valid;
      t.rob_id = h.rob_id;
      t.addr = h.perform_single_mul ? LocalAddr::garbage(amap_) : h.c_addr;  // :885, 898
      t.rows = h.c_rows;
      t.cols = h.c_cols;
      t.op = h.preload_op;
      t.uop = h.preload_uop;
      tags_[next_tag_] = t;
      next_tag_ = (next_tag_ + 1) % unsigned(tags_.size());
      const bool computes = (h.perform_mul_pre || h.perform_single_mul) && h.computes;
      compute_requests_ += computes;
      note.computes = computes;
      note.preloads = h.perform_mul_pre || h.perform_single_preload;
      note.uop = computes ? h.compute_uop : h.preload_uop >= 0 ? h.preload_uop : h.compute_uop;
      note.op = computes ? h.compute_op : h.preload_op >= 0 ? h.preload_op : h.compute_op;
      note.load_uop = h.preload_uop;
      note.load_op = h.preload_op;
    } else {
      note.computes = note.preloads = 0;
      note.uop = flush_uop_;
      note.op = last_op_;
    }
  }
  array_->set_inputs(q_.mi, q_.mesh_req_fire ? &note : nullptr);
  array_->tick();

  // control-signal queue
  cntl_q_.tick(d.cntl_enq, d.cntl, q_.cntl_deq_fire);
  evaluated_ = false;
}

// ------------------------------------------------------------------ micro-ops (bookkeeping)
void ExecuteController::record() {
  MicroOpTable &T = *uops_;
  Decision &d = d_;
  const int64_t t = array_->cycle();
  auto cmd_uop = [&](unsigned place) -> int64_t { return cmd_q_.valid(place) ? cmd_q_.bits(place).uop : -1; };
  auto with = [&](int64_t id, auto fn) {
    if (T.has(id))
      fn(T.at(id));
  };

  // the command handshake (io.cmd.fire)
  if (in_.cmd_valid && tpu_q_.enq_ready()) {
    with(in_.cmd.uop, [&](MicroOp &u) { first_cycle(u.issued, t); });
    if (in_.cmd.rob_valid && in_.cmd.rob_id < uop_of_rob_.size())
      uop_of_rob_[in_.cmd.rob_id] = in_.cmd.uop;
  }
  // a pass starts: create its Request micro-op
  const bool starts = state_ == WAITING &&
                      (d.set_perform_single_preload || d.set_perform_mul_pre || d.set_perform_single_mul);
  if (starts) {
    MicroOp r;
    r.kind = UopKind::Request;
    r.pass_kind = d.set_perform_single_preload ? PassKind::SinglePreload
                  : d.set_perform_mul_pre      ? PassKind::MulPre
                                               : PassKind::SingleMul;
    const unsigned pre_place = cmd_q_.bits(0).funct == PRELOAD_CMD ? 0 : 1;
    if (r.pass_kind != PassKind::SinglePreload)
      r.compute_uop = cmd_uop(0);
    if (r.pass_kind != PassKind::SingleMul)
      r.preload_uop = cmd_uop(pre_place);
    r.parent = r.compute_uop >= 0 ? r.compute_uop : r.preload_uop;
    with(r.parent, [&](MicroOp &p) { r.op = p.op; });
    r.started = t;
    r.total_rows = d.total_rows;
    pass_uop_ = T.add(r);
    d_.cntl.pass_uop = pass_uop_;  // the pass's first control entry is enqueued at this edge
    for (int64_t c : {r.compute_uop, r.preload_uop})
      with(c, [&](MicroOp &u) {
        first_cycle(u.started, t);
        u.pass = pass_uop_;
      });
  }
  if (d.do_config)
    with(cmd_uop(0), [&](MicroOp &u) { first_cycle(u.started, t); });
  // operand rows read (read.req.fire), attributed to the command whose address they read
  for (unsigned b = 0; b < d.sp_read_operand.size(); ++b) {
    const char op = d.sp_read_operand[b];
    if (!op)
      continue;
    const bool pre0 = cmd_q_.bits(0).funct == PRELOAD_CMD;
    const bool a_sbfit = current_dataflow_ == kOS ? !a_transpose_ : a_transpose_;
    const bool b_sbfit = current_dataflow_ == kOS && bd_transpose_;
    const unsigned place = op == 'D' ? (pre0 ? 0 : 1)
                           : op == 'A' ? (pre0 ? 1 : (a_sbfit ? 2 : 0))
                                       : (pre0 ? 1 : (b_sbfit ? 2 : 0));
    MicroOp r;
    r.kind = UopKind::OperandRead;
    r.parent = cmd_uop(place);
    with(r.parent, [&](MicroOp &p) {
      r.op = p.op;
      first_cycle(p.first_read, t);
      last_cycle(p.last_read, t);
      r.row = p.reads++;
    });
    r.operand = op;
    r.cycle = t;
    r.done = t + 1 + cfg_.spad_read_delay;  // earliest the row can be at the ExecuteController
    r.bank = b;
    r.addr = out_.sp_read[b].addr;
    r.pass = pass_uop_;
    T.add(r);
  }
  // the request handshake
  if (q_.mesh_req_fire) {
    const int request = int(array_->requests().size());
    if (q_.mi.req.flush) {
      MicroOp f;
      f.kind = UopKind::Flush;
      f.op = last_op_;
      f.parent = last_uop_;
      f.started = f.accept = t;
      f.request = request;
      f.pass_kind = PassKind::Flush;
      flush_uop_ = T.add(f);
      // the RequestNote of this request (tick) names flush_uop_, set just now
    } else {
      const Cntl &h = q_.head;
      with(h.pass_uop, [&](MicroOp &r) {
        r.accept = t;
        r.request = request;
      });
      for (int64_t c : {h.compute_uop, h.preload_uop})
        with(c, [&](MicroOp &u) {
          first_cycle(u.accept, t);
          u.request = request;
        });
    }
  }
  // pops leave the queue
  for (unsigned k = 0; k < d.pop; ++k)
    with(cmd_uop(k), [&](MicroOp &u) {
      u.popped = t;
      last_op_ = u.op;
      last_uop_ = u.id;
    });
  // result rows (resp.valid with a tag) and their write-back
  const MwdOut &mo = q_.mo;
  if (mo.resp_valid && q_.tag.rob_valid) {
    MicroOp r;
    r.kind = UopKind::ResultRow;
    r.parent = q_.tag.uop;
    r.cycle = t;
    for (unsigned b = 0; b < q_.wb.sp_write.size(); ++b)
      if (q_.wb.sp_write[b].en) {
        r.written = true;
        r.bank = b;
        r.addr = q_.wb.sp_write[b].addr;
        r.done = t;
      }
    for (unsigned b = 0; b < q_.wb.acc_write.size(); ++b)
      if (q_.wb.acc_write[b].valid) {
        r.written = r.to_acc = true;
        r.bank = b;
        r.addr = q_.wb.acc_write[b].addr;
        r.accumulate = q_.wb.acc_write[b].acc;
        r.done = t + 2;  // pipelined_writes(acc_latency - 1) lands at the edge ending t + 2
      }
    with(r.parent, [&](MicroOp &p) {
      r.op = p.op;
      r.row = p.result_rows++;
      first_cycle(p.result_first, t);
      last_cycle(p.result_last, t);
      if (r.written) {
        ++p.wb_rows;
        first_cycle(p.wb_first, t);
        last_cycle(p.wb_last, t);
        last_cycle(p.wb_done, r.done);
      }
    });
    T.add(r);
  }
  // completions
  if (out_.completed_valid && out_.completed_id < uop_of_rob_.size()) {
    with(uop_of_rob_[out_.completed_id], [&](MicroOp &u) {
      if (u.completed < 0)
        u.completed = t;
    });
    uop_of_rob_[out_.completed_id] = -1;
  }
}

}  // namespace systolique
