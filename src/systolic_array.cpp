#include "systolique/systolic_array.h"

#include "systolique/vcd.h"

#include <algorithm>
#include <stdexcept>

namespace systolique {

const char *to_string(PeState s) {
  switch (s) {
    case PeState::Idle: return "idle";
    case PeState::Mac: return "mac";
    case PeState::Load: return "load";
    case PeState::Drain: return "drain";
    case PeState::Bubble: return "bubble";
  }
  return "?";
}

uint64_t StateCounts::total() const {
  uint64_t s = 0;
  for (uint64_t v : n) s += v;
  return s;
}

void StateCounts::add(const StateCounts &o) {
  for (unsigned k = 0; k < kPeStates; ++k) n[k] += o.n[k];
  load_concurrent += o.load_concurrent;
  drain_concurrent += o.drain_concurrent;
}

PeState RequestInfo::state() const {
  if (flush) return dataflow == Dataflow::OS ? PeState::Drain : PeState::Bubble;
  if (computes) return PeState::Mac;
  if (preloads) return PeState::Load;
  return PeState::Bubble;
}

namespace {

// The token twin's configuration: the same structure (tiles, latencies, so the same registers,
// delays, ids and queues), widths wide enough to carry a 31-bit token through every d path
// unchanged: c1/c2 are accType = 32 bits in a BOTH array (PE.scala:58), out_c is 32 bits
// (PE.scala:104, 111, 120, 126 at outputType = 32), the Mesh pipes and MeshWithDelays buffers
// keep full values.
ArrayConfig twin_config(const ArrayConfig &cfg) {
  ArrayConfig t = cfg;
  t.name = cfg.name + "/provenance";
  t.in_bits = 16;
  t.out_bits = 32;
  t.acc_bits = 32;
  t.dataflow = Dataflow::BOTH;
  return t;
}

void require_size(const std::vector<int64_t> &v, unsigned n, const char *what) {
  if (v.size() != n)
    throw std::invalid_argument(std::string("SystolicArray: ") + what + " has " +
                                std::to_string(v.size()) + " lanes, expected " + std::to_string(n));
}

bool any_nonzero(const std::vector<int64_t> &v) {
  return std::any_of(v.begin(), v.end(), [](int64_t x) { return x != 0; });
}

Dataflow effective(const ArrayConfig &cfg, unsigned bit) {
  return cfg.dataflow == Dataflow::BOTH ? (bit ? Dataflow::WS : Dataflow::OS) : cfg.dataflow;
}

}  // namespace

// The twin gets the same request, valid and ready bits as the array, zeros on a and b and a token
// on every d lane: 1 + (number of the d handshake) * cols + lane, 0 being "power-on". With a = 0
// its MacUnits add nothing (WS out_b = b, OS c := c + 0), with shift = 0 the OS output path is
// the identity, so every token stays intact wherever the array moves the d value it stands for.
// What reaches the twin's Mesh is adjusted (adjust_mesh_feed): a and b zero (its transposer may
// hold d tokens that an OS request would read as A or B, MeshWithDelays.scala:152-153, 170-171),
// shift 0, and the array's fixed dataflow bit (a fixed-dataflow PE ignores the bit, PE.scala:102,
// 118; the twin is a BOTH array).
class SystolicArray::Twin final : public MeshWithDelays {
 public:
  explicit Twin(const ArrayConfig &array_cfg)
      : MeshWithDelays(twin_config(array_cfg)), array_dataflow_(array_cfg.dataflow) {}
  ~Twin() override = default;

 protected:
  void adjust_mesh_feed(MeshIn &f) const override {
    for (size_t k = 0; k < f.control.size(); ++k) {
      if (array_dataflow_ != Dataflow::BOTH) f.control[k].dataflow = unsigned(array_dataflow_);
      f.control[k].shift = 0;
    }
    std::fill(f.a.begin(), f.a.end(), 0);
    std::fill(f.b.begin(), f.b.end(), 0);
  }

 private:
  Dataflow array_dataflow_;
};

// The VCD variables (ArrayOptions::vcd_path): the clock, reset and cycle number, the ports of the
// interface (one variable per lane), and per PE its valid bit, state (PeState), active register,
// request of its row, the concurrent flags, its occupied cycles since reset and c1/c2.
struct SystolicArray::Vcd {
  explicit Vcd(const std::string &path) : w(path, "systolique") {}
  VcdWriter w;
  int clock = -1, reset = -1, cycle = -1;
  struct Port {
    int id;
    std::vector<int> lanes;  // empty: a scalar
  };
  std::vector<Port> in, out;
  struct Pe {
    int valid, state, active, request, load_concurrent, drain_concurrent, occupied, c1, c2;
  };
  std::vector<Pe> pes;
};

namespace {

SystolicArray::Vcd::Port vcd_port(VcdWriter &w, const std::string &name, unsigned lanes,
                                  unsigned width) {
  SystolicArray::Vcd::Port p{-1, {}};
  if (lanes == 0)
    p.id = w.add("ports", name, width);
  else
    for (unsigned k = 0; k < lanes; ++k)
      p.lanes.push_back(w.add("ports", name + "_" + std::to_string(k), width));
  return p;
}

void vcd_set(VcdWriter &w, const SystolicArray::Vcd::Port &p, uint64_t v) { w.set(p.id, v); }
template <class T>
void vcd_set(VcdWriter &w, const SystolicArray::Vcd::Port &p, const std::vector<T> &v) {
  for (size_t k = 0; k < p.lanes.size(); ++k) w.set(p.lanes[k], uint64_t(int64_t(v[k])));
}

}  // namespace

SystolicArray::SystolicArray(const ArrayConfig &cfg, ArrayOptions opt) : cfg_(cfg), opt_(opt) {
  build();
  in_.resize(cfg_);
  mesh_in_.resize(cfg_);
  clear_run();
  if (!opt_.vcd_path.empty()) declare_vcd();
}

// Owned through unique_ptr: the VCD file is flushed and closed first, then the components.
SystolicArray::~SystolicArray() {
  vcd_.reset();
  twin_.reset();
  mesh_.reset();
  mwd_.reset();
}

void SystolicArray::build() {
  mwd_.reset();
  mesh_.reset();
  twin_.reset();
  if (opt_.interface == Interface::Mesh) {
    mesh_ = std::make_unique<Mesh>(cfg_);
  } else {
    mwd_ = std::make_unique<MeshWithDelays>(cfg_);
    if (opt_.provenance) twin_ = std::make_unique<Twin>(cfg_);
  }
}

void SystolicArray::require(Interface i) const {
  if (opt_.interface != i)
    throw std::logic_error(i == Interface::Mesh
                               ? "SystolicArray: the bare Mesh interface needs Interface::Mesh"
                               : "SystolicArray: requests need Interface::MeshWithDelays");
}

const MwdOut &SystolicArray::out() const {
  require(Interface::MeshWithDelays);
  return mwd_->out();
}

const MeshOut &SystolicArray::mesh_out() const {
  require(Interface::Mesh);
  return mesh_->out();
}

const Mesh &SystolicArray::mesh() const { return mwd_ ? mwd_->mesh() : *mesh_; }

const MeshWithDelays &SystolicArray::mesh_with_delays() const {
  require(Interface::MeshWithDelays);
  return *mwd_;
}

void SystolicArray::clear_run() {
  const unsigned pes = cfg_.rows() * cfg_.cols();
  cycle_ = 0;
  requests_.clear();
  rows_.assign(opt_.interface == Interface::Mesh ? cfg_.cols() : 1, {});
  d_base_ += drows_.size();
  drows_.clear();
  d_pending_ = -1;
  resp_rows_ = 0;
  tag_fifo_.clear();
  tag_head_ = 0;
  mesh_op_of_id_.clear();
  mesh_open_op_ = -1;
  pe_seen_.assign(pes, 0);
  pe_valid_now_.clear();
  src_.assign(pes, {});
  cyc_begin_.assign(1, 0);
  cyc_ent_.clear();
  pe_runs_.assign(pes, {});
}

void SystolicArray::reset(unsigned cycles) {
  build();
  clear_run();
  if (opt_.interface == Interface::Mesh) {
    MeshIn idle;
    idle.resize(cfg_);
    idle.reset = true;
    for (unsigned k = 0; k < cycles; ++k) step_mesh(idle);
    idle.reset = false;
    mesh_in_ = idle;
  } else {
    MwdIn idle;
    idle.resize(cfg_);
    idle.reset = true;
    for (unsigned k = 0; k < cycles; ++k) step(idle);
    idle.reset = false;
    in_ = idle;
  }
}

void SystolicArray::set_inputs(const MwdIn &in, const RequestNote *note) {
  require(Interface::MeshWithDelays);
  require_size(in.a, cfg_.rows(), "a");
  require_size(in.b, cfg_.cols(), "b");
  require_size(in.d, cfg_.cols(), "d");
  in_ = in;
  has_note_ = note != nullptr;
  if (note) note_ = *note;
}

MwdOut SystolicArray::step(const MwdIn &in, const RequestNote *note) {
  set_inputs(in, note);
  const MwdOut o = mwd_->out();
  tick();
  return o;
}

void SystolicArray::set_mesh_inputs(const MeshIn &in) {
  require(Interface::Mesh);
  require_size(in.a, cfg_.rows(), "a");
  require_size(in.b, cfg_.cols(), "b");
  require_size(in.d, cfg_.cols(), "d");
  mesh_in_ = in;
}

MeshOut SystolicArray::step_mesh(const MeshIn &in) {
  set_mesh_inputs(in);
  const MeshOut o = mesh_->out();
  tick();
  return o;
}

void SystolicArray::tick() {
  if (opt_.interface == Interface::Mesh)
    tick_mesh();
  else
    tick_mwd();
  ++edges_;
}

void SystolicArray::close_request(RequestInfo &r) {
  r.computes = r.note_computes >= 0 ? r.note_computes > 0 : r.a_nonzero;
  r.preloads = r.note_preloads >= 0 ? r.note_preloads > 0 : r.d_nonzero;
  r.decided = true;
}

RegSource SystolicArray::decode(int64_t token) const {
  RegSource s;
  if (token <= 0) return s;
  const uint64_t g = uint64_t(token - 1) / cfg_.cols();
  if (g < d_base_ || g - d_base_ >= drows_.size()) return s;  // before the last reset
  const DRowInfo &d = drows_[g - d_base_];
  const unsigned dim = cfg_.block_size();
  s.d_row = int64_t(g - d_base_);
  s.lane = unsigned(uint64_t(token - 1) % cfg_.cols());
  s.request = d.request;
  s.row = d.row;
  s.transposed = d.transposed;
  s.value = d.values[s.lane];
  if (d.request >= 0) {
    s.op = requests_[size_t(d.request)].load_op;
    s.uop = requests_[size_t(d.request)].load_uop;
    // ExecuteController.scala:253 (d rows bottom row first); through the transposer
    // (MeshWithDelays.scala:160, 172-173) lane l of input row r ends up as element (l, DIM-1-r).
    s.w_row = int(d.transposed ? s.lane : dim - 1 - d.row);
    s.w_col = int(d.transposed ? dim - 1 - d.row : s.lane);
  }
  return s;
}

// OS: is the value leaving through out_c (the register being replaced, PE.scala:104, 111) a
// finished result? It is if it was accumulated into (it is this PE's own result) or if it is
// below the row of the D element it started from (a result of a PE above, moving down).
bool SystolicArray::result_out(unsigned pe, unsigned shadow) const {
  const RegSource &s = src_[pe][shadow];
  if (s.request < 0 || s.w_row < 0) return false;
  return int(pe / cfg_.cols()) > s.w_row || s.uses > 0;
}

PeView SystolicArray::pe(unsigned row, unsigned col) const {
  PeView v;
  const unsigned p = row * cfg_.cols() + col;
  const Mesh &m = mesh();
  v.in = m.pe_inputs(row, col);
  v.dataflow = effective(cfg_, v.in.control.dataflow);
  v.regs = m.pe(row, col);
  v.active = (v.in.valid ? v.in.control.propagate : v.regs.last_s) ? 1 : 0;
  v.src[0] = src_[p][0];
  v.src[1] = src_[p][1];
  if (!v.in.valid) return v;
  const auto &lane = rows_[opt_.interface == Interface::Mesh ? col : 0];
  if (pe_seen_[p] >= lane.size()) return v;
  const RowInfo &ri = lane[pe_seen_[p]];
  v.request = ri.request;
  v.row_in_request = ri.row;
  // The view includes this cycle's use of the active register (account_pes records it when the
  // cycle is ticked).
  RegSource &a = v.src[v.active];
  if (a.active < 0) a.active = cycle_;
  a.last_used = cycle_;
  a.last_use_request = ri.request;
  ++a.uses;
  if (ri.request < 0) {
    v.state = PeState::Bubble;
    return v;
  }
  const RequestInfo &r = requests_[ri.request];
  v.state = r.state();
  v.state_final = r.decided;
  v.load_concurrent = v.state == PeState::Mac && r.preloads;
  v.result_out = v.dataflow == Dataflow::OS && twin_ && result_out(p, 1 - v.active);
  v.drain_concurrent = v.state == PeState::Mac && v.result_out;
  return v;
}

// The occupied PEs of this cycle: the m-th valid cycle of a PE works on the m-th row that
// entered its column (the valid ShiftRegisters and Pipes never drop or reorder a row,
// Mesh.scala:42-46, 91-97), so a row's request is known at every PE without decoding ids.
void SystolicArray::account_pes() {
  pe_valid_now_.clear();
  const Mesh &m = mesh();
  const unsigned cols = cfg_.cols();
  for (unsigned r = 0; r < cfg_.rows(); ++r)
    for (unsigned c = 0; c < cols; ++c) {
      const Mesh::PeInputs in = m.pe_inputs(r, c);
      if (!in.valid) continue;
      const unsigned p = r * cols + c;
      const auto &lane = rows_[opt_.interface == Interface::Mesh ? c : 0];
      const uint64_t k = pe_seen_[p]++;
      if (k >= lane.size())
        throw std::logic_error("SystolicArray: PE " + std::to_string(r) + "," +
                               std::to_string(c) + " valid with no row fed (cycle " +
                               std::to_string(cycle_) + ")");
      const int q = lane[k].request;
      const unsigned act = in.control.propagate ? 1 : 0;
      const bool os = effective(cfg_, in.control.dataflow) == Dataflow::OS;
      const uint32_t res = os && twin_ && result_out(p, 1 - act) ? 1 : 0;
      size_t e = cyc_begin_.back();
      while (e < cyc_ent_.size() && cyc_ent_[e].request != q) ++e;
      if (e == cyc_ent_.size()) cyc_ent_.push_back({q, 0, 0});
      ++cyc_ent_[e].count;
      cyc_ent_[e].result_out += res;
      auto &runs = pe_runs_[p];
      if (runs.empty() || runs.back().request != q) runs.push_back({q, 0, 0});
      ++runs.back().count;
      runs.back().result_out += res;
      if (q >= 0) {
        ++requests_[q].valid_pe_cycles;
        requests_[q].result_out_pe_cycles += res;
      }
      // The active register is the MacUnit's multiplicand (WS) / addend (OS) this cycle.
      RegSource &a = src_[p][act];
      if (a.active < 0) a.active = cycle_;
      a.last_used = cycle_;
      a.last_use_request = q;
      ++a.uses;
      int64_t tw = 0;
      if (twin_) tw = act ? twin_->mesh().pe(r, c).c2 : twin_->mesh().pe(r, c).c1;
      pe_valid_now_.push_back({p, act, q, tw});
    }
}

void SystolicArray::account_mwd(const MwdIn &in, const MwdOut &o, const RequestNote *note) {
  const int64_t t = cycle_;
  const MeshWithDelays &mwd = *mwd_;
  if (tag_fifo_.size() - tag_head_ != mwd.tag_queue().len())
    throw std::logic_error("SystolicArray: tag queue bookkeeping out of step");
  // 1. The row entering the Mesh this cycle (the un-skewed feed = lane group 0's input), in the
  // request held by the request register (MeshWithDelays.scala:110, 149, 188-195).
  const MeshIn &f = mwd.feed_history(0);
  if (f.valid[0]) {
    const int q = requests_.empty() ? -1 : int(requests_.size()) - 1;
    const unsigned r = mwd.fire_counter();
    rows_[0].push_back({q, r, t});
    if (q >= 0) {
      RequestInfo &R = requests_[q];
      if (R.first_in < 0) R.first_in = t;
      R.last_in = t;
      ++R.rows_in;
      if (!R.flush) {
        R.a_nonzero = R.a_nonzero || any_nonzero(f.a);
        R.d_nonzero = R.d_nonzero || any_nonzero(f.d);
        if (!R.decided) {
          R.computes = R.note_computes >= 0 ? R.note_computes > 0 : R.a_nonzero;
          R.preloads = R.note_preloads >= 0 ? R.note_preloads > 0 : R.d_nonzero;
        }
      }
      if (f.last[0] && mwd.req().flush <= 1) close_request(R);
    }
    // The d buffer goes in with this row: written and not consumed yet (MeshWithDelays.scala:
    // 123-126 clear the written flags with every row that enters).
    if (d_pending_ >= 0) {
      DRowInfo &d = drows_[size_t(d_pending_)];
      d.request = q;
      d.row = r;
      d.transposed = mwd.req().dataflow == unsigned(Dataflow::WS) && mwd.req().bd_transpose;
      d_pending_ = -1;
    }
  }
  // 2. A response row: the k-th leaves as the k-th row that went in; it carries the tag of the
  // head of the tag queue when the output id matches (MeshWithDelays.scala:228-235).
  if (o.resp_valid) {
    const uint64_t k = resp_rows_++;
    if (k < rows_[0].size() && rows_[0][k].request >= 0) {
      RequestInfo &R = requests_[rows_[0][k].request];
      if (R.first_out < 0) R.first_out = t;
      R.last_out = t;
      ++R.rows_out;
    }
    if (mwd.tag_queue().deq_valid() && mwd.output_id() == mwd.tag_queue().deq_bits().id) {
      RequestInfo &R = requests_[tag_fifo_[tag_head_]];
      if (R.result_first < 0) R.result_first = t;
      R.result_last = t;
      ++R.result_rows;
      if (o.resp_last) ++tag_head_;
    }
  }
  // 3. Handshakes at the edge that ends this cycle.
  if (in.req_valid && o.req_ready) {
    RequestInfo R;
    R.index = unsigned(requests_.size());
    R.req = in.req;
    R.flush = in.req.flush > 0;
    R.dataflow = effective(cfg_, in.req.dataflow);
    R.accept = t;
    if (note) {
      R.label = note->label;
      R.op = note->op;
      R.uop = note->uop;
      R.load_op = note->load_op;
      R.load_uop = note->load_uop;
      R.note_computes = note->computes;
      R.note_preloads = note->preloads;
      if (note->computes >= 0) R.computes = note->computes > 0;
      if (note->preloads >= 0) R.preloads = note->preloads > 0;
      R.decided = note->computes >= 0 && note->preloads >= 0;
    }
    if (R.flush) R.decided = true;
    if (!R.flush) tag_fifo_.push_back(int(R.index));  // TagQueue enq (MeshWithDelays.scala:223)
    requests_.push_back(std::move(R));
  }
  if (in.d_valid && o.d_ready) {
    DRowInfo d;
    d.cycle = t;
    d.values = in.d;
    d_pending_ = int64_t(drows_.size());
    drows_.push_back(std::move(d));
  }
  // 4. The PEs.
  account_pes();
}

void SystolicArray::after_edge() {
  // Each PE valid in the cycle just ended wrote its other register from in_d (PE.scala:109, 116,
  // 124, 130): it now holds the twin's token for that value.
  const int64_t t = cycle_;
  for (const ValidPe &v : pe_valid_now_) {
    const unsigned p = v.pe, r = p / cfg_.cols(), c = p % cfg_.cols();
    const unsigned shadow = 1 - v.active;
    RegSource s;
    if (twin_) {
      const PeRegs &tw = twin_->mesh().pe(r, c);
      s = decode(shadow ? tw.c2 : tw.c1);
      // The active register keeps its token: WS does not write it, OS adds a*b = 0 to it.
      if ((v.active ? tw.c2 : tw.c1) != v.twin_active)
        throw std::logic_error("SystolicArray: provenance token of an active register changed");
    }
    s.loaded = t;
    s.loaded_by = v.request;
    src_[p][shadow] = s;
  }
}

namespace {

bool rows_in_flight(const ArrayConfig &cfg, const Mesh &mesh, const MeshWithDelays *mwd) {
  for (unsigned r = 0; r < cfg.mesh_rows; ++r)
    for (unsigned c = 0; c < cfg.mesh_cols; ++c)
      for (unsigned j = 0; j < cfg.tile_cols; ++j)
        for (unsigned k = 0; k <= cfg.tile_latency; ++k)
          if (mesh.valid_stage(r, c, j, k)) return true;
  for (unsigned v : mesh.out().valid)
    if (v) return true;
  if (mwd) {
    for (unsigned k = 0; k < mwd->feed_history_size(); ++k)
      if (mwd->feed_history(k).valid[0]) return true;
    for (unsigned k = 0; k < mwd->resp_history_size(); ++k)
      if (mwd->resp_history(k).valid) return true;
  }
  return false;
}

}  // namespace

void SystolicArray::tick_mwd() {
  const RequestNote *note = has_note_ ? &note_ : nullptr;
  has_note_ = false;  // a note belongs to one cycle
  if (in_.reset) {
    if (rows_in_flight(cfg_, mwd_->mesh(), mwd_.get()))
      throw std::logic_error("SystolicArray: reset with rows in flight is not supported");
    if (vcd_) sample_vcd();
    mwd_->set_inputs(in_);
    mwd_->eval();
    if (twin_) {
      MwdIn tin = in_;
      std::fill(tin.a.begin(), tin.a.end(), 0);
      std::fill(tin.b.begin(), tin.b.end(), 0);
      std::fill(tin.d.begin(), tin.d.end(), 0);
      twin_->set_inputs(tin);
      twin_->eval();
    }
    mwd_->tick();
    if (twin_) twin_->tick();
    // Values in the PE registers stay (the Mesh has no reset); their sources are now unknown.
    clear_run();
    return;
  }
  const MwdOut o = mwd_->out();
  const uint64_t token_row = d_base_ + drows_.size();  // the number this cycle's d row gets
  if (vcd_) sample_vcd();  // before the accounting: PeView reads the pre-tick bookkeeping
  // Phase 1: every component evaluates from the pre-edge registers.
  mwd_->set_inputs(in_);
  mwd_->eval();
  if (twin_) {
    MwdIn tin = in_;
    std::fill(tin.a.begin(), tin.a.end(), 0);
    std::fill(tin.b.begin(), tin.b.end(), 0);
    for (unsigned l = 0; l < cfg_.cols(); ++l) tin.d[l] = int64_t(1 + token_row * cfg_.cols() + l);
    if (tin.d.back() >= (int64_t(1) << 31))
      throw std::length_error("SystolicArray: provenance tokens exhausted (2^31 d values)");
    twin_->set_inputs(tin);
    twin_->eval();
    // The twin must make every control decision the array makes.
    const MeshIn &tf = twin_->mesh_in(), &feed = mwd_->mesh_in();
    if (tf.valid != feed.valid || tf.id != feed.id || tf.last != feed.last)
      throw std::logic_error("SystolicArray: provenance twin out of step at cycle " +
                             std::to_string(cycle_));
  }
  account_mwd(in_, o, note);
  // Phase 2: the edge.
  mwd_->tick();
  if (twin_) twin_->tick();
  after_edge();
  ++cycle_;
  cyc_begin_.push_back(cyc_ent_.size());
}

void SystolicArray::account_mesh(const MeshIn &in) {
  const int64_t t = cycle_;
  auto new_op = [&](unsigned lane) {
    RequestInfo R;
    R.index = unsigned(requests_.size());
    R.req.dataflow = in.control[lane].dataflow;
    R.req.propagate = in.control[lane].propagate;
    R.req.shift = in.control[lane].shift;
    R.req.tag_id = in.id[lane];
    R.dataflow = effective(cfg_, in.control[lane].dataflow);
    R.accept = t;
    // The bare Mesh shows no operands per op: every valid PE-cycle counts as a MAC.
    R.computes = true;
    R.decided = true;
    requests_.push_back(R);
    const unsigned id = in.id[lane];
    if (mesh_op_of_id_.size() <= id) mesh_op_of_id_.resize(id + 1, -1);
    mesh_op_of_id_[id] = int(R.index);
    return int(R.index);
  };
  // Lane 0 opens an op at its first row, at an id change and after a row with last.
  for (unsigned j = 0; j < cfg_.cols(); ++j) {
    if (!in.valid[j]) continue;
    int q;
    if (j == 0) {
      if (mesh_open_op_ < 0 || requests_[mesh_open_op_].req.tag_id != in.id[0])
        mesh_open_op_ = new_op(0);
      q = mesh_open_op_;
    } else {
      const unsigned id = in.id[j];
      q = id < mesh_op_of_id_.size() && mesh_op_of_id_[id] >= 0 ? mesh_op_of_id_[id] : new_op(j);
    }
    RequestInfo &R = requests_[q];
    rows_[j].push_back({q, R.rows_in, t});
    if (j == 0) {
      if (R.first_in < 0) R.first_in = t;
      R.last_in = t;
      ++R.rows_in;
      if (in.last[0]) mesh_open_op_ = -1;
    }
  }
  if (mesh_->out().valid[0]) {
    const uint64_t k = resp_rows_++;
    if (k < rows_[0].size()) {
      RequestInfo &R = requests_[rows_[0][k].request];
      if (R.first_out < 0) R.first_out = t;
      R.last_out = t;
      ++R.rows_out;
    }
  }
  account_pes();
}

void SystolicArray::tick_mesh() {
  if (mesh_in_.reset) {
    if (rows_in_flight(cfg_, *mesh_, nullptr))
      throw std::logic_error("SystolicArray: reset with rows in flight is not supported");
    if (vcd_) sample_vcd();
    mesh_->set_inputs(mesh_in_);
    mesh_->tick();
    clear_run();
    return;
  }
  if (vcd_) sample_vcd();
  mesh_->set_inputs(mesh_in_);
  mesh_->eval();
  account_mesh(mesh_in_);
  mesh_->tick();
  after_edge();
  ++cycle_;
  cyc_begin_.push_back(cyc_ent_.size());
}

namespace {

void add_entry(const std::vector<RequestInfo> &reqs, bool provenance, int q, uint64_t count,
               uint64_t result_out, StateCounts &s) {
  if (q < 0) {
    s[PeState::Bubble] += count;
    return;
  }
  const RequestInfo &r = reqs[q];
  const PeState st = r.state();
  s[st] += count;
  if (st == PeState::Mac) {
    if (r.preloads) s.load_concurrent += count;
    if (provenance && r.dataflow == Dataflow::OS) s.drain_concurrent += result_out;
  }
}

}  // namespace

Accounting SystolicArray::accounting() const {
  Accounting a;
  a.pes = cfg_.rows() * cfg_.cols();
  a.cycles = cycle_;
  const bool prov = bool(twin_);
  a.per_cycle.resize(size_t(cycle_));
  for (int64_t t = 0; t < cycle_; ++t) {
    StateCounts &s = a.per_cycle[size_t(t)];
    uint64_t occ = 0;
    for (size_t k = cyc_begin_[size_t(t)]; k < cyc_begin_[size_t(t) + 1]; ++k) {
      add_entry(requests_, prov, cyc_ent_[k].request, cyc_ent_[k].count, cyc_ent_[k].result_out, s);
      occ += cyc_ent_[k].count;
    }
    s[PeState::Idle] = a.pes - occ;
    a.total.add(s);
    if (occ) a.busy_end = t;
  }
  a.per_pe.resize(a.pes);
  for (unsigned p = 0; p < a.pes; ++p) {
    StateCounts &s = a.per_pe[p];
    uint64_t occ = 0;
    for (const Entry &e : pe_runs_[p]) {
      add_entry(requests_, prov, e.request, e.count, e.result_out, s);
      occ += e.count;
    }
    s[PeState::Idle] = uint64_t(cycle_) - occ;
  }
  for (const RequestInfo &r : requests_) {
    StateCounts s;
    add_entry(requests_, prov, int(r.index), r.valid_pe_cycles, r.result_out_pe_cycles, s);
    a.per_request.push_back(s);
    a.per_uop[r.uop].add(s);
    a.per_op[r.op].add(s);
    const uint64_t loads = s[PeState::Load] + s.load_concurrent;
    if (loads) a.loads_by_uop[r.load_uop] += loads;
    const int64_t begin = r.accept >= 0 ? r.accept : r.first_in;
    if (begin >= 0 && (a.busy_begin < 0 || begin < a.busy_begin)) a.busy_begin = begin;
  }
  if (a.busy_end >= 0 && a.busy_begin < 0) a.busy_begin = 0;
  if (a.busy_end >= 0)
    for (int64_t t = a.busy_begin; t <= a.busy_end; ++t) a.busy.add(a.per_cycle[size_t(t)]);
  const double run = double(a.pes) * double(a.cycles), busy = double(a.pes) * double(a.busy_cycles());
  if (run > 0) {
    a.occupancy = double(a.total.occupied()) / run;
    a.utilisation = double(a.total.macs()) / run;
  }
  if (busy > 0) {
    a.busy_occupancy = double(a.busy.occupied()) / busy;
    a.busy_utilisation = double(a.busy.macs()) / busy;
  }
  return a;
}

std::string SystolicArray::check(const Accounting &a) {
  auto fail = [](const std::string &what, int64_t i) { return what + " (" + std::to_string(i) + ")"; };
  auto same = [](const StateCounts &x, const StateCounts &y) {
    return x.n == y.n && x.load_concurrent == y.load_concurrent &&
           x.drain_concurrent == y.drain_concurrent;
  };
  auto flags_ok = [](const StateCounts &s) {
    return s.load_concurrent <= s.macs() && s.drain_concurrent <= s.macs();
  };
  StateCounts sum_c, sum_p, sum_r;
  for (size_t t = 0; t < a.per_cycle.size(); ++t) {
    const StateCounts &s = a.per_cycle[t];
    if (s.total() != a.pes) return fail("cycle: states do not sum to rows x cols", int64_t(t));
    if (s.macs() > a.pes) return fail("cycle: more MACs than PEs", int64_t(t));
    if (!flags_ok(s)) return fail("cycle: a concurrent flag outside the MAC PE-cycles", int64_t(t));
    sum_c.add(s);
  }
  if (a.per_cycle.size() != size_t(a.cycles)) return "per-cycle counts do not cover the run";
  for (size_t p = 0; p < a.per_pe.size(); ++p) {
    const StateCounts &s = a.per_pe[p];
    if (s.total() != uint64_t(a.cycles)) return fail("PE: states do not sum to the cycles", int64_t(p));
    if (s.macs() > uint64_t(a.cycles)) return fail("PE: more MACs than cycles", int64_t(p));
    if (!flags_ok(s)) return fail("PE: a concurrent flag outside the MAC PE-cycles", int64_t(p));
    sum_p.add(s);
  }
  if (a.per_pe.size() != a.pes) return "per-PE counts do not cover the PEs";
  for (size_t r = 0; r < a.per_request.size(); ++r) {
    if (a.per_request[r][PeState::Idle]) return fail("request with idle PE-cycles", int64_t(r));
    sum_r.add(a.per_request[r]);
  }
  sum_r[PeState::Idle] = a.total[PeState::Idle];
  StateCounts sum_u, sum_o;
  for (const auto &u : a.per_uop) {
    if (u.second[PeState::Idle]) return fail("micro-op with idle PE-cycles", u.first);
    if (!flags_ok(u.second)) return fail("micro-op: a concurrent flag outside its MAC PE-cycles", u.first);
    sum_u.add(u.second);
  }
  for (const auto &o : a.per_op) sum_o.add(o.second);
  sum_u[PeState::Idle] = sum_o[PeState::Idle] = a.total[PeState::Idle];
  uint64_t loads = 0;
  for (const auto &l : a.loads_by_uop) loads += l.second;
  if (!same(sum_u, a.total)) return "per-micro-op counts do not sum to the totals";
  if (!same(sum_o, a.total)) return "per-operation counts do not sum to the totals";
  if (loads != a.total[PeState::Load] + a.total.load_concurrent)
    return "loads by micro-op do not sum to Load + load_concurrent";
  if (!same(sum_c, a.total)) return "per-cycle counts do not sum to the totals";
  if (!same(sum_p, a.total)) return "per-PE counts do not sum to the totals";
  if (!same(sum_r, a.total)) return "per-request counts do not sum to the totals";
  if (a.total.total() != uint64_t(a.pes) * uint64_t(a.cycles)) return "totals != rows x cols x cycles";
  if (a.busy.occupied() != a.total.occupied() || a.busy.macs() != a.total.macs())
    return "occupied PE-cycles outside the busy window";
  return "";
}

void SystolicArray::declare_vcd() {
  vcd_ = std::make_unique<Vcd>(opt_.vcd_path);
  Vcd &v = *vcd_;
  VcdWriter &w = v.w;
  v.clock = w.add("", "clock", 1);
  v.reset = w.add("", "reset", 1);
  v.cycle = w.add("", "cycle", 32);
  const unsigned R = cfg_.rows(), C = cfg_.cols(), iw = cfg_.in_bits, ow = cfg_.out_bits;
  if (opt_.interface == Interface::MeshWithDelays) {
    const unsigned rw = cfg_.total_rows_bits(), tb = cfg_.tag_bits;
    v.in = {vcd_port(w, "a_valid", 0, 1), vcd_port(w, "a_bits", R, iw),
            vcd_port(w, "b_valid", 0, 1), vcd_port(w, "b_bits", C, iw),
            vcd_port(w, "d_valid", 0, 1), vcd_port(w, "d_bits", C, iw),
            vcd_port(w, "req_valid", 0, 1), vcd_port(w, "req_dataflow", 0, 1),
            vcd_port(w, "req_propagate", 0, 1), vcd_port(w, "req_shift", 0, cfg_.shift_bits()),
            vcd_port(w, "req_a_transpose", 0, 1), vcd_port(w, "req_bd_transpose", 0, 1),
            vcd_port(w, "req_total_rows", 0, rw), vcd_port(w, "req_tag_valid", 0, 1),
            vcd_port(w, "req_tag_id", 0, tb), vcd_port(w, "req_flush", 0, 2)};
    v.out = {vcd_port(w, "a_ready", 0, 1), vcd_port(w, "b_ready", 0, 1),
             vcd_port(w, "d_ready", 0, 1), vcd_port(w, "req_ready", 0, 1),
             vcd_port(w, "resp_valid", 0, 1), vcd_port(w, "resp_data", C, ow),
             vcd_port(w, "resp_total_rows", 0, rw), vcd_port(w, "resp_tag_valid", 0, 1),
             vcd_port(w, "resp_tag_id", 0, tb), vcd_port(w, "resp_last", 0, 1)};
  } else {
    const unsigned sw = cfg_.shift_bits(), idw = cfg_.id_bits();
    v.in = {vcd_port(w, "in_a", R, iw),          vcd_port(w, "in_b", C, iw),
            vcd_port(w, "in_d", C, iw),          vcd_port(w, "in_dataflow", C, 1),
            vcd_port(w, "in_propagate", C, 1),   vcd_port(w, "in_shift", C, sw),
            vcd_port(w, "in_id", C, idw),        vcd_port(w, "in_last", C, 1),
            vcd_port(w, "in_valid", C, 1)};
    v.out = {vcd_port(w, "out_b", C, ow),        vcd_port(w, "out_c", C, ow),
             vcd_port(w, "out_valid", C, 1),     vcd_port(w, "out_dataflow", C, 1),
             vcd_port(w, "out_propagate", C, 1), vcd_port(w, "out_shift", C, sw),
             vcd_port(w, "out_id", C, idw),      vcd_port(w, "out_last", C, 1)};
  }
  for (unsigned r = 0; r < R; ++r)
    for (unsigned c = 0; c < C; ++c) {
      const std::string s = "pe_" + std::to_string(r) + "_" + std::to_string(c);
      const unsigned cw = cfg_.c_bits();
      v.pes.push_back({w.add(s, "valid", 1), w.add(s, "state", 3), w.add(s, "active", 1),
                       w.add(s, "request", 32), w.add(s, "load_concurrent", 1),
                       w.add(s, "drain_concurrent", 1), w.add(s, "occupied_cycles", 32),
                       w.add(s, "c1", cw), w.add(s, "c2", cw)});
    }
}

// One VCD time step per cycle, time = 10 x clock edges since construction: every value of the
// cycle with the clock high at its start, the clock low half-way. Only reads state.
void SystolicArray::sample_vcd() {
  Vcd &v = *vcd_;
  VcdWriter &w = v.w;
  const bool mwd = opt_.interface == Interface::MeshWithDelays;
  w.set(v.clock, 1);
  w.set(v.reset, mwd ? in_.reset : mesh_in_.reset);
  w.set(v.cycle, uint64_t(cycle_));
  if (mwd) {
    const MwdIn &i = in_;
    const MwdReq &q = i.req;
    const uint64_t scalars[] = {i.a_valid, 0, i.b_valid, 0, i.d_valid, 0, i.req_valid,
                                q.dataflow, q.propagate, q.shift, q.a_transpose, q.bd_transpose,
                                q.total_rows, q.tag_valid, q.tag_id, q.flush};
    for (size_t k = 0; k < v.in.size(); ++k)
      if (v.in[k].lanes.empty()) vcd_set(w, v.in[k], scalars[k]);
    vcd_set(w, v.in[1], i.a);
    vcd_set(w, v.in[3], i.b);
    vcd_set(w, v.in[5], i.d);
    const MwdOut &o = mwd_->out();
    const uint64_t outs[] = {o.a_ready, o.b_ready, o.d_ready, o.req_ready, o.resp_valid, 0,
                             o.resp_total_rows, o.resp_tag_valid, o.resp_tag_id, o.resp_last};
    for (size_t k = 0; k < v.out.size(); ++k)
      if (v.out[k].lanes.empty()) vcd_set(w, v.out[k], outs[k]);
    vcd_set(w, v.out[5], o.resp_data);
  } else {
    const MeshIn &i = mesh_in_;
    std::vector<unsigned> df, prop, shift;
    for (const PeControl &c : i.control) {
      df.push_back(c.dataflow);
      prop.push_back(c.propagate);
      shift.push_back(c.shift);
    }
    vcd_set(w, v.in[0], i.a);
    vcd_set(w, v.in[1], i.b);
    vcd_set(w, v.in[2], i.d);
    vcd_set(w, v.in[3], df);
    vcd_set(w, v.in[4], prop);
    vcd_set(w, v.in[5], shift);
    vcd_set(w, v.in[6], i.id);
    vcd_set(w, v.in[7], i.last);
    vcd_set(w, v.in[8], i.valid);
    const MeshOut &o = mesh_->out();
    df.clear();
    prop.clear();
    shift.clear();
    for (const PeControl &c : o.control) {
      df.push_back(c.dataflow);
      prop.push_back(c.propagate);
      shift.push_back(c.shift);
    }
    vcd_set(w, v.out[0], o.b);
    vcd_set(w, v.out[1], o.c);
    vcd_set(w, v.out[2], o.valid);
    vcd_set(w, v.out[3], df);
    vcd_set(w, v.out[4], prop);
    vcd_set(w, v.out[5], shift);
    vcd_set(w, v.out[6], o.id);
    vcd_set(w, v.out[7], o.last);
  }
  for (unsigned r = 0; r < cfg_.rows(); ++r)
    for (unsigned c = 0; c < cfg_.cols(); ++c) {
      const unsigned p = r * cfg_.cols() + c;
      const PeView pv = pe(r, c);
      const Vcd::Pe &id = v.pes[p];
      w.set(id.valid, pv.in.valid);
      w.set(id.state, unsigned(pv.state));
      w.set(id.active, pv.active);
      w.set_signed(id.request, pv.request);
      w.set(id.load_concurrent, pv.load_concurrent);
      w.set(id.drain_concurrent, pv.drain_concurrent);
      w.set(id.occupied, pe_seen_[p]);
      w.set_signed(id.c1, pv.regs.c1);
      w.set_signed(id.c2, pv.regs.c2);
    }
  w.commit(uint64_t(edges_) * 10);
  w.set(v.clock, 0);
  w.commit(uint64_t(edges_) * 10 + 5);
}

}  // namespace systolique
