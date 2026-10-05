// Derived from Gemmini's MeshWithDelays.scala (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/mesh_with_delays.h"

#include "systolique/arith.h"

#include <algorithm>
#include <stdexcept>

namespace systolique {

using namespace arith;

MeshWithDelays::MeshWithDelays(const ArrayConfig &cfg)
    : cfg_(cfg), depth_(cfg.tile_latency + 1) {
  if (const std::string e = cfg.validate(); !e.empty())
    throw std::invalid_argument("configuration " + cfg.name + ": " + e);
  mesh_ = std::make_unique<Mesh>(cfg);
  transposer_ = std::make_unique<Transposer>(cfg.block_size());
  tagq_ = std::make_unique<TagQueue>(cfg.tagq_len(), cfg.tag_bits);
  rowsq_ = std::make_unique<RowsQueue>(cfg.tagq_len());
  regs_.a_buf.assign(cfg.rows(), 0);
  regs_.b_buf.assign(cfg.cols(), 0);
  regs_.d_buf.assign(cfg.cols(), 0);
  next_ = regs_;
  in_.resize(cfg);
  out_.resize(cfg);
  // Registers start at 0: the skew stages hold zero feeds, the de-skew stages zero samples.
  const unsigned groups = std::max(cfg.mesh_rows, cfg.mesh_cols);
  MeshIn zero;
  zero.resize(cfg);
  feed_hist_.assign((groups - 1) * depth_ + 1, zero);
  mesh_in_ = zero;
  mesh_feed_ = zero;
  RespSample rz;
  rz.data.assign(cfg.cols(), 0);
  resp_hist_.assign((cfg.mesh_cols - 1) * depth_, rz);
  resp_next_ = rz;
  settle();
}

// The children are owned through unique_ptr; released here, Mesh last as it was made first.
MeshWithDelays::~MeshWithDelays() {
  rowsq_.reset();
  tagq_.reset();
  transposer_.reset();
  mesh_.reset();
}

void MeshWithDelays::adjust_mesh_feed(MeshIn &) const {}

void MeshWithDelays::set_inputs(const MwdIn &in) {
  in_ = in;
  evaluated_ = false;
}

MeshWithDelays::Comb MeshWithDelays::comb() const {
  const Regs &r = regs_;
  Comb c;
  // input_next_row_into_spatial_array (MeshWithDelays.scala:110)
  c.input_next = r.req_valid && ((r.a_written && r.b_written && r.d_written) || r.req.flush > 0);
  // last_fire = fire_counter === total_fires - 1.U && input_next (MeshWithDelays.scala:112)
  c.last_fire = r.fire_counter == zext(r.req.total_rows - 1, cfg_.total_rows_bits()) &&
                c.input_next;
  // io.req.ready (MeshWithDelays.scala:248)
  c.req_ready = (!r.req_valid || c.last_fire) && tagq_->enq_ready() && rowsq_->enq_ready();
  // io.a/b/d.ready (MeshWithDelays.scala:143-145)
  c.a_ready = !r.a_written || c.input_next || c.req_ready;
  c.b_ready = !r.b_written || c.input_next || c.req_ready;
  c.d_ready = !r.d_written || c.input_next || c.req_ready;
  c.pause = !r.req_valid || !c.input_next;  // MeshWithDelays.scala:149
  return c;
}

// The un-skewed Mesh feed of the current registers (MeshWithDelays.scala:151-195).
void MeshWithDelays::make_feed(MeshIn &f) const {
  const Regs &r = regs_;
  const Comb &c = comb_;
  const unsigned dim = cfg_.block_size();
  const bool os = r.req.dataflow == unsigned(Dataflow::OS);
  // MeshWithDelays.scala:152-154
  const bool a_t = os ? !r.req.a_transpose : r.req.a_transpose;
  const bool b_t = os && r.req.bd_transpose;
  const bool d_t = !os && r.req.bd_transpose;
  const std::vector<int64_t> &tout = transposer_->out_col();
  f.a = a_t ? tout : r.a_buf;                                 // MeshWithDelays.scala:170
  f.b = b_t ? tout : r.b_buf;                                 // MeshWithDelays.scala:171
  if (d_t)                                                    // MeshWithDelays.scala:172-173
    for (unsigned k = 0; k < dim; ++k) f.d[k] = tout[dim - 1 - k];
  else
    f.d = r.d_buf;
  for (unsigned k = 0; k < cfg_.cols(); ++k) {
    f.control[k] = PeControl{r.req.dataflow, r.in_prop, r.result_shift};  // :179-186
    f.valid[k] = !c.pause;                                    // :188-189
    f.id[k] = r.matmul_id;                                    // :191-192
    f.last[k] = c.last_fire;                                  // :194-195
  }
}

// shifted(x, banks = 1): lane group i delayed by i * (tile_latency + 1) (MeshWithDelays.scala:70-91).
void MeshWithDelays::skew() {
  const unsigned tr = cfg_.tile_rows, tc = cfg_.tile_cols;
  for (unsigned i = 0; i < cfg_.mesh_rows; ++i)
    for (unsigned j = 0; j < tr; ++j) mesh_in_.a[i * tr + j] = feed_hist_[i * depth_].a[i * tr + j];
  for (unsigned i = 0; i < cfg_.mesh_cols; ++i) {
    const MeshIn &f = feed_hist_[i * depth_];
    for (unsigned j = 0; j < tc; ++j) {
      const unsigned k = i * tc + j;
      mesh_in_.b[k] = f.b[k];
      mesh_in_.d[k] = f.d[k];
      mesh_in_.control[k] = f.control[k];
      mesh_in_.valid[k] = f.valid[k];
      mesh_in_.id[k] = f.id[k];
      mesh_in_.last[k] = f.last[k];
    }
  }
}

// The value fed into the output de-skew registers: Mux(out_control(0)(0).dataflow === OS,
// out_c, out_b) and lane 0 of out_valid/out_last/out_id (MeshWithDelays.scala:199-203, 232).
void MeshWithDelays::resp_sample(RespSample &s) const {
  const MeshOut &m = mesh_->out();
  s.data = m.control[0].dataflow == unsigned(Dataflow::OS) ? m.c : m.b;
  s.valid = m.valid[0];
  s.last = m.last[0];
  s.id = m.id[0];
}

// out_matmul_id: lane 0 of out_id, de-skewed (MeshWithDelays.scala:232).
unsigned MeshWithDelays::output_id() const {
  const unsigned delay = (cfg_.mesh_cols - 1) * depth_;
  return delay == 0 ? mesh_->out().id[0] : resp_hist_[delay - 1].id;
}

void MeshWithDelays::settle() {
  comb_ = comb();
  make_feed(feed_hist_[0]);
  skew();
  // Outputs (registers only). Resp lanes with no de-skew delay read the Mesh outputs of this
  // cycle, which are settled already.
  MwdOut &o = out_;
  const Comb &c = comb_;
  o.a_ready = c.a_ready;
  o.b_ready = c.b_ready;
  o.d_ready = c.d_ready;
  o.req_ready = c.req_ready;
  RespSample now;
  resp_sample(now);
  auto at = [&](unsigned delay) -> const RespSample & {
    return delay == 0 ? now : resp_hist_[delay - 1];
  };
  // shifted(.., outBanks, reverse = true): group j delayed by (n-1-j) * (tile_latency + 1)
  const unsigned n = cfg_.mesh_cols, tc = cfg_.tile_cols;
  const RespSample &first = at((n - 1) * depth_);
  o.resp_valid = first.valid;   // MeshWithDelays.scala:201
  o.resp_last = first.last;     // :203-204
  const unsigned out_id = output_id();
  for (unsigned j = 0; j < n; ++j) {
    const RespSample &s = at((n - 1 - j) * depth_);
    for (unsigned l = 0; l < tc; ++l) o.resp_data[j * tc + l] = s.data[j * tc + l];
  }
  // io.resp.bits.tag (MeshWithDelays.scala:228-233): the head of the tag queue if its id matches.
  const TagEntry &th = tagq_->deq_bits();
  if (tagq_->deq_valid() && out_id == th.id) {
    o.resp_tag_valid = th.tag_valid;
    o.resp_tag_id = th.tag_id;
  } else {
    o.resp_tag_valid = 0;  // BenchTag.make_this_garbage
    o.resp_tag_id = unsigned(zext(~uint64_t(0), cfg_.tag_bits));
  }
  // io.resp.bits.total_rows (MeshWithDelays.scala:243-244)
  const TagEntry &rh = rowsq_->deq_bits();
  o.resp_total_rows = rowsq_->deq_valid() && out_id == rh.id ? rh.tag_id : cfg_.block_size();
  const std::vector<TagEntry> &all = tagq_->all();
  for (unsigned k = 0; k < cfg_.tagq_len(); ++k) {  // io.tags_in_progress (:249)
    o.tags_valid[k] = all[k].tag_valid;
    o.tags_id[k] = all[k].tag_id;
  }
}

void MeshWithDelays::eval() {
  const Regs &r = regs_;
  Regs &n = next_;
  const Comb &c = comb_;
  const MwdOut &o = out_;
  const unsigned msm = cfg_.max_simultaneous_matmuls(), idw = cfg_.id_bits() + 2;
  const unsigned dim = cfg_.block_size();
  const bool a_fire = in_.a_valid && c.a_ready;
  const bool b_fire = in_.b_valid && c.b_ready;
  const bool d_fire = in_.d_valid && c.d_ready;
  const bool req_fire = in_.req_valid && c.req_ready;
  const bool os_req = r.req.dataflow == unsigned(Dataflow::OS);

  // Queues (MeshWithDelays.scala:219-246): enqueue with every non-flush request, dequeue with the
  // last response row whose output id matches the head.
  const unsigned out_id = output_id();
  // matmul_id_of_output / _of_current (MeshWithDelays.scala:219-220)
  const unsigned id_of_output = unsigned(wrapping_add_int(
      r.matmul_id, in_.req.dataflow == unsigned(Dataflow::OS) ? 3 : 2, msm, idw));
  const unsigned id_of_current = unsigned(wrapping_add_int(r.matmul_id, 1, msm, idw));
  QueueIn q;
  q.enq_valid = req_fire && in_.req.flush == 0;  // :223, :238
  q.reset = in_.reset;
  q.enq_bits = TagEntry{in_.req.tag_valid, in_.req.tag_id, id_of_output};
  q.deq_ready = o.resp_valid && o.resp_last && out_id == tagq_->deq_bits().id;  // :235
  tagq_->set_inputs(q);
  tagq_->eval();
  q.enq_bits = TagEntry{0, in_.req.total_rows, id_of_current};
  q.deq_ready = o.resp_valid && o.resp_last && out_id == rowsq_->deq_bits().id;  // :246
  rowsq_->set_inputs(q);
  rowsq_->eval();

  // Transposer input (MeshWithDelays.scala:152-161): MuxCase(a_buf, b first, then d reversed).
  const bool a_t = os_req ? !r.req.a_transpose : r.req.a_transpose;
  const bool b_t = os_req && r.req.bd_transpose;
  const bool d_t = !os_req && r.req.bd_transpose;
  TransposerIn ti;
  ti.in_valid = !c.pause && (a_t || b_t || d_t);  // :157
  ti.reset = in_.reset;
  ti.in_row.resize(dim);
  for (unsigned k = 0; k < dim; ++k)
    ti.in_row[k] = b_t ? r.b_buf[k] : (d_t ? r.d_buf[dim - 1 - k] : r.a_buf[k]);
  transposer_->set_inputs(ti);
  transposer_->eval();

  n = r;
  n.result_shift = r.req.shift;  // RegNext(req.bits.pe_control.shift) (:183)
  // MeshWithDelays.scala:114-141 (total_fires is the request register before this edge)
  const unsigned total_fires = r.req.total_rows;
  if (req_fire) {
    n.req_valid = true;
    n.req = in_.req;
    n.in_prop = in_.req.propagate ^ r.in_prop;
    n.matmul_id = unsigned(wrapping_add_int(r.matmul_id, 1, msm, idw));
  } else if (c.last_fire) {
    n.req_valid = r.req.flush > 1;
    n.req.flush = (r.req.flush - 1) & 3;
  }
  if (c.input_next) {
    n.a_written = n.b_written = n.d_written = false;
    // Util.wrappingAdd(fire_counter, 1.U, total_fires) (Util.scala:17-32): rw-bit arithmetic,
    // the result truncated to the counter's width.
    const unsigned rw = cfg_.total_rows_bits(), fw = cfg_.fire_counter_bits();
    const uint64_t u = r.fire_counter, max = zext(uint64_t(total_fires) - 1, rw);
    uint64_t next;
    if (max == 0)
      next = 0;
    else if (u >= zext(zext(max - 1, rw) + 1, rw))
      next = zext(zext(1 - zext(max - u, rw), rw) - 1, rw);
    else
      next = u + 1;
    n.fire_counter = unsigned(zext(next, fw));
  }
  if (a_fire) {  // RegEnable(io.a.bits, io.a.fire) (:100-102, 131-141)
    n.a_written = true;
    n.a_buf = in_.a;
  }
  if (b_fire) {
    n.b_written = true;
    n.b_buf = in_.b;
  }
  if (d_fire) {
    n.d_written = true;
    n.d_buf = in_.d;
  }
  if (in_.reset) {  // RegInit registers and req.valid (:95, 98, 104-106, 251-253)
    n.req_valid = false;
    n.matmul_id = n.fire_counter = 0;
    n.a_written = n.b_written = n.d_written = false;
  }

  // This cycle's sample enters the output de-skew registers at the edge.
  resp_sample(resp_next_);
  // The Mesh, fed from this module's registers.
  mesh_feed_ = mesh_in_;
  adjust_mesh_feed(mesh_feed_);
  mesh_->set_inputs(mesh_feed_);
  mesh_->eval();
  evaluated_ = true;
}

void MeshWithDelays::tick() {
  if (!evaluated_) eval();
  std::swap(regs_, next_);
  transposer_->tick();
  tagq_->tick();
  rowsq_->tick();
  mesh_->tick();
  // The skew and de-skew stages shift; entry 0 of the feed history is recomputed by settle().
  if (!resp_hist_.empty()) {
    resp_hist_.pop_back();
    resp_hist_.push_front(resp_next_);
  }
  feed_hist_.pop_back();
  feed_hist_.push_front(feed_hist_.front());
  settle();
  evaluated_ = false;
}

void MeshWithDelays::registers(std::vector<std::pair<std::string, int64_t>> &out) const {
  auto r = [&](const std::string &n, int64_t v) { out.emplace_back(n, v); };
  const Regs &g = regs_;
  r("req_valid", g.req_valid);
  r("req_bits_pe_control_dataflow", g.req.dataflow);
  r("req_bits_pe_control_shift", g.req.shift);
  r("req_bits_a_transpose", g.req.a_transpose);
  r("req_bits_bd_transpose", g.req.bd_transpose);
  r("req_bits_total_rows", g.req.total_rows);
  r("req_bits_flush", g.req.flush);
  r("matmul_id", g.matmul_id);
  r("fire_counter", g.fire_counter);
  r("a_written", g.a_written);
  r("b_written", g.b_written);
  r("d_written", g.d_written);
  r("in_prop", g.in_prop);
  r("result_shift", g.result_shift);
  const unsigned tr = cfg_.tile_rows, tc = cfg_.tile_cols, dim = cfg_.block_size();
  for (unsigned k = 0; k < cfg_.rows(); ++k)
    r("a_buf_" + std::to_string(k / tr) + "_" + std::to_string(k % tr), g.a_buf[k]);
  for (unsigned k = 0; k < cfg_.cols(); ++k) {
    const std::string s = std::to_string(k / tc) + "_" + std::to_string(k % tc);
    r("b_buf_" + s, g.b_buf[k]);
    r("d_buf_" + s, g.d_buf[k]);
  }
  r("transposer.counter", transposer_->counter());
  r("transposer.dir", transposer_->dir());
  for (unsigned y = 0; y < dim; ++y)
    for (unsigned x = 0; x < dim; ++x)
      r("transposer.pes_" + std::to_string(y) + "_" + std::to_string(x) + ".reg_",
        transposer_->reg(y, x));
  r("tagq.raddr", tagq_->raddr());
  r("tagq.waddr", tagq_->waddr());
  r("tagq.len", tagq_->len());
  for (unsigned k = 0; k < cfg_.tagq_len(); ++k) {
    const std::string p = "tagq.regs_" + std::to_string(k) + "_";
    r(p + "tag_valid", tagq_->all()[k].tag_valid);
    r(p + "tag_id", tagq_->all()[k].tag_id);
    r(p + "id", tagq_->all()[k].id);
  }
  r("total_rows_q.enq_ptr_value", rowsq_->enq_ptr());
  r("total_rows_q.deq_ptr_value", rowsq_->deq_ptr());
  r("total_rows_q.maybe_full", rowsq_->maybe_full());
}

}  // namespace systolique
