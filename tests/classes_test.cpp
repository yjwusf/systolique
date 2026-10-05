// Unit tests of the component classes and of the clocking rules (AGENTS.md):
//   - PE: OS and WS arithmetic and register updates, two-phase (eval changes no register, a
//     repeated eval gives the same result), registers hold while in_valid is low
//   - Tile: tree reduction sums the column's products and in_b
//   - Transposer: a matrix shifted in during one pass comes out transposed during the next
//   - TagQueue / RowsQueue: order, full/empty, garbage tags, reset
//   - Mesh, MeshWithDelays: outputs depend on registers only (set_inputs + eval leave out()
//     unchanged), eval is idempotent, tick() without eval() equals eval() + tick()
//   - SystolicArray: no global state (two arrays fed the same stimulus agree while a third
//     runs something else interleaved), reset() reproduces a run, invalid configurations throw
//   - construction and destruction of every class for every configuration (RAII; run under the
//     sanitizers by ci/local.sh)
// Prints CLASSES ok checks=<n> or the first failure.
#include "run.h"
#include "stimulus.h"

#include "systolique/arith.h"
#include "systolique/mesh.h"
#include "systolique/mesh_with_delays.h"
#include "systolique/pe.h"
#include "systolique/systolic_array.h"
#include "systolique/tag_queue.h"
#include "systolique/tile.h"
#include "systolique/transposer.h"

#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace systolique;

namespace {

unsigned checks = 0;
std::string first_failure;

void expect(bool ok, const std::string &what) {
  ++checks;
  if (!ok && first_failure.empty()) first_failure = what;
}

void test_pe() {
  const ArrayConfig &cfg = *find_config("dim4");  // BOTH, int8 / int20 / int32
  PE pe(cfg);
  PeIn in;
  in.a = 3;
  in.b = 4;
  in.d = 5;
  in.control = PeControl{0, 1, 0};  // OS, propagate 1: c2 accumulates, c1 takes d (PE.scala:103-109)
  in.valid = 1;
  pe.set_inputs(in);
  pe.eval();
  expect(pe.regs().c1 == 0 && pe.regs().c2 == 0, "PE: eval changed a register");
  expect(pe.out().c == 0 && pe.out().b == 4, "PE: OS outputs before the edge");
  pe.eval();
  expect(pe.out().c == 0 && pe.regs().c2 == 0, "PE: a repeated eval differs");
  pe.tick();
  expect(pe.regs().c1 == 5 && pe.regs().c2 == 12 && pe.regs().last_s == 1, "PE: OS update");
  pe.tick();  // same inputs, tick() evaluates itself
  expect(pe.regs().c2 == 24 && pe.regs().c1 == 5, "PE: OS accumulate");
  in.valid = 0;
  in.control.propagate = 0;
  pe.set_inputs(in);
  pe.tick();
  expect(pe.regs().c1 == 5 && pe.regs().c2 == 24 && pe.regs().last_s == 1,
         "PE: registers must hold while in_valid is low (PE.scala:141-146)");
  // OS with a flip and a shift: out_c = round(c, shift) on the first cycle after the flip.
  in.valid = 1;
  in.control = PeControl{0, 0, 2};  // propagate 0 now: out_c = c2 >> 2 rounded (24 / 4 = 6)
  in.a = 0;
  pe.set_inputs(in);
  pe.eval();
  expect(pe.out().c == 6, "PE: OS rounding shift on a propagate flip");
  // WS: propagate 1 multiplies with c2, loads c1 from d (PE.scala:119-125).
  PE ws(cfg);
  in = PeIn{};
  in.control = PeControl{1, 0, 0};
  in.valid = 1;
  in.d = 7;  // propagate 0: c2 := d
  ws.set_inputs(in);
  ws.tick();
  in.control.propagate = 1;
  in.a = -2;
  in.b = 10;
  in.d = 1;
  ws.set_inputs(in);
  ws.eval();
  expect(ws.out().b == 10 + (-2) * 7, "PE: WS out_b = b + a * weight");
  ws.tick();
  expect(ws.regs().c1 == 1 && ws.regs().c2 == 7, "PE: WS stationary register unchanged");
  // The MacUnit result wraps at the output width (PE.scala:23).
  PE w(cfg);
  in = PeIn{};
  in.control = PeControl{0, 1, 0};
  in.valid = 1;
  in.a = 127;
  in.b = 127;
  for (int k = 0; k < 40; ++k) {
    w.set_inputs(in);
    w.tick();
  }
  expect(w.regs().c2 == arith::sext(40 * 127 * 127, 20), "PE: OS sum wraps at 20 bits");
}

void test_tile() {
  const ArrayConfig &cfg = *find_config("ws_tree");  // WS-only, 2x2 PEs per tile, tree reduction
  Tile t(cfg);
  TileIn in;
  in.a = {3, -5};
  in.b = {100, 200};
  in.d = {0, 0};
  in.control.assign(2, PeControl{1, 0, 0});
  in.valid = {1, 1};
  in.id = {0, 0};
  in.last = {0, 0};
  // Load weights: propagate 0 writes c2 from d; d flows down the column through out_c, so the
  // bottom PE takes what the top one held. Two cycles fill a column.
  in.d = {2, 4};
  t.set_inputs(in);
  t.tick();
  in.d = {6, 8};
  t.set_inputs(in);
  t.tick();
  // Column j: PE(0,j).c2 = second d, PE(1,j).c2 = first d's ... out_c of PE 0 before the edge.
  const int64_t w00 = t.pe(0, 0).regs().c2, w10 = t.pe(1, 0).regs().c2;
  in.control.assign(2, PeControl{1, 1, 0});  // multiply with c2
  t.set_inputs(in);
  t.eval();
  expect(t.out().b[0] == 100 + 3 * w00 + (-5) * w10,
         "Tile: tree reduction = in_b + sum of the column's products (Tile.scala:117-121)");
}

void test_transposer() {
  const unsigned dim = 4;
  Transposer tr(dim);
  int64_t m[4][4];
  for (unsigned r = 0; r < dim; ++r)
    for (unsigned c = 0; c < dim; ++c) m[r][c] = 10 * r + c + 1;
  TransposerIn in;
  in.in_valid = true;
  for (unsigned r = 0; r < dim; ++r) {  // pass 1: shift M in, row by row
    in.in_row.assign(m[r], m[r] + dim);
    tr.set_inputs(in);
    tr.tick();
  }
  expect(tr.dir() == 1 && tr.counter() == 0, "Transposer: dir flips after DIM rows");
  for (unsigned k = 0; k < dim; ++k) {  // pass 2: out_col is column k of M
    bool ok = true;
    for (unsigned r = 0; r < dim; ++r) ok = ok && tr.out_col()[r] == m[r][k];
    expect(ok, "Transposer: column " + std::to_string(k) + " of the first matrix");
    in.in_row.assign(dim, 0);
    tr.set_inputs(in);
    tr.tick();
  }
  in.in_valid = false;
  in.reset = true;
  tr.set_inputs(in);
  tr.tick();
  expect(tr.dir() == 0 && tr.counter() == 0, "Transposer: reset");
}

void test_queues() {
  TagQueue q(3, 8);
  QueueIn in;
  for (unsigned k = 0; k < 3; ++k) {
    in.enq_valid = true;
    in.enq_bits = TagEntry{1, 10 + k, k};
    q.set_inputs(in);
    q.tick();
  }
  expect(!q.enq_ready() && q.len() == 3, "TagQueue: full after 3");
  in.enq_valid = true;
  in.enq_bits = TagEntry{1, 99, 9};
  q.set_inputs(in);
  q.tick();  // not accepted
  expect(q.len() == 3 && q.deq_bits().tag_id == 10, "TagQueue: enq ignored when full");
  in.enq_valid = false;
  in.deq_ready = true;
  q.set_inputs(in);
  q.tick();
  expect(q.deq_bits().tag_id == 11 && q.len() == 2, "TagQueue: FIFO order");
  expect(q.all()[0].tag_valid == 0 && q.all()[0].tag_id == 255, "TagQueue: garbage after deq");
  in.deq_ready = false;
  in.reset = true;
  q.set_inputs(in);
  q.tick();
  expect(q.len() == 0 && !q.deq_valid(), "TagQueue: reset");

  RowsQueue r(2);
  in = QueueIn{};
  in.enq_valid = true;
  in.enq_bits = TagEntry{0, 5, 1};
  r.set_inputs(in);
  r.tick();
  in.enq_bits = TagEntry{0, 6, 2};
  r.set_inputs(in);
  r.tick();
  expect(r.full() && r.deq_bits().tag_id == 5, "RowsQueue: full, head");
  in.enq_valid = false;
  in.deq_ready = true;
  r.set_inputs(in);
  r.tick();
  r.tick();
  expect(r.empty(), "RowsQueue: empty after two dequeues");
}

// Random inputs for one cycle.
MeshIn random_mesh_in(const ArrayConfig &cfg, std::mt19937_64 &rng) {
  MeshIn in;
  in.resize(cfg);
  for (auto &v : in.a) v = arith::sext(int64_t(rng()), cfg.in_bits);
  for (unsigned k = 0; k < cfg.cols(); ++k) {
    in.b[k] = arith::sext(int64_t(rng()), cfg.in_bits);
    in.d[k] = arith::sext(int64_t(rng()), cfg.in_bits);
    in.control[k] = PeControl{unsigned(rng() & 1), unsigned(rng() & 1),
                              unsigned(rng() % (1u << cfg.shift_bits()))};
    in.id[k] = unsigned(rng() % (1u << cfg.id_bits()));
    in.last[k] = unsigned(rng() & 1);
    in.valid[k] = unsigned(rng() % 3 != 0);
  }
  return in;
}

void test_mesh(const ArrayConfig &cfg) {
  Mesh a(cfg), b(cfg);
  std::mt19937_64 rng(5);
  for (unsigned t = 0; t < 60; ++t) {
    const MeshIn in = random_mesh_in(cfg, rng);
    const MeshOut before = a.out();
    a.set_inputs(in);
    a.eval();
    a.eval();
    expect(a.out() == before, cfg.name + " Mesh: outputs changed with this cycle's inputs");
    a.tick();
    b.set_inputs(in);
    b.tick();  // no explicit eval
    expect(a.out() == b.out(), cfg.name + " Mesh: tick() without eval() differs");
  }
  std::vector<std::pair<std::string, int64_t>> ra, rb;
  a.registers(ra);
  b.registers(rb);
  expect(ra == rb, cfg.name + " Mesh: registers differ");
}

bool same(const MwdOut &x, const MwdOut &y) {
  return x.a_ready == y.a_ready && x.b_ready == y.b_ready && x.d_ready == y.d_ready &&
         x.req_ready == y.req_ready && x.resp_valid == y.resp_valid &&
         x.resp_total_rows == y.resp_total_rows && x.resp_tag_valid == y.resp_tag_valid &&
         x.resp_tag_id == y.resp_tag_id && x.resp_last == y.resp_last &&
         x.resp_data == y.resp_data && x.tags_valid == y.tags_valid && x.tags_id == y.tags_id;
}

void test_mwd(const ArrayConfig &cfg) {
  MeshWithDelays a(cfg), b(cfg);
  std::mt19937_64 rng(9);
  std::vector<Op> ops = random_ops(cfg, rng, 12);
  MwdDriver drv(cfg, ops, DriverOptions{0.2, 0, 3});
  const auto spec = port_specs(cfg, Top::MeshWithDelays);
  Frame f = make_frame(spec);
  for (unsigned t = 0; t < 400; ++t) {
    mwd_out_to_frame(cfg, a.out(), f);
    MwdIn in;
    in.resize(cfg);
    if (t < 4) {
      in.reset = true;
    } else {
      if (!drv.next(t - 4, f, f)) break;
      frame_to_mwd_in(cfg, f, in);
    }
    const MwdOut before = a.out();
    a.set_inputs(in);
    a.eval();
    a.eval();
    expect(same(a.out(), before), cfg.name + " MeshWithDelays: outputs changed with the inputs");
    a.tick();
    b.set_inputs(in);
    b.tick();
    expect(same(a.out(), b.out()), cfg.name + " MeshWithDelays: tick() without eval() differs");
  }
  std::vector<std::pair<std::string, int64_t>> ra, rb;
  a.registers(ra);
  b.registers(rb);
  expect(ra == rb, cfg.name + " MeshWithDelays: registers differ");
}

void test_independence(const ArrayConfig &cfg) {
  // Two arrays fed the same stimulus agree in every cycle while a third, interleaved, runs a
  // different one: nothing is shared between instances.
  SystolicArray x(cfg), y(cfg), z(cfg);
  std::mt19937_64 r1(1), r2(2);
  MwdDriver d1(cfg, random_ops(cfg, r1, 10), DriverOptions{0.1, 0, 1});
  MwdDriver d2(cfg, random_ops(cfg, r2, 10), DriverOptions{0.3, 0, 2});
  const auto spec = port_specs(cfg, Top::MeshWithDelays);
  Frame f1 = make_frame(spec), f2 = f1;
  x.reset();
  y.reset();
  z.reset();
  bool ok = true;
  for (unsigned t = 0; t < 300; ++t) {
    mwd_out_to_frame(cfg, x.out(), f1);
    mwd_out_to_frame(cfg, z.out(), f2);
    const bool more1 = d1.next(t, f1, f1);
    d2.next(t, f2, f2);
    MwdIn in1, in2;
    frame_to_mwd_in(cfg, more1 ? f1 : make_frame(spec), in1);
    frame_to_mwd_in(cfg, f2, in2);
    const MwdOut ox = x.step(in1);
    z.step(in2);
    const MwdOut oy = y.step(in1);
    ok = ok && same(ox, oy);
  }
  expect(ok, cfg.name + " SystolicArray: instances are not independent");
  const Accounting ax = x.accounting(), ay = y.accounting();
  expect(ax.total.n == ay.total.n, cfg.name + " SystolicArray: accounting differs");
  // reset() reproduces the run exactly.
  SystolicArray w(cfg);
  std::mt19937_64 r3(3);
  const std::vector<Op> ops = random_ops(cfg, r3, 8);
  MwdDriver e1(cfg, ops, DriverOptions{0.2, 0, 4}), e2(cfg, ops, DriverOptions{0.2, 0, 4});
  const ArrayRun g1 = run_stimulus(w, Top::MeshWithDelays, e1);
  const ArrayRun g2 = run_stimulus(w, Top::MeshWithDelays, e2);
  expect(g1.ok && g2.ok && g1.rows == g2.rows, cfg.name + " SystolicArray: reset() does not reproduce a run");
}

void test_lifetimes() {
  for (const ArrayConfig &cfg : named_configs()) {
    for (int k = 0; k < 3; ++k) {
      PE pe(cfg);
      Tile tile(cfg);
      Mesh mesh(cfg);
      MeshWithDelays mwd(cfg);
      SystolicArray a(cfg);
      ArrayOptions o;
      o.interface = Interface::Mesh;
      SystolicArray m(cfg, o);
      a.reset();
      m.reset();
    }
    ++checks;
  }
  ArrayConfig bad = *find_config("dim4");
  bad.mesh_rows = 3;
  bool threw = false;
  try {
    SystolicArray a(bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  expect(threw, "an invalid configuration must throw std::invalid_argument");
  threw = false;
  try {
    SystolicArray a(*find_config("dim4"));
    a.set_mesh_inputs(MeshIn{});
  } catch (const std::logic_error &) {
    threw = true;
  }
  expect(threw, "the bare Mesh interface needs Interface::Mesh");
}

}  // namespace

int main() {
  test_pe();
  test_tile();
  test_transposer();
  test_queues();
  for (const ArrayConfig &cfg : named_configs()) {
    test_mesh(cfg);
    test_mwd(cfg);
    test_independence(cfg);
  }
  test_lifetimes();
  if (!first_failure.empty()) {
    std::printf("FAIL %s\n", first_failure.c_str());
    return 1;
  }
  std::printf("CLASSES ok checks=%u\n", checks);
  return 0;
}
