// The splitter and its micro-ops (docs/micro_ops.md):
//   directed   two command streams into ExecuteUnit (WS: config, preload, compute_preloaded; OS:
//              the same with an OS config), every command micro-op's cycles checked against values
//              derived by hand from the Chisel (ExecuteController.scala, Scratchpad.scala,
//              MeshWithDelays.scala of Gemmini v0.7.2; the derivations are next to the numbers)
//   matmul     64x64x64 WS and OS matmuls through the Engine: C equals a plain C++ matmul, the
//              micro-op counts, every MAC PE-cycle attributed to a compute micro-op, per-op and
//              per-micro-op PE-cycles summing to the totals, useful MACs = M N K exactly, the
//              weight of every WS MAC traced to the preload micro-op that loaded it
//   invariance recording micro-ops and writing the Engine's VCD change no output in any cycle
//   classes    eval() changes nothing, tick() alone = eval() + tick(), two Engines are independent
// Prints "MICRO_OPS ok checks=<n>" or the failures.
#include "fe_ports.h"
#include "fe_run.h"
#include "fe_stimulus.h"

#include "systolique/engine.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace systolique;

namespace {

unsigned g_checks = 0, g_failed = 0;
void expect(bool ok, const std::string &what) {
  ++g_checks;
  if (!ok && g_failed++ < 30)
    std::printf("FAIL %s\n", what.c_str());
}
void expect_eq(int64_t got, int64_t want, const std::string &what) {
  expect(got == want, what + ": " + std::to_string(got) + ", expected " + std::to_string(want));
}

UopKind kind_of(unsigned f) {
  switch (f) {
    case CONFIG_CMD: return UopKind::Config;
    case PRELOAD_CMD: return UopKind::Preload;
    case COMPUTE_AND_FLIP_CMD: return UopKind::ComputePreloaded;
    case COMPUTE_AND_STAY_CMD: return UopKind::ComputeAccumulated;
    default: return UopKind::OtherCmd;
  }
}

// The commands offered one per cycle from cycle 0 (each as soon as the previous one fired), with
// rob ids 16, 17, ... (the execute queue's), run for `cycles` cycles.
struct Directed {
  MicroOpTable T;
  std::unique_ptr<ExecuteUnit> ex;
};
void run_directed(Directed &d, const std::vector<Command> &cmds, int cycles) {
  const FrontendConfig cfg = FrontendConfig::gemmini_default();
  d.ex = std::make_unique<ExecuteUnit>(cfg);
  d.ex->set_micro_ops(&d.T);
  const int op = d.T.add_op(Operation());
  for (const Command &c : cmds) {
    MicroOp u;
    u.op = op;
    u.kind = kind_of(c.funct);
    u.funct = c.funct;
    d.T.add(u);
  }
  size_t next = 0;
  for (int t = 0; t < cycles; ++t) {
    ExecuteTopIn in;
    if (next < cmds.size()) {
      in.cmd_valid = true;
      in.cmd_funct = cmds[next].funct;
      in.cmd_rs1 = cmds[next].rs1;
      in.cmd_rs2 = cmds[next].rs2;
      in.cmd_rob_id = unsigned(16 + next);
      in.cmd_bookkeeping.op = op;
      in.cmd_bookkeeping.uop = int64_t(next);
    }
    d.ex->set_inputs(in);
    d.ex->eval();
    if (in.cmd_valid && d.ex->out().cmd_ready)
      ++next;
    d.ex->tick();
  }
}

void directed_ws() {
  // config_ex WS; preload B (scratchpad rows 16..31, bank 0) with C to accumulator row 0;
  // compute_preloaded A (rows 0..15), B garbage.
  Directed d;
  run_directed(d, {cmd_config_ex(kWS), cmd_preload(16, acc_addr(0, false)), cmd_compute(true, 0, kGarbageAddr)}, 120);
  const MicroOp &cfg = d.T.at(0), &pre = d.T.at(1), &cmp = d.T.at(2);
  const auto &req = d.ex->ex().array().requests();
  // A command that fires at io.cmd in cycle t is in the TransposePreloadUnroller's queue from t+1
  // (TransposePreloadUnroller.scala:30-35), passes into the ExecuteController's MultiHeadedQueue
  // at the end of t+1 and is its head from t+2 (ExecuteController.scala:69).
  expect_eq(cfg.issued, 0, "WS config issued");
  // The head is a config, nothing in progress: config_ex pops it and completes it in the same
  // cycle (ExecuteController.scala:541-582).
  expect_eq(cfg.started, 2, "WS config started");
  expect_eq(cfg.popped, 2, "WS config popped");
  expect_eq(cfg.completed, 2, "WS config completed");
  // The preload (head from 3) waits for cmd.valid(1) (:585): the compute is in the queue from 4.
  expect_eq(pre.issued, 1, "WS preload issued");
  expect_eq(pre.started, 4, "WS preload started");
  // A single preload in WS feeds D only (start_inputting_d; a and b go through the transposer
  // only when transposed, :585-597); total_rows = DIM (:285); one D row per cycle, bottom row first.
  expect_eq(pre.reads, 16, "WS preload rows read");
  expect_eq(pre.first_read, 4, "WS preload first row read");
  expect_eq(pre.last_read, 19, "WS preload last row read");
  // about_to_fire_all_rows with the last D row (:405-410, 639-648): popped at 19
  expect_eq(pre.popped, 19, "WS preload popped");
  // Row 0 read in 4: the bank registers the address (Scratchpad.scala:141-161), its data enters
  // the spad_read_delay = 4 stage Pipeline in 5 (:496) and is at the ExecuteController in 9,
  // where the first row's control entry fires the request (ExecuteController.scala:883):
  expect_eq(pre.accept, 9, "WS preload request accepted");
  // MeshWithDelays latches the row buffers at the handshake; the row enters the Mesh next cycle
  // (MeshWithDelays.scala:110, 123-141)
  expect(pre.request == 0 && req[0].first_in == 10 && req[0].last_in == 25, "WS preload rows into the Mesh 10..25");
  // The compute: the preload's pass ends in 19, the FSM waits in 20 with the compute at the head
  // and no preload behind it: a single mul (:612-620), A rows read 20..35, popped 35, completed
  // from pending_completed_rob_ids in 36 (:675-679, 982-990).
  expect_eq(cmp.started, 20, "WS compute started");
  expect_eq(cmp.first_read, 20, "WS compute first A row");
  expect_eq(cmp.last_read, 35, "WS compute last A row");
  expect_eq(cmp.reads, 16, "WS compute rows read (B is garbage: no B reads)");
  expect_eq(cmp.popped, 35, "WS compute popped");
  expect_eq(cmp.completed, 36, "WS compute completed");
  // its rows: the previous request's last row fires in 24 (req.ready with last_fire), A row 0's
  // data is at the controller 5 cycles after its read in 20: accepted in 25
  expect_eq(cmp.accept, 25, "WS compute request accepted");
  // WS: the preload's tag (C) is on the rows the compute's request pushes out
  // (MeshWithDelays.scala:219: tag id + 2 in WS); 16 result rows written to the accumulator,
  // which holds them two cycles after the last write (AccumulatorMem.scala:110-125); the
  // preload completes with the last result row (ExecuteController.scala:970-977).
  expect(req.size() >= 2 && pre.result_first == req[1].first_out && pre.result_last == req[1].last_out,
         "WS preload's result rows are the compute request's output rows");
  expect_eq(pre.result_rows, 16, "WS preload result rows");
  expect_eq(pre.wb_rows, 16, "WS preload rows written back");
  expect_eq(pre.wb_done, pre.result_last + 2, "WS write-back done (accumulator pipeline)");
  expect_eq(pre.completed, pre.result_last, "WS preload completed with its last result row");
}

void directed_os() {
  // config_ex OS; preload D garbage (zeros) with C to scratchpad row 48; compute_preloaded A
  // (rows 0..15), B (rows 16..31).
  Directed d;
  run_directed(d, {cmd_config_ex(kOS), cmd_preload(kGarbageAddr, 48), cmd_compute(true, 0, 16)}, 120);
  const MicroOp &pre = d.T.at(1), &cmp = d.T.at(2);
  expect_eq(pre.started, 4, "OS preload started");
  // D is garbage: preload_zeros, no D reads (:152, 425); in OS A goes through the transposer one
  // request early, so the preload's pass reads the compute's A (a_address_place = 1, :124)
  expect_eq(pre.reads, 0, "OS preload rows read");
  expect_eq(cmp.first_read, 4, "OS compute's A rows read in the preload's pass");
  expect_eq(cmp.started, 20, "OS compute started");
  expect_eq(cmp.last_read, 35, "OS compute's last B row");
  expect_eq(cmp.reads, 32, "OS compute rows read (A in the preload's pass, B in its own)");
  expect_eq(cmp.completed, 36, "OS compute completed");
  // No command left, a matmul in progress, OS: flush (:627-630) in 37; the flush request is
  // offered once the control queue is empty (:192, 883): the compute's last row is fed in 40.
  std::vector<const MicroOp *> flushes;
  for (const MicroOp &u : d.T.all())
    if (u.kind == UopKind::Flush)
      flushes.push_back(&u);
  expect(!flushes.empty() && flushes[0]->accept == 41, "OS first flush accepted in 41");
  // Each flush runs DIM rows; FLUSHING returns to waiting with req.ready (:687-692) and, while the
  // preload's tag is still in the array, the FSM flushes again: 59 and 77.
  expect(flushes.size() == 3 && flushes[1]->accept == 59 && flushes[2]->accept == 77, "OS flushes at 41, 59, 77");
  for (const MicroOp *f : flushes)
    expect(f->parent == cmp.id && f->op == cmp.op, "a flush belongs to the last command popped");
  // OS: C leaves the array with the rows of the request two after the preload's (tag id + 3,
  // MeshWithDelays.scala:219), the first flush; written to the scratchpad (done in the same cycle)
  const auto &req = d.ex->ex().array().requests();
  expect(req.size() >= 3 && pre.result_first == req[2].first_out, "OS C leaves with the first flush's rows");
  expect_eq(pre.wb_rows, 16, "OS rows written");
  expect_eq(pre.wb_done, pre.result_last, "OS scratchpad write-back done");
  expect_eq(pre.completed, pre.result_last, "OS preload completed with its last result row");
}

void matmul(unsigned df) {
  const std::string n = df == kWS ? "WS" : "OS";
  EngineOptions opt;
  Engine e(opt);
  Matmul m;
  m.dataflow = df;
  m.bias = true;
  m.full_c = true;
  const int op = e.submit(m);
  // provenance: the weight (WS) in the active register of a MAC PE is the preload micro-op's
  std::map<int64_t, int64_t> weight_preload;  // compute uop -> preload uop that loaded its weights
  unsigned prov_checks = 0, prov_bad = 0;
  e.set_hook([&](const Engine &en, const CtrlTopIn &, const CtrlTopOut &) {
    if (df != kWS)
      return;
    const SystolicArray &a = en.array();
    for (unsigned r = 0; r < 16; r += 5)
      for (unsigned c = 0; c < 16; c += 3) {
        const PeView v = a.pe(r, c);
        if (v.state != PeState::Mac)
          continue;
        const int64_t cu = a.requests()[size_t(v.request)].uop;
        const RegSource &s = v.src[v.active];
        ++prov_checks;
        auto it = weight_preload.emplace(cu, s.uop).first;
        if (s.uop < 0 || it->second != s.uop || en.micro_ops().at(s.uop).kind != UopKind::Preload ||
            en.micro_ops().at(s.uop).op != op)
          ++prov_bad;
      }
  });
  const int64_t cycles = e.run();
  expect(cycles > 0, n + " run");
  expect(e.result(op) == e.reference(op), n + " C equals a plain C++ matmul");
  expect(e.check().empty(), n + " Engine::check: " + e.check());
  const Accounting a = e.accounting();
  expect_eq(int64_t(a.total.macs()), 64 * 64 * 64, n + " useful MACs");
  expect_eq(int64_t(a.per_op.at(op).macs()), 64 * 64 * 64, n + " the operation's MAC PE-cycles");
  std::map<UopKind, unsigned> count;
  uint64_t reads = 0, read_rows = 0, result_rows = 0, written = 0;
  for (const MicroOp &u : e.micro_ops().all()) {
    ++count[u.kind];
    expect(u.op == op, n + " every micro-op belongs to the matmul");
    if (u.kind == UopKind::ComputePreloaded || u.kind == UopKind::ComputeAccumulated) {
      reads += u.reads;
      const auto it = a.per_uop.find(u.id);
      expect(it != a.per_uop.end() && it->second.macs() == 16 * 256, n + " a compute micro-op's MACs = DIM^3");
      expect(u.issued >= 0 && u.issued <= u.started && u.started <= u.popped && u.popped <= u.completed,
             n + " compute cycles in order");
      // A request can be accepted before its A rows are read (the garbage B operand fires first,
      // ExecuteController.scala:841-883) and its rows enter when every buffer has a row
      // (MeshWithDelays.scala:110): when the DMA's mvin writes hold A's bank, later.
      expect(u.first_in >= u.accept + 1 && u.first_in > u.first_read && u.last_in >= u.first_in + 15,
             n + " compute #" + std::to_string(u.id) + ": read " + std::to_string(u.first_read) + ", accept " + std::to_string(u.accept) + ", in " + std::to_string(u.first_in) + ".." + std::to_string(u.last_in));
    }
    if (u.kind == UopKind::Preload)
      reads += u.reads;
    if (u.kind == UopKind::OperandRead)
      ++read_rows;
    if (u.kind == UopKind::ResultRow) {
      ++result_rows;
      written += u.written;
    }
  }
  expect_eq(count[UopKind::Preload], 64, n + " preload micro-ops");
  expect_eq(count[UopKind::ComputePreloaded] + count[UopKind::ComputeAccumulated], 64, n + " compute micro-ops");
  expect_eq(int64_t(read_rows), int64_t(reads), n + " operand-read micro-ops = rows read by the commands");
  // WS (LoopMatmul): A rows for every compute, B rows for the 16 preloads with i == 0
  // (LoopMatmul.scala:412-430), D through mvin3 into the accumulator; OS: A and B for every compute.
  expect_eq(int64_t(reads), df == kWS ? 64 * 16 + 16 * 16 : 64 * 32, n + " operand rows read");
  // WS: every preload names C (LoopMatmul.scala:425); OS: only k == K-1 (gemmini.h:458)
  expect_eq(int64_t(result_rows), df == kWS ? 64 * 16 : 16 * 16, n + " result rows");
  expect_eq(int64_t(written), int64_t(result_rows), n + " result rows written back");
  expect_eq(count[UopKind::Mvout], 16, n + " mvouts");
  if (df == kWS) {
    expect(prov_checks > 1000 && prov_bad == 0,
           n + " MAC weights from the preload micro-op: " + std::to_string(prov_bad) + " of " + std::to_string(prov_checks));
    // the weights of compute k come from the last preload with a B address before it
    for (const auto &kv : weight_preload) {
      int64_t last = -1;
      for (const MicroOp &u : e.micro_ops().all())
        if (u.kind == UopKind::Preload && u.id < kv.first && LocalAddr(u.rs1, FrontendConfig::gemmini_default().addr_map()).is_garbage() == false)
          last = u.id;
      expect_eq(kv.second, last, n + " compute #" + std::to_string(kv.first) + "'s weights");
    }
  }
  // operation window
  const Operation &o = e.micro_ops().ops().at(size_t(op));
  expect(o.first_sent == 0 && o.end >= o.last_sent && o.end < cycles, n + " operation window");
}

// Per-cycle outputs of an Engine run as frames.
std::vector<Frame> frames(EngineOptions opt, const std::string &scenario, uint64_t &macs) {
  CtrlScenario s;
  ctrl_scenario(scenario, opt, s);
  const FrontendConfig cfg = FrontendConfig::gemmini_default();
  const FeRun r = run_ctrl(cfg, s);
  macs = s.engine->accounting().total.macs();
  return r.rows;
}

void invariance(const std::string &dir) {
  for (const char *sc : {"two_matmuls", "random_3"}) {
    EngineOptions plain;
    plain.micro_ops = false;
    plain.array.provenance = false;
    uint64_t m0 = 0, m1 = 0, m2 = 0;
    const auto a = frames(plain, sc, m0);
    EngineOptions rec;
    const auto b = frames(rec, sc, m1);
    EngineOptions vcd;
    vcd.vcd_path = dir + "/engine_" + sc + ".vcd";
    vcd.array.vcd_path = dir + "/array_" + sc + ".vcd";
    const auto c = frames(vcd, sc, m2);
    expect(!a.empty() && a == b && m0 == m1, std::string(sc) + ": micro-op recording and provenance change nothing");
    expect(a == c && m0 == m2, std::string(sc) + ": the VCD writers change nothing");
    std::ifstream f(vcd.vcd_path);
    std::stringstream text;
    text << f.rdbuf();
    const std::string v = text.str();
    expect(v.find("$enddefinitions") != std::string::npos && v.find("ex_queue_head") != std::string::npos &&
               v.find("dma_sp_taken") != std::string::npos,
           std::string(sc) + ": the Engine VCD declares the ports and micro-op ids");
  }
}

void classes() {
  const FrontendConfig cfg = FrontendConfig::gemmini_default();
  // tick() alone (it evaluates first) equals eval() + tick(); eval() changes no register:
  // a repeated eval() gives the same outputs, the register outputs are unchanged by it.
  EngineOptions opt;
  opt.array.provenance = false;
  CtrlScenario s;
  ctrl_scenario("two_matmuls", opt, s);
  Controller other(cfg);
  bool same = true, stable = true;
  for (int t = 0; t < 1200; ++t) {
    const CtrlTopIn in = s.engine->host_inputs();
    Frame f0 = make_frame(fe_port_specs(cfg, FeTop::Ctrl)), f1 = f0, f2 = f0, r0 = f0, r1 = f0;
    ctrl_out_to_frame(cfg, other.out_regs(), r0);
    other.set_inputs(in);
    other.eval();
    ctrl_out_to_frame(cfg, other.out(), f1);
    other.eval();
    ctrl_out_to_frame(cfg, other.out(), f2);
    ctrl_out_to_frame(cfg, other.out_regs(), r1);
    stable &= f1 == f2 && r0 == r1;
    const CtrlTopOut o = s.engine->step(in);  // eval + observe + tick
    ctrl_out_to_frame(cfg, o, f0);
    same &= f0 == f1;
    other.set_inputs(in);
    other.tick();  // without eval since set_inputs
  }
  expect(stable, "eval() changes no register and gives the same outputs twice");
  expect(same, "tick() alone equals eval() + tick()");
  // Two Engines are independent: interleaved runs give what separate runs give.
  Matmul m;
  m.M = m.N = m.K = 32;
  auto one = [&](unsigned df) {
    Engine e;
    m.dataflow = df;
    const int op = e.submit(m);
    e.run();
    return std::make_pair(e.cycle(), e.result(op));
  };
  const auto ws = one(kWS), os = one(kOS);
  Engine a, b;
  m.dataflow = kWS;
  const int oa = a.submit(m);
  m.dataflow = kOS;
  const int ob = b.submit(m);
  while (!a.done() || !b.done()) {
    if (!a.done())
      a.tick();
    if (!b.done())
      b.tick();
  }
  expect(a.cycle() == ws.first && b.cycle() == os.first && a.result(oa) == ws.second && b.result(ob) == os.second,
         "two Engines are independent");
  // unsupported configurations are refused
  FrontendConfig bad = cfg;
  bad.sp_banks = 8;
  bool threw = false;
  try {
    Engine e(EngineOptions{bad, {}, {}, true, ""});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  expect(threw, "a configuration other than Gemmini's default is refused");
}

}  // namespace

int main(int argc, char **argv) {
  const std::string out = argc > 2 && std::string(argv[1]) == "--out" ? argv[2] : ".";
  directed_ws();
  directed_os();
  matmul(kWS);
  matmul(kOS);
  invariance(out);
  classes();
  if (g_failed) {
    std::printf("MICRO_OPS FAIL %u of %u checks\n", g_failed, g_checks);
    return 1;
  }
  std::printf("MICRO_OPS ok checks=%u\n", g_checks);
  return 0;
}
