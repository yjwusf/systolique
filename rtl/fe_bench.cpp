// Lockstep bench of the frontend classes (optional part, rtl/rtl.cmake): the Verilated RTL tops of
// rtl/ex/src/ (ExecuteTop: ExecuteController with its banks; CmdTop: the command path; CtrlTop:
// both, wired as Gemmini's Controller) and the model (ExecuteUnit, CommandPath, Controller) get
// the same inputs every cycle from the stimuli of bench/fe_stimulus.h; every output port is
// compared every cycle (all lanes, also those a valid bit does not qualify) and a run stops at the
// first difference. Optionally writes the RTL's inputs and outputs as reference traces, or
// requires the RTL to reproduce the stored ones.
//
//   rtl_fe --top ExecuteTop|CmdTop|CtrlTop [--test <name>]... [--catalog] [--seeds N]
//          [--first-seed S] [--stored <dir>] [--write-ref <dir>] [--check-ref <dir>]
//          [--comment <text>]... [--fault]
//
// --catalog runs every directed test, --seeds N the random seeds S .. S+N-1 (default S = 1),
// --stored every test with a trace in <dir>, which the RTL must reproduce (as --check-ref <dir>).
// --fault flips one bit of one model output (the bench must report it).
// Prints per test "LOCKSTEP <top> test=<t> cycles=<n> values=<n> mismatches=<n>" and a summary
// "LOCKSTEP <top> tests=<n> cycles=<n> values=<n> mismatches=<n>".
#include "VCmdTop.h"
#include "VCtrlTop.h"
#include "VExecuteTop.h"

#include "fe_ports.h"
#include "fe_run.h"
#include "fe_stimulus.h"
#include "rtl_dut.h"
#include "trace_io.h"

#include "verilated.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace systolique;

namespace {

// A Verilated top's ports by name, in fe_port_specs order.
template <class V>
struct FeDut {
  std::vector<PortSpec> spec;
  std::unique_ptr<V> rtl;
  VerilatedContext &ctx;
  FeDut(VerilatedContext &c, const std::vector<PortSpec> &s) : spec(s), ctx(c) {
    rtl = std::make_unique<V>(&ctx, "fe");
    rtl->clock = 0;
    rtl->reset = 1;
    for (int i = 0; i < 4; ++i)  // four cycles with reset high: row 0 of a trace is cycle 0
      edge();
    rtl->reset = 0;
    rtl->eval();
  }
  void edge() {
    rtl->clock = 1;
    rtl->eval();
    rtl->clock = 0;
    rtl->eval();
  }
  int at(const char *n) const {
    const int i = port_index(spec, n);
    if (i < 0) {
      std::fprintf(stderr, "no port %s\n", n);
      std::exit(2);
    }
    return i;
  }
};

#define IN(p) rtl_put(d.rtl->io_##p, f[size_t(d.at(#p))])
#define OUT(p) rtl_get(d.rtl->io_##p, f[size_t(d.at(#p))])

template <class V>
void dma_ports_in(FeDut<V> &d, const Frame &f) {
  IN(dma_sp_en);
  IN(dma_sp_bank);
  IN(dma_sp_addr);
  IN(dma_sp_data);
  IN(dma_sp_mask);
  IN(dma_acc_en);
  IN(dma_acc_bank);
  IN(dma_acc_addr);
  IN(dma_acc_data);
  IN(dma_acc_mask);
  IN(dma_acc_acc);
}
template <class V>
void bank_ports_out(FeDut<V> &d, Frame &f) {
  OUT(dma_sp_taken);
  OUT(dma_acc_taken);
  OUT(sp_read_valid);
  OUT(sp_read_ready);
  OUT(sp_read_addr);
  OUT(sp_resp_ready);
  OUT(sp_write_en);
  OUT(sp_write_addr);
  OUT(sp_write_data);
  OUT(sp_write_mask);
  OUT(acc_read_valid);
  OUT(acc_read_ready);
  OUT(acc_read_addr);
  OUT(acc_write_valid);
  OUT(acc_write_addr);
  OUT(acc_write_data);
  OUT(acc_write_acc);
  OUT(acc_write_mask);
}

void set_in(FeDut<VExecuteTop> &d, const Frame &f) {
  IN(cmd_valid);
  IN(cmd_funct);
  IN(cmd_rs1);
  IN(cmd_rs2);
  IN(cmd_rob_id);
  dma_ports_in(d, f);
}
void get_out(FeDut<VExecuteTop> &d, Frame &f) {
  OUT(cmd_ready);
  OUT(completed_valid);
  OUT(completed_bits);
  OUT(busy);
  bank_ports_out(d, f);
}
void set_in(FeDut<VCmdTop> &d, const Frame &f) {
  IN(cmd_valid);
  IN(cmd_funct);
  IN(cmd_rs1);
  IN(cmd_rs2);
  IN(ld_ready);
  IN(ex_ready);
  IN(st_ready);
  IN(completed_valid);
  IN(completed_bits);
}
template <class V>
void issue_out(FeDut<V> &d, Frame &f) {
  OUT(ld_valid);
  OUT(ld_funct);
  OUT(ld_rs1);
  OUT(ld_rs2);
  OUT(ld_rob_id);
  OUT(st_valid);
  OUT(st_funct);
  OUT(st_rs1);
  OUT(st_rs2);
  OUT(st_rob_id);
  OUT(ex_valid);
  OUT(ex_funct);
  OUT(ex_rs1);
  OUT(ex_rs2);
  OUT(ex_rob_id);
}
void get_out(FeDut<VCmdTop> &d, Frame &f) {
  OUT(cmd_ready);
  OUT(busy);
  OUT(loop_matmul_busy);
  issue_out(d, f);
  OUT(matmul_ld_completed);
  OUT(matmul_ex_completed);
  OUT(matmul_st_completed);
}
void set_in(FeDut<VCtrlTop> &d, const Frame &f) {
  IN(cmd_valid);
  IN(cmd_funct);
  IN(cmd_rs1);
  IN(cmd_rs2);
  IN(ld_ready);
  IN(st_ready);
  IN(ld_completed_valid);
  IN(ld_completed_bits);
  IN(st_completed_valid);
  IN(st_completed_bits);
  dma_ports_in(d, f);
}
void get_out(FeDut<VCtrlTop> &d, Frame &f) {
  OUT(cmd_ready);
  OUT(busy);
  OUT(loop_matmul_busy);
  issue_out(d, f);
  OUT(ld_completed_ready);
  OUT(st_completed_ready);
  OUT(ex_ready);
  OUT(ex_completed_valid);
  OUT(ex_completed_bits);
  OUT(ex_busy);
  bank_ports_out(d, f);
}

struct Result {
  bool ok = true;
  std::string error;
  uint64_t cycles = 0, values = 0;
  std::vector<Frame> rows;  // the RTL's inputs and outputs
};

bool g_fault = false;
int g_argc = 0;
char **g_argv = nullptr;

// Flips one bit of the first output port lane that is 1 in a cycle with a valid issue / write.
void inject(const std::vector<PortSpec> &spec, Frame &mf, bool &done) {
  if (done)
    return;
  for (const char *n : {"sp_write_data", "ex_rs2", "ld_rs1"}) {
    const int p = port_index(spec, n);
    if (p < 0)
      continue;
    const char *v = std::strcmp(n, "sp_write_data") == 0 ? "sp_write_en" : std::strcmp(n, "ex_rs2") == 0 ? "ex_valid" : "ld_valid";
    const int pv = port_index(spec, v);
    if (pv >= 0 && mf[size_t(pv)].get(0, 1)) {
      mf[size_t(p)].set(0, 1, mf[size_t(p)].get(0, 1) ^ 1);
      done = true;
      return;
    }
  }
}

template <class V, class Drive>
Result lockstep(const FrontendConfig &cfg, FeTop top, unsigned max_cycles, Drive drive) {
  VerilatedContext ctx;  // one per test: an assertion stops only its test
  ctx.fatalOnError(false);
  ctx.commandArgs(g_argc, g_argv);  // the ReservationStation's +gemmini_timeout plusarg
  const auto spec = fe_port_specs(cfg, top);
  FeDut<V> d(ctx, spec);
  Result r;
  bool faulted = false;
  for (unsigned cycle = 0; cycle < max_cycles; ++cycle) {
    Frame in_f = make_frame(spec), rtl_f, model_f;
    // drive(cycle, in_f, model_f) fills the inputs and steps the model (its outputs into model_f);
    // it returns false when the stimulus has ended (after this cycle)
    bool more = true;
    auto apply_rtl = [&](const Frame &f) {
      set_in(d, f);
      d.rtl->eval();
      rtl_f = f;
      get_out(d, rtl_f);
      return rtl_f;
    };
    more = drive(cycle, apply_rtl, model_f);
    if (g_fault)
      inject(spec, model_f, faulted);
    const std::string diff = fe_diff(spec, rtl_f, model_f, r.values);
    r.rows.push_back(rtl_f);
    d.edge();
    ++r.cycles;
    if (ctx.gotError() || ctx.gotFinish()) {
      r.ok = false;
      r.error = "cycle " + std::to_string(cycle) + ": the RTL stopped (assertion)";
      return r;
    }
    if (!diff.empty()) {
      r.ok = false;
      r.error = "cycle " + std::to_string(cycle) + ": " + diff;
      return r;
    }
    if (!more)
      return r;
  }
  r.ok = false;
  r.error = "no end after " + std::to_string(max_cycles) + " cycles";
  return r;
}

Result run_test(const FrontendConfig &cfg, FeTop top, const std::string &name) {
  if (top == FeTop::Execute) {
    ExStimulus s;
    if (!ex_stimulus(name, s))
      return {false, "unknown test " + name, 0, 0, {}};
    ExecuteUnit m(cfg);
    return lockstep<VExecuteTop>(cfg, top, 200000, [&](unsigned cycle, auto apply_rtl, Frame &mf) {
      const ExecuteTopIn in = s.next(cycle);
      Frame f = make_frame(fe_port_specs(cfg, top));
      ex_in_to_frame(cfg, in, f);
      const Frame rf = apply_rtl(f);
      m.set_inputs(in);
      m.eval();
      mf = f;
      ex_out_to_frame(cfg, m.out(), mf);
      ExecuteTopOut ro;
      frame_to_ex_out(cfg, rf, ro);
      s.observe(in, ro);
      m.tick();
      return !s.finished();
    });
  }
  if (top == FeTop::Cmd) {
    CmdStimulus s;
    if (!cmd_stimulus(name, s))
      return {false, "unknown test " + name, 0, 0, {}};
    CommandPath m(cfg);
    return lockstep<VCmdTop>(cfg, top, 400000, [&](unsigned cycle, auto apply_rtl, Frame &mf) {
      const CommandPath::In in = s.next(cycle, m.out());
      Frame f = make_frame(fe_port_specs(cfg, top));
      cmd_in_to_frame(cfg, in, f);
      const Frame rf = apply_rtl(f);
      m.set_inputs(in);
      m.eval();
      mf = f;
      cmd_out_to_frame(cfg, m.out(), mf);
      CommandPath::Out ro;
      frame_to_cmd_out(cfg, rf, ro);
      s.observe(in, ro, cycle);
      m.tick();
      return !s.finished();
    });
  }
  EngineOptions opt;
  opt.cfg = cfg;
  opt.array.provenance = false;
  CtrlScenario s;
  if (!ctrl_scenario(name, opt, s))
    return {false, "unknown test " + name, 0, 0, {}};
  unsigned tail = 0;
  return lockstep<VCtrlTop>(cfg, top, 2000000, [&](unsigned, auto apply_rtl, Frame &mf) {
    Engine &e = *s.engine;
    const CtrlTopIn in = e.host_inputs();
    Frame f = make_frame(fe_port_specs(cfg, top));
    ctrl_in_to_frame(cfg, in, f);
    apply_rtl(f);
    const CtrlTopOut o = e.step(in);
    mf = f;
    ctrl_out_to_frame(cfg, o, mf);
    tail = e.done() ? tail + 1 : 0;
    return tail <= s.tail && e.host().error().empty();
  });
}

}  // namespace

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  g_argc = argc;
  g_argv = argv;
  FeTop top = FeTop::Execute;
  std::vector<std::string> tests, comments;
  unsigned seeds = 0, first_seed = 1;
  bool catalog = false;
  std::string write_ref, check_ref;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--top") {
      if (!parse_fe_top(val(), top)) {
        std::fprintf(stderr, "unknown top\n");
        return 2;
      }
    } else if (a == "--test")
      tests.push_back(val());
    else if (a == "--catalog")
      catalog = true;
    else if (a == "--seeds")
      seeds = unsigned(std::stoul(val()));
    else if (a == "--first-seed")
      first_seed = unsigned(std::stoul(val()));
    else if (a == "--write-ref")
      write_ref = val();
    else if (a == "--check-ref")
      check_ref = val();
    else if (a == "--stored") {
      check_ref = val();
      std::vector<std::string> names;
      for (const auto &e : std::filesystem::directory_iterator(check_ref)) {
        const std::string f = e.path().filename().string();
        if (f.size() > 7 && f.substr(f.size() - 7) == ".csv.gz")
          names.push_back(f.substr(0, f.size() - 7));
      }
      std::sort(names.begin(), names.end());
      tests.insert(tests.end(), names.begin(), names.end());
    }
    else if (a == "--comment")
      comments.push_back(val());
    else if (a == "--fault")
      g_fault = true;
    else {
      std::fprintf(stderr,
                   "usage: %s --top ExecuteTop|CmdTop|CtrlTop [--test t]... [--catalog] [--seeds N] "
                   "[--first-seed S] [--stored dir] [--write-ref dir] [--check-ref dir] [--comment c]... [--fault]\n",
                   argv[0]);
      return 2;
    }
  }
  const FrontendConfig cfg = FrontendConfig::gemmini_default();
  if (catalog) {
    if (top == FeTop::Execute)
      for (const auto &kv : ex_catalog()) tests.push_back(kv.first);
    else if (top == FeTop::Cmd)
      for (const auto &kv : cmd_catalog()) tests.push_back(kv.first);
    else
      for (const auto &kv : ctrl_catalog()) tests.push_back(kv.first);
  }
  for (unsigned s = 0; s < seeds; ++s)
    tests.push_back("random_" + std::to_string(first_seed + s));
  const auto spec = fe_port_specs(cfg, top);
  uint64_t cycles = 0, values = 0;
  unsigned failed = 0;
  for (const std::string &t : tests) {
    Result r = run_test(cfg, top, t);
    cycles += r.cycles;
    values += r.values;
    std::string extra;
    if (r.ok && !check_ref.empty()) {
      TraceInfo info;
      std::vector<Frame> ref;
      std::string err;
      if (!read_trace(check_ref + "/" + t + ".csv.gz", spec, info, ref, err))
        extra = " FAIL " + err;
      else if (ref != r.rows)
        extra = " FAIL the RTL no longer produces the stored trace";
    }
    if (r.ok && !write_ref.empty()) {
      TraceInfo info;
      info.comments = comments;
      info.comments.push_back(std::string(fe_top_name(top)) + " stimulus " + t +
                              " (bench/fe_stimulus.cpp), recorded from the Verilated RTL by rtl_fe "
                              "(rtl/fe_bench.cpp); four cycles with reset high precede row 0");
      std::string err;
      if (!write_trace(write_ref + "/" + t + ".csv.gz", spec, info, r.rows, err))
        extra = " FAIL " + err;
    }
    const bool bad = !r.ok || !extra.empty();
    failed += bad;
    std::printf("LOCKSTEP %s test=%s cycles=%llu values=%llu mismatches=%u%s%s\n", fe_top_name(top), t.c_str(),
                (unsigned long long)r.cycles, (unsigned long long)r.values, r.ok ? 0u : 1u,
                r.ok ? "" : (" FAIL " + r.error).c_str(), extra.c_str());
  }
  std::printf("LOCKSTEP %s tests=%zu cycles=%llu values=%llu mismatches=%u%s\n", fe_top_name(top), tests.size(),
              (unsigned long long)cycles, (unsigned long long)values, failed, failed ? " FAIL" : "");
  return failed ? 1 : 0;
}
