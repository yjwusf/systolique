// Conservation of SystolicArray's accounting:
//
//   conservation_test --config <c>
//
// OS/WS streams of K = 1, 4, 16 matmuls with and without input bubbles, and random request mixes
// (both dataflows, flushes, partial rows, transposes): SystolicArray::check, and an independent
// recount from the PeViews of every PE-cycle: exactly one state per PE-cycle (so at most one
// MAC: 1 PE = 1 MAC per cycle), MAC + load + drain + bubble + idle = DIM^2 in every cycle,
// per-cycle and per-PE counts equal the accounting, useful MACs = K x DIM^3 for the streams,
// MACs = rows x DIM^2 per computing request.
//
// Prints CONSERVATION config=<c> runs=<n> errors=<n>
#include "reference.h"
#include "run.h"
#include "stimulus.h"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace systolique;

namespace {

struct Recount {
  // Per cycle, per PE: the request of its row (-1 idle) and the drain flag, from PeView.
  std::vector<std::vector<int>> req;
  std::vector<std::vector<uint8_t>> drain;
};

std::string recount_check(const SystolicArray &a, const Recount &rc) {
  const Accounting acc = a.accounting();
  if (std::string e = SystolicArray::check(acc); !e.empty()) return e;
  if (rc.req.size() != size_t(acc.cycles)) return "recount covers a different number of cycles";
  const unsigned pes = acc.pes;
  std::vector<StateCounts> per_pe(pes);
  for (size_t t = 0; t < rc.req.size(); ++t) {
    StateCounts s;
    for (unsigned p = 0; p < pes; ++p) {
      const int q = rc.req[t][p];
      PeState st = PeState::Idle;
      if (q >= 0) st = a.requests()[q].state();
      StateCounts one;
      one[st] = 1;  // exactly one state, so at most one MAC, per PE-cycle
      if (st == PeState::Mac && a.requests()[q].preloads) one.load_concurrent = 1;
      if (st == PeState::Mac) one.drain_concurrent = rc.drain[t][p];
      if (one.total() != 1 || one.macs() > 1) return "a PE-cycle with more than one state";
      s.add(one);
      per_pe[p].add(one);
    }
    // MAC + load + drain + bubble + idle = DIM^2
    if (s.total() != pes) return "cycle " + std::to_string(t) + ": states do not sum to DIM^2";
    const StateCounts &x = acc.per_cycle[t];
    if (s.n != x.n || s.load_concurrent != x.load_concurrent ||
        s.drain_concurrent != x.drain_concurrent)
      return "cycle " + std::to_string(t) + ": recount differs from the accounting";
  }
  for (unsigned p = 0; p < pes; ++p)
    if (per_pe[p].n != acc.per_pe[p].n) return "PE " + std::to_string(p) + ": recount differs";
  for (const RequestInfo &r : a.requests()) {
    const uint64_t macs = acc.per_request[r.index].macs();
    if (r.state() == PeState::Mac && macs != uint64_t(r.rows_in) * pes)
      return "request " + std::to_string(r.index) + ": MACs != rows x DIM^2";
    if (!r.decided) return "request " + std::to_string(r.index) + " never decided";
  }
  return "";
}

// Runs ops on a fresh array with the recount hook; returns "" or the error.
std::string run_counted(const ArrayConfig &cfg, const std::vector<Op> &ops, double bubble,
                        uint64_t expect_macs) {
  SystolicArray a(cfg);
  Recount rc;
  MwdDriver drv(cfg, ops, DriverOptions{bubble, 0, 7});
  ArrayRun r = run_stimulus(a, Top::MeshWithDelays, drv, 4, 1000000, [&](const SystolicArray &x) {
    const ArrayConfig &c = x.config();
    std::vector<int> q(c.rows() * c.cols());
    std::vector<uint8_t> d(q.size());
    for (unsigned i = 0; i < c.rows(); ++i)
      for (unsigned j = 0; j < c.cols(); ++j) {
        const PeView v = x.pe(i, j);
        q[i * c.cols() + j] = v.in.valid ? v.request : -1;
        d[i * c.cols() + j] = v.in.valid && v.result_out;
      }
    rc.req.push_back(std::move(q));
    rc.drain.push_back(std::move(d));
  });
  if (!r.ok) return r.error;
  if (std::string e = recount_check(a, rc); !e.empty()) return e;
  const Accounting acc = a.accounting();
  // useful MACs = workload MACs exactly
  if (expect_macs && acc.total.macs() != expect_macs)
    return "useful MACs " + std::to_string(acc.total.macs()) + " != " + std::to_string(expect_macs);
  return "";
}

}  // namespace

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  if (argc != 3 || std::string(argv[1]) != "--config" || !find_config(argv[2])) {
    std::fprintf(stderr, "usage: %s --config <name>\n", argv[0]);
    return 2;
  }
  const ArrayConfig &cfg = *find_config(argv[2]);
  const unsigned dim = cfg.block_size();
  unsigned runs = 0, errors = 0;
  auto report = [&](const std::string &what, const std::string &e) {
    ++runs;
    if (!e.empty()) {
      ++errors;
      std::printf("FAIL %s %s: %s\n", cfg.name.c_str(), what.c_str(), e.c_str());
    }
  };
  for (unsigned K : {1u, 4u, 16u})
    for (double bubble : {0.0, 0.3}) {
      const uint64_t macs = uint64_t(K) * dim * dim * dim;
      std::mt19937_64 rng(K * 31 + unsigned(bubble * 10));
      const std::string w = "K=" + std::to_string(K) + " bubble=" + std::to_string(bubble);
      if (cfg.dataflow != Dataflow::WS)
        report("os " + w, run_counted(cfg, os_case(cfg, rng, K, false, false, 0).ops, bubble, macs));
      if (cfg.dataflow != Dataflow::OS)
        report("ws " + w, run_counted(cfg, ws_case(cfg, rng, K, false, false).ops, bubble, macs));
    }
  for (uint64_t seed = 1; seed <= 6; ++seed) {
    std::mt19937_64 rng(seed * 977);
    report("random seed " + std::to_string(seed),
           run_counted(cfg, random_ops(cfg, rng, 24), seed % 2 ? 0.25 : 0.0, 0));
  }
  std::printf("CONSERVATION config=%s runs=%u errors=%u\n", cfg.name.c_str(), runs, errors);
  return errors ? 1 : 0;
}
