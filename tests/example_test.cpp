// The accounting example of perf_reports/README.md and docs/systolic_array.md: 4 back-to-back WS
// matmuls on the 16x16 array (configuration default), as `systolique_dump --config default
// --scenario ws_stream --tiles 4` runs them (ws_case of bench/reference.cpp, seed 1: request j
// preloads W_j while computing A_{j-1} W_{j-1}; 5 requests of 16 rows; the driver's idle tail is
// part of the run), and the same stream in OS. Prints the per-request table and the totals and
// fails if any figure differs from the one the report quotes.
//
// Prints EXAMPLE ok or FAIL <what>.
#include "reference.h"
#include "run.h"
#include "stimulus.h"

#include <cmath>
#include <memory>
#include <cstdio>
#include <random>
#include <string>

using namespace systolique;

namespace {

int failures = 0;

void expect_eq(const std::string &what, long long got, long long want) {
  if (got != want) {
    std::printf("FAIL %s = %lld, expected %lld\n", what.c_str(), got, want);
    ++failures;
  }
}

void expect_near(const std::string &what, double got, double want) {
  if (std::fabs(got - want) > 0.0005) {
    std::printf("FAIL %s = %.4f, expected %.3f\n", what.c_str(), got, want);
    ++failures;
  }
}

std::unique_ptr<SystolicArray> run(Dataflow df, unsigned K) {
  const ArrayConfig &cfg = *find_config("default");
  std::mt19937_64 rng(1);
  const MatmulCase c = df == Dataflow::WS ? ws_case(cfg, rng, K, false, false)
                                          : os_case(cfg, rng, K, false, false, 0);
  auto a = std::make_unique<SystolicArray>(cfg);
  MwdDriver drv(cfg, c.ops, DriverOptions{0, 0, 1});
  const ArrayRun r = run_stimulus(*a, Top::MeshWithDelays, drv);
  if (!r.ok) {
    std::printf("FAIL run: %s\n", r.error.c_str());
    ++failures;
  }
  if (const std::string e = check_responses(cfg, c, r.rows); !e.empty()) {
    std::printf("FAIL results: %s\n", e.c_str());
    ++failures;
  }
  return a;
}

std::string cyc(int64_t c) { return c < 0 ? "-" : std::to_string(c); }

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  {
    const auto ap = run(Dataflow::WS, 4);
    const SystolicArray &a = *ap;
    const Accounting acc = a.accounting();
    if (const std::string e = SystolicArray::check(acc); !e.empty()) {
      std::printf("FAIL conservation: %s\n", e.c_str());
      ++failures;
    }
    std::printf("| op | accept | first in | last in | first out | last out | result rows | MAC | "
                "load | load_concurrent | MACs |\n|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|\n");
    // accept, first_in, last_in, first_out, last_out, result_first, result_last, MAC, load, lc
    const long long want[5][10] = {
        {0, 1, 16, 33, 48, 49, 64, 0, 4096, 0},
        {16, 17, 32, 49, 64, 65, 80, 4096, 0, 4096},
        {32, 33, 48, 65, 80, 81, 96, 4096, 0, 4096},
        {48, 49, 64, 81, 96, 97, 112, 4096, 0, 4096},
        {64, 65, 80, 97, 112, -1, -1, 4096, 0, 0},
    };
    expect_eq("WS requests", (long long)a.requests().size(), 5);
    for (const RequestInfo &r : a.requests()) {
      const StateCounts &s = acc.per_request[r.index];
      std::printf("| %u | %s | %s | %s | %s | %s | %s | %llu | %llu | %llu | %llu |\n", r.index,
                  cyc(r.accept).c_str(), cyc(r.first_in).c_str(), cyc(r.last_in).c_str(),
                  cyc(r.first_out).c_str(), cyc(r.last_out).c_str(),
                  r.result_first < 0 ? "-" : (cyc(r.result_first) + "-" + cyc(r.result_last)).c_str(),
                  (unsigned long long)s.macs(), (unsigned long long)s[PeState::Load],
                  (unsigned long long)s.load_concurrent, (unsigned long long)s.macs());
      if (r.index >= 5) continue;
      const long long got[10] = {r.accept,    r.first_in,         r.last_in,
                                 r.first_out, r.last_out,         r.result_first,
                                 r.result_last, (long long)s.macs(), (long long)s[PeState::Load],
                                 (long long)s.load_concurrent};
      for (int k = 0; k < 10; ++k)
        expect_eq("WS request " + std::to_string(r.index) + " field " + std::to_string(k), got[k],
                  want[r.index][k]);
    }
    std::printf("WS totals: mac=%llu load=%llu drain=%llu bubble=%llu load_concurrent=%llu "
                "run=0-%lld (%lld cycles) occupancy=%.3f utilisation=%.3f busy=%lld-%lld (%lld "
                "cycles) busy_occupancy=%.3f busy_utilisation=%.3f\n",
                (unsigned long long)acc.total.macs(), (unsigned long long)acc.total[PeState::Load],
                (unsigned long long)acc.total[PeState::Drain],
                (unsigned long long)acc.total[PeState::Bubble],
                (unsigned long long)acc.total.load_concurrent, (long long)acc.cycles - 1,
                (long long)acc.cycles, acc.occupancy, acc.utilisation, (long long)acc.busy_begin,
                (long long)acc.busy_end, (long long)acc.busy_cycles(), acc.busy_occupancy,
                acc.busy_utilisation);
    expect_eq("WS MAC PE-cycles = 4 x 16^3", (long long)acc.total.macs(), 16384);
    expect_eq("WS load", (long long)acc.total[PeState::Load], 4096);
    expect_eq("WS drain + bubble",
              (long long)(acc.total[PeState::Drain] + acc.total[PeState::Bubble]), 0);
    expect_eq("WS cycles", acc.cycles, 161);
    expect_eq("WS busy begin", acc.busy_begin, 0);
    expect_eq("WS busy end", acc.busy_end, 111);
    expect_near("WS occupancy", acc.occupancy, 0.497);
    expect_near("WS utilisation", acc.utilisation, 0.398);
    expect_near("WS busy occupancy", acc.busy_occupancy, 0.714);
    expect_near("WS busy utilisation", acc.busy_utilisation, 0.571);
  }
  {
    const auto ap = run(Dataflow::OS, 4);
    const SystolicArray &a = *ap;
    const Accounting acc = a.accounting();
    if (const std::string e = SystolicArray::check(acc); !e.empty()) {
      std::printf("FAIL conservation: %s\n", e.c_str());
      ++failures;
    }
    std::printf("OS totals: requests=%zu mac=%llu load=%llu drain=%llu bubble=%llu "
                "drain_concurrent=%llu busy=%lld-%lld busy_occupancy=%.3f busy_utilisation=%.3f\n",
                a.requests().size(), (unsigned long long)acc.total.macs(),
                (unsigned long long)acc.total[PeState::Load],
                (unsigned long long)acc.total[PeState::Drain],
                (unsigned long long)acc.total[PeState::Bubble],
                (unsigned long long)acc.total.drain_concurrent, (long long)acc.busy_begin,
                (long long)acc.busy_end, acc.busy_occupancy, acc.busy_utilisation);
    expect_eq("OS requests", (long long)a.requests().size(), 6);
    expect_eq("OS MAC PE-cycles", (long long)acc.total.macs(), 16384);
    expect_eq("OS load", (long long)acc.total[PeState::Load], 4096);
    expect_eq("OS drain", (long long)acc.total[PeState::Drain], 4096);
    expect_eq("OS drain_concurrent", (long long)acc.total.drain_concurrent, 6528);
    expect_eq("OS busy begin", acc.busy_begin, 0);
    expect_eq("OS busy end", acc.busy_end, 127);
    expect_near("OS busy occupancy", acc.busy_occupancy, 0.750);
    expect_near("OS busy utilisation", acc.busy_utilisation, 0.500);
  }
  if (failures) return 1;
  std::printf("EXAMPLE ok\n");
  return 0;
}
