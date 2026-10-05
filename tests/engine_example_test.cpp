// The micro-op example of perf_reports/README.md: a 64x64x64 matmul (int8, no bias, C as int8)
// through the Engine with the default DmaParams, in WS (gemmini_loop_ws) and in OS (explicit
// commands), as `systolique_dump --matmul 64x64x64:WS` runs it. Prints the figures of the report
// and fails if one differs from the one the report quotes.
//
// Prints ENGINE_EXAMPLE ok or FAIL <what>.
#include "systolique/engine.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

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

struct Want {
  long long cycles, busy_begin, busy_end, first_sent, last_sent, end;
  double occ, util, busy_occ, busy_util;
  std::map<std::string, long long> counts;  // micro-ops per kind (requests per pass kind)
  long long mac, load, drain, bubble, load_concurrent;
  long long compute_first_issued, compute_last_completed, wait_sum, pass_sum, pass_max;
  // the longest compute pass: started, popped, its A reads, mvin rows written to its A bank meanwhile
  long long long_started, long_popped, long_first_read, long_last_read, long_mvin_rows;
};

void report(unsigned df, const Want &w) {
  const std::string n = df == kWS ? "WS" : "OS";
  Engine e;
  Matmul m;
  m.dataflow = df;
  const int op = e.submit(m);
  const long long cycles = e.run();
  if (e.result(op) != e.reference(op)) {
    std::printf("FAIL %s result\n", n.c_str());
    ++failures;
  }
  const Accounting a = e.accounting();
  const Operation &o = e.micro_ops().ops()[size_t(op)];
  std::map<std::string, long long> counts;
  long long first_issued = -1, last_completed = -1, wait_sum = 0, pass_sum = 0, pass_max = 0;
  const MicroOp *longest = nullptr;
  for (const MicroOp &u : e.micro_ops().all()) {
    std::string k = to_string(u.kind);
    if (u.kind == UopKind::Request)
      k += std::string(" (") + to_string(u.pass_kind) + ")";
    ++counts[k];
    if (u.kind == UopKind::ComputePreloaded || u.kind == UopKind::ComputeAccumulated) {
      if (first_issued < 0 || u.issued < first_issued)
        first_issued = u.issued;
      last_completed = std::max<long long>(last_completed, u.completed);
      wait_sum += u.started - u.issued;
      pass_sum += u.popped - u.started + 1;
      pass_max = std::max<long long>(pass_max, u.popped - u.started + 1);
      if (!longest || u.popped - u.started > longest->popped - longest->started)
        longest = &u;
    }
  }
  unsigned bank = 0;
  for (const MicroOp &u : e.micro_ops().all())
    if (u.kind == UopKind::OperandRead && u.parent == longest->id && u.operand == 'A') {
      bank = u.bank;
      break;
    }
  long long mvin_rows = 0;
  for (const MicroOp &u : e.micro_ops().all())
    mvin_rows += u.kind == UopKind::DmaRow && u.operand == 'I' && !u.to_acc && u.bank == bank &&
                 u.cycle >= longest->started && u.cycle <= longest->popped;
  std::printf("  longest compute pass: #%lld, cycles %lld-%lld, A rows read %lld-%lld from bank %u, %lld mvin rows "
              "written to that bank meanwhile\n",
              (long long)longest->id, (long long)longest->started, (long long)longest->popped,
              (long long)longest->first_read, (long long)longest->last_read, bank, mvin_rows);
  expect_eq(n + " longest pass started", longest->started, w.long_started);
  expect_eq(n + " longest pass popped", longest->popped, w.long_popped);
  expect_eq(n + " longest pass first read", longest->first_read, w.long_first_read);
  expect_eq(n + " longest pass last read", longest->last_read, w.long_last_read);
  expect_eq(n + " mvin rows into its A bank", mvin_rows, w.long_mvin_rows);
  std::printf("%s: %lld cycles (run window 0..%lld), busy window %lld..%lld; operation: first command %lld, "
              "last command %lld, last cycle %lld\n",
              n.c_str(), cycles, cycles - 1, (long long)a.busy_begin, (long long)a.busy_end, (long long)o.first_sent,
              (long long)o.last_sent, (long long)o.end);
  std::printf("  occupancy %.3f utilisation %.3f (run window); %.3f / %.3f (busy window)\n", a.occupancy,
              a.utilisation, a.busy_occupancy, a.busy_utilisation);
  std::printf("  PE-cycles: MAC %llu load %llu drain %llu bubble %llu, concurrent load %llu\n",
              (unsigned long long)a.total.macs(), (unsigned long long)a.total[PeState::Load],
              (unsigned long long)a.total[PeState::Drain], (unsigned long long)a.total[PeState::Bubble],
              (unsigned long long)a.total.load_concurrent);
  std::printf("  computes: first issued %lld, last completed %lld, issued->started %lld cycles in total, "
              "started->popped %lld cycles in total (longest %lld)\n",
              first_issued, last_completed, wait_sum, pass_sum, pass_max);
  std::printf("  micro-ops:");
  for (const auto &kv : counts)
    std::printf(" %s %lld,", kv.first.c_str(), kv.second);
  std::printf("\n");
  expect_eq(n + " cycles", cycles, w.cycles);
  expect_eq(n + " busy begin", a.busy_begin, w.busy_begin);
  expect_eq(n + " busy end", a.busy_end, w.busy_end);
  expect_eq(n + " commands the core sends (with the fence)", o.commands, w.counts.at("config") == 5 ? 12 : 160);
  expect_eq(n + " first command", o.first_sent, w.first_sent);
  expect_eq(n + " last command", o.last_sent, w.last_sent);
  expect_eq(n + " last cycle of the operation", o.end, w.end);
  expect_near(n + " occupancy", a.occupancy, w.occ);
  expect_near(n + " utilisation", a.utilisation, w.util);
  expect_near(n + " busy occupancy", a.busy_occupancy, w.busy_occ);
  expect_near(n + " busy utilisation", a.busy_utilisation, w.busy_util);
  expect_eq(n + " MAC", (long long)a.total.macs(), w.mac);
  expect_eq(n + " load", (long long)a.total[PeState::Load], w.load);
  expect_eq(n + " drain", (long long)a.total[PeState::Drain], w.drain);
  expect_eq(n + " bubble", (long long)a.total[PeState::Bubble], w.bubble);
  expect_eq(n + " concurrent load", (long long)a.total.load_concurrent, w.load_concurrent);
  expect_eq(n + " first compute issued", first_issued, w.compute_first_issued);
  expect_eq(n + " last compute completed", last_completed, w.compute_last_completed);
  expect_eq(n + " compute waits", wait_sum, w.wait_sum);
  expect_eq(n + " compute passes", pass_sum, w.pass_sum);
  expect_eq(n + " longest compute pass", pass_max, w.pass_max);
  for (const auto &kv : w.counts)
    expect_eq(n + " " + kv.first, counts.count(kv.first) ? counts.at(kv.first) : 0, kv.second);
  if (const std::string err = e.check(); !err.empty()) {
    std::printf("FAIL %s check: %s\n", n.c_str(), err.c_str());
    ++failures;
  }
}

}  // namespace

int main() {
  report(kWS, Want{1837, 197, 1585, 0, 10, 1836, 0.568, 0.557, 0.752, 0.737,
                   {{"config", 5}, {"loop_cmd", 6}, {"fence", 1}, {"mvin", 8}, {"mvout", 4}, {"preload", 64},
                    {"compute_preloaded", 16}, {"compute_accumulated", 48}, {"request (preload)", 2},
                    {"request (compute+preload)", 62}, {"request (compute)", 2}, {"flush", 0},
                    {"operand_read", 1280}, {"result_row", 1024}, {"dma_row", 768}},
                   262144, 5120, 0, 0, 253952, 190, 1549, 4661, 1337, 189, 208, 396, 381, 396, 173});
  report(kOS, Want{1957, 379, 1695, 0, 1617, 1956, 0.556, 0.523, 0.826, 0.778,
                   {{"config", 7}, {"loop_cmd", 0}, {"fence", 1}, {"mvin", 8}, {"mvout", 16}, {"preload", 64},
                    {"compute_preloaded", 16}, {"compute_accumulated", 48}, {"request (preload)", 1},
                    {"request (compute+preload)", 63}, {"request (compute)", 1}, {"flush", 3},
                    {"operand_read", 2048}, {"result_row", 256}, {"dma_row", 768}},
                   262144, 4096, 12288, 0, 258048, 376, 1607, 4892, 1024, 16, 583, 598, 567, 598, 0});
  if (failures) {
    std::printf("ENGINE_EXAMPLE FAIL (%d)\n", failures);
    return 1;
  }
  std::printf("ENGINE_EXAMPLE ok\n");
  return 0;
}
