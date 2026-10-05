// Provenance of every PE register value against the operands:
//
//   provenance_test --config <c> [--seeds n]
//
// WS: random streams of K = 1..6 matmuls (A or W through the transposer, with and without input
// bubbles): in every cycle every PE register equals W_k[w_row][w_col] of the matmul it names,
// every MAC at PE (i, j) multiplies with W_k[i][j] of the matmul whose A the request carries,
// and loaded < active <= last_used.
// OS analogue: D on its way down equals D_k[w_row][w_col], an accumulator that has taken all DIM
// rows equals (A_k B_k + D_k)[i][j], a result on its way out equals it, every MAC accumulates
// into D_k[i][j] of the matmul being computed.
//
// Prints PROVENANCE config=<c> runs=<n> checks=<n> failures=<n>
#include "checks.h"
#include "reference.h"
#include "run.h"
#include "stimulus.h"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace systolique;
using namespace systolique::test;
using arith::sext;

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::string name;
  unsigned seeds = 3;
  bool bad = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config" && i + 1 < argc)
      name = argv[++i];
    else if (a == "--seeds" && i + 1 < argc)
      seeds = unsigned(std::stoul(argv[++i]));
    else
      bad = true;
  }
  if (bad || !find_config(name)) {
    std::fprintf(stderr, "usage: %s --config <name> [--seeds n]\n", argv[0]);
    return 2;
  }
  const ArrayConfig &cfg = *find_config(name);
  const unsigned dim = cfg.block_size();
  Checks ck;
  unsigned runs = 0;
  // WS: request j sends W_j through d, rows bottom-up (reverse_rows), transposed if w_t; with
  // w_t W_k enters the array one request later and A_k is multiplied in request k + 1 + lead.
  if (cfg.dataflow != Dataflow::OS)
    for (unsigned s = 0; s < seeds; ++s)
      for (unsigned v = 0; v < 3; ++v) {  // one operand through the transposer at most
        std::mt19937_64 rng(1000 + s * 4 + v);
        const bool a_t = v & 1, w_t = v & 2;
        const unsigned K = 1 + unsigned(rng() % 6), lead = w_t ? 1 : 0;
        const MatmulCase mc = ws_case(cfg, rng, K, a_t, w_t);
        std::vector<Matrix> W;
        for (const Op &op : mc.ops)
          W.push_back(w_t ? transpose(reverse_rows(op.d)) : reverse_rows(op.d));
        SystolicArray a(cfg);
        MwdDriver drv(cfg, mc.ops, DriverOptions{s % 2 ? 0.3 : 0.0, 0, 11 + s});
        ++runs;
        ArrayRun r = run_stimulus(a, Top::MeshWithDelays, drv, 4, 1000000, [&](const SystolicArray &x) {
          check_raw(x, ck);
          for (unsigned i = 0; i < dim; ++i)
            for (unsigned j = 0; j < dim; ++j) {
              const PeView pv = x.pe(i, j);
              for (unsigned k = 0; k < 2; ++k) {
                const RegSource &src = pv.src[k];
                if (src.request < 0 || src.w_row < 0) continue;
                ++ck.done;
                const int64_t reg = k ? pv.regs.c2 : pv.regs.c1;
                if (sext(reg, cfg.in_bits) != W[src.request][src.w_row][src.w_col])
                  ck.fail(x.cycle(), pe_name(i, j, k) + " != W_" + std::to_string(src.request) +
                                         "[" + std::to_string(src.w_row) + "][" +
                                         std::to_string(src.w_col) + "]");
                if (src.loaded >= x.cycle() || (src.active >= 0 && src.active < src.loaded) ||
                    src.last_used > x.cycle())
                  ck.fail(x.cycle(), pe_name(i, j, k) + ": loaded/active/last_used out of order");
              }
              if (pv.state != PeState::Mac) continue;
              // The MAC uses W_k[i][j] of the matmul whose A this request carries.
              const RegSource &w = pv.src[pv.active];
              ++ck.done;
              if (w.w_row != int(i) || w.w_col != int(j) ||
                  w.request != pv.request - 1 - int(lead) || w.active < 0 || w.active > x.cycle())
                ck.fail(x.cycle(), "MAC at PE " + std::to_string(i) + "," + std::to_string(j) +
                                       " of request " + std::to_string(pv.request) +
                                       " uses W_" + std::to_string(w.request) + "[" +
                                       std::to_string(w.w_row) + "][" +
                                       std::to_string(w.w_col) + "]");
            }
        });
        if (!r.ok) ck.fail(-1, r.error);
      }
  // OS: request k preloads D_k, request k+1 accumulates A_k B_k into it, request k+2 (or the
  // flush) shifts C_k out, bottom row first.
  if (cfg.dataflow != Dataflow::WS)
    for (unsigned s = 0; s < seeds; ++s)
      for (unsigned v = 0; v < 3; ++v) {  // one operand through the transposer at most
        std::mt19937_64 rng(2000 + s * 4 + v);
        const unsigned K = 1 + unsigned(rng() % 6);
        const MatmulCase mc = os_case(cfg, rng, K, v & 1, v & 2, 0);
        std::vector<Matrix> D, C;
        for (const Op &op : mc.ops) D.push_back(op.d.empty() ? Matrix() : reverse_rows(op.d));
        for (const ExpectedTile &e : mc.expected) C.push_back(reverse_rows(e.rows));
        SystolicArray a(cfg);
        MwdDriver drv(cfg, mc.ops, DriverOptions{s % 2 ? 0.3 : 0.0, 0, 21 + s});
        ++runs;
        ArrayRun r = run_stimulus(a, Top::MeshWithDelays, drv, 4, 1000000, [&](const SystolicArray &x) {
          for (unsigned i = 0; i < dim; ++i)
            for (unsigned j = 0; j < dim; ++j) {
              const PeView pv = x.pe(i, j);
              for (unsigned k = 0; k < 2; ++k) {
                const RegSource &src = pv.src[k];
                if (src.request < 0 || src.w_row < 0 || x.requests()[src.request].flush) continue;
                const int64_t reg = k ? pv.regs.c2 : pv.regs.c1;
                const unsigned m = unsigned(src.request);
                std::string what;
                ++ck.done;
                // Accumulations already in the value (the view counts this cycle's use too).
                const uint32_t done = src.uses - (src.last_used == x.cycle() ? 1 : 0);
                if (done == 0 && int(i) <= src.w_row) {  // D on its way down
                  if (reg != D[m][src.w_row][src.w_col]) what = "D";
                } else if (m < C.size() && (int(i) > src.w_row || done == dim)) {
                  // C_m, accumulated here or passed down from its PE
                  if (sext(reg, cfg.out_bits) != C[m][src.w_row][src.w_col]) what = "C";
                } else {
                  --ck.done;  // partial sum, or the zero preload after the last matmul
                }
                if (!what.empty())
                  ck.fail(x.cycle(), pe_name(i, j, k) + " = " + std::to_string(reg) + " != " +
                                         what + "_" + std::to_string(m) + "[" +
                                         std::to_string(src.w_row) + "][" +
                                         std::to_string(src.w_col) + "]");
              }
              if (pv.state != PeState::Mac) continue;
              const RegSource &acc = pv.src[pv.active];
              ++ck.done;
              if (acc.w_row != int(i) || acc.w_col != int(j) || acc.request != pv.request - 1)
                ck.fail(x.cycle(), "MAC at PE " + std::to_string(i) + "," + std::to_string(j) +
                                       " of request " + std::to_string(pv.request) +
                                       " accumulates into D_" + std::to_string(acc.request));
            }
        });
        if (!r.ok) ck.fail(-1, r.error);
      }
  std::printf("PROVENANCE config=%s runs=%u checks=%llu failures=%llu%s%s\n", cfg.name.c_str(),
              runs, ck.done, ck.failed, ck.failed ? " FAIL " : "", ck.first.c_str());
  return ck.failed ? 1 : 0;
}
