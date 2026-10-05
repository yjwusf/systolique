// The stored RTL traces of the frontend tops (tests/reference/gemmini_fe/<top>/): every trace
// replayed through the model (ExecuteUnit, CommandPath, Controller) with every output port equal
// in every cycle (fe_qualifiers: the completion id only while its valid bit is high), and
// regenerated from its stimulus (bench/fe_stimulus.cpp) on the model alone: the same cycles,
// every input byte for byte, every output as in the replay. For
// CtrlTop traces the Engine of the regeneration must also conserve its accounting
// (Engine::check) and store every matmul's C equal to a plain C++ matmul.
//
//   fe_trace_test --ref-dir tests/reference/gemmini_fe --top ExecuteTop|CmdTop|CtrlTop
// Prints "FE_TRACES top=<t> traces=<n> cycles=<n> values=<n> mismatches=<n> regenerated=<n>/<n>
//         engine_failures=<n> matmuls=<n>".
#include "fe_run.h"
#include "fe_stimulus.h"
#include "trace_io.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace systolique;

int main(int argc, char **argv) {
  std::string ref, tops;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--ref-dir")
      ref = argv[i + 1];
    else if (a == "--top")
      tops = argv[i + 1];
  }
  FeTop top;
  if (ref.empty() || !parse_fe_top(tops, top)) {
    std::fprintf(stderr, "usage: %s --ref-dir <dir> --top ExecuteTop|CmdTop|CtrlTop\n", argv[0]);
    return 2;
  }
  const FrontendConfig cfg = FrontendConfig::gemmini_default();
  const auto spec = fe_port_specs(cfg, top);
  std::vector<std::string> names;
  for (const auto &e : std::filesystem::directory_iterator(ref + "/" + tops)) {
    const std::string f = e.path().filename().string();
    if (f.size() > 7 && f.substr(f.size() - 7) == ".csv.gz")
      names.push_back(f.substr(0, f.size() - 7));
  }
  std::sort(names.begin(), names.end());
  uint64_t cycles = 0, values = 0;
  unsigned mismatches = 0, regenerated = 0, engine_failures = 0, matmuls = 0;
  for (const std::string &n : names) {
    TraceInfo info;
    std::vector<Frame> rows;
    std::string err;
    if (!read_trace(ref + "/" + tops + "/" + n + ".csv.gz", spec, info, rows, err)) {
      std::printf("FAIL %s: %s\n", n.c_str(), err.c_str());
      ++mismatches;
      continue;
    }
    const FeRun r = replay_fe(cfg, top, rows);
    cycles += r.compared;
    values += r.values;
    if (!r.ok) {
      std::printf("FAIL %s replay: %s\n", n.c_str(), r.error.c_str());
      ++mismatches;
    }
    // regeneration from the stimulus, on the model alone
    FeRun g;
    if (top == FeTop::Execute) {
      ExStimulus s;
      if (ex_stimulus(n, s))
        g = run_ex(cfg, s);
      else
        g.ok = false, g.error = "no such stimulus";
    } else if (top == FeTop::Cmd) {
      CmdStimulus s;
      if (cmd_stimulus(n, s))
        g = run_cmd(cfg, s);
      else
        g.ok = false, g.error = "no such stimulus";
    } else {
      EngineOptions opt;
      opt.array.provenance = false;
      CtrlScenario s;
      if (ctrl_scenario(n, opt, s)) {
        g = run_ctrl(cfg, s);
        if (const std::string e = s.engine->check(); !e.empty()) {
          std::printf("FAIL %s engine: %s\n", n.c_str(), e.c_str());
          ++engine_failures;
        }
        for (const Operation &o : s.engine->micro_ops().ops())
          if (o.is_matmul) {
            ++matmuls;
            if (s.engine->result(o.id) != s.engine->reference(o.id)) {
              std::printf("FAIL %s: matmul op %d result differs from the reference\n", n.c_str(), o.id);
              ++engine_failures;
            }
          }
      } else {
        g.ok = false, g.error = "no such stimulus";
      }
    }
    // regenerated: the same cycles, every input port byte for byte, every output port as in the
    // replay (equal, the completion id while valid)
    bool same = g.ok && g.rows.size() == rows.size();
    for (size_t t = 0; same && t < rows.size(); ++t) {
      for (size_t p = 0; p < spec.size(); ++p)
        if (spec[p].input && g.rows[t][p] != rows[t][p])
          same = false;
      uint64_t v = 0;
      if (!fe_diff(spec, rows[t], g.rows[t], v).empty())
        same = false;
    }
    if (same)
      ++regenerated;
    else
      std::printf("FAIL %s regeneration: %s\n", n.c_str(),
                  g.ok ? (std::to_string(g.rows.size()) + " rows, stored " + std::to_string(rows.size()) +
                          (g.rows.size() == rows.size() ? ", contents differ" : ""))
                             .c_str()
                       : g.error.c_str());
  }
  std::printf("FE_TRACES top=%s traces=%zu cycles=%llu values=%llu mismatches=%u regenerated=%u/%zu "
              "engine_failures=%u matmuls=%u\n",
              tops.c_str(), names.size(), (unsigned long long)cycles, (unsigned long long)values, mismatches,
              regenerated, names.size(), engine_failures, matmuls);
  return mismatches || engine_failures || regenerated != names.size() || names.empty() ? 1 : 0;
}
