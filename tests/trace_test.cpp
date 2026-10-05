// The stored RTL traces through SystolicArray (no RTL needed):
//
//   trace_test --ref-dir <dir> --config <c>
//
// For every test of the catalog (bench/stimulus.h) of the configuration:
//   replay       the recorded inputs into a fresh SystolicArray; every output port must equal
//                the recorded RTL output in every cycle (MeshTop traces through the bare Mesh
//                interface, MeshWithDelaysTop traces through the MeshWithDelays interface)
//   conservation SystolicArray::check on the replayed run
//   provenance   on WS-only runs, every register equals the d value its provenance names, in
//                every cycle (WS never rewrites a stationary register)
//   regenerate   the test's stimulus run on SystolicArray reproduces the trace exactly (inputs
//                and outputs), so the stimulus code still matches the references
//   functional   for the matmul tests, the recorded RTL responses equal a plain C++ matmul
// and every trace file in <dir>/<c> must belong to a test.
//
// Prints one line per test and
//   TRACES config=<c> tests=<n> cycles=<n> compared=<n> functional_values=<n> mismatches=<n>
//     missing=<n> conservation_failures=<n> provenance_checks=<n> provenance_failures=<n>
#include "checks.h"
#include "reference.h"
#include "run.h"
#include "stimulus.h"
#include "trace_io.h"

#include <cstdio>
#include <dirent.h>
#include <set>
#include <string>
#include <vector>

using namespace systolique;
using namespace systolique::test;

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::string ref_dir, name;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--ref-dir" && i + 1 < argc)
      ref_dir = argv[++i];
    else if (a == "--config" && i + 1 < argc)
      name = argv[++i];
    else {
      std::fprintf(stderr, "usage: %s --ref-dir dir --config name\n", argv[0]);
      return 2;
    }
  }
  const ArrayConfig *cfgp = find_config(name);
  if (ref_dir.empty() || !cfgp) {
    std::fprintf(stderr, "need --ref-dir and a known --config\n");
    return 2;
  }
  const ArrayConfig &cfg = *cfgp;
  const std::string dir = ref_dir + "/" + cfg.name;
  unsigned tests = 0, mismatches = 0, missing = 0, conservation = 0;
  unsigned long long cycles = 0, compared = 0, values = 0;
  Checks prov;
  std::set<std::string> expected_files;
  for (const auto &t : test_catalog(cfg)) {
    ++tests;
    expected_files.insert(t.name + ".csv.gz");
    const std::string label = cfg.name + " " + top_name(t.top) + "/" + t.name;
    const auto spec = port_specs(cfg, t.top);
    TraceInfo info;
    std::vector<Frame> rows;
    std::string err;
    if (!read_trace(dir + "/" + t.name + ".csv.gz", spec, info, rows, err)) {
      ++missing;
      std::printf("FAIL %s %s\n", label.c_str(), err.c_str());
      continue;
    }
    ArrayOptions opt;
    opt.interface = t.top == Top::Mesh ? Interface::Mesh : Interface::MeshWithDelays;
    SystolicArray array(cfg, opt);
    // WS-only behaviour: a WS-only array, or a test of WS requests only.
    const bool ws_only = t.top == Top::MeshWithDelays &&
                         (cfg.dataflow == Dataflow::WS || t.name.rfind("ws_", 0) == 0);
    ArrayRun r = replay_trace(array, t.top, rows, [&](const SystolicArray &a) {
      if (ws_only) check_raw(a, prov);
    });
    cycles += rows.size();
    compared += r.compared;
    if (!r.ok) {
      ++mismatches;
      std::printf("FAIL %s replay: %s\n", label.c_str(), r.error.c_str());
      continue;
    }
    const std::string c = SystolicArray::check(array.accounting());
    if (!c.empty()) {
      ++conservation;
      std::printf("FAIL %s conservation: %s\n", label.c_str(), c.c_str());
      continue;
    }
    auto stim = t.make(cfg);
    ArrayRun g = run_stimulus(array, t.top, *stim);
    const bool stalled = !stim->error().empty();
    if (stalled != t.expect_stall || g.rows != rows) {
      size_t k = 0;
      while (k < g.rows.size() && k < rows.size() && g.rows[k] == rows[k]) ++k;
      ++mismatches;
      std::printf("FAIL %s regenerate: first difference at cycle %zu of %zu/%zu %s\n",
                  label.c_str(), k, g.rows.size(), rows.size(), g.error.c_str());
      continue;
    }
    unsigned n = 0;
    if (t.matmul) {
      const std::string e = check_responses(cfg, t.matmul(cfg), rows, &n);
      if (!e.empty()) {
        ++mismatches;
        std::printf("FAIL %s functional (RTL responses vs C++ matmul): %s\n", label.c_str(),
                    e.c_str());
        continue;
      }
      values += n;
    }
    std::printf("PASS %s cycles=%zu requests=%zu%s\n", label.c_str(), rows.size(),
                array.requests().size(),
                t.matmul ? (" functional_values=" + std::to_string(n)).c_str() : "");
  }
  if (DIR *d = opendir(dir.c_str())) {  // every stored trace must belong to a test
    while (dirent *e = readdir(d)) {
      const std::string f = e->d_name;
      if (f.size() > 7 && f.substr(f.size() - 7) == ".csv.gz" && !expected_files.count(f)) {
        ++mismatches;
        std::printf("FAIL %s: %s/%s belongs to no test\n", cfg.name.c_str(), dir.c_str(),
                    f.c_str());
      }
    }
    closedir(d);
  }
  std::printf("TRACES config=%s tests=%u cycles=%llu compared=%llu functional_values=%llu "
              "mismatches=%u missing=%u conservation_failures=%u provenance_checks=%llu "
              "provenance_failures=%llu%s%s\n",
              cfg.name.c_str(), tests, cycles, compared, values, mismatches, missing,
              conservation, prov.done, prov.failed, prov.failed ? " FAIL " : "",
              prov.first.c_str());
  return mismatches + missing + conservation + prov.failed ? 1 : 0;
}
