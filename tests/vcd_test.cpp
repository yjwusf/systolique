// The VCD writer does not change timing, and its files are well formed:
//
//   vcd_test --ref-dir <dir> --out <dir>
//
// For stored RTL traces of every configuration (all of dim4 and tiled, two per other
// configuration) the replay runs once without and once with ArrayOptions::vcd_path: both must
// match the trace in every cycle, and the outputs, requests and accounting of the two runs must
// be identical. Each VCD file must have a header, the port and PE scopes, and two time steps per
// cycle in increasing order. Prints VCD ok traces=<n> cycles=<n> bytes=<n>.
#include "run.h"
#include "stimulus.h"
#include "trace_io.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace systolique;

namespace {

bool same_accounting(const Accounting &x, const Accounting &y) {
  if (x.cycles != y.cycles || x.busy_begin != y.busy_begin || x.busy_end != y.busy_end ||
      x.per_cycle.size() != y.per_cycle.size())
    return false;
  for (size_t t = 0; t < x.per_cycle.size(); ++t)
    if (x.per_cycle[t].n != y.per_cycle[t].n ||
        x.per_cycle[t].load_concurrent != y.per_cycle[t].load_concurrent ||
        x.per_cycle[t].drain_concurrent != y.per_cycle[t].drain_concurrent)
      return false;
  return true;
}

// "" if the file looks like a VCD of `steps` time steps (the initial $dumpvars included).
std::string check_vcd(const std::string &path, size_t steps, unsigned pes, size_t &bytes) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string s = ss.str();
  bytes = s.size();
  for (const char *must : {"$timescale", "$enddefinitions $end", "$scope module ports $end",
                           "$scope module pe_0_0 $end", "$dumpvars", " clock $end"})
    if (s.find(must) == std::string::npos) return std::string("no ") + must;
  if (s.find("$scope module pe_" + std::to_string(pes - 1) + "_") == std::string::npos &&
      s.find("$scope module pe_" + std::to_string(pes == 1 ? 0 : 1)) == std::string::npos)
    return "no PE scopes";
  std::istringstream in(s);
  std::string line;
  long long prev = -1;
  size_t n = 0;
  while (std::getline(in, line))
    if (!line.empty() && line[0] == '#') {
      const long long t = std::stoll(line.substr(1));
      if (t <= prev) return "time steps out of order at " + line;
      prev = t;
      ++n;
    }
  if (n != steps) return std::to_string(n) + " time steps, expected " + std::to_string(steps);
  return "";
}

}  // namespace

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::string ref_dir, out;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--ref-dir") ref_dir = argv[i + 1];
    if (a == "--out") out = argv[i + 1];
  }
  if (ref_dir.empty() || out.empty()) {
    std::fprintf(stderr, "usage: %s --ref-dir dir --out dir\n", argv[0]);
    return 2;
  }
  mkdir(out.c_str(), 0755);
  unsigned traces = 0, failures = 0;
  unsigned long long cycles = 0, bytes = 0;
  for (const ArrayConfig &cfg : named_configs()) {
    unsigned taken = 0;
    for (const TestDef &t : test_catalog(cfg)) {
      const bool all = cfg.name == "dim4" || cfg.name == "tiled";
      if (!all && taken == 2) break;
      if (!all && t.name != "mesh_sparse" && t.name.find("back_to_back") == std::string::npos)
        continue;
      ++taken;
      const auto spec = port_specs(cfg, t.top);
      TraceInfo info;
      std::vector<Frame> rows;
      std::string err;
      const std::string label = cfg.name + "/" + t.name;
      if (!read_trace(ref_dir + "/" + cfg.name + "/" + t.name + ".csv.gz", spec, info, rows, err)) {
        std::printf("FAIL %s\n", err.c_str());
        ++failures;
        continue;
      }
      ArrayOptions plain;
      plain.interface = t.top == Top::Mesh ? Interface::Mesh : Interface::MeshWithDelays;
      ArrayOptions traced = plain;
      traced.vcd_path = out + "/" + cfg.name + "_" + t.name + ".vcd";
      ArrayRun r1, r2;
      Accounting a1, a2;
      size_t requests1 = 0, requests2 = 0;
      {
        SystolicArray a(cfg, plain);
        r1 = replay_trace(a, t.top, rows);
        a1 = a.accounting();
        requests1 = a.requests().size();
      }
      {
        SystolicArray a(cfg, traced);  // the VCD file is complete when `a` is destroyed
        r2 = replay_trace(a, t.top, rows);
        a2 = a.accounting();
        requests2 = a.requests().size();
      }
      ++traces;
      cycles += rows.size();
      if (!r1.ok || !r2.ok || r1.rows != r2.rows || requests1 != requests2 ||
          !same_accounting(a1, a2)) {
        std::printf("FAIL %s: the run with the VCD writer differs (%s%s)\n", label.c_str(),
                    r1.error.c_str(), r2.error.c_str());
        ++failures;
        continue;
      }
      size_t n = 0;
      const std::string e = check_vcd(traced.vcd_path, 2 * rows.size(), cfg.rows() * cfg.cols(), n);
      bytes += n;
      if (!e.empty()) {
        std::printf("FAIL %s: %s: %s\n", label.c_str(), traced.vcd_path.c_str(), e.c_str());
        ++failures;
        continue;
      }
      std::remove(traced.vcd_path.c_str());
    }
  }
  if (failures) return 1;
  std::printf("VCD ok traces=%u cycles=%llu bytes=%llu\n", traces, cycles, bytes);
  return 0;
}
