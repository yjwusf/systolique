// A scenario through SystolicArray, written as JSON for tools/array_view.py (the HTML viewer):
// per cycle every PE's state, the request of its row, its active register and the concurrent
// flags; every change of a register's value or source; the sources (operand
// elements); the requests with their cycles and counters; per-cycle, per-PE and total counters.
//
//   systolique_dump --config <c> --scenario <ws_single|ws_stream|os_single|os_stream>
//                   [--tiles K] [--seed S] [--bubble p] --out <file.json>
//   systolique_dump [--config <c>] --requests <requests.json> --out <file.json>
//   systolique_dump --matmul MxKxN[:WS|:OS]... [--bias] [--relu] [--full-c] [--seed S]
//                   [--dma-latency L] [--dma-bytes B] --out <file.json>
//
// --matmul runs an Engine (include/systolique/engine.h) on Gemmini's default 16x16 configuration:
// each matmul (one operation each, in order, WS unless :OS) as gemmini.h issues it, through the command path, the ExecuteController, the banks
// and the array (cycle-exact against the RTL), with the modelled DMA (DmaParams, unvalidated).
// The JSON then also has the micro-ops ("uops": every command, request and flush micro-op with its
// cycles; per-row micro-ops are counted in their command), the operations ("ops") and, per
// request, its operation and micro-ops (op, uop: whose PE-cycles; load_op, load_uop: whose D).
//
// Scenarios (bench/reference.h): ws_* = ws_case, os_* = os_case (single: K = 1, stream: K =
// --tiles, default 4), random int operands from --seed. A request file (JSON, tools/json.h) is
//   {"config": "dim4", "seed": 1, "requests": [
//     {"dataflow": "WS", "propagate": 1, "rows": 4, "a": "zero", "b": "zero", "d": "random",
//      "tag": 0, "label": "preload W0"},
//     {"dataflow": "WS", "propagate": 1, "rows": 4, "a": "random", "b": "random", "d": "zero",
//      "tag": 1, "a_transpose": false, "bd_transpose": false, "shift": 0, "delay": 0,
//      "computes": true, "preloads": false},
//     {"flush": 1, "dataflow": "OS", "propagate": 1}]}
// where a/b/d are "random", "zero" or a list of rows; "computes"/"preloads" are RequestNotes.
//
// Frames are delta-encoded: a cycle's entry is "f" + every PE in row-major order, or "d" + the
// PEs that changed since the previous cycle, each prefixed with its index (2 base-64 digits).
// A PE is a code (1 base-64 digit: state + 5 * active register + 10 * load_concurrent + 20 *
// drain_concurrent) and the index + 1 of the request of its row (2 base-64 digits, 0 = none).
// Register events per PE, whenever a register's value or source changes: [cycle, register
// (0 = c1), source (-1: none), loaded] plus the value when it is not the source's d value.
#include "json.h"
#include "reference.h"
#include "run.h"
#include "stimulus.h"

#include "systolique/engine.h"
#include "systolique/systolic_array.h"

#include <array>
#include <cstdio>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace systolique;

namespace {

const char kB64[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_";

std::string b64(unsigned v, unsigned digits) {
  std::string s(digits, '0');
  for (unsigned k = digits; k-- > 0; v >>= 6) s[k] = kB64[v & 63];
  return s;
}

std::string jstr(const std::string &s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    if (c == '\n') {
      o += "\\n";
      continue;
    }
    o += c;
  }
  return o + "\"";
}

std::string counts_json(const StateCounts &s) {
  std::ostringstream o;
  o << "{";
  for (unsigned k = 0; k < kPeStates; ++k) o << jstr(to_string(PeState(k))) << ":" << s.n[k] << ",";
  o << "\"load_concurrent\":" << s.load_concurrent << ",\"drain_concurrent\":" << s.drain_concurrent
    << "}";
  return o.str();
}

Matrix matrix_spec(const json::Value &n, unsigned rows, unsigned cols, unsigned bits,
                   std::mt19937_64 &rng) {
  if (n.type == json::Value::Type::Null || (n.is_string() && n.as_string() == "zero"))
    return zero_matrix(rows, cols);
  if (n.is_string() && n.as_string() == "random") return random_matrix(rng, rows, cols, bits);
  if (n.type != json::Value::Type::Array) throw std::runtime_error("a matrix is \"zero\", \"random\" or rows");
  Matrix m;
  for (const auto &r : n.arr) {
    std::vector<int64_t> row;
    for (const auto &v : r.arr) row.push_back(v.as_int());
    if (row.size() != cols) throw std::runtime_error("a matrix row needs " + std::to_string(cols) + " values");
    m.push_back(row);
  }
  if (m.size() != rows) throw std::runtime_error("a matrix needs " + std::to_string(rows) + " rows");
  return m;
}

struct Scenario {
  std::string name;
  std::vector<Op> ops;
  std::vector<RequestNote> notes;  // per op, empty labels and -1 if none
};

Scenario from_file(const std::string &path, std::string &config) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot read " + path);
  std::stringstream text;
  text << in.rdbuf();
  const json::Value root = json::parse(text.str());
  if (config.empty() && root.has("config")) config = root["config"].as_string();
  if (config.empty()) config = "dim4";
  const ArrayConfig *cfg = find_config(config);
  if (!cfg) throw std::runtime_error("unknown configuration " + config);
  std::mt19937_64 rng(root.has("seed") ? uint64_t(root["seed"].as_int()) : 1);
  Scenario s;
  s.name = path;
  const unsigned dim = cfg->block_size();
  for (const auto &r : root["requests"].arr) {
    const std::string dfs = r.has("dataflow") ? r["dataflow"].as_string() : "WS";
    const Dataflow df = dfs == "OS" ? Dataflow::OS : Dataflow::WS;
    const unsigned prop = r.has("propagate") ? unsigned(r["propagate"].as_int()) : 1;
    Op op;
    if (r.has("flush") && r["flush"].as_int() > 0) {
      op = make_flush(*cfg, df, prop, unsigned(r["flush"].as_int()));
    } else {
      const unsigned rows = r.has("rows") ? unsigned(r["rows"].as_int()) : dim;
      const Matrix a = matrix_spec(r["a"], rows, dim, cfg->in_bits, rng);
      const Matrix b = matrix_spec(r["b"], rows, dim, cfg->in_bits, rng);
      const Matrix d = matrix_spec(r["d"], rows, dim, cfg->in_bits, rng);
      op = make_op(*cfg, df, prop, r.has("shift") ? unsigned(r["shift"].as_int()) : 0, a, b, d,
                   r.has("tag") ? unsigned(r["tag"].as_int()) : unsigned(s.ops.size()));
      op.req.a_transpose = r.has("a_transpose") && r["a_transpose"].as_bool();
      op.req.bd_transpose = r.has("bd_transpose") && r["bd_transpose"].as_bool();
    }
    op.delay = r.has("delay") ? unsigned(r["delay"].as_int()) : 0;
    RequestNote n;
    if (r.has("label")) n.label = r["label"].as_string();
    if (r.has("computes")) n.computes = r["computes"].as_bool();
    if (r.has("preloads")) n.preloads = r["preloads"].as_bool();
    s.ops.push_back(op);
    s.notes.push_back(n);
  }
  return s;
}

Scenario builtin(const ArrayConfig &cfg, const std::string &name, unsigned K, uint64_t seed) {
  std::mt19937_64 rng(seed);
  Scenario s;
  s.name = name;
  const bool ws = name.rfind("ws_", 0) == 0;
  if (name.find("_single") != std::string::npos) K = 1;
  const MatmulCase c = ws ? ws_case(cfg, rng, K, false, false) : os_case(cfg, rng, K, false, false, 0);
  s.ops = c.ops;
  // Labels for the legend: what each request of the stream does (reference.cpp).
  for (size_t j = 0; j < s.ops.size(); ++j) {
    RequestNote n;
    const std::string jj = std::to_string(j), jm = std::to_string(int(j) - 1);
    if (ws)
      n.label = j == 0 ? "preload W0" : (j < K ? "compute A" + jm + "W" + jm + ", preload W" + jj
                                               : "compute A" + jm + "W" + jm);
    else if (s.ops[j].req.flush)
      n.label = "flush: shift C" + std::to_string(K - 1) + " out";
    else
      n.label = j == 0 ? "preload D0"
                       : (j < K ? "compute A" + jm + "B" + jm + ", preload D" + jj +
                                      (j >= 2 ? ", shift C" + std::to_string(j - 2) + " out" : "")
                                : "compute A" + jm + "B" + jm +
                                      (j >= 2 ? ", shift C" + std::to_string(j - 2) + " out" : ""));
    s.notes.push_back(n);
  }
  return s;
}

}  // namespace

int main(int argc, char **argv) {
  std::string config, scenario, requests, out;
  std::vector<std::string> matmuls;
  bool bias = false, relu = false, full_c = false;
  DmaParams dma;
  unsigned K = 4;
  uint64_t seed = 1;
  double bubble = 0;
  std::string cmdline = "systolique_dump";  // the command without --out, for the page
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--out") {
      ++i;
      continue;
    }
    cmdline += " " + std::string(argv[i]);
  }
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
      return argv[++i];
    };
    if (a == "--config") config = next();
    else if (a == "--scenario") scenario = next();
    else if (a == "--requests") requests = next();
    else if (a == "--tiles") K = unsigned(std::stoul(next()));
    else if (a == "--seed") seed = std::stoull(next());
    else if (a == "--bubble") bubble = std::stod(next());
    else if (a == "--out") out = next();
    else if (a == "--matmul") matmuls.push_back(next());
    else if (a == "--bias") bias = true;
    else if (a == "--relu") relu = true;
    else if (a == "--full-c") full_c = true;
    else if (a == "--dma-latency") dma.latency = unsigned(std::stoul(next()));
    else if (a == "--dma-bytes") dma.bytes_per_cycle = unsigned(std::stoul(next()));
    else {
      std::fprintf(stderr, "usage: %s --config c --scenario ws_single|ws_stream|os_single|"
                           "os_stream [--tiles K] [--seed S] [--bubble p] --out f.json\n"
                           "       %s [--config c] --requests r.json --out f.json\n",
                   argv[0], argv[0]);
      return 2;
    }
  }
  if (out.empty() || int(!scenario.empty()) + int(!requests.empty()) + int(!matmuls.empty()) != 1) {
    std::fprintf(stderr, "need --out and one of --scenario / --requests / --matmul\n");
    return 2;
  }
  Scenario sc;
  std::vector<Matmul> mms;
  try {
    if (!matmuls.empty()) {
      for (const std::string &spec : matmuls) {
        Matmul mm;
        char df[8] = "WS";
        const int n = std::sscanf(spec.c_str(), "%ux%ux%u:%2s", &mm.M, &mm.K, &mm.N, df);
        if (n < 3 || (std::string(df) != "WS" && std::string(df) != "OS"))
          throw std::runtime_error("--matmul MxKxN[:WS|:OS]");
        mm.dataflow = std::string(df) == "WS" ? kWS : kOS;
        mm.bias = bias;
        mm.full_c = full_c;
        mm.act = relu ? ACT_RELU : ACT_NONE;
        mm.seed = uint32_t(seed + mms.size());
        mms.push_back(mm);
        sc.name += (sc.name.empty() ? "matmul " : ", matmul ") + std::to_string(mm.M) + "x" +
                   std::to_string(mm.K) + "x" + std::to_string(mm.N) + " " + df;
      }
      config = "default";
      sc.name += std::string(bias ? " bias" : "") + (relu ? " relu" : "") + (full_c ? " full_c" : "");
    } else if (!requests.empty()) {
      sc = from_file(requests, config);
    } else {
      if (config.empty()) config = "dim4";
      const ArrayConfig *c = find_config(config);
      if (!c) throw std::runtime_error("unknown configuration " + config);
      if (scenario != "ws_single" && scenario != "ws_stream" && scenario != "os_single" &&
          scenario != "os_stream")
        throw std::runtime_error("unknown scenario " + scenario);
      sc = builtin(*c, scenario, K, seed);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 2;
  }
  const ArrayConfig &cfg = *find_config(config);
  const unsigned dim = cfg.rows(), cols = cfg.cols(), pes = dim * cols;
  if (pes > 64 * 64) return 2;

  std::vector<std::string> frames;
  std::vector<std::string> prev(pes);
  std::vector<std::vector<std::vector<int64_t>>> events(pes);
  std::vector<std::array<int64_t, 2>> last_src(pes, {-2, -2}), last_val(pes, {0, 0}),
      last_loaded(pes, {-2, -2});
  std::map<std::pair<int64_t, unsigned>, unsigned> src_index;  // (d row, lane) -> source
  std::vector<RegSource> sources;
  auto capture = [&](const SystolicArray &a) {
        std::string full, delta;
        for (unsigned p = 0; p < pes; ++p) {
          const PeView v = a.pe(p / cols, p % cols);
          const unsigned code = unsigned(v.state) + 5 * v.active + 10 * v.load_concurrent +
                                20 * v.drain_concurrent;
          const std::string e = b64(code, 1) + b64(unsigned(v.request + 1), 2);
          full += e;
          if (e != prev[p]) delta += b64(p, 2) + e;
          prev[p] = e;
          for (unsigned k = 0; k < 2; ++k) {
            const RegSource &s = v.src[k];
            int64_t si = -1;
            if (s.d_row >= 0) {
              auto key = std::make_pair(s.d_row, s.lane);
              auto it = src_index.find(key);
              if (it == src_index.end()) {
                it = src_index.emplace(key, unsigned(sources.size())).first;
                sources.push_back(s);
              }
              si = it->second;
            }
            const int64_t val = k ? v.regs.c2 : v.regs.c1;
            if (si != last_src[p][k] || val != last_val[p][k] || s.loaded != last_loaded[p][k]) {
              std::vector<int64_t> ev{a.cycle(), k, si, s.loaded};
              if (si < 0 || val != s.value) ev.push_back(val);
              events[p].push_back(ev);
              last_src[p][k] = si;
              last_val[p][k] = val;
              last_loaded[p][k] = s.loaded;
            }
          }
        }
        frames.push_back(delta.size() < full.size() ? "d" + delta : "f" + full);
      };
  std::unique_ptr<Engine> engine;
  std::unique_ptr<SystolicArray> own;
  if (!mms.empty()) {
    EngineOptions eo;
    eo.dma = dma;
    engine = std::make_unique<Engine>(eo);
    for (const Matmul &mm : mms) engine->submit(mm);
    engine->set_hook([&](const Engine &e, const CtrlTopIn &, const CtrlTopOut &) { capture(e.array()); });
    try {
      engine->run();
    } catch (const std::exception &e) {
      std::fprintf(stderr, "run failed: %s\n", e.what());
      return 1;
    }
    if (const std::string e = engine->check(); !e.empty()) {
      std::fprintf(stderr, "micro-op attribution: %s\n", e.c_str());
      return 1;
    }
  } else {
    // Notes go with the request offered in a cycle: the k-th request the driver offers.
    own = std::make_unique<SystolicArray>(cfg);
    SystolicArray &array = *own;
    MwdDriver drv(cfg, sc.ops, DriverOptions{bubble, 0, seed});
    ArrayRun run = run_stimulus(
        array, Top::MeshWithDelays, drv, 4, 1000000, capture,
        [&](const MwdIn &in) -> const RequestNote * {
          // The driver offers ops in order; the note of the op on offer this cycle.
          if (!in.req_valid) return nullptr;
          const size_t k = array.requests().size();
          return k < sc.notes.size() ? &sc.notes[k] : nullptr;
        });
    if (!run.ok) {
      std::fprintf(stderr, "run failed: %s\n", run.error.c_str());
      return 1;
    }
  }
  const SystolicArray &array = engine ? engine->array() : *own;
  const Accounting acc = array.accounting();
  if (const std::string e = SystolicArray::check(acc); !e.empty()) {
    std::fprintf(stderr, "accounting: %s\n", e.c_str());
    return 1;
  }

  std::ofstream f(out);
  f << "{\"meta\":{\"config\":" << jstr(cfg.name) << ",\"dim\":" << dim << ",\"cols\":" << cols
    << ",\"tile_rows\":" << cfg.tile_rows << ",\"tile_cols\":" << cfg.tile_cols
    << ",\"tile_latency\":" << cfg.tile_latency << ",\"output_delay\":" << cfg.output_delay
    << ",\"array_dataflow\":" << jstr(to_string(cfg.dataflow)) << ",\"in_bits\":" << cfg.in_bits
    << ",\"out_bits\":" << cfg.out_bits << ",\"scenario\":" << jstr(sc.name)
    << ",\"command\":" << jstr(cmdline) << ",\"cycles\":" << acc.cycles
    << ",\"busy_begin\":" << acc.busy_begin << ",\"busy_end\":" << acc.busy_end
    << ",\"states\":[";
  for (unsigned k = 0; k < kPeStates; ++k) f << (k ? "," : "") << jstr(to_string(PeState(k)));
  f << "],\"rule\":\"1 PE = 1 MAC per cycle\",\"engine\":" << (engine ? "true" : "false");
  if (engine)
    f << ",\"dma_latency\":" << dma.latency << ",\"dma_bytes_per_cycle\":" << dma.bytes_per_cycle;
  f << "},\n";
  f << "\"totals\":{\"run\":" << counts_json(acc.total) << ",\"busy\":" << counts_json(acc.busy)
    << ",\"occupancy\":" << acc.occupancy << ",\"utilisation\":" << acc.utilisation
    << ",\"busy_occupancy\":" << acc.busy_occupancy << ",\"busy_utilisation\":"
    << acc.busy_utilisation << "},\n\"requests\":[";
  for (const RequestInfo &r : array.requests()) {
    f << (r.index ? ",\n" : "") << "{\"index\":" << r.index << ",\"label\":" << jstr(r.label)
      << ",\"tag_valid\":" << r.req.tag_valid << ",\"tag\":" << r.req.tag_id
      << ",\"dataflow\":" << jstr(to_string(r.dataflow)) << ",\"flush\":" << r.req.flush
      << ",\"propagate\":" << r.req.propagate << ",\"a_transpose\":" << r.req.a_transpose
      << ",\"bd_transpose\":" << r.req.bd_transpose << ",\"total_rows\":" << r.req.total_rows
      << ",\"accept\":" << r.accept << ",\"first_in\":" << r.first_in << ",\"last_in\":" << r.last_in
      << ",\"first_out\":" << r.first_out << ",\"last_out\":" << r.last_out
      << ",\"result_first\":" << r.result_first << ",\"result_last\":" << r.result_last
      << ",\"rows_in\":" << r.rows_in << ",\"rows_out\":" << r.rows_out
      << ",\"result_rows\":" << r.result_rows << ",\"computes\":" << r.computes
      << ",\"preloads\":" << r.preloads << ",\"state\":" << jstr(to_string(r.state()))
      << ",\"op\":" << r.op << ",\"uop\":" << r.uop << ",\"load_op\":" << r.load_op
      << ",\"load_uop\":" << r.load_uop << ",\"pe_cycles\":" << counts_json(acc.per_request[r.index])
      << ",\"macs\":" << acc.per_request[r.index].macs() << "}";
  }
  f << "],\n\"per_cycle\":[";
  for (unsigned k = 0; k < kPeStates + 2; ++k) {
    f << (k ? "," : "") << "[";
    for (int64_t t = 0; t < acc.cycles; ++t) {
      const StateCounts &s = acc.per_cycle[size_t(t)];
      f << (t ? "," : "")
        << (k < kPeStates ? s.n[k] : k == kPeStates ? s.load_concurrent : s.drain_concurrent);
    }
    f << "]";
  }
  f << "],\n\"per_pe\":[";
  for (unsigned p = 0; p < pes; ++p) {
    const StateCounts &s = acc.per_pe[p];
    f << (p ? "," : "") << "[";
    for (unsigned k = 0; k < kPeStates; ++k) f << s.n[k] << ",";
    f << s.load_concurrent << "," << s.drain_concurrent << "]";
  }
  // sources: [request, w_row, w_col, value, d handshake, lane, transposed, handshake cycle,
  //           row of the request's d operand]
  f << "],\n\"sources\":[";
  for (size_t k = 0; k < sources.size(); ++k) {
    const RegSource &s = sources[k];
    f << (k ? "," : "") << "[" << s.request << "," << s.w_row << "," << s.w_col << "," << s.value
      << "," << s.d_row << "," << s.lane << "," << s.transposed << ","
      << array.d_rows()[size_t(s.d_row)].cycle << "," << s.row << "]";
  }
  f << "]";
  if (engine) {
    // uops: [id, kind, op, parent, funct, request, sent, issued, started, accept, first_in, last_in,
    //        first_out, last_out, result_first, result_last, wb_done, completed, reads, wb_rows,
    //        pass kind (Request / Flush: 0 preload, 1 compute+preload, 2 compute, 3 flush; else -1),
    //        compute uop, preload uop, rows_in]
    f << ",\n\"uop_kinds\":[";
    for (unsigned k = 0; k <= unsigned(UopKind::DmaRow); ++k) f << (k ? "," : "") << jstr(to_string(UopKind(k)));
    f << "],\n\"uops\":[";
    bool first = true;
    for (const MicroOp &u : engine->micro_ops().all()) {
      if (u.kind == UopKind::OperandRead || u.kind == UopKind::ResultRow || u.kind == UopKind::DmaRow) continue;
      const bool pass = u.kind == UopKind::Request || u.kind == UopKind::Flush;
      f << (first ? "" : ",\n") << "[" << u.id << "," << unsigned(u.kind) << "," << u.op << "," << u.parent << ","
        << u.funct << "," << u.request << "," << u.sent << "," << u.issued << "," << u.started << ","
        << u.accept << "," << u.first_in << "," << u.last_in << "," << u.first_out << "," << u.last_out << ","
        << u.result_first << "," << u.result_last << "," << u.wb_done << "," << u.completed << "," << u.reads
        << "," << u.wb_rows << "," << (pass ? int(u.pass_kind) : -1) << "," << u.compute_uop << ","
        << u.preload_uop << "," << u.rows_in << "]";
      first = false;
    }
    // ops: [id, label, first_sent, last_sent, end, workload MACs, MAC PE-cycles, micro-ops]
    f << "],\n\"ops\":[";
    for (const Operation &o : engine->micro_ops().ops()) {
      const auto it = acc.per_op.find(o.id);
      uint64_t n = 0;
      for (const MicroOp &u : engine->micro_ops().all()) n += u.op == o.id;
      f << (o.id ? ",\n" : "") << "[" << o.id << "," << jstr(o.label) << "," << o.first_sent << "," << o.last_sent
        << "," << o.end << "," << o.workload_macs << "," << (it == acc.per_op.end() ? 0 : it->second.macs()) << ","
        << n << "]";
    }
    f << "]";
  }
  f << ",\n\"frames\":[";
  for (size_t t = 0; t < frames.size(); ++t) f << (t ? "," : "") << jstr(frames[t]);
  f << "],\n\"events\":[";
  for (unsigned p = 0; p < pes; ++p) {
    f << (p ? ",\n" : "") << "[";
    for (size_t k = 0; k < events[p].size(); ++k) {
      f << (k ? "," : "") << "[";
      for (size_t j = 0; j < events[p][k].size(); ++j) f << (j ? "," : "") << events[p][k][j];
      f << "]";
    }
    f << "]";
  }
  f << "]}\n";
  if (!f) {
    std::fprintf(stderr, "cannot write %s\n", out.c_str());
    return 1;
  }
  std::printf("DUMP config=%s scenario=%s cycles=%lld requests=%zu macs=%llu occupancy=%.3f "
              "utilisation=%.3f busy=[%lld,%lld] wrote %s\n",
              cfg.name.c_str(), sc.name.c_str(), (long long)acc.cycles, array.requests().size(),
              (unsigned long long)acc.total.macs(), acc.occupancy, acc.utilisation,
              (long long)acc.busy_begin, (long long)acc.busy_end, out.c_str());
  return 0;
}
