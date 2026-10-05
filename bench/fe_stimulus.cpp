#include "fe_stimulus.h"

#include <algorithm>

namespace systolique {

uint32_t name_seed(const std::string &name) {
  uint32_t h = 2166136261u;  // FNV-1a
  for (unsigned char c : name) {
    h ^= c;
    h *= 16777619u;
  }
  return h & 0xffff;
}

namespace {

constexpr unsigned DIM = 16;
constexpr uint32_t GARBAGE = kGarbageAddr;

int uniform(std::mt19937 &rng, int lo, int hi) { return lo + int(rng() % uint32_t(hi - lo + 1)); }

// ---------------------------------------------------------------- ExecuteTop
using ExCmd = ExStimulus::Cmd;
ExCmd ex(const Command &c) { return {c.funct, c.rs1, c.rs2, 0}; }

void fill_sp(ExStimulus &s, std::mt19937 &rng, uint32_t first_row, unsigned rows, int lo = -8, int hi = 8,
             bool partial = false) {
  for (unsigned r = 0; r < rows; ++r) {
    DmaSpWrite w;
    w.valid = true;
    const uint32_t row = first_row + r;
    w.bank = row >> 12;
    w.addr = row & 0xfff;
    for (unsigned k = 0; k < DIM; ++k)
      w.data[k] = int8_t(uniform(rng, lo, hi));
    w.mask = partial && (rng() & 3) == 0 ? (rng() & 0xffff) : 0xffff;
    s.sp.push_back(w);
  }
}

void fill_acc(ExStimulus &s, std::mt19937 &rng, uint32_t first_row, unsigned rows) {
  for (unsigned r = 0; r < rows; ++r) {
    DmaAccWrite w;
    w.valid = true;
    const uint32_t row = first_row + r;
    w.bank = row >> 9;
    w.addr = row & 0x1ff;
    for (unsigned k = 0; k < DIM; ++k)
      w.data[k] = uniform(rng, -1000, 1000);
    w.mask = ~0ull;
    w.acc = (rng() & 1) != 0;
    s.acc.push_back(w);
  }
}

// gemmini.h sp_tiled_matmul_os's compute loop for an I x J x K block (in tiles): A tiles at
// a_base + (i*K + k)*DIM, B tiles at b_base + (k*J + j)*DIM, C at c_base + (i*J + j)*DIM
void os_block(ExStimulus &s, unsigned I, unsigned J, unsigned K, uint32_t a_base, uint32_t b_base, uint32_t c_base,
              bool c_acc, std::mt19937 &rng, bool pad = false) {
  for (unsigned i = 0; i < I; ++i)
    for (unsigned j = 0; j < J; ++j)
      for (unsigned k = 0; k < K; ++k) {
        const uint32_t c = c_acc ? acc_addr(c_base + (i * J + j) * DIM, false) : c_base + (i * J + j) * DIM;
        const uint32_t out = k == K - 1 ? c : GARBAGE;
        const unsigned rows = pad && i == I - 1 ? 1 + rng() % DIM : DIM;
        const unsigned cols = pad && j == J - 1 ? 1 + rng() % DIM : DIM;
        const unsigned kk = pad && k == K - 1 ? 1 + rng() % DIM : DIM;
        s.cmds.push_back(ex(cmd_preload(GARBAGE, out, DIM, DIM, cols, rows)));
        s.cmds.push_back(
            ex(cmd_compute(k == 0, a_base + (i * K + k) * DIM, b_base + (k * J + j) * DIM, kk, rows, cols, kk)));
      }
}

// LoopMatmul's Execute FSM order (LoopMatmul.scala:401-492) for an I x J x K block (in tiles)
void ws_block(ExStimulus &s, unsigned I, unsigned J, unsigned K, uint32_t a_base, uint32_t b_base, uint32_t c_row,
              bool c_to_acc, bool accumulate, std::mt19937 &rng, bool pad = false) {
  for (unsigned k = 0; k < K; ++k)
    for (unsigned j = 0; j < J; ++j)
      for (unsigned i = 0; i < I; ++i) {
        const uint32_t b = b_base + (k * J + j) * DIM, a = a_base + (i * K + k) * DIM;
        const uint32_t c = c_to_acc ? acc_addr(c_row + (i * J + j) * DIM, accumulate || k != 0) : c_row + (i * J + j) * DIM;
        const unsigned a_rows = pad && i == I - 1 ? 1 + rng() % DIM : DIM;
        const unsigned b_cols = pad && j == J - 1 ? 1 + rng() % DIM : DIM;
        const unsigned kk = pad && k == K - 1 ? 1 + rng() % DIM : DIM;
        s.cmds.push_back(ex(cmd_preload(i == 0 ? b : GARBAGE, c, b_cols, kk, b_cols, a_rows)));
        s.cmds.push_back(ex(cmd_compute(i == 0, a, GARBAGE, kk, a_rows, DIM, DIM)));
      }
}

ExCmd cfg_ex(unsigned df, unsigned act, unsigned shift, unsigned a_stride = 1, bool a_t = false, bool b_t = false,
             unsigned c_stride = 1, bool set_only_strides = false) {
  return ex(cmd_config_ex(df, act, shift, a_stride, a_t, b_t, c_stride, set_only_strides));
}

}  // namespace

ExecuteTopIn ExStimulus::next(int64_t cycle) const {
  ExecuteTopIn in;
  if (!cmds.empty() && cycle >= cmd_start && cycle >= last_fire + int64_t(cmds.front().gap)) {
    in.cmd_valid = true;
    in.cmd_funct = cmds.front().funct;
    in.cmd_rs1 = cmds.front().rs1;
    in.cmd_rs2 = cmds.front().rs2;
    in.cmd_rob_id = rob % 48;
  }
  if (!sp.empty())
    in.dma_sp = sp.front();
  if (!acc.empty())
    in.dma_acc = acc.front();
  return in;
}

void ExStimulus::observe(const ExecuteTopIn &in, const ExecuteTopOut &out) {
  if (in.cmd_valid && out.cmd_ready) {
    cmds.pop_front();
    ++rob;
  }
  if (in.dma_sp.valid && out.dma_sp_taken)
    sp.pop_front();
  if (in.dma_acc.valid && out.dma_acc_taken)
    acc.pop_front();
  const bool done = cmds.empty() && sp.empty() && acc.empty() && !out.busy;
  idle = done ? idle + 1 : 0;
}

std::map<std::string, std::function<ExStimulus(std::mt19937 &)>> ex_catalog() {
  std::map<std::string, std::function<ExStimulus(std::mt19937 &)>> t;
  // OS: one tile, D preloaded from the scratchpad, C to the scratchpad (bareMetalC/matmul_os.c)
  t["os_tile"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 3 * DIM);
    s.cmds = {cfg_ex(kOS, 0, 0), ex(cmd_preload(2 * DIM, 3 * DIM)), ex(cmd_compute(true, 0, DIM))};
    s.cmd_start = 3 * DIM + 8;
    return s;
  };
  // WS: one tile, C to the accumulator (bareMetalC/matmul_ws.c)
  t["ws_tile"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 3 * DIM);
    s.cmds = {cfg_ex(kWS, 0, 0), ex(cmd_preload(DIM, acc_addr(0, false))), ex(cmd_compute(true, 0, 2 * DIM))};
    s.cmd_start = 3 * DIM + 8;
    return s;
  };
  // gemmini.h's OS tiling: 2x2x3 tiles to the accumulator, then to the scratchpad with a shift
  t["os_stream"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 12 * DIM);
    fill_sp(s, rng, 4096, 12 * DIM);
    s.cmds.push_back(cfg_ex(kOS, 0, 0));
    os_block(s, 2, 2, 3, 0, 4096, 0, true, rng);
    s.cmds.push_back(cfg_ex(kOS, 1, 2));
    os_block(s, 2, 2, 3, 0, 4096, 8192, false, rng);
    s.cmd_start = 40;
    return s;
  };
  // LoopMatmul's WS order: 2x2x3 tiles, accumulating over k in the accumulator
  t["ws_stream"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 6 * DIM);
    fill_sp(s, rng, 8192, 6 * DIM);
    s.cmds.push_back(cfg_ex(kWS, 0, 0));
    ws_block(s, 2, 2, 3, 0, 8192, 0, true, false, rng);
    s.cmd_start = 40;
    return s;
  };
  // padding: partial rows / columns / K in the last tiles, OS and WS
  t["padded"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 12 * DIM);
    fill_sp(s, rng, 4096, 12 * DIM);
    s.cmds.push_back(cfg_ex(kWS, 0, 0));
    ws_block(s, 2, 2, 2, 0, 4096, 0, true, false, rng, true);
    s.cmds.push_back(cfg_ex(kOS, 0, 0));
    os_block(s, 2, 1, 2, 0, 4096, 256, true, rng, true);
    s.cmd_start = 40;
    return s;
  };
  // results to the scratchpad that later commands read (RAW hazards on the tags in flight)
  t["hazards"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 4 * DIM);
    s.cmds = {cfg_ex(kWS, 0, 0),
              ex(cmd_preload(DIM, 2 * DIM)), ex(cmd_compute(true, 0, GARBAGE)),
              ex(cmd_preload(2 * DIM, 3 * DIM)), ex(cmd_compute(true, 2 * DIM, GARBAGE)),
              cfg_ex(kOS, 0, 0),
              ex(cmd_preload(GARBAGE, 4 * DIM)), ex(cmd_compute(true, 3 * DIM, 2 * DIM)),
              ex(cmd_preload(4 * DIM, 5 * DIM)), ex(cmd_compute(true, 4 * DIM, DIM))};
    s.cmd_start = 4 * DIM + 4;
    return s;
  };
  // dataflow switches, set_only_strides, a_stride 2, c_stride 2, transposes, ReLU and shifts
  t["configs"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 16 * DIM, -20, 20);
    s.cmds.push_back(cfg_ex(kWS, 1, 0, 2));
    s.cmds.push_back(ex(cmd_preload(DIM, 8 * DIM)));
    s.cmds.push_back(ex(cmd_compute(true, 0, GARBAGE)));
    s.cmds.push_back(cfg_ex(kWS, 0, 0, 1, true));
    s.cmds.push_back(ex(cmd_preload(DIM, acc_addr(64, false))));
    s.cmds.push_back(ex(cmd_compute(true, 2 * DIM, GARBAGE)));
    s.cmds.push_back(cfg_ex(kOS, 0, 3, 1, true, true));
    s.cmds.push_back(ex(cmd_preload(GARBAGE, 10 * DIM)));
    s.cmds.push_back(ex(cmd_compute(true, 3 * DIM, 4 * DIM)));
    s.cmds.push_back(cfg_ex(kOS, 1, 0, 1, false, false, 2));
    s.cmds.push_back(ex(cmd_preload(5 * DIM, 11 * DIM)));
    s.cmds.push_back(ex(cmd_compute(true, 6 * DIM, 7 * DIM)));
    s.cmds.push_back(cfg_ex(kWS, 0, 0, 1, false, false, 1, true));
    s.cmds.push_back(ex(cmd_preload(DIM, acc_addr(128, false))));
    s.cmds.push_back(ex(cmd_compute(true, 0, GARBAGE)));
    s.cmd_start = 16 * DIM + 4;
    return s;
  };
  // mvin writes competing with the ExecuteController's bank reads and writes
  t["bank_conflicts"] = [](std::mt19937 &rng) {
    ExStimulus s;
    fill_sp(s, rng, 0, 4 * DIM);
    s.cmds.push_back(cfg_ex(kOS, 0, 0));
    os_block(s, 2, 2, 2, 0, 0, 2048, false, rng);
    fill_sp(s, rng, 1024, 4 * DIM, -8, 8, true);
    fill_acc(s, rng, 256, 32);
    s.cmd_start = 4 * DIM;
    return s;
  };
  return t;
}

// A random mix of the patterns above with random parameters, gaps and concurrent mvin writes.
ExStimulus ex_random(std::mt19937 &rng) {
  ExStimulus s;
  fill_sp(s, rng, 0, 32 * DIM);
  fill_sp(s, rng, 4096, 16 * DIM);
  s.cmd_start = 64 + rng() % 400;
  const unsigned blocks = 2 + rng() % 4;
  for (unsigned b = 0; b < blocks; ++b) {
    const bool ws = rng() & 1;
    const unsigned I = 1 + rng() % 2, J = 1 + rng() % 2, K = 1 + rng() % 3;
    if (ws) {
      const bool a_t = (rng() % 5) == 0;
      s.cmds.push_back(cfg_ex(kWS, rng() % 2, 0, 1, a_t));
      const bool to_acc = rng() % 3 != 0;
      const uint32_t c = to_acc ? (rng() % 2) * 256 : 9216 + (rng() % 4) * 256;
      const bool accumulate = rng() & 1;
      const bool pad = (rng() % 3) == 0;
      ws_block(s, I, J, K, 0, 4096, c, to_acc, accumulate, rng, pad);
    } else {
      const bool t = (rng() % 6) == 0;
      const unsigned act = rng() % 2, shift = rng() % 4;
      s.cmds.push_back(cfg_ex(kOS, act, shift, 1, t, t));
      const bool to_acc = rng() & 1;
      const uint32_t c = to_acc ? 512 + (rng() % 2) * 128 : 9216 + (rng() % 4) * 256;
      const bool pad = (rng() % 3) == 0;
      os_block(s, I, J, K, 0, 4096, c, to_acc, rng, pad);
    }
  }
  for (auto &c : s.cmds)
    if (rng() % 4 == 0)
      c.gap = rng() % 24;
  fill_sp(s, rng, 12288, 64, -8, 8, true);  // bank 3 rows nobody reads, written meanwhile
  fill_acc(s, rng, 768, 64);
  return s;
}

// ---------------------------------------------------------------- CmdTop
namespace {
using CCmd = CmdStimulus::Cmd;
CCmd cc(const Command &c) { return {c.funct, c.rs1, c.rs2, 0}; }

// gemmini.h tiled_matmul_auto's WS sequence with the loop instruction: configurations, one
// gemmini_loop_ws per block (sp_tiled_matmul_ws), a fence.
void tiled_ws(CmdStimulus &s, std::mt19937 &rng, unsigned blocks, bool random_params) {
  const uint64_t base = 0x80010000ull;
  s.cmds.push_back(cc(cmd_config_ex(kWS, 0, 0, 1, random_params && rng() % 4 == 0, false)));
  s.cmds.push_back(cc(cmd_config_st(256, random_params ? rng() % 2 : 0)));
  s.cmds.push_back(cc(cmd_config_ld(256, 0)));
  s.cmds.push_back(cc(cmd_config_ld(256, 1)));
  s.cmds.push_back(cc(cmd_config_ld(1024, 2)));
  for (unsigned b = 0; b < blocks; ++b) {
    LoopWsArgs l;
    if (random_params) {
      l.I = 1 + rng() % 4;
      l.J = 1 + rng() % 4;
      l.K = 1 + rng() % 4;
      l.pad_I = rng() % 3 == 0 ? rng() % DIM : 0;
      l.pad_J = rng() % 3 == 0 ? rng() % DIM : 0;
      l.pad_K = rng() % 3 == 0 ? rng() % DIM : 0;
      l.D = rng() % 2 ? base + 0x40000 + (rng() % 64) * 64 : 0;
      l.C = rng() % 5 ? base + 0x80000 + (rng() % 64) * 64 : 0;
      l.full_C = rng() % 4 == 0;
      l.low_D = rng() % 4 == 0;
      l.ex_accumulate = rng() % 3 == 0;
      l.act = rng() % 2;
      l.A_transpose = rng() % 5 == 0;
      l.B_transpose = !l.A_transpose && rng() % 5 == 0;  // not both (LoopMatmul.scala:494)
      l.a_spad_id = rng() % 3;
      l.b_spad_id = rng() % 3;
    } else {
      l.I = l.J = l.K = 4;
      l.D = base + 0x40000;
      l.C = base + 0x80000;
    }
    l.A = base + b * 0x1000;
    l.B = base + 0x20000 + b * 0x1000;
    l.A_stride = 64 * (1 + rng() % 4);
    l.B_stride = 64 * (1 + rng() % 4);
    l.D_stride = 64 * (1 + rng() % 4);
    l.C_stride = 64 * (1 + rng() % 4);
    std::vector<Command> q;
    cmd_loop_ws(l, q);
    for (const auto &c : q)
      s.cmds.push_back(cc(c));
  }
  s.cmds.push_back(cc(cmd_fence()));
}

// explicit commands as bareMetalC/matmul_{os,ws}.c and mvin_mvout.c issue them, with addresses
// that overlap (the ReservationStation's dependencies)
template <class S, class C>
void explicit_cmds(S &s, std::mt19937 &rng, unsigned n, C conv) {
  s.cmds.push_back(conv(cmd_config_ld(DIM, 0)));
  s.cmds.push_back(conv(cmd_config_st(DIM)));
  for (unsigned k = 0; k < n; ++k) {
    const unsigned r = rng() % 12;
    const uint32_t sp = (rng() % 8) * DIM, sp2 = (rng() % 8) * DIM;
    const uint32_t acc_row = (rng() % 4) * DIM;
    const bool acc_accumulate = rng() & 1;
    const uint32_t acc = acc_addr(acc_row, acc_accumulate);
    if (r < 3) {
      const unsigned which = rng() % 3;
      const uint64_t dram = 0x80020000ull + (rng() % 16) * 256;
      const uint32_t local = rng() % 4 ? sp : acc;
      const unsigned cols = 1 + rng() % DIM;
      const unsigned rows = 1 + rng() % DIM;
      s.cmds.push_back(conv(cmd_mvin(which, dram, local, cols, rows)));
    } else if (r < 5) {
      const uint64_t dram = 0x80040000ull + (rng() % 16) * 256;
      const uint32_t local = rng() % 2 ? sp : acc & ~0x40000000u;
      s.cmds.push_back(conv(cmd_mvout(dram, local, DIM, DIM)));
    } else if (r < 9) {
      const uint32_t bd = rng() % 2 ? sp2 : GARBAGE;
      const uint32_t c = rng() % 2 ? acc : sp;
      s.cmds.push_back(conv(cmd_preload(bd, c)));
      const bool preloaded = rng() % 4 != 0;
      const uint32_t b = rng() % 2 ? sp2 : GARBAGE;
      s.cmds.push_back(conv(cmd_compute(preloaded, sp, b)));
    } else if (r < 10) {
      const unsigned df = rng() % 2, act = rng() % 2, shift = rng() % 4;
      s.cmds.push_back(conv(cmd_config_ex(df, act, shift)));
    } else if (r < 11) {
      const unsigned stride = DIM * (1 + rng() % 4);
      s.cmds.push_back(conv(cmd_config_ld(stride, rng() % 3)));
    } else {
      s.cmds.push_back(conv(rng() % 2 ? cmd_fence() : cmd_flush()));
    }
    if (rng() % 5 == 0)
      s.cmds.back().gap = rng() % 10;
  }
  s.cmds.push_back(conv(cmd_fence()));
}
}  // namespace

CommandPath::In CmdStimulus::next(int64_t cycle, const CommandPath::Out &regs) {
  CommandPath::In in;
  // the core: the next command, unless it is a fence that waits for busy
  bool fence_wait = false;
  while (!cmds.empty() && cmds.front().funct == kFence) {
    if (regs.busy) {
      fence_wait = true;
      break;
    }
    cmds.pop_front();
  }
  if (!fence_wait && !cmds.empty() && cycle >= last_sent + int64_t(cmds.front().gap)) {
    in.cmd_valid = true;
    in.cmd.funct = cmds.front().funct & 0x7f;
    in.cmd.rs1 = cmds.front().rs1;
    in.cmd.rs2 = cmds.front().rs2;
  }
  // the controllers
  for (unsigned i = 0; i < 3; ++i)
    in.issue_ready[i] = unsigned(rng() % 100) < ready_pct[i];
  pick = -1;
  for (size_t k = 0; k < pending.size(); ++k)
    if (pending[k].due <= cycle && (pick < 0 || pending[k].due < pending[size_t(pick)].due))
      pick = int(k);
  if (pick >= 0) {
    in.completed_valid = true;
    in.completed_id = pending[size_t(pick)].id;
  }
  return in;
}

void CmdStimulus::observe(const CommandPath::In &in, const CommandPath::Out &out, int64_t cycle) {
  for (unsigned i = 0; i < 3; ++i)
    if (out.issue[i].valid && in.issue_ready[i]) {
      // the ReservationStation completes load / store configurations on issue (:343)
      const bool on_issue = out.issue[i].funct == CONFIG_CMD && i != 1;
      if (!on_issue) {
        const unsigned d = delay_min[i] + rng() % (delay_max[i] - delay_min[i] + 1);
        pending.push_back({cycle + d, out.issue[i].rob_id});
      }
    }
  if (pick >= 0)
    pending.erase(pending.begin() + pick);
  if (in.cmd_valid && out.cmd_ready) {
    cmds.pop_front();
    last_sent = cycle;
  }
  const bool done = cmds.empty() && pending.empty() && !out.busy;
  idle = done ? idle + 1 : 0;
}

std::map<std::string, std::function<CmdStimulus(std::mt19937 &)>> cmd_catalog() {
  std::map<std::string, std::function<CmdStimulus(std::mt19937 &)>> t;
  // one 64x64x64 block (4x4x4 tiles) with bias, controllers always ready
  t["loop_ws_64"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    tiled_ws(s, rng, 1, false);
    return s;
  };
  // two blocks back to back: LoopMatmul's two loops in flight
  t["loop_ws_two"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    tiled_ws(s, rng, 2, false);
    return s;
  };
  // padding, no bias / no store, transposes, accumulate, ReLU, scratchpad ids
  t["loop_ws_params"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    tiled_ws(s, rng, 4, true);
    return s;
  };
  // slow controllers (ReservationStation pressure, LoopMatmul's rob_overloaded)
  t["loop_ws_slow"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    tiled_ws(s, rng, 3, false);
    for (unsigned i = 0; i < 3; ++i) {
      s.ready_pct[i] = 33;
      s.delay_min[i] = 10;
    }
    s.delay_max[0] = 120;
    s.delay_max[1] = 160;
    s.delay_max[2] = 120;
    return s;
  };
  t["explicit"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    explicit_cmds(s, rng, 200, cc);
    return s;
  };
  t["explicit_slow"] = [](std::mt19937 &rng) {
    CmdStimulus s;
    explicit_cmds(s, rng, 200, cc);
    const unsigned pct[3] = {40, 60, 30};
    for (unsigned i = 0; i < 3; ++i) {
      s.ready_pct[i] = pct[i];
      s.delay_min[i] = 5;
    }
    s.delay_max[0] = 80;
    s.delay_max[1] = 100;
    s.delay_max[2] = 80;
    return s;
  };
  return t;
}

CmdStimulus cmd_random(std::mt19937 &rng) {
  CmdStimulus s;
  const unsigned parts = 1 + rng() % 3;
  for (unsigned p = 0; p < parts; ++p) {
    if (rng() % 2)
      tiled_ws(s, rng, 1 + rng() % 3, true);
    else
      explicit_cmds(s, rng, 20 + rng() % 60, cc);
  }
  for (unsigned i = 0; i < 3; ++i) {
    s.ready_pct[i] = rng() % 2 ? 100 : 20 + rng() % 80;
    s.delay_min[i] = 1 + rng() % 4;
    s.delay_max[i] = s.delay_min[i] + rng() % 100;
  }
  return s;
}

// ---------------------------------------------------------------- CtrlTop
namespace {
CtrlScenario make(const EngineOptions &opt, DmaParams dma) {
  EngineOptions o = opt;
  o.dma = dma;
  CtrlScenario s;
  s.engine = std::make_unique<Engine>(o);
  return s;
}

Matmul mm(unsigned M, unsigned K, unsigned N, unsigned df, uint32_t seed, bool bias = false, bool full_c = false,
          unsigned act = ACT_NONE) {
  Matmul m;
  m.M = M;
  m.K = K;
  m.N = N;
  m.dataflow = df;
  m.seed = seed;
  m.bias = bias;
  m.full_c = full_c;
  m.act = act;
  return m;
}

Command plain(const Command &c) { return c; }
struct CmdList {
  struct Item : Command {
    unsigned gap = 0;
    Item(const Command &c) : Command(c) {}
  };
  std::vector<Item> cmds;
  std::vector<Command> list() const { return std::vector<Command>(cmds.begin(), cmds.end()); }
};
}  // namespace

std::map<std::string, std::function<CtrlScenario(std::mt19937 &, const EngineOptions &)>> ctrl_catalog() {
  std::map<std::string, std::function<CtrlScenario(std::mt19937 &, const EngineOptions &)>> t;
  t["matmul_ws_64"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    s.engine->submit(mm(64, 64, 64, kWS, rng()));
    return s;
  };
  t["matmul_os_64"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    s.engine->submit(mm(64, 64, 64, kOS, rng()));
    return s;
  };
  t["matmul_ws_bias_relu"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    s.engine->submit(mm(48, 40, 56, kWS, rng(), true, false, ACT_RELU));
    return s;
  };
  t["matmul_os_bias_full"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    s.engine->submit(mm(40, 72, 24, kOS, rng(), true, true));
    return s;
  };
  t["two_matmuls"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    s.engine->submit(mm(32, 32, 32, kWS, rng()));
    s.engine->submit(mm(32, 48, 16, kOS, rng(), true));
    return s;
  };
  t["fast_dma"] = [](std::mt19937 &rng, const EngineOptions &o) {
    DmaParams d;
    d.latency = 4;
    d.bytes_per_cycle = 64;
    d.max_cmds = 4;
    CtrlScenario s = make(o, d);
    s.engine->submit(mm(64, 64, 64, kWS, rng(), false, true));
    return s;
  };
  t["explicit"] = [](std::mt19937 &rng, const EngineOptions &o) {
    CtrlScenario s = make(o, DmaParams());
    CmdList l;
    explicit_cmds(l, rng, 150, plain);
    s.engine->submit(l.list(), "explicit commands");
    return s;
  };
  return t;
}

CtrlScenario ctrl_random(std::mt19937 &rng, const EngineOptions &o) {
  DmaParams d;
  d.latency = 2 + rng() % 80;
  d.bytes_per_cycle = 4u << (rng() % 5);  // 4 .. 64
  d.max_cmds = 1 + rng() % 4;
  CtrlScenario s = make(o, d);
  const unsigned parts = 1 + rng() % 3;
  for (unsigned p = 0; p < parts; ++p) {
    if (rng() % 4 == 0) {
      CmdList l;
      explicit_cmds(l, rng, 20 + rng() % 60, plain);
      s.engine->submit(l.list(), "explicit commands");
    } else {
      const unsigned M = 8 + rng() % 73, K = 8 + rng() % 73, N = 8 + rng() % 73;
      const unsigned df = rng() % 2 ? kWS : kOS;
      const bool bias = rng() % 2, full_c = rng() % 3 == 0;
      const unsigned act = rng() % 3 == 0 ? ACT_RELU : ACT_NONE;
      s.engine->submit(mm(M, K, N, df, rng(), bias, full_c, act));
    }
  }
  return s;
}

namespace {
bool random_seed(const std::string &name, unsigned &seed) {
  if (name.rfind("random_", 0) != 0)
    return false;
  seed = unsigned(std::stoul(name.substr(7)));
  return true;
}
}  // namespace

bool ex_stimulus(const std::string &name, ExStimulus &s) {
  unsigned seed;
  if (random_seed(name, seed)) {
    std::mt19937 rng(seed);
    s = ex_random(rng);
    return true;
  }
  const auto cat = ex_catalog();
  const auto it = cat.find(name);
  if (it == cat.end())
    return false;
  std::mt19937 rng(name_seed(name));
  s = it->second(rng);
  return true;
}

bool cmd_stimulus(const std::string &name, CmdStimulus &s) {
  unsigned seed;
  if (random_seed(name, seed)) {
    std::mt19937 rng(seed);
    s = cmd_random(rng);
    s.rng.seed(seed * 7919u);
    return true;
  }
  const auto cat = cmd_catalog();
  const auto it = cat.find(name);
  if (it == cat.end())
    return false;
  std::mt19937 rng(name_seed(name));
  s = it->second(rng);
  s.rng.seed(name_seed(name) + 1);
  return true;
}

bool ctrl_scenario(const std::string &name, const EngineOptions &opt, CtrlScenario &s) {
  unsigned seed;
  if (random_seed(name, seed)) {
    std::mt19937 rng(seed);
    s = ctrl_random(rng, opt);
    return true;
  }
  const auto cat = ctrl_catalog();
  const auto it = cat.find(name);
  if (it == cat.end())
    return false;
  std::mt19937 rng(name_seed(name));
  s = it->second(rng, opt);
  return true;
}

}  // namespace systolique
