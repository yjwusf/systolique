// Stimuli of the frontend RTL tops (rtl/ex/src/), shared by the live lockstep bench (rtl/fe_bench.cpp)
// and the offline tests that regenerate the stored traces from them. Each stimulus is a driver:
// per cycle it chooses the top's inputs from its own state and the top's outputs that depend on
// registers only, and after the cycle it sees the handshakes (observe). Run against the RTL or
// against the model it produces the same inputs as long as both give the same outputs.
//
//   ExecuteTop  a stream of execute commands (as the ReservationStation issues them: config,
//               preload, compute) with gaps, and mvin row writes on the DMA ports competing for
//               the banks: 8 directed tests and random seeds
//   CmdTop      the core's commands (gemmini.h's loop_ws blocks, explicit mvin / preload /
//               compute / mvout / config / flush streams, fences) and the three controllers
//               (random acceptance, completions after random delays): 6 directed, random seeds
//   CtrlTop     an Engine (engine.h): its Host plays the core and the load / store side;
//               matmuls (WS through loop_ws, OS through explicit commands), explicit streams
#pragma once

#include "fe_ports.h"

#include "systolique/engine.h"

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace systolique {

uint32_t name_seed(const std::string &name);  // FNV-1a of the name, 16 bits

// ---------------------------------------------------------------- ExecuteTop
struct ExStimulus {
  struct Cmd {
    unsigned funct = 0;
    uint64_t rs1 = 0, rs2 = 0;
    unsigned gap = 0;  // cycles to hold it back after the previous command fired
  };
  std::deque<Cmd> cmds;
  std::deque<DmaSpWrite> sp;
  std::deque<DmaAccWrite> acc;
  int64_t cmd_start = 0;  // no command before this cycle
  // driver state
  unsigned rob = 0;
  int64_t last_fire = 0;
  unsigned idle = 0;

  ExecuteTopIn next(int64_t cycle) const;
  void observe(const ExecuteTopIn &in, const ExecuteTopOut &out);
  bool finished() const { return idle > 64; }  // 64 quiet cycles after the last command
};
std::map<std::string, std::function<ExStimulus(std::mt19937 &)>> ex_catalog();
ExStimulus ex_random(std::mt19937 &rng);

// ---------------------------------------------------------------- CmdTop
struct CmdStimulus {
  struct Cmd {
    unsigned funct = 0;  // kFence: the core's fence
    uint64_t rs1 = 0, rs2 = 0;
    unsigned gap = 0;
  };
  std::deque<Cmd> cmds;
  unsigned ready_pct[3] = {100, 100, 100};  // chance that ld / ex / st accept an issue
  unsigned delay_min[3] = {1, 1, 1}, delay_max[3] = {30, 40, 30};
  // driver state
  std::mt19937 rng;
  struct Pending {
    int64_t due;
    unsigned id;
  };
  std::vector<Pending> pending;
  int pick = -1;  // the pending completion offered this cycle
  int64_t last_sent = 0;
  unsigned idle = 0;

  CommandPath::In next(int64_t cycle, const CommandPath::Out &regs);
  void observe(const CommandPath::In &in, const CommandPath::Out &out, int64_t cycle);
  bool finished() const { return idle > 32; }
};
std::map<std::string, std::function<CmdStimulus(std::mt19937 &)>> cmd_catalog();
CmdStimulus cmd_random(std::mt19937 &rng);

// ---------------------------------------------------------------- CtrlTop
// An Engine with its operations submitted; the run ends 16 cycles after Engine::done().
struct CtrlScenario {
  std::unique_ptr<Engine> engine;
  unsigned tail = 16;
};
std::map<std::string, std::function<CtrlScenario(std::mt19937 &, const EngineOptions &)>> ctrl_catalog();
CtrlScenario ctrl_random(std::mt19937 &rng, const EngineOptions &opt);

// The stimulus of a stored trace: "<test>" of the catalog or "random_<seed>".
bool ex_stimulus(const std::string &name, ExStimulus &s);
bool cmd_stimulus(const std::string &name, CmdStimulus &s);
bool ctrl_scenario(const std::string &name, const EngineOptions &opt, CtrlScenario &s);

}  // namespace systolique
