// The frontend classes driven through the port frames of the RTL tops (fe_ports.h): running a
// stimulus (fe_stimulus.h) on the model, as the RTL lockstep bench runs it on the Verilated RTL,
// and replaying a stored RTL trace with every output port compared in every cycle.
#pragma once

#include "fe_ports.h"
#include "fe_stimulus.h"

#include <string>
#include <vector>

namespace systolique {

struct FeRun {
  bool ok = true;
  std::string error;        // first mismatch / timeout
  std::vector<Frame> rows;  // per cycle from cycle 0: inputs and the model's outputs
  uint64_t compared = 0;    // cycles compared (replay)
  uint64_t values = 0;      // port lanes compared (replay)
};

// A stimulus on the model; rows hold every cycle's inputs and outputs.
FeRun run_ex(const FrontendConfig &cfg, ExStimulus s, unsigned max_cycles = 200000);
FeRun run_cmd(const FrontendConfig &cfg, CmdStimulus s, unsigned max_cycles = 400000);
FeRun run_ctrl(const FrontendConfig &cfg, CtrlScenario &s, unsigned max_cycles = 2000000);

// A stored trace on the model: the recorded inputs cycle by cycle, every output port equal to
// the recorded one in every cycle. `values` counts the compared port lanes.
FeRun replay_fe(const FrontendConfig &cfg, FeTop top, const std::vector<Frame> &trace);

// Lanes of the output ports that differ ("port[lane]: rtl=.. model=.."), "" if none; counts the
// lanes compared into `values`.
std::string fe_diff(const std::vector<PortSpec> &spec, const Frame &ref, const Frame &dut, uint64_t &values);

}  // namespace systolique
