// SystolicArray driven through the RTL tops' port frames (ports.h): replaying a stored RTL trace
// and running a Stimulus (stimulus.h) the way the RTL lockstep bench runs the Verilated RTL, so
// the same traces and stimuli check both.
#pragma once

#include "ports.h"
#include "stimulus.h"

#include "systolique/systolic_array.h"

#include <functional>
#include <string>
#include <vector>

namespace systolique {

struct ArrayRun {
  bool ok = true;
  std::string error;          // first mismatch / stimulus error
  std::vector<Frame> rows;    // per cycle from power-on: inputs and the array's outputs
  unsigned compared = 0;      // cycles whose outputs were compared (replay)
};

// Called before every cycle with reset low, with the array in that cycle's state.
using CycleHook = std::function<void(const SystolicArray &)>;

// The array's outputs of the current cycle into the output ports of f.
void outputs_to_frame(const SystolicArray &a, Top top, Frame &f);
// Applies the inputs of f in the current cycle (set_inputs / set_mesh_inputs, not the edge).
void frame_to_inputs(SystolicArray &a, Top top, const Frame &f, const RequestNote *note = nullptr);

// Power-on, then the recorded inputs of `trace` cycle by cycle; every output port must equal
// the recorded one in every cycle.
ArrayRun replay_trace(SystolicArray &array, Top top, const std::vector<Frame> &trace,
                      const CycleHook &hook = {});

// Power-on, `reset_cycles` with reset high, then `stim` until it ends.
// `note(in)` may annotate the request offered in a cycle.
ArrayRun run_stimulus(SystolicArray &array, Top top, Stimulus &stim, unsigned reset_cycles = 4,
                      unsigned max_cycles = 1000000, const CycleHook &hook = {},
                      const std::function<const RequestNote *(const MwdIn &)> &note = {});

}  // namespace systolique
