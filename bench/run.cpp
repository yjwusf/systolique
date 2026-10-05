#include "run.h"

#include <algorithm>

namespace systolique {

namespace {

void clear_inputs(const std::vector<PortSpec> &spec, Frame &f) {
  for (size_t p = 0; p < spec.size(); ++p)
    if (spec[p].input) std::fill(f[p].w.begin(), f[p].w.end(), 0);
}

std::string at_cycle(size_t cycle, const std::string &what) {
  return "cycle " + std::to_string(cycle) + ": " + what;
}

}  // namespace

void outputs_to_frame(const SystolicArray &a, Top top, Frame &f) {
  if (top == Top::Mesh)
    mesh_out_to_frame(a.config(), a.mesh_out(), f);
  else
    mwd_out_to_frame(a.config(), a.out(), f);
}

void frame_to_inputs(SystolicArray &a, Top top, const Frame &f, const RequestNote *note) {
  if (top == Top::Mesh) {
    MeshIn in;
    frame_to_mesh_in(a.config(), f, in);
    a.set_mesh_inputs(in);
  } else {
    MwdIn in;
    frame_to_mwd_in(a.config(), f, in);
    a.set_inputs(in, note);
  }
}

ArrayRun replay_trace(SystolicArray &array, Top top, const std::vector<Frame> &trace,
                      const CycleHook &hook) {
  const auto spec = port_specs(array.config(), top);
  ArrayRun res;
  array.reset(0);
  Frame mine = make_frame(spec);
  for (size_t cycle = 0; cycle < trace.size(); ++cycle) {
    mine = trace[cycle];
    outputs_to_frame(array, top, mine);
    ++res.compared;
    const std::string d = diff_outputs(spec, trace[cycle], mine);
    if (!d.empty()) {
      res.ok = false;
      res.error = at_cycle(cycle, d);
      return res;
    }
    if (hook && !trace[cycle][0].get(0, 1)) hook(array);
    frame_to_inputs(array, top, trace[cycle]);
    array.tick();
    res.rows.push_back(mine);
  }
  return res;
}

ArrayRun run_stimulus(SystolicArray &array, Top top, Stimulus &stim, unsigned reset_cycles,
                      unsigned max_cycles, const CycleHook &hook,
                      const std::function<const RequestNote *(const MwdIn &)> &note) {
  const auto spec = port_specs(array.config(), top);
  ArrayRun res;
  array.reset(0);
  Frame f = make_frame(spec);
  unsigned cycle = 0;
  for (; cycle < max_cycles; ++cycle) {
    outputs_to_frame(array, top, f);  // this cycle's outputs decide the stimulus
    clear_inputs(spec, f);
    if (cycle < reset_cycles) {
      f[0].set(0, 1, 1);
    } else if (!stim.next(cycle - reset_cycles, f, f)) {
      break;
    }
    const RequestNote *n = nullptr;
    if (cycle >= reset_cycles) {
      if (hook) hook(array);
      if (note && top == Top::MeshWithDelays) {
        MwdIn in;
        frame_to_mwd_in(array.config(), f, in);
        n = note(in);
      }
    }
    frame_to_inputs(array, top, f, n);
    array.tick();
    res.rows.push_back(f);
  }
  if (cycle >= max_cycles) {
    res.ok = false;
    res.error = "no end after " + std::to_string(max_cycles) + " cycles";
  }
  if (res.ok && !stim.error().empty()) {
    res.ok = false;
    res.error = stim.error();
  }
  return res;
}

}  // namespace systolique
