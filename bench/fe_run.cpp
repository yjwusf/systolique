#include "fe_run.h"

#include <memory>

namespace systolique {

std::string fe_diff(const std::vector<PortSpec> &spec, const Frame &ref, const Frame &dut, uint64_t &values) {
  std::string d;
  unsigned items = 0;
  const std::vector<FeQualifier> q = fe_qualifiers(spec);
  for (size_t p = 0; p < spec.size(); ++p) {
    if (spec[p].input)
      continue;
    const unsigned w = spec[p].lane_bits;
    for (unsigned l = 0; l < spec[p].lanes; ++l) {
      if (q[p].port >= 0 && !ref[size_t(q[p].port)].get(l / q[p].div, 1) && !dut[size_t(q[p].port)].get(l / q[p].div, 1))
        continue;
      ++values;
      bool same = true;
      for (unsigned b = 0; b < w; b += 64) {
        const unsigned n = std::min(64u, w - b);
        if (ref[p].get(l * w + b, n) != dut[p].get(l * w + b, n))
          same = false;
      }
      if (same || items >= 8)
        continue;
      ++items;
      char buf[200];
      std::snprintf(buf, sizeof(buf), "%s%s[%u]: rtl=0x%llx model=0x%llx", d.empty() ? "" : ", ",
                    spec[p].name.c_str(), l, (unsigned long long)ref[p].get(l * w, std::min(64u, w)),
                    (unsigned long long)dut[p].get(l * w, std::min(64u, w)));
      d += buf;
    }
  }
  return d;
}

FeRun run_ex(const FrontendConfig &cfg, ExStimulus s, unsigned max_cycles) {
  FeRun r;
  const auto spec = fe_port_specs(cfg, FeTop::Execute);
  ExecuteUnit m(cfg);
  for (unsigned cycle = 0; cycle < max_cycles; ++cycle) {
    const ExecuteTopIn in = s.next(cycle);
    m.set_inputs(in);
    m.eval();
    Frame f = make_frame(spec);
    ex_in_to_frame(cfg, in, f);
    ex_out_to_frame(cfg, m.out(), f);
    r.rows.push_back(f);
    s.observe(in, m.out());
    m.tick();
    if (s.finished())
      return r;
  }
  r.ok = false;
  r.error = "no end after " + std::to_string(max_cycles) + " cycles";
  return r;
}

FeRun run_cmd(const FrontendConfig &cfg, CmdStimulus s, unsigned max_cycles) {
  FeRun r;
  const auto spec = fe_port_specs(cfg, FeTop::Cmd);
  CommandPath m(cfg);
  for (unsigned cycle = 0; cycle < max_cycles; ++cycle) {
    const CommandPath::In in = s.next(cycle, m.out());
    m.set_inputs(in);
    m.eval();
    Frame f = make_frame(spec);
    cmd_in_to_frame(cfg, in, f);
    cmd_out_to_frame(cfg, m.out(), f);
    r.rows.push_back(f);
    s.observe(in, m.out(), cycle);
    m.tick();
    if (s.finished())
      return r;
  }
  r.ok = false;
  r.error = "no end after " + std::to_string(max_cycles) + " cycles";
  return r;
}

FeRun run_ctrl(const FrontendConfig &cfg, CtrlScenario &s, unsigned max_cycles) {
  FeRun r;
  const auto spec = fe_port_specs(cfg, FeTop::Ctrl);
  Engine &e = *s.engine;
  unsigned tail = 0;
  for (unsigned cycle = 0; cycle < max_cycles; ++cycle) {
    const CtrlTopIn in = e.host_inputs();
    const CtrlTopOut o = e.step(in);
    Frame f = make_frame(spec);
    ctrl_in_to_frame(cfg, in, f);
    ctrl_out_to_frame(cfg, o, f);
    r.rows.push_back(f);
    if (!e.host().error().empty()) {
      r.ok = false;
      r.error = e.host().error();
      return r;
    }
    tail = e.done() ? tail + 1 : 0;
    if (tail > s.tail) {
      e.finalize();
      return r;
    }
  }
  r.ok = false;
  r.error = "no end after " + std::to_string(max_cycles) + " cycles";
  return r;
}

FeRun replay_fe(const FrontendConfig &cfg, FeTop top, const std::vector<Frame> &trace) {
  FeRun r;
  const auto spec = fe_port_specs(cfg, top);
  std::unique_ptr<ExecuteUnit> ex;
  std::unique_ptr<CommandPath> cp;
  std::unique_ptr<Controller> ct;
  if (top == FeTop::Execute)
    ex = std::make_unique<ExecuteUnit>(cfg);
  else if (top == FeTop::Cmd)
    cp = std::make_unique<CommandPath>(cfg);
  else
    ct = std::make_unique<Controller>(cfg);
  for (size_t cycle = 0; cycle < trace.size(); ++cycle) {
    Frame mine = trace[cycle];
    if (ex) {
      ExecuteTopIn in;
      frame_to_ex_in(cfg, trace[cycle], in);
      ex->set_inputs(in);
      ex->eval();
      ex_out_to_frame(cfg, ex->out(), mine);
    } else if (cp) {
      CommandPath::In in;
      frame_to_cmd_in(cfg, trace[cycle], in);
      cp->set_inputs(in);
      cp->eval();
      cmd_out_to_frame(cfg, cp->out(), mine);
    } else {
      CtrlTopIn in;
      frame_to_ctrl_in(cfg, trace[cycle], in);
      ct->set_inputs(in);
      ct->eval();
      ctrl_out_to_frame(cfg, ct->out(), mine);
    }
    ++r.compared;
    const std::string d = fe_diff(spec, trace[cycle], mine, r.values);
    if (!d.empty()) {
      r.ok = false;
      r.error = "cycle " + std::to_string(cycle) + ": " + d;
      return r;
    }
    r.rows.push_back(mine);
    if (ex)
      ex->tick();
    else if (cp)
      cp->tick();
    else
      ct->tick();
  }
  return r;
}

}  // namespace systolique
