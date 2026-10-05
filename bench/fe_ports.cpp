#include "fe_ports.h"

#include <functional>
#include <stdexcept>

namespace systolique {

namespace {

// One port: its spec and how it maps to a field of a struct S.
template <class S>
struct Binding {
  PortSpec spec;
  std::function<void(const S &, Bits &)> put;  // struct -> bits
  std::function<void(const Bits &, S &)> get;  // bits -> struct
};

template <class S>
using Bindings = std::vector<Binding<S>>;

template <class S, class F>
void scalar(Bindings<S> &b, const std::string &name, bool input, unsigned width, F field) {
  b.push_back({{name, input, 1, width, false},
               [field](const S &s, Bits &x) { x.set(0, x.width, uint64_t(field(const_cast<S &>(s)))); },
               [field](const Bits &x, S &s) {
                 auto &f = field(s);
                 f = static_cast<std::remove_reference_t<decltype(f)>>(x.get(0, x.width));
               }});
}

// lanes x width, lane k from get_lane(s, k) / into set_lane(s, k, v)
template <class S>
void vec(Bindings<S> &b, const std::string &name, bool input, unsigned lanes, unsigned width, bool is_signed,
         std::function<uint64_t(const S &, unsigned)> get_lane, std::function<void(S &, unsigned, uint64_t)> set_lane) {
  b.push_back({{name, input, lanes, width, is_signed},
               [=](const S &s, Bits &x) {
                 for (unsigned k = 0; k < lanes; ++k)
                   x.set(k * width, width, get_lane(s, k));
               },
               [=](const Bits &x, S &s) {
                 for (unsigned k = 0; k < lanes; ++k)
                   set_lane(s, k, x.get(k * width, width));
               }});
}


struct Widths {
  unsigned dim, sp_banks, acc_banks, sp_row, acc_row, rob;
  explicit Widths(const FrontendConfig &c)
      : dim(c.dim()), sp_banks(c.sp_banks), acc_banks(c.acc_banks), sp_row(c.addr_map().sp_bank_row_bits),
        acc_row(c.addr_map().acc_bank_row_bits), rob(2 + c.rob_type_bits()) {}
};

// ---- the DMA inputs, shared by ExecuteTop and CtrlTop
template <class S>
void dma_inputs(Bindings<S> &b, const Widths &w, std::function<DmaSpWrite &(S &)> sp,
                std::function<DmaAccWrite &(S &)> acc) {
  scalar(b, "dma_sp_en", true, 1, [sp](S &s) -> bool & { return sp(s).valid; });
  scalar(b, "dma_sp_bank", true, log2_up(w.sp_banks), [sp](S &s) -> unsigned & { return sp(s).bank; });
  scalar(b, "dma_sp_addr", true, w.sp_row, [sp](S &s) -> uint32_t & { return sp(s).addr; });
  vec<S>(b, "dma_sp_data", true, w.dim, 8, true,
         [sp](const S &s, unsigned k) { return uint64_t(uint8_t(sp(const_cast<S &>(s)).data[k])); },
         [sp](S &s, unsigned k, uint64_t v) { sp(s).data[k] = int8_t(v); });
  scalar(b, "dma_sp_mask", true, w.dim, [sp](S &s) -> uint64_t & { return sp(s).mask; });
  scalar(b, "dma_acc_en", true, 1, [acc](S &s) -> bool & { return acc(s).valid; });
  scalar(b, "dma_acc_bank", true, log2_up(w.acc_banks), [acc](S &s) -> unsigned & { return acc(s).bank; });
  scalar(b, "dma_acc_addr", true, w.acc_row, [acc](S &s) -> uint32_t & { return acc(s).addr; });
  vec<S>(b, "dma_acc_data", true, w.dim, 32, true,
         [acc](const S &s, unsigned k) { return uint64_t(uint32_t(acc(const_cast<S &>(s)).data[k])); },
         [acc](S &s, unsigned k, uint64_t v) { acc(s).data[k] = int32_t(uint32_t(v)); });
  scalar(b, "dma_acc_mask", true, 4 * w.dim, [acc](S &s) -> uint64_t & { return acc(s).mask; });
  scalar(b, "dma_acc_acc", true, 1, [acc](S &s) -> bool & { return acc(s).acc; });
}

// ---- the ExecuteController's bank ports and the DMA's taken bits (outputs)
template <class S>
void bank_outputs(Bindings<S> &b, const Widths &w, std::function<ExecuteTopOut &(S &)> o) {
  const unsigned D = w.dim, SB = w.sp_banks, AB = w.acc_banks;
  auto sized = [=](S &s) -> ExecuteTopOut & {
    ExecuteTopOut &x = o(s);
    if (x.sp_read.size() != SB) {
      x.sp_read.assign(SB, SpReadReq());
      x.sp_read_ready.assign(SB, false);
      x.sp_resp_ready.assign(SB, false);
      x.sp_write.assign(SB, SpWrite());
    }
    if (x.acc_read.size() != AB) {
      x.acc_read.assign(AB, AccReadReq());
      x.acc_read_ready.assign(AB, false);
      x.acc_write.assign(AB, AccWrite());
    }
    return x;
  };
  auto cs = [sized](const S &s) -> ExecuteTopOut & { return sized(const_cast<S &>(s)); };
  scalar(b, "dma_sp_taken", false, 1, [o](S &s) -> bool & { return o(s).dma_sp_taken; });
  scalar(b, "dma_acc_taken", false, 1, [o](S &s) -> bool & { return o(s).dma_acc_taken; });
  vec<S>(b, "sp_read_valid", false, SB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_read[k].valid); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_read[k].valid = v; });
  vec<S>(b, "sp_read_ready", false, SB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_read_ready[k]); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_read_ready[k] = v; });
  vec<S>(b, "sp_read_addr", false, SB, w.sp_row, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_read[k].addr); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_read[k].addr = uint32_t(v); });
  vec<S>(b, "sp_resp_ready", false, SB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_resp_ready[k]); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_resp_ready[k] = v; });
  vec<S>(b, "sp_write_en", false, SB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_write[k].en); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_write[k].en = v; });
  vec<S>(b, "sp_write_addr", false, SB, w.sp_row, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).sp_write[k].addr); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_write[k].addr = uint32_t(v); });
  vec<S>(b, "sp_write_data", false, SB * D, 8, true,
         [cs, D](const S &s, unsigned k) { return uint64_t(uint8_t(cs(s).sp_write[k / D].data[k % D])); },
         [sized, D](S &s, unsigned k, uint64_t v) { sized(s).sp_write[k / D].data[k % D] = int8_t(v); });
  vec<S>(b, "sp_write_mask", false, SB, D, false, [cs](const S &s, unsigned k) { return cs(s).sp_write[k].mask; },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).sp_write[k].mask = v; });
  vec<S>(b, "acc_read_valid", false, AB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_read[k].valid); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_read[k].valid = v; });
  vec<S>(b, "acc_read_ready", false, AB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_read_ready[k]); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_read_ready[k] = v; });
  vec<S>(b, "acc_read_addr", false, AB, w.acc_row, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_read[k].addr); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_read[k].addr = uint32_t(v); });
  vec<S>(b, "acc_write_valid", false, AB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_write[k].valid); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_write[k].valid = v; });
  vec<S>(b, "acc_write_addr", false, AB, w.acc_row, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_write[k].addr); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_write[k].addr = uint32_t(v); });
  vec<S>(b, "acc_write_data", false, AB * D, 32, true,
         [cs, D](const S &s, unsigned k) { return uint64_t(uint32_t(cs(s).acc_write[k / D].data[k % D])); },
         [sized, D](S &s, unsigned k, uint64_t v) { sized(s).acc_write[k / D].data[k % D] = int32_t(uint32_t(v)); });
  vec<S>(b, "acc_write_acc", false, AB, 1, false, [cs](const S &s, unsigned k) { return uint64_t(cs(s).acc_write[k].acc); },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_write[k].acc = v; });
  vec<S>(b, "acc_write_mask", false, AB, 4 * D, false, [cs](const S &s, unsigned k) { return cs(s).acc_write[k].mask; },
         [sized](S &s, unsigned k, uint64_t v) { sized(s).acc_write[k].mask = v; });
}

// One issue port (ld / ex / st) as outputs.
template <class S>
void issue_outputs(Bindings<S> &b, const Widths &w, const std::string &p,
                   std::function<CommandPath::IssuePort &(S &)> port) {
  scalar(b, p + "_valid", false, 1, [port](S &s) -> bool & { return port(s).valid; });
  scalar(b, p + "_funct", false, 7, [port](S &s) -> unsigned & { return port(s).funct; });
  scalar(b, p + "_rs1", false, 64, [port](S &s) -> uint64_t & { return port(s).rs1; });
  scalar(b, p + "_rs2", false, 64, [port](S &s) -> uint64_t & { return port(s).rs2; });
  scalar(b, p + "_rob_id", false, w.rob, [port](S &s) -> unsigned & { return port(s).rob_id; });
}

// ---- ExecuteTop
Bindings<ExecuteTopIn> ex_in(const Widths &w) {
  Bindings<ExecuteTopIn> b;
  scalar(b, "cmd_valid", true, 1, [](ExecuteTopIn &s) -> bool & { return s.cmd_valid; });
  scalar(b, "cmd_funct", true, 7, [](ExecuteTopIn &s) -> unsigned & { return s.cmd_funct; });
  scalar(b, "cmd_rs1", true, 64, [](ExecuteTopIn &s) -> uint64_t & { return s.cmd_rs1; });
  scalar(b, "cmd_rs2", true, 64, [](ExecuteTopIn &s) -> uint64_t & { return s.cmd_rs2; });
  scalar(b, "cmd_rob_id", true, w.rob, [](ExecuteTopIn &s) -> unsigned & { return s.cmd_rob_id; });
  dma_inputs<ExecuteTopIn>(b, w, [](ExecuteTopIn &s) -> DmaSpWrite & { return s.dma_sp; },
                           [](ExecuteTopIn &s) -> DmaAccWrite & { return s.dma_acc; });
  return b;
}
Bindings<ExecuteTopOut> ex_out(const Widths &w) {
  Bindings<ExecuteTopOut> b;
  scalar(b, "cmd_ready", false, 1, [](ExecuteTopOut &s) -> bool & { return s.cmd_ready; });
  scalar(b, "completed_valid", false, 1, [](ExecuteTopOut &s) -> bool & { return s.completed_valid; });
  scalar(b, "completed_bits", false, w.rob, [](ExecuteTopOut &s) -> unsigned & { return s.completed_bits; });
  scalar(b, "busy", false, 1, [](ExecuteTopOut &s) -> bool & { return s.busy; });
  bank_outputs<ExecuteTopOut>(b, w, [](ExecuteTopOut &s) -> ExecuteTopOut & { return s; });
  return b;
}

// ---- CmdTop
Bindings<CommandPath::In> cmd_in(const Widths &w) {
  using S = CommandPath::In;
  Bindings<S> b;
  scalar(b, "cmd_valid", true, 1, [](S &s) -> bool & { return s.cmd_valid; });
  scalar(b, "cmd_funct", true, 7, [](S &s) -> unsigned & { return s.cmd.funct; });
  scalar(b, "cmd_rs1", true, 64, [](S &s) -> uint64_t & { return s.cmd.rs1; });
  scalar(b, "cmd_rs2", true, 64, [](S &s) -> uint64_t & { return s.cmd.rs2; });
  scalar(b, "ld_ready", true, 1, [](S &s) -> bool & { return s.issue_ready[0]; });
  scalar(b, "ex_ready", true, 1, [](S &s) -> bool & { return s.issue_ready[1]; });
  scalar(b, "st_ready", true, 1, [](S &s) -> bool & { return s.issue_ready[2]; });
  scalar(b, "completed_valid", true, 1, [](S &s) -> bool & { return s.completed_valid; });
  scalar(b, "completed_bits", true, w.rob, [](S &s) -> unsigned & { return s.completed_id; });
  return b;
}
Bindings<CommandPath::Out> cmd_out(const Widths &w) {
  using S = CommandPath::Out;
  Bindings<S> b;
  scalar(b, "cmd_ready", false, 1, [](S &s) -> bool & { return s.cmd_ready; });
  scalar(b, "busy", false, 1, [](S &s) -> bool & { return s.busy; });
  scalar(b, "loop_matmul_busy", false, 1, [](S &s) -> bool & { return s.loop_matmul_busy; });
  issue_outputs<S>(b, w, "ld", [](S &s) -> CommandPath::IssuePort & { return s.issue[0]; });
  issue_outputs<S>(b, w, "ex", [](S &s) -> CommandPath::IssuePort & { return s.issue[1]; });
  issue_outputs<S>(b, w, "st", [](S &s) -> CommandPath::IssuePort & { return s.issue[2]; });
  scalar(b, "matmul_ld_completed", false, 2, [](S &s) -> unsigned & { return s.matmul_completed[0]; });
  scalar(b, "matmul_ex_completed", false, 2, [](S &s) -> unsigned & { return s.matmul_completed[1]; });
  scalar(b, "matmul_st_completed", false, 2, [](S &s) -> unsigned & { return s.matmul_completed[2]; });
  return b;
}

// ---- CtrlTop
Bindings<CtrlTopIn> ctrl_in(const Widths &w) {
  using S = CtrlTopIn;
  Bindings<S> b;
  scalar(b, "cmd_valid", true, 1, [](S &s) -> bool & { return s.cmd_valid; });
  scalar(b, "cmd_funct", true, 7, [](S &s) -> unsigned & { return s.cmd.funct; });
  scalar(b, "cmd_rs1", true, 64, [](S &s) -> uint64_t & { return s.cmd.rs1; });
  scalar(b, "cmd_rs2", true, 64, [](S &s) -> uint64_t & { return s.cmd.rs2; });
  scalar(b, "ld_ready", true, 1, [](S &s) -> bool & { return s.ld_ready; });
  scalar(b, "st_ready", true, 1, [](S &s) -> bool & { return s.st_ready; });
  scalar(b, "ld_completed_valid", true, 1, [](S &s) -> bool & { return s.ld_completed_valid; });
  scalar(b, "ld_completed_bits", true, w.rob, [](S &s) -> unsigned & { return s.ld_completed_bits; });
  scalar(b, "st_completed_valid", true, 1, [](S &s) -> bool & { return s.st_completed_valid; });
  scalar(b, "st_completed_bits", true, w.rob, [](S &s) -> unsigned & { return s.st_completed_bits; });
  dma_inputs<S>(b, w, [](S &s) -> DmaSpWrite & { return s.dma_sp; }, [](S &s) -> DmaAccWrite & { return s.dma_acc; });
  return b;
}
Bindings<CtrlTopOut> ctrl_out(const Widths &w) {
  using S = CtrlTopOut;
  Bindings<S> b;
  scalar(b, "cmd_ready", false, 1, [](S &s) -> bool & { return s.cmd_ready; });
  scalar(b, "busy", false, 1, [](S &s) -> bool & { return s.busy; });
  scalar(b, "loop_matmul_busy", false, 1, [](S &s) -> bool & { return s.loop_matmul_busy; });
  issue_outputs<S>(b, w, "ld", [](S &s) -> CommandPath::IssuePort & { return s.ld; });
  issue_outputs<S>(b, w, "st", [](S &s) -> CommandPath::IssuePort & { return s.st; });
  scalar(b, "ld_completed_ready", false, 1, [](S &s) -> bool & { return s.ld_completed_ready; });
  scalar(b, "st_completed_ready", false, 1, [](S &s) -> bool & { return s.st_completed_ready; });
  issue_outputs<S>(b, w, "ex", [](S &s) -> CommandPath::IssuePort & { return s.ex; });
  scalar(b, "ex_ready", false, 1, [](S &s) -> bool & { return s.ex_ready; });
  scalar(b, "ex_completed_valid", false, 1, [](S &s) -> bool & { return s.ex_completed_valid; });
  scalar(b, "ex_completed_bits", false, w.rob, [](S &s) -> unsigned & { return s.ex_completed_bits; });
  scalar(b, "ex_busy", false, 1, [](S &s) -> bool & { return s.ex_busy; });
  bank_outputs<S>(b, w, [](S &s) -> ExecuteTopOut & { return s.bank; });
  return b;
}

template <class S>
void to_frame(const Bindings<S> &b, size_t offset, const S &s, Frame &f) {
  for (size_t i = 0; i < b.size(); ++i)
    b[i].put(s, f[offset + i]);
}
template <class S>
void from_frame(const Bindings<S> &b, size_t offset, const Frame &f, S &s) {
  for (size_t i = 0; i < b.size(); ++i)
    b[i].get(f[offset + i], s);
}
template <class I, class O>
std::vector<PortSpec> specs(const Bindings<I> &in, const Bindings<O> &out) {
  std::vector<PortSpec> v;
  for (const auto &x : in)
    v.push_back(x.spec);
  for (const auto &x : out)
    v.push_back(x.spec);
  return v;
}

}  // namespace

std::vector<FeQualifier> fe_qualifiers(const std::vector<PortSpec> &spec) {
  // io.completed.bits := DontCare (ExecuteController.scala:172): with completed.valid low the
  // FIRRTL compiler drives what simplifies its logic. Every other output of the three tops is
  // equal in every cycle, valid or not (rtl_fe compares them all).
  static const struct {
    const char *port, *valid;
    unsigned div;
  } rules[] = {{"completed_bits", "completed_valid", 1}, {"ex_completed_bits", "ex_completed_valid", 1}};
  std::vector<FeQualifier> q(spec.size());
  for (const auto &r : rules) {
    const int p = port_index(spec, r.port), v = port_index(spec, r.valid);
    if (p >= 0 && v >= 0 && !spec[size_t(p)].input)
      q[size_t(p)] = {v, r.div};
  }
  return q;
}

const char *fe_top_name(FeTop t) {
  return t == FeTop::Execute ? "ExecuteTop" : t == FeTop::Cmd ? "CmdTop" : "CtrlTop";
}

bool parse_fe_top(const std::string &s, FeTop &t) {
  for (FeTop x : {FeTop::Execute, FeTop::Cmd, FeTop::Ctrl})
    if (s == fe_top_name(x)) {
      t = x;
      return true;
    }
  return false;
}

std::vector<PortSpec> fe_port_specs(const FrontendConfig &cfg, FeTop top) {
  const Widths w(cfg);
  switch (top) {
    case FeTop::Execute: return specs(ex_in(w), ex_out(w));
    case FeTop::Cmd: return specs(cmd_in(w), cmd_out(w));
    case FeTop::Ctrl: return specs(ctrl_in(w), ctrl_out(w));
  }
  return {};
}

void ex_in_to_frame(const FrontendConfig &cfg, const ExecuteTopIn &in, Frame &f) { to_frame(ex_in(Widths(cfg)), 0, in, f); }
void frame_to_ex_in(const FrontendConfig &cfg, const Frame &f, ExecuteTopIn &in) { from_frame(ex_in(Widths(cfg)), 0, f, in); }
void ex_out_to_frame(const FrontendConfig &cfg, const ExecuteTopOut &o, Frame &f) {
  const Widths w(cfg);
  to_frame(ex_out(w), ex_in(w).size(), o, f);
}
void frame_to_ex_out(const FrontendConfig &cfg, const Frame &f, ExecuteTopOut &o) {
  const Widths w(cfg);
  from_frame(ex_out(w), ex_in(w).size(), f, o);
}
void cmd_in_to_frame(const FrontendConfig &cfg, const CommandPath::In &in, Frame &f) { to_frame(cmd_in(Widths(cfg)), 0, in, f); }
void frame_to_cmd_in(const FrontendConfig &cfg, const Frame &f, CommandPath::In &in) { from_frame(cmd_in(Widths(cfg)), 0, f, in); }
void cmd_out_to_frame(const FrontendConfig &cfg, const CommandPath::Out &o, Frame &f) {
  const Widths w(cfg);
  to_frame(cmd_out(w), cmd_in(w).size(), o, f);
}
void frame_to_cmd_out(const FrontendConfig &cfg, const Frame &f, CommandPath::Out &o) {
  const Widths w(cfg);
  from_frame(cmd_out(w), cmd_in(w).size(), f, o);
}
void ctrl_in_to_frame(const FrontendConfig &cfg, const CtrlTopIn &in, Frame &f) { to_frame(ctrl_in(Widths(cfg)), 0, in, f); }
void frame_to_ctrl_in(const FrontendConfig &cfg, const Frame &f, CtrlTopIn &in) { from_frame(ctrl_in(Widths(cfg)), 0, f, in); }
void ctrl_out_to_frame(const FrontendConfig &cfg, const CtrlTopOut &o, Frame &f) {
  const Widths w(cfg);
  to_frame(ctrl_out(w), ctrl_in(w).size(), o, f);
}
void frame_to_ctrl_out(const FrontendConfig &cfg, const Frame &f, CtrlTopOut &o) {
  const Widths w(cfg);
  from_frame(ctrl_out(w), ctrl_in(w).size(), f, o);
}

}  // namespace systolique
