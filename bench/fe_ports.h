// The boundary signals of the frontend RTL tops (rtl/ex/src/{ExecuteTop,CmdTop,CtrlTop}.scala) as
// port Frames (ports.h), and their conversion to and from the model's port structs
// (ExecuteTopIn/Out, CommandPath::In/Out, CtrlTopIn/Out). Vec ports are packed as in the RTL:
// lane k (bank k, element k) at bits [k*w +: w]. There is no reset port: the frontend traces
// start at cycle 0, after the four reset cycles (tests/reference/gemmini_fe/README.md).
#pragma once

#include "ports.h"

#include "systolique/command_path.h"
#include "systolique/controller.h"

#include <string>
#include <vector>

namespace systolique {

enum class FeTop { Execute, Cmd, Ctrl };
const char *fe_top_name(FeTop t);  // "ExecuteTop", "CmdTop", "CtrlTop"
bool parse_fe_top(const std::string &s, FeTop &t);

std::vector<PortSpec> fe_port_specs(const FrontendConfig &cfg, FeTop top);

// Output ports whose value the RTL leaves DontCare when a valid bit is low (the FIRRTL compiler
// then drives whatever simplifies its logic): lane l of such a port is compared only when lane
// l / div of its valid port is 1. Every other output lane is compared in every cycle. The only
// ones: the ExecuteController's completion id (ExecuteController.scala:172).
struct FeQualifier {
  int port = -1;  // the valid port (-1: always compared)
  unsigned div = 1;
};
std::vector<FeQualifier> fe_qualifiers(const std::vector<PortSpec> &spec);

// ExecuteTop
void ex_in_to_frame(const FrontendConfig &cfg, const ExecuteTopIn &in, Frame &f);
void frame_to_ex_in(const FrontendConfig &cfg, const Frame &f, ExecuteTopIn &in);
void ex_out_to_frame(const FrontendConfig &cfg, const ExecuteTopOut &o, Frame &f);
void frame_to_ex_out(const FrontendConfig &cfg, const Frame &f, ExecuteTopOut &o);
// CmdTop
void cmd_in_to_frame(const FrontendConfig &cfg, const CommandPath::In &in, Frame &f);
void frame_to_cmd_in(const FrontendConfig &cfg, const Frame &f, CommandPath::In &in);
void cmd_out_to_frame(const FrontendConfig &cfg, const CommandPath::Out &o, Frame &f);
void frame_to_cmd_out(const FrontendConfig &cfg, const Frame &f, CommandPath::Out &o);
// CtrlTop
void ctrl_in_to_frame(const FrontendConfig &cfg, const CtrlTopIn &in, Frame &f);
void frame_to_ctrl_in(const FrontendConfig &cfg, const Frame &f, CtrlTopIn &in);
void ctrl_out_to_frame(const FrontendConfig &cfg, const CtrlTopOut &o, Frame &f);
void frame_to_ctrl_out(const FrontendConfig &cfg, const Frame &f, CtrlTopOut &o);

}  // namespace systolique
