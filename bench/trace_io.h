// The stored reference traces (tests/reference/gemmini_rtl/<config>/<test>.csv.gz). Format: '#' comment lines (what made the trace), a header `cycle,<port>,...` with every
// port of the top in PortSpec order (inputs, then outputs), then one row per clock cycle from
// power-on: the inputs applied in that cycle and the outputs seen in it, packed lanes as hex
// (lane 0 in the least significant bits, as the RTL ports). The first rows hold reset.
#pragma once

#include "ports.h"

#include <string>
#include <vector>

namespace systolique {

struct TraceInfo {
  std::vector<std::string> comments;  // without the leading "# "
};
bool write_trace(const std::string &path, const std::vector<PortSpec> &spec,
                 const TraceInfo &info, const std::vector<Frame> &rows, std::string &err);
bool read_trace(const std::string &path, const std::vector<PortSpec> &spec, TraceInfo &info,
                std::vector<Frame> &rows, std::string &err);

}  // namespace systolique
