// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/config.h"

#include <cmath>

namespace systolique {

const char *to_string(Dataflow df) {
  switch (df) {
    case Dataflow::OS: return "OS";
    case Dataflow::WS: return "WS";
    case Dataflow::BOTH: return "BOTH";
  }
  return "?";
}

unsigned log2_ceil(uint64_t x) {
  unsigned n = 0;
  while ((uint64_t(1) << n) < x) ++n;
  return n;
}

unsigned ArrayConfig::max_simultaneous_matmuls() const {
  // latency_per_pe = ((tile_latency + 1).toFloat / (tileRows min tileColumns)) max 1.0f
  // (MeshWithDelays.scala:48-54); float arithmetic as in Scala.
  float lat = float(tile_latency + 1) / float(tile_rows < tile_cols ? tile_rows : tile_cols);
  if (lat < 1.0f) lat = 1.0f;
  return unsigned(std::ceil(5 * lat));
}

std::string ArrayConfig::validate() const {
  auto pow2 = [](unsigned x) { return x && !(x & (x - 1)); };
  if (rows() != cols()) return "meshRows*tileRows must equal meshColumns*tileColumns";
  if (!pow2(block_size()) || block_size() < 2)
    return "DIM must be a power of two >= 2 (AlwaysOutTransposer, Transposer.scala:95)";
  if (in_bits < 2 || in_bits > 16) return "inputType width must be 2..16";
  if (out_bits < in_bits || out_bits > 32) return "output width must be inputType..32";
  if (acc_bits < out_bits || acc_bits > 32) return "accType width must be output..32";
  if (tag_bits < 1 || tag_bits > 16) return "tag width must be 1..16";
  if (tile_latency > 4 || output_delay > 4) return "tile_latency and output_delay must be 0..4";
  if (rows() > 64) return "DIM must be <= 64";
  return "";
}

const std::vector<ArrayConfig> &named_configs() {
  // name, in, out, acc, dataflow, tileRows, tileColumns, meshRows, meshColumns, tile_latency,
  // output_delay, tag bits -- rtl/src/GemminiTops.scala ArrayCfgs.all.
  // Status: validated (systolique_trace_<config>, rtl_<config>).
  static const std::vector<ArrayConfig> cfgs = {
      {"default", 8, 20, 32, Dataflow::BOTH, 1, 1, 16, 16, 0, 1, 8},
      {"dim4", 8, 20, 32, Dataflow::BOTH, 1, 1, 4, 4, 0, 1, 8},
      {"tiled", 8, 20, 32, Dataflow::BOTH, 2, 2, 4, 4, 1, 2, 8},
      {"ws_tree", 8, 20, 32, Dataflow::WS, 2, 2, 4, 4, 0, 1, 8},
      {"os16", 16, 24, 32, Dataflow::OS, 1, 1, 8, 8, 0, 1, 8},
  };
  return cfgs;
}

const ArrayConfig *find_config(const std::string &name) {
  for (const auto &c : named_configs())
    if (c.name == name) return &c;
  return nullptr;
}

}  // namespace systolique
