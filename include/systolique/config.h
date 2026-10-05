// Systolic-array configuration: the GemminiArrayConfig fields the spatial array uses
// (GemminiConfigs.scala:17-82, Gemmini v0.7.2) and the values derived from them.
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// The named configurations are the ones elaborated to Verilog (rtl/src/GemminiTops.scala,
// ArrayCfgs); "default" is Gemmini's defaultConfig (Configs.scala:20-35).
//
// Parameter annotations (docs/validation.md keeps the same table):
//   source     where the value or rule comes from
//   status     validated (a test compares it with the RTL) / partially validated / unvalidated
//   test       the ctest that checks it
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace systolique {

// Dataflow.scala:4-6. The PE control bit uses OS = 0, WS = 1 (PE.scala:94-95).
enum class Dataflow { OS = 0, WS = 1, BOTH = 2 };
const char *to_string(Dataflow df);

// Chisel's log2Up (at least 1) and log2Ceil.
unsigned log2_ceil(uint64_t x);
inline unsigned log2_up(uint64_t x) { return x <= 2 ? 1 : log2_ceil(x); }

// Every field below: source Configs.scala / GemminiConfigs.scala (Gemmini v0.7.2) as cited;
// status validated for the five named configurations (all their values replay the stored RTL
// traces exactly: systolique_trace_<config>; live lockstep: rtl_<config>), unvalidated for any
// other value inside the range ArrayConfig::validate() accepts.
struct ArrayConfig {
  std::string name;
  unsigned in_bits = 8;    // inputType       SInt(8)   (Configs.scala:23)
  unsigned out_bits = 20;  // spatialArrayOutputType SInt(20) (Configs.scala:26)
  unsigned acc_bits = 32;  // accType         SInt(32)  (Configs.scala:24)
  Dataflow dataflow = Dataflow::BOTH;        // Configs.scala:35
  unsigned tile_rows = 1, tile_cols = 1;     // PEs per Tile, combinational (Configs.scala:29-30)
  unsigned mesh_rows = 16, mesh_cols = 16;   // Tiles per Mesh, registered (Configs.scala:31-32)
  unsigned tile_latency = 0;                 // GemminiConfigs.scala:78
  unsigned output_delay = 1;                 // mesh_output_delay, GemminiConfigs.scala:79
  // Width of the bench tag id (rtl/src/GemminiTops.scala BenchTag). Gemmini's ExecuteController
  // tag carries a ROB id and an address that the array only stores and returns.
  unsigned tag_bits = 8;

  unsigned rows() const { return mesh_rows * tile_rows; }   // PE rows = DIM
  unsigned cols() const { return mesh_cols * tile_cols; }   // PE columns = DIM
  unsigned block_size() const { return rows(); }             // MeshWithDelays.scala:45-46
  // GemminiConfigs.scala:175 (use_tree_reduction_if_possible defaults to true).
  // Status: validated (ws_tree configuration).
  bool tree_reduction() const { return dataflow == Dataflow::WS && tile_rows > 1; }
  // MeshWithDelays.scala:48-54 (n_simultaneous_matmuls = -1). Status: validated for the five
  // configurations (5 everywhere: the formula's other branch, latency_per_pe > 1, is unvalidated).
  unsigned max_simultaneous_matmuls() const;
  unsigned tagq_len() const { return max_simultaneous_matmuls() + 1; }  // MeshWithDelays.scala:56
  unsigned id_bits() const { return log2_up(max_simultaneous_matmuls()); }  // Mesh.scala:27
  unsigned shift_bits() const { return log2_up(acc_bits); }            // PE.scala:10
  unsigned total_rows_bits() const { return log2_up(block_size() + 1); }  // MeshWithDelays.scala:14
  unsigned fire_counter_bits() const { return log2_up(block_size()); }  // MeshWithDelays.scala:98
  // Width of the PE's c1/c2 registers: inputType for a WS-only array, else accType (PE.scala:58).
  unsigned c_bits() const { return dataflow == Dataflow::WS ? in_bits : acc_bits; }
  // Width of the MacUnit's addend: outputType for WS-only, else accType (PE.scala:64-65).
  unsigned mac_c_bits() const { return dataflow == Dataflow::WS ? out_bits : acc_bits; }

  // Empty if the configuration is inside the supported range, else the reason (the components
  // throw std::invalid_argument on such a configuration). The range is what the model is written
  // for; only the five named configurations are validated against the RTL.
  std::string validate() const;
};

// The configurations elaborated to Verilog (same order and values as ArrayCfgs.all in
// rtl/src/GemminiTops.scala; tools/rtl_provenance.py check-traces compares them with
// tests/reference/gemmini_rtl/provenance.json).
const std::vector<ArrayConfig> &named_configs();
const ArrayConfig *find_config(const std::string &name);

}  // namespace systolique
