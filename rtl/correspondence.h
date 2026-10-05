// Internal-state checks of the lockstep bench, read from the Verilated RTL through VPI
// (docs/microarchitecture.md, "Cycle correspondence"), every cycle before the clock edge:
//
//   exact      registers the model keeps one for one (PE c1/c2/last_s, the request register,
//              counters, input buffers, transposer, tag and total-rows queues) equal their RTL
//              register (MeshWithDelays::registers, Mesh::registers)
//   feed       stage k of the input-skew ShiftRegister of lane l of a Mesh input
//              == model feed history entry k+1, lane l
//   resp       stage k of the output de-skew ShiftRegister of lane l
//              == model response history entry k, lane l
//   valid      stage k of the in_valid ShiftRegister in front of tile (r, c), lane j
//              == model valid chain stage k (exact)
//   pipev      the valid flop of a Pipe in front of tile (r, c), lane j, stage k (one per Pipe in
//              the RTL) == the same model valid chain stage k (merged)
//
// The register names of the shift chains come from <verilog>/<config>/correspondence.txt
// (tools/rtl_provenance.py correspondence).
#pragma once

#include "ports.h"

#include "systolique/systolic_array.h"

#include <cstdint>
#include <string>
#include <vector>

namespace systolique {

class Correspondence {
 public:
  // `scope`: the VPI name of the RTL top instance (e.g. "mwd_top.MeshWithDelaysTop").
  Correspondence(const ArrayConfig &cfg, Top top, const SystolicArray &model, std::string scope,
                 std::string map_file);
  // "" if every mapped RTL register equals its model state in this cycle, else the first
  // differences. The first call (after the RTL model exists) looks up the VPI handles.
  std::string check();
  unsigned long long values_checked() const { return checked_; }
  unsigned mapped() const { return unsigned(items_.size()); }
  // Model registers without an RTL register (removed by the FIRRTL compiler as unread).
  const std::vector<std::string> &unused() const { return unused_; }

 private:
  enum class Kind { Exact, Feed, Resp, Valid, PipeV };
  struct Item {
    Kind kind;
    std::string name;  // RTL path below the scope
    std::string sig;   // feed/resp signal
    unsigned a = 0, b = 0, c = 0, stage = 0;
    void *handle = nullptr;
    unsigned width = 0;
    size_t reg_index = 0;  // Exact: index into the model's register list
  };
  std::string init();
  int64_t model_value(const Item &it,
                      const std::vector<std::pair<std::string, int64_t>> &regs) const;

  ArrayConfig cfg_;
  Top top_;
  const SystolicArray &model_;
  std::string scope_, map_file_;
  bool ready_ = false;
  std::vector<Item> items_;
  std::vector<std::string> unused_;
  unsigned long long checked_ = 0;
  int fault_kind_ = -1;
  unsigned fault_count_ = 0;
};

}  // namespace systolique
