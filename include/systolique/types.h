// Small value types of the systolic array's interfaces: PE control, PE registers, the Mesh and
// MeshWithDelays ports (one cycle each), requests and responses. Plain data, no behaviour.
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Values are SInt held sign-extended in int64_t; lane vectors are flat, lane k = Vec(i)(j) with
// k = i * tileColumns + j (rows: i * tileRows + j), as in the packed ports of the RTL tops
// (rtl/src/GemminiTops.scala).
#pragma once

#include "systolique/config.h"

#include <cstdint>
#include <vector>

namespace systolique {

// PEControl (PE.scala:7-12).
struct PeControl {
  unsigned dataflow = 0;   // 0 = OS, 1 = WS
  unsigned propagate = 0;  // which of c1/c2 is propagated (1 = c1)
  unsigned shift = 0;      // rounding shift applied to the output on a propagate flip
  bool operator==(const PeControl &o) const {
    return dataflow == o.dataflow && propagate == o.propagate && shift == o.shift;
  }
  bool operator!=(const PeControl &o) const { return !(*this == o); }
};

// The three registers of a PE (PE.scala:70-71, 89).
struct PeRegs {
  int64_t c1 = 0, c2 = 0;
  unsigned last_s = 0;
};

// Mesh inputs (Mesh.scala:22-36), one cycle.
struct MeshIn {
  std::vector<int64_t> a;          // rows()
  std::vector<int64_t> b, d;       // cols()
  std::vector<PeControl> control;  // cols()
  std::vector<unsigned> id, last, valid;
  // The reset port of the MeshTop RTL top. The Mesh has no reset (Mesh.scala has no RegInit);
  // only SystolicArray reads it (a reset cycle restarts its cycle numbering and accounting).
  bool reset = false;
  void resize(const ArrayConfig &cfg);
};

// Mesh outputs (Mesh.scala:29-35), one cycle.
struct MeshOut {
  std::vector<int64_t> b, c;
  std::vector<unsigned> valid, id, last;
  std::vector<PeControl> control;
  void resize(const ArrayConfig &cfg);
  bool operator==(const MeshOut &o) const;
};

// MeshWithDelaysReq (MeshWithDelays.scala:9-17) with the bench tag {valid, id}.
struct MwdReq {
  unsigned dataflow = 0, propagate = 0, shift = 0;
  unsigned a_transpose = 0, bd_transpose = 0;
  unsigned total_rows = 0;
  unsigned tag_valid = 0, tag_id = 0;
  unsigned flush = 0;
};

// Inputs of MeshWithDelays (MeshWithDelays.scala:58-68) for one cycle: the valid halves of the
// a/b/d/req Decoupled ports, their bits, and the module reset.
struct MwdIn {
  unsigned a_valid = 0, b_valid = 0, d_valid = 0, req_valid = 0;
  std::vector<int64_t> a, b, d;  // rows() / cols() / cols()
  MwdReq req;
  bool reset = false;
  void resize(const ArrayConfig &cfg);
};

// Outputs of MeshWithDelays for one cycle: the ready halves, the resp Valid port and
// tags_in_progress. They depend on registers only (no combinational path from the inputs).
struct MwdOut {
  unsigned a_ready = 0, b_ready = 0, d_ready = 0, req_ready = 0;
  unsigned resp_valid = 0, resp_total_rows = 0, resp_tag_valid = 0, resp_tag_id = 0;
  unsigned resp_last = 0;
  std::vector<int64_t> resp_data;               // cols()
  std::vector<unsigned> tags_valid, tags_id;    // tagq_len()
  void resize(const ArrayConfig &cfg);
};

}  // namespace systolique
