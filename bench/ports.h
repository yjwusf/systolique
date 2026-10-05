// The boundary signals of the two RTL tops (rtl/src/GemminiTops.scala) as packed bit vectors:
// one Frame holds every port of a top for one cycle, in PortSpec order (reset and the other
// inputs first, then the outputs). The stored reference traces, the Verilated tops (rtl/) and
// SystolicArray all meet here.
#pragma once

#include "systolique/config.h"
#include "systolique/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace systolique {

// A packed vector of `width` bits, 32 per word, bit 0 = LSB of word 0 (Verilator's layout).
struct Bits {
  unsigned width = 0;
  std::vector<uint32_t> w;

  void resize(unsigned bits);
  uint64_t get(unsigned lo, unsigned n) const;  // n <= 64
  void set(unsigned lo, unsigned n, uint64_t v);
  std::string hex() const;                       // ceil(width/4) digits, MSB first
  bool parse_hex(const std::string &s);          // into the current width
  bool operator==(const Bits &o) const { return width == o.width && w == o.w; }
  bool operator!=(const Bits &o) const { return !(*this == o); }
};

enum class Top { Mesh, MeshWithDelays };
const char *top_name(Top t);  // "MeshTop" / "MeshWithDelaysTop"

struct PortSpec {
  std::string name;  // RTL port without the "io_" prefix ("reset" for the reset input)
  bool input;
  unsigned lanes, lane_bits;
  bool is_signed;    // lanes hold SInt values
  unsigned width() const { return lanes * lane_bits; }
};

std::vector<PortSpec> port_specs(const ArrayConfig &cfg, Top top);

using Frame = std::vector<Bits>;
Frame make_frame(const std::vector<PortSpec> &spec);
int port_index(const std::vector<PortSpec> &spec, const std::string &name);  // -1 if absent

// Conversions between frames and the model's structs (reset is port 0 of every frame and the
// `reset` field of MeshIn / MwdIn).
void mesh_in_to_frame(const ArrayConfig &cfg, const MeshIn &in, Frame &f);
void frame_to_mesh_in(const ArrayConfig &cfg, const Frame &f, MeshIn &in);
void mesh_out_to_frame(const ArrayConfig &cfg, const MeshOut &o, Frame &f);
void mwd_in_to_frame(const ArrayConfig &cfg, const MwdIn &in, Frame &f);
void frame_to_mwd_in(const ArrayConfig &cfg, const Frame &f, MwdIn &in);
void mwd_out_to_frame(const ArrayConfig &cfg, const MwdOut &o, Frame &f);
void frame_to_mwd_out(const ArrayConfig &cfg, const Frame &f, MwdOut &o);

// "port[lane]: rtl=<hex> model=<hex>" for every differing output port lane, or "".
std::string diff_outputs(const std::vector<PortSpec> &spec, const Frame &ref, const Frame &dut,
                         unsigned max_items = 8);

}  // namespace systolique
