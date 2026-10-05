// Gemmini's RoCC command encoding (GemminiISA.scala, Gemmini v0.7.2), local scratchpad /
// accumulator addresses (LocalAddr.scala), the command bundle that travels through the
// controllers (GemminiCmd, Controller.scala:16-21) and builders for the commands as gemmini.h
// (gemmini-rocc-tests 1a1a1c6, the commit Gemmini v0.7.2 pins) encodes them. Derived from Gemmini
// (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#pragma once

#include "systolique/fe_util.h"
#include "systolique/frontend_config.h"

#include <cstdint>
#include <string>
#include <vector>

namespace systolique {

// funct values (GemminiISA.scala:8-56)
enum Funct : unsigned {
  CONFIG_CMD = 0,
  LOAD2_CMD = 1,
  LOAD_CMD = 2,
  STORE_CMD = 3,
  COMPUTE_AND_FLIP_CMD = 4,  // compute_preloaded
  COMPUTE_AND_STAY_CMD = 5,  // compute_accumulated
  PRELOAD_CMD = 6,
  FLUSH_CMD = 7,
  LOOP_WS = 8,
  LOOP_WS_CONFIG_BOUNDS = 9,
  LOOP_WS_CONFIG_ADDRS_AB = 10,
  LOOP_WS_CONFIG_ADDRS_DC = 11,
  LOOP_WS_CONFIG_STRIDES_AB = 12,
  LOOP_WS_CONFIG_STRIDES_DC = 13,
  LOAD3_CMD = 14,
  LOOP_CONV_WS = 15,
  LOOP_CONV_WS_CONFIG_1 = 16,
  LOOP_CONV_WS_CONFIG_6 = 21,
  CLKGATE_EN = 22,
  COUNTER_OP = 126,
};
// rs1[1:0] of CONFIG_CMD (GemminiISA.scala:38-41)
enum ConfigType : unsigned { CONFIG_EX = 0, CONFIG_LOAD = 1, CONFIG_STORE = 2, CONFIG_NORM = 3 };
// Activation (Activation.scala): NONE 0, RELU 1, LAYERNORM 2, IGELU 3, SOFTMAX 4
enum Act : unsigned { ACT_NONE = 0, ACT_RELU = 1, ACT_LAYERNORM = 2, ACT_IGELU = 3, ACT_SOFTMAX = 4 };
// OS = 0, WS = 1 (Dataflow.scala:4-6)
constexpr unsigned kOS = 0, kWS = 1;

const char *funct_name(unsigned funct);

// LocalAddr (LocalAddr.scala:6-102), 32 bits, MSB first: is_acc_addr, accumulate,
// read_full_acc_row, norm_cmd(3), garbage, garbage_bit, data(data_bits).
struct LocalAddr {
  uint32_t raw = 0;
  AddrMap m;

  LocalAddr() = default;
  LocalAddr(uint64_t v, const AddrMap &map) : raw(uint32_t(v)), m(map) {}

  bool is_acc() const { return raw >> 31 & 1; }
  bool accumulate() const { return raw >> 30 & 1; }
  bool read_full() const { return raw >> 29 & 1; }
  unsigned norm_cmd() const { return raw >> 26 & 7; }
  bool garbage_bit() const { return raw >> m.data_bits & 1; }
  uint32_t data() const { return uint32_t(raw & mask_bits(m.data_bits)); }
  uint32_t sp_bank() const { return data() >> m.sp_bank_row_bits; }
  uint32_t sp_row() const { return data() & uint32_t(mask_bits(m.sp_bank_row_bits)); }
  uint32_t acc_bank() const {
    return (data() & uint32_t(mask_bits(m.acc_addr_bits))) >> m.acc_bank_row_bits;
  }
  uint32_t acc_row() const { return data() & uint32_t(mask_bits(m.acc_bank_row_bits)); }
  uint32_t full_sp_addr() const { return data() & uint32_t(mask_bits(m.sp_addr_bits)); }
  uint32_t full_acc_addr() const { return data() & uint32_t(mask_bits(m.acc_addr_bits)); }
  // LocalAddr.scala:39-44
  bool is_garbage() const {
    return is_acc() && accumulate() && read_full() && data() == mask_bits(m.data_bits) &&
           garbage_bit();
  }
  bool is_same_address(const LocalAddr &o) const { return is_acc() == o.is_acc() && data() == o.data(); }
  // `+` (LocalAddr.scala:46-53): data + other, truncated
  LocalAddr plus(uint64_t other) const {
    LocalAddr r = *this;
    r.set_data(data() + other);
    return r;
  }
  // add_with_overflow (LocalAddr.scala:67-79)
  LocalAddr plus_overflow(uint64_t other, bool &overflow) const {
    const uint64_t sum = uint64_t(data()) + other;
    overflow = (sum >> (is_acc() ? m.acc_addr_bits : m.sp_addr_bits)) & 1;
    LocalAddr r = *this;
    r.set_data(sum);
    return r;
  }
  // floorSub (LocalAddr.scala:82-92)
  LocalAddr floor_sub(uint64_t other, uint64_t floor, bool &underflow) const {
    underflow = data() < floor + other;
    LocalAddr r = *this;
    r.set_data(underflow ? floor : data() - other);
    return r;
  }
  bool le(const LocalAddr &o) const {
    return is_acc() == o.is_acc() &&
           (is_acc() ? full_acc_addr() <= o.full_acc_addr() : full_sp_addr() <= o.full_sp_addr());
  }
  bool lt(const LocalAddr &o) const {
    return is_acc() == o.is_acc() &&
           (is_acc() ? full_acc_addr() < o.full_acc_addr() : full_sp_addr() < o.full_sp_addr());
  }
  void set_data(uint64_t d) {
    raw = uint32_t((raw & ~uint32_t(mask_bits(m.data_bits))) | (d & mask_bits(m.data_bits)));
  }
  void set_flags(bool acc, bool accum, bool full) {
    raw = (raw & 0x1fffffffu) | (uint32_t(acc) << 31) | (uint32_t(accum) << 30) | (uint32_t(full) << 29);
  }
  // garbage_addr / make_this_garbage (LocalAddr.scala:94-100, 139-144): is_acc_addr,
  // accumulate, read_full_acc_row, garbage_bit and data all ones; the DontCare fields (norm_cmd,
  // garbage) are 0 in the elaborated RTL (0xe0007fff by default; the CmdTop lockstep found it).
  static LocalAddr garbage(const AddrMap &m) {
    return LocalAddr(uint32_t(0xe0000000u | (1ull << m.data_bits) | mask_bits(m.data_bits)), m);
  }
  // cast_to_local_addr / cast_to_sp_addr / cast_to_acc_addr (LocalAddr.scala:104-137)
  static LocalAddr cast(uint64_t v, const AddrMap &m);
  static LocalAddr cast_sp(uint64_t v, const AddrMap &m) {
    LocalAddr r = cast(v, m);
    r.set_flags(false, false, false);
    return r;
  }
  static LocalAddr cast_acc(uint64_t v, bool accumulate, bool read_full, const AddrMap &m) {
    LocalAddr r = cast(v, m);
    r.set_flags(true, accumulate, read_full);
    return r;
  }
};

// Fields of rs1/rs2 (MvinRs2, PreloadRs, ComputeRs, MvoutRs2: rows at bit 48, cols at bit 32,
// the local address in the low 32 bits; GemminiISA.scala:67-237).
inline unsigned rs_rows(uint64_t rs, unsigned bits) { return unsigned((rs >> 48) & mask_bits(bits)); }
inline unsigned rs_cols(uint64_t rs, unsigned bits) { return unsigned((rs >> 32) & mask_bits(bits)); }
inline uint64_t pack_rs(unsigned rows, unsigned cols, uint32_t addr) {
  return (uint64_t(rows) << 48) | (uint64_t(cols) << 32) | addr;
}

// The RoCCCommand fields the frontend uses, and the GemminiCmd around it. `op` and `uop` are
// bookkeeping of the model, not hardware: the operation and the micro-op (micro_ops.h) the
// command belongs to. They travel with the command and never steer a decision.
struct GemminiCmd {
  unsigned funct = 0;
  uint64_t rs1 = 0, rs2 = 0;
  bool rob_valid = false;  // rob_id (UDValid)
  unsigned rob_id = 0;
  bool from_matmul_fsm = false, from_conv_fsm = false;
  int op = -1;
  int64_t uop = -1;
  int64_t parent_uop = -1;  // LoopMatmul's commands: the loop_ws command's micro-op
};

// A command as software issues it (gemmini.h): funct, rs1, rs2. `kFence` is not a Gemmini
// command: the core's fence, which waits until Gemmini is not busy (gemmini.h gemmini_fence).
struct Command {
  unsigned funct = 0;
  uint64_t rs1 = 0, rs2 = 0;
};
constexpr unsigned kFence = 1000;
constexpr uint32_t kGarbageAddr = 0xffffffffu;  // gemmini.h GARBAGE_ADDR

// Builders, as gemmini.h's macros encode them (gemmini.h:198-292, 349-357).
uint32_t acc_addr(uint32_t row, bool accumulate, bool read_full = false);
Command cmd_config_ex(unsigned dataflow, unsigned act = 0, unsigned shift = 0, unsigned a_stride = 1,
                      bool a_transpose = false, bool b_transpose = false, unsigned c_stride = 1,
                      bool set_only_strides = false, float acc_scale = 1.0f);
Command cmd_config_ld(uint64_t stride, unsigned id = 0, bool shrunk = false, unsigned block_stride = 16,
                      float scale = 1.0f);
Command cmd_config_st(uint64_t stride, unsigned act = 0, float acc_scale = 1.0f);
Command cmd_mvin(unsigned which, uint64_t dram, uint32_t local, unsigned cols, unsigned rows);  // which: 0,1,2
Command cmd_mvout(uint64_t dram, uint32_t local, unsigned cols, unsigned rows);
Command cmd_preload(uint32_t bd, uint32_t c, unsigned bd_cols = 16, unsigned bd_rows = 16,
                    unsigned c_cols = 16, unsigned c_rows = 16);
Command cmd_compute(bool preloaded, uint32_t a, uint32_t bd, unsigned a_cols = 16, unsigned a_rows = 16,
                    unsigned bd_cols = 16, unsigned bd_rows = 16);
Command cmd_flush();
Command cmd_fence();
// gemmini_loop_ws (gemmini.h:349-357): six commands.
struct LoopWsArgs {
  unsigned I = 1, J = 1, K = 1, pad_I = 0, pad_J = 0, pad_K = 0;
  uint64_t A = 0, B = 0, D = 0, C = 0, A_stride = 0, B_stride = 0, D_stride = 0, C_stride = 0;
  bool A_transpose = false, B_transpose = false, full_C = false, low_D = false, ex_accumulate = false;
  unsigned act = 0, a_spad_id = 0, b_spad_id = 0;
  bool is_resadd = false;
};
void cmd_loop_ws(const LoopWsArgs &l, std::vector<Command> &out);
uint32_t float_bits(float f);
std::string describe(const Command &c);  // "preload rs1=... rs2=..."

}  // namespace systolique
