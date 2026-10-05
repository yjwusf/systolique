// Gemmini's command encoding (GemminiISA.scala, LocalAddr.scala, Gemmini v0.7.2; gemmini.h of
// gemmini-rocc-tests 1a1a1c6). Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/isa.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <vector>

namespace systolique {

LocalAddr LocalAddr::cast(uint64_t v, const AddrMap &m) {
  // asTypeOf keeps the low 32 bits; the garbage field (between norm_cmd and garbage_bit) is
  // cleared (cast_to_local_addr, LocalAddr.scala:105-111)
  LocalAddr r(static_cast<uint32_t>(v), m);
  const unsigned lo = m.data_bits + 1, hi = 26;  // garbage field bits [25 : data_bits + 1]
  if (hi > lo)
    r.raw &= ~uint32_t(((1ull << (hi - lo)) - 1) << lo);
  return r;
}

const char *funct_name(unsigned f) {
  switch (f) {
    case CONFIG_CMD: return "config";
    case LOAD2_CMD: return "mvin2";
    case LOAD_CMD: return "mvin";
    case STORE_CMD: return "mvout";
    case COMPUTE_AND_FLIP_CMD: return "compute_preloaded";
    case COMPUTE_AND_STAY_CMD: return "compute_accumulated";
    case PRELOAD_CMD: return "preload";
    case FLUSH_CMD: return "flush";
    case LOOP_WS: return "loop_ws";
    case LOOP_WS_CONFIG_BOUNDS: return "loop_ws_config_bounds";
    case LOOP_WS_CONFIG_ADDRS_AB: return "loop_ws_config_addrs_ab";
    case LOOP_WS_CONFIG_ADDRS_DC: return "loop_ws_config_addrs_dc";
    case LOOP_WS_CONFIG_STRIDES_AB: return "loop_ws_config_strides_ab";
    case LOOP_WS_CONFIG_STRIDES_DC: return "loop_ws_config_strides_dc";
    case LOAD3_CMD: return "mvin3";
    case LOOP_CONV_WS: return "loop_conv_ws";
    case CLKGATE_EN: return "clkgate_en";
    case COUNTER_OP: return "counter_op";
    case kFence: return "fence";
    default:
      if (f > LOOP_CONV_WS && f <= LOOP_CONV_WS_CONFIG_6)
        return "loop_conv_ws_config";
      return "unknown";
  }
}

uint32_t float_bits(float f) {
  uint32_t b;
  std::memcpy(&b, &f, 4);
  return b;
}

uint32_t acc_addr(uint32_t row, bool accumulate, bool read_full) {
  return 0x80000000u | (accumulate ? 0x40000000u : 0) | (read_full ? 0x20000000u : 0) | row;
}

// gemmini_extended3_config_ex (gemmini.h:244-247)
Command cmd_config_ex(unsigned df, unsigned act, unsigned shift, unsigned a_stride, bool a_t, bool b_t,
                      unsigned c_stride, bool set_only_strides, float acc_scale) {
  return {CONFIG_CMD,
          (uint64_t(float_bits(acc_scale)) << 32) | (uint64_t(a_stride) << 16) | (uint64_t(b_t) << 9) |
              (uint64_t(a_t) << 8) | (uint64_t(set_only_strides) << 7) | (uint64_t(act) << 3) |
              (uint64_t(df) << 2) | CONFIG_EX,
          (uint64_t(c_stride) << 48) | shift};
}

// gemmini_extended5_config_ld with pixel_repeats 1 (gemmini.h:258-260)
Command cmd_config_ld(uint64_t stride, unsigned id, bool shrunk, unsigned block_stride, float scale) {
  return {CONFIG_CMD,
          (uint64_t(float_bits(scale)) << 32) | (uint64_t(block_stride) << 16) | (uint64_t(1) << 8) |
              (uint64_t(id) << 3) | (uint64_t(shrunk) << 2) | CONFIG_LOAD,
          stride};
}

// gemmini_extended_config_st (gemmini.h:275-281)
Command cmd_config_st(uint64_t stride, unsigned act, float acc_scale) {
  return {CONFIG_CMD, (uint64_t(act) << 2) | CONFIG_STORE,
          (uint64_t(float_bits(acc_scale)) << 32) | uint32_t(stride)};
}

Command cmd_mvin(unsigned which, uint64_t dram, uint32_t local, unsigned cols, unsigned rows) {
  static const unsigned f[3] = {LOAD_CMD, LOAD2_CMD, LOAD3_CMD};
  return {f[which % 3], dram, pack_rs(rows, cols, local)};
}

Command cmd_mvout(uint64_t dram, uint32_t local, unsigned cols, unsigned rows) {
  return {STORE_CMD, dram, pack_rs(rows, cols, local)};
}

Command cmd_preload(uint32_t bd, uint32_t c, unsigned bd_cols, unsigned bd_rows, unsigned c_cols,
                    unsigned c_rows) {
  return {PRELOAD_CMD, pack_rs(bd_rows, bd_cols, bd), pack_rs(c_rows, c_cols, c)};
}

Command cmd_compute(bool preloaded, uint32_t a, uint32_t bd, unsigned a_cols, unsigned a_rows,
                    unsigned bd_cols, unsigned bd_rows) {
  return {preloaded ? unsigned(COMPUTE_AND_FLIP_CMD) : unsigned(COMPUTE_AND_STAY_CMD),
          pack_rs(a_rows, a_cols, a), pack_rs(bd_rows, bd_cols, bd)};
}

Command cmd_flush() { return {FLUSH_CMD, 0, 0}; }
Command cmd_fence() { return {kFence, 0, 0}; }

void cmd_loop_ws(const LoopWsArgs &l, std::vector<Command> &q) {
  q.push_back({LOOP_WS_CONFIG_BOUNDS, (uint64_t(l.pad_K) << 32) | (uint64_t(l.pad_J) << 16) | l.pad_I,
               (uint64_t(l.K) << 32) | (uint64_t(l.J) << 16) | l.I});
  q.push_back({LOOP_WS_CONFIG_ADDRS_AB, l.A, l.B});
  q.push_back({LOOP_WS_CONFIG_ADDRS_DC, l.D, l.C});
  q.push_back({LOOP_WS_CONFIG_STRIDES_AB, l.A_stride, l.B_stride});
  q.push_back({LOOP_WS_CONFIG_STRIDES_DC, l.D_stride, l.C_stride});
  q.push_back({LOOP_WS,
               (uint64_t(l.a_spad_id) << 18) | (uint64_t(l.b_spad_id) << 16) | (uint64_t(l.act) << 8) |
                   (uint64_t(l.low_D) << 2) | (uint64_t(l.full_C) << 1) | uint64_t(l.ex_accumulate),
               (uint64_t(l.is_resadd) << 2) | (uint64_t(l.B_transpose) << 1) | uint64_t(l.A_transpose)});
}

std::string describe(const Command &c) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s rs1=0x%" PRIx64 " rs2=0x%" PRIx64, funct_name(c.funct), c.rs1, c.rs2);
  return buf;
}

}  // namespace systolique
