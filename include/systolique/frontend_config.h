// Configuration of Gemmini's frontend: the GemminiArrayConfig fields the ExecuteController, the
// scratchpad and accumulator banks, LoopMatmul and the ReservationStation use
// (GemminiConfigs.scala:17-108, Gemmini v0.7.2), at Gemmini's defaultConfig values
// (Configs.scala:21-162), with the values derived from them. Derived from Gemmini (BSD-3-Clause,
// see LICENSE.gemmini and NOTICE).
//
// Parameter annotations (docs/validation.md keeps the same table): every field below is
// Gemmini's defaultConfig value, source as cited, status validated at that value (the stored
// ExecuteTop / CmdTop / CtrlTop traces replay exactly: systolique_fe_trace_<top>; live lockstep:
// rtl_fe_*), unvalidated at any other value. validate() accepts only these values.
#pragma once

#include "systolique/config.h"

#include <cstdint>
#include <string>

namespace systolique {

// The geometry of local (scratchpad / accumulator) addresses (LocalAddr.scala:6-48), from the
// configuration. Every LocalAddr carries a copy (no global state).
struct AddrMap {
  unsigned data_bits = 14;          // max(sp_addr_bits, acc_addr_bits)
  unsigned sp_addr_bits = 14;       // log2(sp rows)
  unsigned acc_addr_bits = 10;      // log2(acc rows)
  unsigned sp_bank_row_bits = 12;   // log2(sp_bank_entries)
  unsigned acc_bank_row_bits = 9;   // log2(acc_bank_entries)
  unsigned sp_rows = 16384;
};

struct FrontendConfig {
  ArrayConfig array;  // the systolic array: Gemmini's 16x16 "default"

  // Scratchpad and accumulator (Configs.scala:38-45; GemminiConfigs.scala:42, 48)
  unsigned sp_banks = 4;              // Configs.scala:41
  unsigned sp_capacity_kib = 256;     // Configs.scala:38
  bool sp_singleported = true;        // Configs.scala:44
  unsigned acc_banks = 2;             // Configs.scala:42
  unsigned acc_capacity_kib = 64;     // Configs.scala:39
  bool acc_singleported = false;      // Configs.scala:45
  unsigned spad_read_delay = 4;       // GemminiConfigs.scala:42
  unsigned acc_latency = 2;           // GemminiConfigs.scala:48
  // Reservation station (Configs.scala:53-55) and the ExecuteController's command queue (:60)
  unsigned rs_entries_ld = 8, rs_entries_ex = 16, rs_entries_st = 4;
  unsigned ex_queue_length = 8;
  // DMA geometry the command path uses (Configs.scala:65)
  unsigned dma_maxbytes = 64;
  bool has_first_layer_optimizations = true;  // GemminiConfigs.scala:90 default
  bool has_nonlinear_activations = true;      // Configs.scala:50
  unsigned core_max_addr_bits = 40;  // Rocket coreMaxAddrBits with Sv39 (rtl/ex/src/CmdTop.scala)

  // Derived (GemminiConfigs.scala:85-200)
  unsigned dim() const { return array.block_size(); }
  unsigned sp_width_bits() const { return array.cols() * array.in_bits; }
  unsigned sp_row_bytes() const { return sp_width_bits() / 8; }
  unsigned acc_row_bytes() const { return array.cols() * array.acc_bits / 8; }
  unsigned sp_bank_entries() const { return sp_capacity_kib * 1024 * 8 / (sp_banks * sp_width_bits()); }
  unsigned acc_bank_entries() const {
    return acc_capacity_kib * 1024 * 8 / (acc_banks * array.cols() * array.acc_bits);
  }
  unsigned sp_rows() const { return sp_banks * sp_bank_entries(); }
  unsigned acc_rows() const { return acc_banks * acc_bank_entries(); }
  unsigned res_max_per_type() const;                 // GemminiConfigs.scala:194
  unsigned rs_entries() const { return res_max_per_type() * 3; }
  unsigned rob_type_bits() const { return log2_up(res_max_per_type()); }
  unsigned mvin_cols_bits() const;                   // GemminiConfigs.scala:160
  unsigned mvin_rows_bits() const { return log2_up(dim() + 1); }
  unsigned max_block_len() const;                    // LoopMatmul.scala:772
  unsigned max_block_len_acc() const;                // LoopMatmul.scala:773
  AddrMap addr_map() const;

  // Empty if the frontend classes support this configuration (Gemmini's defaultConfig, the one
  // elaborated in rtl/ex/), else the reason.
  std::string validate() const;
  static FrontendConfig gemmini_default();
};

}  // namespace systolique
