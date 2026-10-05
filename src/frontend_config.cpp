// Gemmini's frontend configuration (GemminiConfigs.scala, Configs.scala, Gemmini v0.7.2).
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/frontend_config.h"

#include <algorithm>

namespace systolique {

unsigned FrontendConfig::res_max_per_type() const {
  return std::max(rs_entries_ld, std::max(rs_entries_st, rs_entries_ex));
}

unsigned FrontendConfig::mvin_cols_bits() const {
  return log2_up(std::max(dma_maxbytes / (array.in_bits / 8), array.cols()) + 1);
}

unsigned FrontendConfig::max_block_len() const {
  return std::max(1u, dma_maxbytes / (dim() * array.in_bits / 8));
}

unsigned FrontendConfig::max_block_len_acc() const {
  return std::max(1u, dma_maxbytes / (dim() * array.acc_bits / 8));
}

AddrMap FrontendConfig::addr_map() const {
  AddrMap m;
  m.sp_addr_bits = log2_ceil(sp_rows());
  m.acc_addr_bits = log2_ceil(acc_rows());
  m.data_bits = std::max(m.sp_addr_bits, m.acc_addr_bits);
  m.sp_bank_row_bits = log2_up(sp_bank_entries());
  m.acc_bank_row_bits = log2_up(acc_bank_entries());
  m.sp_rows = sp_rows();
  return m;
}

std::string FrontendConfig::validate() const {
  const FrontendConfig d = gemmini_default();
  if (array.name != "default" || array.validate() != "")
    return "the frontend classes are validated with Gemmini's default 16x16 array only";
  if (sp_banks != d.sp_banks || sp_capacity_kib != d.sp_capacity_kib ||
      sp_singleported != d.sp_singleported || acc_banks != d.acc_banks ||
      acc_capacity_kib != d.acc_capacity_kib || acc_singleported != d.acc_singleported ||
      spad_read_delay != d.spad_read_delay || acc_latency != d.acc_latency ||
      rs_entries_ld != d.rs_entries_ld || rs_entries_ex != d.rs_entries_ex ||
      rs_entries_st != d.rs_entries_st || ex_queue_length != d.ex_queue_length ||
      dma_maxbytes != d.dma_maxbytes ||
      has_first_layer_optimizations != d.has_first_layer_optimizations ||
      has_nonlinear_activations != d.has_nonlinear_activations ||
      core_max_addr_bits != d.core_max_addr_bits)
    return "the frontend classes support Gemmini's defaultConfig only (the configuration "
           "elaborated in rtl/ex/)";
  return "";
}

FrontendConfig FrontendConfig::gemmini_default() {
  FrontendConfig c;
  c.array = *find_config("default");
  return c;
}

}  // namespace systolique
