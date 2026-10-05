#include "systolique/micro_ops.h"

namespace systolique {

const char *to_string(UopKind k) {
  switch (k) {
    case UopKind::Config: return "config";
    case UopKind::Preload: return "preload";
    case UopKind::ComputePreloaded: return "compute_preloaded";
    case UopKind::ComputeAccumulated: return "compute_accumulated";
    case UopKind::Mvin: return "mvin";
    case UopKind::Mvout: return "mvout";
    case UopKind::Flush: return "flush";
    case UopKind::LoopCmd: return "loop_cmd";
    case UopKind::Fence: return "fence";
    case UopKind::OtherCmd: return "other";
    case UopKind::Request: return "request";
    case UopKind::OperandRead: return "operand_read";
    case UopKind::ResultRow: return "result_row";
    case UopKind::DmaRow: return "dma_row";
  }
  return "?";
}

bool is_command(UopKind k) { return k < UopKind::Request; }

const char *to_string(PassKind k) {
  switch (k) {
    case PassKind::SinglePreload: return "preload";
    case PassKind::MulPre: return "compute+preload";
    case PassKind::SingleMul: return "compute";
    case PassKind::Flush: return "flush";
  }
  return "?";
}

}  // namespace systolique
