// Engine: Controller + Host + the micro-op table (engine.h). The matmul command streams follow
// gemmini.h of gemmini-rocc-tests 1a1a1c6 (tiled_matmul_auto, tiled_matmul_outer,
// sp_tiled_matmul_os, sp_tiled_matmul_ws); that part is derived from Gemmini's software
// (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
#include "systolique/engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>

namespace systolique {

namespace {
UopKind kind_of(unsigned funct) {
  switch (funct) {
    case CONFIG_CMD: return UopKind::Config;
    case PRELOAD_CMD: return UopKind::Preload;
    case COMPUTE_AND_FLIP_CMD: return UopKind::ComputePreloaded;
    case COMPUTE_AND_STAY_CMD: return UopKind::ComputeAccumulated;
    case LOAD_CMD:
    case LOAD2_CMD:
    case LOAD3_CMD: return UopKind::Mvin;
    case STORE_CMD: return UopKind::Mvout;
    case kFence: return UopKind::Fence;
    default:
      return funct >= LOOP_WS && funct <= LOOP_WS_CONFIG_STRIDES_DC ? UopKind::LoopCmd : UopKind::OtherCmd;
  }
}

int8_t clip8(int64_t v) { return int8_t(std::min<int64_t>(127, std::max<int64_t>(-128, v))); }
}  // namespace

struct Engine::Vcd {
  explicit Vcd(const std::string &path) : w(path, "engine") {}
  VcdWriter w;
  int clock = 0, cycle = 0;
  std::vector<int> ports;
  int uop_head = 0, uop_pass = 0, uop_in = 0, uop_out = 0, op_in = 0, ex_state = 0;
};

Engine::Engine(EngineOptions opt) : opt_(std::move(opt)) {
  if (const std::string e = opt_.cfg.validate(); !e.empty())
    throw std::invalid_argument("Engine: " + e);
  ctrl_ = std::make_unique<Controller>(opt_.cfg, opt_.array);
  host_ = std::make_unique<Host>(opt_.cfg, opt_.dma, &dram_);
  if (opt_.micro_ops) {
    ctrl_->set_micro_ops(&uops_);
    host_->set_micro_ops(&uops_);
  }
  if (!opt_.vcd_path.empty())
    declare_vcd();
}

Engine::~Engine() = default;

int Engine::submit(const std::vector<Command> &cmds, const std::string &label) {
  // Operands from the accumulator need its read path (normalizer, AccumulatorScale) to the
  // ExecuteController, which the model does not have (scratchpad.h): refuse them up front.
  const AddrMap am = opt_.cfg.addr_map();
  for (const Command &c : cmds) {
    const bool pre = c.funct == PRELOAD_CMD, comp = c.funct == COMPUTE_AND_FLIP_CMD || c.funct == COMPUTE_AND_STAY_CMD;
    const LocalAddr r1(c.rs1, am), r2(c.rs2, am);
    if (((pre || comp) && r1.is_acc() && !r1.is_garbage()) || (comp && r2.is_acc() && !r2.is_garbage()))
      throw std::invalid_argument("Engine::submit: " + describe(c) +
                                  " reads an operand from the accumulator, which the model does not support");
    if (c.funct >= LOOP_CONV_WS && c.funct <= LOOP_CONV_WS_CONFIG_6)
      throw std::invalid_argument("Engine::submit: loop_conv_ws is not modelled");
  }
  Operation o;
  o.label = label;
  o.commands = unsigned(cmds.size());
  const int op = uops_.add_op(o);
  for (const Command &c : cmds) {
    int64_t id = -1;
    if (opt_.micro_ops) {
      MicroOp u;
      u.kind = kind_of(c.funct);
      u.op = op;
      u.funct = c.funct;
      u.rs1 = c.rs1;
      u.rs2 = c.rs2;
      id = uops_.add(u);
    }
    host_->push(HostCmd{c, op, id, 0});
  }
  finalized_ = false;
  return op;
}

// tiled_matmul_auto -> tiled_matmul -> tiled_matmul_outer (gemmini.h:1231-1330, 1104-1218, 692-848)
std::vector<Command> Engine::matmul_commands(const Matmul &m, const FrontendConfig &cfg, uint64_t A, uint64_t B,
                                             uint64_t D, uint64_t C) {
  const size_t DIM = cfg.dim();
  const size_t BANK_NUM = cfg.sp_banks, BANK_ROWS = cfg.sp_bank_entries(), ACC_ROWS = cfg.acc_rows();
  const bool ws = m.dataflow == kWS;
  const size_t dim_I = m.M, dim_J = m.N, dim_K = m.K;
  const size_t dim_I_padded = (dim_I / DIM + (dim_I % DIM != 0)) * DIM;
  const size_t dim_J_padded = (dim_J / DIM + (dim_J % DIM != 0)) * DIM;
  const size_t dim_K_padded = (dim_K / DIM + (dim_K % DIM != 0)) * DIM;
  // tiling factors (gemmini.h:1236-1300)
  const bool double_buffered = ws;
  const size_t max_spad_rows = double_buffered ? BANK_NUM * BANK_ROWS / 2 : BANK_NUM * BANK_ROWS;
  const size_t max_acc_rows = double_buffered ? ACC_ROWS / 2 : ACC_ROWS;
  const size_t partition_rows = BANK_NUM * BANK_ROWS / 2;
  const size_t mats_in_partition = partition_rows / DIM, mats_in_acc = ACC_ROWS / DIM;
  const size_t max_tile_i_j = size_t(std::sqrt(double(mats_in_acc))), max_tile_k = mats_in_partition / max_tile_i_j;
  const size_t db_partition_rows = (BANK_NUM * BANK_ROWS / 2) / 2;
  const size_t db_mats_in_partition = db_partition_rows / DIM, db_mats_in_acc = (ACC_ROWS / 2) / DIM;
  const size_t db_max_tile_i_j = size_t(std::sqrt(double(db_mats_in_acc)));
  const size_t db_max_tile_k = db_mats_in_partition / db_max_tile_i_j;
  size_t tile_I, tile_J, tile_K;
  if (double_buffered) {
    tile_I = std::min(dim_I_padded / DIM, db_max_tile_i_j);
    tile_J = std::min(dim_J_padded / DIM, db_max_tile_i_j);
    tile_K = std::min(dim_K_padded / DIM, db_max_tile_k);
  } else {
    tile_I = std::min(dim_I_padded / DIM, max_tile_i_j);
    tile_J = std::min(dim_J_padded / DIM, max_tile_i_j);
    tile_K = std::min(dim_K_padded / DIM, max_tile_k);
  }
  auto spad_rows = [&](size_t I, size_t J, size_t K) { return (I * K + K * J) * DIM; };
  auto acc_rows = [&](size_t I, size_t J) { return I * J * DIM; };
  while (true) {
    bool increased = false;
    if (spad_rows(tile_I, tile_J + 1, tile_K) <= max_spad_rows && acc_rows(tile_I, tile_J + 1) <= max_acc_rows &&
        (tile_J + 1) * DIM <= dim_J_padded) {
      ++tile_J;
      increased = true;
    }
    if (spad_rows(tile_I + 1, tile_J, tile_K) <= max_spad_rows && acc_rows(tile_I + 1, tile_J) <= max_acc_rows &&
        (tile_I + 1) * DIM <= dim_I_padded) {
      ++tile_I;
      increased = true;
    }
    if (spad_rows(tile_I, tile_J, tile_K + 1) <= max_spad_rows && (tile_K + 1) * DIM <= dim_K_padded) {
      ++tile_K;
      increased = true;
    }
    if (!increased)
      break;
  }

  // tiled_matmul_outer (gemmini.h:692-848)
  const size_t stride_A = dim_K, stride_B = dim_J, stride_D = dim_J, stride_C = dim_J;
  const size_t I0 = dim_I_padded / (tile_I * DIM) + (dim_I_padded % (tile_I * DIM) != 0);
  const size_t J0 = dim_J_padded / (tile_J * DIM) + (dim_J_padded % (tile_J * DIM) != 0);
  const size_t K0 = dim_K_padded / (tile_K * DIM) + (dim_K_padded % (tile_K * DIM) != 0);
  const size_t last_I = dim_I_padded % (tile_I * DIM) == 0 ? tile_I : (dim_I_padded / DIM) % tile_I;
  const size_t last_J = dim_J_padded % (tile_J * DIM) == 0 ? tile_J : (dim_J_padded / DIM) % tile_J;
  const size_t last_K = dim_K_padded % (tile_K * DIM) == 0 ? tile_K : (dim_K_padded / DIM) % tile_K;
  const size_t padding_I = dim_I_padded - dim_I, padding_J = dim_J_padded - dim_J, padding_K = dim_K_padded - dim_K;
  const bool no_bias = !m.bias;
  const uint64_t Dp = no_bias ? 1 : D;  // gemmini.h:731-734: a dummy address that is not NULL
  const bool low_D = false, full_C = m.full_c;
  const size_t sizeof_D = low_D ? 1 : 4, sizeof_C = full_C ? 4 : 1;
  const unsigned act = m.act;

  std::vector<Command> q;
  q.push_back(cmd_config_ex(m.dataflow, act & 3, 0, 1, false, false));
  q.push_back(cmd_config_st(stride_C * sizeof_C, act & 3, 1.0f));
  q.push_back(cmd_config_ld(stride_A, 0, false, unsigned(DIM)));
  q.push_back(cmd_config_ld(stride_B, 1, false, unsigned(DIM)));
  q.push_back(cmd_config_ld(stride_D * sizeof_D, 2, low_D, unsigned(DIM)));

  const bool b_reuse = (J0 * K0 <= 2) && ws, a_reuse = (I0 * K0 <= 2) && ws;
  for (size_t i0 = 0; i0 < I0; i0++)
    for (size_t j0 = 0; j0 < J0; j0++)
      for (size_t k0 = 0; k0 < K0; k0++) {
        unsigned a_spad_id = 0, b_spad_id = 0;
        if (a_reuse)
          a_spad_id = (i0 + k0) == 0 ? 1 : 2;
        if (b_reuse)
          b_spad_id = (j0 + k0) == 0 ? 1 : 2;
        const uint64_t pre = k0 != 0 ? 0 : Dp + (i0 * tile_I * DIM * stride_D + j0 * tile_J * DIM) * sizeof_D;
        const uint64_t out = k0 == K0 - 1 ? C + (i0 * tile_I * DIM * stride_C + j0 * tile_J * DIM) * sizeof_C : 0;
        const size_t I = i0 < I0 - 1 ? tile_I : last_I, J = j0 < J0 - 1 ? tile_J : last_J,
                     K = k0 < K0 - 1 ? tile_K : last_K;
        const size_t pad_I = i0 == I0 - 1 ? padding_I : 0, pad_J = j0 == J0 - 1 ? padding_J : 0,
                     pad_K = k0 == K0 - 1 ? padding_K : 0;
        uint64_t a = A + i0 * tile_I * DIM * stride_A + k0 * tile_K * DIM;
        uint64_t b = B + k0 * tile_K * DIM * stride_B + j0 * tile_J * DIM;
        if (a_reuse && j0 >= 1)
          a = 0;
        if (b_reuse && i0 >= 1)
          b = 0;
        if (ws) {
          // sp_tiled_matmul_ws (gemmini.h:683-689): one gemmini_loop_ws
          LoopWsArgs l;
          l.I = unsigned(I);
          l.J = unsigned(J);
          l.K = unsigned(K);
          l.pad_I = unsigned(pad_I);
          l.pad_J = unsigned(pad_J);
          l.pad_K = unsigned(pad_K);
          l.A = a;
          l.B = b;
          l.D = no_bias ? 0 : pre;
          l.C = out;
          l.A_stride = stride_A;
          l.B_stride = stride_B;
          l.D_stride = stride_D;
          l.C_stride = stride_C;
          l.full_C = full_C;
          l.low_D = low_D;
          l.ex_accumulate = !no_bias || pre == 0;
          l.act = act;
          l.a_spad_id = a_spad_id;
          l.b_spad_id = b_spad_id;
          cmd_loop_ws(l, q);
        } else {
          // sp_tiled_matmul_os (gemmini.h:380-500)
          const uint32_t ADDR_LEN = 32;
          const uint32_t A_sp_addr_start = 0;
          const uint32_t B_sp_addr_start = uint32_t(BANK_NUM * BANK_ROWS - K * J * DIM);
          const uint32_t D_sp_addr_start = 1u << (ADDR_LEN - 1);
          const uint32_t C_sp_addr_start = (3u << (ADDR_LEN - 2)) | (uint32_t(full_C) << (ADDR_LEN - 3));
          const size_t MAX_BLOCK_LEN = cfg.max_block_len(), MAX_BLOCK_LEN_ACC = cfg.max_block_len_acc();
          const size_t A_blocks = std::min(K, MAX_BLOCK_LEN), B_blocks = std::min(J, MAX_BLOCK_LEN),
                       D_blocks = std::min(J, MAX_BLOCK_LEN_ACC);
          if (pre != 0 && !no_bias) {
            q.push_back(cmd_config_ld(stride_D * 4, 0, false, unsigned(DIM)));
            for (size_t i = 0; i < I; i++)
              for (size_t j = 0; j < J; j += D_blocks) {
                const uint64_t dram = pre + (i * DIM * stride_D + j * DIM) * 4;
                const uint32_t sp = uint32_t(D_sp_addr_start + (i * J + j) * DIM);
                const size_t blocks = j + D_blocks <= J ? D_blocks : J - j;
                const size_t cols = blocks * DIM - (j + blocks >= J ? pad_J : 0);
                const size_t rows = DIM - (i == I - 1 ? pad_I : 0);
                q.push_back(cmd_mvin(0, dram, sp, unsigned(cols), unsigned(rows)));
              }
          }
          q.push_back(cmd_config_ld(stride_B, 0, false, unsigned(DIM)));
          for (size_t j = 0; j < J; j += B_blocks)
            for (size_t k = 0; k < K; k++) {
              const uint64_t dram = b + (k * DIM * stride_B + j * DIM);
              const uint32_t sp = uint32_t(B_sp_addr_start + (k * J + j) * DIM);
              const size_t blocks = j + B_blocks <= J ? B_blocks : J - j;
              const size_t cols = blocks * DIM - (j + blocks >= J ? pad_J : 0);
              const size_t rows = DIM - (k == K - 1 ? pad_K : 0);
              q.push_back(cmd_mvin(0, dram, sp, unsigned(cols), unsigned(rows)));
            }
          q.push_back(cmd_config_ld(stride_A, 0, false, unsigned(DIM)));
          for (size_t i = 0; i < I; i++)
            for (size_t k = 0; k < K; k += A_blocks) {
              const uint64_t dram = a + (i * DIM * stride_A + k * DIM);
              const uint32_t sp = uint32_t(A_sp_addr_start + (i * K + k) * DIM);
              const size_t blocks = k + A_blocks <= K ? A_blocks : K - k;
              const size_t cols = blocks * DIM - (k + blocks >= K ? pad_K : 0);
              const size_t rows = DIM - (i == I - 1 ? pad_I : 0);
              q.push_back(cmd_mvin(0, dram, sp, unsigned(cols), unsigned(rows)));
            }
          for (size_t i = 0; i < I; i++)
            for (size_t j = 0; j < J; j++) {
              const uint32_t C_sp_addr = uint32_t(C_sp_addr_start + (i * J + j) * DIM);
              for (size_t k = 0; k < K; k++) {
                const uint32_t A_sp_addr = uint32_t(A_sp_addr_start + (i * K + k) * DIM);
                const uint32_t B_sp_addr = uint32_t(B_sp_addr_start + (k * J + j) * DIM);
                uint32_t out_sp_addr = k == K - 1 ? C_sp_addr : kGarbageAddr;
                if (no_bias && Dp != 0 && k == K - 1)  // no_bias_new_matrix (gemmini.h:468-471)
                  out_sp_addr &= ~(1u << (ADDR_LEN - 2));
                const size_t A_cols = DIM - (k == K - 1 ? pad_K : 0), A_rows = DIM - (i == I - 1 ? pad_I : 0);
                const size_t B_cols = DIM - (j == J - 1 ? pad_J : 0), B_rows = DIM - (k == K - 1 ? pad_K : 0);
                const size_t C_cols = DIM - (j == J - 1 ? pad_J : 0), C_rows = DIM - (i == I - 1 ? pad_I : 0);
                q.push_back(cmd_preload(kGarbageAddr, out_sp_addr, unsigned(DIM), unsigned(DIM), unsigned(C_cols),
                                        unsigned(C_rows)));
                q.push_back(cmd_compute(k == 0, A_sp_addr, B_sp_addr, unsigned(A_cols), unsigned(A_rows),
                                        unsigned(B_cols), unsigned(B_rows)));
              }
            }
          if (out != 0)
            for (size_t i = 0; i < I; i++)
              for (size_t j = 0; j < J; j++) {
                const uint64_t dram = out + (i * stride_C + j) * DIM * sizeof_C;
                const uint32_t sp = uint32_t(C_sp_addr_start + (i * J + j) * DIM);
                const size_t C_cols = DIM - (j == J - 1 ? pad_J : 0), C_rows = DIM - (i == I - 1 ? pad_I : 0);
                q.push_back(cmd_mvout(dram, sp, unsigned(C_cols), unsigned(C_rows)));
              }
        }
      }
  q.push_back(cmd_fence());  // gemmini_fence (gemmini.h:847)
  return q;
}

int Engine::submit(const Matmul &m) {
  if (m.M == 0 || m.N == 0 || m.K == 0 || (m.dataflow != kOS && m.dataflow != kWS) ||
      (m.act != ACT_NONE && m.act != ACT_RELU))
    throw std::invalid_argument("Engine::submit: M, N, K > 0, dataflow OS or WS, act NONE or RELU");
  MatmulData md;
  md.m = m;
  auto alloc = [&](uint64_t bytes) {
    const uint64_t base = next_base_;
    next_base_ += (bytes + 0xfffff) & ~0xfffffull;
    return base;
  };
  md.a = alloc(uint64_t(m.M) * m.K);
  md.b = alloc(uint64_t(m.K) * m.N);
  md.d = alloc(uint64_t(m.M) * m.N * 4);
  md.c = alloc(uint64_t(m.M) * m.N * 4);
  // operands: a fixed generator, the same on every platform
  std::mt19937 rng(m.seed);
  auto in_range = [&](int lo, int span) { return int(rng() % uint32_t(span)) + lo; };
  for (uint64_t i = 0; i < uint64_t(m.M) * m.K; ++i)
    dram_.write8(md.a + i, uint8_t(int8_t(in_range(-m.range, 2 * m.range))));
  for (uint64_t i = 0; i < uint64_t(m.K) * m.N; ++i)
    dram_.write8(md.b + i, uint8_t(int8_t(in_range(-m.range, 2 * m.range))));
  if (m.bias)
    for (uint64_t i = 0; i < uint64_t(m.M) * m.N; ++i) {
      const int32_t v = in_range(-1000, 2001);
      uint8_t bytes[4];
      std::memcpy(bytes, &v, 4);
      dram_.write(md.d + 4 * i, bytes, 4);
    }
  const std::vector<Command> cmds = matmul_commands(m, opt_.cfg, md.a, md.b, md.d, md.c);
  std::string label = m.label;
  if (label.empty())
    label = std::string("matmul ") + std::to_string(m.M) + "x" + std::to_string(m.K) + "x" + std::to_string(m.N) +
            (m.dataflow == kWS ? " WS" : " OS");
  md.op = submit(cmds, label);
  Operation &o = uops_.op(md.op);
  o.is_matmul = true;
  o.workload_macs = uint64_t(m.M) * m.N * m.K;
  matmuls_.push_back(md);
  return md.op;
}

CtrlTopIn Engine::host_inputs() const { return host_->inputs(ctrl_->out_regs(), cycle()); }

CtrlTopOut Engine::step(const CtrlTopIn &in) {
  const int64_t t = cycle();
  ctrl_->set_inputs(in);
  ctrl_->eval();
  const CtrlTopOut o = ctrl_->out();
  host_->observe(in, o, t, ctrl_->execute_unit());
  if (vcd_)
    sample_vcd(in, o);
  if (hook_)
    hook_(*this, in, o);
  ctrl_->tick();
  finalized_ = false;
  return o;
}

void Engine::tick() { step(host_inputs()); }

bool Engine::done() const { return !host_->commands_left() && host_->dma_idle() && ctrl_->idle(); }

int64_t Engine::run(int64_t max_cycles) {
  while (!done()) {
    if (cycle() >= max_cycles)
      throw std::runtime_error("Engine::run: not done after " + std::to_string(max_cycles) + " cycles");
    if (!host_->error().empty())
      throw std::runtime_error("Engine::run: " + host_->error());
    if (const std::string e = ctrl_->command_path().unsupported(); !e.empty())
      throw std::runtime_error("Engine::run: " + e);
    tick();
  }
  finalize();
  return cycle();
}

void Engine::finalize() {
  if (finalized_ || !opt_.micro_ops)
    return;
  const auto &reqs = array().requests();
  std::vector<MicroOp> &all = uops_.all_mut();
  for (MicroOp &u : all) {
    if (u.request < 0 || size_t(u.request) >= reqs.size())
      continue;
    const RequestInfo &r = reqs[size_t(u.request)];
    u.first_in = r.first_in;
    u.last_in = r.last_in;
    u.first_out = r.first_out;
    u.last_out = r.last_out;
    u.rows_in = r.rows_in;
  }
  for (Operation &o : uops_.ops_mut())
    o.end = -1;
  for (const MicroOp &u : all) {
    if (u.op < 0)
      continue;
    Operation &o = uops_.op(u.op);
    if (is_command(u.kind) && u.kind != UopKind::Fence && u.parent < 0 && u.sent >= 0) {  // the core's commands
      first_cycle(o.first_sent, u.sent);
      last_cycle(o.last_sent, u.sent);
    }
    for (int64_t c : {u.sent, u.alloc, u.issued, u.started, u.popped, u.first_read, u.last_read, u.accept,
                      u.first_in, u.last_in, u.first_out, u.last_out, u.result_last, u.wb_done, u.completed,
                      u.cycle, u.done})
      last_cycle(o.end, c);
  }
  finalized_ = true;
}

std::string Engine::check() const {
  const Accounting a = accounting();
  if (std::string e = SystolicArray::check(a); !e.empty())
    return e;
  if (!opt_.micro_ops)
    return "";
  // every MAC PE-cycle belongs to a compute micro-op
  std::vector<uint64_t> macs(uops_.ops().size(), 0);
  for (const auto &kv : a.per_uop) {
    if (!kv.second.macs())
      continue;
    if (!uops_.has(kv.first))
      return "MAC PE-cycles without a micro-op";
    const MicroOp &u = uops_.at(kv.first);
    if (u.kind != UopKind::ComputePreloaded && u.kind != UopKind::ComputeAccumulated)
      return "MAC PE-cycles of a micro-op that is not a compute (" + std::to_string(u.id) + ")";
    if (u.op < 0 || a.per_op.count(u.op) == 0)
      return "a compute micro-op without its operation";
    macs[size_t(u.op)] += kv.second.macs();
  }
  for (const auto &kv : a.per_op)
    if (kv.first >= 0 && kv.second.macs() != macs[size_t(kv.first)])
      return "per-operation MACs differ from their micro-ops' (op " + std::to_string(kv.first) + ")";
  // a matmul whose dimensions are multiples of DIM: MAC PE-cycles = M N K exactly
  const unsigned dim = opt_.cfg.dim();
  for (const MatmulData &md : matmuls_) {
    const Matmul &m = md.m;
    if (m.M % dim || m.N % dim || m.K % dim)
      continue;
    const auto it = a.per_op.find(md.op);
    const uint64_t got = it == a.per_op.end() ? 0 : it->second.macs();
    if (got != uint64_t(m.M) * m.N * m.K)
      return "matmul op " + std::to_string(md.op) + ": " + std::to_string(got) + " MAC PE-cycles, " +
             std::to_string(uint64_t(m.M) * m.N * m.K) + " MACs";
  }
  return "";
}

std::vector<int32_t> Engine::result(int op) const {
  for (const MatmulData &md : matmuls_)
    if (md.op == op) {
      std::vector<int32_t> c(size_t(md.m.M) * md.m.N);
      for (size_t i = 0; i < c.size(); ++i) {
        if (md.m.full_c) {
          uint8_t bytes[4];
          dram_.read(md.c + 4 * i, bytes, 4);
          std::memcpy(&c[i], bytes, 4);
        } else {
          c[i] = int8_t(dram_.read8(md.c + i));
        }
      }
      return c;
    }
  throw std::invalid_argument("Engine::result: not a matmul");
}

std::vector<int32_t> Engine::reference(int op) const {
  for (const MatmulData &md : matmuls_)
    if (md.op == op) {
      const Matmul &m = md.m;
      std::vector<int32_t> c(size_t(m.M) * m.N);
      for (unsigned i = 0; i < m.M; ++i)
        for (unsigned j = 0; j < m.N; ++j) {
          int64_t s = 0;
          if (m.bias) {
            uint8_t bytes[4];
            int32_t v;
            dram_.read(md.d + 4 * (uint64_t(i) * m.N + j), bytes, 4);
            std::memcpy(&v, bytes, 4);
            s = v;
          }
          for (unsigned k = 0; k < m.K; ++k)
            s += int64_t(int8_t(dram_.read8(md.a + uint64_t(i) * m.K + k))) *
                 int64_t(int8_t(dram_.read8(md.b + uint64_t(k) * m.N + j)));
          // full_c: the accumulator's raw rows (read_full_acc_row bypasses AccumulatorScale's
          // activation and scale, AccumulatorScale.scala); else ACC_SCALE (1.0), clip, activation
          // (gemmini.h scale_and_sat)
          int64_t v = m.full_c ? int64_t(int32_t(s)) : int64_t(clip8(s));
          if (!m.full_c && m.act == ACT_RELU && v < 0)
            v = 0;
          c[size_t(i) * m.N + j] = int32_t(v);
        }
      return c;
    }
  throw std::invalid_argument("Engine::reference: not a matmul");
}

// ------------------------------------------------------------------ VCD
void Engine::declare_vcd() {
  vcd_ = std::make_unique<Vcd>(opt_.vcd_path);
  VcdWriter &w = vcd_->w;
  vcd_->clock = w.add("", "clock", 1);
  vcd_->cycle = w.add("", "cycle", 32);
  for (const char *n : {"cmd_valid", "cmd_ready", "busy", "loop_matmul_busy", "ld_valid", "ld_ready", "st_valid",
                        "st_ready", "ex_valid", "ex_ready", "ex_completed_valid", "ld_completed_valid",
                        "st_completed_valid", "dma_sp_en", "dma_sp_taken", "dma_acc_en", "dma_acc_taken",
                        "ex_busy"})
    vcd_->ports.push_back(w.add("ports", n, 1));
  for (const char *n : {"cmd_funct", "ex_funct", "ld_funct", "st_funct"})
    vcd_->ports.push_back(w.add("ports", n, 7));
  for (const char *n : {"ex_rob_id", "ex_completed_bits", "ld_rob_id", "st_rob_id"})
    vcd_->ports.push_back(w.add("ports", n, 6));
  for (const char *n : {"sp_read_valid", "sp_read_ready", "sp_resp_ready", "sp_write_en"})
    vcd_->ports.push_back(w.add("ports", n, 4));
  for (const char *n : {"acc_read_valid", "acc_write_valid"})
    vcd_->ports.push_back(w.add("ports", n, 2));
  vcd_->ex_state = w.add("ex", "control_state", 2);
  vcd_->uop_head = w.add("uop", "ex_queue_head", 32);
  vcd_->uop_pass = w.add("uop", "pass", 32);
  vcd_->uop_in = w.add("uop", "feed", 32);
  vcd_->op_in = w.add("uop", "feed_op", 32);
  vcd_->uop_out = w.add("uop", "result", 32);
}

void Engine::sample_vcd(const CtrlTopIn &in, const CtrlTopOut &o) {
  Vcd &v = *vcd_;
  VcdWriter &w = v.w;
  const uint64_t t = uint64_t(cycle()) + 4;  // the four reset cycles come first
  w.set(v.clock, 1);
  w.set(v.cycle, uint64_t(cycle()));
  const uint64_t bits[] = {in.cmd_valid, o.cmd_ready, o.busy, o.loop_matmul_busy, o.ld.valid, in.ld_ready,
                           o.st.valid, in.st_ready, o.ex.valid, o.ex_ready, o.ex_completed_valid,
                           in.ld_completed_valid, in.st_completed_valid, in.dma_sp.valid, o.bank.dma_sp_taken,
                           in.dma_acc.valid, o.bank.dma_acc_taken, o.ex_busy};
  size_t k = 0;
  for (uint64_t b : bits)
    w.set(v.ports[k++], b);
  for (uint64_t f : {uint64_t(in.cmd.funct), uint64_t(o.ex.funct), uint64_t(o.ld.funct), uint64_t(o.st.funct)})
    w.set(v.ports[k++], f);
  for (uint64_t r : {uint64_t(o.ex.rob_id), uint64_t(o.ex_completed_bits), uint64_t(o.ld.rob_id), uint64_t(o.st.rob_id)})
    w.set(v.ports[k++], r);
  auto pack = [](const std::vector<bool> &x) {
    uint64_t r = 0;
    for (size_t i = 0; i < x.size(); ++i)
      r |= uint64_t(x[i]) << i;
    return r;
  };
  uint64_t rv = 0, we = 0, av = 0, aw = 0;
  for (size_t i = 0; i < o.bank.sp_read.size(); ++i) {
    rv |= uint64_t(o.bank.sp_read[i].valid) << i;
    we |= uint64_t(o.bank.sp_write[i].en) << i;
  }
  for (size_t i = 0; i < o.bank.acc_read.size(); ++i) {
    av |= uint64_t(o.bank.acc_read[i].valid) << i;
    aw |= uint64_t(o.bank.acc_write[i].valid) << i;
  }
  w.set(v.ports[k++], rv);
  w.set(v.ports[k++], pack(o.bank.sp_read_ready));
  w.set(v.ports[k++], pack(o.bank.sp_resp_ready));
  w.set(v.ports[k++], we);
  w.set(v.ports[k++], av);
  w.set(v.ports[k++], aw);
  const ExecuteController &ex = ctrl_->execute_unit().ex();
  w.set(v.ex_state, ex.state());
  // micro-op ids (all ones: none)
  w.set_signed(v.uop_head, ex.head_uop());
  w.set_signed(v.uop_pass, ex.pass_uop());
  w.set_signed(v.uop_in, ex.feed_uop());
  w.set_signed(v.op_in, ex.feed_op());
  w.set_signed(v.uop_out, ex.result_uop());
  w.commit(10 * t);
  w.set(v.clock, 0);
  w.commit(10 * t + 5);
}

}  // namespace systolique
