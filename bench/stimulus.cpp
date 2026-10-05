#include "stimulus.h"

#include "systolique/arith.h"
#include "reference.h"

namespace systolique {

Matrix random_matrix(std::mt19937_64 &rng, unsigned rows, unsigned cols, unsigned bits) {
  Matrix m(rows, std::vector<int64_t>(cols));
  for (auto &r : m)
    for (auto &v : r) v = arith::sext(int64_t(rng()), bits);
  return m;
}

Matrix zero_matrix(unsigned rows, unsigned cols) {
  return Matrix(rows, std::vector<int64_t>(cols, 0));
}

Matrix transpose(const Matrix &m) {
  Matrix t(m.empty() ? 0 : m[0].size(), std::vector<int64_t>(m.size()));
  for (size_t i = 0; i < m.size(); ++i)
    for (size_t j = 0; j < m[i].size(); ++j) t[j][i] = m[i][j];
  return t;
}

Op make_op(const ArrayConfig &cfg, Dataflow df, unsigned propagate, unsigned shift,
           const Matrix &a, const Matrix &b, const Matrix &d, unsigned tag) {
  Op op;
  op.req.dataflow = unsigned(df);
  op.req.propagate = propagate;
  op.req.shift = shift;
  op.req.total_rows = unsigned(a.size());
  op.req.tag_valid = 1;
  op.req.tag_id = unsigned(arith::zext(tag, cfg.tag_bits));
  op.a = a;
  op.b = b;
  op.d = d;
  return op;
}

Op make_flush(const ArrayConfig &cfg, Dataflow df, unsigned propagate, unsigned flush) {
  Op op;
  op.req.dataflow = unsigned(df);
  op.req.propagate = propagate;
  op.req.total_rows = cfg.block_size();  // ExecuteController.scala:200
  op.req.flush = flush;
  return op;
}

std::vector<Op> random_ops(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned n) {
  std::vector<Op> ops;
  const unsigned dim = cfg.block_size();
  auto coin = [&](double p) { return std::uniform_real_distribution<double>(0, 1)(rng) < p; };
  Dataflow last_df = Dataflow::WS;
  for (unsigned k = 0; k < n; ++k) {
    Dataflow df = cfg.dataflow == Dataflow::BOTH ? (coin(0.5) ? Dataflow::OS : Dataflow::WS)
                                                 : cfg.dataflow;
    if (cfg.dataflow != Dataflow::BOTH && coin(0.05))  // the other value of the bit, rarely
      df = df == Dataflow::OS ? Dataflow::WS : Dataflow::OS;
    const unsigned prop = coin(0.7);
    if (df == Dataflow::WS && last_df == Dataflow::OS) ops.push_back(make_flush(cfg, last_df, 1));
    if (coin(0.15)) {
      Op f = make_flush(cfg, df, prop, coin(0.2) ? 2 : 1);
      f.delay = coin(0.3) ? unsigned(rng() % 4) : 0;
      ops.push_back(f);
      continue;
    }
    const unsigned rows = coin(0.7) ? dim : 1 + unsigned(rng() % dim);
    const unsigned shift = coin(0.5) ? 0 : unsigned(rng() % 12);
    Op op = make_op(cfg, df, prop, shift, random_matrix(rng, rows, dim, cfg.in_bits),
                    random_matrix(rng, rows, dim, cfg.in_bits),
                    random_matrix(rng, rows, dim, cfg.in_bits), unsigned(rng()));
    op.req.a_transpose = coin(0.3);
    op.req.bd_transpose = coin(0.3);
    op.req.tag_valid = coin(0.9);
    op.delay = coin(0.3) ? unsigned(rng() % 4) : 0;
    ops.push_back(op);
    last_df = df;
  }
  return ops;
}

MwdDriver::MwdDriver(const ArrayConfig &cfg, std::vector<Op> ops, DriverOptions opt)
    : cfg_(cfg), ops_(std::move(ops)), opt_(opt), rng_(opt.seed) {
  tail_left_ = opt.tail ? opt.tail
                        : 4 * cfg.block_size() * (cfg.tile_latency + 1) + cfg.output_delay + 16;
}

bool MwdDriver::next(unsigned, const Frame &out, Frame &in) {
  MwdOut o;
  frame_to_mwd_out(cfg_, out, o);
  MwdIn x;
  x.resize(cfg_);
  if (draining_) {
    mwd_in_to_frame(cfg_, x, in);
    return tail_left_-- > 0;
  }
  bool req_fire = false;
  if (next_req_ < ops_.size()) {
    if (!waiting_) {
      wait_ = ops_[next_req_].delay;
      waiting_ = true;
    }
    if (wait_ > 0) {
      --wait_;
    } else {
      x.req_valid = 1;
      x.req = ops_[next_req_].req;
      req_fire = o.req_ready;
    }
  }
  const size_t fired = next_req_ + (req_fire ? 1 : 0);
  const unsigned ready[3] = {o.a_ready, o.b_ready, o.d_ready};
  unsigned *valid[3] = {&x.a_valid, &x.b_valid, &x.d_valid};
  std::vector<int64_t> *bits[3] = {&x.a, &x.b, &x.d};
  for (unsigned ch = 0; ch < 3; ++ch) {
    Chan &c = chan_[ch];
    const bool bubble = std::uniform_real_distribution<double>(0, 1)(rng_) < opt_.bubble;
    while (c.op < fired && ops_[c.op].a.empty()) {  // flushes carry no rows
      ++c.op;
      c.row = 0;
    }
    if (c.op >= fired || bubble) continue;
    const Op &op = ops_[c.op];
    const Matrix &m = ch == 0 ? op.a : (ch == 1 ? op.b : op.d);
    *valid[ch] = 1;
    *bits[ch] = m[c.row];
    if (ready[ch] && ++c.row == m.size()) {
      ++c.op;
      c.row = 0;
    }
  }
  const bool any_fire = req_fire || (x.a_valid && o.a_ready) || (x.b_valid && o.b_ready) ||
                        (x.d_valid && o.d_ready);
  idle_ = any_fire ? 0 : idle_ + 1;
  if (idle_ > 50 * cfg_.block_size() * (cfg_.tile_latency + 1) + 200) {
    error_ = "stalled: no handshake for " + std::to_string(idle_) + " cycles (request " +
             std::to_string(next_req_) + " of " + std::to_string(ops_.size()) + ")";
    return false;
  }
  if (req_fire) {
    ++next_req_;
    waiting_ = false;
  }
  if (next_req_ == ops_.size()) {
    bool rows_left = false;
    for (auto &c : chan_) {
      while (c.op < ops_.size() && ops_[c.op].a.empty()) ++c.op;
      rows_left |= c.op < ops_.size();
    }
    draining_ = !rows_left;
  }
  mwd_in_to_frame(cfg_, x, in);
  return true;
}

MeshRandom::MeshRandom(const ArrayConfig &cfg, unsigned cycles, double valid_p, uint64_t seed)
    : cfg_(cfg), cycles_(cycles), valid_p_(valid_p), rng_(seed) {}

bool MeshRandom::next(unsigned cycle, const Frame &, Frame &in) {
  if (cycle >= cycles_) return false;
  MeshIn x;
  x.resize(cfg_);
  auto coin = [&](double p) { return std::uniform_real_distribution<double>(0, 1)(rng_) < p; };
  for (auto &v : x.a) v = arith::sext(int64_t(rng_()), cfg_.in_bits);
  for (unsigned k = 0; k < cfg_.cols(); ++k) {
    x.b[k] = arith::sext(int64_t(rng_()), cfg_.in_bits);
    x.d[k] = arith::sext(int64_t(rng_()), cfg_.in_bits);
    x.control[k].dataflow = coin(0.5);
    x.control[k].propagate = coin(0.5);
    x.control[k].shift = unsigned(rng_() % (1u << cfg_.shift_bits()));
    x.id[k] = unsigned(rng_() % (1u << cfg_.id_bits()));
    x.last[k] = coin(0.1);
    x.valid[k] = coin(valid_p_);
  }
  mesh_in_to_frame(cfg_, x, in);
  return true;
}

std::vector<TestDef> test_catalog(const ArrayConfig &cfg) {
  std::vector<TestDef> t;
  const bool os = cfg.dataflow != Dataflow::WS, ws = cfg.dataflow != Dataflow::OS;
  const Dataflow df0 = os ? Dataflow::OS : Dataflow::WS;
  const unsigned mesh_cycles = cfg.rows() >= 16 ? 400 : 1500;
  auto add = [&](std::string name, std::string what,
                 std::function<std::vector<Op>(const ArrayConfig &, std::mt19937_64 &)> ops,
                 double bubble, uint64_t seed) {
    t.push_back({name, Top::MeshWithDelays, what,
                 [ops, bubble, seed](const ArrayConfig &c) -> std::unique_ptr<Stimulus> {
                   std::mt19937_64 rng(seed);
                   DriverOptions o;
                   o.bubble = bubble;
                   o.seed = seed ^ 0x5bd1e995;
                   return std::make_unique<MwdDriver>(c, ops(c, rng), o);
                 },
                 {}});
  };
  t.push_back({"mesh_random", Top::Mesh, "Mesh alone: random values on every input lane, valid 70%",
               [mesh_cycles](const ArrayConfig &c) {
                 return std::make_unique<MeshRandom>(c, mesh_cycles, 0.7, 11);
               },
               {}});
  t.push_back({"mesh_sparse", Top::Mesh, "Mesh alone: random values, valid 20%",
               [mesh_cycles](const ArrayConfig &c) {
                 return std::make_unique<MeshRandom>(c, mesh_cycles / 2, 0.2, 12);
               },
               {}});
  // Matmul tests: Gemmini's request sequences with known results (reference.h).
  auto add_mm = [&](std::string name, std::string what,
                    std::function<MatmulCase(const ArrayConfig &, std::mt19937_64 &)> mk,
                    uint64_t seed) {
    auto matmul = [mk, seed](const ArrayConfig &c) {
      std::mt19937_64 rng(seed);
      return mk(c, rng);
    };
    t.push_back({name, Top::MeshWithDelays, what,
                 [matmul](const ArrayConfig &c) -> std::unique_ptr<Stimulus> {
                   return std::make_unique<MwdDriver>(c, matmul(c).ops, DriverOptions{});
                 },
                 matmul});
  };
  if (os) {
    add_mm("os_single", "OS: one matmul C = A*B + D (preload D, compute, flush), A given transposed",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return os_case(c, r, 1, false, false, 0); },
           101);
    add_mm("os_back_to_back", "OS: 4 matmuls, each compute overlapped with the next preload",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return os_case(c, r, 4, false, false, 0); },
           102);
    add_mm("os_a_transposer", "OS: 3 matmuls, A through the transposer (fed with the preload)",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return os_case(c, r, 3, true, false, 0); },
           103);
    add_mm("os_b_transposed", "OS: 3 matmuls, B given transposed, through the transposer",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return os_case(c, r, 3, false, true, 0); },
           104);
    add_mm("os_shift", "OS: 3 matmuls, outputs rounded and shifted right by 3",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return os_case(c, r, 3, false, false, 3); },
           105);
  }
  if (ws) {
    add_mm("ws_single", "WS: one matmul C = A*W + B (preload W, stream A with bias B)",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return ws_case(c, r, 1, false, false); },
           201);
    add_mm("ws_back_to_back", "WS: 4 matmuls, each stream overlapped with the next preload",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return ws_case(c, r, 4, false, false); },
           202);
    add_mm("ws_a_transposed", "WS: 3 matmuls, A given transposed, through the transposer",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return ws_case(c, r, 3, true, false); },
           203);
    add_mm("ws_w_transposed", "WS: 3 matmuls, W given transposed, through the transposer",
           [](const ArrayConfig &c, std::mt19937_64 &r) { return ws_case(c, r, 3, false, true); },
           204);
  }
  if (os && ws)
    add("df_switch", "OS and WS requests alternating (a flush before each WS one), with and without flips",
        [](const ArrayConfig &c, std::mt19937_64 &r) {
          std::vector<Op> ops;
          const unsigned n = c.block_size();
          for (unsigned k = 0; k < 8; ++k) {
            const Dataflow df = k % 2 ? Dataflow::WS : Dataflow::OS;
            if (df == Dataflow::WS) ops.push_back(make_flush(c, Dataflow::OS, 1));
            ops.push_back(make_op(c, df, k % 3 != 2, 0, random_matrix(r, n, n, c.in_bits),
                                  random_matrix(r, n, n, c.in_bits),
                                  random_matrix(r, n, n, c.in_bits), k));
          }
          ops.push_back(make_flush(c, Dataflow::OS, 1));
          return ops;
        },
        0, 301);
  if (os && ws) {
    // MeshWithDelays.scala:219: an OS tag waits for matmul id +3, a WS tag for +2; OS then WS
    // without a flush makes both wait for the same id, the WS one misses it, the tag queue
    // fills and io.req.ready stays low (the ExecuteController flushes on a dataflow change).
    add("os_ws_no_flush", "OS and WS requests alternating without a flush: the tag queue "
                          "deadlocks (the test passes when the watchdog stops it)",
        [](const ArrayConfig &c, std::mt19937_64 &r) {
          std::vector<Op> ops;
          const unsigned n = c.block_size();
          for (unsigned k = 0; k < 10; ++k)
            ops.push_back(make_op(c, k % 2 ? Dataflow::WS : Dataflow::OS, 1, 0,
                                  random_matrix(r, n, n, c.in_bits),
                                  random_matrix(r, n, n, c.in_bits),
                                  random_matrix(r, n, n, c.in_bits), k));
          return ops;
        },
        0, 306);
    t.back().expect_stall = true;
  }
  add("accumulate", "requests without a propagate flip accumulate onto the same registers",
      [df0](const ArrayConfig &c, std::mt19937_64 &r) {
        std::vector<Op> ops;
        const unsigned n = c.block_size();
        for (unsigned k = 0; k < 6; ++k)
          ops.push_back(make_op(c, df0, k == 0 || k == 3, 0, random_matrix(r, n, n, c.in_bits),
                                random_matrix(r, n, n, c.in_bits),
                                random_matrix(r, n, n, c.in_bits), k));
        ops.push_back(make_flush(c, df0, 1));
        return ops;
      },
      0, 302);
  add("partial_rows", "requests with total_rows below DIM",
      [df0](const ArrayConfig &c, std::mt19937_64 &r) {
        std::vector<Op> ops;
        const unsigned n = c.block_size();
        for (unsigned k = 0; k < 6; ++k) {
          const unsigned rows = 1 + (k * 5 + 1) % n;
          ops.push_back(make_op(c, df0, 1, 0, random_matrix(r, rows, n, c.in_bits),
                                random_matrix(r, rows, n, c.in_bits),
                                random_matrix(r, rows, n, c.in_bits), k));
        }
        ops.push_back(make_flush(c, df0, 1));
        return ops;
      },
      0, 303);
  add("flush2", "a flush of 2 (two passes) between matmuls",
      [df0](const ArrayConfig &c, std::mt19937_64 &r) {
        std::vector<Op> ops;
        const unsigned n = c.block_size();
        for (unsigned k = 0; k < 4; ++k) {
          ops.push_back(make_op(c, df0, 1, 0, random_matrix(r, n, n, c.in_bits),
                                random_matrix(r, n, n, c.in_bits),
                                random_matrix(r, n, n, c.in_bits), k));
          if (k == 1) ops.push_back(make_flush(c, df0, 1, 2));
        }
        ops.push_back(make_flush(c, df0, 0));
        return ops;
      },
      0, 304);
  add("bubbles", "random valid gaps on a, b, d and delayed requests",
      [](const ArrayConfig &c, std::mt19937_64 &r) {
        std::vector<Op> ops = random_ops(c, r, 10);
        for (auto &op : ops) op.delay = unsigned(r() % 6);
        return ops;
      },
      0.35, 305);
  add("random_1", "random requests (dataflow, flips, shifts, transposes, rows, flushes)",
      [](const ArrayConfig &c, std::mt19937_64 &r) { return random_ops(c, r, 40); }, 0.15, 401);
  add("random_2", "random requests, different seed, no gaps",
      [](const ArrayConfig &c, std::mt19937_64 &r) { return random_ops(c, r, 40); }, 0.0, 402);
  if (cfg.block_size() <= 8)
    add("random_long", "random requests, 160 of them",
        [](const ArrayConfig &c, std::mt19937_64 &r) { return random_ops(c, r, 160); }, 0.1, 403);
  return t;
}

std::vector<TestDef> soak_tests(const ArrayConfig &cfg, unsigned n) {
  std::vector<TestDef> t;
  const unsigned mesh_cycles = cfg.rows() >= 16 ? 200 : 600;
  for (unsigned k = 0; k < n; ++k) {
    const uint64_t seed = 1000 + k;
    t.push_back({"soak_mesh_" + std::to_string(k), Top::Mesh, "random Mesh inputs",
                 [mesh_cycles, seed](const ArrayConfig &c) {
                   return std::make_unique<MeshRandom>(c, mesh_cycles, 0.3 + 0.05 * (seed % 10),
                                                       seed);
                 },
                 {}});
    t.push_back({"soak_mwd_" + std::to_string(k), Top::MeshWithDelays, "random requests",
                 [seed](const ArrayConfig &c) -> std::unique_ptr<Stimulus> {
                   std::mt19937_64 rng(seed);
                   DriverOptions o;
                   o.bubble = 0.05 * double(seed % 6);
                   o.seed = seed * 7 + 1;
                   return std::make_unique<MwdDriver>(c, random_ops(c, rng, 60), o);
                 },
                 {}});
  }
  return t;
}

const TestDef *find_test(const std::vector<TestDef> &tests, const std::string &name) {
  for (const auto &t : tests)
    if (t.name == name) return &t;
  return nullptr;
}

}  // namespace systolique
