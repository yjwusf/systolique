#include "reference.h"

#include "systolique/arith.h"

#include <algorithm>
#include <cstdio>
#include <map>

namespace systolique {

using namespace arith;

Matrix matmul_add(const Matrix &a, const Matrix &b, const Matrix &d) {
  const size_t n = a.size(), m = b[0].size(), k = b.size();
  Matrix c(n, std::vector<int64_t>(m));
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < m; ++j) {
      int64_t s = d[i][j];
      for (size_t x = 0; x < k; ++x) s += a[i][x] * b[x][j];
      c[i][j] = s;
    }
  return c;
}

namespace {

Matrix reverse_rows(Matrix m) {
  std::reverse(m.begin(), m.end());
  return m;
}

}  // namespace

MatmulCase os_case(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned K,
                   bool a_via_transposer, bool b_transposed, unsigned shift) {
  const unsigned n = cfg.block_size(), w = cfg.in_bits;
  std::vector<Matrix> A, B, D;
  for (unsigned k = 0; k < K; ++k) {
    A.push_back(random_matrix(rng, n, n, w));
    B.push_back(random_matrix(rng, n, n, w));
    D.push_back(random_matrix(rng, n, n, w));
  }
  const Matrix Z = zero_matrix(n, n);
  MatmulCase c;
  // Request j preloads D_j and computes A_{j-1} B_{j-1}; operands through the transposer go in
  // one request early.
  for (unsigned j = 0; j <= K; ++j) {
    const Matrix d = j < K ? reverse_rows(D[j]) : Z;
    Matrix a = Z, b = Z;
    if (a_via_transposer) {
      if (j < K) a = A[j];
    } else if (j > 0) {
      a = transpose(A[j - 1]);
    }
    if (b_transposed) {
      if (j < K) b = transpose(B[j]);
    } else if (j > 0) {
      b = B[j - 1];
    }
    Op op = make_op(cfg, Dataflow::OS, 1, shift, a, b, d, j);
    op.req.a_transpose = !a_via_transposer;  // a_is_from_transposer = !a_transpose in OS
    op.req.bd_transpose = b_transposed;
    c.ops.push_back(op);
  }
  Op flush = make_flush(cfg, Dataflow::OS, 1);
  flush.req.shift = shift;
  c.ops.push_back(flush);
  // Matmul k: c = D + sum a*b wrapping at the MacUnit's output width (PE.scala:23), then the
  // rounding shift and saturation on the way out (PE.scala:104, 111); bottom row first.
  for (unsigned k = 0; k < K; ++k) {
    Matrix r = matmul_add(A[k], B[k], D[k]);
    for (auto &row : r)
      for (auto &v : row)
        v = clip(round_shift(sext(v, cfg.out_bits), shift, cfg.c_bits(), cfg.shift_bits()),
                 cfg.out_bits);
    c.expected.push_back({k, reverse_rows(r)});
  }
  return c;
}

MatmulCase ws_case(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned K, bool a_transposed,
                   bool w_transposed) {
  const unsigned n = cfg.block_size(), w = cfg.in_bits;
  std::vector<Matrix> A, B, W;
  for (unsigned k = 0; k < K; ++k) {
    A.push_back(random_matrix(rng, n, n, w));
    B.push_back(random_matrix(rng, n, n, w));
    W.push_back(random_matrix(rng, n, n, w));
  }
  const Matrix Z = zero_matrix(n, n);
  MatmulCase c;
  // W_k is preloaded in request k + lead (through the transposer its rows go in one request
  // earlier), A_k streams in request k + lead + 1 (through the transposer, one earlier).
  const unsigned lead = w_transposed ? 1 : 0;
  for (unsigned j = 0; j < K + lead + 1; ++j) {
    Matrix d = Z, a = Z, b = Z;
    if (j < K) d = w_transposed ? reverse_rows(transpose(W[j])) : reverse_rows(W[j]);
    const int ka = a_transposed ? int(j) - int(lead) : int(j) - int(lead) - 1;
    if (ka >= 0 && ka < int(K)) a = a_transposed ? transpose(A[ka]) : A[ka];
    const int kb = int(j) - int(lead) - 1;
    if (kb >= 0 && kb < int(K)) b = B[kb];
    Op op = make_op(cfg, Dataflow::WS, 1, 0, a, b, d, j);
    op.req.a_transpose = a_transposed;
    op.req.bd_transpose = w_transposed;
    c.ops.push_back(op);
  }
  for (unsigned k = 0; k < K; ++k) {
    Matrix r = matmul_add(A[k], W[k], B[k]);
    for (auto &row : r)
      for (auto &v : row) v = sext(v, cfg.out_bits);  // out_b wraps at the output width
    c.expected.push_back({k + lead, r});
  }
  return c;
}

std::string check_responses(const ArrayConfig &cfg, const MatmulCase &c,
                            const std::vector<Frame> &rows, unsigned *checked_values) {
  std::map<unsigned, std::vector<std::vector<int64_t>>> got;
  for (const Frame &f : rows) {
    MwdOut o;
    frame_to_mwd_out(cfg, f, o);
    if (f[0].get(0, 1) || !o.resp_valid || !o.resp_tag_valid) continue;
    got[o.resp_tag_id].push_back(o.resp_data);
  }
  unsigned values = 0;
  for (const auto &e : c.expected) {
    const auto it = got.find(e.tag);
    if (it == got.end()) return "no response rows with tag " + std::to_string(e.tag);
    if (it->second.size() != e.rows.size())
      return "tag " + std::to_string(e.tag) + ": " + std::to_string(it->second.size()) +
             " response rows, expected " + std::to_string(e.rows.size());
    for (size_t r = 0; r < e.rows.size(); ++r)
      for (size_t k = 0; k < e.rows[r].size(); ++k) {
        ++values;
        if (it->second[r][k] != e.rows[r][k]) {
          char buf[200];
          std::snprintf(buf, sizeof buf, "tag %u row %zu lane %zu: got %lld, expected %lld",
                        e.tag, r, k, (long long)it->second[r][k], (long long)e.rows[r][k]);
          return buf;
        }
      }
  }
  if (checked_values) *checked_values = values;
  return "";
}

}  // namespace systolique
