// Matmul sequences as Gemmini's ExecuteController issues them to MeshWithDelays, with the
// results a plain C++ matmul gives, and the check of the array's responses against them.
#pragma once

#include "stimulus.h"

#include <string>
#include <vector>

namespace systolique {

// Plain C++: C = A * B + D (all DIM x DIM).
Matrix matmul_add(const Matrix &a, const Matrix &b, const Matrix &d);

// The expected response rows for the request tagged `tag`, in output order.
struct ExpectedTile {
  unsigned tag = 0;
  Matrix rows;
};

struct MatmulCase {
  std::vector<Op> ops;
  std::vector<ExpectedTile> expected;
};

// Output stationary, K matmuls C_k = A_k B_k + D_k (rounding shift `shift`, saturated to the
// output width). Request j preloads D_j (rows bottom-up, ExecuteController.scala:253) while the
// array computes A_{j-1} B_{j-1}; the result of matmul k leaves during request k+2 with the tag
// of request k (matmul_id + 3, MeshWithDelays.scala:219), bottom row first. A is given transposed
// (a_transpose: no transposer) unless a_via_transposer; then A_j goes in with the preload.
// b_transposed gives B transposed through the transposer (bd_transpose).
MatmulCase os_case(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned K,
                   bool a_via_transposer, bool b_transposed, unsigned shift);

// Weight stationary, K matmuls C_k = A_k W_k + B_k (wrapping at the output width). Request j
// preloads W_j through d (rows bottom-up) while A_{j-1} streams through W_{j-1} with the bias
// B_{j-1}; the result leaves during the same request, top row first, with the tag of the request
// before (matmul_id + 2). a_transposed / w_transposed give A / W transposed through the
// transposer (a_transpose / bd_transpose).
MatmulCase ws_case(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned K, bool a_transposed,
                   bool w_transposed);

// Checks the responses recorded in `rows` (MeshWithDelaysTop frames) against `c.expected`:
// the valid response rows carrying each expected tag, in order. Returns "" or the first error.
std::string check_responses(const ArrayConfig &cfg, const MatmulCase &c,
                            const std::vector<Frame> &rows, unsigned *checked_values = nullptr);

}  // namespace systolique
