// Stimuli for the benches and the test catalog. A Stimulus decides the inputs of each cycle from
// that cycle's outputs (the ready signals), like Gemmini's ExecuteController
// (ExecuteController.scala) does; it is deterministic for a seed (std::mt19937_64 and
// std::uniform_real_distribution: the stored traces were generated with libc++, and the
// regeneration test checks that this still reproduces them).
//
// The test catalog (test_catalog) is what the stored RTL traces were recorded from: changing a
// stimulus here breaks the regeneration check until the traces are re-recorded (rtl/README.md).
#pragma once

#include "systolique/config.h"
#include "systolique/types.h"
#include "ports.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace systolique {

class Stimulus {
 public:
  virtual ~Stimulus() = default;
  // Inputs (reset excluded) for this cycle; false once the test is over.
  virtual bool next(unsigned cycle, const Frame &out, Frame &in) = 0;
  // Why the stimulus gave up ("" if it ran to its end).
  virtual std::string error() const { return ""; }
};

using Matrix = std::vector<std::vector<int64_t>>;  // [row][col]

// One request to MeshWithDelays with the rows it consumes (empty for a flush).
struct Op {
  MwdReq req;
  Matrix a, b, d;  // req.total_rows rows each, or none for a flush
  unsigned delay = 0;  // idle cycles before the request is offered
};

struct DriverOptions {
  double bubble = 0.0;  // probability that a channel holds its valid low in a cycle
  unsigned tail = 0;    // cycles to run after the last row (0: enough to drain the array)
  uint64_t seed = 1;
};

// Feeds a list of Ops: offers op k's request after op k-1's, and the rows of op k on a, b and d
// once its request fires (in the same cycle at the earliest), so no buffered row is overwritten.
class MwdDriver : public Stimulus {
 public:
  MwdDriver(const ArrayConfig &cfg, std::vector<Op> ops, DriverOptions opt);
  bool next(unsigned cycle, const Frame &out, Frame &in) override;
  std::string error() const override { return error_; }
  const std::vector<Op> &ops() const { return ops_; }

 private:
  ArrayConfig cfg_;
  std::vector<Op> ops_;
  DriverOptions opt_;
  std::mt19937_64 rng_;
  size_t next_req_ = 0;      // first op whose request has not fired
  unsigned wait_ = 0;        // idle cycles left before offering it
  bool waiting_ = false;
  struct Chan {
    size_t op = 0, row = 0;  // next row to send
  } chan_[3];
  unsigned tail_left_ = 0;
  bool draining_ = false;
  unsigned idle_ = 0;  // cycles without any handshake (watchdog)
  std::string error_;
};

// Random values in every input lane of the Mesh, every cycle.
class MeshRandom : public Stimulus {
 public:
  MeshRandom(const ArrayConfig &cfg, unsigned cycles, double valid_p, uint64_t seed);
  bool next(unsigned cycle, const Frame &out, Frame &in) override;

 private:
  ArrayConfig cfg_;
  unsigned cycles_;
  double valid_p_;
  std::mt19937_64 rng_;
};

struct MatmulCase;

struct TestDef {
  std::string name;
  Top top;
  std::string what;  // one line for the docs
  std::function<std::unique_ptr<Stimulus>(const ArrayConfig &)> make;
  // For the matmul tests: the requests and the plain C++ results (reference.h), else empty.
  std::function<MatmulCase(const ArrayConfig &)> matmul;
  // The test shows a deadlock: it passes only if the driver's watchdog stops it.
  bool expect_stall = false;
};

// The tests run on a configuration (directed, then random); all are stored as reference traces.
std::vector<TestDef> test_catalog(const ArrayConfig &cfg);
const TestDef *find_test(const std::vector<TestDef> &tests, const std::string &name);
// Extra random tests for the live lockstep bench only (no stored trace): n seeds, each a Mesh
// run and a MeshWithDelays run of random requests with random gaps.
std::vector<TestDef> soak_tests(const ArrayConfig &cfg, unsigned n);

// Builders shared with the functional tests.
Matrix random_matrix(std::mt19937_64 &rng, unsigned rows, unsigned cols, unsigned bits);
Matrix zero_matrix(unsigned rows, unsigned cols);
Matrix transpose(const Matrix &m);
Op make_op(const ArrayConfig &cfg, Dataflow df, unsigned propagate, unsigned shift,
           const Matrix &a, const Matrix &b, const Matrix &d, unsigned tag);
Op make_flush(const ArrayConfig &cfg, Dataflow df, unsigned propagate, unsigned flush = 1);
// Random requests. An OS request followed by a WS one is separated by a flush, as the
// ExecuteController does on a dataflow change (ExecuteController.scala:623-630): their tags
// would both wait for the same matmul id (+3 and +2, MeshWithDelays.scala:219), the second
// would miss it and the tag queue could fill up for good.
std::vector<Op> random_ops(const ArrayConfig &cfg, std::mt19937_64 &rng, unsigned n);

}  // namespace systolique
