// ReservationStation (ReservationStation.scala, Gemmini v0.7.2): per-queue (load, execute, store)
// entries with their scratchpad / accumulator operand ranges, dependency bit vectors (RAW, WAR,
// WAW across queues; in order within a queue), in-order issue to the three controllers,
// completion from them, and the completion counts the loop unrollers use. Derived from Gemmini
// (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Clock (two-phase): issue(q), busy() and alloc_ready() depend on registers only (alloc_ready on
// the command offered); set_inputs() gives the cycle's allocation, issue handshakes and
// completion; eval() computes the next entries and this cycle's completion counts (the
// io.matmul_*_completed outputs, combinational from the inputs, ReservationStation.scala:154-160)
// without changing a register; tick() commits.
#pragma once

#include "systolique/isa.h"

#include <array>
#include <vector>

namespace systolique {

class ReservationStation {
 public:
  enum Q : unsigned { LDQ = 0, EXQ = 1, STQ = 2 };
  struct Issue {
    bool valid = false;
    GemminiCmd cmd;       // Mux1H of the selected entry (all zero when none is)
    unsigned rob_id = 0;  // Cat(q, issue_id) (:398)
    unsigned entry = 0;
  };
  struct In {
    bool alloc_valid = false;  // io.alloc.valid
    GemminiCmd alloc;
    bool issue_ready[3] = {false, false, false};  // io.issue.{ld,ex,st}.ready
    bool completed_valid = false;                 // io.completed
    unsigned completed_id = 0;
  };
  struct Completed {  // io.matmul_{ld,ex,st}_completed (:154-160)
    unsigned matmul_ld = 0, matmul_ex = 0, matmul_st = 0;
  };

  explicit ReservationStation(const FrontendConfig &cfg);
  ~ReservationStation();
  ReservationStation(const ReservationStation &) = default;
  ReservationStation &operator=(const ReservationStation &) = delete;

  const Issue &issue(unsigned q) const { return issue_[q]; }
  bool alloc_ready(const GemminiCmd &cmd) const;  // io.alloc.ready for this command
  bool busy() const;                              // io.busy (:135)
  void set_inputs(const In &in);
  void eval();
  void tick();
  const Completed &completed() const { return done_; }  // after eval()
  bool alloc_fire() const { return alloc_fire_; }        // after eval()
  unsigned occupancy(unsigned q) const;
  unsigned rob_type_bits() const { return type_bits_; }

 private:
  struct Op {  // OpT (:63-73) with its UDValid
    bool valid = false;
    LocalAddr start, end;
    bool wraps_around = false;
    bool overlaps(const Op &o) const {
      return ((o.start.le(start) && (start.lt(o.end) || o.wraps_around)) ||
              (start.le(o.start) && (o.start.lt(end) || wraps_around))) &&
             !(start.is_garbage() || o.start.is_garbage());
    }
  };
  struct Entry {  // (:81-111) with its UDValid
    bool valid = false;
    unsigned q = 0;
    bool is_config = false;
    Op opa, opb;
    bool opa_is_dst = false;
    bool issued = false;
    bool complete_on_issue = false;
    GemminiCmd cmd;
    std::vector<bool> deps[3];  // deps_ld, deps_ex, deps_st
    bool ready() const {
      for (const auto &d : deps)
        for (bool b : d)
          if (b)
            return false;
      return true;
    }
  };
  struct Config {  // config registers (:163-169) and solitary_preload (:134)
    uint64_t a_stride = 0, c_stride = 0;
    bool a_transpose = false;
    uint64_t ld_block_strides[3] = {0, 0, 0}, ld_pixel_repeats[3] = {0, 0, 0};
    bool pooling_is_enabled = false;
    bool solitary_preload = false;
  };
  Entry make_entry(const GemminiCmd &cmd) const;
  unsigned queue_of(const GemminiCmd &cmd) const;
  void settle();  // issue_ from the entries

  FrontendConfig cfg_;
  AddrMap amap_;
  unsigned block_, type_bits_;
  std::vector<Entry> e_[3];
  Config conf_;
  Issue issue_[3];
  // this cycle
  In in_;
  std::vector<Entry> next_[3];
  Config next_conf_;
  Completed done_;
  bool alloc_fire_ = false;
  bool evaluated_ = false;
};

}  // namespace systolique
