// Derived from Gemmini's TagQueue.scala and MeshWithDelays.scala (BSD-3-Clause, see
// LICENSE.gemmini and NOTICE).
#include "systolique/tag_queue.h"

#include "systolique/arith.h"
#include "systolique/config.h"

namespace systolique {

using arith::wrapping_add_int;
using arith::zext;

TagQueue::TagQueue(unsigned entries, unsigned tag_bits)
    : entries_(entries), tag_bits_(tag_bits), regs_(entries), next_regs_(entries) {}

TagQueue::~TagQueue() = default;

// make_this_garbage (BenchTag in rtl/src/GemminiTops.scala): valid 0, id all ones.
TagEntry TagQueue::garbage() const {
  TagEntry g;
  g.tag_valid = 0;
  g.tag_id = unsigned(zext(~uint64_t(0), tag_bits_));
  return g;
}

void TagQueue::set_inputs(const QueueIn &in) {
  in_ = in;
  evaluated_ = false;
}

void TagQueue::eval() {
  next_regs_ = regs_;
  next_raddr_ = raddr_;
  next_waddr_ = waddr_;
  next_len_ = len_;
  const bool enq = in_.enq_valid && enq_ready();
  const bool deq = in_.deq_ready && deq_valid();
  const unsigned w = log2_up(entries_) + 1;  // wrappingAdd's literal width covers `entries`
  if (enq) {  // TagQueue.scala:31-34
    next_regs_[waddr_] = in_.enq_bits;
    next_waddr_ = unsigned(wrapping_add_int(waddr_, 1, entries_, w));
  }
  if (deq) {  // TagQueue.scala:36-39 (the id of a garbage entry is kept)
    const TagEntry g = garbage();
    next_regs_[raddr_].tag_valid = g.tag_valid;
    next_regs_[raddr_].tag_id = g.tag_id;
    next_raddr_ = unsigned(wrapping_add_int(raddr_, 1, entries_, w));
  }
  if (enq && !deq) ++next_len_;  // TagQueue.scala:41-45
  if (!enq && deq) --next_len_;
  if (in_.reset) {  // RegInit pointers, regs.foreach(make_this_garbage) (TagQueue.scala:19-21, 47-49)
    const TagEntry g = garbage();
    for (auto &e : next_regs_) {
      e.tag_valid = g.tag_valid;
      e.tag_id = g.tag_id;
    }
    next_raddr_ = next_waddr_ = next_len_ = 0;
  }
  evaluated_ = true;
}

void TagQueue::tick() {
  if (!evaluated_) eval();
  regs_.swap(next_regs_);
  raddr_ = next_raddr_;
  waddr_ = next_waddr_;
  len_ = next_len_;
  evaluated_ = false;
}

RowsQueue::RowsQueue(unsigned entries) : entries_(entries), ram_(entries), next_ram_(entries) {}

RowsQueue::~RowsQueue() = default;

void RowsQueue::set_inputs(const QueueIn &in) {
  in_ = in;
  evaluated_ = false;
}

// chisel3.util.Queue: a ram, enq_ptr/deq_ptr counters, maybe_full (all pointers RegInit).
void RowsQueue::eval() {
  next_ram_ = ram_;
  next_enq_ = enq_;
  next_deq_ = deq_;
  next_maybe_full_ = maybe_full_;
  const bool enq = in_.enq_valid && enq_ready();
  const bool deq = in_.deq_ready && deq_valid();
  if (enq) {
    next_ram_[enq_] = in_.enq_bits;
    next_enq_ = enq_ + 1 == entries_ ? 0 : enq_ + 1;
  }
  if (deq) next_deq_ = deq_ + 1 == entries_ ? 0 : deq_ + 1;
  if (enq != deq) next_maybe_full_ = enq;
  if (in_.reset) {
    next_enq_ = next_deq_ = 0;
    next_maybe_full_ = false;
  }
  evaluated_ = true;
}

void RowsQueue::tick() {
  if (!evaluated_) eval();
  ram_.swap(next_ram_);
  enq_ = next_enq_;
  deq_ = next_deq_;
  maybe_full_ = next_maybe_full_;
  evaluated_ = false;
}

}  // namespace systolique
