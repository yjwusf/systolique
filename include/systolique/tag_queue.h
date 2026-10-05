// The two queues of MeshWithDelays (MeshWithDelays.scala:206-249, Gemmini v0.7.2):
//   TagQueue   Gemmini's TagQueue (TagQueue.scala:11-52) of TagWithIdAndTotalRows: the tag of
//              every non-flush request with the matmul id of the rows that will carry its
//              result; io.all is tags_in_progress
//   RowsQueue  the chisel3.util.Queue total_rows_q (MeshWithDelays.scala:237-246): total_rows
//              with the matmul id of the request's own rows
// Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Both: enq.ready = not full, deq.valid = not empty, deq.bits = the head; a fire is valid &&
// ready. Two-phase: set_inputs, eval() (next state), tick() (commit). Outputs depend on
// registers only.
#pragma once

#include <vector>

namespace systolique {

// TagWithIdAndTotalRows as far as it is ever read: the bench tag {valid, id} and the matmul id.
// (Its total_rows field is DontCare on enq and only make_this_garbage writes it,
// MeshWithDelays.scala:207-226; nothing reads it, the FIRRTL compiler removes it.)
struct TagEntry {
  unsigned tag_valid = 0, tag_id = 0, id = 0;
};

struct QueueIn {
  bool enq_valid = false;
  TagEntry enq_bits;      // TagQueue: tag + id; RowsQueue: id + total_rows (in tag_id)
  bool deq_ready = false;
  bool reset = false;
};

class TagQueue {
 public:
  // `entries` = tagqlen (MeshWithDelays.scala:56); `tag_bits` for the garbage tag (all ones).
  TagQueue(unsigned entries, unsigned tag_bits);
  ~TagQueue();

  void set_inputs(const QueueIn &in);
  void eval();
  void tick();

  bool enq_ready() const { return len_ != entries_; }      // !full  (TagQueue.scala:24-27)
  bool deq_valid() const { return len_ != 0; }             // !empty
  const TagEntry &deq_bits() const { return regs_[raddr_]; }
  const std::vector<TagEntry> &all() const { return regs_; }  // io.all
  unsigned raddr() const { return raddr_; }
  unsigned waddr() const { return waddr_; }
  unsigned len() const { return len_; }

 private:
  TagEntry garbage() const;
  unsigned entries_, tag_bits_;
  std::vector<TagEntry> regs_, next_regs_;
  unsigned raddr_ = 0, waddr_ = 0, len_ = 0;
  unsigned next_raddr_ = 0, next_waddr_ = 0, next_len_ = 0;
  QueueIn in_;
  bool evaluated_ = false;
};

// Entries {id, total_rows}; total_rows travels in TagEntry::tag_id.
class RowsQueue {
 public:
  explicit RowsQueue(unsigned entries);
  ~RowsQueue();

  void set_inputs(const QueueIn &in);
  void eval();
  void tick();

  bool empty() const { return enq_ == deq_ && !maybe_full_; }
  bool full() const { return enq_ == deq_ && maybe_full_; }
  bool enq_ready() const { return !full(); }
  bool deq_valid() const { return !empty(); }
  const TagEntry &deq_bits() const { return ram_[deq_]; }
  unsigned enq_ptr() const { return enq_; }
  unsigned deq_ptr() const { return deq_; }
  bool maybe_full() const { return maybe_full_; }

 private:
  unsigned entries_;
  std::vector<TagEntry> ram_, next_ram_;
  unsigned enq_ = 0, deq_ = 0, next_enq_ = 0, next_deq_ = 0;
  bool maybe_full_ = false, next_maybe_full_ = false;
  QueueIn in_;
  bool evaluated_ = false;
};

}  // namespace systolique
