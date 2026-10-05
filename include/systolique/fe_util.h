// Cycle-level equivalents of the chisel3.util building blocks Gemmini's frontend is made of
// (chisel3 3.6.0: Queue, Arbiter; Gemmini v0.7.2: Pipeline.scala, MultiHeadedQueue.scala,
// Util.scala). Derived from Gemmini (BSD-3-Clause, see LICENSE.gemmini and NOTICE).
//
// Each block keeps its registers as the RTL does; its outputs are const functions of the
// registers (and, where Chisel makes a path combinational, of the inputs passed to them), and
// tick() is the rising edge: it applies the handshakes of the cycle. They are the registers of
// the classes that own them (ExecuteController, Scratchpad, ...), which evaluate them in their
// eval() and call tick() in their own tick().
//
// The queues keep their RAM and pointers, so the bits of an empty queue's head are the stale RAM
// entry, as in the hardware: some ExecuteController paths read heads that are not valid, and
// their values still steer bank selection and hazard checks.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace systolique {

inline uint64_t mask_bits(unsigned n) { return n >= 64 ? ~0ull : ((1ull << n) - 1); }

// Util.wrappingAdd (Util.scala:17-32) with the hardware width of u + n.
inline uint64_t wrapping_add(uint64_t u, uint64_t n, uint64_t max_plus_one, unsigned width,
                             bool en = true) {
  if (!en)
    return u;
  const uint64_t max = (max_plus_one - 1) & mask_bits(width + 1);
  if (max == 0)
    return 0;
  if (n != 0 && u >= max - n + 1)
    return (n - (max - u) - 1) & mask_bits(width);
  return (u + n) & mask_bits(width);
}

// Util.floorAdd (Util.scala:38-45): u + n, or 0 past max_plus_one - 1.
inline uint64_t floor_add(uint64_t u, uint64_t n, uint64_t max_plus_one, unsigned width,
                          bool en = true) {
  if (!en)
    return u;
  const uint64_t max = (max_plus_one - 1) & mask_bits(width);
  if (u + n > max)
    return 0;
  return (u + n) & mask_bits(width);
}

// chisel3.util.Queue(gen, entries, pipe, flow): a RAM, enq/deq pointers and maybe_full.
template <typename T>
class Queue {
 public:
  explicit Queue(unsigned entries = 2, bool pipe = false, bool flow = false)
      : ram_(entries), pipe_(pipe), flow_(flow) {
    if (entries == 0)
      throw std::invalid_argument("Queue: zero entries");
  }
  ~Queue() = default;

  unsigned entries() const { return unsigned(ram_.size()); }
  unsigned count() const {
    const unsigned n = entries();
    if (enq_ == deq_)
      return maybe_full_ ? n : 0;
    return (enq_ + n - deq_) % n;
  }
  bool empty() const { return enq_ == deq_ && !maybe_full_; }
  bool full() const { return enq_ == deq_ && maybe_full_; }
  // io.enq.ready (pipe: also when the head leaves this cycle)
  bool enq_ready(bool deq_ready = false) const { return !full() || (pipe_ && deq_ready); }
  // io.deq.valid (flow: an incoming element passes straight through an empty queue)
  bool deq_valid(bool enq_valid = false) const { return !empty() || (flow_ && enq_valid); }
  // io.deq.bits
  const T &deq_bits(const T *enq_bits = nullptr) const {
    if (flow_ && empty() && enq_bits)
      return *enq_bits;
    return ram_[deq_];
  }
  // RAM entry i positions behind the head (stale when not valid)
  const T &at(unsigned i) const { return ram_[(deq_ + i) % entries()]; }

  // Rising edge with this cycle's handshakes.
  void tick(bool enq_fire, const T &enq_bits, bool deq_fire) {
    bool do_enq = enq_fire, do_deq = deq_fire;
    if (flow_ && empty() && enq_fire) {  // chisel3 Queue: flow through an empty queue
      do_deq = false;
      if (deq_fire)
        do_enq = false;
    }
    if (do_enq) {
      ram_[enq_] = enq_bits;
      enq_ = (enq_ + 1) % entries();
    }
    if (do_deq)
      deq_ = (deq_ + 1) % entries();
    if (do_enq != do_deq)
      maybe_full_ = do_enq;
  }

 private:
  std::vector<T> ram_;
  unsigned enq_ = 0, deq_ = 0;
  bool maybe_full_ = false;
  bool pipe_, flow_;
};

// Gemmini Pipeline(gen, latency) (Pipeline.scala:6-65): `latency` stall-able register stages.
template <typename T>
class Pipeline {
 public:
  explicit Pipeline(unsigned latency = 1) : stages_(latency), valids_(latency, false) {
    if (latency == 0)
      throw std::invalid_argument("Pipeline: latency 0");
  }
  ~Pipeline() = default;

  unsigned latency() const { return unsigned(stages_.size()); }
  bool out_valid() const { return valids_.back(); }
  const T &out_bits() const { return stages_.back(); }
  // io.in.ready for this cycle's io.out.ready
  bool in_ready(bool out_ready) const {
    const unsigned L = latency();
    bool stall = valids_[L - 1] && !out_ready;
    for (int i = int(L) - 2; i >= 0; --i)
      stall = valids_[unsigned(i)] && stall;
    return !stall;
  }
  bool any_valid() const {
    for (bool v : valids_)
      if (v)
        return true;
    return false;
  }
  // Rising edge with this cycle's handshakes (Pipeline.scala:26-63; later `when`s win).
  void tick(bool in_fire, const T &in_bits, bool out_ready) {
    const unsigned L = latency();
    std::vector<bool> stall(L, false);
    stall[L - 1] = valids_[L - 1] && !out_ready;
    for (int i = int(L) - 2; i >= 0; --i)
      stall[unsigned(i)] = valids_[unsigned(i)] && stall[unsigned(i) + 1];
    std::vector<bool> v = valids_;
    std::vector<T> s = stages_;
    if (out_ready)
      v[L - 1] = false;
    for (unsigned i = 0; i + 1 < L; ++i)
      if (!stall[i + 1])
        v[i] = false;
    if (in_fire)
      v[0] = true;
    for (unsigned i = 1; i < L; ++i)
      if (valids_[i - 1])
        v[i] = true;
    if (in_fire)
      s[0] = in_bits;
    for (unsigned i = 1; i < L; ++i)
      if (!stall[i])
        s[i] = stages_[i - 1];
    valids_.swap(v);
    stages_.swap(s);
  }

 private:
  std::vector<T> stages_;
  std::vector<bool> valids_;
};

// Gemmini MultiHeadedQueue (MultiHeadedQueue.scala:7-49): `heads` entries visible at once,
// popped up to `heads` at a time.
template <typename T>
class MultiHeadedQueue {
 public:
  MultiHeadedQueue(unsigned entries, unsigned heads) : regs_(entries), heads_(heads) {
    if (entries == 0 || heads == 0 || heads > entries)
      throw std::invalid_argument("MultiHeadedQueue: bad size");
  }
  ~MultiHeadedQueue() = default;
  unsigned entries() const { return unsigned(regs_.size()); }
  unsigned len() const { return len_; }
  bool enq_ready() const { return len_ < regs_.size(); }
  bool valid(unsigned i) const { return len_ > i; }
  const T &bits(unsigned i) const { return regs_[(raddr_ + i) % regs_.size()]; }
  void tick(bool enq_fire, const T &enq_bits, unsigned pop) {
    const unsigned n = unsigned(regs_.size());
    if (pop > len_ || pop > heads_)
      throw std::logic_error("MultiHeadedQueue: pop past the valid heads");
    unsigned len = len_;
    if (enq_fire) {
      regs_[waddr_] = enq_bits;
      waddr_ = (waddr_ + 1) % n;
      len = len_ + 1;
    }
    if (pop > 0) {
      raddr_ = (raddr_ + pop) % n;
      len = len_ - pop + (enq_fire ? 1 : 0);
    }
    len_ = len;
  }

 private:
  std::vector<T> regs_;
  unsigned heads_;
  unsigned raddr_ = 0, waddr_ = 0, len_ = 0;
};

// chisel3.util.Arbiter: the lowest-index valid input wins (-1: none).
inline int arbiter_choice(const std::vector<bool> &valid) {
  for (size_t i = 0; i < valid.size(); ++i)
    if (valid[i])
      return int(i);
  return -1;
}

}  // namespace systolique
