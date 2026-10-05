// The Verilated RTL tops (rtl/src/GemminiTops.scala: MeshTop / MeshWithDelaysTop) driven through
// port Frames (bench/ports.h). The port names are the same in every configuration, so one
// template serves all Verilated classes. Per cycle: set_inputs, eval, get_outputs, edge.
#pragma once

#include "ports.h"

#include "verilated.h"

#include <memory>

namespace systolique {

template <class T>
void rtl_put(T &port, const Bits &b) {
  port = T(b.get(0, b.width));
}
template <std::size_t N>
void rtl_put(VlWide<N> &port, const Bits &b) {
  for (std::size_t i = 0; i < N; ++i) port[i] = i < b.w.size() ? b.w[i] : 0;
}
template <class T>
void rtl_get(const T &port, Bits &b) {
  b.set(0, b.width, uint64_t(port));
}
template <std::size_t N>
void rtl_get(const VlWide<N> &port, Bits &b) {
  for (std::size_t i = 0; i < b.w.size(); ++i) b.w[i] = i < N ? port[i] : 0;
  if (b.width % 32) b.w.back() &= (1u << (b.width % 32)) - 1;
}

template <class V>
class RtlDut {
 public:
  // Every RtlDut of a process shares one VerilatedContext (`ctx`); `name` must be unique.
  RtlDut(VerilatedContext &ctx, const char *name) : ctx_(ctx), name_(name) {}
  // Power-on: a fresh Verilated model (every register 0 with --x-initial 0).
  void reinit() {
    rtl_.reset();  // one model per context and name at a time
    rtl_ = std::make_unique<V>(&ctx_, name_);
    rtl_->clock = 0;
    rtl_->reset = 0;
    rtl_->eval();
  }
  void set_inputs(const Frame &f);
  void eval() { rtl_->eval(); }
  void get_outputs(Frame &f);
  void edge() {
    rtl_->clock = 1;
    rtl_->eval();
    rtl_->clock = 0;
    rtl_->eval();
  }
  bool failed() const { return ctx_.gotError() || ctx_.gotFinish(); }
  V &rtl() { return *rtl_; }
  VerilatedContext &context() { return ctx_; }

 private:
  VerilatedContext &ctx_;
  const char *name_;
  std::unique_ptr<V> rtl_;
};

// MeshTop and MeshWithDelaysTop share no port name except clock and reset, so one class per
// top: the members below only compile for the matching Verilated class.
template <class V>
struct MeshPorts {
  static void set(V &t, const Frame &f) {
    rtl_put(t.reset, f[0]);
    rtl_put(t.io_in_a, f[1]);
    rtl_put(t.io_in_b, f[2]);
    rtl_put(t.io_in_d, f[3]);
    rtl_put(t.io_in_dataflow, f[4]);
    rtl_put(t.io_in_propagate, f[5]);
    rtl_put(t.io_in_shift, f[6]);
    rtl_put(t.io_in_id, f[7]);
    rtl_put(t.io_in_last, f[8]);
    rtl_put(t.io_in_valid, f[9]);
  }
  static void get(const V &t, Frame &f) {
    rtl_get(t.io_out_b, f[10]);
    rtl_get(t.io_out_c, f[11]);
    rtl_get(t.io_out_valid, f[12]);
    rtl_get(t.io_out_dataflow, f[13]);
    rtl_get(t.io_out_propagate, f[14]);
    rtl_get(t.io_out_shift, f[15]);
    rtl_get(t.io_out_id, f[16]);
    rtl_get(t.io_out_last, f[17]);
  }
};

template <class V>
struct MwdPorts {
  static void set(V &t, const Frame &f) {
    rtl_put(t.reset, f[0]);
    rtl_put(t.io_a_valid, f[1]);
    rtl_put(t.io_a_bits, f[2]);
    rtl_put(t.io_b_valid, f[3]);
    rtl_put(t.io_b_bits, f[4]);
    rtl_put(t.io_d_valid, f[5]);
    rtl_put(t.io_d_bits, f[6]);
    rtl_put(t.io_req_valid, f[7]);
    rtl_put(t.io_req_dataflow, f[8]);
    rtl_put(t.io_req_propagate, f[9]);
    rtl_put(t.io_req_shift, f[10]);
    rtl_put(t.io_req_a_transpose, f[11]);
    rtl_put(t.io_req_bd_transpose, f[12]);
    rtl_put(t.io_req_total_rows, f[13]);
    rtl_put(t.io_req_tag_valid, f[14]);
    rtl_put(t.io_req_tag_id, f[15]);
    rtl_put(t.io_req_flush, f[16]);
  }
  static void get(const V &t, Frame &f) {
    rtl_get(t.io_a_ready, f[17]);
    rtl_get(t.io_b_ready, f[18]);
    rtl_get(t.io_d_ready, f[19]);
    rtl_get(t.io_req_ready, f[20]);
    rtl_get(t.io_resp_valid, f[21]);
    rtl_get(t.io_resp_data, f[22]);
    rtl_get(t.io_resp_total_rows, f[23]);
    rtl_get(t.io_resp_tag_valid, f[24]);
    rtl_get(t.io_resp_tag_id, f[25]);
    rtl_get(t.io_resp_last, f[26]);
    rtl_get(t.io_tags_valid, f[27]);
    rtl_get(t.io_tags_id, f[28]);
  }
};

// Ports<V> selects MeshPorts or MwdPorts; specialised in the bench per Verilated class.
template <class V>
struct Ports;

template <class V>
void RtlDut<V>::set_inputs(const Frame &f) {
  Ports<V>::set(*rtl_, f);
}
template <class V>
void RtlDut<V>::get_outputs(Frame &f) {
  Ports<V>::get(*rtl_, f);
}

}  // namespace systolique
