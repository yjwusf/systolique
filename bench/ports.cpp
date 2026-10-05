#include "ports.h"

#include "systolique/arith.h"

#include <algorithm>
#include <cstdio>

namespace systolique {

void Bits::resize(unsigned bits) {
  width = bits;
  w.assign((bits + 31) / 32, 0);
}

uint64_t Bits::get(unsigned lo, unsigned n) const {
  uint64_t v = 0;
  for (unsigned i = 0; i < n; ++i) {
    const unsigned b = lo + i;
    if (b < width && ((w[b / 32] >> (b % 32)) & 1)) v |= uint64_t(1) << i;
  }
  return v;
}

void Bits::set(unsigned lo, unsigned n, uint64_t v) {
  for (unsigned i = 0; i < n; ++i) {
    const unsigned b = lo + i;
    if (b >= width) break;
    if ((v >> i) & 1)
      w[b / 32] |= 1u << (b % 32);
    else
      w[b / 32] &= ~(1u << (b % 32));
  }
}

std::string Bits::hex() const {
  const unsigned digits = (width + 3) / 4;
  std::string s(digits, '0');
  for (unsigned d = 0; d < digits; ++d) {
    const unsigned v = unsigned(get(d * 4, 4));
    s[digits - 1 - d] = "0123456789abcdef"[v];
  }
  return s;
}

bool Bits::parse_hex(const std::string &s) {
  std::fill(w.begin(), w.end(), 0);
  const unsigned digits = unsigned(s.size());
  for (unsigned d = 0; d < digits; ++d) {
    const char c = s[digits - 1 - d];
    unsigned v;
    if (c >= '0' && c <= '9')
      v = unsigned(c - '0');
    else if (c >= 'a' && c <= 'f')
      v = unsigned(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F')
      v = unsigned(c - 'A' + 10);
    else
      return false;
    if (d * 4 >= width) {
      if (v) return false;
      continue;
    }
    set(d * 4, 4, v);
  }
  return true;
}

const char *top_name(Top t) { return t == Top::Mesh ? "MeshTop" : "MeshWithDelaysTop"; }

std::vector<PortSpec> port_specs(const ArrayConfig &cfg, Top top) {
  const unsigned R = cfg.rows(), C = cfg.cols(), sw = cfg.shift_bits(), iw = cfg.id_bits();
  const unsigned in = cfg.in_bits, out = cfg.out_bits;
  if (top == Top::Mesh)
    return {{"reset", true, 1, 1, false},          {"in_a", true, R, in, true},
            {"in_b", true, C, in, true},           {"in_d", true, C, in, true},
            {"in_dataflow", true, C, 1, false},    {"in_propagate", true, C, 1, false},
            {"in_shift", true, C, sw, false},      {"in_id", true, C, iw, false},
            {"in_last", true, C, 1, false},        {"in_valid", true, C, 1, false},
            {"out_b", false, C, out, true},        {"out_c", false, C, out, true},
            {"out_valid", false, C, 1, false},     {"out_dataflow", false, C, 1, false},
            {"out_propagate", false, C, 1, false}, {"out_shift", false, C, sw, false},
            {"out_id", false, C, iw, false},       {"out_last", false, C, 1, false}};
  const unsigned rw = cfg.total_rows_bits(), tb = cfg.tag_bits, q = cfg.tagq_len();
  return {{"reset", true, 1, 1, false},           {"a_valid", true, 1, 1, false},
          {"a_bits", true, R, in, true},          {"b_valid", true, 1, 1, false},
          {"b_bits", true, C, in, true},          {"d_valid", true, 1, 1, false},
          {"d_bits", true, C, in, true},          {"req_valid", true, 1, 1, false},
          {"req_dataflow", true, 1, 1, false},    {"req_propagate", true, 1, 1, false},
          {"req_shift", true, 1, sw, false},      {"req_a_transpose", true, 1, 1, false},
          {"req_bd_transpose", true, 1, 1, false}, {"req_total_rows", true, 1, rw, false},
          {"req_tag_valid", true, 1, 1, false},   {"req_tag_id", true, 1, tb, false},
          {"req_flush", true, 1, 2, false},       {"a_ready", false, 1, 1, false},
          {"b_ready", false, 1, 1, false},        {"d_ready", false, 1, 1, false},
          {"req_ready", false, 1, 1, false},      {"resp_valid", false, 1, 1, false},
          {"resp_data", false, C, out, true},     {"resp_total_rows", false, 1, rw, false},
          {"resp_tag_valid", false, 1, 1, false}, {"resp_tag_id", false, 1, tb, false},
          {"resp_last", false, 1, 1, false},      {"tags_valid", false, q, 1, false},
          {"tags_id", false, q, tb, false}};
}

Frame make_frame(const std::vector<PortSpec> &spec) {
  Frame f(spec.size());
  for (size_t i = 0; i < spec.size(); ++i) f[i].resize(spec[i].width());
  return f;
}

int port_index(const std::vector<PortSpec> &spec, const std::string &name) {
  for (size_t i = 0; i < spec.size(); ++i)
    if (spec[i].name == name) return int(i);
  return -1;
}

namespace {

// Lane helpers: frame port p, lane k, lane width lw.
template <class T>
void put(Frame &f, size_t p, unsigned lw, const std::vector<T> &v) {
  for (size_t k = 0; k < v.size(); ++k) f[p].set(unsigned(k) * lw, lw, uint64_t(int64_t(v[k])));
}
void put1(Frame &f, size_t p, unsigned lw, uint64_t v) { f[p].set(0, lw, v); }
void get_s(const Frame &f, size_t p, unsigned lw, std::vector<int64_t> &v) {
  for (size_t k = 0; k < v.size(); ++k)
    v[k] = arith::sext(int64_t(f[p].get(unsigned(k) * lw, lw)), lw);
}
void get_u(const Frame &f, size_t p, unsigned lw, std::vector<unsigned> &v) {
  for (size_t k = 0; k < v.size(); ++k) v[k] = unsigned(f[p].get(unsigned(k) * lw, lw));
}

}  // namespace

void mesh_in_to_frame(const ArrayConfig &cfg, const MeshIn &in, Frame &f) {
  const unsigned C = cfg.cols();
  std::vector<unsigned> df(C), prop(C), shift(C);
  for (unsigned k = 0; k < C; ++k) {
    df[k] = in.control[k].dataflow;
    prop[k] = in.control[k].propagate;
    shift[k] = in.control[k].shift;
  }
  put1(f, 0, 1, in.reset);
  put(f, 1, cfg.in_bits, in.a);
  put(f, 2, cfg.in_bits, in.b);
  put(f, 3, cfg.in_bits, in.d);
  put(f, 4, 1, df);
  put(f, 5, 1, prop);
  put(f, 6, cfg.shift_bits(), shift);
  put(f, 7, cfg.id_bits(), in.id);
  put(f, 8, 1, in.last);
  put(f, 9, 1, in.valid);
}

void frame_to_mesh_in(const ArrayConfig &cfg, const Frame &f, MeshIn &in) {
  const unsigned C = cfg.cols();
  in.resize(cfg);
  std::vector<unsigned> df(C), prop(C), shift(C);
  in.reset = f[0].get(0, 1);
  get_s(f, 1, cfg.in_bits, in.a);
  get_s(f, 2, cfg.in_bits, in.b);
  get_s(f, 3, cfg.in_bits, in.d);
  get_u(f, 4, 1, df);
  get_u(f, 5, 1, prop);
  get_u(f, 6, cfg.shift_bits(), shift);
  get_u(f, 7, cfg.id_bits(), in.id);
  get_u(f, 8, 1, in.last);
  get_u(f, 9, 1, in.valid);
  for (unsigned k = 0; k < C; ++k) in.control[k] = PeControl{df[k], prop[k], shift[k]};
}

void mesh_out_to_frame(const ArrayConfig &cfg, const MeshOut &o, Frame &f) {
  const unsigned C = cfg.cols();
  std::vector<unsigned> df(C), prop(C), shift(C);
  for (unsigned k = 0; k < C; ++k) {
    df[k] = o.control[k].dataflow;
    prop[k] = o.control[k].propagate;
    shift[k] = o.control[k].shift;
  }
  put(f, 10, cfg.out_bits, o.b);
  put(f, 11, cfg.out_bits, o.c);
  put(f, 12, 1, o.valid);
  put(f, 13, 1, df);
  put(f, 14, 1, prop);
  put(f, 15, cfg.shift_bits(), shift);
  put(f, 16, cfg.id_bits(), o.id);
  put(f, 17, 1, o.last);
}

void mwd_in_to_frame(const ArrayConfig &cfg, const MwdIn &in, Frame &f) {
  put1(f, 0, 1, in.reset);
  put1(f, 1, 1, in.a_valid);
  put(f, 2, cfg.in_bits, in.a);
  put1(f, 3, 1, in.b_valid);
  put(f, 4, cfg.in_bits, in.b);
  put1(f, 5, 1, in.d_valid);
  put(f, 6, cfg.in_bits, in.d);
  put1(f, 7, 1, in.req_valid);
  put1(f, 8, 1, in.req.dataflow);
  put1(f, 9, 1, in.req.propagate);
  put1(f, 10, cfg.shift_bits(), in.req.shift);
  put1(f, 11, 1, in.req.a_transpose);
  put1(f, 12, 1, in.req.bd_transpose);
  put1(f, 13, cfg.total_rows_bits(), in.req.total_rows);
  put1(f, 14, 1, in.req.tag_valid);
  put1(f, 15, cfg.tag_bits, in.req.tag_id);
  put1(f, 16, 2, in.req.flush);
}

void frame_to_mwd_in(const ArrayConfig &cfg, const Frame &f, MwdIn &in) {
  in.resize(cfg);
  in.reset = f[0].get(0, 1);
  in.a_valid = unsigned(f[1].get(0, 1));
  get_s(f, 2, cfg.in_bits, in.a);
  in.b_valid = unsigned(f[3].get(0, 1));
  get_s(f, 4, cfg.in_bits, in.b);
  in.d_valid = unsigned(f[5].get(0, 1));
  get_s(f, 6, cfg.in_bits, in.d);
  in.req_valid = unsigned(f[7].get(0, 1));
  in.req.dataflow = unsigned(f[8].get(0, 1));
  in.req.propagate = unsigned(f[9].get(0, 1));
  in.req.shift = unsigned(f[10].get(0, cfg.shift_bits()));
  in.req.a_transpose = unsigned(f[11].get(0, 1));
  in.req.bd_transpose = unsigned(f[12].get(0, 1));
  in.req.total_rows = unsigned(f[13].get(0, cfg.total_rows_bits()));
  in.req.tag_valid = unsigned(f[14].get(0, 1));
  in.req.tag_id = unsigned(f[15].get(0, cfg.tag_bits));
  in.req.flush = unsigned(f[16].get(0, 2));
}

void mwd_out_to_frame(const ArrayConfig &cfg, const MwdOut &o, Frame &f) {
  put1(f, 17, 1, o.a_ready);
  put1(f, 18, 1, o.b_ready);
  put1(f, 19, 1, o.d_ready);
  put1(f, 20, 1, o.req_ready);
  put1(f, 21, 1, o.resp_valid);
  put(f, 22, cfg.out_bits, o.resp_data);
  put1(f, 23, cfg.total_rows_bits(), o.resp_total_rows);
  put1(f, 24, 1, o.resp_tag_valid);
  put1(f, 25, cfg.tag_bits, o.resp_tag_id);
  put1(f, 26, 1, o.resp_last);
  put(f, 27, 1, o.tags_valid);
  put(f, 28, cfg.tag_bits, o.tags_id);
}

void frame_to_mwd_out(const ArrayConfig &cfg, const Frame &f, MwdOut &o) {
  o.resize(cfg);
  o.a_ready = unsigned(f[17].get(0, 1));
  o.b_ready = unsigned(f[18].get(0, 1));
  o.d_ready = unsigned(f[19].get(0, 1));
  o.req_ready = unsigned(f[20].get(0, 1));
  o.resp_valid = unsigned(f[21].get(0, 1));
  get_s(f, 22, cfg.out_bits, o.resp_data);
  o.resp_total_rows = unsigned(f[23].get(0, cfg.total_rows_bits()));
  o.resp_tag_valid = unsigned(f[24].get(0, 1));
  o.resp_tag_id = unsigned(f[25].get(0, cfg.tag_bits));
  o.resp_last = unsigned(f[26].get(0, 1));
  get_u(f, 27, 1, o.tags_valid);
  get_u(f, 28, cfg.tag_bits, o.tags_id);
}

std::string diff_outputs(const std::vector<PortSpec> &spec, const Frame &ref, const Frame &dut,
                         unsigned max_items) {
  std::string s;
  unsigned n = 0;
  for (size_t p = 0; p < spec.size(); ++p) {
    if (spec[p].input || ref[p] == dut[p]) continue;
    for (unsigned k = 0; k < spec[p].lanes; ++k) {
      const unsigned lw = spec[p].lane_bits;
      const uint64_t r = ref[p].get(k * lw, lw), d = dut[p].get(k * lw, lw);
      if (r == d) continue;
      if (n++ == max_items) return s + " ...";
      char buf[160];
      std::snprintf(buf, sizeof buf, "%s%s[%u]: rtl=%llx model=%llx", s.empty() ? "" : "; ",
                    spec[p].name.c_str(), k, (unsigned long long)r, (unsigned long long)d);
      s += buf;
    }
  }
  return s;
}

}  // namespace systolique
