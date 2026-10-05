#include "correspondence.h"

#include "systolique/arith.h"

#include "vpi_user.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace systolique {

Correspondence::Correspondence(const ArrayConfig &cfg, Top top, const SystolicArray &model,
                               std::string scope, std::string map_file)
    : cfg_(cfg), top_(top), model_(model), scope_(std::move(scope)),
      map_file_(std::move(map_file)) {
  // SYSTOLIQUE_CORR_FAULT=<exact|feed|resp|valid|pipev>: flip one model bit of the first item of
  // that kind once, to show the check would see it (rtl_corr_fault_* tests).
  if (const char *f = std::getenv("SYSTOLIQUE_CORR_FAULT")) {
    const std::string k = f;
    const char *names[] = {"exact", "feed", "resp", "valid", "pipev"};
    for (int i = 0; i < 5; ++i)
      if (k == names[i]) fault_kind_ = i;
  }
}

namespace {

std::string top_modules() {
  std::string s;
  if (vpiHandle it = vpi_iterate(vpiModule, nullptr))
    while (vpiHandle m = vpi_scan(it)) s += std::string(" ") + vpi_get_str(vpiFullName, m);
  return s;
}

uint64_t read_vpi(vpiHandle h, unsigned width) {
  s_vpi_value v;
  v.format = vpiVectorVal;
  vpi_get_value(h, &v);
  uint64_t x = v.value.vector[0].aval;
  if (width > 32) x |= uint64_t(v.value.vector[1].aval) << 32;
  return arith::zext(x, width);
}

}  // namespace

std::string Correspondence::init() {
  const bool mwd = top_ == Top::MeshWithDelays;
  const std::string mwd_scope = scope_ + ".m.", mesh_scope = scope_ + (mwd ? ".m.mesh." : ".mesh.");
  std::vector<std::pair<std::string, int64_t>> regs;
  if (mwd) model_.mesh_with_delays().registers(regs);
  const size_t n_mwd = regs.size();
  model_.mesh().registers(regs);
  for (size_t i = 0; i < regs.size(); ++i) {
    Item it{Kind::Exact, (i < n_mwd ? mwd_scope : mesh_scope) + regs[i].first, "", 0, 0, 0, 0};
    it.reg_index = i;
    items_.push_back(it);
  }
  std::ifstream f(map_file_);
  if (!f) return "no correspondence map " + map_file_ + " (tools/rtl_provenance.py correspondence)";
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    std::string kind, sig, reg;
    Item it{};
    ls >> kind;
    if (kind == "feed" || kind == "resp") {
      if (!mwd) continue;
      ls >> it.sig >> it.a >> it.b >> it.stage >> reg;
      it.kind = kind == "feed" ? Kind::Feed : Kind::Resp;
      it.name = mwd_scope + reg;
    } else if (kind == "valid" || kind == "pipev") {
      ls >> it.a >> it.b >> it.c >> it.stage >> reg;
      it.kind = kind == "valid" ? Kind::Valid : Kind::PipeV;
      it.name = mesh_scope + reg;
    } else {
      return map_file_ + ": unknown line '" + line + "'";
    }
    if (!ls) return map_file_ + ": bad line '" + line + "'";
    items_.push_back(it);
  }
  std::vector<Item> found;
  for (auto &it : items_) {
    std::vector<char> name(it.name.begin(), it.name.end());
    name.push_back(0);
    vpiHandle h = vpi_handle_by_name(name.data(), nullptr);
    // A register the FIRRTL compiler removed because nothing reads it in this configuration
    // (e.g. the shift registers of a WS-only array) has nothing to compare with.
    if (!h && it.kind == Kind::Exact) {
      unused_.push_back(it.name.substr(scope_.size() + 1));
      continue;
    }
    if (!h) return "VPI: no " + it.name + " (top modules:" + top_modules() + ")";
    it.handle = h;
    it.width = unsigned(vpi_get(vpiSize, h));
    if (it.width == 0 || it.width > 64) return "VPI: " + it.name + " has width " +
                                               std::to_string(it.width);
    found.push_back(it);
  }
  items_.swap(found);
  ready_ = true;
  return "";
}

int64_t Correspondence::model_value(
    const Item &it, const std::vector<std::pair<std::string, int64_t>> &regs) const {
  switch (it.kind) {
    case Kind::Exact:
      return regs[it.reg_index].second;
    case Kind::Feed: {
      // feed <signal> <group> <lane in group> <stage>: ShiftRegister stage k holds the feed of
      // k+1 cycles ago.
      const MeshIn &h = model_.mesh_with_delays().feed_history(it.stage + 1);
      const unsigned lane = it.a * (it.sig == "a" ? cfg_.tile_rows : cfg_.tile_cols) + it.b;
      if (it.sig == "a") return h.a[lane];
      if (it.sig == "b") return h.b[lane];
      if (it.sig == "d") return h.d[lane];
      if (it.sig == "valid") return h.valid[lane];
      if (it.sig == "id") return h.id[lane];
      if (it.sig == "last") return h.last[lane];
      if (it.sig == "dataflow") return h.control[lane].dataflow;
      if (it.sig == "propagate") return h.control[lane].propagate;
      return h.control[lane].shift;
    }
    case Kind::Resp: {
      const auto &s = model_.mesh_with_delays().resp_history(it.stage);
      if (it.sig == "data") return s.data[it.a * cfg_.tile_cols + it.b];
      if (it.sig == "valid") return s.valid;
      if (it.sig == "last") return s.last;
      return s.id;
    }
    case Kind::Valid:
    case Kind::PipeV: {
      return model_.mesh().valid_stage(it.a, it.b, it.c, it.stage);
    }
  }
  return 0;
}

std::string Correspondence::check() {
  if (!ready_) {
    const std::string e = init();
    if (!e.empty()) return e;
  }
  std::vector<std::pair<std::string, int64_t>> regs;
  if (top_ == Top::MeshWithDelays) model_.mesh_with_delays().registers(regs);
  model_.mesh().registers(regs);
  std::string err;
  unsigned n = 0;
  for (size_t i = 0; i < items_.size(); ++i) {
    const Item &it = items_[i];
    const uint64_t rtl = read_vpi(static_cast<vpiHandle>(it.handle), it.width);
    uint64_t mine = arith::zext(uint64_t(model_value(it, regs)), it.width);
    if (fault_kind_ == int(it.kind) && fault_count_ == 0) {  // SYSTOLIQUE_CORR_FAULT self-test
      mine ^= 1;
      fault_count_ = 1;
    }
    ++checked_;
    if (rtl == mine) continue;
    if (n++ < 6)
      err += (err.empty() ? "" : "; ") + it.name + ": rtl=" + std::to_string(rtl) +
             " model=" + std::to_string(mine);
  }
  if (n > 6) err += "; ... (" + std::to_string(n) + " registers differ)";
  return err;
}

}  // namespace systolique
