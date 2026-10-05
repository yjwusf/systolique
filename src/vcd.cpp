#include "systolique/vcd.h"

#include <stdexcept>

namespace systolique {

VcdWriter::VcdWriter(const std::string &path, const std::string &root, const std::string &timescale)
    : out_(path), root_(root), timescale_(timescale) {
  if (!out_) throw std::runtime_error("VcdWriter: cannot write " + path);
}

VcdWriter::~VcdWriter() { out_.flush(); }

int VcdWriter::add(const std::string &scope, const std::string &name, unsigned width) {
  if (started_) throw std::logic_error("VcdWriter: add() after the first commit()");
  if (width < 1 || width > 64) throw std::invalid_argument("VcdWriter: width must be 1..64");
  // Identifier codes: printable ASCII 33..126, base 94.
  std::string code;
  for (size_t n = vars_.size();; n = n / 94 - 1) {
    code += char(33 + n % 94);
    if (n < 94) break;
  }
  vars_.push_back(Var{scope, name, code, width});
  return int(vars_.size()) - 1;
}

void VcdWriter::set(int id, uint64_t value) {
  Var &v = vars_[size_t(id)];
  if (v.width < 64) value &= (uint64_t(1) << v.width) - 1;
  if (value != v.value) {
    v.value = value;
    v.dirty = true;
  }
}

void VcdWriter::write_value(const Var &v) {
  if (v.width == 1) {
    out_ << (v.value & 1) << v.code << '\n';
    return;
  }
  std::string bits;
  for (unsigned k = v.width; k-- > 0;) bits += char('0' + ((v.value >> k) & 1));
  const size_t first = bits.find('1');  // leading zeros may be dropped (IEEE 1364 18.2.3.5)
  out_ << 'b' << (first == std::string::npos ? "0" : bits.substr(first)) << ' ' << v.code << '\n';
}

void VcdWriter::header() {
  out_ << "$version systolique VcdWriter $end\n$timescale " << timescale_ << " $end\n";
  out_ << "$scope module " << root_ << " $end\n";
  std::vector<std::string> scopes;
  for (const Var &v : vars_) {
    if (v.scope.empty()) {
      out_ << "$var wire " << v.width << ' ' << v.code << ' ' << v.name << " $end\n";
      continue;
    }
    bool seen = false;
    for (const auto &s : scopes) seen = seen || s == v.scope;
    if (!seen) scopes.push_back(v.scope);
  }
  for (const auto &s : scopes) {
    out_ << "$scope module " << s << " $end\n";
    for (const Var &v : vars_)
      if (v.scope == s)
        out_ << "$var wire " << v.width << ' ' << v.code << ' ' << v.name << " $end\n";
    out_ << "$upscope $end\n";
  }
  out_ << "$upscope $end\n$enddefinitions $end\n";
}

void VcdWriter::commit(uint64_t time) {
  if (!started_) {
    header();
    out_ << "#" << time << "\n$dumpvars\n";
    for (Var &v : vars_) {
      write_value(v);
      v.dirty = false;
    }
    out_ << "$end\n";
    started_ = true;
    return;
  }
  bool any = false;
  for (Var &v : vars_) {
    if (!v.dirty) continue;
    if (!any) out_ << '#' << time << '\n';
    any = true;
    write_value(v);
    v.dirty = false;
  }
}

}  // namespace systolique
