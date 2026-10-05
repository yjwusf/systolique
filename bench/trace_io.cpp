#include "trace_io.h"

#include <zlib.h>

#include <sstream>

namespace systolique {

bool write_trace(const std::string &path, const std::vector<PortSpec> &spec,
                 const TraceInfo &info, const std::vector<Frame> &rows, std::string &err) {
  gzFile g = gzopen(path.c_str(), "wb9");
  if (!g) {
    err = path + ": cannot write";
    return false;
  }
  std::string s;
  for (const auto &c : info.comments) s += "# " + c + "\n";
  s += "cycle";
  for (const auto &p : spec) s += "," + p.name;
  s += "\n";
  for (size_t r = 0; r < rows.size(); ++r) {
    s += std::to_string(r);
    for (const auto &b : rows[r]) s += "," + b.hex();
    s += "\n";
  }
  const bool ok = gzwrite(g, s.data(), unsigned(s.size())) == int(s.size());
  if (gzclose(g) != Z_OK || !ok) {
    err = path + ": write failed";
    return false;
  }
  return true;
}

bool read_trace(const std::string &path, const std::vector<PortSpec> &spec, TraceInfo &info,
                std::vector<Frame> &rows, std::string &err) {
  gzFile g = gzopen(path.c_str(), "rb");
  if (!g) {
    err = path + ": cannot open";
    return false;
  }
  std::string text;
  char buf[1 << 16];
  int n;
  while ((n = gzread(g, buf, sizeof buf)) > 0) text.append(buf, size_t(n));
  gzclose(g);
  std::istringstream in(text);
  std::string line, header = "cycle";
  for (const auto &p : spec) header += "," + p.name;
  bool have_header = false;
  unsigned lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    if (line.empty()) continue;
    if (line[0] == '#') {
      info.comments.push_back(line.size() > 2 ? line.substr(2) : "");
      continue;
    }
    if (!have_header) {
      if (line != header) {
        err = path + ": the columns are not the ports of this top and configuration";
        return false;
      }
      have_header = true;
      continue;
    }
    Frame f = make_frame(spec);
    std::istringstream ls(line);
    std::string field;
    std::getline(ls, field, ',');
    if (field != std::to_string(rows.size())) {
      err = path + ":" + std::to_string(lineno) + ": cycle " + field + " out of order";
      return false;
    }
    for (size_t p = 0; p < spec.size(); ++p)
      if (!std::getline(ls, field, ',') || !f[p].parse_hex(field)) {
        err = path + ":" + std::to_string(lineno) + ": bad value for " + spec[p].name;
        return false;
      }
    rows.push_back(std::move(f));
  }
  if (!have_header) {
    err = path + ": no header";
    return false;
  }
  return true;
}

}  // namespace systolique
