#include "systolique/types.h"

namespace systolique {

void MeshIn::resize(const ArrayConfig &cfg) {
  a.assign(cfg.rows(), 0);
  b.assign(cfg.cols(), 0);
  d.assign(cfg.cols(), 0);
  control.assign(cfg.cols(), PeControl{});
  id.assign(cfg.cols(), 0);
  last.assign(cfg.cols(), 0);
  valid.assign(cfg.cols(), 0);
}

void MeshOut::resize(const ArrayConfig &cfg) {
  b.assign(cfg.cols(), 0);
  c.assign(cfg.cols(), 0);
  valid.assign(cfg.cols(), 0);
  id.assign(cfg.cols(), 0);
  last.assign(cfg.cols(), 0);
  control.assign(cfg.cols(), PeControl{});
}

bool MeshOut::operator==(const MeshOut &o) const {
  return b == o.b && c == o.c && valid == o.valid && id == o.id && last == o.last &&
         control == o.control;
}

void MwdIn::resize(const ArrayConfig &cfg) {
  a.assign(cfg.rows(), 0);
  b.assign(cfg.cols(), 0);
  d.assign(cfg.cols(), 0);
}

void MwdOut::resize(const ArrayConfig &cfg) {
  resp_data.assign(cfg.cols(), 0);
  tags_valid.assign(cfg.tagq_len(), 0);
  tags_id.assign(cfg.tagq_len(), 0);
}

}  // namespace systolique
