#!/usr/bin/env bash
# Elaborate Gemmini's ExecuteController with its scratchpad and accumulator banks
# (src/ExecuteTop.scala -> $OUT/ExecuteTop.v), its command path, raw_cmd_q, LoopConv, LoopMatmul
# and ReservationStation (src/CmdTop.scala -> $OUT/CmdTop.v), and both wired together
# (src/CtrlTop.scala -> $OUT/CtrlTop.v), to Verilog with Chisel 3.6.0's Scala FIRRTL compiler,
# from pinned, unmodified sources:
#
#   GEMMINI_DIR     ucb-bar/gemmini v0.7.2 (709bc56)          default ~/opt/src/gemmini
#   ROCKETCHIP_DIR  chipsalliance/rocket-chip 67ceb1d         default ~/opt/src/rocket-chip-67ceb1d
#   CDE_DIR         chipsalliance/cde 384c06b                 default ~/opt/src/cde-384c06b
#   HARDFLOAT_DIR   ucb-bar/berkeley-hardfloat 9deaf1d        default ~/opt/src/berkeley-hardfloat-9deaf1d
#   OUT             output directory                          default ~/opt/gemmini-verilog-709bc56/frontend
#   WORK            sbt work directory                        default ~/opt/build/systolique-frontend-elab
#   JAVA_HOME / SBT as for rtl/elaborate.sh
#
#   mkdir -p ~/opt/src/rocket-chip-67ceb1d ~/opt/src/cde-384c06b
#   curl -sSL https://codeload.github.com/chipsalliance/rocket-chip/tar.gz/67ceb1ddbfd1c6f50d2b4fdadf68f304f5e62287 |
#     tar xz -C ~/opt/src/rocket-chip-67ceb1d --strip-components=1
#   curl -sSL https://codeload.github.com/chipsalliance/cde/tar.gz/384c06b8d45c8184ca2f3fba2f8e78f79d2c1b51 |
#     tar xz -C ~/opt/src/cde-384c06b --strip-components=1
#
# Writes $OUT/{ExecuteTop,CmdTop,CtrlTop}.v, plusarg_reader.v (rocket-chip's black box) and
# $OUT/provenance.json (tools/rtl_provenance.py fe-provenance). The generated files stay out of git.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
export GEMMINI_DIR="${GEMMINI_DIR:-$HOME/opt/src/gemmini}"
export ROCKETCHIP_DIR="${ROCKETCHIP_DIR:-$HOME/opt/src/rocket-chip-67ceb1d}"
export CDE_DIR="${CDE_DIR:-$HOME/opt/src/cde-384c06b}"
export HARDFLOAT_DIR="${HARDFLOAT_DIR:-$HOME/opt/src/berkeley-hardfloat-9deaf1d}"
OUT="${OUT:-$HOME/opt/gemmini-verilog-709bc56/frontend}"
WORK="${WORK:-$HOME/opt/build/systolique-frontend-elab}"
export JAVA_HOME="${JAVA_HOME:-/opt/homebrew/opt/openjdk@17}"
SBT="${SBT:-$HOME/opt/sbt-1.10.11/bin/sbt}"

for f in "$GEMMINI_DIR/src/main/scala/gemmini/ExecuteController.scala" \
         "$ROCKETCHIP_DIR/src/main/scala/tile/LazyRoCC.scala" \
         "$CDE_DIR/cde/src/chipsalliance/rocketchip/config.scala" \
         "$HARDFLOAT_DIR/src/main/scala/common.scala" "$JAVA_HOME/bin/java" "$SBT"; do
  [ -e "$f" ] || { echo "elaborate_ex.sh: missing $f (see the header of this script)"; exit 1; }
done

# sbt writes target/ next to build.sbt: build in a copy outside the repository.
mkdir -p "$WORK/project" "$WORK/src" "$WORK/macros" "$OUT"
cp "$HERE/build.sbt" "$WORK/"
cp "$HERE/project/build.properties" "$WORK/project/"
cp "$HERE/src/"*.scala "$WORK/src/"
cd "$WORK"
"$SBT" -batch -Dsbt.server.forcestart=false "root/runMain systolique.ElaborateEx $OUT" \
  "root/runMain systolique.ElaborateCmd $OUT" "root/runMain systolique.ElaborateCtrl $OUT"

python3 "$HERE/../../tools/rtl_provenance.py" fe-provenance --out "$OUT" --gemmini "$GEMMINI_DIR" \
  --rocket-chip "$ROCKETCHIP_DIR" --cde "$CDE_DIR" --hardfloat "$HARDFLOAT_DIR" \
  --java "$JAVA_HOME/bin/java" --sbt "$SBT" --generator "$HERE"
