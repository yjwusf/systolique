#!/usr/bin/env bash
# Elaborate Gemmini's Mesh and MeshWithDelays (every configuration in src/GemminiTops.scala) to
# Verilog with Chisel 3.6.0's Scala FIRRTL compiler, from pinned, unmodified sources:
#
#   GEMMINI_DIR     ucb-bar/gemmini v0.7.2 (709bc56)     default ~/opt/src/gemmini
#   HARDFLOAT_DIR   ucb-bar/berkeley-hardfloat 9deaf1d   default ~/opt/src/berkeley-hardfloat-9deaf1d
#   OUT             output directory                     default ~/opt/gemmini-verilog-709bc56
#   WORK            sbt work directory (target/, caches) default ~/opt/build/gemmini-elab
#   JAVA_HOME       a JDK 17                             default Homebrew openjdk@17
#   SBT             sbt launcher                         default ~/opt/sbt-1.10.11/bin/sbt
#
#   rtl/elaborate.sh [config ...]     (no arguments: all configurations)
#
# Writes OUT/<config>/{MeshTop,MeshWithDelaysTop}.v, OUT/provenance.json (sources, tools, digest
# of every generated file; tools/rtl_provenance.py check-verilog compares it with the provenance
# of the stored reference traces) and OUT/<config>/correspondence.txt (the RTL shift registers
# the lockstep bench compares with the model's merged state). The generated files stay out of
# git. build.sbt, project/build.properties and src/GemminiTops.scala are byte for byte the
# generator sources the stored traces were made with (their sha256 is in
# tests/reference/gemmini_rtl/provenance.json; ctest systolique_reference_provenance checks it),
# so their comments still name the directory they were first written in.
#   git clone --branch v0.7.2 https://github.com/ucb-bar/gemmini.git ~/opt/src/gemmini
#   curl -sSL https://codeload.github.com/ucb-bar/berkeley-hardfloat/tar.gz/9deaf1d49f487347be371641ba6ea637b669ad21 |
#     tar xz -C ~/opt/src/berkeley-hardfloat-9deaf1d --strip-components=1
#   brew install openjdk@17; sbt 1.10.11 from https://github.com/sbt/sbt/releases into ~/opt/sbt-1.10.11
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
export GEMMINI_DIR="${GEMMINI_DIR:-$HOME/opt/src/gemmini}"
export HARDFLOAT_DIR="${HARDFLOAT_DIR:-$HOME/opt/src/berkeley-hardfloat-9deaf1d}"
OUT="${OUT:-$HOME/opt/gemmini-verilog-709bc56}"
WORK="${WORK:-$HOME/opt/build/gemmini-elab}"
export JAVA_HOME="${JAVA_HOME:-/opt/homebrew/opt/openjdk@17}"
SBT="${SBT:-$HOME/opt/sbt-1.10.11/bin/sbt}"

for f in "$GEMMINI_DIR/src/main/scala/gemmini/Mesh.scala" "$HARDFLOAT_DIR/src/main/scala/common.scala" \
         "$JAVA_HOME/bin/java" "$SBT"; do
  [ -e "$f" ] || { echo "elaborate.sh: missing $f (see the header of this script)"; exit 1; }
done

# sbt writes target/ and project/target/ next to build.sbt: build in a copy outside the repo.
mkdir -p "$WORK/project" "$WORK/src" "$OUT"
cp "$HERE/build.sbt" "$WORK/"
cp "$HERE/project/build.properties" "$WORK/project/"
cp "$HERE/src/"*.scala "$WORK/src/"
cd "$WORK"
"$SBT" -batch -Dsbt.server.forcestart=false "run $OUT $*"

TOOL="$HERE/../tools/rtl_provenance.py"
python3 "$TOOL" provenance --out "$OUT" --gemmini "$GEMMINI_DIR" --hardfloat "$HARDFLOAT_DIR" \
  --java "$JAVA_HOME/bin/java" --sbt "$SBT" --generator "$HERE"
python3 "$TOOL" correspondence --verilog "$OUT"
