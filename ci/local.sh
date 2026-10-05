#!/usr/bin/env bash
# Local CI: clean configure + build with warnings as errors for the project's sources, every
# ctest (the optional RTL tests run when Verilator and the generated Verilog are found, else
# CMake warns and they are skipped), then a second build with AddressSanitizer and
# UndefinedBehaviorSanitizer running the offline tests, and a leak check of the test programs.
#
#   ci/local.sh                  full run in build-ci/ and build-ci-asan/
#   ci/local.sh -L trace         extra arguments go to the first ctest (here: only label trace)
#   BUILD_DIR=... ci/local.sh    other build directory
#   KEEP_BUILD=1 ci/local.sh     reuse the build directories instead of starting clean
#   SKIP_SANITIZE=1 ci/local.sh  skip the sanitizer build and the leak check
#   JOBS=8 ci/local.sh           build and test with 8 jobs (default: all cores)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build-ci}"
JOBS="${JOBS:-$( (sysctl -n hw.ncpu || nproc) 2>/dev/null)}"
step() { printf '\n== %s\n' "$*"; }

if [ "${KEEP_BUILD:-0}" != 1 ]; then rm -rf "$BUILD"; fi

step configure
mkdir -p "$BUILD"
cmake -S "$ROOT" -B "$BUILD" -DSYSTOLIQUE_WERROR=ON >"$BUILD/configure.log" 2>&1 ||
  { cat "$BUILD/configure.log"; exit 1; }
grep -E "systolique:|Warning" -A2 "$BUILD/configure.log" || true

step build
# (`|| true` would reset PIPESTATUS, so the build status is taken with errexit off instead.)
set +e
cmake --build "$BUILD" -j "$JOBS" >"$BUILD/build.log" 2>&1
build_status=$?
set -e
if [ "$build_status" != 0 ]; then grep -E "error" "$BUILD/build.log" | head -40; echo "build failed"; exit 1; fi
if grep -E "^$ROOT/(include|src|bench|tests|tools|rtl)/.*warning:" "$BUILD/build.log"; then
  echo "compiler warnings in the project's sources"; exit 1
fi

step test
ctest --test-dir "$BUILD" -j "$JOBS" --output-on-failure "$@" | grep -vE "^ +Start|Test +#.*Passed"

if [ "${SKIP_SANITIZE:-0}" != 1 ]; then
  step "configure + build + offline tests with -DSYSTOLIQUE_SANITIZE=ON"
  BASAN="$BUILD-asan"
  if [ "${KEEP_BUILD:-0}" != 1 ]; then rm -rf "$BASAN"; fi
  mkdir -p "$BASAN"
  cmake -S "$ROOT" -B "$BASAN" -DSYSTOLIQUE_WERROR=ON -DSYSTOLIQUE_SANITIZE=ON -DSYSTOLIQUE_RTL=OFF \
    >"$BASAN/configure.log" 2>&1 || { cat "$BASAN/configure.log"; exit 1; }
  cmake --build "$BASAN" -j "$JOBS" >"$BASAN/build.log" 2>&1 || { tail -50 "$BASAN/build.log"; exit 1; }
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ASAN_OPTIONS=halt_on_error=1 \
    ctest --test-dir "$BASAN" -j "$JOBS" --output-on-failure | grep -vE "^ +Start|Test +#.*Passed"

  # Leaks: macOS `leaks` on the plain build (LeakSanitizer is not available there).
  if command -v leaks >/dev/null 2>&1; then
    step "leak check (leaks --atExit)"
    REF="$ROOT/tests/reference/gemmini_rtl"
    for cmd in "classes_test" "trace_test --ref-dir $REF --config tiled" \
               "conservation_test --config dim4" "provenance_test --config dim4" \
               "example_test" "micro_ops_test --out $BUILD" \
               "fe_trace_test --ref-dir $ROOT/tests/reference/gemmini_fe --top CmdTop"; do
      # shellcheck disable=SC2086
      out="$(leaks --atExit -- "$BUILD"/$cmd 2>&1)" || { echo "$out" | tail -30; echo "leaks: $cmd"; exit 1; }
      echo "$out" | grep -E "^Process [0-9]+: [0-9]+ leaks" | sed "s|^|$cmd: |"
    done
  fi
fi

step "ok"
