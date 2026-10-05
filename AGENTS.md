# Rules for working on systolique

For every agent or person changing this repository. They are the owner's requirements; follow
them unless the owner changes them here.

## Structure of the model

- Every hardware block is a C++ class with: a constructor that builds all its state from the
  configuration and allocates the children it owns; an explicit destructor that releases what it
  owns (RAII: `std::unique_ptr`/values, no raw `new`/`delete`, no leaks); `tick()` that advances
  exactly one clock cycle. A parent's `tick()` ticks its children.
- Two-phase update wherever a block's state feeds another block in the same cycle:
  `set_inputs()` (this cycle's inputs), `eval()` (phase 1: every next register value from the
  current registers and the inputs, changing no register), `tick()` (phase 2: commit, then settle
  the combinational values that depend only on registers). `tick()` runs `eval()` if it has not
  run since the inputs were set. Document in the class header which scheme it uses and why; keep
  `systolique_classes` checking that `eval()` changes nothing and `tick()` == `eval()` + `tick()`.
- Inputs are set before the tick, outputs are read after it. Outputs that the RTL computes from
  registers only must stay register-only, so `out()` is valid before the cycle's inputs are
  chosen.
- Interfaces are non-blocking valid/ready: no callbacks that wait, no threads, no coroutines. A
  handshake is a cycle with valid and ready both 1.
- No SystemC or any other dependency in the core library (`include/`, `src/`): C++17 standard
  library only. zlib only in `bench/` (trace files). The RTL part stays optional.
- No global or static mutable state. Two instances must be independent (`systolique_classes`).
- Value types (configuration, port bundles, requests, responses, notes) are small structs without
  behaviour.

## Cycle-exactness

- The reference is Gemmini's RTL at the pinned commit (v0.7.2,
  `709bc56b6dd859fc2b1a9027a96a0b5be6ad7ed6`), elaborated unmodified (`rtl/README.md`).
- Every change must keep all 86 stored RTL traces replaying exactly (every output port, every
  cycle) and regenerating byte for byte from their stimuli (`ctest -L trace`). When Verilator and
  the Verilog are available, run `ctest -L rtl` too (0 mismatches, every fault detected).
- Never claim accuracy that a test has not shown. A number in a document names the test that
  produces it, or is removed.
- Any abstraction of RTL state (merging registers, histories instead of shift registers, a
  different encoding) needs a documented cycle correspondence (an equation that holds every
  cycle, `docs/microarchitecture.md` section 5) and a test that checks it (the VPI
  correspondence of the RTL bench, with a fault-injection test showing the check can fail).
- Comments explain why and cite the Chisel source as `File.scala:line` of the pinned commit.

## Cycles, occupancy, states

- One PE does at most one MAC per cycle (one MacUnit, `PE.scala:64-65`): every PE-cycle is one
  MAC slot; useful MACs = PE-cycles in state MAC.
- Cycle numbering is explicit: cycle 0 is the first cycle with reset low after the last reset
  cycle. Per request, record the accept, first-in, last-in, first-out, last-out and result
  cycles, each defined in `docs/systolic_array.md`.
- Occupancy and utilisation always come with their window (run window, busy window), stated
  wherever a figure is given.
- PE states are a closed set (idle, mac, load, drain, bubble): exactly one per PE-cycle.
  Concurrent work in a MAC cycle (load_concurrent, drain_concurrent) is a flag, never a second
  state or a second MAC.
- Conservation must hold and be tested: MAC + load + drain + bubble + idle = DIM² every cycle;
  per-PE states sum to the cycles; useful MACs equal the workload's MACs exactly
  (`SystolicArray::check`, `systolique_conservation_<config>`).

## Provenance and the viewer

- Keep weight provenance for weight-stationary (each register's value traced to the W element
  and the op it came from) and the OS analogue (D, partial sums, results), checked against values
  every cycle (`systolique_provenance_<config>`).
- The per-cycle HTML viewer (`tools/array_view.py`) shows which op each PE's stationary weight
  (OS: D/result) comes from, the state, the flags and the counters. Its committed examples in
  `docs/examples/` must be what their commands generate (`systolique_view`); regenerate them when
  the output changes.

## Validation notes

- Keep the validation log in `docs/validation.md`: a dated entry for every change that affects
  what is validated, with the toolchain and the figures.
- Annotate parameters in code comments with their source, status (validated / partially
  validated / unvalidated / fitted on <data>) and the test that checks them; keep the comments and
  the table in `docs/validation.md` consistent with each other.

## Waveforms and dumps

- Waveform and trace dumps must be possible and off by default (`ArrayOptions::vcd_path`, the VCD
  writer of `include/systolique/vcd.h`, ports and per-PE state and occupancy). Dumping only reads
  state and must not change timing (`systolique_vcd`).

## References and toolchain

- Pin the toolchain and record the provenance of every stored reference: Gemmini commit, Chisel
  version, generator command and its sources' hashes, Verilator version
  (`tests/reference/gemmini_rtl/provenance.json`, `tools/rtl_provenance.py`). Never edit a stored
  trace by hand; re-record it (`rtl/README.md`) and log why.
- Generated Verilog and build trees stay out of git.

## Documents

- Performance and accounting numbers live in `perf_reports/`, not in `README.md`.
- `README.md` says what the repository is, how to build and test, the layout and how to
  regenerate traces and the viewer pages.

## Tests

- Tests run offline and fast (`ctest -LE rtl`: seconds). Optional RTL parts are skipped with a
  CMake warning when Verilator or the generated Verilog is absent.
- `ci/local.sh` must pass before pushing: warnings are errors for the project's sources
  (`-Wall -Wextra -Werror`), all tests, the sanitizer build and the leak check.

## Commits

- Small, logical commits with plain messages. No AI attribution, no `Co-Authored-By` lines.
- Author: `Wu Yu Jian <wu_yujian@aiap.sg>`
  (`git -c user.name="Wu Yu Jian" -c user.email=wu_yujian@aiap.sg commit ...`).
- Nothing private or unrelated: no other projects' code, credentials, hostnames or local paths
  beyond the documented tool locations.
- Licensing: the Gemmini-derived files are BSD-3-Clause (LICENSE.gemmini, NOTICE); list every new
  derived file in NOTICE. Everything else is Apache-2.0 (LICENSE); new files are Apache-2.0
  unless derived from Gemmini.
