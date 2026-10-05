# Validation

What the model is checked against, by which test, with which result. The reference is the RTL of
Gemmini v0.7.2 (`709bc56b6dd859fc2b1a9027a96a0b5be6ad7ed6`), elaborated unmodified with the
pinned toolchain of `rtl/README.md` and simulated with Verilator 5.038 (`--x-initial 0`). A
claim of accuracy in this repository is only as good as the test named next to it.

## Checks

| what | test (ctest, label) | figures (last run, see the log) |
|---|---|---|
| every stored RTL trace replayed through `SystolicArray`: every output port equal in every cycle (MeshTop traces through the bare Mesh interface) | `systolique_trace_<config>` (trace) | 86 traces, 29,804 cycles, 5 configurations, 0 mismatches |
| each trace's stimulus, run on `SystolicArray`, regenerates the trace exactly (inputs and outputs) | `systolique_trace_<config>` | 86 of 86 |
| matmul results (the recorded RTL responses) equal a plain C++ matmul | `systolique_trace_<config>` | 10,000 result values |
| WS-only runs: every register equals the d value its provenance names, every cycle | `systolique_trace_<config>` | 615,030 register checks |
| the stored traces are the recorded files (sha256, cycles), the configurations equal the Verilog's, the generator sources are the recorded ones | `systolique_reference_provenance` (trace) | 86 traces |
| conservation: `SystolicArray::check` and an independent recount of every PE-cycle (one state each, at most one MAC, MAC + load + drain + bubble + idle = DIM² every cycle, per-PE and per-cycle counts equal the accounting); useful MACs = K x DIM³ for OS/WS streams of K = 1, 4, 16 with and without input bubbles; MACs = rows x DIM² per computing request; random request mixes | `systolique_conservation_<config>` (conservation) | 78 runs, 0 errors |
| provenance against values, every PE, both registers, every cycle: random WS streams (A or W transposed, bubbles): each register equals W_k[w_row][w_col] of the matmul it names, every MAC at PE (i, j) uses W_k[i][j] of the matmul whose A the request carries, loaded < active <= last_used; random OS streams: D on its way down equals D_k[w_row][w_col], an accumulator that has taken all DIM rows equals (A_k B_k + D_k)[i][j], a result on its way out equals it, every MAC accumulates into D_k[i][j] of the matmul computed | `systolique_provenance_<config>` (provenance) | 2,038,938 checks, 0 failures |
| the class rules: PE/Tile/Transposer/queue behaviour, `eval()` changes no register or output, `tick()` alone = `eval()` + `tick()`, outputs depend on registers only, instances independent, `reset()` reproduces a run, invalid configurations throw | `systolique_classes` (unit) | |
| the accounting example of perf_reports/ | `systolique_accounting_example` (unit) | |
| the VCD writer changes no output and no count; its files are well formed | `systolique_vcd` (vcd) | 46 traces, 15,032 cycles |
| the viewer pages are well formed, offline, consistent; the committed examples are what their commands generate | `systolique_view` (view) | 4 pages |
| **live lockstep** (optional): every output port of `SystolicArray` and the Verilated RTL, every cycle; the RTL checked to depend on registers only; the internal correspondence of docs/microarchitecture.md section 5 through VPI before every edge; the RTL reproduces the stored traces; accounting conserved | `rtl_<config>` (rtl) | 286 runs (catalog + 20 random seeds per configuration), 161,811 cycles, 145,326,126 register comparisons, 0 mismatches |
| the correspondence check sees a wrong value of each kind (exact, feed, resp, valid, pipev) | `rtl_corr_fault_<kind>` (rtl) | 5 of 5 detected |
| the Verilog is the one the traces were made from | `rtl_verilog` (rtl) | 10 files |

## Parameters

Every parameter of the model with its source, status and the test that checks it (the same
annotations are in `include/systolique/config.h` and `src/config.cpp`). Status: *validated* = a
test compares the model with the RTL at this value; *partially validated* = only some values or
some uses are compared; *unvalidated* = no test compares it with the RTL. Nothing is fitted:
the model has no free parameter.

| parameter | value(s) | source | status | test |
|---|---|---|---|---|
| `in_bits` (inputType) | 8, 16 | `Configs.scala:23`; `GemminiTops.scala` | validated at 8 and 16; unvalidated elsewhere in 2..16 | `systolique_trace_*`, `rtl_*` |
| `out_bits` (spatialArrayOutputType) | 20, 24 | `Configs.scala:26` | validated at 20 and 24; unvalidated elsewhere | same |
| `acc_bits` (accType) | 32 | `Configs.scala:24` | validated at 32 only | same |
| `dataflow` | BOTH, WS, OS | `Configs.scala:35` | validated (all three) | same |
| `tile_rows` x `tile_cols` | 1x1, 2x2 | `Configs.scala:29-30` | validated at 1x1 and 2x2; non-square tiles unvalidated | same |
| `mesh_rows` x `mesh_cols` | 16x16, 4x4, 8x8 (DIM 4, 8, 16) | `Configs.scala:31-32` | validated at DIM 4, 8, 16; DIM 2, 32, 64 unvalidated | same |
| `tile_latency` | 0, 1 | `GemminiConfigs.scala:78` | validated at 0 and 1; 2..4 unvalidated | same |
| `output_delay` | 1, 2 | `GemminiConfigs.scala:79` | validated at 1 and 2; 0, 3, 4 unvalidated | same |
| tree reduction | WS-only, tileRows > 1 | `GemminiConfigs.scala:175` | validated (`ws_tree`) | same |
| `max_simultaneous_matmuls` | 5 | `MeshWithDelays.scala:48-54` | validated at 5 (every named configuration); the formula's branch above 5 unvalidated | same |
| `tag_bits` (bench tag) | 8 | `GemminiTops.scala` `BenchTag` | validated at 8 | same |
| initial register values | 0 | Verilator `--x-initial 0` | validated (all runs start from it) | same |

## Log

Newest first. Each entry: date, what was run, toolchain, result.

### 2026-10-05: first standalone run

- Built with Apple clang 17.0.0 (arm64-apple-darwin24.3.0), CMake 4.3.0, C++17, `-Wall -Wextra
  -Werror` on the project's sources; Python 3.14.8 for the viewer and provenance tests;
  Verilator 5.038 (2025-07-08) with the Verilog of `~/opt/gemmini-verilog-709bc56` (digests equal
  to `tests/reference/gemmini_rtl/provenance.json`: `rtl_verilog`).
- The classes (PE, Tile, Mesh, Transposer, TagQueue, RowsQueue, MeshWithDelays, SystolicArray)
  replaced the earlier flat model; every figure in the table above was reproduced unchanged:
  86 traces / 29,804 cycles replayed and regenerated, 10,000 matmul values, 615,030 WS register
  checks, 78 conservation runs, 2,038,938 provenance checks, live lockstep 286 runs / 161,811
  cycles / 145,326,126 register comparisons, 0 mismatches; the viewer data of the two examples
  identical to those of the earlier model.
- `ctest -LE rtl`: 20 tests, about 2 s; `ctest -L rtl`: 11 tests, about 16 s on 6 jobs.
- `rtl/elaborate.sh` (sbt 1.10.11, OpenJDK 17, Chisel 3.6.0) run into a scratch directory
  regenerated all 10 Verilog files with the digests recorded in `provenance.json`
  (`tools/rtl_provenance.py check-verilog`: `VERILOG ok`) and the same correspondence maps;
  the generator sources' sha256 equal the recorded ones.
- `ci/local.sh`: also the sanitizer build (ASan + UBSan) of the offline tests and macOS `leaks`
  on five test programs: 0 errors, 0 leaks.

## Provenance of the references

`tests/reference/gemmini_rtl/provenance.json` records, for the stored traces: the Gemmini
repository, tag, commit and the files elaborated; berkeley-hardfloat's commit; Chisel 3.6.0 with
the Scala FIRRTL compiler, Scala 2.13.10, the Java version; the generator command and the sha256
of its sources (`rtl/build.sbt`, `rtl/project/build.properties`, `rtl/src/GemminiTops.scala`);
the configurations; a digest of every generated Verilog file; the Verilator version that ran the
lockstep; sha256 and cycle count of every trace. The traces were recorded by the same lockstep
bench before this repository existed (their comment lines name that bench); they are kept byte
for byte so the recorded hashes stay valid. (The `sbt` field holds the first line sbt printed,
not its version; the version is pinned in `rtl/project/build.properties`: 1.10.11.)
