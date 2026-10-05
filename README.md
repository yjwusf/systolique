# systolique

A cycle-exact C++ model of the systolic array of [Gemmini](https://github.com/ucb-bar/gemmini)
(v0.7.2, `709bc56`): PE, Tile, Mesh, Transposer, tag queues and MeshWithDelays as plain C++17
classes with a per-cycle `tick()`, and `SystolicArray`, which runs them and says, for every cycle
and every PE, what the PE does (one MAC slot per PE per cycle, a closed set of states), which
request's row it works on, and where every value in its registers came from (the op whose weight
a weight-stationary PE holds). It is checked against Gemmini's RTL: stored traces of the
Verilated RTL replay through the classes with every output port equal in every cycle, and an
optional bench runs the RTL and the model in lockstep with their internal registers compared
through VPI.

The core library uses the standard library only. There is no SystemC, no Gemmini frontend
(controllers, scratchpad, DMA, RoCC) and no CPU model here.

## Build and test

```sh
cmake -S . -B build            # C++17, CMake >= 3.16; zlib for the stored traces
cmake --build build -j
ctest --test-dir build         # offline tests: a few seconds
ci/local.sh                    # clean build with -Werror, all tests, sanitizer build, leak check
```

The optional RTL part (label `rtl`) is built when Verilator 5 (`~/opt/verilator-5.038`,
`VERILATOR_ROOT` or the PATH) and the generated Verilog (`-DSYSTOLIQUE_VERILOG_DIR`, default
`~/opt/gemmini-verilog-709bc56`) are found; otherwise CMake prints a warning and those tests are
skipped. How to elaborate the Verilog with the pinned toolchain: [rtl/README.md](rtl/README.md).

| ctest | label | what |
|---|---|---|
| `systolique_trace_<config>` | trace | the 86 stored RTL traces: replay, regeneration from their stimuli, matmul results, conservation, WS provenance |
| `systolique_reference_provenance` | trace | the traces are the recorded files; configurations and generator sources as recorded |
| `systolique_conservation_<config>` | conservation | 1 PE = 1 MAC per cycle, states sum to DIM² every cycle, useful MACs = workload MACs |
| `systolique_provenance_<config>` | provenance | every register equals the operand element its provenance names, every cycle (WS and OS) |
| `systolique_classes`, `systolique_accounting_example` | unit | class rules (two-phase clock, independence, RAII), the perf_reports example |
| `systolique_vcd`, `systolique_view` | vcd, view | the VCD writer changes nothing; the HTML viewer and its committed examples |
| `rtl_<config>`, `rtl_corr_fault_<kind>`, `rtl_verilog` | rtl (optional) | live lockstep with the Verilated RTL, fault injection, Verilog digests |

## Use

```cpp
#include "systolique/systolic_array.h"
using namespace systolique;

SystolicArray array(*find_config("default"));  // 16x16, int8 in, int20 out, OS and WS
array.reset();                                  // 4 reset cycles; cycle() == 0
MwdIn in;
in.resize(array.config());
// each cycle: read array.out() (a/b/d/req ready, resp), choose valid bits and data, then
array.set_inputs(in);
array.tick();
Accounting acc = array.accounting();            // PE-cycles per state, per cycle / PE / request
```

The classes, their two-phase clock, cycle numbering, PE states, occupancy, provenance, the
viewer and the VCD writer: [docs/systolic_array.md](docs/systolic_array.md). Gemmini's
microarchitecture and the register-by-register correspondence with the RTL:
[docs/microarchitecture.md](docs/microarchitecture.md). What is validated and how:
[docs/validation.md](docs/validation.md). Figures: [perf_reports/](perf_reports/README.md).

## Layout

| path | contents |
|---|---|
| `include/systolique/`, `src/` | the library: `PE`, `Tile`, `Mesh`, `Transposer`, `TagQueue`/`RowsQueue`, `MeshWithDelays`, `SystolicArray`, `VcdWriter`; configurations, port types, SInt arithmetic |
| `bench/` | test support: RTL port frames, trace I/O (zlib), stimuli (the test catalog), reference matmuls, replay/run helpers |
| `tests/` | test programs; `tests/reference/gemmini_rtl/` the stored RTL traces and their provenance |
| `tools/` | `array_dump.cpp` (a run as JSON), `array_view.py` + `array_view.html` (the HTML viewer), `rtl_provenance.py` (trace and Verilog provenance) |
| `rtl/` | optional: elaboration of Gemmini's RTL (sbt/Chisel), the Verilator lockstep bench, VPI correspondence |
| `docs/` | documentation; `docs/examples/` two viewer pages |
| `perf_reports/` | figures (not in this README) |
| `ci/local.sh` | local CI |
| `AGENTS.md` | rules for changing this repository |

## Regenerating

- **Viewer pages**: `cmake --build build --target systolique_dump`, then
  `python3 tools/array_view.py --build build --config dim4 --scenario ws_stream --tiles 3 --out
  docs/examples/array_ws_dim4.html` and the same with `--config default --tiles 4 --out
  docs/examples/array_ws_dim16.html` (`systolique_view` fails until the committed pages match).
  Any scenario or request file: [docs/systolic_array.md](docs/systolic_array.md#7-the-viewer).
- **Traces from their stimuli**: `systolique_trace_<config>` regenerates every trace from its
  stimulus on the model and compares it byte for byte with the stored one (no files written).
- **Traces from the RTL**: `rtl/elaborate.sh`, build with Verilator, then
  `python3 tools/rtl_provenance.py record --build build` ([rtl/README.md](rtl/README.md)).

## License

The model is derived from Gemmini, Copyright (c) 2018-2019, The Regents of the University of
California, under the BSD-3-Clause license: its text is in [LICENSE.gemmini](LICENSE.gemmini),
and [NOTICE](NOTICE) lists the files derived from or generated with Gemmini; those files keep
Gemmini's terms. Everything else is licensed under the [Apache License, Version 2.0](LICENSE).
