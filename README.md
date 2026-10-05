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

On top of the array, Gemmini's frontend splits operations into the micro-ops Gemmini executes:
the command path (raw command queue, LoopMatmul, ReservationStation) and the ExecuteController
with its scratchpad and accumulator banks, as C++ classes checked port by port against the RTL
of Gemmini's controllers. An `Engine` takes Gemmini command streams or whole matmuls and gives
their cycle-by-cycle timing, a table of micro-ops (commands, array passes, operand reads, result
rows, write-backs, DMA rows) with their cycles, and the attribution of every PE-cycle and every
bank access to a micro-op and an operation. Data movement to and from DRAM (DMA, load / store
controllers) is a simple latency / bandwidth model, not validated against the RTL.

The core library uses the standard library only. There is no SystemC and no CPU model here.

## Build and test

```sh
cmake -S . -B build            # C++17, CMake >= 3.16; zlib for the stored traces
cmake --build build -j
ctest --test-dir build         # offline tests: a few seconds
ci/local.sh                    # clean build with -Werror, all tests, sanitizer build, leak check
```

The optional RTL part (label `rtl`) is built when Verilator 5 (`~/opt/verilator-5.038`,
`VERILATOR_ROOT` or the PATH) and the generated Verilog (`-DSYSTOLIQUE_VERILOG_DIR`, default
`~/opt/gemmini-verilog-709bc56`; `-DSYSTOLIQUE_FE_VERILOG_DIR`, default
`~/opt/gemmini-verilog-709bc56/frontend`, for the frontend) are found; otherwise CMake prints a
warning and those tests are skipped. How to elaborate the Verilog with the pinned toolchain:
[rtl/README.md](rtl/README.md).

| ctest | label | what |
|---|---|---|
| `systolique_trace_<config>` | trace | the 86 stored RTL traces: replay, regeneration from their stimuli, matmul results, conservation, WS provenance |
| `systolique_reference_provenance` | trace | the traces are the recorded files; configurations and generator sources as recorded |
| `systolique_conservation_<config>` | conservation | 1 PE = 1 MAC per cycle, states sum to DIM² every cycle, useful MACs = workload MACs |
| `systolique_provenance_<config>` | provenance | every register equals the operand element its provenance names, every cycle (WS and OS) |
| `systolique_classes`, `systolique_accounting_example` | unit | class rules (two-phase clock, independence, RAII), the perf_reports example |
| `systolique_vcd`, `systolique_view` | vcd, view | the VCD writer changes nothing; the HTML viewer and its committed examples |
| `systolique_fe_trace_<top>`, `systolique_fe_reference_provenance` | trace | the 97 stored frontend RTL traces (ExecuteTop, CmdTop, CtrlTop): replay, regeneration, matmul results; their provenance |
| `systolique_micro_ops`, `systolique_engine_example` | unit, conservation | the splitter's micro-op cycles, attribution and conservation, 64³ WS / OS matmuls, recording changes nothing; the perf_reports example |
| `rtl_<config>`, `rtl_corr_fault_<kind>`, `rtl_verilog` | rtl (optional) | live lockstep with the Verilated RTL, fault injection, Verilog digests |
| `rtl_fe_<top>`, `rtl_fe_reference_<top>`, `rtl_fe_fault_<top>`, `rtl_fe_verilog` | rtl (optional) | live lockstep of the frontend classes with the Verilated ExecuteTop / CmdTop / CtrlTop |

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

```cpp
#include "systolique/engine.h"
Engine e;                                       // Gemmini's defaultConfig, the modelled DMA
int op = e.submit(Matmul{64, 64, 64, kWS});     // M, K, N; gemmini.h's command stream
e.run();                                        // until every operation is done
const MicroOpTable &t = e.micro_ops();          // micro-ops with their cycles and parents
Accounting a = e.accounting();                  // per_op, per_uop: PE-cycles per state
```

The classes, their two-phase clock, cycle numbering, PE states, occupancy, provenance, the
viewer and the VCD writer: [docs/systolic_array.md](docs/systolic_array.md). The micro-ops, the
splitting rules with their Chisel sources and what is exact and what is modelled:
[docs/micro_ops.md](docs/micro_ops.md). Gemmini's
microarchitecture and the register-by-register correspondence with the RTL:
[docs/microarchitecture.md](docs/microarchitecture.md). What is validated and how:
[docs/validation.md](docs/validation.md). Figures: [perf_reports/](perf_reports/README.md).

## Layout

| path | contents |
|---|---|
| `include/systolique/`, `src/` | the library: `PE`, `Tile`, `Mesh`, `Transposer`, `TagQueue`/`RowsQueue`, `MeshWithDelays`, `SystolicArray`, `VcdWriter`; the frontend: `ExecuteController`, `ScratchpadBank`/`Scratchpad`, `AccumulatorBank`/`Accumulator`, `ReservationStation`, `LoopMatmul`, `CommandPath`, `ExecuteUnit`, `Controller`, `Host` (modelled DMA), `Engine`, the micro-op table; configurations, port types, commands, SInt arithmetic |
| `bench/` | test support: RTL port frames, trace I/O (zlib), stimuli (the test catalogs of the array and of the frontend tops), reference matmuls, replay/run helpers |
| `tests/` | test programs; `tests/reference/gemmini_rtl/` the stored array RTL traces, `tests/reference/gemmini_fe/` the stored frontend RTL traces, each with its provenance |
| `tools/` | `array_dump.cpp` (a run as JSON, also an Engine run), `array_view.py` + `array_view.html` (the HTML viewer), `rtl_provenance.py` (trace and Verilog provenance) |
| `rtl/` | optional: elaboration of Gemmini's RTL (sbt/Chisel; `rtl/ex/` the frontend tops), the Verilator lockstep benches, VPI correspondence |
| `docs/` | documentation; `docs/examples/` three viewer pages |
| `perf_reports/` | figures (not in this README) |
| `ci/local.sh` | local CI |
| `AGENTS.md` | rules for changing this repository |

## Regenerating

- **Viewer pages**: `cmake --build build --target systolique_dump`, then
  `python3 tools/array_view.py --build build --config dim4 --scenario ws_stream --tiles 3 --out
  docs/examples/array_ws_dim4.html` and the same with `--config default --tiles 4 --out
  docs/examples/array_ws_dim16.html`; `--build build --matmul 32x16x32:WS --matmul 16x32x16:WS
  --out docs/examples/engine_ws_two_matmuls.html` for the Engine example (`systolique_view` fails
  until the committed pages match).
  Any scenario or request file: [docs/systolic_array.md](docs/systolic_array.md#7-the-viewer).
- **Traces from their stimuli**: `systolique_trace_<config>` regenerates every trace from its
  stimulus on the model and compares it byte for byte with the stored one (no files written).
- **Traces from the RTL**: `rtl/elaborate.sh`, build with Verilator, then
  `python3 tools/rtl_provenance.py record --build build`; the frontend's: `rtl/ex/elaborate_ex.sh`,
  then `python3 tools/rtl_provenance.py fe-record --build build` ([rtl/README.md](rtl/README.md)).

## License

The model is derived from Gemmini, Copyright (c) 2018-2019, The Regents of the University of
California, under the BSD-3-Clause license: its text is in [LICENSE.gemmini](LICENSE.gemmini),
and [NOTICE](NOTICE) lists the files derived from or generated with Gemmini; those files keep
Gemmini's terms. Everything else is licensed under the [Apache License, Version 2.0](LICENSE).
