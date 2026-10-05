# Performance reports

Figures the model produces, kept out of the main README. Every number here is asserted or
printed by a test named next to it; the definitions (cycle numbering, PE states, windows) are in
[docs/systolic_array.md](../docs/systolic_array.md).

## Accounting example: 4 back-to-back WS matmuls on the 16x16 array

Configuration `default` (Gemmini's: 16x16 PEs, int8 inputs, int20 outputs, BOTH dataflows).
`systolique_dump --config default --scenario ws_stream --tiles 4`: `ws_case` of
`bench/reference.cpp`, seed 1; request j preloads W_j while computing A_{j-1} W_{j-1}; 5 requests
of 16 rows; the driver's idle tail is part of the run. Asserted by ctest
`systolique_accounting_example`; cycles counted from reset release (cycle 0).

| op | what | accept | first in | last in | first out | last out | result rows | MAC | load | load_concurrent | MACs |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| 0 | preload W0 | 0 | 1 | 16 | 33 | 48 | 49-64 | 0 | 4,096 | 0 | 0 |
| 1 | compute A0 W0, preload W1 | 16 | 17 | 32 | 49 | 64 | 65-80 | 4,096 | 0 | 4,096 | 4,096 |
| 2 | compute A1 W1, preload W2 | 32 | 33 | 48 | 65 | 80 | 81-96 | 4,096 | 0 | 4,096 | 4,096 |
| 3 | compute A2 W2, preload W3 | 48 | 49 | 64 | 81 | 96 | 97-112 | 4,096 | 0 | 4,096 | 4,096 |
| 4 | compute A3 W3 | 64 | 65 | 80 | 97 | 112 | - | 4,096 | 0 | 0 | 4,096 |

(MAC, load, load_concurrent: PE-cycles of the request's rows; MACs = its useful MACs.)

- Totals: 16,384 MAC PE-cycles = 4 x 16³ (the workload's MACs exactly); 4,096 load; 0 drain,
  0 bubble; 12,288 concurrent loads (MAC PE-cycles that also shift the next weights in).
- Run window 0-160 (161 cycles): occupancy 0.497, utilisation 0.398.
- Busy window 0-111 (112 cycles, first accept to last occupied PE): occupancy 0.714,
  utilisation 0.571.
- The last request's tag has no result rows: it waits for the rows of a request that never comes
  (`MeshWithDelays.scala:219`); the ExecuteController always follows up.
- The same stream in OS (`--scenario os_stream`): 6 requests (preload D0, four computes, the
  flush), 16,384 MAC, 4,096 load, 4,096 drain, 6,528 drain_concurrent PE-cycles; busy window
  0-127, occupancy 0.750, utilisation 0.500.

The per-cycle picture of this run: [docs/examples/array_ws_dim16.html](../docs/examples/array_ws_dim16.html).

## Micro-op example: a 64 x 64 x 64 matmul through the Engine

`Engine` with Gemmini's defaultConfig and the default `DmaParams` (latency 40 cycles, 16 bytes
per cycle, 2 commands per channel: **a model, not validated**, docs/micro_ops.md section 7);
`Matmul{64, 64, 64}`: int8 A, B (seed 1, values in [-8, 8)), no bias, C as int8, the command
stream of gemmini.h's tiled_matmul_auto. Asserted by ctest `systolique_engine_example`; cycles
from reset release (cycle 0). The command path, ExecuteController, banks and array are
cycle-exact against the RTL given the DMA model's inputs; every figure below depends on that
model through the mvins' timing.

| | WS (`gemmini_loop_ws`) | OS (explicit commands) |
|---|---:|---:|
| run (cycle 0 to the fence finding Gemmini idle) | 1,837 cycles (0-1836) | 1,957 cycles (0-1956) |
| commands the core sends (and a fence) | 11, cycles 0-10 | 159, cycles 0-1617 |
| busy window (first request accepted to the last occupied PE) | 197-1585 (1,389 cycles) | 379-1695 (1,317 cycles) |
| occupancy / utilisation, run window | 0.568 / 0.557 | 0.556 / 0.523 |
| occupancy / utilisation, busy window | 0.752 / 0.737 | 0.826 / 0.778 |
| MAC PE-cycles (= useful MACs = 64³) | 262,144 | 262,144 |
| load / drain / bubble PE-cycles | 5,120 / 0 / 0 | 4,096 / 12,288 / 0 |
| concurrent loads (MAC PE-cycles that also take the next D) | 253,952 | 258,048 |
| command micro-ops | 5 config, 6 loop_ws, 1 fence; from LoopMatmul: 8 mvin, 64 preload, 16 compute_preloaded, 48 compute_accumulated, 4 mvout | 7 config, 1 fence, 8 mvin, 64 preload, 16 compute_preloaded, 48 compute_accumulated, 16 mvout |
| array passes (Request micro-ops) | 2 preload, 62 compute + preload, 2 compute | 1 preload, 63 compute + preload, 1 compute, 3 flush |
| operand rows read / result rows / DMA rows | 1,280 / 1,024 / 768 | 2,048 / 256 / 768 |
| computes: first issued, last completed | 190, 1549 | 376, 1607 |
| computes: cycles issued to started (sum over 64) | 4,661 | 4,892 |
| computes: cycles started to popped (sum; longest) | 1,337; 189 | 1,024; 16 |

- 1 PE = 1 MAC per cycle: the 64 compute micro-ops have 4,096 MAC PE-cycles each; every MAC
  PE-cycle is attributed to one of them (`Engine::check`), and the per-micro-op and
  per-operation counts sum to the totals.
- WS reads A for every compute and B (the weights) only for the 16 preloads with i = 0
  (LoopMatmul.scala:412-430); every preload names its C in the accumulator, so all 1,024 result
  rows are written there (and accumulated over k). OS reads A and B for every compute; only the
  16 preloads with k = K-1 name C.
- A compute waits 73 (WS) / 76 (OS) cycles on average from its issue to the start of its pass.
  The longest WS pass, the first compute's (cycles 208-396), reads its A rows only in 381-396:
  173 mvin rows are written into its A bank (bank 0) during the pass, and the single-ported
  bank takes no read while it is written (docs/micro_ops.md, Findings).
- The OS stream ends with three flush passes (the 12,288 drain PE-cycles).

Interactive per-cycle view of a smaller Engine run (two WS matmuls, 32x16x32 and 16x32x16):
[docs/examples/engine_ws_two_matmuls.html](../docs/examples/engine_ws_two_matmuls.html); any run:
`tools/array_view.py --build build --matmul 64x64x64:WS --out page.html`.

## Validation summary

Details and the log: [docs/validation.md](../docs/validation.md).

| check | test | result |
|---|---|---|
| stored RTL traces replayed, every output port every cycle | `systolique_trace_<config>` | 86 traces, 29,804 cycles, 5 configurations, 0 mismatches |
| traces regenerated from their stimuli | `systolique_trace_<config>` | 86 of 86 identical |
| matmul results vs plain C++ | `systolique_trace_<config>` | 10,000 values |
| conservation (1 PE = 1 MAC per cycle; MAC + load + drain + bubble + idle = DIM² per cycle; useful MACs = K x DIM³) | `systolique_conservation_<config>` | 78 runs, 0 errors |
| provenance (register = the W / D / C element it names, every cycle) | `systolique_provenance_<config>`, `systolique_trace_<config>` | 2,038,938 + 615,030 checks, 0 failures |
| live lockstep with the Verilated RTL (optional) | `rtl_<config>` | 286 runs, 161,811 cycles, 145,326,126 internal register comparisons, 0 mismatches |
| frontend: stored RTL traces of ExecuteTop / CmdTop / CtrlTop replayed and regenerated, every output port lane every cycle | `systolique_fe_trace_<top>` | 97 traces, 127,259 cycles, 14,603,951 comparisons, 0 mismatches; 97 of 97 regenerated |
| frontend live lockstep (optional) | `rtl_fe_<top>` | 681 runs, 788,441 cycles, 78,557,176 comparisons, 0 mismatches |
| the splitter (micro-op cycles derived from the Chisel, attribution, conservation, 64³ matmuls) | `systolique_micro_ops` | 7,672 checks, 0 failures |
