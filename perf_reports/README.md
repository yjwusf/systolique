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
