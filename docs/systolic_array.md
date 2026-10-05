# The classes, the clock, and SystolicArray: cycles, occupancy and provenance

Gemmini's systolic array (MeshWithDelays with its Mesh of Tiles of PEs, ucb-bar/gemmini v0.7.2,
`709bc56`) as plain C++ classes, and `SystolicArray`, which says per cycle and per PE what the
array does, counts it, and tracks where every value in every PE register came from. A viewer
(`tools/array_view.py`) turns a run into one self-contained HTML page; a VCD writer dumps the
ports and the PE states for a waveform viewer.

Chisel citations are `file:line` in `src/main/scala/gemmini/` of v0.7.2; the microarchitecture is
described in [microarchitecture.md](microarchitecture.md).

## 1. Classes and the clock

Every hardware block is a class (`include/systolique/`):

| class | constructor | destructor | `eval()` (phase 1) | `tick()` (phase 2) |
|---|---|---|---|---|
| `PE` | widths and dataflow from the configuration; c1, c2, last_s = 0 | nothing owned beyond its registers | out_b, out_c and the next c1/c2/last_s from the inputs and the registers | registers := next |
| `Tile` | allocates tileRows x tileColumns `PE`s | releases the PEs | evaluates each PE column top to bottom (b and d flow down through the PEs in the same cycle) | ticks every PE |
| `Mesh` | validates the configuration; allocates the `Tile`s, the inter-tile registers and the output delay (all 0); settles | releases the Tiles | next values of the inter-tile ShiftRegister/Pipe registers and of the output delay from the Mesh inputs and the settled Tile outputs | commits them, ticks every Tile, settles the Tiles for the new cycle |
| `Transposer` | DIM x DIM registers, counter, dir (0) | nothing owned beyond its registers | next registers from in_row/in_valid/reset | commit; out_col for the new cycle |
| `TagQueue`, `RowsQueue` | entries and pointers (0) | nothing owned | next entries/pointers from enq/deq/reset | commit |
| `MeshWithDelays` | validates; allocates a `Mesh`, a `Transposer`, a `TagQueue`, a `RowsQueue`; its registers and skew/de-skew histories (0); settles | releases the four children | next state of its registers, evaluates every child with this cycle's inputs (the Mesh with the skewed feed of the current registers) | commits its registers, ticks every child, shifts the skew/de-skew stages, settles feed and outputs |
| `SystolicArray` | builds the `MeshWithDelays` (or a bare `Mesh`) and the provenance twin; opens the VCD file if asked | closes the VCD file, releases the components | (inside `tick()`) | evaluates every component, records the cycle (accounting, provenance), commits every component |

**Two-phase update.** `set_inputs()` gives a block this cycle's inputs; `eval()` computes every
next register value from the *current* registers and those inputs without changing any register;
`tick()` is the rising edge: every register of the hierarchy takes its next value, then the
combinational values that depend only on registers (Tile outputs, the skewed Mesh feed, every
output port) settle for the new cycle. `tick()` calls `eval()` itself if it has not run since the
inputs were set, and a parent's `eval()`/`tick()` calls its children's. The split is needed for
exactness in two places: inside a Tile the PEs are combinational in series, so every PE of a
column must be evaluated from pre-edge registers before any commits; and MeshWithDelays feeds the
Mesh from its own registers while reading the Mesh's outputs, so both must compute their next
state from pre-edge values (a single-phase "update in place" would let one see the other's
post-edge state). `classes_test` checks that `eval()` changes no output and no register, that a
repeated `eval()` changes nothing, and that `tick()` alone equals `eval()` + `tick()`.

**Interfaces are non-blocking ready/valid.** The outputs of `Mesh` and `MeshWithDelays` depend on
registers only (no combinational path from an input to an output: the RTL bench checks the same
of the RTL every cycle), so `out()` is valid before the inputs of the cycle are chosen. A driver
reads `out()` (the ready bits, the response), sets its valid bits and data with `set_inputs()`,
and calls `tick()`; a handshake is a cycle in which valid and ready are both 1. Nothing blocks,
nothing calls back, there are no threads and no global or static mutable state (two instances are
independent: `classes_test`).

```cpp
#include "systolique/systolic_array.h"
using namespace systolique;

SystolicArray array(*find_config("default"));  // ArrayOptions{}: MeshWithDelays interface, provenance on
array.reset();                                  // power-on, 4 cycles with reset high; cycle() == 0
MwdIn in;  in.resize(array.config());
// per cycle: read array.out() (ready, resp), set in.req/a/b/d and their valid bits, then
array.set_inputs(in);
array.tick();                                   // or: MwdOut seen = array.step(in);
PeView v = array.pe(3, 5);                      // registers, control, state, request, provenance
Accounting acc = array.accounting();            // per cycle / PE / request counters
std::string err = SystolicArray::check(acc);    // conservation; "" if it holds
```

- `ArrayOptions{Interface::Mesh}`: the bare Mesh interface (`set_mesh_inputs`, `mesh_out()`,
  `step_mesh`), as the MeshTop RTL top; no requests and no provenance there (below).
- `RequestNote{computes, preloads, label, op, uop, load_op, load_uop}` given with the cycle in
  which a request is offered overrides the data rule of section 3 for it and names the
  operation and micro-op its PE-cycles and its D belong to ([micro_ops.md](micro_ops.md)).
- State: `regs(r, c)`, `pe(r, c)` (`PeView`), `requests()` (`RequestInfo`), `d_rows()`, `rows()`,
  `mesh()`, `mesh_with_delays()`.

## 2. Cycles

- **Cycle 0** is the first cycle with reset low after the last cycle with reset high (`reset()`
  leaves `cycle() == 0`); without a reset, the first cycle after construction. All registers
  start at 0, as Verilator's `--x-initial 0` makes them. The stored traces have 4 reset cycles,
  so their row 4 is cycle 0. A reset cycle restarts numbering and accounting (rows must not be in
  flight; the Mesh has no reset and keeps its values, whose sources become unknown).
- **In cycle t**: the inputs applied in t, the outputs seen in t, the register values during t.
  A PE that is valid in t updates its registers at the edge that ends t (`PE.scala:141-146`), so a
  value latched "at cycle t" is in the register from t+1 on.
- Per request (`RequestInfo`, -1 if it has not happened):

| field | the cycle in which |
|---|---|
| `accept` | `req.valid && req.ready` (the request register takes it at the end of the cycle, `MeshWithDelays.scala:114-117`) |
| `first_in`, `last_in` | its first / last row is at the Mesh input of lane group 0 (no skew; the un-skewed feed, `MeshWithDelays.scala:110, 149, 188`). PE (0, 0) works on that row tile_latency+1 cycles later |
| `first_out`, `last_out` | its own first / last row leaves as a response (`resp.valid`); WS: the results its rows computed, OS: what its rows pushed out |
| `result_first`, `result_last` | the first / last response row carrying its tag (TagQueue head with a matching output id, `MeshWithDelays.scala:219, 228-235`): its matmul's result, computed by the next request's rows (WS) or pushed out by the one after (OS) |

Rows are attributed without decoding matmul ids: the valid ShiftRegisters and Pipes never drop
or reorder a row (`Mesh.scala:42-46, 91-97`), so the m-th valid cycle of any PE works on the m-th
row fed into the Mesh, and the k-th response row is the k-th row fed. The tag FIFO the component
keeps is checked against the TagQueue's length every cycle.

## 3. One PE = one MAC per cycle; the PE states

A PE has one MacUnit (`PE.scala:64-65`) and uses it at most once per cycle. Every PE-cycle is
therefore one MAC slot, the peak is rows() x cols() = DIM² MACs per cycle however the PEs are
grouped into tiles, and **useful MACs = PE-cycles in state MAC**. Every PE-cycle gets exactly one
state of a closed set (`PeState`), first matching rule wins:

| state | rule, for PE (r, c) in cycle t working on a row of request q | Chisel |
|---|---|---|
| `idle` | in_valid low: the registers hold, the MacUnit result is unused | `PE.scala:141-146` |
| `drain` | q is a flush (`flush > 0`) and the dataflow is OS: out_c shifts the finished results down; the accumulator adds stale buffers | `PE.scala:103-104, 110-111`, `MeshWithDelays.scala:110` |
| `bubble` | q is a flush in WS: valid rows that carry nothing (a/b/d are stale buffers) | `MeshWithDelays.scala:110` |
| `mac` | q computes (carries an A operand): WS `out_b = b + a*w` with the active register as w, OS `c += a*b` into the active register | `PE.scala:119-130`, `:107-108, 114-115` |
| `load` | q does not compute but preloads (carries a D operand): only the shadow register's `:= in_d` matters, WS weights / OS D | `PE.scala:124, 130`, `:109, 116` |
| `bubble` | anything else: valid, neither A nor D | |

"Computes" / "preloads" come from a `RequestNote` if the caller gave one, else from the data the
array received for the request's rows: computes iff any A value fed into the Mesh for its rows
is non-zero (after the transposer, `MeshWithDelays.scala:170`), preloads iff any such D value is
(`:172`). They are final once the request's last row has entered (`RequestInfo::decided`); a
`PeView` of an undecided request says `state_final = false`. The ExecuteController feeds zeros
when a request has no operand, so the array alone cannot tell a preload-only request from one
multiplying zeros.

What a PE does in a MAC cycle besides the MAC is a **flag, not a state** and never a second MAC:

- `load_concurrent`: a MAC PE-cycle whose request also preloads: the shadow register takes a
  weight (WS) or D (OS) in the same cycle (the overlap of preload j with compute j-1).
- `drain_concurrent`: an OS MAC PE-cycle in which out_c carries a finished result down: the
  register being replaced (`PE.scala:104, 111`) holds a value that was accumulated at this PE, or
  that sits below the PE row of the D element it started from (a result from above passing
  through). It needs provenance (0 without).

The active register is the one propagate selects (`PE.scala:103, 119`: propagate = 1 selects c2
as the WS multiplicand / OS accumulator and writes c1 from in_d); when the PE is idle, `last_s`
(the propagate of its last valid cycle, `PE.scala:89`) selects it.

## 4. Counters and windows

`Accounting` (from `accounting()`):

- `per_cycle[t]`: PE-cycles in each state in cycle t (they sum to DIM²) and the two flags;
- `per_pe[p]`: cycles in each state over the run for PE p (they sum to the run's cycles);
- `per_request[q]`: the PE-cycles its rows occupied, all in `requests()[q].state()`, and the
  flags; MACs of q = its MAC PE-cycles = rows_in x DIM² when it computes;
- `total` over the run window, `busy` over the busy window;
- **run window**: cycles [0, cycles()); **busy window**: [busy_begin, busy_end] = from the first
  request accepted (Mesh interface: the first row in) to the last cycle with an occupied PE;
- **occupancy** = PE-cycles with in_valid / (DIM² x cycles of the window), **utilisation** = MAC
  PE-cycles / (DIM² x cycles of the window), for each window.

- `per_uop`, `per_op`: the occupied PE-cycles by the micro-op / operation of their request
  (`RequestInfo::uop`, `op`, from its `RequestNote`; key -1: none); `loads_by_uop`: Load PE-cycles
  plus concurrent loads by the preload micro-op whose D they take (`load_uop`). The ExecuteController
  fills them ([micro_ops.md](micro_ops.md) section 5).

`SystolicArray::check(acc)` asserts conservation: every cycle's states sum to DIM² with at most
DIM² MACs; every PE's states sum to the cycles; the flags are counted only on MAC PE-cycles;
per-cycle, per-PE, per-request, per-micro-op and per-operation sums equal the totals state by
state, and the loads by micro-op sum to Load + load_concurrent; the total is DIM² x
cycles; no occupied PE-cycle lies outside the busy window. `conservation_test` recounts every
PE-cycle independently from the `PeView`s.

## 5. Provenance of every register value

`RegSource` (one per register, `PeView::src[0]` = c1, `src[1]` = c2):

| field | meaning |
|---|---|
| `request` | the request that consumed the d row the value came in with (the first row to enter the Mesh after the d handshake takes the d buffer, `MeshWithDelays.scala:123-126`); with its `tag` in `requests()` |
| `d_row`, `row`, `lane` | the d handshake (numbered from reset), its row within that request's d operand (fire_counter), the lane |
| `transposed` | WS `bd_transpose`: the row went through the transposer (`MeshWithDelays.scala:154`) |
| `w_row`, `w_col` | the operand element (W in WS, D in OS): rows are sent bottom row first (`ExecuteController.scala:253`), so d row r holds matrix row DIM-1-r, element (DIM-1-r, lane); through the transposer (`MeshWithDelays.scala:160, 172-173`) element (lane, DIM-1-r) |
| `value` | the d value as received |
| `loaded`, `loaded_by` | the cycle at whose end this PE latched it, and the request of that row |
| `active` | the first cycle it was the PE's active register while valid: the propagate flip that made it the multiplicand (WS) / accumulator (OS) |
| `last_used`, `uses`, `last_use_request` | the last cycle and the number of cycles the MacUnit used it (whatever the state), and the request of that row |
| `op`, `uop` | the operation and micro-op of `request`'s D (`RequestInfo::load_op`, `load_uop`): in WS, the preload micro-op whose weight the PE holds ([micro_ops.md](micro_ops.md)) |

In WS the op of the active register's value is the matmul whose weights the PE multiplies with
(the stationary weight's op); in OS it is the matmul whose output tile the PE accumulates (its D
preload, `request`) while `last_use_request` is the request whose MACs go into it; a result on its
way out keeps the source of the D element it started from.

**How it is tracked.** A second `MeshWithDelays` (the "token twin", `SystolicArray::Twin`,
configuration widened to carry 31-bit values: inputType 16, output and accumulator 32 bits,
BOTH) gets the same requests, valid bits and handshakes as the array, zero on a and b at its
Mesh, shift 0, the array's fixed dataflow bit if it has one, and a unique token on every d lane:
1 + d-handshake x cols + lane. With a = b = 0 its MacUnits add nothing and its OS output path is
the identity, so a token goes wherever the array moves the value it stands for: d buffer,
transposer, skew registers, inter-tile pipes, tile columns and PE registers, through every
request, flush and bubble. After each edge the shadow register of every PE that was valid takes
the twin's token. The component checks every cycle that the twin makes the same control
decisions (valid, id and last at the Mesh input) and that the token of an active register never
changes. The twin's Mesh input is adjusted through `MeshWithDelays::adjust_mesh_feed`, the
identity in the hardware class.

The bare Mesh interface has no requests (an "op" there is a run of rows on lane 0 with the same
id, ended by `last`), no provenance, and counts every valid PE-cycle as MAC.

## 6. Example

4 back-to-back WS matmuls on the 16x16 array: [perf_reports/README.md](../perf_reports/README.md)
(the figures are asserted by ctest `systolique_accounting_example`).

## 7. The viewer

```sh
cmake --build build --target systolique_dump
python3 tools/array_view.py --build build --config dim4 --scenario ws_stream --tiles 3 \
    --out docs/examples/array_ws_dim4.html
python3 tools/array_view.py --build build --config default --scenario ws_stream --tiles 4 \
    --out docs/examples/array_ws_dim16.html
python3 tools/array_view.py --build build --requests tools/ws_requests_example.json --out page.html
```

Committed examples (open them in a browser, they load nothing):
[array_ws_dim4.html](examples/array_ws_dim4.html) (DIM 4, 3 WS matmuls, 38 KB) and
[array_ws_dim16.html](examples/array_ws_dim16.html) (DIM 16, 4 WS matmuls, 380 KB);
`systolique_view` fails when they are no longer what these commands generate.

Scenarios: `ws_single`, `ws_stream`, `os_single`, `os_stream` (`--tiles K`, default 4, `--seed`,
`--bubble p` for random input bubbles), or `--requests file.json`: a request list
(`dataflow`, `propagate`, `rows`, `a`/`b`/`d` as `"random"`, `"zero"` or rows of numbers, `tag`,
`a_transpose`, `bd_transpose`, `shift`, `delay`, `flush`, `label`, and `computes`/`preloads` as
a `RequestNote`; format in `tools/array_dump.cpp`). `--json` takes a dump written earlier.

The page shows, per cycle (scrubber, play/step, keys: arrows step, Shift+arrows 10, Home/End,
Space play, T theme, Esc clear selection; `#c=<cycle>&pe=<r>,<c>&theme=light|dark` in the URL):

- the DIM x DIM PEs, each coloured by the op whose weight (WS; OS: D/result) is in its active
  register, an inner square in the colour of the shadow register's op, hatching for power-on
  values, a badge for the state (M, L, D, B; idle PEs faded) and corner triangles for the
  concurrent-load and concurrent-drain flags, which are marks over the state, not states;
- counters of the cycle (PEs per state, flags), useful MACs so far, occupancy and utilisation
  over cycles 0..t, and the whole-run and busy-window figures;
- a timeline of PEs per state over the run with the concurrent-load PE-cycles as a line and the
  current cycle marked (click to jump);
- the ops: colour, request index, label, dataflow, tag, accept / first-in / last-in / first-out /
  last-out cycles, result rows, PE-cycles per state, concurrent loads, MACs;
- on hover or click, a PE's state, its row's op, both registers (value, role, op, W/D element,
  d row and lane, loaded at, active since, last used) and its cycles per state so far and over
  the run. The rule "1 PE = 1 MAC per cycle" is stated in the legend.

## 8. Waveforms (VCD)

`ArrayOptions::vcd_path = "run.vcd"` (off by default) writes a VCD file (`include/systolique/
vcd.h`, standard library only): per cycle the clock (high at the start of the cycle, time =
10 x clock edges since construction, reset cycles included), reset, the cycle number, every port
of the chosen interface (one variable per lane) and, per PE (`pe_<r>_<c>`), in_valid, the state
(`PeState` code: 0 idle, 1 mac, 2 load, 3 drain, 4 bubble; it may still change for a request
whose operands are not all in, `PeView::state_final`), the active register, the request of its
row (-1 = none, as all ones), the two concurrent flags, its occupied cycles since reset, c1 and
c2. The writer only reads state; `systolique_vcd` replays 46 stored traces with and without it
and requires identical outputs and accounting.

## 9. Limitations

- Provenance needs the MeshWithDelays interface; the bare Mesh interface has neither requests nor
  provenance and counts every valid PE-cycle as MAC.
- "Computes"/"preloads" come from the data unless noted: a request whose A is all zero counts as
  not computing.
- `w_row`/`w_col` follow the ExecuteController's layout (bottom row first, full DIM-row
  operands); with fewer rows a value can stop above "its" row, and OS `drain_concurrent` uses
  that row to tell a result passing down from a D on its way to its PE.
- A d row is attributed to the first row that enters after its handshake, a flush row included
  (a flush feeds the stale d buffer, and its values then carry the flush's index); a d row
  overwritten before any row enters has no consumer. A and B values are not tracked, only what
  enters through d (WS weights, OS D and the results that follow the same path).
- With A and W (or A and B) both through the transposer in one stream the operands collide in
  Gemmini's single transposer; the provenance stays exact (it follows the values), the matmul
  meaning does not, and the tests use one transposed operand at a time.
- Tokens are 31 bits: provenance stops (an exception) after about 2^31 / DIM d values (134 M
  handshakes at DIM 16); `ArrayOptions{provenance = false}` runs without the twin at half the cost.
- A reset with rows in flight is refused (the accounting would lose track of them).
