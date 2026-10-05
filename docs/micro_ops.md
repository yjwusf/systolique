# Micro-ops: splitting operations the way Gemmini's controllers do

An operation (a Gemmini command stream, or a whole matmul) is split into the micro-ops Gemmini
executes, cycle by cycle, by C++ ports of Gemmini's own controllers: the command path
(raw command queue, LoopMatmul, ReservationStation) and the ExecuteController with its
scratchpad and accumulator banks, around the systolic array of
[systolic_array.md](systolic_array.md). Every micro-op gets an id, a parent micro-op and an
operation, and its cycles; every array PE-cycle and every bank access is attributed to a
micro-op and an operation. The classes are checked against Gemmini's RTL port by port, cycle by
cycle ([validation.md](validation.md)); only the data movement to and from DRAM is a model.

Chisel citations are `file:line` in `src/main/scala/gemmini/` of Gemmini v0.7.2 (`709bc56`);
gemmini.h is that of gemmini-rocc-tests `1a1a1c6`, the commit v0.7.2 pins.

## 1. Use

```cpp
#include "systolique/engine.h"
using namespace systolique;

Engine e;                                     // Gemmini's defaultConfig, DmaParams{}
Matmul m;                                     // 64 x 64 x 64, int8, WS by default
int op = e.submit(m);                         // gemmini.h's command stream; A, B (D) in DRAM
int64_t cycles = e.run();                     // tick() until done
for (const MicroOp &u : e.micro_ops().all())  // every micro-op with its cycles
  ...;
Accounting a = e.accounting();                // a.per_op[op], a.per_uop[id]: PE-cycles per state
bool ok = e.result(op) == e.reference(op);    // C as the mvouts stored it vs plain C++
std::string err = e.check();                  // conservation and attribution, "" if they hold
```

- `submit(std::vector<Command>, label)`: any command stream (`include/systolique/isa.h` has
  builders for gemmini.h's macros; `cmd_fence()` is the core's fence); one operation.
- `submit(Matmul)`: `Matmul{M, K, N, dataflow, bias, full_c, act, seed, range}`; the commands are
  `Engine::matmul_commands()`: tiled_matmul_auto's tiling (gemmini.h:1231-1330) and
  tiled_matmul_outer's configs and loops (:692-848), WS through `gemmini_loop_ws` (:349-357,
  unrolled by LoopMatmul), OS through sp_tiled_matmul_os's explicit mvin / preload / compute /
  mvout commands (:380-500), then a fence.
- `tick()` is one clock edge; `host_inputs()` + `step(in)` split it for benches that drive a
  second copy (the RTL) with the same inputs; `set_hook()` sees every cycle before its edge.
- `EngineOptions`: `array` (the SystolicArray's options: provenance, its VCD), `dma`
  (`DmaParams`, section 7), `micro_ops` (record the table; on by default), `vcd_path` (a VCD of
  the Controller's ports and the micro-ops in flight; off by default).
- The viewer: `tools/array_view.py --build build --matmul 32x16x32:WS --matmul 16x32x16:WS
  --out page.html` (committed example:
  [examples/engine_ws_two_matmuls.html](examples/engine_ws_two_matmuls.html)): the PE grid
  coloured by (operation, micro-op), the weight-stationary "which preload is this weight from"
  view, a Gantt of the command micro-ops grouped by operation, the micro-ops in flight, the
  operations and the array passes.

## 2. Classes

Each is a class with a constructor that builds its state and allocates its children, an
explicit destructor, `set_inputs()`, `eval()` (every next register value, no register
changes) and `tick()` (commit; runs `eval()` if it has not run since the inputs were set).
Outputs that the RTL computes from registers only are available before the inputs are chosen
(`cmd_ready()`, `resp()`, `out()` of the issue ports, `Controller::out_regs()`). Where the RTL
has a combinational path between blocks in the same cycle, the class offers it as a `const`
function of its registers and the signals it depends on, and the owner evaluates in the RTL's
order (`ExecuteUnit::eval`): the scratchpad's responses (registers) → the ExecuteController's
write-back and response pops (`writeback()`) → the banks' read readiness (`read_ready()`,
`ready_view()`, which depend on those writes and pops) → the ExecuteController's operand reads
→ the banks' inputs. `systolique_micro_ops` checks that `eval()` changes nothing and that
`tick()` alone equals `eval()` + `tick()`.

| class | header | Chisel | owns |
|---|---|---|---|
| `ExecuteController` | `execute_controller.h` | `ExecuteController.scala`, `TransposePreloadUnroller.scala` | the unroller's and the 3-headed command queues, the FSM, fire counters, the control-signal queue, the tag table, a `SystolicArray` |
| `ScratchpadBank`, `Scratchpad` | `scratchpad.h` | `Scratchpad.scala:97-169, 446-557` | per bank a single-ported memory, its 1-entry response queue, the spad_read_delay Pipeline |
| `AccumulatorBank`, `Accumulator` | `scratchpad.h` | `AccumulatorMem.scala`, `Scratchpad.scala:618-823` | per bank the two-ported memory, the acc_latency write pipeline, the response queue; the shared adder (AccPipeShared) |
| `ExecuteUnit` | `controller.h` | `rtl/ex/src/ExecuteTop.scala` | ExecuteController + Scratchpad + Accumulator |
| `ReservationStation` | `reservation_station.h` | `ReservationStation.scala` | the ld / ex / st entries with operand ranges and dependency bits |
| `LoopMatmul` | `loop_matmul.h` | `LoopMatmul.scala` | its input queue, two loop states, the LdA / LdB / LdD / Execute / StC sub-FSMs |
| `CommandPath` | `command_path.h` | `Controller.scala:124-170, 233-246, 358-401`; `rtl/ex/src/CmdTop.scala` | raw_cmd_q, LoopConv's queue, LoopMatmul, the unrolled queue, the ReservationStation |
| `Controller` | `controller.h` | `Controller.scala:243-246, 312-327`; `rtl/ex/src/CtrlTop.scala` | CommandPath + ExecuteUnit + the completion Arbiter |
| `Host` | `host.h` | (model) | the core's command issue; the load / store side and DMA (section 7) |
| `Engine` | `engine.h` | (front door) | Controller + Host + the micro-op table + DRAM |

Building blocks (`fe_util.h`): chisel3 `Queue`, Gemmini's `Pipeline` and `MultiHeadedQueue`,
`wrappingAdd`, `floorAdd`; `isa.h`: commands, `LocalAddr` (with its address geometry as a
value, no global state), the builders. `GemminiCmd::op / uop / parent_uop` are bookkeeping
that travels with a command and never steers a decision.

## 3. The micro-op set

`micro_ops.h`; ids are indices into `MicroOpTable`; every micro-op has `op` (its operation)
and `parent`.

| kind | one per | created by | parent |
|---|---|---|---|
| `Config`, `Preload`, `ComputePreloaded`, `ComputeAccumulated`, `Mvin`, `Mvout`, `LoopCmd`, `Fence`, `OtherCmd` | Gemmini command | `Engine::submit` (the core's commands); `CommandPath` for every command LoopMatmul emits | -; the `loop_ws` command for LoopMatmul's |
| `Flush` | flush pass the ExecuteController inserts (`:620-630`) | `ExecuteController` | the last command it popped |
| `Request` | array pass: preload alone, compute + preload, compute alone (`:532-693`) | `ExecuteController` | its compute (else its preload) |
| `OperandRead` | A / B / D row read from a scratchpad bank (read.req.fire) | `ExecuteController` | the command whose address it reads |
| `ResultRow` | row leaving the array with a tag, and its write-back | `ExecuteController` | the preload that owns the tag (its C) |
| `DmaRow` | row an mvin writes into a bank / an mvout reads | `Host` | the mvin / mvout |

Cycles of a command micro-op (-1: has not happened):

| field | the cycle in which |
|---|---|
| `sent` | the core's command entered raw_cmd_q (io.cmd.fire); LoopMatmul's: it entered the unrolled queue |
| `alloc` | it entered the ReservationStation (alloc.fire) |
| `issued` | the ReservationStation issued it to its controller (issue.fire) |
| `started` | the ExecuteController decided to perform the pass that executes it (or, for a config, executed it); DMA: its data phase began |
| `first_read`, `last_read`, `reads` | its operand rows were read from a bank |
| `accept` | its pass's request was accepted by MeshWithDelays (req.fire) |
| `first_in`, `last_in` | its request's first / last row at the Mesh input (systolic_array.md section 2) |
| `first_out`, `last_out` | its request's first / last row left the array (resp.valid) |
| `popped` | it left the ExecuteController's command queue |
| `result_first`, `result_last`, `result_rows` | rows carrying its tag left the array (a preload: its C) |
| `wb_first`, `wb_last`, `wb_rows` | those rows were written to the scratchpad / accumulator (write fire) |
| `wb_done` | the last of them is in the memory: the write's cycle for the scratchpad, two cycles later for the accumulator (the write pipeline, `AccumulatorMem.scala:110-125`) |
| `completed` | its completion reached the ReservationStation (`io.completed`, through the Arbiter) |

Per-row micro-ops have `cycle` (the event) and `done` (its effect in memory; an operand read:
the earliest cycle its data can be at the ExecuteController, `cycle + 1 + spad_read_delay`).
An `Operation` has `first_sent` / `last_sent` (the core's commands, fences excluded) and `end`
(the last cycle any of its micro-ops did something; a trailing fence: the cycle it found Gemmini
idle).

## 4. Splitting rules

Command path (`CommandPath`): the core's command enters raw_cmd_q (2 entries), LoopConv's input
queue (2; every command passes through while no conv loop is configured), LoopMatmul (its
input queue of 2; `LOOP_WS_CONFIG_*` and `LOOP_WS` configure one of two loops,
`LoopMatmul.scala:901-951`; a configured loop's five sub-FSMs emit mvin (A, `:28-122`), mvin2 (B,
`:139-237`), mvin3 (D, `:253-332`), preload / compute (Execute, `:352-495`, order k, j, i) and
mvout (StC, `:514-690`) through a fixed-priority Arbiter StC > Execute > LdD > LdA/LdB
(`:830-835`), each throttled by its count of ReservationStation entries in use, `:842-870`),
the unrolled-command queue (2), then the ReservationStation: one entry per command in its queue
(load, execute, store, `ReservationStation.scala:300-304`), with dependency bits on overlapping
scratchpad / accumulator ranges of the other queues and in-order issue within a queue
(`:309-339, 395-404`); load / store configs complete on issue (`:343`), everything else when its
controller reports it. Completions reach it through an Arbiter: execute, then load, then store
(`Controller.scala:312-327`).

ExecuteController: an issued command goes through the TransposePreloadUnroller's queue (one
cycle) into the 3-headed command queue (the next), so it can be acted on two cycles after its
issue (`TransposePreloadUnroller.scala:30-81`, `ExecuteController.scala:69`). In the waiting
state the head decides (`:533-631`):

| head | condition | micro-op |
|---|---|---|
| config | no matmul in progress, no completion pending | executed and completed in the same cycle (`:541-582`) |
| preload | a second command is in the queue, no RAW hazard on the tags in the array | pass: preload alone (`:585-597`) |
| compute, then preload | a third command is in the queue (RAW hazards are possible), no RAW hazard | pass: compute + the next preload (`:600-610`) |
| compute | otherwise | pass: compute alone (`:612-620`) |
| none / config | a matmul in progress and OS (or a config waiting) | flush (`:623-630`), repeated after each flush while one is in progress |

What a pass reads (`:118-127, 244-386`; place = the command in the queue whose address is used):

| pass | WS | OS |
|---|---|---|
| preload alone | D of the preload (its weights) | D of the preload; A of the compute after it (A goes through the transposer one request early) |
| compute + preload | A, B of the compute; D of the preload | A of the compute after the preload; B of the compute; D of the preload |
| compute alone | A, B of the compute | B of the compute (its A came in the previous pass) |

A garbage address reads nothing and feeds zeros (`:150-152`). One row of each operand per cycle
(`total_rows` = DIM, or in WS without D max(A rows, B rows, 4), `:285-302`), D bottom row first
(`:253`); two operands on the same bank go one behind the other by priority A > B > D, and none
runs ahead of another by two rows (`:316-357`); a bank does not take a read while it is
written, by the ExecuteController or by an mvin (single-ported, `Scratchpad.scala:166`). Each
row's control entry waits in a queue of spad_read_delay + 1 (`:178-181`) for the bank's response
(one cycle in the bank, spad_read_delay = 4 in the Pipeline); the first row's entry fires the
request (`:883`, req.ready needed), and MeshWithDelays feeds a row into the Mesh once A, B and D
have each sent one (`MeshWithDelays.scala:110`). A pass ends with its last row
(`about_to_fire_all_rows`, `:405-410`), which pops its commands (`:632-681`).

Results: the request's tag is the preload's (rob id, C address, rows, cols, `:183-207, 885`); it
comes back on the rows of the next request in WS and of the one after in OS
(`MeshWithDelays.scala:219`). Each row with a valid rob id is written to C (WS top row first, OS
bottom row first, rows c_stride apart, `:903-951`): to the scratchpad clipped to int8 with the
configured activation, to the accumulator as int32, accumulating if C's accumulate bit is set.
Completions (`:579, 643-681, 958-990`): a preload with C when its last result row leaves; a
preload without C and a compute when their pass ends (one cycle later, from the pending
registers); at most one per cycle, the mesh's first.

## 5. Attribution

The ExecuteController gives the array a `RequestNote` with every request
(systolic_array.md section 3): `computes` = the pass includes a compute whose A address is not
garbage, `preloads` = it includes a preload; `uop` / `op` = the compute's micro-op and operation
(else the preload's, else the flush's), `load_uop` / `load_op` = the preload's. So:

- every PE-cycle's state is attributed to the micro-op whose row the PE works on (`MAC`: a
  compute; `Load`: a preload alone; `Drain` / `Bubble` of a flush: the flush); the concurrent-load
  flag of a MAC PE-cycle belongs to the preload that shares the pass;
  `Accounting::per_uop`, `per_op` (occupied PE-cycles; with the idle ones they sum to the
  totals), `loads_by_uop` (Load + load_concurrent by preload micro-op);
- every register value's source (`RegSource`) names the preload micro-op and operation whose D
  brought it in (`RegSource::uop`, `op`): in WS, the preload whose weights a PE multiplies with;
- every scratchpad / accumulator access is a micro-op: `OperandRead`, `ResultRow` (the write),
  `DmaRow` (mvin writes, mvout reads), with bank and row.

`SystolicArray::check` requires the per-micro-op and per-operation counts to sum to the totals;
`Engine::check` also requires every MAC PE-cycle to belong to a compute micro-op of the
operation, and a matmul whose dimensions are multiples of DIM to have exactly M N K MAC
PE-cycles (1 PE = 1 MAC per cycle). With padding, the MAC PE-cycles are the rows each compute
pass feeds (`total_rows`) x DIM², zeros included.

## 6. What is exact and what is modelled

| part | status | shown by |
|---|---|---|
| ExecuteController with its banks (`ExecuteUnit`), command path (`CommandPath`), both wired as Gemmini's Controller (`Controller`), and the array inside | **cycle-exact at every port** of the RTL tops ExecuteTop, CmdTop, CtrlTop | stored traces (`systolique_fe_trace_<top>`), live lockstep (`rtl_fe_*`) |
| the micro-op records, the attribution | bookkeeping of those classes' state; never steers them | `systolique_micro_ops` (directed cycles derived from the Chisel; runs with and without recording identical) |
| the load / store controllers, the DMA, DRAM (`Host`) | **modelled, not validated against any RTL** (section 7) | — |
| the core | sends one command per cycle when raw_cmd_q has room; a fence waits until Gemmini and the DMA model are idle (Rocket's RoCC timing; not validated here) | — |
| operands from the accumulator, the accumulator's read path (normalizer, AccumulatorScale), mvin / acc scales other than 1.0, LoopConv, LayerNorm / Softmax, im2col, counters, pooling | not modelled; `Engine::submit` refuses accumulator operands and loop_conv_ws, the Host reports scales | — |

"Exact" means: given the same inputs at the Controller's ports (the core's commands, the
load / store controllers' acceptance and completions, the DMA's row writes), every output port
equals the RTL's in every cycle. An Engine run's timing is therefore exact for the behaviour the
Host gives those inputs; how long an mvin or mvout takes is the model's.

## 7. Data movement (modelled)

`DmaParams{latency = 40, bytes_per_cycle = 16, max_cmds = 2}` (`host.h`; status: unvalidated,
not fitted): one load and one store channel, each serving its commands in order and holding at
most `max_cmds`; DRAM row r of an mvin arrives `latency + ceil((r + 1) row_bytes /
bytes_per_cycle)` cycles after its data phase starts (behind the previous command's on the same
channel); each arrived row is split into its DIM-element blocks and written into its bank
through the DMA's row-write port (one scratchpad and one accumulator row per cycle, in the DMA's
lower-priority slot, `Scratchpad.scala:519-543, 718-823`: these bank conflicts with the
ExecuteController are the RTL's); the mvin completes the cycle after its last row was taken. An
mvout completes `latency + ceil(rows row_bytes / bytes_per_cycle)` after acceptance and reads its
rows functionally at completion (it does not use the banks' read ports; the ReservationStation
keeps it behind the commands that write its rows). The page of the viewer, the perf report and
`host.h` say so wherever a figure depends on it.

## 8. Findings

- Only one output of the three RTL tops is not equal in every cycle: the ExecuteController's
  completion id while `completed.valid` is low (`io.completed.bits := DontCare`,
  `ExecuteController.scala:172`). Every other port, including addresses, data and masks of
  disabled bank ports and the commands of idle issue ports, matches the RTL every cycle
  (rtl_fe compares them all).
- A compute pass can be accepted by MeshWithDelays before any of its A rows has been read: in
  WS LoopMatmul's computes have a garbage B, whose zero rows "fire" at once (`:841-883`); the
  rows enter the Mesh only when A arrives. When mvins write A's bank, the single-ported bank
  takes no read meanwhile: in the 64³ WS example of perf_reports/ the first compute's pass lasts
  189 cycles (started to popped) instead of 16, while 173 mvin rows are written into its A bank.
- An OS stream ends with as many flush passes as it takes for the last result's tag to leave the
  array: after each flush the FSM finds the matmul still in progress and flushes again (three
  in the directed test of `systolique_micro_ops`, `:627-630, 687-692`).
- `AccPipeShared` (`AccumulatorMem.scala:74-90`) selects the banks' adder operands with a Mux1H:
  if both accumulator banks accumulate in the same cycle (an ExecuteController write and a DMA
  write with the accumulate bit), the operands are OR-ed. The model does the same; no test is
  known to exercise it.

## 9. Limitations

- Gemmini's defaultConfig only (16x16, 4 scratchpad banks of 4096 rows, 2 accumulator banks of
  512 rows): the configuration the RTL tops are elaborated with (`FrontendConfig::validate`).
- The DMA, DRAM and the core are models (section 7); Gemmini's own LoadController,
  StoreController, StreamReader / StreamWriter, TLB and TileLink are not in this repository.
- The ExecuteController cannot take operands from the accumulator (its read responses have no
  consumer here, as in ExecuteTop).
- Per-row micro-ops grow with the run (a 64³ matmul: about 3,300 micro-ops); the viewer shows the
  command and request micro-ops and counts the rows.
