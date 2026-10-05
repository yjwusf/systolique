# Gemmini's systolic array: microarchitecture, model and cycle correspondence

Source: [ucb-bar/gemmini](https://github.com/ucb-bar/gemmini) **v0.7.2**
(`709bc56b6dd859fc2b1a9027a96a0b5be6ad7ed6`; Chisel 3.6.0, `build.sbt:7-10`). Citations below
are `file:line` in `src/main/scala/gemmini/` of that commit. The RTL the model is checked
against is elaborated from these files, unmodified (`rtl/`).

## 1. Where the array sits

Gemmini is a RoCC accelerator: `Controller.scala` instantiates the reservation station
(`Controller.scala:124`), the load, store and execute controllers (`:188-190`) and the
scratchpad/accumulator (`:36`). The **ExecuteController** reads operand rows from the
scratchpad or the accumulator, feeds them row by row into **MeshWithDelays**
(`ExecuteController.scala:186-187`), and writes the result rows back
(`ExecuteController.scala:903-951`). MeshWithDelays wraps the **Mesh** of **Tiles** of **PEs**.
This repository models MeshWithDelays and everything inside it; the controllers, memories, DMA
and the RoCC interface around it are not modelled here.

## 2. Configuration

`GemminiArrayConfig` (`GemminiConfigs.scala:17-82`); Gemmini's `defaultConfig`
(`Configs.scala:21-35`); `include/systolique/config.h`:

| parameter | default | where |
|---|---|---|
| `inputType` | `SInt(8)` | `Configs.scala:23` |
| `accType` | `SInt(32)` | `Configs.scala:24` |
| `spatialArrayOutputType` | `SInt(20)` | `Configs.scala:26` |
| `tileRows` x `tileColumns` | 1 x 1 (PEs per Tile, combinational) | `Configs.scala:29-30` |
| `meshRows` x `meshColumns` | 16 x 16 (Tiles, registered) | `Configs.scala:31-32` |
| DIM = `meshRows*tileRows` | 16 | `MeshWithDelays.scala:45-46` |
| `dataflow` | `BOTH` (OS or WS per request) | `Configs.scala:35`, `Dataflow.scala:4-6` |
| `tile_latency` | 0 (registers between tiles: `tile_latency+1`) | `GemminiConfigs.scala:78` |
| `mesh_output_delay` | 1 | `GemminiConfigs.scala:79` |
| tree reduction | only WS-only arrays with `tileRows > 1` | `GemminiConfigs.scala:81, 175` |
| `shifter_banks` | 1 | `GemminiConfigs.scala:53` |
| `max_simultaneous_matmuls` | `ceil(5 * max(1, (tile_latency+1)/min(tileRows,tileColumns)))` = 5 | `MeshWithDelays.scala:48-54` |

The five named configurations (`src/config.cpp`, `rtl/src/GemminiTops.scala` `ArrayCfgs`), all
elaborated and lockstep-tested: `default` (Gemmini's: 16x16, int8/int20/int32, BOTH), `dim4`
(4x4), `tiled` (DIM 8 as 4x4 tiles of 2x2 PEs, `tile_latency` 1, `output_delay` 2), `ws_tree`
(WS-only, 2x2 tiles, tree reduction), `os16` (OS-only, DIM 8, int16 inputs, int24 outputs).

## 3. Microarchitecture

### PE (`PE.scala:31-147`) -- class `PE`

Ports: `in_a` (inputType), `in_b`, `in_d` (outputType), `in_control` = {dataflow, propagate,
shift} (`PE.scala:7-12`), `in_id`, `in_last`, `in_valid`, and the same as outputs
(`PE.scala:35-56`). Everything but `out_b`/`out_c` passes straight through (`PE.scala:79-85`).
Registers: `c1`, `c2` (inputType in a WS-only array, accType otherwise, `PE.scala:58, 70-71`)
and `last_s = RegEnable(propagate, valid)` (`PE.scala:89`). The multiply-add is a separate
`MacUnit` (`PE.scala:14-24, 64-65`) whose result has the **output width** (20 bits): partial sums
wrap at 20 bits even though `c1/c2` are 32 bits wide.

- **Output stationary** (`PE.scala:102-117`): `propagate` selects which register accumulates
  `a*b` and which shifts down: with `propagate = 1`, `out_c = c1 >> shift_offset`,
  `c2 += a*b`, `c1 := in_d`; with 0 the roles swap. `out_b = in_b`. The rounding shift
  (`Arithmetic.scala:97-108`, round to nearest, ties to even) is applied only in the first valid
  cycle after `propagate` changed at that PE (`flip = last_s =/= propagate`, `PE.scala:89-91`), so
  every result is shifted once, at the PE that computed it, then saturated to the output width
  (`clippedToWidthOf`, `Arithmetic.scala:122-126`).
- **Weight stationary** (`PE.scala:118-131`): the register selected by `propagate` is the
  stationary weight (low `inputType` bits), `out_b = in_b + a*weight` (MacUnit), the other
  register loads `in_d`; `out_c` is the register being propagated.
- `when (!valid)` the registers hold (`PE.scala:141-146`); the `DontCare` on the MacUnit inputs
  there is resolved by the FIRRTL compiler to the valid-path values, so `out_b` still carries the
  product-sum.

### Tile (`Tile.scala:16-131`) -- class `Tile`

A purely combinational `tileRows x tileColumns` grid: `a` broadcast right along each row
(`:47-53`), `b`, `d` (through `out_c`), control, valid, id and last down each column (`:56-107`),
outputs from the bottom row (`:110-125`). With tree reduction each PE gets `b = 0` and the Tile
sums the column's products and `in_b` (`accumulateTree`, `:117-121`, `Util.scala:112-126`).

### Mesh (`Mesh.scala:17-129`) -- class `Mesh`

`meshRows x meshColumns` Tiles with `tile_latency+1` registers in front of every Tile:
`ShiftRegister` for `a` (from the left, `:50-56`), `in_valid`, `id`, `last` (from above,
`:91-115`); `Pipe(valid, x, tile_latency+1)` for `b`, `d` (enabled by lane 0's valid,
`valid.head`, `:59-74`) and the control fields (per lane, `:78-88`). `Pipe` is
`RegNext(valid)` + `RegEnable(bits, valid)` per stage, without reset (`:42-46`). The bottom
Tiles' outputs go through `output_delay` more registers (`:117-128`). There is no reset in the
Mesh.

### MeshWithDelays (`MeshWithDelays.scala:32-256`) -- class `MeshWithDelays`

Interface (`:58-68`): `a`, `b`, `d` are `Decoupled` rows (`Vec(DIM)` of inputType), `req` a
`Decoupled` `MeshWithDelaysReq` = {pe_control, a_transpose, bd_transpose, total_rows, tag,
flush} (`:9-17`), `resp` a `Valid` `MeshWithDelaysResp` = {data, total_rows, tag, last}
(`:19-25`), `tags_in_progress` the tags in the tag queue.

- **Request and row handshake** (`:93-149`): one request register (`req`, valid cleared by reset,
  `:251-253`); `a_buf/b_buf/d_buf` latch a row on its handshake; a row enters the array when all
  three have been written, or every cycle of a flush (`input_next_row_into_spatial_array`,
  `:110`). `fire_counter` counts rows up to `total_rows` (`:98, 128`); `last_fire` ends the
  request (a flush of 2 runs twice, `:118-121`). `a/b/d.ready = !written || input_next ||
  req.ready` (`:143-145`); `req.ready = (!req.valid || last_fire) && both queues have room`
  (`:248`). Every request flips `in_prop` if its `propagate` is 1 (`:116`) and advances
  `matmul_id` modulo `max_simultaneous_matmuls` (`:117`).
- **Transposer** (`:151-173`, `Transposer.scala:94-151`, `AlwaysOutTransposer`; class
  `Transposer`): a DIM x DIM register array that shifts rows in from the right while it shifts
  the previous matrix out on the left (or the same vertically), switching direction every DIM
  rows. In OS, `a` goes through it unless `a_transpose`; `b` if `bd_transpose`; in WS `a` if
  `a_transpose`, `d` (reversed) if `bd_transpose`. Its output is one request late: the matrix fed
  during request j comes out transposed during request j+1.
- **Skew** (`shifted()`, `:70-91`, used at `:175-195`): lane group i of every Mesh input (a, b,
  d, valid, id, last, dataflow, propagate, shift) is delayed by `i*(tile_latency+1)` cycles with
  its own `ShiftRegister` (a triangle of registers). `shift` additionally goes through
  `result_shift = RegNext(req.bits.shift)` (`:183`), one cycle later than the other control
  fields: the first row of a request carries the previous request's shift when it enters the
  cycle after the request's handshake (the source's own TODO at `:183`).
- **De-skew** (`:199-204, 232`): `resp.data` is `out_c` when the bottom-left PE's dataflow is OS,
  else `out_b`; lane group j is delayed by `(meshColumns-1-j)*(tile_latency+1)`, `resp.valid`,
  `resp.last` and the output matmul id take lane 0's delay.
- **Tags** (`:206-249`; classes `TagQueue`, `RowsQueue`): a `TagQueue` (`TagQueue.scala:11-52`)
  holds each non-flush request's tag with the id of the rows that will carry its result:
  `matmul_id + 3` in OS (D is preloaded in request j, computed in j+1, pushed out in j+2) and
  `+ 2` in WS (`:219`); `resp.tag` is the head tag when its id matches the output rows' id, else
  garbage, and the head is dequeued with the last row. A second queue (chisel3 `Queue`) returns
  `total_rows` the same way (`:237-246`).

### How the ExecuteController drives it

Commands `preload` and `compute.preloaded`/`compute.accumulated` (`ExecuteController.scala:84-90`)
become requests: a compute overlapped with the next preload (`perform_mul_pre`, `:599-610`), a
preload alone, a compute alone; a flush (`flush = 1`, `in_prop_flush`) at the end of an OS
sequence or on a dataflow change (`:620-627, 682-693, 207`). D is read bottom row first
(`:253`) and OS results are written bottom row first (`:907-908`). In OS the matmul C = A*B + D
takes three requests' worth of time: preload D_k (request k), compute A_k B_k (request k+1),
push the result out (request k+2, the next compute or the flush). The bench's stimuli
(`bench/stimulus.cpp`, `bench/reference.cpp`) issue the same request sequences.

## 4. The model

`include/systolique/`, `src/`: one class per hardware block, each with a constructor that builds
its state from the configuration and allocates its children, an explicit destructor, and a
two-phase clock (`eval()` then `tick()`, [systolic_array.md](systolic_array.md#1-classes-and-the-clock)):

| class | Chisel | owns |
|---|---|---|
| `PE` | `PE.scala` | its registers c1, c2, last_s |
| `Tile` | `Tile.scala` | tileRows x tileColumns `PE`s |
| `Mesh` | `Mesh.scala` | meshRows x meshColumns `Tile`s, the inter-tile ShiftRegister/Pipe registers, the output delay |
| `Transposer` | `Transposer.scala` `AlwaysOutTransposer` | DIM x DIM registers, counter, dir |
| `TagQueue`, `RowsQueue` | `TagQueue.scala`, chisel3 `Queue` (`MeshWithDelays.scala:222, 237`) | entries and pointers |
| `MeshWithDelays` | `MeshWithDelays.scala` | a `Mesh`, a `Transposer`, a `TagQueue`, a `RowsQueue`, its own registers and the skew/de-skew histories |
| `SystolicArray` | (none: the observer) | a `MeshWithDelays` (or a bare `Mesh`), the provenance twin, the accounting |

`arith.h` holds the SInt operations bit for bit, `config.h` the configurations and the supported
range, `types.h` the port value types. All registers start at 0, as Verilator's
`--x-initial 0` makes them. The RTL tops (`rtl/src/GemminiTops.scala`) have the same ports as
`MeshIn/MeshOut` and `MwdIn/MwdOut`, with the lanes of a vector packed into one port (lane k at
bits `[k*w +: w]`) and `io_` prefixes; `bench/ports.h` converts. The tag is a bench tag {valid,
id} (`BenchTag`); the ExecuteController's tag carries a ROB id and an address, which the array
only stores and returns.

## 5. Cycle correspondence

Every boundary signal of both tops is equal to the RTL's in every cycle (section 7). Inside,
the model keeps every register of the RTL one for one, except where noted as
**correspondent**: there the model holds the same information in another form, and the stated
equation holds every cycle t (t counted in clock cycles from power-on; "stage k" = the k-th
register of a chain, k = 0 next to its source). The lockstep bench (`rtl/rtl_bench.cpp`) reads
the RTL registers through VPI before every clock edge and checks every row of this table that
says "VPI" (`rtl/correspondence.cpp`).

| model state | RTL register / signal | Chisel | relation | checked |
|---|---|---|---|---|
| `Mesh` / `MeshWithDelays` ports (`MeshIn/Out`, `MwdIn/Out`) | `MeshTop` / `MeshWithDelaysTop` `io_*` (packed lanes) | `GemminiTops.scala` | equal every cycle | ports |
| `PE::regs()` c1, c2, last_s of PE (R, C) | `mesh_r_c.tile_i_j.{c1,c2,last_s}`, r = R / tileRows, i = R % tileRows (same for C) | `PE.scala:70-71, 89` | equal (`last_s` is removed from WS-only RTL: never read) | VPI |
| `Mesh::TileRegs::a` stage k | `ShiftRegister(in_a, L+1)` stage k | `Mesh.scala:53` | equal | ports |
| `TileRegs::b, d, df, prop, shift` stage k | `Pipe` bit registers (`pipe_b_*`, `*_pipe_b`, `*_pipe_pipe_b`) | `Mesh.scala:62, 71, 82-84` | equal | ports |
| `TileRegs::valid` stage k, tile (r, c), lane j | `ShiftRegister(in_valid, L+1)` stage k | `Mesh.scala:94` | equal | VPI |
| `TileRegs::valid` stage k, tile (r, c), lane j | the `RegNext(valid)` flop of stage k of **every** `Pipe` of tile (r, c) whose valid is lane j (`pipe_v_*`, `*_pipe_v`) | `Mesh.scala:42-46, 62, 71, 82-84` | **correspondent**: RTL `pipe_v`(r, c, j, k) at t == model `valid`(r, c, j, k) at t. Besides the in_valid chain the RTL has one such flop per Pipe: 3 per lane (5 for lane 0, which also enables `b` and `d`); the model keeps one chain. They exist for `tile_latency` >= 1 only (the last stage's valid is never read) | VPI |
| `TileRegs::id, last` | `ShiftRegister(in_id/in_last, L+1)` | `Mesh.scala:103, 112` | equal | ports |
| `Mesh` output delay (`outq_`) | `ShiftRegister(out_*, output_delay)` | `Mesh.scala:122-127` | equal | ports |
| `MeshWithDelays::Regs` req_valid, req | `req_valid`, `req_bits_*` | `MeshWithDelays.scala:93, 114-121, 251-253` | equal (`req_bits_pe_control_shift` is removed from WS-only RTL) | VPI |
| `Regs::matmul_id, fire_counter` | `matmul_id`, `fire_counter` | `:95, 98, 117, 128` | equal | VPI |
| `Regs::a_buf, b_buf, d_buf`, `*_written` | `a_buf_i_j`, `b_buf_i_j`, `d_buf_i_j`, `a/b/d_written` | `:100-106, 123-141` | equal | VPI |
| `Regs::in_prop, result_shift` | `in_prop`, `result_shift` | `:108, 116, 183` | equal (`result_shift` removed from WS-only RTL) | VPI |
| `Transposer` regs, counter, dir | `transposer.pes_y_x.reg_`, `transposer.counter`, `transposer.dir` | `Transposer.scala:110, 117-118, 144-150` | equal | VPI |
| `TagQueue` entries, raddr, waddr, len | `tagq.regs_k_{tag_valid,tag_id,id}`, `tagq.raddr/waddr/len` | `TagQueue.scala:18-49` | equal | VPI |
| `RowsQueue` pointers, maybe_full | `total_rows_q.enq_ptr_value`, `deq_ptr_value`, `maybe_full` | chisel3 `Queue` | equal (the `ram` is not read through VPI) | VPI |
| `MeshWithDelays::feed_history(k+1)`, lane l of signal s | stage k of the `shifted()` `ShiftRegister` of Mesh input s, lane l (`RegShifted_*`, `*_shift_r_*`, ...) | `MeshWithDelays.scala:70-91, 175-195` | **correspondent**: RTL stage k of lane l at t == model feed history entry k+1, lane l, at t (the un-skewed feed of cycle t-1-k); the RTL has a triangle of sum(i*(L+1)) registers per signal, the model one history of (meshCols-1)(L+1)+1 vectors | VPI |
| `MeshWithDelays::resp_history(k)`, lane l | stage k of the output de-skew `ShiftRegister`s of `resp.data` lane l, `resp.valid`, `resp.last` and the output matmul id | `:199-204, 232` | **correspondent**: RTL stage k at t == model response history entry k at t (what entered the de-skew at cycle t-1-k) | VPI |
| `MeshWithDelays::mesh_in()`, `Mesh::out()` | `mesh.io_in_*`, `mesh.io_out_*` | `:167-204` | equal (functions of the state above) | via state |

The register names behind the chains (which differ per configuration, e.g. `RegShifted_r_656_0`)
are found in the generated Verilog by following its register-to-register copies
(`tools/rtl_provenance.py correspondence` writes `<verilog>/<config>/correspondence.txt`); the
`rtl_corr_fault_<kind>` tests flip one model value of each kind and require the bench to
report it.

Registers the FIRRTL compiler removed because nothing reads them (`last_s`, `result_shift` and
`req_bits_pe_control_shift` in a WS-only array; the last stage of a `Pipe`'s valid chain) have no
RTL counterpart; the bench lists them (`NOTE model registers the FIRRTL compiler removed`).

## 6. Not modelled

- Everything outside MeshWithDelays (the ExecuteController, scratchpad, accumulator, DMA,
  load/store controllers, reservation station, loop unrollers, the RoCC interface).
- Floating-point PEs (hardfloat types), `shifter_banks > 1` (skew in `ShiftSRAM`).
- Chisel assertions: the model does not check them; the RTL bench stops if one fires.
- X / random initial values: all registers start at 0 (RTL with `RANDOMIZE_REG_INIT` differs
  until a register is written or reset).
- Configurations other than the five named ones are inside the supported range
  (`ArrayConfig::validate`) but not validated against the RTL.

## 7. Validation

What is checked, by which test, with which figures: [validation.md](validation.md).

## 8. Findings

- **Dataflow change without a flush can deadlock MeshWithDelays.** An OS request's tag waits
  for matmul id `m+3`, a WS request's for `m+2` (`MeshWithDelays.scala:219`); an OS request
  followed directly by a WS one makes both wait for the same id, the second misses it, and once
  six tags are queued `req.ready` stays low for good. Test `os_ws_no_flush` shows it on the RTL
  and the model alike (lockstep until the driver's watchdog). The ExecuteController flushes on
  a dataflow change (`ExecuteController.scala:623-630`); the bench's random requests do the same.
- **`shift` reaches the array a cycle after the other control fields**
  (`result_shift = RegNext(req.bits.pe_control.shift)`, `MeshWithDelays.scala:183`): when a
  request's first row enters the cycle after its handshake, that row, where an OS propagate
  flip applies the shift, carries the previous request's `shift`. The model reproduces it
  register for register (the random tests vary `shift` per request); the matmul tests keep it
  constant.
- The MacUnit result is `spatialArrayOutputType` wide, so OS partial sums wrap at 20 bits in a
  32-bit register (`PE.scala:23, 64-65`).
