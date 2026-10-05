# Gemmini RTL: elaboration, lockstep bench, trace recording (optional)

Nothing here is needed to build the library or run the offline tests: the stored traces in
`tests/reference/gemmini_rtl/` check the model without any RTL. This directory makes the
references and runs the live lockstep when the tools are present.

## Pinned toolchain

| what | version | where |
|---|---|---|
| Gemmini | v0.7.2, `709bc56b6dd859fc2b1a9027a96a0b5be6ad7ed6`, unmodified | `GEMMINI_DIR` (default `~/opt/src/gemmini`) |
| berkeley-hardfloat | `9deaf1d49f487347be371641ba6ea637b669ad21` (the commit Chipyard pins for Gemmini v0.7.2) | `HARDFLOAT_DIR` |
| Chisel | 3.6.0, Scala FIRRTL compiler (`ChiselStage.emitVerilog`) | `build.sbt` |
| Scala / sbt | 2.13.10 / 1.10.11 | `build.sbt`, `project/build.properties` |
| JDK | 17 | `JAVA_HOME` |
| Verilator | 5.038 | `~/opt/verilator-5.038`, else `VERILATOR_ROOT`, else the PATH |

Only the Gemmini files the array needs are compiled (`Arithmetic, Dataflow, Mesh, MeshWithDelays,
PE, Shifter, SyncMem, TagQueue, Tile, Transposer, Util`); `src/GemminiTops.scala` adds
wiring-only tops (`MeshTop`, `MeshWithDelaysTop`) that pack the Vec ports, a `BenchTag` and the
generator main. `build.sbt`, `project/build.properties` and `src/GemminiTops.scala` are byte for
byte the sources the stored traces were made with (sha256 in
`tests/reference/gemmini_rtl/provenance.json`, checked by `systolique_reference_provenance`), so
their comments still name the directory they were first written in; edit them only together
with a new recording.

## Steps

```sh
rtl/elaborate.sh            # -> ~/opt/gemmini-verilog-709bc56/<config>/{MeshTop,MeshWithDelaysTop}.v,
                            #    correspondence.txt, provenance.json (not in git)
cmake -S . -B build         # finds Verilator and the Verilog, builds rtl_<config>
ctest --test-dir build -L rtl
```

`-DSYSTOLIQUE_VERILOG_DIR=<dir>` points at Verilog elsewhere; `-DSYSTOLIQUE_RTL=OFF` leaves the
part out. Without Verilator or the Verilog, CMake prints a warning and the `rtl` tests do not
exist; everything else is unchanged.

## Tests (label `rtl`)

- `rtl_<config>`: the Verilated MeshTop and MeshWithDelaysTop and `SystolicArray` get the same
  inputs every cycle (the stimulus reads the RTL's ready signals); every output port is compared
  every cycle; the RTL's outputs are checked not to change with the cycle's inputs; the internal
  correspondence (docs/microarchitecture.md section 5) is read through VPI before every clock edge;
  the RTL must still produce the stored trace of every catalog test; 20 seeds of random tests
  that are not stored follow; the accounting of every run must conserve.
- `rtl_corr_fault_<kind>`: `SYSTOLIQUE_CORR_FAULT=<kind>` flips one model value of that kind; the
  bench must report it.
- `rtl_verilog`: the Verilog's digests equal the ones the traces were made from.

## Recording new traces

Only when the stimulus or the generator changes on purpose:

```sh
python3 tools/rtl_provenance.py record --build build    # runs rtl_<config> --write-ref
```

It rewrites `tests/reference/gemmini_rtl/<config>/*.csv.gz` and `provenance.json` (Verilog
provenance, the Verilator that ran it, sha256 and cycles of every trace). Commit them together
with the change that needed them and a validation log entry (docs/validation.md).

## The frontend tops (`rtl/ex/`)

The ExecuteController with its scratchpad and accumulator banks (`ExecuteTop`), the command
path (`CmdTop`: raw_cmd_q, LoopConv, LoopMatmul, the unrolled queue, the ReservationStation)
and both wired as Gemmini's Controller wires them (`CtrlTop`), for the frontend classes of
[docs/micro_ops.md](../docs/micro_ops.md). Every Gemmini source but the Chipyard configurations
is compiled, unmodified, with the rocket-chip Gemmini v0.7.2 was released against:

| what | version | where |
|---|---|---|
| rocket-chip | `67ceb1ddbfd1c6f50d2b4fdadf68f304f5e62287` (pinned by Chipyard `ef3409f`, Gemmini v0.7.2's `CHIPYARD.hash`) | `ROCKETCHIP_DIR` (default `~/opt/src/rocket-chip-67ceb1d`) |
| cde | `384c06b8d45c8184ca2f3fba2f8e78f79d2c1b51` (rocket-chip's submodule) | `CDE_DIR` |
| hardfloat, Chisel, Scala, sbt, JDK, Verilator | as above | |

`src/MidasTargetutils.scala` stands in for FireSim's `midas.targetutils` (Gemmini uses it only
with `use_firesim_simulation_counters`, false in its defaultConfig); the three tops are wiring
only (ports packed as `bench/fe_ports.h` reads them; the DMA replaced by row-write test ports).

```sh
rtl/ex/elaborate_ex.sh      # -> ~/opt/gemmini-verilog-709bc56/frontend/{ExecuteTop,CmdTop,CtrlTop}.v,
                            #    plusarg_reader.v, provenance.json (not in git)
cmake -S . -B build         # builds rtl_fe (rtl/fe_bench.cpp) when Verilator and the Verilog exist
ctest --test-dir build -L rtl
python3 tools/rtl_provenance.py fe-record --build build   # re-record tests/reference/gemmini_fe/
```

Tests: `rtl_fe_<top>` (every directed test and random seeds of `bench/fe_stimulus.cpp`, every
output port lane every cycle), `rtl_fe_reference_<top>` (the RTL reproduces every stored trace),
`rtl_fe_fault_<top>` (a flipped model bit is reported), `rtl_fe_verilog` (the Verilog digests).
The frontend traces start at cycle 0: the four reset cycles before it are not stored.
