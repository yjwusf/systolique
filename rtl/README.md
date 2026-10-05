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
