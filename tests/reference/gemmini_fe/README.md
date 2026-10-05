# Stored Gemmini frontend RTL traces

`<top>/<test>.csv.gz`: the inputs and outputs of a Verilated frontend top (`rtl/ex/src/`:
`ExecuteTop`, the ExecuteController with its scratchpad and accumulator banks; `CmdTop`, the
command path; `CtrlTop`, both wired as Gemmini's Controller) in every clock cycle of one stimulus
of `bench/fe_stimulus.cpp` (a directed test of its catalog or `random_<seed>`), from cycle 0:
the four cycles with reset high before it are not stored. Format: `#` comment lines (the Gemmini
commit, the Verilog digest, the stimulus and the bench that recorded it), a header
`cycle,<port>,...` with every port in `bench/fe_ports.cpp` order (inputs, then outputs), then
one row per cycle with the packed lanes in hex (lane 0 in the least significant bits).

97 traces, 127,259 cycles: every directed test and random seeds 1-24 (ExecuteTop, 32 traces),
1-40 (CmdTop, 46), 1-12 (CtrlTop, 19).

`provenance.json` says how they were made (Gemmini, rocket-chip, cde and hardfloat commits,
Chisel, Scala, Java, the generator `rtl/ex/elaborate_ex.sh` and the sha256 of its sources, the
Verilog digests, Verilator) and lists every trace with its sha256 and cycle count
(`systolique_fe_reference_provenance` checks them).

Tests that use them: `systolique_fe_trace_<top>` (replay through the model, regeneration from the
stimuli, Engine results), and with the RTL `rtl_fe_reference_<top>` (the RTL still produces
them). Re-recording: `python3 tools/rtl_provenance.py fe-record --build <build>` (`rtl/README.md`).
