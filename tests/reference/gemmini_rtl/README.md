# Stored Gemmini RTL traces

`<config>/<test>.csv.gz`: the inputs and outputs of the Verilated Gemmini RTL top (MeshTop or
MeshWithDelaysTop, `rtl/src/GemminiTops.scala`) in every clock cycle of one test of the catalog
(`bench/stimulus.cpp`, `test_catalog`), from power-on: `#` comment lines (what made the trace:
the test, the Gemmini commit, the digests of the Verilog), a header `cycle,<port>,...` with every
port in `bench/ports.cpp` order (inputs, then outputs), then one row per cycle with the packed
lanes in hex (lane 0 in the least significant bits). The first 4 rows hold reset.

86 traces, 29,804 cycles, 5 configurations (`default`, `dim4`, `tiled`, `ws_tree`, `os16`).

`provenance.json` says how they were made (Gemmini commit and files, hardfloat, Chisel, Scala,
Java, the generator and its sources' sha256, the configurations, the Verilog digests, Verilator)
and lists every trace with its sha256 and cycle count. The traces were recorded by the lockstep
bench this repository's `rtl/rtl_bench.cpp` descends from; their comment lines name that bench.
They are kept byte for byte so the hashes stay valid (`systolique_reference_provenance`).

Tests that use them: `systolique_trace_<config>` (replay, regeneration, conservation, provenance,
matmul results), `systolique_vcd`, and with the RTL `rtl_<config>` (the RTL still produces them).
Re-recording: `rtl/README.md`.
