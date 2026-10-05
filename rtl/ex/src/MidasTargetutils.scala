// No-op stand-ins for FireSim's midas.targetutils (firesim sim/midas/targetutils), which
// Gemmini imports for FireSim performance counters and synthesized printfs. Gemmini uses them
// only when use_firesim_simulation_counters is set (false in its defaultConfig), so they never
// produce hardware here.
package midas.targetutils

import chisel3._

object PerfCounter {
  def apply(target: Bool, label: String, description: String): Unit = {}
}

object SynthesizePrintf {
  def apply(format: String, args: Bits*): Printable = Printable.pack(format, args: _*)
}
