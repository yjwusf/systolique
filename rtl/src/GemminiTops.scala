// Wiring-only top levels around Gemmini's Mesh and MeshWithDelays, and the generator main.
//
// The wrappers add no logic: every Vec port of the Gemmini module is packed into one flat UInt
// (lane k = Vec(i)(j) with k = i * tileColumns + j at bits [k*w +: w], i.e. Chisel's asUInt
// order) and every bundle field gets its own port, so the Verilated tops of all configurations
// have the same port names. accelsim/gemmini/gemmini_config.h holds the same configurations.

package accelsim

import chisel3._
import chisel3.util._
import gemmini._

// One Gemmini array configuration (the GemminiArrayConfig fields the array uses:
// gemmini/GemminiConfigs.scala:17-82; defaults from gemmini/Configs.scala:20-35).
case class ArrayCfg(name: String, inBits: Int, outBits: Int, accBits: Int, df: Dataflow.Value,
                    tileRows: Int, tileColumns: Int, meshRows: Int, meshColumns: Int,
                    tileLatency: Int, outputDelay: Int, tagBits: Int = 8) {
  // gemmini/GemminiConfigs.scala:175 (use_tree_reduction_if_possible = true)
  def treeReduction: Boolean = df == Dataflow.WS && tileRows > 1
  def blockSize: Int = meshRows * tileRows
  // gemmini/MeshWithDelays.scala:48-53 (n_simultaneous_matmuls = -1)
  def maxSimultaneousMatmuls: Int = {
    val latencyPerPe = ((tileLatency + 1).toFloat / (tileRows min tileColumns)) max 1.0f
    (5 * latencyPerPe).ceil.toInt
  }
  def inType = SInt(inBits.W)
  def outType = SInt(outBits.W)
  def accType = SInt(accBits.W)
}

object ArrayCfgs {
  val all = Seq(
    // Gemmini's defaultConfig: int8 inputs, int20 array outputs, int32 accumulators, 16x16 PEs as
    // meshRows x tileRows = 16 x 1, both dataflows, tile_latency 0, mesh_output_delay 1.
    ArrayCfg("default", 8, 20, 32, Dataflow.BOTH, 1, 1, 16, 16, 0, 1),
    ArrayCfg("dim4", 8, 20, 32, Dataflow.BOTH, 1, 1, 4, 4, 0, 1),
    // Combinational 2x2 tiles, a register between tiles of depth 2, 2 output delay stages.
    ArrayCfg("tiled", 8, 20, 32, Dataflow.BOTH, 2, 2, 4, 4, 1, 2),
    // Weight-stationary only with 2x2 tiles: tree reduction of the partial sums inside a tile.
    ArrayCfg("ws_tree", 8, 20, 32, Dataflow.WS, 2, 2, 4, 4, 0, 1),
    // Output-stationary only, 16-bit inputs, 24-bit outputs.
    ArrayCfg("os16", 16, 24, 32, Dataflow.OS, 1, 1, 8, 8, 0, 1),
  )
  def apply(name: String): ArrayCfg = all.find(_.name == name).getOrElse(
    throw new IllegalArgumentException(s"unknown configuration $name"))
}

object Pack {
  def apply[T <: Data](xs: Seq[T]): UInt = VecInit(xs.map(_.asUInt)).asUInt
  def lane(u: UInt, k: Int, w: Int): UInt = u(k * w + w - 1, k * w)
}

// Mesh (gemmini/Mesh.scala:17-36) with packed ports.
class MeshTop(c: ArrayCfg) extends Module {
  override def desiredName = "MeshTop"
  val rows = c.meshRows * c.tileRows
  val cols = c.meshColumns * c.tileColumns
  val sw = log2Up(c.accBits)                       // PEControl.shift (gemmini/PE.scala:10)
  val iw = log2Up(c.maxSimultaneousMatmuls)       // id (gemmini/Mesh.scala:27)
  val io = IO(new Bundle {
    val in_a = Input(UInt((rows * c.inBits).W))
    val in_b = Input(UInt((cols * c.inBits).W))
    val in_d = Input(UInt((cols * c.inBits).W))
    val in_dataflow = Input(UInt(cols.W))
    val in_propagate = Input(UInt(cols.W))
    val in_shift = Input(UInt((cols * sw).W))
    val in_id = Input(UInt((cols * iw).W))
    val in_last = Input(UInt(cols.W))
    val in_valid = Input(UInt(cols.W))
    val out_b = Output(UInt((cols * c.outBits).W))
    val out_c = Output(UInt((cols * c.outBits).W))
    val out_valid = Output(UInt(cols.W))
    val out_dataflow = Output(UInt(cols.W))
    val out_propagate = Output(UInt(cols.W))
    val out_shift = Output(UInt((cols * sw).W))
    val out_id = Output(UInt((cols * iw).W))
    val out_last = Output(UInt(cols.W))
  })
  val mesh = Module(new Mesh(c.inType, c.outType, c.accType, c.df, c.treeReduction, c.tileLatency,
    c.maxSimultaneousMatmuls, c.outputDelay, c.tileRows, c.tileColumns, c.meshRows, c.meshColumns))
  for (i <- 0 until c.meshRows; j <- 0 until c.tileRows)
    mesh.io.in_a(i)(j) := Pack.lane(io.in_a, i * c.tileRows + j, c.inBits).asSInt
  for (i <- 0 until c.meshColumns; j <- 0 until c.tileColumns) {
    val k = i * c.tileColumns + j
    mesh.io.in_b(i)(j) := Pack.lane(io.in_b, k, c.inBits).asSInt
    mesh.io.in_d(i)(j) := Pack.lane(io.in_d, k, c.inBits).asSInt
    mesh.io.in_control(i)(j).dataflow := io.in_dataflow(k)
    mesh.io.in_control(i)(j).propagate := io.in_propagate(k)
    mesh.io.in_control(i)(j).shift := Pack.lane(io.in_shift, k, sw)
    mesh.io.in_id(i)(j) := Pack.lane(io.in_id, k, iw)
    mesh.io.in_last(i)(j) := io.in_last(k)
    mesh.io.in_valid(i)(j) := io.in_valid(k)
  }
  io.out_b := Pack(mesh.io.out_b.flatten)
  io.out_c := Pack(mesh.io.out_c.flatten)
  io.out_valid := Pack(mesh.io.out_valid.flatten)
  io.out_dataflow := Pack(mesh.io.out_control.flatten.map(_.dataflow))
  io.out_propagate := Pack(mesh.io.out_control.flatten.map(_.propagate))
  io.out_shift := Pack(mesh.io.out_control.flatten.map(_.shift))
  io.out_id := Pack(mesh.io.out_id.flatten)
  io.out_last := Pack(mesh.io.out_last.flatten)
}

// The tag the bench sends with every request. Gemmini's ExecuteController uses a ROB id, an
// address and the C size (gemmini/ExecuteController.scala:52-62); the array only stores and
// returns the tag, so a valid bit and an id are enough. Garbage = not valid, id all ones.
class BenchTag(val w: Int) extends Bundle with TagQueueTag {
  val valid = Bool()
  val id = UInt(w.W)
  override def make_this_garbage(dummy: Int = 0): Unit = {
    valid := false.B
    id := ((BigInt(1) << w) - 1).U
  }
}

// MeshWithDelays (gemmini/MeshWithDelays.scala:32-68) with packed ports; shifter banks = 1 as in
// the ExecuteController (gemmini/ExecuteController.scala:186-187, shifter_banks default 1).
class MeshWithDelaysTop(c: ArrayCfg) extends Module {
  override def desiredName = "MeshWithDelaysTop"
  val dim = c.blockSize
  val cols = c.meshColumns * c.tileColumns
  val sw = log2Up(c.accBits)
  val rw = log2Up(dim + 1)
  val tagqlen = c.maxSimultaneousMatmuls + 1
  val io = IO(new Bundle {
    val a_valid = Input(Bool())
    val a_ready = Output(Bool())
    val a_bits = Input(UInt((dim * c.inBits).W))
    val b_valid = Input(Bool())
    val b_ready = Output(Bool())
    val b_bits = Input(UInt((cols * c.inBits).W))
    val d_valid = Input(Bool())
    val d_ready = Output(Bool())
    val d_bits = Input(UInt((cols * c.inBits).W))
    val req_valid = Input(Bool())
    val req_ready = Output(Bool())
    val req_dataflow = Input(UInt(1.W))
    val req_propagate = Input(UInt(1.W))
    val req_shift = Input(UInt(sw.W))
    val req_a_transpose = Input(Bool())
    val req_bd_transpose = Input(Bool())
    val req_total_rows = Input(UInt(rw.W))
    val req_tag_valid = Input(Bool())
    val req_tag_id = Input(UInt(c.tagBits.W))
    val req_flush = Input(UInt(2.W))
    val resp_valid = Output(Bool())
    val resp_data = Output(UInt((cols * c.outBits).W))
    val resp_total_rows = Output(UInt(rw.W))
    val resp_tag_valid = Output(Bool())
    val resp_tag_id = Output(UInt(c.tagBits.W))
    val resp_last = Output(Bool())
    val tags_valid = Output(UInt(tagqlen.W))
    val tags_id = Output(UInt((tagqlen * c.tagBits).W))
  })
  val m = Module(new MeshWithDelays(c.inType, c.outType, c.accType, new BenchTag(c.tagBits), c.df,
    c.treeReduction, c.tileLatency, c.outputDelay, c.tileRows, c.tileColumns, c.meshRows,
    c.meshColumns, 1, 1))
  m.io.a.valid := io.a_valid
  io.a_ready := m.io.a.ready
  for (i <- 0 until c.meshRows; j <- 0 until c.tileRows)
    m.io.a.bits(i)(j) := Pack.lane(io.a_bits, i * c.tileRows + j, c.inBits).asSInt
  m.io.b.valid := io.b_valid
  io.b_ready := m.io.b.ready
  m.io.d.valid := io.d_valid
  io.d_ready := m.io.d.ready
  for (i <- 0 until c.meshColumns; j <- 0 until c.tileColumns) {
    val k = i * c.tileColumns + j
    m.io.b.bits(i)(j) := Pack.lane(io.b_bits, k, c.inBits).asSInt
    m.io.d.bits(i)(j) := Pack.lane(io.d_bits, k, c.inBits).asSInt
  }
  m.io.req.valid := io.req_valid
  io.req_ready := m.io.req.ready
  m.io.req.bits.pe_control.dataflow := io.req_dataflow
  m.io.req.bits.pe_control.propagate := io.req_propagate
  m.io.req.bits.pe_control.shift := io.req_shift
  m.io.req.bits.a_transpose := io.req_a_transpose
  m.io.req.bits.bd_transpose := io.req_bd_transpose
  m.io.req.bits.total_rows := io.req_total_rows
  m.io.req.bits.tag.valid := io.req_tag_valid
  m.io.req.bits.tag.id := io.req_tag_id
  m.io.req.bits.flush := io.req_flush
  io.resp_valid := m.io.resp.valid
  io.resp_data := Pack(m.io.resp.bits.data.flatten)
  io.resp_total_rows := m.io.resp.bits.total_rows
  io.resp_tag_valid := m.io.resp.bits.tag.valid
  io.resp_tag_id := m.io.resp.bits.tag.id
  io.resp_last := m.io.resp.bits.last
  io.tags_valid := Pack(m.io.tags_in_progress.map(_.valid))
  io.tags_id := Pack(m.io.tags_in_progress.map(_.id))
}

// sbt "run <out_dir> [config ...]": <out_dir>/<config>/{MeshTop,MeshWithDelaysTop}.v
object Elaborate extends App {
  require(args.nonEmpty, "usage: Elaborate <out_dir> [config ...]")
  val names = if (args.length > 1) args.toSeq.tail else ArrayCfgs.all.map(_.name)
  for (n <- names) {
    val c = ArrayCfgs(n)
    val dir = s"${args(0)}/$n"
    (new chisel3.stage.ChiselStage).emitVerilog(new MeshTop(c), Array("--target-dir", dir))
    (new chisel3.stage.ChiselStage).emitVerilog(new MeshWithDelaysTop(c), Array("--target-dir", dir))
  }
}
