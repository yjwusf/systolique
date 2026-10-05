// Gemmini's ExecuteController with its scratchpad and accumulator banks, for the lockstep bench of
// systolique's ExecuteController, Scratchpad and Accumulator classes (rtl/ex_bench.cpp).
//
// ExecuteTop instantiates the unmodified ExecuteController (with the MeshWithDelays inside it),
// ScratchpadBank x sp_banks, AccumulatorMem x acc_banks and AccPipeShared, and wires them as
// Gemmini's Scratchpad (Scratchpad.scala:446-557, 618-823) and Controller (Controller.scala:
// 271-302) wire them for the ExecuteController. What the DMA does is replaced by test ports:
// the scratchpad / accumulator row writes that mvins make (one each per cycle, behind the
// ExecuteController's writes, as Scratchpad gives them lower priority), and no mvouts (so the
// accumulator read path through the normalizer and AccumulatorScale, which only DMA reads drain
// in v0.7.2, stays idle). The command port takes what the ReservationStation issues; Im2Col is
// tied to its !hasIm2Col defaults (Im2Col.scala:445-450). Every Vec port is packed into one UInt.
package systolique

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import gemmini._

class ExecuteTop(implicit p: Parameters) extends Module {
  override def desiredName = "ExecuteTop"
  val config = GemminiConfigs.defaultConfig
  import config._

  val spad_w = inputType.getWidth * meshColumns * tileColumns
  val acc_row_t = Vec(meshColumns, Vec(tileColumns, accType))
  val acc_w = acc_row_t.getWidth
  val robW = log2Up(reservation_station_entries)
  val spRowW = log2Ceil(sp_bank_entries)
  val accRowW = log2Ceil(acc_bank_entries)

  val io = IO(new Bundle {
    // the ReservationStation's execute issue (Controller.scala:263-266)
    val cmd_valid = Input(Bool())
    val cmd_ready = Output(Bool())
    val cmd_funct = Input(UInt(7.W))
    val cmd_rs1 = Input(UInt(64.W))
    val cmd_rs2 = Input(UInt(64.W))
    val cmd_rob_id = Input(UInt(robW.W))
    val completed_valid = Output(Bool())
    val completed_bits = Output(UInt(robW.W))
    val busy = Output(Bool())
    // test ports: an mvin's scratchpad row write and accumulator row write
    val dma_sp_en = Input(Bool())
    val dma_sp_bank = Input(UInt(log2Up(sp_banks).W))
    val dma_sp_addr = Input(UInt(spRowW.W))
    val dma_sp_data = Input(UInt(spad_w.W))
    val dma_sp_mask = Input(UInt((spad_w / 8).W))
    val dma_sp_taken = Output(Bool())
    val dma_acc_en = Input(Bool())
    val dma_acc_bank = Input(UInt(log2Up(acc_banks).W))
    val dma_acc_addr = Input(UInt(accRowW.W))
    val dma_acc_data = Input(UInt(acc_w.W))
    val dma_acc_mask = Input(UInt((acc_w / 8).W))
    val dma_acc_acc = Input(Bool())
    val dma_acc_taken = Output(Bool())
    // observed: the ExecuteController's bank ports (lane = bank)
    val sp_read_valid = Output(UInt(sp_banks.W))
    val sp_read_ready = Output(UInt(sp_banks.W))
    val sp_read_addr = Output(UInt((sp_banks * spRowW).W))
    val sp_resp_ready = Output(UInt(sp_banks.W))
    val sp_write_en = Output(UInt(sp_banks.W))
    val sp_write_addr = Output(UInt((sp_banks * spRowW).W))
    val sp_write_data = Output(UInt((sp_banks * spad_w).W))
    val sp_write_mask = Output(UInt((sp_banks * spad_w / 8).W))
    val acc_read_valid = Output(UInt(acc_banks.W))
    val acc_read_ready = Output(UInt(acc_banks.W))
    val acc_read_addr = Output(UInt((acc_banks * accRowW).W))
    val acc_write_valid = Output(UInt(acc_banks.W))
    val acc_write_addr = Output(UInt((acc_banks * accRowW).W))
    val acc_write_data = Output(UInt((acc_banks * acc_w).W))
    val acc_write_acc = Output(UInt(acc_banks.W))
    val acc_write_mask = Output(UInt((acc_banks * acc_w / 8).W))
  })

  val ex = Module(new ExecuteController(64, 32, config))

  // command port: a RoCCCommand with funct, rs1, rs2 (the other fields zero), rob id valid
  ex.io.cmd.valid := io.cmd_valid
  io.cmd_ready := ex.io.cmd.ready
  ex.io.cmd.bits := 0.U.asTypeOf(ex.io.cmd.bits)
  ex.io.cmd.bits.cmd.inst.funct := io.cmd_funct
  ex.io.cmd.bits.cmd.rs1 := io.cmd_rs1
  ex.io.cmd.bits.cmd.rs2 := io.cmd_rs2
  ex.io.cmd.bits.rob_id.valid := true.B
  ex.io.cmd.bits.rob_id.bits := io.cmd_rob_id
  io.completed_valid := ex.io.completed.valid
  io.completed_bits := ex.io.completed.bits
  io.busy := ex.io.busy

  // Im2Col with hasIm2Col false (Im2Col.scala:445-450)
  ex.io.im2col.req.ready := true.B
  ex.io.im2col.resp.valid := false.B
  ex.io.im2col.resp.bits := 0.U.asTypeOf(ex.io.im2col.resp.bits)
  ex.io.counter.external_reset := false.B

  // ---- scratchpad banks (Scratchpad.scala:446-557, without DMA mvout reads)
  val spBanks = Seq.fill(sp_banks)(Module(new ScratchpadBank(sp_bank_entries, spad_w, aligned_to,
    sp_singleported, use_shared_ext_mem, is_dummy)))
  val spTaken = Wire(Vec(sp_banks, Bool()))
  for ((bank, i) <- spBanks.zipWithIndex) {
    val bio = bank.io
    val ex_read_req = ex.io.srams.read(i).req
    bio.read.req.valid := ex_read_req.valid
    ex_read_req.ready := bio.read.req.ready
    bio.read.req.bits.addr := ex_read_req.bits.addr
    bio.read.req.bits.fromDMA := false.B

    val dma_read_resp = Wire(Decoupled(new ScratchpadReadResp(spad_w)))
    dma_read_resp.valid := bio.read.resp.valid && bio.read.resp.bits.fromDMA
    dma_read_resp.bits := bio.read.resp.bits
    val ex_read_resp = Wire(Decoupled(new ScratchpadReadResp(spad_w)))
    ex_read_resp.valid := bio.read.resp.valid && !bio.read.resp.bits.fromDMA
    ex_read_resp.bits := bio.read.resp.bits
    val dma_read_pipe = Pipeline(dma_read_resp, spad_read_delay)
    val ex_read_pipe = Pipeline(ex_read_resp, spad_read_delay)
    bio.read.resp.ready := Mux(bio.read.resp.bits.fromDMA, dma_read_resp.ready, ex_read_resp.ready)
    dma_read_pipe.ready := false.B  // no StreamWriter: no mvouts in this harness
    ex.io.srams.read(i).resp <> ex_read_pipe

    val exwrite = ex.io.srams.write(i).en
    val dmaread = io.dma_sp_en && io.dma_sp_bank === i.U
    bio.write.en := exwrite || dmaread
    when(exwrite) {
      bio.write.addr := ex.io.srams.write(i).addr
      bio.write.data := ex.io.srams.write(i).data
      bio.write.mask := ex.io.srams.write(i).mask
    }.elsewhen(dmaread) {
      bio.write.addr := io.dma_sp_addr
      bio.write.data := io.dma_sp_data
      bio.write.mask := io.dma_sp_mask.asBools
    }.otherwise {
      bio.write.addr := DontCare
      bio.write.data := DontCare
      bio.write.mask := DontCare
    }
    spTaken(i) := dmaread && !exwrite  // the pixel repeater's resp.ready (Scratchpad.scala:538-543)
  }
  io.dma_sp_taken := spTaken.asUInt.orR

  // ---- accumulator banks (Scratchpad.scala:628-823; the read responses only drain to the DMA)
  val acc_adders = Module(new AccPipeShared(acc_latency - 1, acc_row_t, acc_banks))
  val accBanks = Seq.fill(acc_banks)(Module(new AccumulatorMem(acc_bank_entries, acc_row_t, acc_scale_func,
    acc_scale_t.asInstanceOf[Float], acc_singleported, acc_sub_banks, use_shared_ext_mem, acc_latency,
    accType, is_dummy)))
  val accTaken = Wire(Vec(acc_banks, Bool()))
  for ((bank, i) <- accBanks.zipWithIndex) {
    val bio = bank.io
    acc_adders.io.in_sel(i) := bio.adder.valid
    acc_adders.io.ina(i) := bio.adder.op1
    acc_adders.io.inb(i) := bio.adder.op2
    bio.adder.sum := acc_adders.io.out

    val ex_read_req = ex.io.acc.read_req(i)
    bio.read.req.valid := ex_read_req.valid
    ex_read_req.ready := bio.read.req.ready
    bio.read.req.bits.addr := ex_read_req.bits.addr
    bio.read.req.bits.act := ex_read_req.bits.act
    bio.read.req.bits.igelu_qb := ex_read_req.bits.igelu_qb
    bio.read.req.bits.igelu_qc := ex_read_req.bits.igelu_qc
    bio.read.req.bits.iexp_qln2 := ex_read_req.bits.iexp_qln2
    bio.read.req.bits.iexp_qln2_inv := ex_read_req.bits.iexp_qln2_inv
    bio.read.req.bits.scale := ex_read_req.bits.scale
    bio.read.req.bits.full := false.B
    bio.read.req.bits.fromDMA := false.B
    bio.read.resp.ready := false.B  // (Scratchpad.scala:695-714: drained only for DMA mvouts)
    ex.io.acc.read_resp(i).valid := false.B
    ex.io.acc.read_resp(i).bits := 0.U.asTypeOf(ex.io.acc.read_resp(i).bits)

    val exwrite = ex.io.acc.write(i).valid
    ex.io.acc.write(i).ready := true.B
    val dmaread = io.dma_acc_en && io.dma_acc_bank === i.U
    bio.write.valid := false.B
    bio.write.bits.acc := Mux(exwrite, ex.io.acc.write(i).bits.acc, io.dma_acc_acc)
    bio.write.bits.addr := Mux(exwrite, ex.io.acc.write(i).bits.addr, io.dma_acc_addr)
    when(exwrite) {
      bio.write.valid := true.B
      bio.write.bits.data := ex.io.acc.write(i).bits.data
      bio.write.bits.mask := ex.io.acc.write(i).bits.mask
    }.elsewhen(dmaread) {
      bio.write.valid := true.B
      bio.write.bits.data := io.dma_acc_data.asTypeOf(acc_row_t)
      bio.write.bits.mask := io.dma_acc_mask.asBools
    }.otherwise {
      bio.write.bits.data := DontCare
      bio.write.bits.mask := DontCare
    }
    accTaken(i) := dmaread && !exwrite && bio.write.ready
  }
  io.dma_acc_taken := accTaken.asUInt.orR

  // ---- observed ports
  def pack(xs: Seq[UInt]): UInt = VecInit(xs).asUInt
  io.sp_read_valid := pack(ex.io.srams.read.map(_.req.valid))
  io.sp_read_ready := pack(ex.io.srams.read.map(_.req.ready))
  io.sp_read_addr := pack(ex.io.srams.read.map(_.req.bits.addr))
  io.sp_resp_ready := pack(ex.io.srams.read.map(_.resp.ready))
  io.sp_write_en := pack(ex.io.srams.write.map(_.en))
  io.sp_write_addr := pack(ex.io.srams.write.map(_.addr))
  io.sp_write_data := pack(ex.io.srams.write.map(_.data))
  io.sp_write_mask := pack(ex.io.srams.write.map(_.mask.asUInt))
  io.acc_read_valid := pack(ex.io.acc.read_req.map(_.valid))
  io.acc_read_ready := pack(ex.io.acc.read_req.map(_.ready))
  io.acc_read_addr := pack(ex.io.acc.read_req.map(_.bits.addr))
  io.acc_write_valid := pack(ex.io.acc.write.map(_.valid))
  io.acc_write_addr := pack(ex.io.acc.write.map(_.bits.addr))
  io.acc_write_data := pack(ex.io.acc.write.map(_.bits.data.asUInt))
  io.acc_write_acc := pack(ex.io.acc.write.map(_.bits.acc))
  io.acc_write_mask := pack(ex.io.acc.write.map(_.bits.mask.asUInt))
}

// sbt "run <out_dir>": <out_dir>/ExecuteTop.v
object ElaborateEx extends App {
  require(args.nonEmpty, "usage: ElaborateEx <out_dir>")
  // a Rocket tile's parameters for the RoCCCommand inside GemminiCmd (XLen 64)
  implicit val p: Parameters = (new freechips.rocketchip.system.DefaultConfig).alterPartial {
    case freechips.rocketchip.tile.TileKey => freechips.rocketchip.tile.RocketTileParams()
  }
  (new chisel3.stage.ChiselStage).emitVerilog(new ExecuteTop, Array("--target-dir", args(0)))
}
