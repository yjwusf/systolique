// Gemmini's command path and ExecuteController together, wired as Gemmini's Controller wires them,
// for the lockstep bench of systolique's Engine (rtl/ex_bench.cpp): CmdTop (raw_cmd_q, LoopConv,
// LoopMatmul, the unrolled-command queue, the ReservationStation) issues its execute commands to
// ExecuteTop (the ExecuteController with its MeshWithDelays, scratchpad banks, accumulator banks)
// (Controller.scala:243-246), and the ReservationStation's completions come through the
// three-input Arbiter of Controller.scala:312-327 (execute first, then load, then store).
//
// What is left to the bench: the load and store controllers (the ReservationStation's ld / st
// issue ports and their completion ports) and the DMA's bank writes (ExecuteTop's test write
// ports). Everything the two tops already observe is observed here as well.
package systolique

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import gemmini._

class CtrlTop(implicit p: Parameters) extends Module {
  override def desiredName = "CtrlTop"
  val config = GemminiConfigs.defaultConfig
  import config._

  val cmd = Module(new CmdTop)
  val ex = Module(new ExecuteTop)
  val robW = log2Up(reservation_station_entries)

  val io = IO(new Bundle {
    // the core
    val cmd_valid = Input(Bool())
    val cmd_ready = Output(Bool())
    val cmd_funct = Input(UInt(7.W))
    val cmd_rs1 = Input(UInt(64.W))
    val cmd_rs2 = Input(UInt(64.W))
    val busy = Output(Bool())
    val loop_matmul_busy = Output(Bool())
    // the load and store controllers: issue (from the ReservationStation) and completion
    val ld = new cmd.IssuePort
    val st = new cmd.IssuePort
    val ld_completed_valid = Input(Bool())
    val ld_completed_ready = Output(Bool())
    val ld_completed_bits = Input(UInt(robW.W))
    val st_completed_valid = Input(Bool())
    val st_completed_ready = Output(Bool())
    val st_completed_bits = Input(UInt(robW.W))
    // observed: the execute issue and completion
    val ex_valid = Output(Bool())
    val ex_ready = Output(Bool())
    val ex_funct = Output(UInt(7.W))
    val ex_rs1 = Output(UInt(64.W))
    val ex_rs2 = Output(UInt(64.W))
    val ex_rob_id = Output(UInt(robW.W))
    val ex_completed_valid = Output(Bool())
    val ex_completed_bits = Output(UInt(robW.W))
    val ex_busy = Output(Bool())
    // the DMA's bank writes and the ExecuteController's bank ports, as ExecuteTop's
    val dma_sp_en = Input(Bool())
    val dma_sp_bank = Input(chiselTypeOf(ex.io.dma_sp_bank))
    val dma_sp_addr = Input(chiselTypeOf(ex.io.dma_sp_addr))
    val dma_sp_data = Input(chiselTypeOf(ex.io.dma_sp_data))
    val dma_sp_mask = Input(chiselTypeOf(ex.io.dma_sp_mask))
    val dma_sp_taken = Output(Bool())
    val dma_acc_en = Input(Bool())
    val dma_acc_bank = Input(chiselTypeOf(ex.io.dma_acc_bank))
    val dma_acc_addr = Input(chiselTypeOf(ex.io.dma_acc_addr))
    val dma_acc_data = Input(chiselTypeOf(ex.io.dma_acc_data))
    val dma_acc_mask = Input(chiselTypeOf(ex.io.dma_acc_mask))
    val dma_acc_acc = Input(Bool())
    val dma_acc_taken = Output(Bool())
    val sp_read_valid = Output(chiselTypeOf(ex.io.sp_read_valid))
    val sp_read_ready = Output(chiselTypeOf(ex.io.sp_read_ready))
    val sp_read_addr = Output(chiselTypeOf(ex.io.sp_read_addr))
    val sp_resp_ready = Output(chiselTypeOf(ex.io.sp_resp_ready))
    val sp_write_en = Output(chiselTypeOf(ex.io.sp_write_en))
    val sp_write_addr = Output(chiselTypeOf(ex.io.sp_write_addr))
    val sp_write_data = Output(chiselTypeOf(ex.io.sp_write_data))
    val sp_write_mask = Output(chiselTypeOf(ex.io.sp_write_mask))
    val acc_read_valid = Output(chiselTypeOf(ex.io.acc_read_valid))
    val acc_read_ready = Output(chiselTypeOf(ex.io.acc_read_ready))
    val acc_read_addr = Output(chiselTypeOf(ex.io.acc_read_addr))
    val acc_write_valid = Output(chiselTypeOf(ex.io.acc_write_valid))
    val acc_write_addr = Output(chiselTypeOf(ex.io.acc_write_addr))
    val acc_write_data = Output(chiselTypeOf(ex.io.acc_write_data))
    val acc_write_acc = Output(chiselTypeOf(ex.io.acc_write_acc))
    val acc_write_mask = Output(chiselTypeOf(ex.io.acc_write_mask))
  })

  cmd.io.cmd_valid := io.cmd_valid
  io.cmd_ready := cmd.io.cmd_ready
  cmd.io.cmd_funct := io.cmd_funct
  cmd.io.cmd_rs1 := io.cmd_rs1
  cmd.io.cmd_rs2 := io.cmd_rs2
  io.busy := cmd.io.busy
  io.loop_matmul_busy := cmd.io.loop_matmul_busy
  io.ld <> cmd.io.ld
  io.st <> cmd.io.st

  // the execute issue (Controller.scala:243-246)
  ex.io.cmd_valid := cmd.io.ex.valid
  cmd.io.ex.ready := ex.io.cmd_ready
  ex.io.cmd_funct := cmd.io.ex.funct
  ex.io.cmd_rs1 := cmd.io.ex.rs1
  ex.io.cmd_rs2 := cmd.io.ex.rs2
  ex.io.cmd_rob_id := cmd.io.ex.rob_id
  io.ex_valid := cmd.io.ex.valid
  io.ex_ready := ex.io.cmd_ready
  io.ex_funct := cmd.io.ex.funct
  io.ex_rs1 := cmd.io.ex.rs1
  io.ex_rs2 := cmd.io.ex.rs2
  io.ex_rob_id := cmd.io.ex.rob_id
  io.ex_completed_valid := ex.io.completed_valid
  io.ex_completed_bits := ex.io.completed_bits
  io.ex_busy := ex.io.busy

  // completions (Controller.scala:312-327)
  val arb = Module(new Arbiter(UInt(robW.W), 3))
  arb.io.in(0).valid := ex.io.completed_valid
  arb.io.in(0).bits := ex.io.completed_bits
  arb.io.in(1).valid := io.ld_completed_valid
  arb.io.in(1).bits := io.ld_completed_bits
  io.ld_completed_ready := arb.io.in(1).ready
  arb.io.in(2).valid := io.st_completed_valid
  arb.io.in(2).bits := io.st_completed_bits
  io.st_completed_ready := arb.io.in(2).ready
  cmd.io.completed_valid := arb.io.out.valid
  cmd.io.completed_bits := arb.io.out.bits
  arb.io.out.ready := true.B

  // the DMA test ports in, the bank ports out (ExecuteTop's command and completion ports are
  // driven and observed above)
  ex.io.dma_sp_en := io.dma_sp_en
  ex.io.dma_sp_bank := io.dma_sp_bank
  ex.io.dma_sp_addr := io.dma_sp_addr
  ex.io.dma_sp_data := io.dma_sp_data
  ex.io.dma_sp_mask := io.dma_sp_mask
  ex.io.dma_acc_en := io.dma_acc_en
  ex.io.dma_acc_bank := io.dma_acc_bank
  ex.io.dma_acc_addr := io.dma_acc_addr
  ex.io.dma_acc_data := io.dma_acc_data
  ex.io.dma_acc_mask := io.dma_acc_mask
  ex.io.dma_acc_acc := io.dma_acc_acc
  io.dma_sp_taken := ex.io.dma_sp_taken
  io.dma_acc_taken := ex.io.dma_acc_taken
  io.sp_read_valid := ex.io.sp_read_valid
  io.sp_read_ready := ex.io.sp_read_ready
  io.sp_read_addr := ex.io.sp_read_addr
  io.sp_resp_ready := ex.io.sp_resp_ready
  io.sp_write_en := ex.io.sp_write_en
  io.sp_write_addr := ex.io.sp_write_addr
  io.sp_write_data := ex.io.sp_write_data
  io.sp_write_mask := ex.io.sp_write_mask
  io.acc_read_valid := ex.io.acc_read_valid
  io.acc_read_ready := ex.io.acc_read_ready
  io.acc_read_addr := ex.io.acc_read_addr
  io.acc_write_valid := ex.io.acc_write_valid
  io.acc_write_addr := ex.io.acc_write_addr
  io.acc_write_data := ex.io.acc_write_data
  io.acc_write_acc := ex.io.acc_write_acc
  io.acc_write_mask := ex.io.acc_write_mask
}

object ElaborateCtrl extends App {
  require(args.nonEmpty, "usage: ElaborateCtrl <out_dir>")
  implicit val p: Parameters = (new freechips.rocketchip.system.DefaultConfig).alterPartial {
    case freechips.rocketchip.tile.TileKey => freechips.rocketchip.tile.RocketTileParams()
  }
  (new chisel3.stage.ChiselStage).emitVerilog(new CtrlTop, Array("--target-dir", args(0)))
}
