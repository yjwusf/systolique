// Gemmini's command path, for the lockstep bench of systolique's command-path classes (rtl/ex_bench.cpp):
// raw_cmd_q, LoopConv, LoopMatmul, the unrolled-command queue and the ReservationStation, all
// unmodified and wired as Gemmini's Controller wires them (Controller.scala:124-170, 233-246,
// 312-330, 355-401).
//
// The command port takes what the core sends (a RoCCCommand with funct, rs1, rs2; the other
// fields zero). The ReservationStation's three issue ports and its completion port are the
// bench's: it plays the load, store and execute controllers (when they accept a command and
// when they complete one). Counter operations are not driven (the CounterController is left
// out), CLKGATE_EN is not driven (the clock is not gated) and the TLB flush is accepted as
// Controller.scala:376-380 does. coreMaxAddrBits is Rocket's with Sv39 (vaddrBitsExtended 40),
// which the model uses too (FrontendConfig::core_max_addr_bits).
package systolique

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import gemmini._
import GemminiISA._

class CmdTop(implicit p: Parameters) extends Module {
  override def desiredName = "CmdTop"
  val config = GemminiConfigs.defaultConfig
  import config._

  val coreMaxAddrBits = 40
  val robW = log2Up(reservation_station_entries)

  class IssuePort extends Bundle {
    val valid = Output(Bool())
    val ready = Input(Bool())
    val funct = Output(UInt(7.W))
    val rs1 = Output(UInt(64.W))
    val rs2 = Output(UInt(64.W))
    val rob_id = Output(UInt(robW.W))
  }

  val io = IO(new Bundle {
    val cmd_valid = Input(Bool())
    val cmd_ready = Output(Bool())
    val cmd_funct = Input(UInt(7.W))
    val cmd_rs1 = Input(UInt(64.W))
    val cmd_rs2 = Input(UInt(64.W))
    val ld = new IssuePort
    val ex = new IssuePort
    val st = new IssuePort
    val completed_valid = Input(Bool())
    val completed_bits = Input(UInt(robW.W))
    // io.busy without the scratchpad's (Controller.scala:330)
    val busy = Output(Bool())
    val loop_matmul_busy = Output(Bool())
    val matmul_ld_completed = Output(UInt(2.W))
    val matmul_ex_completed = Output(UInt(2.W))
    val matmul_st_completed = Output(UInt(2.W))
  })

  val reservation_station = Module(new ReservationStation(config, new GemminiCmd(reservation_station_entries)))
  reservation_station.io.counter.external_reset := false.B

  val raw_cmd_q = Module(new Queue(new GemminiCmd(reservation_station_entries), entries = 2))
  raw_cmd_q.io.enq.valid := io.cmd_valid
  io.cmd_ready := raw_cmd_q.io.enq.ready
  raw_cmd_q.io.enq.bits := 0.U.asTypeOf(raw_cmd_q.io.enq.bits)
  raw_cmd_q.io.enq.bits.cmd.inst.funct := io.cmd_funct
  raw_cmd_q.io.enq.bits.cmd.rs1 := io.cmd_rs1
  raw_cmd_q.io.enq.bits.cmd.rs2 := io.cmd_rs2
  val raw_cmd = raw_cmd_q.io.deq

  val max_lds = reservation_station_entries_ld
  val max_exs = reservation_station_entries_ex
  val max_sts = reservation_station_entries_st

  val (conv_cmd, loop_conv_unroller_busy) = LoopConv(raw_cmd, reservation_station.io.conv_ld_completed,
    reservation_station.io.conv_st_completed, reservation_station.io.conv_ex_completed,
    meshRows*tileRows, coreMaxAddrBits, reservation_station_entries, max_lds, max_exs, max_sts,
    sp_banks * sp_bank_entries, acc_banks * acc_bank_entries,
    inputType.getWidth, accType.getWidth, dma_maxbytes,
    new ConfigMvinRs1(mvin_scale_t_bits, block_stride_bits, pixel_repeats_bits),
    new MvinRs2(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new ConfigMvoutRs2(acc_scale_t_bits, 32), new MvoutRs2(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ConfigExRs1(acc_scale_t_bits), new PreloadRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new PreloadRs(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    has_training_convs, has_max_pool, has_first_layer_optimizations, has_dw_convs)

  val (loop_cmd, loop_matmul_unroller_busy) = LoopMatmul(conv_cmd, reservation_station.io.matmul_ld_completed,
    reservation_station.io.matmul_st_completed, reservation_station.io.matmul_ex_completed,
    meshRows*tileRows, coreMaxAddrBits, reservation_station_entries, max_lds, max_exs, max_sts,
    sp_banks * sp_bank_entries, acc_banks * acc_bank_entries,
    inputType.getWidth, accType.getWidth, dma_maxbytes, new MvinRs2(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new PreloadRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new PreloadRs(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new MvoutRs2(mvout_rows_bits, mvout_cols_bits, local_addr_t))

  val unrolled_cmd = Queue(loop_cmd)
  unrolled_cmd.ready := false.B
  reservation_station.io.alloc.valid := false.B
  reservation_station.io.alloc.bits := unrolled_cmd.bits

  // issue ports (Controller.scala:233-246)
  def issue(port: IssuePort, rs: ReservationStationIssue[GemminiCmd]): Unit = {
    port.valid := rs.valid
    rs.ready := port.ready
    port.funct := rs.cmd.cmd.inst.funct
    port.rs1 := rs.cmd.cmd.rs1
    port.rs2 := rs.cmd.cmd.rs2
    port.rob_id := rs.rob_id
  }
  issue(io.ld, reservation_station.io.issue.ld)
  issue(io.ex, reservation_station.io.issue.ex)
  issue(io.st, reservation_station.io.issue.st)

  // completions (Controller.scala:312-327: the Arbiter's output, here the bench's)
  reservation_station.io.completed.valid := io.completed_valid
  reservation_station.io.completed.bits := io.completed_bits

  // Controller.scala:358-401
  when (unrolled_cmd.valid) {
    val risc_funct = unrolled_cmd.bits.cmd.inst.funct
    val is_flush = risc_funct === FLUSH_CMD
    val is_counter_op = risc_funct === COUNTER_OP
    val is_clock_gate_en = risc_funct === CLKGATE_EN
    when (is_flush) {
      unrolled_cmd.ready := true.B
    }.elsewhen (is_counter_op) {
      unrolled_cmd.ready := true.B
    }.elsewhen (is_clock_gate_en) {
      unrolled_cmd.ready := true.B
    }.otherwise {
      reservation_station.io.alloc.valid := true.B
      when (reservation_station.io.alloc.fire) {
        unrolled_cmd.ready := true.B
      }
    }
  }

  io.busy := raw_cmd.valid || loop_conv_unroller_busy || loop_matmul_unroller_busy || reservation_station.io.busy ||
    unrolled_cmd.valid || loop_cmd.valid || conv_cmd.valid
  io.loop_matmul_busy := loop_matmul_unroller_busy
  io.matmul_ld_completed := reservation_station.io.matmul_ld_completed
  io.matmul_ex_completed := reservation_station.io.matmul_ex_completed
  io.matmul_st_completed := reservation_station.io.matmul_st_completed
}

object ElaborateCmd extends App {
  require(args.nonEmpty, "usage: ElaborateCmd <out_dir>")
  implicit val p: Parameters = (new freechips.rocketchip.system.DefaultConfig).alterPartial {
    case freechips.rocketchip.tile.TileKey => freechips.rocketchip.tile.RocketTileParams()
  }
  (new chisel3.stage.ChiselStage).emitVerilog(new CmdTop, Array("--target-dir", args(0)))
}
