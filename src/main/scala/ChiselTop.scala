import chisel3._
import chisel3.util._

class ChiselTop extends Module {
  val io = IO(new Bundle {
    val ui_in   = Input(UInt(8.W))   // Dedicated inputs
    val uo_out  = Output(UInt(8.W))  // Dedicated outputs
    val uio_in  = Input(UInt(8.W))   // Bidirectional inputs
    val uio_out = Output(UInt(8.W))  // Bidirectional outputs
    val uio_oe  = Output(UInt(8.W))  // Bidirectional output enable
  })

  // ============================================================
  // Input mapping
  // ui_in[3:0] = pilot pitch command  (-8 .. +7)
  // ui_in[7:4] = pilot roll command   (-8 .. +7)
  //
  // uio_in[3:0] = pitch correction from external IMU controller (-8 .. +7)
  // uio_in[7:4] = roll correction from external IMU controller  (-8 .. +7)
  //
  // Outputs:
  // uo_out[0] = left elevon servo PWM
  // uo_out[1] = right elevon servo PWM
  // uo_out[7:2] = debug/status bits
  // ============================================================

  // Treat 4-bit fields as signed two's complement
  val pilotPitch = io.ui_in(3, 0).asSInt
  val pilotRoll  = io.ui_in(7, 4).asSInt
  val corrPitch  = io.uio_in(3, 0).asSInt
  val corrRoll   = io.uio_in(7, 4).asSInt

  // Gain on correction path: divide by 2 to keep things tame
  val corrPitchScaled = (corrPitch.asSInt >> 1).asSInt
  val corrRollScaled  = (corrRoll.asSInt >> 1).asSInt

  val totalPitch = (pilotPitch +& corrPitchScaled).asSInt
  val totalRoll  = (pilotRoll  +& corrRollScaled).asSInt

  // Delta wing elevon mixing:
  // left  = pitch + roll
  // right = pitch - roll
  val leftMixRaw  = (totalPitch +& totalRoll).asSInt
  val rightMixRaw = (totalPitch -& totalRoll).asSInt

  // Saturate to signed 5-bit range we want to use: -8 .. +7
  def sat4(x: SInt): SInt = {
    val y = Wire(SInt(4.W))
    when(x > 7.S) {
      y := 7.S
    }.elsewhen(x < (-8).S) {
      y := (-8).S
    }.otherwise {
      y := x(3, 0).asSInt
    }
    y
  }

  val leftMix  = sat4(leftMixRaw)
  val rightMix = sat4(rightMixRaw)

  // ============================================================
  // Servo PWM generator
  // 50 MHz clock
  // 20 ms period  = 1,000,000 cycles
  // 1.0 ms pulse  =   50,000 cycles
  // 1.5 ms pulse  =   75,000 cycles
  // 2.0 ms pulse  =  100,000 cycles
  //
  // command range: -8 .. +7
  // map to pulse width around center
  // step = 3,000 cycles (~60 us)
  // so:
  // -8 -> 51,000
  //  0 -> 75,000
  // +7 -> 96,000
  // ============================================================

  val periodCount = 1000000.U(20.W)
  val pwmCounter  = RegInit(0.U(20.W))

  when(pwmCounter === (periodCount - 1.U)) {
    pwmCounter := 0.U
  }.otherwise {
    pwmCounter := pwmCounter + 1.U
  }

  def servoPulseWidth(cmd: SInt): UInt = {
    val center = 75000.S(18.W)
    val step   = 3000.S(18.W)
    val width  = center + (cmd * step)
    width.asUInt
  }

  val leftPulse  = servoPulseWidth(leftMix)
  val rightPulse = servoPulseWidth(rightMix)

  val leftPwm  = pwmCounter < leftPulse
  val rightPwm = pwmCounter < rightPulse

  // ============================================================
  // Outputs
  // ============================================================

  io.uo_out := Cat(
    0.U(1.W),                // bit 7 reserved
    leftMix.asUInt()(3),     // bit 6 debug
    leftMix.asUInt()(2),     // bit 5 debug
    rightMix.asUInt()(3),    // bit 4 debug
    rightMix.asUInt()(2),    // bit 3 debug
    (pwmCounter === 0.U),    // bit 2 frame sync pulse
    rightPwm,                // bit 1
    leftPwm                  // bit 0
  )

  // We are only using uio as inputs in this version
  io.uio_out := 0.U
  io.uio_oe  := 0.U
}

object ChiselTop extends App {
  emitVerilog(new ChiselTop(), Array("--target-dir", "src"))
}