import chisel3.DontCare.:=
import chisel3._
import chisel3.util._

import scala.Byte.MaxValue

class ChiselTop extends Module {
  val io = IO(new Bundle {
    val ui_in   = Input(UInt(8.W))   // Dedicated inputs
    val uo_out  = Output(UInt(8.W))  // Dedicated outputs
    val uio_in  = Input(UInt(8.W))   // Bidirectional inputs
    val uio_out = Output(UInt(8.W))  // Bidirectional outputs
    val uio_oe  = Output(UInt(8.W))  // Bidirectional output enable
  })


  // Instantiate DebounceInit for each input
  val debounce_up_command = Module(new DebounceInit)
  debounce_up_command.io.d := io.ui_in(0)

  val debounce_down_command = Module(new DebounceInit)
  debounce_down_command.io.d := io.ui_in(1)

  val debounce_down_end_command = Module(new DebounceInit)
  debounce_down_end_command.io.d := io.ui_in(2)

  val debounce_up_end_command = Module(new DebounceInit)
  debounce_up_end_command.io.d := io.ui_in(3)

  val debounce_down_sensor = Module(new DebounceInit)
  debounce_down_sensor.io.d := io.ui_in(4)

  val debounce_up_sensor = Module(new DebounceInit)
  debounce_up_sensor.io.d := io.ui_in(5)

  // Instantiate ManualAutoControl and connect debounced signals
  val manual_auto_control = Module(new ManualAutoControl)
  manual_auto_control.io.up_command := debounce_up_command.io.output
  manual_auto_control.io.down_command := debounce_down_command.io.output
  manual_auto_control.io.down_end_command := debounce_down_end_command.io.output
  manual_auto_control.io.up_end_command := debounce_up_end_command.io.output
  manual_auto_control.io.down_sensor := debounce_down_sensor.io.output
  manual_auto_control.io.up_sensor := debounce_up_sensor.io.output

  // Connect outputs
  io.uo_out(0) := manual_auto_control.io.motor_up
  io.uo_out(1) := manual_auto_control.io.motor_down

}

object ChiselTop extends App {
  emitVerilog(new ChiselTop(), Array("--target-dir", "src"))
}

class DebounceInit extends Module {
  val io = IO(new Bundle {
    val d: UInt = Input(UInt (1.W))
    val output: UInt = Output(UInt (1.W))
  })
  //Double D-flip flop
  val q = RegNext(io.d)
  val qq  = RegNext(q)
  // AND GATE
  val output = q & qq
  io.output := output
}

class ManualAutoControl extends Module {
  val io = IO(new Bundle {
    // Input
    val up_command: UInt  = Input(UInt (1.W))
    val down_command: UInt  = Input(UInt (1.W))
    val down_end_command: UInt = Input(UInt (1.W))
    val up_end_command: UInt = Input(UInt (1.W))
    val down_sensor: UInt = Input(UInt (1.W))
    val up_sensor: UInt = Input(UInt (1.W))

    //
    val motor_up: UInt = Output(UInt (1.W))
    val motor_down: UInt = Output(UInt (1.W))
  })

  val not_0: UInt = (~io.down_command).asUInt
  val not_1: UInt = (~io.up_command).asUInt

  val and_0: UInt = not_0 & io.down_command
  val and_1: UInt = io.down_command & not_1

  val nand_0: UInt = (~(io.down_end_command & io.up_end_command)).asUInt
  val not_2: UInt = (~io.down_end_command).asUInt
  val not_3: UInt = (~io.up_end_command).asUInt
  val and_2: UInt = not_2 & nand_0
  val and_3: UInt = nand_0 & not_3

  val and_4: UInt = not_0 & not_1

  //

  val and_0_0: UInt = and_0 & and_2
  val and_0_1: UInt = and_1 & and_3

  //

  val and_5: UInt = and_4 & io.down_sensor
  val and_6: UInt = and_4 & io.up_sensor

  //

  val or_0: UInt = and_0_0 | and_5
  val or_1: UInt = and_0_1 | and_6

  // Output

  io.motor_up := or_0
  io.motor_down := or_1

}