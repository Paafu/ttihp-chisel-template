import chisel3.DontCare.:=
import chisel3._
import chisel3.util._

// 1. Lopez I, Abdallah CT, Canudas de Wit C. Compensation Schemes for a Delta-Modulation-Based NCS. European Control Conference, ECC'07, Kos : Greece. 2007.
// 2. Tal E, Karaman S. Global Incremental Flight Control for Agile Maneuvering of a Tailsitter Flying Wing. Journal of Guidance, Control, and Dynamics. 2022;45(12).
// 3. Oh S, Tavella D, Roberts L. Theoretical Studies on Flapped Delta Wings. National Aeronautics and Space Administration, Ames Research Center, JIAA TR - 85, Stanford University, Department of Aeronautics and Astronautics, Stanford, CA 94305. 1988.
// 4. Dumitru C, Pantelimon E, Guzu A, Nicolae G. Dragonfang: An Open-Source Embedded Flight Controller with IMU-Based Stabilization for Quadcopter Applications. Computational Materials Continuum. 2026;87(1):13.
// 5. http://www.digitalsignallabs.com/fp.pdf
// Assuming we are using BNO085 IMU and DS1050 PWM driver.

class ChiselTop extends Module {
  val io = IO(new Bundle {
    val ui_in   = Input(UInt(8.W))   // Dedicated inputs
    val uo_out  = Output(UInt(8.W))  // Dedicated outputs
    val uio_in  = Input(UInt(8.W))   // Bidirectional inputs
    val uio_out = Output(UInt(8.W))  // Bidirectional outputs
    val uio_oe  = Output(UInt(8.W))  // Bidirectional output enable

    // The following values need to be able to fit within the limited IO.
    // I will leave them here to make it easier to understand the protocol and the thought process of how
    // I ended up here.
    /*
    // IMU quaternion inputs (from external sensor)
    val bno_quat_w = Input(SInt(16.W))
    val bno_quat_x = Input(SInt(16.W))
    val bno_quat_y = Input(SInt(16.W))
    val bno_quat_z = Input(SInt(16.W))

    // PID desired setpoints and register-loader interface
    val pitch_desired = Input(SInt(8.W))
    val roll_desired  = Input(SInt(8.W))
    val yaw_desired   = Input(SInt(8.W))
    val control = Input(UInt(8.W))
    val data_in = Input(UInt(8.W))

    // Outputs: direct 8-bit elevon/rudder control and 5-bit PWM outputs
    val left_elevon = Output(UInt(8.W))
    val right_elevon = Output(UInt(8.W))
    val rudder = Output(UInt(8.W))
    val pwm_left = Output(UInt(5.W))
    val pwm_right = Output(UInt(5.W))
    val pwm_rudder = Output(UInt(5.W))
     */

  })

  // Instantiate modules
  val imu_adapter = Module(new BNO085IMUAdapter)
  val pid = Module(new PIDControllerTop)
  val ds_left = Module(new DS1050PWMAdapter)
  val ds_right = Module(new DS1050PWMAdapter)
  val ds_rudder = Module(new DS1050PWMAdapter)
  // Instantiate the finite-state-machine based data collector to handle
  // transactions over the limited-width IO bus.
  val dataCollector = Module(new DataCollector)

  // Wire the dataCollector to the limited external IO
  dataCollector.io.ui_in := io.ui_in
  dataCollector.io.uio_in := io.uio_in
  // Drive the uio outputs from the data collector (keep protocol behavior)
  // Note: we do not directly assign `io.uo_out` here because the single
  // dedicated 8-bit output `uo_out` is repurposed below to carry packed
  // PWM values (time-multiplexed). The DataCollector will be given
  // precedence during its word_valid cycle.
  io.uio_out := dataCollector.io.uio_out
  io.uio_oe := dataCollector.io.uio_oe

  // Internal IMU/quaternion registers (reconstructed from the limited IO by
  // a higher-level protocol). They are kept internal so we do not expand the
  // public IO. For now initialize to zero; later these should be written by
  // logic that decodes incoming words from `dataCollector`.
  val bno_quat_w_reg = RegInit(0.S(16.W))
  val bno_quat_x_reg = RegInit(0.S(16.W))
  val bno_quat_y_reg = RegInit(0.S(16.W))
  val bno_quat_z_reg = RegInit(0.S(16.W))

  // Desired setpoints and loader interface registers (internal)
  val pitch_desired_reg = RegInit(0.S(8.W))
  val roll_desired_reg = RegInit(0.S(8.W))
  val yaw_desired_reg = RegInit(0.S(8.W))
  val control_reg = RegInit(0.U(8.W))
  val data_in_reg = RegInit(0.U(8.W))

  // Simple two-word protocol decoder for writing internal registers over
  // the limited 8-bit interface. Protocol:
  // 1) Send a selector word: [0xFF, target_id] (16-bit word where MSB byte == 0xFF)
  // 2) Send a data word:   [msb, lsb] (16-bit data to store into the target)
  // Supported target_id values:
  //   0x00 - 0x03 : quaternion w/x/y/z (16-bit signed)
  //   0x10 - 0x12 : pitch/roll/yaw desired (8-bit signed, stored in low byte)
  //   0x20 : control (8-bit)
  //   0x21 : data_in (8-bit)
  // Note: Holy shit this was a mess to think about.

  val pending_target = RegInit(0.U(8.W))
  val expecting_data = RegInit(false.B)
  when(dataCollector.io.word_valid) {
    val w = dataCollector.io.word_out
    when(!expecting_data) {
      when(w(15,8) === 0xFF.U) {
        pending_target := w(7,0)
        expecting_data := true.B
      }
    }.otherwise {
      // write data into the selected register
      switch(pending_target) {
        is(0.U) { bno_quat_w_reg := w.asSInt }
        is(1.U) { bno_quat_x_reg := w.asSInt }
        is(2.U) { bno_quat_y_reg := w.asSInt }
        is(3.U) { bno_quat_z_reg := w.asSInt }
        is(0x10.U) { pitch_desired_reg := w(7,0).asSInt }
        is(0x11.U) { roll_desired_reg := w(7,0).asSInt }
        is(0x12.U) { yaw_desired_reg := w(7,0).asSInt }
        is(0x20.U) { control_reg := w(7,0) }
        is(0x21.U) { data_in_reg := w(7,0) }
      }
      expecting_data := false.B
    }
  }

  // Connect IMU adapter to the internal registers
  imu_adapter.io.bno_quat_w := bno_quat_w_reg
  imu_adapter.io.bno_quat_x := bno_quat_x_reg
  imu_adapter.io.bno_quat_y := bno_quat_y_reg
  imu_adapter.io.bno_quat_z := bno_quat_z_reg

  // Connect PID inputs (sensor and desired setpoints)
  // BNO adapter produces 16-bit SInt; PID expects 8-bit SInt values in 0.8 format
  // We downscale by taking the top 8 bits of the 16-bit angle representation.
  pid.io.pitch_sensor := imu_adapter.io.pitch(15, 8).asSInt
  pid.io.roll_sensor := imu_adapter.io.roll(15, 8).asSInt
  pid.io.yaw_sensor := imu_adapter.io.yaw(15, 8).asSInt


  pid.io.pitch_desired := pitch_desired_reg
  pid.io.roll_desired := roll_desired_reg
  pid.io.yaw_desired := yaw_desired_reg

  // Loader interface (kept internal until a decoder writes to control/data_in)
  pid.io.control := control_reg
  pid.io.data_in := data_in_reg

  // Route PID outputs through internal wires (do not expand top-level IO)
  val left_elevon_wire = Wire(UInt(8.W))
  val right_elevon_wire = Wire(UInt(8.W))
  val rudder_wire = Wire(UInt(8.W))
  left_elevon_wire := pid.io.left_elevon
  right_elevon_wire := pid.io.right_elevon
  rudder_wire := pid.io.rudder

  // DS1050 adapters take 16-bit pid_pwm; expand 8-bit to 16-bit by zero-extending
  ds_left.io.pid_pwm := Cat(0.U(8.W), left_elevon_wire)
  ds_right.io.pid_pwm := Cat(0.U(8.W), right_elevon_wire)
  ds_rudder.io.pid_pwm := Cat(0.U(8.W), rudder_wire)

  // Internal PWM wires (not exposed on top-level IO)
  val pwm_left_wire = ds_left.io.ds_pwm
  val pwm_right_wire = ds_right.io.ds_pwm
  val pwm_rudder_wire = ds_rudder.io.ds_pwm

  // Pack the three 5-bit PWM channels into the single 8-bit `uo_out`.
  //
  // Encoding (8 bits -> [7:5] tag, [4:0] payload):
  //  tag 0b001 -> left elevon PWM (payload = pwm_left_wire[4:0])
  //  tag 0b010 -> right elevon PWM (payload = pwm_right_wire[4:0])
  //  tag 0b011 -> rudder PWM (payload = pwm_rudder_wire[4:0])
  //  tag 0b000 -> reserved/idle (payload ignored)
  //
  // Time-multiplex scheme:
  // - When `dataCollector.io.word_valid` is asserted for one cycle it has
  //   priority: the original `DataCollector` 8-bit `uo_out` value is
  //   presented on `io.uo_out` for that cycle to preserve existing protocol
  //   behavior.
  // - Otherwise, we cycle through the three PWM channels using a 2-bit
  //   counter and present a tagged 8-bit word carrying one PWM per cycle.
  //
  // This keeps the external pin usage to a single dedicated byte while
  // still allowing observation of the three PWM channels over time.

  // 2-bit counter to choose which PWM to present when DataCollector is idle
  val pwm_cycle = RegInit(0.U(2.W))
  pwm_cycle := pwm_cycle + 1.U

  // Construct tagged payloads
  val tag_left = Cat("b001".U(3.W), pwm_left_wire)
  val tag_right = Cat("b010".U(3.W), pwm_right_wire)
  val tag_rudder = Cat("b011".U(3.W), pwm_rudder_wire)

  // Default output is the DataCollector's uo_out (it will take effect when valid)
  val packed_uo = WireDefault(dataCollector.io.uo_out)

  // If DataCollector is not asserting a valid word this cycle, present PWM
  when(!dataCollector.io.word_valid) {
    packed_uo := MuxLookup(pwm_cycle, 0.U(8.W), Array(
      0.U -> tag_left,
      1.U -> tag_right,
      2.U -> tag_rudder
    ))
  }

  // Drive the single dedicated uo_out with the packed value
  io.uo_out := packed_uo

  // The following rudder controls output decoding more or less should look like this.
  // Input-Tag  | Output-Left | Output-Right | Output-Rudder
  // 001        |  1          |   0          |    0
  // 010        |  0          |   1          |    0
  // 011        |  0          |   0          |    1
  //
  // Something something
  // val tag = io.packed_word(7,5)  // 3-bit tag
  // val enable_left = ~tag(2) & ~tag(1) & tag(0)
  // val enable_right = ~tag(2) & tag(1) & ~tag(0)
  // val enable_rudder = ~tag(2) & tag(1) & tag(0)
  //
  // Something something use these enables to gate your PWM outputs something something
  // val pwm_left_output = Mux(enable_left, pwm_value, 0.U(5.W))
  // val pwm_right_output = Mux(enable_right, pwm_value, 0.U(5.W))
  // val pwm_rudder_output = Mux(enable_rudder, pwm_value, 0.U(5.W))
}

object ChiselTop extends App {
  emitVerilog(new ChiselTop(), Array("--target-dir", "src"))
}

// Simple debounce (keeps previous behavior but uses correct Bundle syntax)
class DebounceInit extends Module {
  val io = IO(new Bundle {
    val d = Input(UInt(1.W))
    val output = Output(UInt(1.W))
  })
  val q = RegNext(io.d)
  val qq = RegNext(q)
  io.output := q & qq
}

// ManualAutoControl kept but corrected bundle syntax
class ManualAutoControl extends Module {
  val io = IO(new Bundle {
    val up_command = Input(UInt(1.W))
    val down_command = Input(UInt(1.W))
    val down_end_command = Input(UInt(1.W))
    val up_end_command = Input(UInt(1.W))
    val down_sensor = Input(UInt(1.W))
    val up_sensor = Input(UInt(1.W))

    val motor_up = Output(UInt(1.W))
    val motor_down = Output(UInt(1.W))
  })

  val not_0 = (~io.down_command).asUInt
  val not_1 = (~io.up_command).asUInt
  val and_0 = not_0 & io.down_command
  val and_1 = io.down_command & not_1
  val nand_0 = (~(io.down_end_command & io.up_end_command)).asUInt
  val not_2 = (~io.down_end_command).asUInt
  val not_3 = (~io.up_end_command).asUInt
  val and_2 = not_2 & nand_0
  val and_3 = nand_0 & not_3
  val and_4 = not_0 & not_1
  val and_0_0 = and_0 & and_2
  val and_0_1 = and_1 & and_3
  val and_5 = and_4 & io.down_sensor
  val and_6 = and_4 & io.up_sensor
  val or_0 = and_0_0 | and_5
  val or_1 = and_0_1 | and_6

  io.motor_up := or_0
  io.motor_down := or_1
}

class QuaternionToEuler extends Module {
  // Placeholder simplified converter: for now emit zero angles. Replace with
  // a proper fixed-point quaternion->euler implementation when needed.
  val io = IO(new Bundle {
    val qw = Input(SInt(32.W))
    val qx = Input(SInt(32.W))
    val qy = Input(SInt(32.W))
    val qz = Input(SInt(32.W))
    val roll = Output(SInt(32.W))
    val pitch = Output(SInt(32.W))
    val yaw = Output(SInt(32.W))
  })

  io.roll := 0.S
  io.pitch := 0.S
  io.yaw := 0.S
}

class BNO085IMUAdapter extends Module {
  val io = IO(new Bundle {
    // Inputs from BNO085 (16-bit signed in 1.14 format)
    val bno_quat_w = Input(SInt(16.W))
    val bno_quat_x = Input(SInt(16.W))
    val bno_quat_y = Input(SInt(16.W))
    val bno_quat_z = Input(SInt(16.W))

    // Outputs to PID controller (16-bit signed in 1.15 format)
    val pitch = Output(SInt(16.W))
    val roll = Output(SInt(16.W))
    val yaw = Output(SInt(16.W))
  })

  // Convert quaternions to Euler angles
  // Use QuaternionToEuler module for fixed-point conversion
  val quat_to_euler = Module(new QuaternionToEuler)
  quat_to_euler.io.qw := io.bno_quat_w
  quat_to_euler.io.qx := io.bno_quat_x
  // Sign-extend 16-bit quaternion inputs to 32-bit Q15.16 format expected by the converter.
  quat_to_euler.io.qy := io.bno_quat_y
  quat_to_euler.io.qz := io.bno_quat_z

  // The QuaternionToEuler module outputs roll/pitch/yaw in Q15.16 (SInt(32.W)).
  // Truncate/resize to 16 bits by taking the top 16 bits (31 downto 16). The
  // top 16 bits include the integer portion and the top fractional bits.
  io.roll := quat_to_euler.io.roll(31, 16).asSInt
  io.pitch := quat_to_euler.io.pitch(31, 16).asSInt
  io.yaw := quat_to_euler.io.yaw(31, 16).asSInt
}

class DS1050PWMAdapter extends Module {
  val io = IO(new Bundle {
    val pid_pwm = Input(UInt(16.W)) // 0 to 65535
    val ds_pwm = Output(UInt(5.W))  // 0 to 31
  })

  // Maximum truncation error = 2,047 counts (approx. 3.12% of full scale).
  // Half‑step (typical error) about 1,024 counts (approx. 1.56% of full scale).

  // Convert from 16-bit to 5-bit: right shift by 11 bits
  // Todo add more PWM adapters?
  io.ds_pwm := io.pid_pwm >> 11
}


class PIDControllerTop extends Module {
  val io = IO(new Bundle {
    // Inputs (all 8 bits)
    val pitch_sensor = Input(SInt(8.W)) // signed 0.8 format
    val roll_sensor = Input(SInt(8.W))  // signed 0.8 format
    val yaw_sensor = Input(SInt(8.W))   // signed 0.8 format
    val pitch_desired = Input(SInt(8.W)) // signed 0.8 format
    val roll_desired = Input(SInt(8.W))  // signed 0.8 format
    val yaw_desired = Input(SInt(8.W))   // signed 0.8 format
    val control = Input(UInt(8.W))      // control signal for loading registers
    val data_in = Input(UInt(8.W))      // data to load into registers

    // Outputs (all 8 bits)
    val left_elevon = Output(UInt(8.W)) // PWM output for left elevon
    val right_elevon = Output(UInt(8.W)) // PWM output for right elevon
    val rudder = Output(UInt(8.W))      // PWM output for rudder
    // Remaining outputs unused
  })

  // Internal registers for PID gains (16 bits each, unsigned 8.8 format)
  val Kp_pitch = RegInit(128.U(16.W)) // 0.5 * 256 = 128
  val Ki_pitch = RegInit(26.U(16.W)) // 0.1 * 256 ≈ 26
  val Kd_pitch = RegInit(3.U(16.W))  // 0.01 * 256 ≈ 3
  val Kp_roll = RegInit(256.U(16.W)) // 1.0 * 256 = 256
  val Kd_roll = RegInit(51.U(16.W))  // 0.2 * 256 = 51.2 → 51
  val Kp_yaw = RegInit(77.U(16.W))   // 0.3 * 256 = 76.8 → 77

  // State variables (16 bits each, signed 8.8 format)
  val ez_prev = RegInit(0.S(16.W))
  val ey_prev = RegInit(0.S(16.W))
  val integral_ez = RegInit(0.S(16.W))

  // Convert 8-bit sensor inputs to 16-bit signed 8.8 format
  val current_pitch = io.pitch_sensor.asSInt
  val current_roll = io.roll_sensor.asSInt
  val current_yaw_rate = io.yaw_sensor.asSInt

  // Convert desired values to 8.8 format
  val desired_pitch = io.pitch_desired.asSInt
  val desired_roll = io.roll_desired.asSInt
  val desired_yaw = io.yaw_desired.asSInt

  // Compute errors (in 8.8 format)
  val ez = desired_pitch - current_pitch
  val ey = desired_roll - current_roll
  val er = desired_yaw - current_yaw_rate

  // Update integral term for pitch (dt is assumed to be 1.0 in 8.8 format, i.e., 256)
  val dt = 256.S(16.W)
  // Parenthesize expressions so shifts apply to the multiplication before addition
  // Cast the shifted result to SInt so the addition matches types
  integral_ez := integral_ez + (((ez * dt) >> 8).asSInt)

  // Compute control outputs
  val delta_pitch = (((Kp_pitch.asSInt * ez) >> 8).asSInt) +
    (((Ki_pitch.asSInt * integral_ez) >> 8).asSInt) +
    (((Kd_pitch.asSInt * (ez - ez_prev)) >> 8).asSInt)
  val delta_roll = (((Kp_roll.asSInt * ey) >> 8).asSInt) +
    (((Kd_roll.asSInt * (ey - ey_prev)) >> 8).asSInt)
  val delta_yaw = (((Kp_yaw.asSInt * er) >> 8).asSInt)

  // Update previous errors
  ez_prev := ez
  ey_prev := ey

  // Combine control outputs for elevons
  val elevon_left = delta_pitch + delta_roll
  val elevon_right = delta_pitch - delta_roll

  // Map control outputs to 8-bit PWM outputs. Parenthesize arithmetic before bit-slicing.
  val left_elevon_pwm = ((elevon_left.asUInt + 256.U)(7, 0))
  val right_elevon_pwm = ((elevon_right.asUInt + 256.U)(7, 0))
  val rudder_pwm = ((delta_yaw.asUInt + 256.U)(7, 0))

  // Output PWM signals
  io.left_elevon := left_elevon_pwm
  io.right_elevon := right_elevon_pwm
  io.rudder := rudder_pwm

  // Logic to load PID gain registers from data_in
  /*
   Loading new values into the 16-bit PID gain registers (stored in 8.8 fixed-point)
   Protocol summary (how to write a new 16-bit value using an 8-bit data bus):

   - The `control` input (8 bits) encodes both which register to target and whether
     the incoming byte on `data_in` should become the high byte or the low byte of
     that 16-bit register.

     control[2:0]  => register select (3-bit index)
       0 -> Kp_pitch
       1 -> Ki_pitch
       2 -> Kd_pitch
       3 -> Kp_roll
       4 -> Kd_roll
       5 -> Kp_yaw

     control[3] => high/low selector
       - If control[3] == 0: we are loading the HIGH byte (bits 15..8) of the
         register. The code sets the new register value with the pattern:
           reg := Cat(io.data_in, reg(7,0))
         That keeps the existing low byte (reg(7,0)) and replaces the high byte
         with `data_in`.

       - If control[3] == 1: we are loading the LOW byte (bits 7..0) of the
         register. The code sets the new register value with the pattern:
           reg := Cat(reg(15,8), io.data_in)
         That keeps the current high byte and replaces the low byte with
         `data_in`.

   - Important details and best practices:
     * The registers use 8.8 fixed-point format (upper 8 bits = integer, lower 8
       bits = fractional). To write a complete new value you must perform two
       writes: one to the high byte and one to the low byte. The order of these
       writes is flexible (high then low, or low then high) but during the
       intermediate cycle the register will contain a partially-updated value.
       If the rest of the system can sample these gains while a partial update
       is in progress, you may want to write high then low (or otherwise
       coordinate) so the register does not briefly contain a large unintended
       value.

     * The actual register update occurs synchronously as a next-state update
       to the RegInit registers (i.e. when the clock edge occurs with the
       `control` and `data_in` present, the corresponding `Kx := Cat(...)`
       assignment will take effect). Therefore ensure `control` and `data_in`
       are stable for at least one clock cycle when asserting the write.

     * `Cat(a, b)` concatenates bits with `a` as the more-significant portion
       and `b` as the less-significant portion. For example
         Cat(io.data_in, Kp_pitch(7,0))
       produces a 16-bit value where io.data_in becomes bits [15:8] and
       Kp_pitch(7,0) becomes bits [7:0].

   Example: load a new Kp_pitch representing 0.6 in 8.8 fixed-point
   - Desired real value = 0.6
   - Convert to 8.8 fixed-point: 0.6 * 256 = 153.6 → round to 154 decimal
     (0x009A in hex). The 16-bit representation is 0x009A, where
       high byte = 0x00
       low  byte = 0x9A

   - To write this value using the protocol above (two 8-bit writes):
     1) Load the HIGH byte (0x00): set
          control = 0b0000 (bits 3..0 = 0000)  // reg_select = 0 (Kp_pitch), bit3=0 => high
          data_in  = 0x00
        Clock the interface so the high byte is written.

     2) Load the LOW byte (0x9A): set
          control = 0b1000 (bits 3..0 = 1000)  // reg_select = 0 (Kp_pitch), bit3=1 => low
          data_in  = 0x9A
        Clock the interface so the low byte is written.

     After these two writes the register Kp_pitch will contain 0x009A which
     corresponds to the fixed-point value 154/256 = 0.6015625 (close to 0.6).

   - Note: the code maps control[2:0] values 0..5 to the registers shown above.
     Any other control[2:0] values have no effect in the current switch.
  */

  // TL;DR
  // Pick which register with control[2:0], set control[3]=0 to write the high byte or =1 to write the low byte,
  // put that 8-bit byte on data_in, then clock it; do both high+low writes to update the full 16-bit (8.8) gain.
  // Example (write 0.6 → 0x009A)
  // High byte: control=0x00, data_in=0x00 (clock) — Low byte: control=0x08, data_in=0x9A (clock).


  val reg_select = io.control(2, 0)
  val load_high = io.control(3) === 0.U

  when(load_high) {
    switch(reg_select) {
      is(0.U) { Kp_pitch := Cat(io.data_in, Kp_pitch(7, 0)) }
      is(1.U) { Ki_pitch := Cat(io.data_in, Ki_pitch(7, 0)) }
      is(2.U) { Kd_pitch := Cat(io.data_in, Kd_pitch(7, 0)) }
      is(3.U) { Kp_roll := Cat(io.data_in, Kp_roll(7, 0)) }
      is(4.U) { Kd_roll := Cat(io.data_in, Kd_roll(7, 0)) }
      is(5.U) { Kp_yaw := Cat(io.data_in, Kp_yaw(7, 0)) }
    }
  }.otherwise {
    switch(reg_select) {
      is(0.U) { Kp_pitch := Cat(Kp_pitch(15, 8), io.data_in) }
      is(1.U) { Ki_pitch := Cat(Ki_pitch(15, 8), io.data_in) }
      is(2.U) { Kd_pitch := Cat(Kd_pitch(15, 8), io.data_in) }
      is(3.U) { Kp_roll := Cat(Kp_roll(15, 8), io.data_in) }
      is(4.U) { Kd_roll := Cat(Kd_roll(15, 8), io.data_in) }
      is(5.U) { Kp_yaw := Cat(Kp_yaw(15, 8), io.data_in) }
    }
  }
}

// DataCollector: FSM that collects 16-bit words over an 8-bit wire interface
class DataCollector extends Module {
  val io = IO(new Bundle {
    val ui_in = Input(UInt(8.W)) // Dedicated inputs
    val uo_out = Output(UInt(8.W)) // Dedicated outputs
    val uio_in = Input(UInt(8.W)) // Bidirectional inputs
    val uio_out = Output(UInt(8.W)) // Bidirectional outputs
    val uio_oe = Output(UInt(8.W)) // Bidirectional output enable
    // When DataCollector has assembled a 16-bit word it presents it here and
    // asserts word_valid for one cycle.
    val word_out = Output(UInt(16.W))
    val word_valid = Output(Bool())
  })

  // Define the states for the FSM
  val s_idle :: s_collect :: s_process :: s_output :: Nil = Enum(4)
  val state = RegInit(s_idle)

  // Registers to store collected data (two bytes -> 16 bits) and a counter
  val dataReg = RegInit(0.U(16.W))
  val byteCount = RegInit(0.U(1.W)) // 0 or 1
  val word_out_reg = RegInit(0.U(16.W))
  val word_valid_reg = RegInit(false.B)

  // Default outputs
  io.uo_out := 0.U
  io.uio_out := 0.U
  io.uio_oe := 0.U
  io.word_out := word_out_reg
  io.word_valid := word_valid_reg

  switch(state) {
    is(s_idle) {
      // Wait for non-zero input to start collection (user may change condition)
      when(io.ui_in =/= 0.U) {
        state := s_collect
        byteCount := 0.U
        // Capture first incoming byte in high position
        dataReg := Cat(io.ui_in, 0.U(8.W))
        word_valid_reg := false.B
      }
    }
    is(s_collect) {
      when(byteCount === 0.U) {
        // store low byte on next cycle
        dataReg := Cat(dataReg(15,8), io.ui_in)
        byteCount := 1.U
        state := s_process
      }
    }
    is(s_process) {
      // For now just pass-through: place low/high bytes into separate outputs
      io.uo_out := dataReg(7,0)
      // publish assembled 16-bit word
      word_out_reg := dataReg
      word_valid_reg := true.B
      state := s_output
    }
    is(s_output) {
      io.uio_out := dataReg(15,8)
      io.uio_oe := Fill(8, 1.U(1.W)) // enable all uio bits
      // clear valid on next cycle
      state := s_idle
      word_valid_reg := false.B
    }
  }
}