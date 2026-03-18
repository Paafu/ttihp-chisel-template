module PIDControllerTop(
  input        clock,
  input        reset,
  input  [7:0] io_pitch_desired,
  input  [7:0] io_roll_desired,
  input  [7:0] io_yaw_desired,
  input  [7:0] io_control,
  input  [7:0] io_data_in,
  output [7:0] io_left_elevon,
  output [7:0] io_right_elevon,
  output [7:0] io_rudder
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
`endif // RANDOMIZE_REG_INIT
  reg [15:0] Kp_pitch; // @[ChiselTop.scala 356:25]
  reg [15:0] Ki_pitch; // @[ChiselTop.scala 357:25]
  reg [15:0] Kd_pitch; // @[ChiselTop.scala 358:25]
  reg [15:0] Kp_roll; // @[ChiselTop.scala 359:24]
  reg [15:0] Kd_roll; // @[ChiselTop.scala 360:24]
  reg [15:0] Kp_yaw; // @[ChiselTop.scala 361:23]
  reg [15:0] ez_prev; // @[ChiselTop.scala 364:24]
  reg [15:0] ey_prev; // @[ChiselTop.scala 365:24]
  reg [15:0] integral_ez; // @[ChiselTop.scala 366:28]
  wire [7:0] ez = $signed(io_pitch_desired) - 8'sh0; // @[ChiselTop.scala 379:26]
  wire [7:0] ey = $signed(io_roll_desired) - 8'sh0; // @[ChiselTop.scala 380:25]
  wire [7:0] er = $signed(io_yaw_desired) - 8'sh0; // @[ChiselTop.scala 381:24]
  wire [23:0] _integral_ez_T = $signed(ez) * 16'sh100; // @[ChiselTop.scala 387:38]
  wire [15:0] _integral_ez_T_1 = _integral_ez_T[23:8]; // @[ChiselTop.scala 387:44]
  wire [15:0] _integral_ez_T_4 = $signed(integral_ez) + $signed(_integral_ez_T_1); // @[ChiselTop.scala 387:30]
  wire [23:0] _delta_pitch_T_1 = $signed(Kp_pitch) * $signed(ez); // @[ChiselTop.scala 390:40]
  wire [15:0] _delta_pitch_T_2 = _delta_pitch_T_1[23:8]; // @[ChiselTop.scala 390:46]
  wire [31:0] _delta_pitch_T_4 = $signed(Ki_pitch) * $signed(integral_ez); // @[ChiselTop.scala 391:24]
  wire [23:0] _delta_pitch_T_5 = _delta_pitch_T_4[31:8]; // @[ChiselTop.scala 391:39]
  wire [23:0] _GEN_48 = {{8{_delta_pitch_T_2[15]}},_delta_pitch_T_2}; // @[ChiselTop.scala 390:60]
  wire [23:0] _delta_pitch_T_8 = $signed(_GEN_48) + $signed(_delta_pitch_T_5); // @[ChiselTop.scala 390:60]
  wire [15:0] _GEN_49 = {{8{ez[7]}},ez}; // @[ChiselTop.scala 392:30]
  wire [15:0] _delta_pitch_T_12 = $signed(_GEN_49) - $signed(ez_prev); // @[ChiselTop.scala 392:30]
  wire [31:0] _delta_pitch_T_13 = $signed(Kd_pitch) * $signed(_delta_pitch_T_12); // @[ChiselTop.scala 392:24]
  wire [23:0] _delta_pitch_T_14 = _delta_pitch_T_13[31:8]; // @[ChiselTop.scala 392:42]
  wire [23:0] delta_pitch = $signed(_delta_pitch_T_8) + $signed(_delta_pitch_T_14); // @[ChiselTop.scala 391:53]
  wire [23:0] _delta_roll_T_1 = $signed(Kp_roll) * $signed(ey); // @[ChiselTop.scala 393:38]
  wire [15:0] _delta_roll_T_2 = _delta_roll_T_1[23:8]; // @[ChiselTop.scala 393:44]
  wire [15:0] _GEN_50 = {{8{ey[7]}},ey}; // @[ChiselTop.scala 394:29]
  wire [15:0] _delta_roll_T_6 = $signed(_GEN_50) - $signed(ey_prev); // @[ChiselTop.scala 394:29]
  wire [31:0] _delta_roll_T_7 = $signed(Kd_roll) * $signed(_delta_roll_T_6); // @[ChiselTop.scala 394:23]
  wire [23:0] _delta_roll_T_8 = _delta_roll_T_7[31:8]; // @[ChiselTop.scala 394:41]
  wire [23:0] _GEN_51 = {{8{_delta_roll_T_2[15]}},_delta_roll_T_2}; // @[ChiselTop.scala 393:58]
  wire [23:0] delta_roll = $signed(_GEN_51) + $signed(_delta_roll_T_8); // @[ChiselTop.scala 393:58]
  wire [23:0] _delta_yaw_T_1 = $signed(Kp_yaw) * $signed(er); // @[ChiselTop.scala 395:36]
  wire [23:0] _left_elevon_pwm_T = $signed(delta_pitch) + $signed(delta_roll); // @[ChiselTop.scala 406:39]
  wire [23:0] _left_elevon_pwm_T_2 = _left_elevon_pwm_T + 24'h100; // @[ChiselTop.scala 406:46]
  wire [23:0] _right_elevon_pwm_T = $signed(delta_pitch) - $signed(delta_roll); // @[ChiselTop.scala 407:41]
  wire [23:0] _right_elevon_pwm_T_2 = _right_elevon_pwm_T + 24'h100; // @[ChiselTop.scala 407:48]
  wire [15:0] _rudder_pwm_T = _delta_yaw_T_1[23:8]; // @[ChiselTop.scala 408:32]
  wire [15:0] _rudder_pwm_T_2 = _rudder_pwm_T + 16'h100; // @[ChiselTop.scala 408:39]
  wire [2:0] reg_select = io_control[2:0]; // @[ChiselTop.scala 500:30]
  wire  load_high = ~io_control[3]; // @[ChiselTop.scala 501:33]
  wire  _T = 3'h0 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Kp_pitch_T_1 = {io_data_in,Kp_pitch[7:0]}; // @[Cat.scala 33:92]
  wire  _T_1 = 3'h1 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Ki_pitch_T_1 = {io_data_in,Ki_pitch[7:0]}; // @[Cat.scala 33:92]
  wire  _T_2 = 3'h2 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Kd_pitch_T_1 = {io_data_in,Kd_pitch[7:0]}; // @[Cat.scala 33:92]
  wire  _T_3 = 3'h3 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Kp_roll_T_1 = {io_data_in,Kp_roll[7:0]}; // @[Cat.scala 33:92]
  wire  _T_4 = 3'h4 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Kd_roll_T_1 = {io_data_in,Kd_roll[7:0]}; // @[Cat.scala 33:92]
  wire  _T_5 = 3'h5 == reg_select; // @[ChiselTop.scala 504:24]
  wire [15:0] _Kp_yaw_T_1 = {io_data_in,Kp_yaw[7:0]}; // @[Cat.scala 33:92]
  wire [15:0] _GEN_0 = 3'h5 == reg_select ? _Kp_yaw_T_1 : Kp_yaw; // @[ChiselTop.scala 361:23 504:24 510:24]
  wire [15:0] _GEN_1 = 3'h4 == reg_select ? _Kd_roll_T_1 : Kd_roll; // @[ChiselTop.scala 360:24 504:24 509:25]
  wire [15:0] _GEN_2 = 3'h4 == reg_select ? Kp_yaw : _GEN_0; // @[ChiselTop.scala 361:23 504:24]
  wire [15:0] _GEN_3 = 3'h3 == reg_select ? _Kp_roll_T_1 : Kp_roll; // @[ChiselTop.scala 359:24 504:24 508:25]
  wire [15:0] _GEN_4 = 3'h3 == reg_select ? Kd_roll : _GEN_1; // @[ChiselTop.scala 360:24 504:24]
  wire [15:0] _GEN_5 = 3'h3 == reg_select ? Kp_yaw : _GEN_2; // @[ChiselTop.scala 361:23 504:24]
  wire [15:0] _GEN_6 = 3'h2 == reg_select ? _Kd_pitch_T_1 : Kd_pitch; // @[ChiselTop.scala 504:24 358:25 507:26]
  wire [15:0] _GEN_7 = 3'h2 == reg_select ? Kp_roll : _GEN_3; // @[ChiselTop.scala 359:24 504:24]
  wire [15:0] _GEN_8 = 3'h2 == reg_select ? Kd_roll : _GEN_4; // @[ChiselTop.scala 360:24 504:24]
  wire [15:0] _GEN_9 = 3'h2 == reg_select ? Kp_yaw : _GEN_5; // @[ChiselTop.scala 361:23 504:24]
  wire [15:0] _Kp_pitch_T_3 = {Kp_pitch[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _Ki_pitch_T_3 = {Ki_pitch[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _Kd_pitch_T_3 = {Kd_pitch[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _Kp_roll_T_3 = {Kp_roll[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _Kd_roll_T_3 = {Kd_roll[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _Kp_yaw_T_3 = {Kp_yaw[15:8],io_data_in}; // @[Cat.scala 33:92]
  wire [15:0] _GEN_21 = _T_5 ? _Kp_yaw_T_3 : Kp_yaw; // @[ChiselTop.scala 361:23 513:24 519:24]
  wire [15:0] _GEN_22 = _T_4 ? _Kd_roll_T_3 : Kd_roll; // @[ChiselTop.scala 360:24 513:24 518:25]
  wire [15:0] _GEN_23 = _T_4 ? Kp_yaw : _GEN_21; // @[ChiselTop.scala 361:23 513:24]
  wire [15:0] _GEN_24 = _T_3 ? _Kp_roll_T_3 : Kp_roll; // @[ChiselTop.scala 359:24 513:24 517:25]
  wire [15:0] _GEN_25 = _T_3 ? Kd_roll : _GEN_22; // @[ChiselTop.scala 360:24 513:24]
  wire [15:0] _GEN_26 = _T_3 ? Kp_yaw : _GEN_23; // @[ChiselTop.scala 361:23 513:24]
  wire [15:0] _GEN_27 = _T_2 ? _Kd_pitch_T_3 : Kd_pitch; // @[ChiselTop.scala 513:24 358:25 516:26]
  wire [15:0] _GEN_28 = _T_2 ? Kp_roll : _GEN_24; // @[ChiselTop.scala 359:24 513:24]
  wire [15:0] _GEN_29 = _T_2 ? Kd_roll : _GEN_25; // @[ChiselTop.scala 360:24 513:24]
  wire [15:0] _GEN_30 = _T_2 ? Kp_yaw : _GEN_26; // @[ChiselTop.scala 361:23 513:24]
  assign io_left_elevon = _left_elevon_pwm_T_2[7:0]; // @[ChiselTop.scala 406:54]
  assign io_right_elevon = _right_elevon_pwm_T_2[7:0]; // @[ChiselTop.scala 407:56]
  assign io_rudder = _rudder_pwm_T_2[7:0]; // @[ChiselTop.scala 408:47]
  always @(posedge clock) begin
    if (reset) begin // @[ChiselTop.scala 356:25]
      Kp_pitch <= 16'h80; // @[ChiselTop.scala 356:25]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (3'h0 == reg_select) begin // @[ChiselTop.scala 504:24]
        Kp_pitch <= _Kp_pitch_T_1; // @[ChiselTop.scala 505:26]
      end
    end else if (_T) begin // @[ChiselTop.scala 513:24]
      Kp_pitch <= _Kp_pitch_T_3; // @[ChiselTop.scala 514:26]
    end
    if (reset) begin // @[ChiselTop.scala 357:25]
      Ki_pitch <= 16'h1a; // @[ChiselTop.scala 357:25]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (!(3'h0 == reg_select)) begin // @[ChiselTop.scala 504:24]
        if (3'h1 == reg_select) begin // @[ChiselTop.scala 504:24]
          Ki_pitch <= _Ki_pitch_T_1; // @[ChiselTop.scala 506:26]
        end
      end
    end else if (!(_T)) begin // @[ChiselTop.scala 513:24]
      if (_T_1) begin // @[ChiselTop.scala 513:24]
        Ki_pitch <= _Ki_pitch_T_3; // @[ChiselTop.scala 515:26]
      end
    end
    if (reset) begin // @[ChiselTop.scala 358:25]
      Kd_pitch <= 16'h3; // @[ChiselTop.scala 358:25]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (!(3'h0 == reg_select)) begin // @[ChiselTop.scala 504:24]
        if (!(3'h1 == reg_select)) begin // @[ChiselTop.scala 504:24]
          Kd_pitch <= _GEN_6;
        end
      end
    end else if (!(_T)) begin // @[ChiselTop.scala 513:24]
      if (!(_T_1)) begin // @[ChiselTop.scala 513:24]
        Kd_pitch <= _GEN_27;
      end
    end
    if (reset) begin // @[ChiselTop.scala 359:24]
      Kp_roll <= 16'h100; // @[ChiselTop.scala 359:24]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (!(3'h0 == reg_select)) begin // @[ChiselTop.scala 504:24]
        if (!(3'h1 == reg_select)) begin // @[ChiselTop.scala 504:24]
          Kp_roll <= _GEN_7;
        end
      end
    end else if (!(_T)) begin // @[ChiselTop.scala 513:24]
      if (!(_T_1)) begin // @[ChiselTop.scala 513:24]
        Kp_roll <= _GEN_28;
      end
    end
    if (reset) begin // @[ChiselTop.scala 360:24]
      Kd_roll <= 16'h33; // @[ChiselTop.scala 360:24]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (!(3'h0 == reg_select)) begin // @[ChiselTop.scala 504:24]
        if (!(3'h1 == reg_select)) begin // @[ChiselTop.scala 504:24]
          Kd_roll <= _GEN_8;
        end
      end
    end else if (!(_T)) begin // @[ChiselTop.scala 513:24]
      if (!(_T_1)) begin // @[ChiselTop.scala 513:24]
        Kd_roll <= _GEN_29;
      end
    end
    if (reset) begin // @[ChiselTop.scala 361:23]
      Kp_yaw <= 16'h4d; // @[ChiselTop.scala 361:23]
    end else if (load_high) begin // @[ChiselTop.scala 503:19]
      if (!(3'h0 == reg_select)) begin // @[ChiselTop.scala 504:24]
        if (!(3'h1 == reg_select)) begin // @[ChiselTop.scala 504:24]
          Kp_yaw <= _GEN_9;
        end
      end
    end else if (!(_T)) begin // @[ChiselTop.scala 513:24]
      if (!(_T_1)) begin // @[ChiselTop.scala 513:24]
        Kp_yaw <= _GEN_30;
      end
    end
    if (reset) begin // @[ChiselTop.scala 364:24]
      ez_prev <= 16'sh0; // @[ChiselTop.scala 364:24]
    end else begin
      ez_prev <= {{8{ez[7]}},ez}; // @[ChiselTop.scala 398:11]
    end
    if (reset) begin // @[ChiselTop.scala 365:24]
      ey_prev <= 16'sh0; // @[ChiselTop.scala 365:24]
    end else begin
      ey_prev <= {{8{ey[7]}},ey}; // @[ChiselTop.scala 399:11]
    end
    if (reset) begin // @[ChiselTop.scala 366:28]
      integral_ez <= 16'sh0; // @[ChiselTop.scala 366:28]
    end else begin
      integral_ez <= _integral_ez_T_4; // @[ChiselTop.scala 387:15]
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  Kp_pitch = _RAND_0[15:0];
  _RAND_1 = {1{`RANDOM}};
  Ki_pitch = _RAND_1[15:0];
  _RAND_2 = {1{`RANDOM}};
  Kd_pitch = _RAND_2[15:0];
  _RAND_3 = {1{`RANDOM}};
  Kp_roll = _RAND_3[15:0];
  _RAND_4 = {1{`RANDOM}};
  Kd_roll = _RAND_4[15:0];
  _RAND_5 = {1{`RANDOM}};
  Kp_yaw = _RAND_5[15:0];
  _RAND_6 = {1{`RANDOM}};
  ez_prev = _RAND_6[15:0];
  _RAND_7 = {1{`RANDOM}};
  ey_prev = _RAND_7[15:0];
  _RAND_8 = {1{`RANDOM}};
  integral_ez = _RAND_8[15:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module DS1050PWMAdapter(
  input  [15:0] io_pid_pwm,
  output [4:0]  io_ds_pwm
);
  assign io_ds_pwm = io_pid_pwm[15:11]; // @[ChiselTop.scala 332:27]
endmodule
module DataCollector(
  input         clock,
  input         reset,
  input  [7:0]  io_ui_in,
  output [7:0]  io_uo_out,
  output [7:0]  io_uio_out,
  output [7:0]  io_uio_oe,
  output [15:0] io_word_out,
  output        io_word_valid
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
`endif // RANDOMIZE_REG_INIT
  reg [1:0] state; // @[ChiselTop.scala 540:22]
  reg [15:0] dataReg; // @[ChiselTop.scala 543:24]
  reg  byteCount; // @[ChiselTop.scala 544:26]
  reg [15:0] word_out_reg; // @[ChiselTop.scala 545:29]
  reg  word_valid_reg; // @[ChiselTop.scala 546:31]
  wire [15:0] _dataReg_T = {io_ui_in,8'h0}; // @[Cat.scala 33:92]
  wire [15:0] _dataReg_T_2 = {dataReg[15:8],io_ui_in}; // @[Cat.scala 33:92]
  wire  _GEN_5 = ~byteCount | byteCount; // @[ChiselTop.scala 567:31 570:19 544:26]
  wire [7:0] _GEN_7 = 2'h3 == state ? dataReg[15:8] : 8'h0; // @[ChiselTop.scala 550:14 555:17 583:18]
  wire [7:0] _GEN_8 = 2'h3 == state ? 8'hff : 8'h0; // @[ChiselTop.scala 551:13 555:17 584:17]
  wire [1:0] _GEN_9 = 2'h3 == state ? 2'h0 : state; // @[ChiselTop.scala 555:17 586:13 540:22]
  wire  _GEN_10 = 2'h3 == state ? 1'h0 : word_valid_reg; // @[ChiselTop.scala 555:17 587:22 546:31]
  wire [7:0] _GEN_11 = 2'h2 == state ? dataReg[7:0] : 8'h0; // @[ChiselTop.scala 549:13 555:17 576:17]
  wire  _GEN_13 = 2'h2 == state | _GEN_10; // @[ChiselTop.scala 555:17 579:22]
  wire [7:0] _GEN_15 = 2'h2 == state ? 8'h0 : _GEN_7; // @[ChiselTop.scala 550:14 555:17]
  wire [7:0] _GEN_16 = 2'h2 == state ? 8'h0 : _GEN_8; // @[ChiselTop.scala 551:13 555:17]
  wire [7:0] _GEN_20 = 2'h1 == state ? 8'h0 : _GEN_11; // @[ChiselTop.scala 549:13 555:17]
  wire [7:0] _GEN_23 = 2'h1 == state ? 8'h0 : _GEN_15; // @[ChiselTop.scala 550:14 555:17]
  wire [7:0] _GEN_24 = 2'h1 == state ? 8'h0 : _GEN_16; // @[ChiselTop.scala 551:13 555:17]
  assign io_uo_out = 2'h0 == state ? 8'h0 : _GEN_20; // @[ChiselTop.scala 549:13 555:17]
  assign io_uio_out = 2'h0 == state ? 8'h0 : _GEN_23; // @[ChiselTop.scala 550:14 555:17]
  assign io_uio_oe = 2'h0 == state ? 8'h0 : _GEN_24; // @[ChiselTop.scala 551:13 555:17]
  assign io_word_out = word_out_reg; // @[ChiselTop.scala 552:15]
  assign io_word_valid = word_valid_reg; // @[ChiselTop.scala 553:17]
  always @(posedge clock) begin
    if (reset) begin // @[ChiselTop.scala 540:22]
      state <= 2'h0; // @[ChiselTop.scala 540:22]
    end else if (2'h0 == state) begin // @[ChiselTop.scala 555:17]
      if (io_ui_in != 8'h0) begin // @[ChiselTop.scala 558:30]
        state <= 2'h1; // @[ChiselTop.scala 559:15]
      end
    end else if (2'h1 == state) begin // @[ChiselTop.scala 555:17]
      if (~byteCount) begin // @[ChiselTop.scala 567:31]
        state <= 2'h2; // @[ChiselTop.scala 571:15]
      end
    end else if (2'h2 == state) begin // @[ChiselTop.scala 555:17]
      state <= 2'h3; // @[ChiselTop.scala 580:13]
    end else begin
      state <= _GEN_9;
    end
    if (reset) begin // @[ChiselTop.scala 543:24]
      dataReg <= 16'h0; // @[ChiselTop.scala 543:24]
    end else if (2'h0 == state) begin // @[ChiselTop.scala 555:17]
      if (io_ui_in != 8'h0) begin // @[ChiselTop.scala 558:30]
        dataReg <= _dataReg_T; // @[ChiselTop.scala 562:17]
      end
    end else if (2'h1 == state) begin // @[ChiselTop.scala 555:17]
      if (~byteCount) begin // @[ChiselTop.scala 567:31]
        dataReg <= _dataReg_T_2; // @[ChiselTop.scala 569:17]
      end
    end
    if (reset) begin // @[ChiselTop.scala 544:26]
      byteCount <= 1'h0; // @[ChiselTop.scala 544:26]
    end else if (2'h0 == state) begin // @[ChiselTop.scala 555:17]
      if (io_ui_in != 8'h0) begin // @[ChiselTop.scala 558:30]
        byteCount <= 1'h0; // @[ChiselTop.scala 560:19]
      end
    end else if (2'h1 == state) begin // @[ChiselTop.scala 555:17]
      byteCount <= _GEN_5;
    end
    if (reset) begin // @[ChiselTop.scala 545:29]
      word_out_reg <= 16'h0; // @[ChiselTop.scala 545:29]
    end else if (!(2'h0 == state)) begin // @[ChiselTop.scala 555:17]
      if (!(2'h1 == state)) begin // @[ChiselTop.scala 555:17]
        if (2'h2 == state) begin // @[ChiselTop.scala 555:17]
          word_out_reg <= dataReg; // @[ChiselTop.scala 578:20]
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 546:31]
      word_valid_reg <= 1'h0; // @[ChiselTop.scala 546:31]
    end else if (2'h0 == state) begin // @[ChiselTop.scala 555:17]
      if (io_ui_in != 8'h0) begin // @[ChiselTop.scala 558:30]
        word_valid_reg <= 1'h0; // @[ChiselTop.scala 563:24]
      end
    end else if (!(2'h1 == state)) begin // @[ChiselTop.scala 555:17]
      word_valid_reg <= _GEN_13;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  state = _RAND_0[1:0];
  _RAND_1 = {1{`RANDOM}};
  dataReg = _RAND_1[15:0];
  _RAND_2 = {1{`RANDOM}};
  byteCount = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  word_out_reg = _RAND_3[15:0];
  _RAND_4 = {1{`RANDOM}};
  word_valid_reg = _RAND_4[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
module ChiselTop(
  input        clock,
  input        reset,
  input  [7:0] io_ui_in,
  output [7:0] io_uo_out,
  input  [7:0] io_uio_in,
  output [7:0] io_uio_out,
  output [7:0] io_uio_oe
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
`endif // RANDOMIZE_REG_INIT
  wire  pid_clock; // @[ChiselTop.scala 49:19]
  wire  pid_reset; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_pitch_desired; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_roll_desired; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_yaw_desired; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_control; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_data_in; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_left_elevon; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_right_elevon; // @[ChiselTop.scala 49:19]
  wire [7:0] pid_io_rudder; // @[ChiselTop.scala 49:19]
  wire [15:0] ds_left_io_pid_pwm; // @[ChiselTop.scala 50:23]
  wire [4:0] ds_left_io_ds_pwm; // @[ChiselTop.scala 50:23]
  wire [15:0] ds_right_io_pid_pwm; // @[ChiselTop.scala 51:24]
  wire [4:0] ds_right_io_ds_pwm; // @[ChiselTop.scala 51:24]
  wire [15:0] ds_rudder_io_pid_pwm; // @[ChiselTop.scala 52:25]
  wire [4:0] ds_rudder_io_ds_pwm; // @[ChiselTop.scala 52:25]
  wire  dataCollector_clock; // @[ChiselTop.scala 55:29]
  wire  dataCollector_reset; // @[ChiselTop.scala 55:29]
  wire [7:0] dataCollector_io_ui_in; // @[ChiselTop.scala 55:29]
  wire [7:0] dataCollector_io_uo_out; // @[ChiselTop.scala 55:29]
  wire [7:0] dataCollector_io_uio_out; // @[ChiselTop.scala 55:29]
  wire [7:0] dataCollector_io_uio_oe; // @[ChiselTop.scala 55:29]
  wire [15:0] dataCollector_io_word_out; // @[ChiselTop.scala 55:29]
  wire  dataCollector_io_word_valid; // @[ChiselTop.scala 55:29]
  reg [7:0] pitch_desired_reg; // @[ChiselTop.scala 78:34]
  reg [7:0] roll_desired_reg; // @[ChiselTop.scala 79:33]
  reg [7:0] yaw_desired_reg; // @[ChiselTop.scala 80:32]
  reg [7:0] control_reg; // @[ChiselTop.scala 81:28]
  reg [7:0] data_in_reg; // @[ChiselTop.scala 82:28]
  reg [7:0] pending_target; // @[ChiselTop.scala 95:31]
  reg  expecting_data; // @[ChiselTop.scala 96:31]
  wire  _GEN_1 = dataCollector_io_word_out[15:8] == 8'hff | expecting_data; // @[ChiselTop.scala 100:32 102:24 96:31]
  wire [7:0] _pitch_desired_reg_T_1 = dataCollector_io_word_out[7:0]; // @[ChiselTop.scala 111:50]
  wire [7:0] _GEN_2 = 8'h21 == pending_target ? dataCollector_io_word_out[7:0] : data_in_reg; // @[ChiselTop.scala 106:30 115:34 82:28]
  wire [7:0] _GEN_3 = 8'h20 == pending_target ? dataCollector_io_word_out[7:0] : control_reg; // @[ChiselTop.scala 106:30 114:34 81:28]
  wire [7:0] _GEN_4 = 8'h20 == pending_target ? data_in_reg : _GEN_2; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_5 = 8'h12 == pending_target ? $signed(_pitch_desired_reg_T_1) : $signed(yaw_desired_reg); // @[ChiselTop.scala 106:30 113:38 80:32]
  wire [7:0] _GEN_6 = 8'h12 == pending_target ? control_reg : _GEN_3; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_7 = 8'h12 == pending_target ? data_in_reg : _GEN_4; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_8 = 8'h11 == pending_target ? $signed(_pitch_desired_reg_T_1) : $signed(roll_desired_reg); // @[ChiselTop.scala 106:30 112:39 79:33]
  wire [7:0] _GEN_9 = 8'h11 == pending_target ? $signed(yaw_desired_reg) : $signed(_GEN_5); // @[ChiselTop.scala 106:30 80:32]
  wire [7:0] _GEN_10 = 8'h11 == pending_target ? control_reg : _GEN_6; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_11 = 8'h11 == pending_target ? data_in_reg : _GEN_7; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_12 = 8'h10 == pending_target ? $signed(_pitch_desired_reg_T_1) : $signed(pitch_desired_reg); // @[ChiselTop.scala 106:30 111:40 78:34]
  wire [7:0] _GEN_13 = 8'h10 == pending_target ? $signed(roll_desired_reg) : $signed(_GEN_8); // @[ChiselTop.scala 106:30 79:33]
  wire [7:0] _GEN_14 = 8'h10 == pending_target ? $signed(yaw_desired_reg) : $signed(_GEN_9); // @[ChiselTop.scala 106:30 80:32]
  wire [7:0] _GEN_15 = 8'h10 == pending_target ? control_reg : _GEN_10; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_16 = 8'h10 == pending_target ? data_in_reg : _GEN_11; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_18 = 8'h3 == pending_target ? $signed(pitch_desired_reg) : $signed(_GEN_12); // @[ChiselTop.scala 106:30 78:34]
  wire [7:0] _GEN_19 = 8'h3 == pending_target ? $signed(roll_desired_reg) : $signed(_GEN_13); // @[ChiselTop.scala 106:30 79:33]
  wire [7:0] _GEN_20 = 8'h3 == pending_target ? $signed(yaw_desired_reg) : $signed(_GEN_14); // @[ChiselTop.scala 106:30 80:32]
  wire [7:0] _GEN_21 = 8'h3 == pending_target ? control_reg : _GEN_15; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_22 = 8'h3 == pending_target ? data_in_reg : _GEN_16; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_25 = 8'h2 == pending_target ? $signed(pitch_desired_reg) : $signed(_GEN_18); // @[ChiselTop.scala 106:30 78:34]
  wire [7:0] _GEN_26 = 8'h2 == pending_target ? $signed(roll_desired_reg) : $signed(_GEN_19); // @[ChiselTop.scala 106:30 79:33]
  wire [7:0] _GEN_27 = 8'h2 == pending_target ? $signed(yaw_desired_reg) : $signed(_GEN_20); // @[ChiselTop.scala 106:30 80:32]
  wire [7:0] _GEN_28 = 8'h2 == pending_target ? control_reg : _GEN_21; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_29 = 8'h2 == pending_target ? data_in_reg : _GEN_22; // @[ChiselTop.scala 106:30 82:28]
  wire [7:0] _GEN_33 = 8'h1 == pending_target ? $signed(pitch_desired_reg) : $signed(_GEN_25); // @[ChiselTop.scala 106:30 78:34]
  wire [7:0] _GEN_34 = 8'h1 == pending_target ? $signed(roll_desired_reg) : $signed(_GEN_26); // @[ChiselTop.scala 106:30 79:33]
  wire [7:0] _GEN_35 = 8'h1 == pending_target ? $signed(yaw_desired_reg) : $signed(_GEN_27); // @[ChiselTop.scala 106:30 80:32]
  wire [7:0] _GEN_36 = 8'h1 == pending_target ? control_reg : _GEN_28; // @[ChiselTop.scala 106:30 81:28]
  wire [7:0] _GEN_37 = 8'h1 == pending_target ? data_in_reg : _GEN_29; // @[ChiselTop.scala 106:30 82:28]
  wire  _GEN_48 = ~expecting_data & _GEN_1; // @[ChiselTop.scala 117:22 99:27]
  wire [7:0] left_elevon_wire = pid_io_left_elevon; // @[ChiselTop.scala 144:30 147:20]
  wire [7:0] right_elevon_wire = pid_io_right_elevon; // @[ChiselTop.scala 145:31 148:21]
  wire [7:0] rudder_wire = pid_io_rudder; // @[ChiselTop.scala 146:25 149:15]
  reg [1:0] pwm_cycle; // @[ChiselTop.scala 181:26]
  wire [1:0] _pwm_cycle_T_1 = pwm_cycle + 2'h1; // @[ChiselTop.scala 182:26]
  wire [7:0] tag_left = {3'h1,ds_left_io_ds_pwm}; // @[Cat.scala 33:92]
  wire [7:0] tag_right = {3'h2,ds_right_io_ds_pwm}; // @[Cat.scala 33:92]
  wire [7:0] tag_rudder = {3'h3,ds_rudder_io_ds_pwm}; // @[Cat.scala 33:92]
  wire [7:0] _packed_uo_T_1 = 2'h0 == pwm_cycle ? tag_left : 8'h0; // @[Mux.scala 81:58]
  wire [7:0] _packed_uo_T_3 = 2'h1 == pwm_cycle ? tag_right : _packed_uo_T_1; // @[Mux.scala 81:58]
  wire [7:0] _packed_uo_T_5 = 2'h2 == pwm_cycle ? tag_rudder : _packed_uo_T_3; // @[Mux.scala 81:58]
  PIDControllerTop pid ( // @[ChiselTop.scala 49:19]
    .clock(pid_clock),
    .reset(pid_reset),
    .io_pitch_desired(pid_io_pitch_desired),
    .io_roll_desired(pid_io_roll_desired),
    .io_yaw_desired(pid_io_yaw_desired),
    .io_control(pid_io_control),
    .io_data_in(pid_io_data_in),
    .io_left_elevon(pid_io_left_elevon),
    .io_right_elevon(pid_io_right_elevon),
    .io_rudder(pid_io_rudder)
  );
  DS1050PWMAdapter ds_left ( // @[ChiselTop.scala 50:23]
    .io_pid_pwm(ds_left_io_pid_pwm),
    .io_ds_pwm(ds_left_io_ds_pwm)
  );
  DS1050PWMAdapter ds_right ( // @[ChiselTop.scala 51:24]
    .io_pid_pwm(ds_right_io_pid_pwm),
    .io_ds_pwm(ds_right_io_ds_pwm)
  );
  DS1050PWMAdapter ds_rudder ( // @[ChiselTop.scala 52:25]
    .io_pid_pwm(ds_rudder_io_pid_pwm),
    .io_ds_pwm(ds_rudder_io_ds_pwm)
  );
  DataCollector dataCollector ( // @[ChiselTop.scala 55:29]
    .clock(dataCollector_clock),
    .reset(dataCollector_reset),
    .io_ui_in(dataCollector_io_ui_in),
    .io_uo_out(dataCollector_io_uo_out),
    .io_uio_out(dataCollector_io_uio_out),
    .io_uio_oe(dataCollector_io_uio_oe),
    .io_word_out(dataCollector_io_word_out),
    .io_word_valid(dataCollector_io_word_valid)
  );
  assign io_uo_out = ~dataCollector_io_word_valid ? _packed_uo_T_5 : dataCollector_io_uo_out; // @[ChiselTop.scala 193:38 194:15 190:30]
  assign io_uio_out = dataCollector_io_uio_out; // @[ChiselTop.scala 65:14]
  assign io_uio_oe = dataCollector_io_uio_oe; // @[ChiselTop.scala 66:13]
  assign pid_clock = clock;
  assign pid_reset = reset;
  assign pid_io_pitch_desired = pitch_desired_reg; // @[ChiselTop.scala 135:24]
  assign pid_io_roll_desired = roll_desired_reg; // @[ChiselTop.scala 136:23]
  assign pid_io_yaw_desired = yaw_desired_reg; // @[ChiselTop.scala 137:22]
  assign pid_io_control = control_reg; // @[ChiselTop.scala 140:18]
  assign pid_io_data_in = data_in_reg; // @[ChiselTop.scala 141:18]
  assign ds_left_io_pid_pwm = {8'h0,left_elevon_wire}; // @[Cat.scala 33:92]
  assign ds_right_io_pid_pwm = {8'h0,right_elevon_wire}; // @[Cat.scala 33:92]
  assign ds_rudder_io_pid_pwm = {8'h0,rudder_wire}; // @[Cat.scala 33:92]
  assign dataCollector_clock = clock;
  assign dataCollector_reset = reset;
  assign dataCollector_io_ui_in = io_ui_in; // @[ChiselTop.scala 58:26]
  always @(posedge clock) begin
    if (reset) begin // @[ChiselTop.scala 78:34]
      pitch_desired_reg <= 8'sh0; // @[ChiselTop.scala 78:34]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (!(~expecting_data)) begin // @[ChiselTop.scala 99:27]
        if (!(8'h0 == pending_target)) begin // @[ChiselTop.scala 106:30]
          pitch_desired_reg <= _GEN_33;
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 79:33]
      roll_desired_reg <= 8'sh0; // @[ChiselTop.scala 79:33]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (!(~expecting_data)) begin // @[ChiselTop.scala 99:27]
        if (!(8'h0 == pending_target)) begin // @[ChiselTop.scala 106:30]
          roll_desired_reg <= _GEN_34;
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 80:32]
      yaw_desired_reg <= 8'sh0; // @[ChiselTop.scala 80:32]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (!(~expecting_data)) begin // @[ChiselTop.scala 99:27]
        if (!(8'h0 == pending_target)) begin // @[ChiselTop.scala 106:30]
          yaw_desired_reg <= _GEN_35;
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 81:28]
      control_reg <= 8'h0; // @[ChiselTop.scala 81:28]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (!(~expecting_data)) begin // @[ChiselTop.scala 99:27]
        if (!(8'h0 == pending_target)) begin // @[ChiselTop.scala 106:30]
          control_reg <= _GEN_36;
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 82:28]
      data_in_reg <= 8'h0; // @[ChiselTop.scala 82:28]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (!(~expecting_data)) begin // @[ChiselTop.scala 99:27]
        if (!(8'h0 == pending_target)) begin // @[ChiselTop.scala 106:30]
          data_in_reg <= _GEN_37;
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 95:31]
      pending_target <= 8'h0; // @[ChiselTop.scala 95:31]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      if (~expecting_data) begin // @[ChiselTop.scala 99:27]
        if (dataCollector_io_word_out[15:8] == 8'hff) begin // @[ChiselTop.scala 100:32]
          pending_target <= dataCollector_io_word_out[7:0]; // @[ChiselTop.scala 101:24]
        end
      end
    end
    if (reset) begin // @[ChiselTop.scala 96:31]
      expecting_data <= 1'h0; // @[ChiselTop.scala 96:31]
    end else if (dataCollector_io_word_valid) begin // @[ChiselTop.scala 97:37]
      expecting_data <= _GEN_48;
    end
    if (reset) begin // @[ChiselTop.scala 181:26]
      pwm_cycle <= 2'h0; // @[ChiselTop.scala 181:26]
    end else begin
      pwm_cycle <= _pwm_cycle_T_1; // @[ChiselTop.scala 182:13]
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  pitch_desired_reg = _RAND_0[7:0];
  _RAND_1 = {1{`RANDOM}};
  roll_desired_reg = _RAND_1[7:0];
  _RAND_2 = {1{`RANDOM}};
  yaw_desired_reg = _RAND_2[7:0];
  _RAND_3 = {1{`RANDOM}};
  control_reg = _RAND_3[7:0];
  _RAND_4 = {1{`RANDOM}};
  data_in_reg = _RAND_4[7:0];
  _RAND_5 = {1{`RANDOM}};
  pending_target = _RAND_5[7:0];
  _RAND_6 = {1{`RANDOM}};
  expecting_data = _RAND_6[0:0];
  _RAND_7 = {1{`RANDOM}};
  pwm_cycle = _RAND_7[1:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
