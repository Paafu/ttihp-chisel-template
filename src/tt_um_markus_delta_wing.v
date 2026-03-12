`default_nettype none

module tt_um_markus_delta_wing (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  wire reset = ~rst_n;
  wire _unused = &{ena};

  ChiselTop top (
    .clock(clk),
    .reset(reset),
    .io_ui_in(ui_in),
    .io_uo_out(uo_out),
    .io_uio_in(uio_in),
    .io_uio_out(uio_out),
    .io_uio_oe(uio_oe)
  );

endmodule

`default_nettype wire