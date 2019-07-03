`include "usb_serial.v"
`include "../uart.v"
`include "../util.v"

module top (
  input  pin_clk,

  inout  pin_usbp,
  inout  pin_usbn,
  output pin_pu,

  output pin_led,
  output pin_8,
  output pin_9
);
  wire serial_tx = pin_8;

  wire clk_48mhz;
  wire lock;
  wire reset = !lock;

`ifdef TINYFPGA
  SB_PLL40_CORE #(
    .DIVR(4'b0000),
    .DIVF(7'b0101111),
    .DIVQ(3'b100),
    .FILTER_RANGE(3'b001),
    .FEEDBACK_PATH("SIMPLE"),
    .DELAY_ADJUSTMENT_MODE_FEEDBACK("FIXED"),
    .FDA_FEEDBACK(4'b0000),
    .DELAY_ADJUSTMENT_MODE_RELATIVE("FIXED"),
    .FDA_RELATIVE(4'b0000),
    .SHIFTREG_DIV_MODE(2'b00),
    .PLLOUT_SELECT("GENCLK"),
    .ENABLE_ICEGATE(1'b0)
  ) usb_pll_inst (
    .REFERENCECLK(pin_clk),
    .PLLOUTCORE(clk_48mhz),
    .PLLOUTGLOBAL(),
    .EXTFEEDBACK(),
    .DYNAMICDELAY(),
    .RESETB(1'b1),
    .BYPASS(1'b0),
    .LATCHINPUTVALUE(),
    .LOCK(lock),
    .SDI(),
    .SDO(),
    .SCLK()
`define TOMU
  );
`endif

`define TOMU
`ifdef TOMU
	assign lock = 1;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48mhz)
	);
`endif

	reg clk_24mhz;
	reg clk_12mhz;
	reg clk_6mhz;
	reg clk_3mhz;
	always @(posedge clk_48mhz) clk_24mhz = !clk_24mhz;
	always @(posedge clk_24mhz) clk_12mhz = !clk_12mhz;
	always @(posedge clk_12mhz) clk_6mhz = !clk_6mhz;
	always @(posedge clk_6mhz) clk_3mhz = !clk_3mhz;

	wire clk = clk_12mhz;

	wire uart_ready;
	reg [7:0] uart_data;
	reg uart_strobe;
	wire clk_1;
	divide_by_n #(.N(16)) div1(clk_48mhz, reset, clk_1);
	uart_tx uart(
		.mclk(clk),
		.reset(reset),
		.baud_x1(clk_1),
		.serial(serial_tx),
		.ready(uart_ready),
		.data(uart_data),
		.data_strobe(uart_strobe)
	);

  wire usb_p_tx;
  wire usb_n_tx;
  wire usb_p_rx;
  wire usb_n_rx;
  wire usb_tx_en;

  wire uart_tx_ready;
  reg [7:0] uart_tx_data;
  reg uart_tx_strobe;

  wire uart_rx_strobe;
  wire [7:0] uart_rx_data;

  reg [21:0] counter = 1;
initial pin_led <= 0;
  reg [4:0] out;
  wire host_presence;

  reg [23:0] init;
  reg active;
  always @(posedge clk) begin
	if (reset) begin
		init <= 0;
		active <= 0;
	end else
	if (&init == 1) begin
		active <= 1;
	end else begin
		init <= init + 1;
	end
  end

  wire fifo_data_available;
  reg [7:0] fifo_tx_data;
  reg fifo_tx_strobe;
  wire [7:0] fifo_rx_data;
  reg fifo_rx_strobe;
  
  fifo tx_buffer(
	.clk(clk),
	.reset(reset),
	.data_available(fifo_data_available),
	.write_data(fifo_tx_data),
	.write_strobe(fifo_tx_strobe),
	.read_data(fifo_rx_data),
	.read_strobe(fifo_rx_strobe)
  );

  always @(posedge clk) if (!reset) begin
	if (uart_tx_ready
	&& fifo_data_available
	&& !uart_tx_strobe
	) begin
		uart_tx_data <= fifo_rx_data;
		uart_tx_strobe <= 1;
		fifo_rx_strobe <= 1;
	end else begin
		uart_tx_strobe <= 0;
		fifo_rx_strobe <= 0;
	end
  end

  always @(posedge clk) if (active) begin
	fifo_tx_strobe <= 0;
	uart_strobe <= 0;

	counter <= counter + 1;
	pin_9 <= counter[21];

	if (uart_tx_ready
	&& !uart_tx_strobe
	&&  counter == 0
	) begin
		uart_data <= "A" + out[4:0];
		fifo_tx_data <= "A" + out[4:0];
		fifo_tx_strobe <= 1;
		out <= out + 1;
		pin_led <= !pin_led;

		uart_strobe <= 1;
	end

	if (uart_rx_strobe) begin
		pin_led <= 1;
		uart_data <= uart_rx_data;
		fifo_tx_data <= uart_rx_data;
		fifo_tx_strobe <= 1;

		if (!uart_strobe)
			uart_strobe <= 1;
	end
  end



  usb_serial serial(
    .clk_48mhz(clk_48mhz),
    .clk(clk),
    .reset(reset),
    .host_presence(host_presence),
    .uart_tx_ready(uart_tx_ready),
    .uart_tx_strobe(uart_tx_strobe),
    .uart_tx_data(uart_tx_data),
    .uart_rx_strobe(uart_rx_strobe),
    .uart_rx_data(uart_rx_data),
    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en)
  );

  assign pin_pu = 1'b1;

  wire usb_p_rx_io;
  wire usb_n_rx_io;
  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_rx_io;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_rx_io;

  tristate usbn_buffer(
	.pin(pin_usbn),
	.enable(usb_tx_en),
	.data_in(usb_n_rx_io),
	.data_out(usb_n_tx)
  );

  tristate usbp_buffer(
	.pin(pin_usbp),
	.enable(usb_tx_en),
	.data_in(usb_p_rx_io),
	.data_out(usb_p_tx)
  );
endmodule

module tristate(
  inout pin,
  input enable,
  input data_out,
  output data_in
);
  SB_IO #(
    .PIN_TYPE(6'b1010_01) // tristatable output
  ) buffer(
    .PACKAGE_PIN(pin),
    .OUTPUT_ENABLE(enable),
    .D_IN_0(data_in),
    .D_OUT_0(data_out)
  );
endmodule
