/*
 * USB serial test
 */
`default_nettype none
`include "uart.v"
`include "pll_132.v"
`include "pll_48.v"
`include "usb_serial.v"

module top(
	input clk_25mhz,
	output [7:0] led,
	output wifi_gpio0,
	input ftdi_txd, // from the ftdi chip
	output ftdi_rxd, // to the ftdi chip

	// USB port directly wired to serial port
	inout usb_fpga_bd_dn,
	inout usb_fpga_bd_dp,
	output usb_fpga_pu_dp,
	output usb_fpga_pu_dn,
);
	// gpio0 must be tied high to prevent board from rebooting
	assign wifi_gpio0 = 1;

	reg [7:0] led_reg;
	assign led = led_reg;

	// Generate a 132 MHz clock from the 25 MHz reference
	// with a 180 degree out of phase sdram clk
	wire clk_132, clk_132_180, locked, reset = !locked;
	pll_132 pll_132_i(clk_25mhz, clk_132, clk_132_180, locked);
	wire clk = clk_132;

	// generate a 48 mhz clock for the USB serial port
	wire clk_48;
	pll_48 pll_48_i(clk_132, clk_48);

	// serial fifo, either usb serial or ftdi serial
	wire usb_txd_ready;
	reg [7:0] usb_txd;
	reg usb_txd_strobe;
	wire usb_rxd_strobe;
	wire [7:0] usb_rxd;

	wire usb_tx_en;
	wire usb_n_in, usb_n_out;
	wire usb_p_in, usb_p_out;
	assign usb_fpga_pu_dp = 1; // full speed 1.1 device
	assign usb_fpga_pu_dn = 0; // full speed 1.1 device
	
	TRELLIS_IO #(.DIR("BIDIR")) usb_p_buf(
		.T(!usb_tx_en),
		.B(usb_fpga_bd_dp),
		.I(usb_p_out),
		.O(usb_p_in),
	);
	TRELLIS_IO #(.DIR("BIDIR")) usb_n_buf(
		.T(!usb_tx_en),
		.B(usb_fpga_bd_dn),
		.I(usb_n_out),
		.O(usb_n_in),
	);

	usb_serial usb_serial_i(
		.clk_48mhz(clk_48),
		.clk(clk),
		.reset(reset),
		// physical
		.usb_p_tx(usb_p_out),
		.usb_n_tx(usb_n_out),
		.usb_p_rx(usb_tx_en ? 1'b1 : usb_p_in),
		.usb_n_rx(usb_tx_en ? 1'b0 : usb_n_in),
		.usb_tx_en(usb_tx_en),
		// logical
		.uart_tx_ready(usb_txd_ready),
		.uart_tx_data(usb_txd),
		.uart_tx_strobe(usb_txd_strobe),
		.uart_rx_data(usb_rxd),
		.uart_rx_strobe(usb_rxd_strobe),
		// .host_presence (not used)
	);

	// ftdi serial port interface for talking to the host system
	// 132 MHz clock / 48 == 3 megabaud
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_rxd_strobe;
	wire [7:0] uart_rxd;

	uart #(
		.DIVISOR(132 / 3), // 132 MHz
		.FIFO(32),
	) uart_i(
		.clk(clk),
		.reset(reset),
		// physical
		.serial_txd(ftdi_rxd), // fpga --> ftdi
		.serial_rxd(ftdi_txd), // fpga <-- ftdi
		// logical
		.txd(uart_txd),
		.txd_ready(uart_txd_ready),
		.txd_strobe(uart_txd_strobe),
		.rxd(uart_rxd),
		.rxd_strobe(uart_rxd_strobe),
	);

	reg [7:0] count;
	reg [7:0] byte;

	always @(posedge clk)
	begin
		usb_txd_strobe <= 0;

		if (reset) begin
			// nothing to do
			count <= 0;
		end else
		if (uart_rxd_strobe)
		begin
			led <= uart_rxd;
			byte <= uart_rxd;
			count <= 1;
		end else
		if (count != 0
		&& usb_txd_ready)
		begin
			usb_txd_strobe <= 1;
			usb_txd <= byte;
			count <= count - 1;
		end
	end
endmodule
