/** \file
 * Test UART fifo
 *
 * This configures the serial port at 3 Mb/s for test output
 * and sends data to in bursts of max speed
 *
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"

module top(
	input clk_100mhz,
	inout [7:0] pmod1
);
	wire locked, clk_96mhz, clk;
	wire reset = !locked;
	pll_96 pll(clk_100mhz, clk_96mhz, locked);
//`define CLK48
`ifdef CLK48
	always @(posedge clk_96mhz) clk <= !clk;
`else
	assign clk = clk_96mhz;
`endif

	wire serial_txd;
	wire serial_rxd;

	gpio gpio_txd(
		.enable(1), // always on
		.pin(pmod1[0]),
		.out(serial_txd),
	);

	gpio #(.PULLUP(1)) gpio_rxd(
		.enable(0), // always input, with pullup
		.pin(pmod1[1]),
		.in(serial_rxd),
		.out(1),
	);

	// generate a 3 MHz/12 MHz serial clock from the 96 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	wire clk_3mhz, clk_12mhz;
`ifdef CLK48
	divide_by_n #(.N(4)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(16)) div4(clk, reset, clk_3mhz);
`else
	divide_by_n #(.N(8)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(32)) div4(clk, reset, clk_3mhz);
`endif

	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_txd_ready;

	reg [30:0] counter;
	reg [20:0] val;

	always @(posedge clk)
	begin
		counter <= counter + 1;
		uart_txd_strobe <= 0;

		if (counter[29:0] == 0) begin
			val <= 0;
		end else
		if (uart_txd_ready && val != 21'h100 && !uart_txd_strobe)
		begin
			uart_txd <= val[7:0];
			uart_txd_strobe <= 1;
			val <= val + 1;
		end
	end

	//uart_tx_fifo #(.NUM(8)) txd(
	uart_tx txd(
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready),
	);

endmodule
