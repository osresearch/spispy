/** \file
 * Test serial echo to TTL pins.
 *
 * This configures the serial port at 3 Mb/s and directly routes
 * the incoming data to the outbound serial port.
 *
 */
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_120.v"

module top(
	input clk_100mhz,
	inout [7:0] pmod1
	//output pmod1_0,
	//input pmod1_1
);
	wire locked, clk_120mhz;
	wire reset = !locked;
	pll_120 pll(clk_100mhz, clk_120mhz, locked);

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

	// generate a 3 MHz/12 MHz serial clock from the 100 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	wire clk_3mhz, clk_12mhz;
	divide_by_n #(.N(10)) div1(clk_120mhz, reset, clk_12mhz);
	divide_by_n #(.N(40)) div4(clk_120mhz, reset, clk_3mhz);

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

	reg [28:0] counter;
	always @(posedge clk_120mhz)
	begin
		counter <= counter + 1;
		uart_txd_strobe <= 0;

		if (uart_rxd_strobe) begin
			uart_txd <= uart_rxd;
			uart_txd_strobe <= 1;
		end else
		if (counter[24:0] == 0) begin
			uart_txd <= "A" + counter[28:25];
			uart_txd_strobe <= 1;
		end
	end

	uart_tx txd(
		.mclk(clk_120mhz),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe)
/*
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
*/
	);

	uart_rx rxd(
		.mclk(clk_120mhz),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

endmodule
