/** \file
 * Test fifo
 *
 * This configures the serial port at 3 Mb/s for test output
 * and sends data to in bursts of max speed
 *
 */
`default_nettype none
`include "util.v"
`include "fifo.v"
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
	//divide_by_n #(.N(8)) div1(clk, reset, clk_12mhz);
	//divide_by_n #(.N(32)) div4(clk, reset, clk_3mhz);
	divide_by_n #(.N(24)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(96)) div4(clk, reset, clk_3mhz);
`endif

	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_txd_ready;

	reg [30:0] counter;
	reg [20:0] val;

	reg [7:0] write_counter;
	reg [7:0] read_counter;

	reg fifo_reset = 1;
	wire werror, rerror;
	reg [7:0] write_data;
	wire [7:0] read_data;
	reg write_strobe;
	reg read_strobe;
	wire data_available, space_available;
	reg derror;

	fifo #(.NUM(8), .FREESPACE(3)) f(
		.clk(clk),
		.reset(fifo_reset),
		.werror(werror),
		.rerror(rerror),
		.data_available(data_available),
		.space_available(space_available),
		.read_data(read_data),
		.read_strobe(read_strobe),
		.write_data(write_data),
		.write_strobe(write_strobe),
	);

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		write_strobe <= 0;
		read_strobe <= 0;
		counter <= counter + 1;

		if (fifo_reset) begin
			write_counter <= 0;
			read_counter <= 0;
			fifo_reset <= counter != 0;
			derror <= 0;
		end else
		if (werror || rerror || derror) begin
			uart_txd <= "0"
				+ (derror << 2)
				+ (werror << 1)
				+ (rerror << 0)
				;
			uart_txd_strobe <= 1;
			fifo_reset <= 1;
		end else
		if (counter[24:0] == 1 && space_available && !write_strobe) begin
			// try to add to the fifo all the time
			write_data <= write_counter;
			write_counter <= write_counter + 1;
			write_strobe <= 1;
			uart_txd <= "A" + write_counter[4:0];
			uart_txd_strobe <= 1;
		end else
		if (data_available && counter[27:0] == 0)
		begin
			// slowly drain the fifo
			read_strobe <= 1;
			if (read_data != read_counter)
				derror <= 1;
			read_counter <= read_counter + 1;
			uart_txd <= "a" + read_data[4:0];
			uart_txd_strobe <= 1;
		end
	end

	//uart_tx_fifo #(.NUM(512)) txd(
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
