/** \file
 * Test sdram read/writes
 *
 * This configures the serial port at 3 Mb/s for test output
 *
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_120.v"
`include "sdram_controller.v"

module top(
	input clk_100mhz,
	inout [7:0] pmod1,

	// SDRAM physical interface
	output [12:0] sdram_addr,
	inout [7:0] sdram_data,
	output [1:0] sdram_bank,
	output sdram_clk,
	output sdram_cke,
	output sdram_we,
	output sdram_cs,
	output sdram_dqm,
	output sdram_ras,
	output sdram_cas
);
	wire locked, clk_120mhz;
	wire reset = !locked;
	pll_120 pll(clk_100mhz, clk_120mhz, locked);

	// sdram logical interface
	reg [24:0] sd_addr;
	wire [7:0] sd_rd_data;
	reg [7:0] sd_wr_data;
	reg sd_wr_enable;
	reg sd_rd_enable = 0;
	wire sd_busy;
	wire sd_rd_ready;
	assign sdram_clk = clk_120mhz;

	sdram_controller sdram(
		.clk(clk_120mhz),
		.rst_n(!reset),

		// physical interface
		.addr(sdram_addr),
		.bank_addr(sdram_bank),
		.data(sdram_data),
		.clock_enable(sdram_cke),
		.cs_n(sdram_cs),
		.ras_n(sdram_ras),
		.cas_n(sdram_cas),
		.we_n(sdram_we),
		.data_mask(sdram_dqm),

		// logical interface
		.wr_addr(sd_addr),
		.wr_enable(sd_wr_enable),
		.wr_data(sd_wr_data),
		.rd_addr(sd_addr),
		.rd_enable(sd_rd_enable),
		.rd_data(sd_rd_data),
		.rd_ready(sd_rd_ready),
		.busy(sd_busy),
	);

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

	// where the writer is wrapping
	reg [24:0] wr_addr = 0;

	reg [28:0] counter;
	always @(posedge clk_120mhz)
	begin
		counter <= counter + 1;
		uart_txd_strobe <= 0;

		// stop the write
		sd_wr_enable <= 0;

		if (uart_rxd_strobe) begin
			uart_txd <= uart_rxd;
			uart_txd_strobe <= 1;
		end else
		if (counter[24:0] == 0) begin
			uart_txd <= "A" + counter[28:25];
			uart_txd_strobe <= 1;
		end else
		if (!sd_busy && !sd_wr_enable) begin
			// write something to it!
			sd_addr <= wr_addr;
			sd_wr_data <= wr_addr ^ 8'h55;
			wr_addr <= wr_addr + 1;
			sd_wr_enable <= 1;
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
