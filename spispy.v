/** \file
 * SPI bus emulator.
 *
 * This uses the onboard DRAM to store a flash image and can provide
 * it over a SPI bus to an external device.
 *
 * This also configures the serial port at 3 Mb/s for the command
 * protocol.
 *
 * Command protocol looks like:
 * FF CMD [L2 L1 L0 A3 A2 A1 A0 .... ]
 *
 * Command bytes are "R", "W" and "V"
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"
`include "sdram_controller.v"
`include "user.v"

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
	wire locked, clk_96mhz, clk;
	wire reset = !locked;
	pll_96 pll(clk_100mhz, clk_96mhz, locked);
//`define CLK48
`ifdef CLK48
	always @(posedge clk_96mhz) clk <= !clk;
`else
	assign clk = clk_96mhz;
`endif

	// sdram logical interface
	parameter ADDR_BITS = 25; // 32 MB SDRAM chip
	wire [ADDR_BITS-1:0] sd_addr;
	wire [7:0] sd_wr_data;
	wire [7:0] sd_rd_data;
	wire sd_we;
	wire sd_enable;
	wire sd_rd_ready;
	wire sd_refresh_inhibit = 0;

	wire sd_busy;
	assign sdram_clk = clk;

	sdram_controller sdram(
		.clk(clk),
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
		.refresh_inhibit(sd_refresh_inhibit),
		.wr_addr(sd_addr),
		.wr_enable(sd_enable & sd_we),
		.wr_data(sd_wr_data),
		.rd_addr(sd_addr),
		.rd_enable(sd_enable & !sd_we),
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

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

`undef UART_FIFO
`ifdef UART_FIFO
	uart_tx_fifo #(.NUM(512))
`else
	uart_tx
`endif
	txd (
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready)
	);

	uart_rx rxd(
		.clk(clk),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	user_command_parser #(
		.ADDR_BITS(ADDR_BITS)
	) parser(
		.clk(clk),
		.reset(reset),
		// serial I/O
		.uart_rxd(uart_rxd),
		.uart_rxd_strobe(uart_rxd_strobe),
		.uart_txd(uart_txd),
		.uart_txd_strobe(uart_txd_strobe),
		.uart_txd_ready(uart_txd_ready),
		// SDRAM
		.sd_addr(sd_addr),
		.sd_wr_data(sd_wr_data),
		.sd_rd_data(sd_rd_data),
		.sd_rd_ready(sd_rd_ready),
		.sd_we(sd_we),
		.sd_enable(sd_enable),
		.sd_busy(sd_busy),
	);
endmodule
