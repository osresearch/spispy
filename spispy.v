/** \file
 * SPI bus emulator.
 *
 * This uses the onboard DRAM to store a flash image and can provide
 * it over a SPI bus to an external device.
 *
 * This also configures the serial port at 3 Mb/s for the command
 * protocol in user.v.
 *
 * FLCOMP in the IFD should be adjusted to use single SPI at 17 MHz.
 * Offset 0x30.  SPI_FREQUENCY_17MHZ=6 SPI_FREQUENCY_20MHZ=0
 * bit 30 = 0 == no dual fast read support
 * bit 20 = 0 == no fast read support
 * speed offsets 17, 21, 24, 27
 * 0x00 10 00 24
 *
 * 20 17
 * 1 000 00000000000100100
 *
 * minnow board dediprog pinout:
 * VCC  GND
 * CS   CLK
 * MISO MOSI
 * NC   NC
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"
`include "sdram_controller.v"
`include "user.v"
`include "spi_device.v"

module top(
	input clk_100mhz,
	inout [7:0] pmod1, // serial port
	inout [7:0] pmod2, // spi flash clip

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
	// beaglewire pinouts
	wire serial_txd_pin	= pmod1[0];
	wire serial_rxd_pin	= pmod1[1];

	wire spi_clk_pin	= pmod2[0];
	wire spi_cs_pin		= pmod2[1];
	wire spi_miso_pin	= pmod2[2];
	wire spi_mosi_pin	= pmod2[3];


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
	wire sd_refresh_inhibit;

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
		.pin(serial_txd_pin),
		.out(serial_txd),
	);

	gpio #(.PULLUP(1)) gpio_rxd(
		.enable(0), // always input, with pullup
		.pin(serial_rxd_pin),
		.in(serial_rxd),
		.out(1),
	);

	// generate a 3 MHz/12 MHz serial clock from the 96 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	// note that some Linux tools can have trouble keeping up with
	// this data rate; xxd -g1 for instance will drop bytes on
	// long bursts (more than 1KB).
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

`define UART_FIFO
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

	// SPI flash
	// in spispy mode all of them are inputs.
	// in toctou mode the cs and miso are output;
	// cs is driven high to deselect the flash chip
	// and miso is driven with the fpga provided data
	reg spi_tristate = 0;

	// if we're faking the !CS pin (driving the output high)
	// tell our SPI device that it is still selected.
	reg spi_cs_enable = 0;
	wire fake_spi_cs = spi_cs_enable ? 0 : spi_cs_in;

	reg spi_cs_out = 1; // always high
	reg spi_clk_out = 0;
	reg spi_mosi_out = 0;
	wire spi_miso_out; // controlled by spi_device

	wire spi_cs_in;
	wire spi_clk_in;
	wire spi_mosi_in;
	wire spi_miso_in;

	gpio #(.PULLUP(0)) gpio_spi_cs(
		.enable(spi_tristate & spi_cs_enable),
		.pin(spi_cs_pin),
		.in(spi_cs_in),
		.out(spi_cs_out),
	);

	gpio gpio_spi_clk(
		.enable(0), // always input, until we have a reader built in
		.pin(spi_clk_pin),
		.in(spi_clk_in),
		.out(spi_clk_out),
	);

	gpio gpio_spi_mosi(
		.enable(0), // always input, until we have a reader
		.pin(spi_mosi_pin),
		.in(spi_mosi_in),
		.out(spi_mosi_out),
	);

	gpio gpio_spi_miso(
		.enable(spi_tristate & spi_cs_enable),
		.pin(spi_miso_pin),
		.in(spi_miso_in),
		.out(spi_miso_out),
	);

	// remember that this is clocked in spi_clk domain,
	// so it is not to be trusted.
	wire spi_rx_strobe;
	wire [7:0] spi_rx_data;
	reg [7:0] spi_tx_data = 0;

	spi_device spi(
		.clk(clk),
		.reset(reset),
		.spi_clk(spi_clk_in),
		.spi_cs(fake_spi_cs),
		.spi_miso(spi_miso_out),
		.spi_mosi(spi_mosi_in),
		.spi_rx_strobe(spi_rx_strobe),
		.spi_rx_data(spi_rx_data),
		.spi_tx_data(spi_tx_data),
	);

	// serial output arbitrator between the user command
	// parser and the spi device decoder
	reg [7:0] spi_data_buffer;
	reg spi_data_pending;
	reg [7:0] user_data_buffer;
	reg user_data_pending;
	wire user_txd_strobe;
	wire [7:0] user_txd_data;
	wire user_txd_ready = !user_data_pending;

	reg [5:0] count = 0;
	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;

`ifdef 0
		if (spi_rx_strobe) begin
			uart_txd <= spi_rx_data;
			uart_txd_strobe <= 1;
			count <= count + 1;
		end
`else
		if (spi_rx_strobe) begin
			spi_data_buffer <= spi_rx_data;
			spi_data_pending <= 1;
		end else
		if (spi_data_pending && uart_txd_ready) begin
			uart_txd <= spi_data_buffer;
			uart_txd_strobe <= 1;
			spi_data_pending <= 0;
		end

		if (user_txd_strobe) begin
			user_data_buffer <= user_txd_data;
			user_data_pending <= 1;
		end else
		if (user_data_pending && uart_txd_ready && !spi_data_pending) begin
			uart_txd <= user_data_buffer;
			uart_txd_strobe <= 1;
			user_data_pending <= 0;
		end
`endif
	end


	// user command parser pulls in data from SPI
	// and from the serial port, drives the SDRAM
	user_command_parser #(
		.ADDR_BITS(ADDR_BITS)
	) parser(
		.clk(clk),
		.reset(reset),
		// serial I/O
		.uart_rxd(uart_rxd),
		.uart_rxd_strobe(uart_rxd_strobe),
		.uart_txd(user_txd_data),
		.uart_txd_strobe(user_txd_strobe),
		.uart_txd_ready(user_txd_ready),
		// SDRAM
		.sd_refresh_inhibit(sd_refresh_inhibit),
		.sd_addr(sd_addr),
		.sd_wr_data(sd_wr_data),
		.sd_rd_data(sd_rd_data),
		.sd_rd_ready(sd_rd_ready),
		.sd_we(sd_we),
		.sd_enable(sd_enable),
		.sd_busy(sd_busy),
	);
endmodule
