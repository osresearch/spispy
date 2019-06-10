/** \file
 * SPI flash tool.
 *
 * This reads/writes interactively to a SPI flash attached to the FPGA.
 *
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"
`include "spi_controller.v"

module top(
	input clk_100mhz,
	//inout [7:0] pmod1, // serial port
	output pmod1_0,
	input pmod1_1,
	output pmod1_2,
	output pmod1_3,
	output pmod1_4,
	inout [7:0] pmod2, // spi flash clip
);
	// beaglewire pinouts
	wire serial_txd_pin	= pmod1_0; // pmod1[0];
	wire serial_rxd_pin	= pmod1_1; // pmod1[1];

	wire spi_clk_pin	= pmod2[0];
	wire spi_cs_pin		= pmod2[1];
	wire spi_miso_pin	= pmod2[2];
	wire spi_mosi_pin	= pmod2[3];

	wire locked, clk_96mhz, clk;
	wire reset = !locked;
	pll_96 pll(clk_100mhz, clk_96mhz, locked);
	assign clk = clk_96mhz;

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
	// 1 megabaud
	divide_by_n #(.N(24)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(96)) div4(clk, reset, clk_3mhz);

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

`define UART_FIFO
`ifdef UART_FIFO
	uart_tx_fifo #(
		.NUM(512),
		.FREESPACE(15'h32), // ensure there is always space
	)
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

	// if we're faking the !CS pin (driving the output high)
	// tell our SPI device that it is still selected.
	reg spi_cs_enable = 0;
	wire fake_spi_cs = spi_cs_enable ? 0 : spi_cs_in;

	wire spi_cs_out;
	wire spi_clk_out;
	wire spi_mosi_out;
	wire spi_miso_out = 0; // controlled by spi_device

	wire spi_cs_in;
	wire spi_clk_in;
	wire spi_mosi_in;
	wire spi_miso_in;

	gpio gpio_spi_cs(
		.enable(1), // always an output
		.pin(spi_cs_pin),
		.in(spi_cs_in),
		.out(spi_cs_out),
	);

	gpio gpio_spi_clk(
		.enable(1), // always output
		.pin(spi_clk_pin),
		.in(spi_clk_in),
		.out(spi_clk_out),
	);

	gpio gpio_spi_mosi(
		.enable(1), // always output
		.pin(spi_mosi_pin),
		.in(spi_mosi_in),
		.out(spi_mosi_out),
	);

	gpio gpio_spi_miso(
		.enable(0), // always input
		.pin(spi_miso_pin),
		.in(spi_miso_in),
		.out(spi_miso_out),
	);

/*
	reg pmod1_2, pmod1_3, pmod1_4;
	always @(posedge clk) begin
		pmod1_2 <= spi_rx_strobe; //spi_cs_in;
		pmod1_3 <= spi_clk_in;
		pmod1_4 <= spi_mosi_in;
	end
*/
	//assign pmod1_2 = spi_cs_in;
	//assign pmod1_3 = spi_clk_in;

	reg rxd_bits;
	reg [7:0] bytes;
	reg [23:0] addr;
	reg rdid_strobe;
	reg read_flag;

	wire spi_data_strobe;
	wire [7:0] spi_data;

	spi_flash_controller spi(
		.clk(clk),
		.reset(reset),
		// physical interface
		.spi_clk(spi_clk_out),
		.spi_cs(spi_cs_out),
		.spi_miso(spi_miso_in),
		.spi_mosi(spi_mosi_out),

		// logical
		.data_strobe(spi_data_strobe),
		.data(spi_data),
		.rdid_strobe(rdid_strobe),
		.addr(addr),
		.read_flag(read_flag),
	);

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		if (reset) begin
			rxd_bits <= 0;
		end else begin
			if (spi_data_strobe) begin
				uart_txd <= hexdigit(spi_data[7:4]);
				uart_txd_strobe <= 1;
				rxd_bits <= 1;
			end else
			if (rxd_bits) begin
				uart_txd <= hexdigit(spi_data[4:0]);
				uart_txd_strobe <= 1;
				rxd_bits <= 0;
			end
		end
	end

	always @(posedge clk)
	begin
		rdid_strobe <= 0;

		if (uart_rxd_strobe) begin
			if (uart_rxd == "i") begin
				rdid_strobe <= 1;
			end else
			if (uart_rxd == "r") begin
				addr <= addr + 32;
				bytes <= 31;
				read_flag <= 1;
			end else
			if (uart_rxd == "0") begin
				addr <= -32;
			end
		end else
		if (spi_data_strobe && read_flag) begin
			if (bytes == 0)
				read_flag <= 0;
			else
				bytes <= bytes - 1;
		end
	end

endmodule


/*
 * Read a block of bytes from the flash chip, returning them one at a time.
 * as long as the read_byte flag is set, the read command will continue.
 *
 * To read another address, strobe the addr_strobe line.
 */
module spi_flash_controller(
	input clk,
	input reset,

	// data read from MISO
	output [7:0] data,
	output data_strobe,

	// read logical interface
	input [23:0] addr,
	input read_flag, // set high and leave high until done

	// flash id logical interface
	input rdid_strobe, // set high for a clock cycle

	// physical interface
	output spi_enable,
	output spi_clk,
	output spi_cs,
	output spi_miso,
	input spi_mosi
);
	wire spi_rx_strobe;
	wire [7:0] spi_rx_data;
	reg [7:0] spi_tx_data = 0;
	reg spi_tx_strobe = 0;
	wire spi_tx_ready;
	reg spi_enable = 0;
	reg data_strobe;

	spi_controller #(.CLOCK_DIV(128)) spi(
		.clk(clk),
		.reset(reset),
		// physical interface
		.spi_enable(spi_enable),
		.spi_clk(spi_clk),
		.spi_cs(spi_cs),
		.spi_miso(spi_miso),
		.spi_mosi(spi_mosi),
		// logical interface
		.spi_rx_strobe(spi_rx_strobe),
		.spi_rx_data(data),
		.spi_tx_ready(spi_tx_ready),
		.spi_tx_strobe(spi_tx_strobe),
		.spi_tx_data(spi_tx_data),
	);

	localparam
		CMD_RDID = 8'h9F,
		CMD_READ = 8'h03,
		CMD_RESET = 8'hFF;

	localparam
		MODE_IDLE = 3'b000,
		MODE_RDID = 3'b001,
		MODE_READ = 3'b010,
		MODE_INVALID = 3'b111;

	reg [2:0] mode;
	reg [7:0] bytes;
	reg [31:0] addr_reg;

	always @(posedge clk)
	begin
		spi_tx_strobe <= 0;
		data_strobe <= 0;

		if (reset) begin
			spi_enable <= 0;
			bytes <= 0;
		end else
		if (spi_enable && !spi_rx_strobe) begin
			// nothing to do
		end else
		if (spi_rx_strobe) begin
			bytes <= bytes + 1;

			if (mode == MODE_RDID) begin
				if (bytes == 0) begin
					spi_tx_data <= addr_reg[31:24];
					spi_tx_strobe <= 1;
				end else
				if (bytes == 1) begin
					spi_tx_data <= addr_reg[23:16];
					spi_tx_strobe <= 1;
					data_strobe <= 1;
				end else
				if (bytes == 2) begin
					spi_tx_data <= addr_reg[15: 8];
					spi_tx_strobe <= 1;
					data_strobe <= 1;
				end else
				if (bytes == 3) begin
					spi_tx_data <= addr_reg[ 7: 0];
					spi_tx_strobe <= 1;
					data_strobe <= 1;
				end else begin
					data_strobe <= 1;
					spi_enable <= 0;
				end
			end else
			if (mode == MODE_READ) begin
				if (bytes == 0) begin
					spi_tx_data <= addr_reg[23:16];
					spi_tx_strobe <= 1;
				end else
				if (bytes == 1) begin
					spi_tx_data <= addr_reg[15: 8];
					spi_tx_strobe <= 1;
				end else
				if (bytes == 2) begin
					spi_tx_data <= addr_reg[ 7: 0];
					spi_tx_strobe <= 1;
				end else
				begin
					// started receiving read data
					data_strobe <= 1;
					if (!read_flag)
						spi_enable <= 0;
				end
			end
		end else
		if (rdid_strobe) begin
			// send a read id command
			bytes <= 0;
			spi_enable <= 1;
			spi_tx_strobe <= 1;
			spi_tx_data <= CMD_RDID;
			mode <= MODE_RDID;
			addr_reg <= 32'h01020417;
		end else
		if (read_flag) begin
			// latch the input address
			bytes <= 0;
			spi_enable <= 1;
			spi_tx_strobe <= 1;
			spi_tx_data <= CMD_READ;
			mode <= MODE_READ;
			addr_reg <= addr;
		end else begin
			// nothing to do
		end
	end

endmodule
