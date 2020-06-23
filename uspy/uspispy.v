/*
 * States:
 * !CS not asserted: do nothing
 * !CS asserted, route it to the outputs and wait for SPI command
 */

`ifndef _uspispy_v_
`define _uspispy_v_

`include "qspi.v"

module uspispy(
	input clk,
	input reset,

	// serial interface to the outside world
	input [7:0] uart_rx,
	input uart_rx_strobe,
	output [7:0] uart_tx,
	output uart_tx_strobe,

	// physical interface to the external SPI bus
	// this involves driving the !CS pin sometimes, so it has a output enable pin
	// the data pins are bidirectional
	input spi_clk,
	input spi_cs_i,
	input spi_cs_o,
	output spi_cs_enable,
	input [3:0] spi_di,
	output [3:0] spi_do,
	output [3:0] spi_do_enable,

	// physical interface to the PSRAM chips
	// !CS is always output, so it does not require an output enable pin
	// the data pins are bidirectional
	output ram0_clk,
	output ram0_cs,
	input [3:0] ram0_di,
	output [3:0] ram0_do,
	output [3:0] ram0_do_enable,

	output ram1_clk,
	output ram1_cs,
	input [3:0] ram1_di,
	output [3:0] ram1_do,
	output [3:0] ram1_do_enable,
);
	// copy the CS and CLK from the SPI bus to the RAM chips
	assign spi_cs_o = spi_cs_i;
	assign ram0_cs = spi_cs_i;
	assign ram1_cs = spi_cs_i;
	assign ram0_clk = spi_clk;
	assign ram1_clk = spi_clk;

	// these will need some adjustment
	assign spi_cs_enable = 0;
	assign spi_do_enable = 4'b0000; // No output driven
	assign ram0_do_enable = 4'b0001; // DO pin to ram0
	assign ram1_do_enable = 4'b0001; // DO pin to ram1
	assign ram0_do = spi_di;
	assign ram1_do = spi_di;

	wire spi_byte_strobe;
	wire spi_start_strobe;
	wire [7:0] spi_byte;
	reg [7:0] spi_cmd;
	reg [7:0] spi_phase;
	reg [7:0] spi_byte_out = 0;

	reg [7:0] uart_tx;
	reg uart_tx_strobe;

	qspi_sync qspi(
		.clk(clk),
		.reset(reset),
		// physical interface
		.spi_cs_in(spi_cs_i),
		.spi_clk_in(spi_clk),
		.spi_data_in(spi_di),
		.spi_data_out(spi_do),
		// logical interface
		.byte_strobe(spi_byte_strobe),
		.start_strobe(spi_start_strobe),
		.byte(spi_byte),
		.byte_tx(spi_byte_out)
	);

	always @(posedge clk)
	begin
		uart_tx_strobe <= 0;

		if (spi_cs_i || reset) begin
			// nothing to do unless selected
		end else
		if (!spi_byte_strobe) begin
			// nothing to do; wait for a command to start
		end else
		if (spi_start_strobe) begin
			// new command!
			uart_tx_strobe <= 1;
			uart_tx <= spi_byte;
			spi_cmd <= spi_byte;
			spi_phase <= 0; // start of new command
		end else begin
			// accumulate command bytes
			spi_phase <= spi_phase + 1;
			uart_tx_strobe <= 1;
			uart_tx <= spi_byte;
		end
	end
endmodule

`endif
