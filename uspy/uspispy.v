/*
 * States:
 * !CS not asserted: do nothing
 * !CS asserted, route it to the outputs and wait for SPI command
 */

`ifndef _uspispy_v_
`define _uspispy_v_

`include "qspi.v"
`include "util.v"

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
	input spi_cs_in,
	input spi_cs_out,
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
	assign spi_cs_out = spi_cs_in;
	assign ram0_cs = spi_cs_in;
	assign ram1_cs = spi_cs_in;
	assign ram0_clk = spi_clk;
	assign ram1_clk = spi_clk;

	// these will need some adjustment
	assign spi_cs_enable = 0;
	reg [3:0] spi_do_enable = 0;
	assign ram0_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram0
	assign ram1_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram1
	assign ram0_do = spi_di;
	assign ram1_do = spi_di;

	wire spi_byte_strobe;
	wire spi_start_strobe;
	wire [7:0] spi_byte;
	reg [7:0] spi_cmd;
	reg [7:0] spi_phase;
	reg [7:0] spi_byte_out = 8'hA5;

	reg [7:0] uart_tx;
	reg uart_tx_strobe;

	// select which output goes to the PCH
	wire output_fpga = 1;
	wire output_ram = 0;
	wire [3:0] spi_do_fpga;
	assign spi_do = output_fpga ? spi_do_fpga : output_ram ? ram0_di : ram1_di;

	wire spi_cs_sync;

	qspi_sync qspi(
		.clk(clk),
		.reset(reset),
		// physical interface
		.spi_cs_in(spi_cs_in),
		.spi_cs_out(spi_cs_sync),
		.spi_clk_in(spi_clk),
		.spi_data_in(spi_di),
		.spi_data_out(spi_do_fpga),
		// logical interface
		.spi_byte_strobe(spi_byte_strobe),
		.spi_cmd_strobe(spi_start_strobe),
		.byte(spi_byte),
		.byte_tx(spi_byte_out)
	);

	always @(posedge clk)
	begin
		uart_tx_strobe <= 0;

		if (spi_cs_sync || reset) begin
			// nothing to do and no output drivers unless selected
			spi_do_enable <= 0;
			spi_phase <= 0;
		end else
		if (spi_start_strobe) begin
			// new command!
			uart_tx_strobe <= 1;
			uart_tx <= "A"; //spi_byte;
			spi_cmd <= spi_byte;

			// enable our data out pin TO the pch
			// todo: update spi_do_enable based on the single/dual/quad mode
			spi_do_enable <= 4'b0010;
		end else
		if (spi_byte_strobe) begin
			// accumulate command bytes
			spi_phase <= spi_phase + 1;
			uart_tx_strobe <= 1;
			uart_tx <= `hexdigit(spi_phase); // spi_byte;
			uart_tx <= spi_byte;
			spi_byte_out <= spi_byte_out + 1;
		end
	end
endmodule

`endif
