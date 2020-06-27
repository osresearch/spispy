/*
 * States:
 * !CS not asserted: do nothing
 * !CS asserted, route it to the outputs and wait for SPI command

Supported commands:

0x9F JDEC ID - uspispy
0x03 Normal Read - sent to chips
0x06 Write enable - uspispy
0x04 Write disable - uspispy
0x05 Read status register - uspispy

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

	// spi data commands
	localparam SPI_CMD_RDID = 8'h9F;
	localparam SPI_CMD_RDSR = 8'h05;
	localparam SPI_CMD_WREN = 8'h06;
	localparam SPI_CMD_WRDS = 8'h04;

	// these will need some adjustment
	assign spi_cs_enable = 0;
	reg [3:0] spi_do_enable = 0;
	assign ram0_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram0
	assign ram1_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram1
	assign ram0_do = spi_di;
	assign ram1_do = spi_di;

	wire spi_byte_strobe;
	wire spi_cmd_strobe;
	wire [7:0] spi_byte_raw;
	reg [7:0] spi_cmd;
	reg [7:0] spi_byte;
	reg [7:0] spi_phase;
	reg [7:0] spi_byte_out;

	reg [7:0] uart_tx;
	reg uart_tx_strobe;

	// select which output goes to the PCH
	wire output_fpga = 1;
	wire output_ram = 0;
	wire [3:0] spi_do_fpga;
	assign spi_do = output_fpga ? spi_do_fpga : output_ram ? ram0_di : ram1_di;

	wire spi_cmd_strobe_raw;
	wire spi_byte_strobe_raw;

	qspi_raw qspi_raw0(
		// physical interface
		.spi_cs_in(spi_cs_in),
		.spi_clk_in(spi_clk),
		.spi_data_in(spi_di),
		.spi_data_out(spi_do_fpga),
		.spi_mode(1),
		// logical interface
		.spi_byte_strobe(spi_byte_strobe_raw),
		.spi_cmd_strobe(spi_cmd_strobe_raw),
		.spi_byte_rx(spi_byte_raw),
		.spi_byte_tx(spi_byte_out)
	);

	reg [7:0] spi_sr = 8'hA5;
	wire [23:0] spi_id = 24'hC22018;

	always @(posedge spi_clk)
	begin
		if (spi_cs_in || reset) begin
			// nothing to do, turn off any output drivers
			spi_do_enable <= 4'b0000;
		end else
		if (spi_cmd_strobe_raw) begin
			spi_cmd <= spi_byte_raw;
			spi_byte <= spi_byte_raw;
			spi_phase <= 0;
			spi_do_enable <= 4'b0010;

			if (spi_byte_raw == SPI_CMD_RDID) begin
				spi_byte_out <= spi_id[23:16];
			end else
			if (spi_byte_raw == SPI_CMD_RDSR) begin
				spi_byte_out <= spi_sr;
			end else begin
				// unknown command, do not respond
				spi_byte_out <= 8'bXXXXXXXX;
				spi_do_enable <= 4'b0000;
			end
		end else
		if (spi_byte_strobe_raw) begin
			spi_byte <= spi_byte_raw;
			spi_phase <= spi_phase + 1;

			if (spi_cmd == SPI_CMD_RDID) begin
				if (spi_phase == 1)
					spi_byte_out <= spi_id[15:8];
				else
				if (spi_phase == 2)
					spi_byte_out <= spi_id[7:0];
				else
					spi_byte_out <= 8'bXXXXXXXX;
			end
		end
	end

	// synchronize the raw strobes to the clk domain
	strobe2strobe spi_cmd_sync(spi_clk, spi_cmd_strobe_raw, clk, spi_cmd_strobe);
	strobe2strobe spi_byte_sync(spi_clk, spi_byte_strobe_raw, clk, spi_byte_strobe);

	always @(posedge clk)
	begin
		uart_tx_strobe <= 0;

		if (spi_cmd_strobe) begin
			// new command!
			uart_tx_strobe <= 1;
			uart_tx <= spi_byte;

			// enable our data out pin TO the pch
			// todo: update spi_do_enable based on the single/dual/quad mode
		end else
		if (spi_byte_strobe) begin
			// accumulate command bytes
			uart_tx_strobe <= 1;
			uart_tx <= spi_byte;
			//spi_byte_out <= spi_byte_out + 1;
		end
	end
endmodule

`endif
