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
	localparam SPI_CMD_READ = 8'h03;

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
	reg [1:0] spi_phase; // only track the first four bytes
	reg [7:0] spi_byte_out;

	reg [7:0] uart_tx;
	reg uart_tx_strobe;

	// select which output goes to the PCH
	reg read_in_progress = 0;
	wire output_fpga = !read_in_progress;
	wire output_ram = read_addr[23]; // select which RAM based on the top bit
	wire [3:0] spi_do_fpga;
	assign spi_do = output_fpga ? spi_do_fpga : output_ram ? ram1_di : ram0_di;

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

	reg spi_sr_srp0 = 0;
	reg spi_sr_sec = 0;
	reg spi_sr_tb = 0;
	reg [2:0] spi_sr_bp = 0;
	reg spi_sr_wel = 0;
	reg spi_sr_busy = 0;
	wire [7:0] spi_sr = { spi_sr_srp0, spi_sr_sec, spi_sr_tb, spi_sr_bp, spi_sr_wel, spi_sr_busy };
	wire [23:0] spi_id = 24'hC22018;

	reg [23:0] read_addr = 0;

	always @(posedge spi_clk or posedge spi_cs_in)
	begin
		if (spi_cs_in) begin
			// nothing to do, turn off any output drivers
			spi_do_enable <= 4'b0000;
			read_in_progress <= 0;
		end else
		if (spi_cmd_strobe_raw) begin
			spi_cmd <= spi_byte_raw;
			spi_byte <= spi_byte_raw;
			spi_phase <= 0;

			case(spi_byte_raw)
			SPI_CMD_RDID: begin
				spi_byte_out <= spi_id[23:16];
				spi_do_enable <= 4'b0010;
			end
			SPI_CMD_RDSR: begin
				spi_byte_out <= spi_sr;
				spi_do_enable <= 4'b0010;
			end
			SPI_CMD_WREN: begin
				// write-enable latch
				spi_sr_wel <= 1;
			end
			SPI_CMD_WRDS: begin
				spi_sr_wel <= 0;
			end
			SPI_CMD_READ: begin
				// a read has started -- output will be routed from
				// the RAM chips, not the fpga
				read_in_progress <= 1;
			end
			default: begin
				// unknown command, do not respond
			end
			endcase
		end else
		if (spi_byte_strobe_raw) begin
			spi_byte <= spi_byte_raw;

			// saturate spi_phase at 3 since we don't care beyond that
			if (spi_phase != 3)
				spi_phase <= spi_phase + 1;

			case (spi_cmd)
			SPI_CMD_RDID: begin
				if (spi_phase == 0)
					spi_byte_out <= spi_id[15:8];
				else
				if (spi_phase == 1)
					spi_byte_out <= spi_id[7:0];
			end
			SPI_CMD_READ: begin
				if (spi_phase == 0) begin
					read_addr[23:16] <= spi_byte_raw;
				end else
				if (spi_phase == 1) begin
					read_addr[15:8] <= spi_byte_raw;
				end else
				if (spi_phase == 2) begin
					read_addr[7:0] <= spi_byte_raw;
					spi_do_enable <= 4'b0010; // from the RAM chips
				end else
				begin
					// update the read address for each byte that is strobed
					read_addr <= read_addr + 1;
				end
			end
			default: begin
				// nothing to do
			end
			endcase
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
