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

module mem(
	input wr_clk,
	input wr_enable,
	input [7:0] wr_addr,
	input [7:0] wr_data,
	input rd_clk,
	input [7:0] rd_addr,
	output [7:0] rd_data
);
	reg [7:0] ram[0:255];
	reg [7:0] rd_data;

	always @(posedge wr_clk)
	begin
		if (wr_enable)
			ram[wr_addr] <= wr_data;
	end

	always @(posedge rd_clk)
	begin
		rd_data <= ram[rd_addr];
	end
endmodule


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
	// most of them time the commands will be sent to both of them,
	// with the exception of the write commands that must be filtered.
	assign spi_cs_out = spi_cs_in;
	reg ram0_enabled = 1;
	reg ram1_enabled = 1;
	assign ram0_cs = spi_cs_in || !ram0_enabled; // negative logic
	assign ram1_cs = spi_cs_in || !ram1_enabled; // negative logic
	assign ram0_clk = spi_clk;
	assign ram1_clk = spi_clk;

	// spi data commands
	localparam SPI_CMD_RDID = 8'h9F;
	localparam SPI_CMD_RDSR = 8'h05;
	localparam SPI_CMD_WREN = 8'h06;
	localparam SPI_CMD_WRDS = 8'h04;
	localparam SPI_CMD_READ = 8'h03;
	localparam SPI_CMD_PP3	= 8'h02;

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
	reg write_in_progress = 0;
	wire output_fpga = !read_in_progress;
	wire output_ram = spi_addr[23]; // select which RAM based on the top bit
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
	wire [7:0] spi_sr = {
		spi_sr_srp0,
		spi_sr_sec,
		spi_sr_tb,
		spi_sr_bp,
		spi_sr_wel,
		spi_sr_busy
	};
	wire [23:0] spi_id = 24'hC22018;

	reg [23:0] spi_addr = 0;

	reg write_buffer_enable = 0;
	reg [7:0] write_offset = 0;
	reg [7:0] read_offset = 0;
	wire [7:0] read_data;
	mem write_buffer(spi_clk, write_buffer_enable, write_offset, spi_byte_raw, clk, read_offset, read_data);

	always @(posedge spi_clk or posedge spi_cs_in)
	begin
		write_buffer_enable <= 0;

		if (spi_cs_in) begin
			// nothing to do, turn off any output drivers
			spi_do_enable <= 4'b0000;
			read_in_progress <= 0;
			write_in_progress <= 0;

			// select both ram chips most of the time
			ram1_enabled <= 1;
			ram0_enabled <= 1;
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
			SPI_CMD_PP3: begin
				// a write has started -- start buffering it
				write_in_progress <= 1;
				write_offset <= 0;
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

			// assume that the first three bytes are the address
			if (spi_phase == 0) spi_addr[23:16] <= spi_byte_raw;
			if (spi_phase == 1) spi_addr[15: 8] <= spi_byte_raw;
			if (spi_phase == 2) spi_addr[ 7: 0] <= spi_byte_raw;

			case (spi_cmd)
			SPI_CMD_RDID: begin
				if (spi_phase == 0)
					spi_byte_out <= spi_id[15:8];
				else
				if (spi_phase == 1)
					spi_byte_out <= spi_id[7:0];
			end
			SPI_CMD_READ: begin
				if (spi_phase == 2) begin
					// enable our output pin, which will be routed
					// from the appropriate RAM chip
					spi_do_enable <= 4'b0010;
				end else
				if (spi_phase == 3) begin
					// update the read address for each byte that is strobed
					spi_addr <= spi_addr + 1;
				end
			end
			SPI_CMD_PP3: begin
				if (spi_phase == 0) begin
					// select the correct RAM chip based on the top bit
					// of the address.
					ram0_enabled <= spi_byte_raw[7] == 0;
					ram1_enabled <= spi_byte_raw[7] == 1;
				end else
				if (spi_phase == 3) begin
					// store the byte into the write buffer
					write_buffer_enable <= 1;
					write_offset <= write_offset + 1;
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
