/*
 * High-speed QSPI mux and router.
 *
 * This module implements the high-speed, low-latency part of the uspispy,
 * which involves routing the incoming SPI/QSPI read commands from the PCH to the
 * PSRAM chips.  Any other commands are routed instead to the external control module,
 * which is not latency sensitive, so it runs in a riscv soft cpu.  All commands are logged
 * to the riscv CPU, so that it can monitor the transactions as well.

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

	// logical controller to the outside world in clk domain
	output reg [7:0] spi_cmd_out,
	output reg [31:0] spi_addr_out,
	output reg [11:0] spi_len_out,
	output spi_cmd_strobe_out, // at the end of the command
	output reg [7:0] spi_byte_out,
	output spi_byte_strobe_out, // for every byte during a command, optional

	// allow the controller to track the status register and update it
	// after operations
	output [7:0] spi_sr,
	input [7:0] spi_sr_in,
	input spi_sr_strobe,

	// write to memory buffer output in spi_clk domain
	// this is used by write commands to buffer the incoming data before
	// committing it to the flash chip.
	output reg [7:0] write_data,
	output reg [7:0] write_addr,
	output reg write_strobe,

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
	localparam SPI_CMD_ERASE= 8'h20;

	// these will need some adjustment
	assign spi_cs_enable = 0;
	reg [3:0] spi_do_enable = 0;
	assign ram0_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram0
	assign ram1_do_enable = 4'b0001; // DI pin from the PCH is the DO pin TO ram1
	assign ram0_do = spi_di;
	assign ram1_do = spi_di;

	wire [7:0] spi_byte_raw;
	reg spi_got_cmd;
	reg [7:0] spi_cmd;
	reg [11:0] spi_len;
	reg [7:0] spi_byte_tx = 0;

	// select which output goes to the PCH
	reg read_in_progress = 0;
	wire output_fpga = !read_in_progress;
	wire output_ram = spi_addr[23]; // select which RAM based on the top bit
	wire [3:0] spi_do_fpga;
	assign spi_do = output_fpga ? spi_do_fpga : output_ram ? ram1_di : ram0_di;

	wire spi_cmd_strobe_raw;
	wire spi_byte_strobe_raw;
	wire spi_byte_strobe;
	wire spi_cmd_strobe;
	reg spi_cmd_done_flag = 0;

	// synchronize the raw strobes from spi_clk to the clk domain
	strobe_sync spi_cmd_sync(spi_cmd_done_flag, clk, spi_cmd_strobe);
	strobe2strobe spi_byte_sync(spi_clk, spi_byte_strobe_raw, clk, spi_byte_strobe_out);

	// watch for a spi_cmd_strobe and a return to cs idle
	wire spi_cs_rising;
	edge spi_cs_edge(clk, spi_cs_in, spi_cs_rising);
	assign spi_cmd_strobe_out = spi_cs_rising && spi_len_out != 0;

	always @(posedge clk)
	begin
		if (spi_cmd_strobe_out) begin
			spi_cmd_out <= spi_cmd;
			spi_len_out <= spi_len;
			spi_addr_out <= spi_addr;
		end
	end


	// decoder for the external SPI bus, from the PCH
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
		.spi_byte_tx(spi_byte_tx)
	);

	reg [7:0] spi_sr;
	wire [23:0] spi_id = 24'hC22018;

	reg [23:0] spi_addr = 0;

	// convert the spi sr update strobe from clk to spi_clk
	wire spi_clk_sr_strobe;
	strobe2strobe spi_sr_strobe_sync(clk, spi_sr_strobe, spi_clk, spi_clk_sr_strobe);

	always @(posedge spi_clk or posedge spi_cs_in)
	begin
		if (spi_cs_in) begin
			// signal that we have a command, if we have one

			// turn off any output drivers
			spi_do_enable <= 4'b0000;

			// reset the write address to the start of the buffer
			write_addr <= 0;
			spi_len <= 0;

			// select both ram chips to be ready for a read command
			ram1_enabled <= 1;
			ram0_enabled <= 1;
		end else
		if (spi_cmd_strobe_raw) begin
			spi_cmd_done_flag <= 0;
			spi_cmd <= spi_byte_raw;
			spi_byte_out <= spi_byte_raw;
			spi_len <= 1;

			// default is to *NOT* route the command to any of the PSRAM,
			// with the exception of the read command
			ram0_enabled <= 0;
			ram1_enabled <= 0;

			case(spi_byte_raw)
			SPI_CMD_RDID: begin
				spi_byte_tx <= spi_id[23:16];
				spi_do_enable <= 4'b0010;
			end
			SPI_CMD_RDSR: begin
				spi_byte_tx <= spi_sr;
				spi_do_enable <= 4'b0010;
			end
			SPI_CMD_WREN: begin
				// write-enable latch
				spi_sr[0] <= 1;
			end
			SPI_CMD_WRDS: begin
				spi_sr[0] <= 0;
			end
			SPI_CMD_READ: begin
				// a read has started!
				// route the read command to any of the PSRAM
				read_in_progress <= 1;
				ram0_enabled <= 1;
				ram1_enabled <= 1;
			end
			SPI_CMD_ERASE: begin
				// erase a 4 KB sector if writes are enabled
				spi_sr[1] <= 1;
			end
			SPI_CMD_PP3: begin
				// a write has started -- start buffering it if writes are enabled
				// and then select the correct chip to write once it is done
				spi_sr[1] <= 1;
			end
			default: begin
				// unknown command, do not respond
			end
			endcase
		end else
		if (spi_byte_strobe_raw) begin
			spi_byte_out <= spi_byte_raw;

			spi_len <= spi_len + 1;

			// assume that the first three bytes are the address
			if (spi_len == 1) spi_addr[23:16] <= spi_byte_raw;
			if (spi_len == 2) spi_addr[15: 8] <= spi_byte_raw;
			if (spi_len == 3) spi_addr[ 7: 0] <= spi_byte_raw;

			case (spi_cmd)
			SPI_CMD_RDID: begin
				if (spi_len == 1)
					spi_byte_tx <= spi_id[15:8];
				else
				if (spi_len == 2)
					spi_byte_tx <= spi_id[7:0];
			end
			SPI_CMD_READ: begin
				if (spi_len == 3) begin
					// enable our output pin, which will be routed
					// from the appropriate RAM chip
					spi_do_enable <= 4'b0010;
				end else
				if (spi_len > 3) begin
					// update the read address for each byte that is strobed
					// todo: add toctou capability
					spi_addr <= spi_addr + 1;
				end
			end
			SPI_CMD_PP3: begin
				if (spi_len > 3) begin
					// store the byte into the write buffer
					write_addr <= write_addr + 1;
					write_data <= spi_byte_raw;
					write_strobe <= 1;
				end
			end
			SPI_CMD_ERASE: begin
				// nothing to do; the external controller will handle it
			end
			default: begin
				// nothing to do
			end
			endcase
		end else begin
			// only write one byte per clk
			write_strobe <= 0;

			// update the spi_sr if the external controller has signaled a write
			if (spi_clk_sr_strobe)
				spi_sr <= spi_sr_in;
		end

	end

endmodule

`endif
