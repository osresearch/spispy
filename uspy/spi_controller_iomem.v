/*
 * riscv iomem mapped SPI controller.
 */
`ifndef _spi_controller_iomem_v_
`define _spi_controller_iomem_v_
`include "spi_controller.v"

module spi_controller_iomem(
	input clk,
	input reset,

	// physical interface
	output spi_cs,
	output spi_clk,
	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	output [3:0] spi_data_enable,

	// iomem interface
	input sel,
	input [3:0] wstrb,
	input [7:0] addr,
	input [31:0] wdata,

	output ready,
	output [31:0] rdata
);
	reg spi_cs = 1;
	reg [3:0] spi_data_enable;
	reg [2:0] spi_mode;
	reg [7:0] spi_byte_tx;
	wire [7:0] spi_byte_rx;
	reg spi_byte_tx_strobe;
	wire spi_idle;

	spi_controller spi_i(
		.clk(clk),
		.reset(reset),

		.spi_clk(spi_clk),
		.spi_data_in(spi_data_in),
		.spi_data_out(spi_data_out),

		.spi_mode_in(spi_mode),
		.spi_byte_tx_strobe(spi_byte_tx_strobe),
		.spi_byte_tx(spi_byte_tx),
		.spi_byte_rx(spi_byte_rx),
		.spi_idle(spi_idle)
	);

	// this one is always ready
	assign ready = 1;
	assign rdata = {
		spi_idle, // 31:31
		15'h0000,
		spi_cs,		// 15:15
		spi_mode,	// 14:12
		spi_data_enable, // 11:8
		spi_byte_rx	// 7:0
	};

	//assign spi_byte_tx = wdata[7:0];
	//assign spi_byte_tx_strobe = sel && addr == 0 && wstrb[0];

	reg spi_idle_prev = 0;

	always @(posedge clk)
	begin
		spi_byte_tx_strobe <= 0;

		if (reset)
		begin
			spi_cs <= 1;
			spi_data_enable <= 0;
		end

		// only one address to write to
		if (sel && addr == 0)
		begin
			// allow per-byte updates to the various parts of the cr
			// top bits are not writeable
			//if (wstrb[3]) cr[31:24] <= wdata[31:24];
			//if (wstrb[2]) cr[23:16] <= wdata[23:16];
			if (wstrb[1]) begin
				spi_cs <= wdata[15];
				spi_mode <= wdata[14:12];
				spi_data_enable <= wdata[11:8];
			end
			if (wstrb[0]) begin
				// start a new transfer if this is a write to the bottom byte
				spi_byte_tx <= wdata[7:0];
				spi_byte_tx_strobe <= 1;
			end
		end
	end
endmodule

`endif
