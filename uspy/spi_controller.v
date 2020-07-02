/*
 * SPI controller to drive !CS and CLK.
 * Supports single/dual/quad SPI.
 * Drives spi clock at clk/2.
 */
`ifndef _spi_controller_v_
`define _spi_controller_v_

module spi_controller(
	input clk,
	input reset,

	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	output spi_clk,

	input [2:0] spi_mode,
	input spi_byte_tx_strobe,
	input [7:0] spi_byte_tx,
	output [7:0] spi_byte_rx,
	output spi_byte_rx_strobe
);
	reg running = 0;
	reg spi_clk = 1;
	reg [7:0] byte_rx = 0;
	reg [7:0] byte_tx = 0;
	reg [2:0] bits = 0;
	wire [2:0] bits_next = bits + spi_mode;

	wire [3:0] spi_data_out_1 = { 1'bx, 1'bx, byte_tx[7], 1'bx };
	wire [3:0] spi_data_out_2 = { 1'bx, 1'bx, byte_tx[7:6] };
	wire [3:0] spi_data_out_4 = { byte_tx[7:3] };

	assign spi_data_out =
		spi_mode == 1 ? spi_data_out_1 :
		spi_mode == 2 ? spi_data_out_2 :
		spi_mode == 4 ? spi_data_out_4 :
		4'bXXXX;

	wire [7:0] byte_tx_1_next = { byte_tx[6:0], 1'bX };
	wire [7:0] byte_tx_2_next = { byte_tx[5:0], 2'bXX };
	wire [7:0] byte_tx_4_next = { byte_tx[3:0], 4'bXXXX };

	wire [7:0] byte_tx_next =
		spi_mode == 1 ? byte_tx_1_next :
		spi_mode == 2 ? byte_tx_2_next :
		spi_mode == 4 ? byte_tx_4_next :
		8'bXXXXXXXX;

	wire [7:0] byte_rx_1_next = { byte_rx[6:0], spi_data_in[0:0] };
	wire [7:0] byte_rx_2_next = { byte_rx[5:0], spi_data_in[1:0] };
	wire [7:0] byte_rx_4_next = { byte_rx[3:0], spi_data_in[3:0] };

	wire [7:0] byte_rx_next =
		spi_mode == 1 ? byte_rx_1_next :
		spi_mode == 2 ? byte_rx_2_next :
		spi_mode == 4 ? byte_rx_4_next :
		8'bXXXXXXXX;

	assign spi_byte_rx = byte_rx_next;
	assign spi_byte_rx_strobe = running && bits_next == 0;

	always @(posedge clk)
	begin
		if (reset) begin
			byte_rx <= 0;
			byte_tx <= 0;
			bits <= 0;
			running <= 0;
			spi_clk <= 1; // idle high
		end else
		if (spi_byte_tx_strobe) begin
			byte_tx <= spi_byte_tx;
			bits <= 0;
			spi_clk <= 0;
			running <= 1;
		end else
		if (running) begin
			spi_clk <= ~spi_clk;
			if (spi_clk == 1) begin
				byte_tx <= byte_tx_next;
				byte_rx <= byte_rx_next;
				bits <= bits_next;
			end

			if (bits_next == 0 && spi_clk == 0)
				running <= 0;
		end
	end
endmodule

`endif
