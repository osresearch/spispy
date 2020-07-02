/*
 * SPI controller to drive !CS and CLK.
 * Supports single/dual/quad SPI.
 * Drives spi clock at clk/2.
 *
 * Since this is the controller side, this is *backwards* from the uspispy code.
 * DI == D0 == output from FPGA to SPI
 * DO == D1 == input to FPGA from SPI
 *
 * For single SPI, data enable == 4'b0001
 * For quad SPI writes, data enable = 4'b1111 (all output)
 * For quad SPI reads, data enable = 4'b0000 (all input)
 *
 *               +------+
 *  !CS       ---| o    |----  +V
 *   DO / D1  ---|      |---- !RST / D3 
 *  !WP / D2  ---|      |----  CLK
 *  GND       ---|      |----  DI / D0
 *               +------+
 */
`ifndef _spi_controller_v_
`define _spi_controller_v_

module spi_controller(
	input clk,
	input reset,

	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	output spi_clk,

	input [2:0] spi_mode_in,
	input spi_byte_tx_strobe,
	input [7:0] spi_byte_tx,
	output [7:0] spi_byte_rx,
	output spi_idle
);
	reg running = 0;
	reg spi_clk = 1;
	reg [7:0] byte_rx = 0;
	reg [7:0] byte_tx = 0;
	reg [2:0] bits = 0;
	reg [2:0] spi_mode;
	wire [2:0] bits_next = bits + spi_mode;

	wire [3:0] spi_data_out_1 = { 1'bx, 1'bx, 1'bx, byte_tx[7] }; // NOTE!
	wire [3:0] spi_data_out_2 = { 1'bx, 1'bx, byte_tx[7:6] };
	wire [3:0] spi_data_out_4 = { byte_tx[7:4] };

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

	wire [7:0] byte_rx_1_next = { byte_rx[6:0], spi_data_in[1:1] }; // NOTE!
	wire [7:0] byte_rx_2_next = { byte_rx[5:0], spi_data_in[1:0] };
	wire [7:0] byte_rx_4_next = { byte_rx[3:0], spi_data_in[3:0] };

	wire [7:0] byte_rx_next =
		spi_mode == 1 ? byte_rx_1_next :
		spi_mode == 2 ? byte_rx_2_next :
		spi_mode == 4 ? byte_rx_4_next :
		8'bXXXXXXXX;

	assign spi_byte_rx = byte_rx_next;
	assign spi_idle = bits_next == 0;

	always @(posedge clk)
	begin
		if (reset) begin
			byte_rx <= 0;
			byte_tx <= 0;
			bits <= 0;
			running <= 0;
			spi_clk <= 0; // idle low
		end else
		if (spi_byte_tx_strobe) begin
			byte_tx <= spi_byte_tx;
			spi_mode <= spi_mode_in; // only update on tx start
			bits <= 0;
			spi_clk <= 0; // ensure that the data changes with clock low
			running <= 1;
		end else
		if (running) begin
			spi_clk <= ~spi_clk;
			if (spi_clk == 1) begin
				// clock OUT bytes on the falling edge
				byte_tx <= byte_tx_next;
				bits <= bits_next;
			end else begin
 				// clock IN bits on the rising edge
				byte_rx <= byte_rx_next;
			end

			running <= bits_next != 0;
		end else begin
			spi_clk <= 0;
		end
	end
endmodule

`endif
