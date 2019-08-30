`ifndef _spi_controller_v_
`define _spi_controller_v_

/** \file
 * SPI controller, with its own clock.
 *
 * Usual wiring is:
 *
 *          +------+
 *  !CE  ---| o    |----  +V
 *   SO  ---|      |---- !RST
 *  !WP  ---|      |----  SCK
 *  GND  ---|      |----  SI
 *          +------+
 *
 */

`include "util.v"

module spi_controller(
	input clk,
	input reset,
	output	spi_clk,
	output	spi_cs,
	output	spi_mosi,
	input	spi_miso,
	output	spi_rx_strobe,
	output [7:0] spi_rx_data,
	input   spi_enable,
	output 	spi_tx_ready,
	input spi_tx_strobe,
	input [7:0] spi_tx_data
);
	reg [2:0]     bit_count;
	reg [7:0]     mosi_reg;
	reg [7:0]     miso_reg;

	parameter CLOCK_DIV = 48;
	wire spi_baud;
	divide_by_n #(CLOCK_DIV) div_n(clk, reset, spi_baud);

	reg [7:0] spi_rx_data;
	reg cmd_started;
	reg spi_rx_cmd;
	reg spi_rx_strobe;
	reg spi_cs;
	reg spi_mosi;
	reg spi_clk;

	reg spi_tx_ready;

	always @(posedge clk)
	begin
		spi_rx_strobe <= 0;
		spi_tx_ready <= 0;

		if (reset || !spi_enable) begin
			// wait for one clock cycle to reset the cs
			if (spi_baud)
				spi_cs <= 1;

			// don't do anything when not enabled
			mosi_reg <= 8'hFF;
			spi_mosi <= 1;
			spi_clk <= 1;
			bit_count <= 0;
		end else
		if (spi_tx_strobe) begin
			// start a new byte
			spi_cs <= 0;
			bit_count <= 7;
			spi_clk <= 1;
			mosi_reg <= spi_tx_data;
		end else
		if (spi_baud && !spi_clk) begin
			// clock out the data on the rising edge
			// and the data in on the falling edge
			spi_clk <= !spi_clk;
			miso_reg <= miso_reg << 1 | spi_miso;
			bit_count <= bit_count - 1;

			if (bit_count == 0) begin
				spi_rx_strobe <= 1;
				spi_tx_ready <= 1;
				spi_rx_data <= miso_reg << 1 | spi_miso;
			end
		end else
		if (spi_baud && spi_clk) begin
			// falling edge, update the output MOSI pin
			spi_clk <= !spi_clk;
			spi_mosi <= mosi_reg[7];
			mosi_reg <= mosi_reg << 1 | 1;
		end
	end
endmodule

`endif
