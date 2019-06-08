`ifndef _spi_device_v_
`define _spi_device_v_

/** \file
 * SPI device clocked in the spi_clk domain.
 *
 * If the system clock were sufficiently fast this could be done in
 * the other domain, but until then it is clocked by the SPI bus
 * itself.
 *
 * Inputs are unregistered so that they can be processed on the
 * rising edge of the 8th bit, allowing new data to be available
 * before the falling edge of the next spi_clk.
 *
 * Currently only supports single mode SPI. Dual and quad SPI require
 * that we a) parse the commands to know when to use it and b) have
 * bidirectional GPIO pins that we can swing to outputs.
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

module spi_device(
	input	spi_clk,
	input	spi_cs,
	input	spi_mosi,
	output	spi_miso,
	output	spi_rx_strobe,
	output [7:0] spi_rx_data,
	input 	spi_tx_ready,
	input [7:0] spi_tx_data,
	output spi_rx_cmd // flops from 0 to 1
);
	reg [2:0]     bit_count;
	reg [7:0]     mosi_reg;

	// these are unlatched so that they are available immediately
	//assign spi_rx_strobe = bit_count == 7;
	wire [7:0] mosi_reg_next = { mosi_reg[6:0], spi_mosi };
	reg [7:0] spi_rx_data;
	reg cmd_started;
	reg spi_rx_cmd;
	reg spi_rx_strobe;

	always @(posedge spi_clk or posedge spi_cs)
	begin
		if (spi_cs) begin
			// anytime the spi_cs goes high, reset the bit count
			bit_count <= 0;
			cmd_started <= 0;
		end else
		begin
			// shift in the rx data on the rising edge
			bit_count <= bit_count + 1;
			mosi_reg <= mosi_reg_next;

			if (bit_count == 7)
			begin
				if (!cmd_started)
					spi_rx_cmd <= !spi_rx_cmd;
				spi_rx_data <= mosi_reg_next;
				spi_rx_strobe <= !spi_rx_strobe;
				cmd_started <= 1;
			end
		end
	end

	always @(negedge spi_clk) begin
		// shift out the tx data on the falling edge
		spi_miso <= spi_tx_data[7-bit_count];
	end
endmodule

`endif
