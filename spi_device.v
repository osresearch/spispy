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
	input clk,
	input reset,
	input	spi_clk,
	input	spi_cs,
	input	spi_mosi,
	output	spi_miso,
	output  spi_rx_cmd, // set for the first byte of a new transaction
	output	spi_rx_strobe,
	output [7:0] spi_rx_data,
	input [7:0] spi_tx_data
);
	reg [2:0]     bit_count;
	reg [7:0]     mosi_reg;

	reg [1:0] spi_cs_sync;
	reg [1:0] spi_clk_sync;
	reg [1:0] spi_mosi_sync;
	wire spi_clk_falling = spi_clk_sync[1] && !spi_clk_sync[0];
	wire spi_clk_rising = !spi_clk_sync[1] && spi_clk_sync[0];
	always @(posedge clk) begin
		spi_cs_sync <= { spi_cs_sync[0], spi_cs };
		spi_clk_sync <= { spi_clk_sync[0], spi_clk };
		spi_mosi_sync <= { spi_mosi_sync[0], spi_mosi };
	end

	wire [7:0] mosi_reg_next = { mosi_reg[6:0], spi_mosi };
	reg cmd_started;
	//reg [7:0] spi_rx_data;
	//reg spi_rx_cmd;
	//reg spi_rx_strobe;

`define SPI_UNLATCHED
`ifdef SPI_UNLATCHED
	// these are unlatched so that they are available immediately
	assign spi_rx_strobe = bit_count == 7 && spi_clk_rising && !spi_cs_sync[1];
	assign spi_rx_data = mosi_reg_next;
	assign spi_rx_cmd = spi_rx_strobe && cmd_started == 0;
`else
	// these are latched so that they are stable
	reg spi_rx_strobe;
	reg spi_rx_cmd;
	reg [7:0] spi_rx_data;
`endif

	reg [2:0] output_bit;
	assign spi_miso = spi_tx_data[output_bit];

	always @(posedge clk)
	begin
`ifndef SPI_UNLATCHED
		spi_rx_strobe <= 0;
		spi_rx_cmd <= 0;
`endif

		if (spi_cs_sync[1])
		begin
			// anytime the spi_cs goes high, reset the bit count
			cmd_started <= 0;
			bit_count <= 0;
			output_bit <= 7;
		end else
		if (spi_clk_rising)
		begin
			// shift in the rx data on the rising edge
			bit_count <= bit_count + 1;
			mosi_reg <= mosi_reg_next;
			if (bit_count == 7)
			begin
				cmd_started <= 1;
`ifndef SPI_UNLATCHED
				spi_rx_strobe <= 1;
				spi_rx_data <= mosi_reg_next;
				spi_rx_cmd <= cmd_started == 0;
`endif
			end
		end else
		if (spi_clk_falling)
		begin
			// shift out data on the falling edge
			output_bit <= output_bit - 1;
		end
	end

endmodule

`endif
