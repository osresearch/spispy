`ifndef _spi_device_v_
`define _spi_device_v_

/** \file
 * SPI device clocked in the clk domain.
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
 * To ensure glitch-free MISO, the spi_tx_strobe should be set sometime after
 * spi_rx_strobe, but before the next falling edge of the SPI clock.  It is safe
 * to set the output on that same clock cycle.
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
	input [7:0] spi_tx_data,
	input spi_tx_strobe, // reload the output register
	output spi_timeout
);
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

	// track how long since the last clock and abort the transaction
	// if the clock is no longer ticking.
	reg [3:0] spi_clk_count;
	assign spi_timeout = spi_clk_count == 0;
	always @(posedge clk) begin
		if (spi_cs || reset || spi_clk_falling)
			spi_clk_count <= ~0;
		else
		if (spi_clk_count != 0)
			spi_clk_count <= spi_clk_count - 1;
	end

	reg [2:0] bit_count;
	reg [7:0] mosi_reg;
	wire [7:0] mosi_reg_next = { mosi_reg[6:0], spi_mosi };
	reg cmd_started;

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
	reg [7:0] miso_reg;
	//assign spi_miso = miso_reg[7]; // current output is top bit
	assign spi_miso = miso_reg[output_bit];

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
		if (spi_tx_strobe)
		begin
			// re-load the output register; this will cause the new
			// value to appear immediately.
			miso_reg <= spi_tx_data;
		end else
		if (spi_clk_falling)
		begin
			// shift out data on the falling edge
			// maintain the last bit forever
			//miso_reg <= { miso_reg[6:0], miso_reg[0] };
		end
	end

	always @(negedge spi_clk or posedge spi_cs)
	begin
		if (spi_cs)
			output_bit <= 7;
		else
			output_bit <= output_bit - 1;
	end


endmodule

`endif
