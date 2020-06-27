`ifndef _qspi_v_
`define _qspi_v_

`include "util.v"

/*
 * qspi run entirely in the spi_clk_in domain.
 * This will need to be wrapped with clock crossing primitives to
 * make it safe for use in the FPGA clk domain.
 *
 * Data pins are in the order { RST/D3, WP/D2, DO/D1, DI/D0 }
 */
module qspi_raw(
	// bits to receive per spi_clk: 1, 2 or 4
	input [2:0] spi_mode,
	// physical layer in spi_clk_in domain
	input spi_clk_in,
	input spi_cs_in,
	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	// outputs in spi_clk_in domain, available on rising edge of spi_clk_in
	output spi_cmd_strobe,
	output spi_byte_strobe,
	output [7:0] spi_byte_rx,
	input [7:0] spi_byte_tx
);
	// how many bits have been received so far
	reg [2:0] spi_bits = 0;
	reg [7:0] spi_byte = 0;

	// have we received the command byte (the first one)
	reg spi_got_cmd = 0;

	// these need to be updated for single/dual/quad SPI
	wire [7:0] spi_byte_next = { spi_byte[6:0], spi_data_in[0] };
	wire [2:0] spi_bits_next = spi_bits + 1;

	// async outputs so that values are available on the rising edge of spi_clk_in
	wire spi_byte_strobe = spi_bits_next == 0;
	wire spi_cmd_strobe = spi_byte_strobe && !spi_got_cmd;
	assign spi_byte_rx = spi_byte_next;

	always @(posedge spi_clk_in or posedge spi_cs_in)
	begin
		if (spi_cs_in) begin
			spi_bits <= 0;
			spi_got_cmd <= 0;
		end else begin
			spi_bits <= spi_bits_next;
			spi_byte <= spi_byte_next;

			// if this is the end of a byte, update the flag so that
			// only the first byte is flagged as a command
			if (spi_byte_strobe)
				spi_got_cmd <= 1;
		end
	end

	// these need to be updated for single/dual/quad SPI
	reg [7:0] tx_byte;
	assign spi_data_out = { 1'bx, 1'bx, tx_byte[7], 1'bx };

	always @(negedge spi_clk_in)
	begin
		if (spi_bits == 0) begin
			tx_byte <= spi_byte_tx;
		end else begin
			tx_byte <= { tx_byte[6:0], 1'bX };
		end
	end

endmodule
`endif
