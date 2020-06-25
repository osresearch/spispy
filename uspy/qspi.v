`ifndef _qspi_v_
`define _qspi_v_

`include "util.v"

/*
 * qspi run entirely in the clk domain.
 *
 * Data pins are in the order { RST/D3, WP/D2, DO/D1, DI/D0 }
 */
module qspi_sync(
	input reset,
	input clk,
	// physical layer in spi_clk_in domain
	input spi_clk_in,
	input spi_cs_in,
	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	// outputs in clk domain
	output spi_cs_out,
	output spi_cmd_strobe,
	output spi_byte_strobe,
	output [7:0] byte,
	input [7:0] byte_tx
);
	// spi_clk_in domain
	wire [5:0] spi_data_0 = {
		spi_cs_in,
		spi_clk_in,
		spi_data_in
	};
	reg [5:0] spi_data_1;
	reg [5:0] spi_data_2;
	reg [5:0] spi_data;

	// translate the polarity swapping cmd flag and byte flag into strobes in clk domain
	reg [7:0] spi_byte = 8'bXXXXXXXX;
	reg [7:0] spi_byte_out = 8'bXXXXXXXX;
	reg [7:0] spi_byte_tx = 8'bXXXXXXXX;
	reg [2:0] spi_bits = 0;
	reg spi_got_cmd = 0;

	assign byte = spi_byte_out;

	reg spi_cmd_flag = 0;
	reg spi_byte_flag = 0;
	strobe_sync spi_cmd_sync(clk, spi_cmd_flag, spi_cmd_strobe);
	strobe_sync spi_byte_sync(clk, spi_byte_flag, spi_byte_strobe);

	// sync the spi_cs_in
	reg [2:0] spi_cs_sync;
	assign spi_cs_out = spi_cs_sync[2];
	always @(posedge clk)
	begin
		spi_cs_sync <= { spi_cs_sync[1:0], spi_cs_in };
	end

	wire [7:0] spi_byte_next = { spi_byte[6:0], spi_data_in[0] };
	wire [2:0] spi_bits_next = spi_bits + 1;

	always @(posedge spi_clk_in or posedge spi_cs_in)
	begin
		if (spi_cs_in) begin
			spi_bits <= 0;
			spi_got_cmd <= 0;
		end else
		if (spi_bits_next != 0) begin
			// clock in some data, update the bit counter
			spi_bits <= spi_bits_next;
			spi_byte <= spi_byte_next;
		end else begin
			// 8 bits received, let the user know
			if (!spi_got_cmd)
				spi_cmd_flag <= ~spi_cmd_flag;
			spi_got_cmd <= 1;

			// spi_byte_out will have settled by the time spi_byte_flag
			// is translated into clk domain
			spi_bits <= spi_bits_next;
			spi_byte_out <= spi_byte_next;
			spi_byte_flag <= ~spi_byte_flag;
		end
	end

	assign spi_data_out = { 1'bx, 1'bx, spi_byte_tx[7], 1'bx };
	always @(negedge spi_clk_in)
	begin
		if (spi_bits == 0) begin
			spi_byte_tx <= byte_tx;
		end else begin
			spi_byte_tx <= { spi_byte_tx[6:0], 1'bX };
		end
	end

endmodule
`endif
