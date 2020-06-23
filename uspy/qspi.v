`ifndef _qspi_v_
`define _qspi_v_

/* qspi run entirely in the clk domain */
module qspi_sync(
	input reset,
	input clk,
	// physical layer in spi_clk_in domain
	input spi_cs_in,
	input spi_clk_in,
	input [3:0] spi_data_in,
	output [3:0] spi_data_out,
	// outputs in clk domain
	output start_strobe,
	output byte_strobe,
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

	reg [2:0] spi_mode = 1; // 1, 2 or 4 bits at a time
	reg [7:0] spi_byte = 0;
	reg [2:0] spi_bits = 0;
	reg spi_got_cmd = 0;

	wire [7:0] spi_byte_1_next = { spi_byte[6:0], spi_data[0:0] };
	wire [7:0] spi_byte_2_next = { spi_byte[5:0], spi_data[1:0] };
	wire [7:0] spi_byte_4_next = { spi_byte[3:0], spi_data[3:0] };

	wire [7:0] next_spi_byte =
		spi_mode == 1 ? spi_byte_1_next : 
		spi_mode == 2 ? spi_byte_2_next : 
		spi_mode == 4 ? spi_byte_4_next : 
		8'hFF;

	wire [2:0] next_spi_bits = spi_bits + spi_mode;

	assign byte_strobe = next_spi_bits == 0 && spi_clk_rising;
	assign start_strobe = !spi_got_cmd && byte_strobe && spi_clk_rising;

	reg [7:0] spi_byte_out;
	//assign byte = spi_byte_out;
	assign byte = next_spi_byte;

	wire spi_cs = spi_data[5];
	wire spi_clk = spi_data[4];
	reg spi_clk_prev;
	wire spi_clk_rising = !spi_clk_prev && spi_clk;
	wire spi_clk_falling = spi_clk_prev && !spi_clk;
	wire spi_clk_edge = spi_clk_prev != spi_clk;

	reg [7:0] spi_byte_tx;
	assign spi_data_out = spi_byte_tx[7:4]; // assume always qspi

	always @(posedge clk)
	begin
		// buffer the incoming wires to sync into clk domain
		spi_data <= spi_data_2;
		spi_data_2 <= spi_data_1;
		spi_data_1 <= spi_data_0;
		spi_clk_prev = spi_clk;

		if (spi_cs)
		begin
			spi_mode <= 1;
			spi_bits <= 0;
			spi_got_cmd <= 0;
		end else
		if (spi_clk_rising)
		begin
			spi_bits <= next_spi_bits;
			spi_byte <= next_spi_byte;

			if (byte_strobe)
			begin
				// switch into high-speed when an EB is received as a command
				if (!spi_got_cmd && 8'hEB == next_spi_byte)
					spi_mode <= 4;

				spi_got_cmd <= 1;
				spi_byte_out <= next_spi_byte;
			end
		end else
		if (spi_clk_falling)
		begin
			if (spi_bits == 0)
				spi_byte_tx <= byte_tx;
			else
			if (spi_mode == 1)
				spi_byte_tx <= { spi_byte_tx[6:0], 1'b0 };
			else
			if (spi_mode == 2)
				spi_byte_tx <= { spi_byte_tx[5:0], 2'b00 };
			else
			if (spi_mode == 4)
				spi_byte_tx <= { spi_byte_tx[3:0], 4'b0000 };
			else
				spi_byte_tx <= 8'hFF;
		end
	end
endmodule
`endif
