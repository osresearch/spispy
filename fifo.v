`ifndef _fifo_v_
`define _fifo_v_

`include "util.v"

/*
 * Same clock domain FIFO with first-word-fallthrough.
 *
 * Holds up to NUM entries, which should be a power of two.
 * Can accept a new item every clock cycle,
 * Can export an item every clock cycle.
 *
 */

module fifo(
	input clk,
	input reset,
	// write side
	output space_available,
	input [WIDTH-1:0] write_data,
	input write_strobe,
	// read side
	output data_available,
	output more_available,
	output [WIDTH-1:0] read_data,
	input read_strobe
);
	parameter WIDTH = 8;
	parameter NUM = 256;
	parameter BITS = `CLOG2(NUM);
	parameter [BITS-1:0] FREESPACE = 1;

	reg [BITS:0] count;
	reg [BITS-1:0] rd_ptr;
	reg [BITS-1:0] wr_ptr;
	reg [WIDTH-1:0] read_data_ram;
	reg [WIDTH-1:0] read_data_fwft;
	reg [WIDTH-1:0] ram[0:NUM-1];

	assign more_available = (count > 1);
	assign data_available = (count != 0);
	assign space_available = (NUM - count > FREESPACE);

	reg fwft;
	assign read_data = fwft ? read_data_fwft : read_data_ram;

	always @(posedge clk) begin
		if (reset) begin
			rd_ptr <= 0;
			wr_ptr <= 0;
			count <= 0;
		end else begin
			if (write_strobe)
				ram[wr_ptr] <= write_data;

			read_data_ram <= ram[rd_ptr + read_strobe];
			fwft <= 0;

			// first word fall through
			if ((write_strobe && count == 0)
			||  (write_strobe && read_strobe && count == 1))
			begin
				read_data_fwft <= write_data;
				fwft <= 1;
			end

			rd_ptr <= rd_ptr + read_strobe;
			wr_ptr <= wr_ptr + write_strobe;
			count <= count + write_strobe - read_strobe;
		end
	end
endmodule


`endif
