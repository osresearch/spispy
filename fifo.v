`ifndef _fifo_v_
`define _fifo_v_

`include "util.v"

/*
 * Same clock domain FIFO.
 *
 * Holds up to NUM entries, which should be a power of two.
 * Can accept a new item every clock cycle,
 * Can export an item every clock cycle.
 * Items will live at least one clock cycle in the ring
 * and become visible the next clock cycle.
 */

module fifo(
	input clk,
	input reset,
	output data_available,
	output space_available,
	input [WIDTH-1:0] write_data,
	input write_strobe,
	output [WIDTH-1:0] read_data,
	input read_strobe,

	// debugging ports
	output reg werror,
	output reg rerror,
	output [BITS-1:0] write_ptr,
	output [BITS-1:0] read_ptr
);
	parameter WIDTH = 8;
	parameter NUM = 256;
	parameter BITS = `CLOG2(NUM);
	parameter [BITS-1:0] FREESPACE = 1;

	reg [WIDTH-1:0] buffer[0:NUM-1];
	reg [BITS-1:0] write_ptr;
	reg [BITS-1:0] read_ptr;

	reg [WIDTH-1:0] read_data;

	assign data_available = read_ptr != write_ptr;
	wire [BITS-1:0] remaining = read_ptr > write_ptr
			? read_ptr - write_ptr
			: (NUM-1'b1) - (write_ptr - read_ptr);
	assign space_available = remaining > FREESPACE;
	//wire space_available = !data_available;

	always @(posedge clk) begin
		if (reset) begin
			write_ptr <= 0;
			read_ptr <= 0;
			werror <= 0;
			rerror <= 0;
		end else begin
			if (write_strobe) begin
				//if (!space_available)
				begin
					//werror <= 1;
				//end else begin
					buffer[write_ptr] <= write_data;
					write_ptr <= write_ptr + 1;
				end
			end

			if (read_strobe) begin
				if (!data_available)
					rerror <= 1;
				else
					read_ptr <= read_ptr + 1;
			end

			read_data <= buffer[read_ptr];
		end
	end
endmodule


`endif
