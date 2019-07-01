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
	output [BITS-1:0] read_ptr,
	output [BITS-1:0] count
);
	parameter WIDTH = 8;
	parameter NUM = 256;
	parameter BITS = `CLOG2(NUM);
	parameter [BITS-1:0] FREESPACE = 1;

	reg [WIDTH-1:0] buffer[0:NUM-1];
	reg [BITS-1:0] write_ptr;
	reg [BITS-1:0] read_ptr;

`define simple_test
`ifdef simple_test
	reg [BITS-1:0] write_ptr_prev;
	wire [BITS-1:0] write_ptr_next = write_ptr + write_strobe;
	wire [BITS-1:0] read_ptr_next = read_ptr + read_strobe;
	//assign read_data = buffer[read_ptr];
	reg [WIDTH-1:0] read_data;
	reg [2:0] data_available_delay;
	assign data_available = !read_strobe && read_ptr_next != write_ptr_prev;
	assign space_available = write_ptr[BITS-1] == read_ptr[BITS-1]; // {{BITS}{write_ptr_next + 1'h1}} != read_ptr_next;
	//reg space_available;

	always @(posedge clk)
	begin
		if (reset) begin
			write_ptr <= 0;
			read_ptr <= 0;
		end else begin
			$display("strobe=%b %b data=%02x buf=[%02x,%02x,%02x,%02x]", write_strobe, read_strobe, read_data, buffer[0], buffer[1], buffer[2], buffer[3]);

			if (write_strobe)
				buffer[write_ptr] <= write_data;

			read_data <= buffer[read_ptr_next];

			write_ptr_prev <= write_ptr;
			write_ptr <= write_ptr_next;
			read_ptr <= read_ptr_next;

			//space_available <= {{BITS}{write_ptr + 1'b1}} != read_ptr_next;
			//data_available <= read_ptr_next != write_ptr;
		end
	end
`else
	reg [BITS-1:0] count;

/*
	wire [BITS-1:0] remaining = read_ptr > write_ptr
			? read_ptr - write_ptr
			: (NUM-1'b1) - (write_ptr - read_ptr);
	assign space_available = remaining > FREESPACE;
	//wire space_available = !data_available;
	assign space_available = write_ptr + 1'b1 != read_ptr;
*/

	wire [BITS-1:0] next_read = read_strobe ? read_ptr + 1 : read_ptr;
	wire [BITS-1:0] next_write = write_strobe ? write_ptr + 1 : write_ptr;
	wire [BITS-1:0] next_count =
		read_strobe && !write_strobe ? count - 1 :
		!read_strobe && write_strobe ? count + 1 :
		count;

	//reg space_available;
	//reg data_available;
	//assign data_available = read_strobe ? 0 : count != 0;
	assign data_available = next_count != 0;
	assign space_available = (NUM - 1'b1) - next_count > FREESPACE;

	//assign read_data = write_strobe && data_available ? write_data : buffer[read_ptr];
	assign read_data = buffer[read_ptr];
		//count == 0 && next_count == 0 ? write_data : buffer[read_ptr];
	//reg [WIDTH-1:0] read_data;

	always @(posedge clk) begin
		if (reset) begin
			write_ptr <= 0;
			read_ptr <= 0;
			werror <= 0;
			rerror <= 0;
			count <= 0;
			//space_available <= 0;
			//data_available <= 0;
		end else begin
			$display("strobe=%b %b data=%02x count=%d next_count=%d buf=[%02x,%02x,%02x,%02x]", write_strobe, read_strobe, read_data, count, next_count, buffer[0], buffer[1], buffer[2], buffer[3]);
			if (write_strobe)
				buffer[write_ptr] <= write_data;
/*
			if (write_strobe && next_count == 1)
			begin
				$display("fast forward write");
				read_data <= write_data;
			end else
				read_data <= buffer[read_ptr];
*/

			// if the fifo is empty, allow the write data to flow directly
			// to the read data to avoid a clock cycle in the fifo
			//if (write_strobe && read_ptr == write_ptr)
				//read_data <= write_data;
			//else
				//read_data <= buffer[read_ptr];
			

			read_ptr <= next_read;
			write_ptr <= next_write;
			count <= next_count;

			//data_available <= next_count != 0;

			//read_data <= buffer[next_read];
		end
	end
`endif
endmodule


`endif
