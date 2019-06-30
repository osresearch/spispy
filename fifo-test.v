/** \file
 * Test fifo
 *
 */
`default_nettype none
`include "util.v"
`include "fifo.v"

module top();
	reg clk;
	reg reset;

	initial begin
		clk = 0;
		reset = 1;
		repeat(4) #5 clk = ~clk;
		reset = 0;
		forever #5 clk = ~clk;
	end

	wire space_available, data_available;
	wire [7:0] read_data;
	reg [7:0] write_data;
	reg read_strobe;
	reg write_strobe;

	wire [7:0] write_ptr;
	wire [7:0] read_ptr;
	wire [7:0] fifo_count;

	fifo #(.NUM(4)) fifo_i(
		.clk(clk),
		.reset(reset),
		.space_available(space_available),
		.write_data(write_data),
		.write_strobe(write_strobe),
		.data_available(data_available),
		.read_data(read_data),
		.read_strobe(read_strobe),
		// debug port
		.write_ptr(write_ptr),
		.read_ptr(read_ptr),
		.count(fifo_count)
	);
	
	always @(posedge clk)
	begin

		#10000
		$finish;
	end

	reg [7:0] counter;
	always @(posedge clk)
	begin
		write_strobe <= 0;

		if (reset) begin
			counter <= 8'h0f;
			write_data <= 0;
		end else begin
			counter <= counter + 1;
			if (counter[3:0] < counter[7:4])
			begin
				if (space_available)
				begin
					$display("%02x: writing %02x", counter, write_data+1);
					write_strobe <= 1;
					write_data <= write_data + 1;
				end else begin
					$display("STALL");
					counter <= counter;
				end
			end
		end
	end


	always @(posedge clk)
	begin
		read_strobe <= 0;

		if (reset) begin
		end else
		if (data_available)
		begin
			$display("read: %02x", read_data);
			read_strobe <= 1;
		end
	end

	always
	begin
		#5
		$display("%b write(%b,%02x) read(%b,%02x) avail=%b space=%b %02x %02x %02x",
			clk,
			write_strobe, write_data,
			read_strobe, read_data,
			data_available, space_available,
			write_ptr, read_ptr, fifo_count
		);
	end
endmodule
