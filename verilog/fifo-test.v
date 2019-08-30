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
		$dumpfile("fifo-test.vcd");
		$dumpvars(0,top);
		clk = 0;
		reset = 1;
		repeat(4) #5 clk = ~clk;
		reset = 0;
		forever #5 clk = ~clk;
	end

	wire space_available;
	wire data_available;
	wire more_available;
	wire [7:0] read_data;
	reg [7:0] write_data;
	wire read_strobe;
	reg write_strobe;

	fifo #(.NUM(16), .FREESPACE(2)) fifo_i(
		.clk(clk),
		.reset(reset),
		.space_available(space_available),
		.write_data(write_data),
		.write_strobe(write_strobe),
		.data_available(data_available),
		.more_available(more_available),
		.read_data(read_data),
		.read_strobe(read_strobe)
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
			write_data <= 8'hA0 - 1;
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

	reg [7:0] expected;
	reg [1:0] spin;

	assign read_strobe = data_available && spin == 0;

	always @(posedge clk)
	begin
		if (reset) begin
			expected <= 8'hA0;
			spin <= 0;
		end else
		if (spin) begin
			// burn some cycles
			spin <= spin - 1;
		end else
		if (data_available)
		begin
			if (read_data != expected)
				$display("BAD read: %02x != expected %02x", read_data, expected);
			else
				$display("read: %02x", read_data);
			expected <= expected + 1;

			if (expected[1:0] == 2'b11)
				spin <= ~0;
		end
	end

	always
	begin
		#5
		$display("%b write(%b,%02x) read(%b,%02x) avail=%b more=%b space=%b",
			clk,
			write_strobe, write_data,
			read_strobe, read_data,
			data_available, more_available, space_available
		);
	end
endmodule
