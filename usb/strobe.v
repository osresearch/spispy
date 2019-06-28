`ifndef strobe_v
`define strobe_v

module strobe(
	input clk_in,
	input clk_out,
	input strobe_in,
	output strobe_out,
	input [WIDTH-1:0] data_in,
	output [WIDTH-1:0] data_out
);
	parameter WIDTH = 1;
	parameter DELAY = 2; // 2 for metastability, larger for testing

`define CLOCK_CROSS
`ifdef CLOCK_CROSS
	reg flag;
	reg prev_strobe;
	reg [DELAY:0] sync;
	reg [WIDTH-1:0] data;

	// flip the flag and clock in the data when strobe is high
	always @(posedge clk_in) begin
		//if ((strobe_in && !prev_strobe)
		//|| (!strobe_in &&  prev_strobe))
		flag <= flag ^ strobe_in;

		if (strobe_in)
			data <= data_in;

		prev_strobe <= strobe_in;
	end

	// shift through a chain of flipflop to ensure stability
	always @(posedge clk_out)
		sync <= { sync[DELAY-1:0], flag };

	assign strobe_out = sync[DELAY] ^ sync[DELAY-1];
	assign data_out = data;
`else
	assign strobe_out = strobe_in;
	assign data_out = data_in;
`endif
endmodule


module dflip(
	input clk,
	input in,
	output out
);
	reg [2:0] d;
	always @(posedge clk)
		d <= { d[1:0], in };
	assign out = d[2];
endmodule


module delay(
	input clk,
	input in,
	output out
);
	parameter DELAY = 1;

	generate
	if (DELAY == 0) begin
		assign out = in;
	end else
	if (DELAY == 1) begin
		reg buffer;
		always @(posedge clk)
			buffer <= in;
		assign out = buffer;
	end else begin
		reg [DELAY-1:0] buffer;
		always @(posedge clk)
			buffer <= { buffer[DELAY-2:0], in };
		assign out = buffer[DELAY-1];
	end
	endgenerate
endmodule

`endif

