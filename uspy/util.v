/** \file
 * Utility modules.
 */
`ifndef util_v
`define util_v

`define CLOG2(x) \
   x <= 2	 ? 1 : \
   x <= 4	 ? 2 : \
   x <= 8	 ? 3 : \
   x <= 16	 ? 4 : \
   x <= 32	 ? 5 : \
   x <= 64	 ? 6 : \
   x <= 128	 ? 7 : \
   x <= 256	 ? 8 : \
   x <= 512	 ? 9 : \
   x <= 1024	 ? 10 : \
   x <= 2048	 ? 11 : \
   x <= 4096	 ? 12 : \
   x <= 8192	 ? 13 : \
   x <= 16384	 ? 14 : \
   x <= 32768	 ? 15 : \
   x <= 65536	 ? 16 : \
   x <= 131072	 ? 18 : \
   x <= 262144	 ? 18 : \
   -1

`define hexdigit(x) ( \
	x == 0 ? "0" : \
	x == 1 ? "1" : \
	x == 2 ? "2" : \
	x == 3 ? "3" : \
	x == 4 ? "4" : \
	x == 5 ? "5" : \
	x == 6 ? "6" : \
	x == 7 ? "7" : \
	x == 8 ? "8" : \
	x == 9 ? "9" : \
	x == 10 ? "a" : \
	x == 11 ? "b" : \
	x == 12 ? "c" : \
	x == 13 ? "d" : \
	x == 14 ? "e" : \
	x == 15 ? "f" : \
	"?" )

module divide_by_n(
	input clk,
	input reset,
	output reg out
);
	parameter N = 2;

	reg [`CLOG2(N)-1:0] counter;

	always @(posedge clk)
	begin
		out <= 0;

		if (reset)
			counter <= 0;
		else
		if (counter == 0)
		begin
			out <= 1;
			counter <= N - 1;
		end else
			counter <= counter - 1;
	end
endmodule

module pwm(
	input clk,
	input [BITS-1:0] bright,
	output out
);
	parameter BITS = 8;

	reg [BITS-1:0] counter;
	assign out = counter < bright;

	always @(posedge clk)
		counter <= counter + 1;

endmodule


/************************************************************************
 *
 * Random utility modules.
 *
 * Micah Dowty <micah@navi.cx>
 *
 ************************************************************************/


module d_flipflop(clk, reset, d_in, d_out);
   input clk, reset, d_in;
   output d_out;

   reg    d_out;

   always @(posedge clk or posedge reset)
     if (reset) begin
         d_out   <= 0;
     end
     else begin
         d_out   <= d_in;
     end
endmodule


module d_flipflop_pair(clk, reset, d_in, d_out);
   input  clk, reset, d_in;
   output d_out;
   wire   intermediate;

   d_flipflop dff1(clk, reset, d_in, intermediate);
   d_flipflop dff2(clk, reset, intermediate, d_out);
endmodule


/*
 * A set/reset flipflop which is set on sync_set and reset by sync_reset.
 */
module set_reset_flipflop(clk, reset, sync_set, sync_reset, out);
   input clk, reset, sync_set, sync_reset;
   output out;
   reg    out;

   always @(posedge clk or posedge reset)
     if (reset)
       out   <= 0;
     else if (sync_set)
       out   <= 1;
     else if (sync_reset)
       out   <= 0;
endmodule


/*
 * Pulse stretcher.
 *
 * When the input goes high, the output goes high
 * for as long as the input is high, or as long as
 * it takes our timer to roll over- whichever is
 * longer.
 */
module pulse_stretcher(clk, reset, in, out);
   parameter BITS = 20;

   input  clk, reset, in;
   output out;
   reg    out;

   reg [BITS-1:0] counter;

   always @(posedge clk or posedge reset)
     if (reset) begin
        out <= 0;
        counter <= 0;
     end
     else if (counter == 0) begin
        out <= in;
        counter <= in ? 1 : 0;
     end
     else if (&counter) begin
        if (in) begin
           out <= 1;
        end
        else begin
           out <= 0;
           counter <= 0;
        end
     end
     else begin
        out <= 1;
        counter <= counter + 1;
     end
endmodule


/*
 * Clock crossing strobe.
 *
 * The input should switch polarity to signal changes,
 * which will be translated into single clock strobes in the clk domain.
 */
module strobe_sync(
	input clk,
	input flop,
	output strobe
);
	parameter DELAY = 1;
	reg [DELAY:0] sync;
	assign strobe = sync[DELAY] != sync[DELAY-1];

	always @(posedge clk)
		sync <= { sync[DELAY-1:0], flop };
endmodule


module strobe2strobe(
	input clk_a,
	input strobe_a,
	input clk_b,
	output strobe_b
);
	reg flag_a = 0;
	always @(posedge clk_a)
		flag_a <= strobe_a ^ flag_a;

	strobe_sync sync(clk_b, flag_a, strobe_b);
endmodule


`endif
