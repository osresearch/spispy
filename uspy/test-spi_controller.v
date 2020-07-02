/*
 * Test the SPI device controller
 *
 */
`default_nettype none
`include "spi_controller.v"
`include "util.v"

module top();
	reg clk;
	reg reset;

	initial begin
		$dumpfile("test-spi_controller.vcd");
		$dumpvars(0,top);
		clk = 0;
		reset = 1;
		repeat(4) #5 clk = ~clk;
		reset = 0;
		forever #5 clk = ~clk;
	end

	always @(posedge clk)
	begin

		#10000
		$finish;
	end

	wire spi_clk;
	reg spi_cs = 1;

	reg [3:0] spi_data = 4'b0000;
	wire [3:0] spi_data_out;

	wire [7:0] spi_byte_rx;
	reg [7:0] spi_byte;
	reg [7:0] spi_byte_tx = 0;
	reg [2:0] spi_mode = 1;
	reg spi_byte_tx_strobe = 0;
	wire spi_idle;

	spi_controller spi_controller_i(
		.clk(clk),
		.reset(reset),
		// physical
		.spi_clk(spi_clk),
		.spi_data_in(spi_data),
		.spi_data_out(spi_data_out),
		.spi_mode_in(spi_mode),
		// logical
		.spi_byte_tx_strobe(spi_byte_tx_strobe),
		.spi_byte_tx(spi_byte_tx),
		.spi_byte_rx(spi_byte_rx),
		.spi_idle(spi_idle)
	);

	reg started = 0;

	always @(posedge clk)
	begin
		spi_byte_tx_strobe <= 0;
		if (reset) begin
			started <= 0;
		end else
		if (spi_idle && !spi_byte_tx_strobe)
		begin
			$display("rx=%02x", spi_byte_rx);
			spi_byte_tx <= spi_byte_tx + 1;
			spi_byte_tx_strobe <= 1;
		end else
		if (!started)
		begin
			started <= 1;
			spi_cs <= 0;
			spi_byte_tx <= 8'hA5;
			spi_byte_tx_strobe <= 1;
		end
	end

	always
	begin
		#1000

		$finish;
	end
endmodule
