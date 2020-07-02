/*
 * Test the SPI device controller with iomem interface
 *
 */
`default_nettype none
`include "spi_controller_iomem.v"
`include "util.v"

module top();
	reg clk;
	reg reset;

	initial begin
		$dumpfile("test-spi_controller_iomem.vcd");
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
	wire spi_cs;

	reg [3:0] spi_data = 4'b0000;
	wire [3:0] spi_data_out;
	wire [3:0] spi_data_enable;

	reg sel = 0;
	wire [7:0] addr = 0;
	wire [31:0] rdata;
	reg [31:0] wdata = 32'h0000_12_A5;
	reg [3:0] wstrb = 4'b1111;

	spi_controller_iomem spi_controller_i(
		.clk(clk),
		.reset(reset),
		// physical
		.spi_cs(spi_cs),
		.spi_clk(spi_clk),
		.spi_data_in(spi_data),
		.spi_data_out(spi_data_out),
		.spi_data_enable(spi_data_enable),
		// logical
		.sel(sel),
		.addr(8'h00),
		.rdata(rdata),
		.wstrb(wstrb),
		.wdata(wdata)
	);

	reg started = 0;

	always
	begin
		#105
		wdata = 32'h0000_12_A5;
		wstrb = 4'b1111;
		sel <= 1;
		#10
		sel <= 0;

		#180
		wdata <= 32'h0000_4F_5A;
		sel <= 1;
		#10
		sel <= 0;

		#100
		wdata <= 32'h0000_92_00;
		wstrb <= 4'b0010;
		sel <= 1;
		#10
		sel <= 0;

		#100

		$finish;
	end
endmodule
