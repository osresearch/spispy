/** \file
 * SPI flash tool.
 *
 * This reads/writes interactively to a SPI flash attached to the FPGA.
 *
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"
`include "spi_controller.v"

module top(
	input clk_100mhz,
	//inout [7:0] pmod1, // serial port
	output pmod1_0,
	input pmod1_1,
	output pmod1_2,
	output pmod1_3,
	output pmod1_4,
	inout [7:0] pmod2, // spi flash clip
);
	// beaglewire pinouts
	wire serial_txd_pin	= pmod1_0; // pmod1[0];
	wire serial_rxd_pin	= pmod1_1; // pmod1[1];

	wire spi_clk_pin	= pmod2[0];
	wire spi_cs_pin		= pmod2[1];
	wire spi_miso_pin	= pmod2[2];
	wire spi_mosi_pin	= pmod2[3];

	wire locked, clk_96mhz, clk;
	wire reset = !locked;
	pll_96 pll(clk_100mhz, clk_96mhz, locked);
	assign clk = clk_96mhz;

	wire serial_txd;
	wire serial_rxd;

	gpio gpio_txd(
		.enable(1), // always on
		.pin(serial_txd_pin),
		.out(serial_txd),
	);

	gpio #(.PULLUP(1)) gpio_rxd(
		.enable(0), // always input, with pullup
		.pin(serial_rxd_pin),
		.in(serial_rxd),
		.out(1),
	);


	// generate a 3 MHz/12 MHz serial clock from the 96 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	// note that some Linux tools can have trouble keeping up with
	// this data rate; xxd -g1 for instance will drop bytes on
	// long bursts (more than 1KB).
	wire clk_3mhz, clk_12mhz;
	// 1 megabaud
	divide_by_n #(.N(24)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(96)) div4(clk, reset, clk_3mhz);

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

`define UART_FIFO
`ifdef UART_FIFO
	uart_tx_fifo #(
		.NUM(512),
		.FREESPACE(15'h32), // ensure there is always space
	)
`else
	uart_tx
`endif
	txd (
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready)
	);

	uart_rx rxd(
		.clk(clk),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	// SPI flash

	// if we're faking the !CS pin (driving the output high)
	// tell our SPI device that it is still selected.
	reg spi_cs_enable = 0;
	wire fake_spi_cs = spi_cs_enable ? 0 : spi_cs_in;

	wire spi_cs_out;
	wire spi_clk_out;
	wire spi_mosi_out;
	wire spi_miso_out = 0; // controlled by spi_device

	wire spi_cs_in;
	wire spi_clk_in;
	wire spi_mosi_in;
	wire spi_miso_in;

	gpio gpio_spi_cs(
		.enable(1), // always an output
		.pin(spi_cs_pin),
		.in(spi_cs_in),
		.out(spi_cs_out),
	);

	gpio gpio_spi_clk(
		.enable(1), // always output
		.pin(spi_clk_pin),
		.in(spi_clk_in),
		.out(spi_clk_out),
	);

	gpio gpio_spi_mosi(
		.enable(1), // always output
		.pin(spi_mosi_pin),
		.in(spi_mosi_in),
		.out(spi_mosi_out),
	);

	gpio gpio_spi_miso(
		.enable(0), // always input
		.pin(spi_miso_pin),
		.in(spi_miso_in),
		.out(spi_miso_out),
	);

	reg pmod1_2, pmod1_3, pmod1_4;
	always @(posedge clk) begin
		pmod1_2 <= spi_rx_strobe; //spi_cs_in;
		pmod1_3 <= spi_clk_in;
		pmod1_4 <= spi_mosi_in;
	end
	//assign pmod1_2 = spi_cs_in;
	//assign pmod1_3 = spi_clk_in;

	wire spi_rx_strobe;
	wire [7:0] spi_rx_data;
	reg [7:0] spi_tx_data = 0;
	reg spi_tx_strobe = 0;
	wire spi_tx_ready;
	reg spi_enable = 0;

	spi_controller #(.CLOCK_DIV(128)) spi(
		.clk(clk),
		.reset(reset),
		.spi_enable(spi_enable),
		.spi_clk(spi_clk_out),
		.spi_cs(spi_cs_out),
		.spi_miso(spi_miso_in),
		.spi_mosi(spi_mosi_out),
		.spi_rx_strobe(spi_rx_strobe),
		.spi_rx_data(spi_rx_data),
		.spi_tx_ready(spi_tx_ready),
		.spi_tx_strobe(spi_tx_strobe),
		.spi_tx_data(spi_tx_data),
	);

	reg [3:0] rxd_bits;
	reg [7:0] bytes;

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		spi_tx_strobe <= 0;

		if (reset) begin
			spi_enable <= 0;
			bytes <= 0;
		end else
		if (!spi_enable && uart_rxd_strobe && !uart_txd_strobe) begin
			// send another id command
			bytes <= 0;
			spi_enable <= 1;
			spi_tx_data <= 8'h9F;
			spi_tx_strobe <= 1;
			uart_txd <= "!";
			uart_txd_strobe <= 1;
		end else begin
			if (spi_rx_strobe) begin
				rxd_bits <= 2;
				//uart_txd <= "R";
				//uart_txd_strobe <= 1;
			end else
			if (rxd_bits == 2 && uart_txd_ready) begin
				uart_txd <= hexdigit(spi_rx_data[7:4]);
				uart_txd_strobe <= 1;
				rxd_bits = 1;
			end else
			if (rxd_bits == 1 && uart_txd_ready) begin
				uart_txd <= hexdigit(spi_rx_data[3:0]);
				uart_txd_strobe <= 1;
				rxd_bits = 0;
			end

			if (spi_tx_ready) begin
				spi_tx_strobe <= 1;
				spi_tx_data <= bytes;
				bytes <= bytes + 1;
				if (bytes == 3)
					spi_enable <= 0;
			end
		end
	end

endmodule
