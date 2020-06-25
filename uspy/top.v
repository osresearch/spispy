/*
 * Usual wiring is:
 *
 *               +------+
 *  !CS       ---| o    |----  +V
 *   DO / D1  ---|      |---- !RST / D3 
 *  !WP / D2  ---|      |----  CLK
 *  GND       ---|      |----  DI / D0
 *               +------+
// teensy3 pins
#define SPI_CS   10 // white or yellow
#define SPI_SCLK 13 // green
#define SPI_MOSI 11 // blue or purple
#define SPI_MISO 12 // brown
 */
`default_nettype none
`include "uart.v"
`include "util.v"
`include "uspispy.v"

module top(
	output serial_txd,
	input serial_rxd,
	output fpga_spi_cs,
	output led_r,
	output led_g,
	output led_b,
	// left side GPIO
	inout gpio_23,
	inout gpio_25, // D0 / DI
	inout gpio_26, // CLK
	inout gpio_27, // D3
	inout gpio_32, // D2
	//inout gpio_35,
	//inout gpio_31, // !CS
	inout gpio_37, // D1 / DO
	inout gpio_34, // !CS ram0
	inout gpio_43, // D1 ram0
	inout gpio_36, // D2 ram0
	inout gpio_42, // !CS ram1
	inout gpio_38, // D1 ram1
	inout gpio_28, // D2 ram1

	// right side GPIO
	inout gpio_12,
	inout gpio_21,
	inout gpio_13,
	inout gpio_19,
	inout gpio_18, // !CS
	inout gpio_11,
	inout gpio_9,
	inout gpio_6,
	inout gpio_44,
	inout gpio_4,
	inout gpio_3, // D3 ram0
	inout gpio_48, // clk ram0
	inout gpio_45, // D0 ram0
	inout gpio_47, // D3 ram1
	inout gpio_46, // clk ram1
	inout gpio_2, // D0 ram1
);
	assign fpga_spi_cs = 1;

	wire clk_48mhz;
	SB_HFOSC u_hfosc(
		.CLKHFPU(1),
		.CLKHFEN(1),
		.CLKHF(clk_48mhz),
	);

/*
	wire locked;
	wire clk;
	wire reset = !locked;
	pll_120 pll(clk_48mhz, clk, locked);
*/
	wire clk = clk_48mhz;
	wire reset = 0;

	assign led_b = serial_rxd;
	assign led_g = serial_txd;
	assign led_r = spi_cs_in; // negative logic

	// divide clk60 by 20 to get a 3 MHz baud clock
	wire clk_3;
	divide_by_n #(16) baud_clock(clk, reset, clk_3);
	wire [7:0] uart_tx;
	wire uart_tx_strobe;
	
	uart_tx_fifo serial_tx_fifo(
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3),
		.data(uart_tx),
		.data_strobe(uart_tx_strobe),
		.serial(serial_txd),
	);

	// physical pins for the three SPI ports
	wire spi_clk_pin = gpio_26;
	wire spi_cs_pin = gpio_18;
	wire [3:0] spi_data_pins = { gpio_27, gpio_32, gpio_37, gpio_25 };

	wire ram0_clk_pin = gpio_48;
	wire ram0_cs_pin = gpio_34;
	wire [3:0] ram0_data_pins = { gpio_3, gpio_36, gpio_43, gpio_45 };

	wire ram1_clk_pin = gpio_46;
	wire ram1_cs_pin = gpio_42;
	wire [3:0] ram1_data_pins = { gpio_47, gpio_28, gpio_38, gpio_2 };

	// bidirectional pins for the spi bus data
	wire [3:0] spi_di;
	wire [3:0] spi_do;
	wire [3:0] spi_do_enable;
	wire spi_cs_enable;
	wire spi_cs_in;
	wire spi_cs_out;
	wire spi_clk;

	SB_IO #(
		.PIN_TYPE(6'b1010_01), // tristatable outputs
		//.PULLUP(4'b1111),
	) spi_data_buffer[3:0] (
		.OUTPUT_ENABLE(spi_do_enable),
		.PACKAGE_PIN(spi_data_pins),
		.D_IN_0(spi_di),
		.D_OUT_0(spi_do),
	);

	SB_IO #(
		.PIN_TYPE(6'b1010_01), // tristatable outputs
		//.PULLUP(1'b1),
	) spi_cs_buffer (
		.OUTPUT_ENABLE(spi_cs_enable),
		.PACKAGE_PIN(spi_cs_pin),
		.D_IN_0(spi_cs_in),
		.D_OUT_0(spi_cs_out),
	);

	// unidirectional for the clk pin
	SB_IO #(
		.PIN_TYPE(6'b1010_01),
	) spi_clk_buffer(
		.PACKAGE_PIN(spi_clk_pin),
		.OUTPUT_ENABLE(0),
		.D_IN_0(spi_clk),
	);


	// bidirectional pins for the ram0 data
	wire [3:0] ram0_di;
	wire [3:0] ram0_do;
	wire [3:0] ram0_do_enable;
	SB_IO #(
		.PIN_TYPE(6'b1010_01) // tristatable outputs
	) ram0_data_buffer[3:0] (
		.OUTPUT_ENABLE(ram0_do_enable),
		.PACKAGE_PIN(ram0_data_pins),
		.D_IN_0(ram0_di),
		.D_OUT_0(ram0_do),
	);

	// bidirectional pins for the ram1 data
	wire [3:0] ram1_di;
	wire [3:0] ram1_do;
	wire [3:0] ram1_do_enable;
	SB_IO #(
		.PIN_TYPE(6'b1010_01) // tristatable outputs
	) ram1_data_buffer[3:0] (
		.OUTPUT_ENABLE(ram1_do_enable),
		.PACKAGE_PIN(ram1_data_pins),
		.D_IN_0(ram1_di),
		.D_OUT_0(ram1_do),
	);

	uspispy uspi(
		.clk(clk),
		.reset(reset),
		.uart_tx(uart_tx),
		.uart_tx_strobe(uart_tx_strobe),
		.spi_clk(spi_clk),
		.spi_cs_in(spi_cs_in),
		.spi_cs_out(spi_cs_out),
		.spi_cs_enable(spi_cs_enable),
		.spi_di(spi_di),
		.spi_do(spi_do),
		.spi_do_enable(spi_do_enable),
		.ram0_clk(ram0_clk_pin),
		.ram0_cs(ram0_cs_pin),
		.ram0_di(ram0_di),
		.ram0_do(ram0_do),
		.ram0_do_enable(ram0_do_enable),
		.ram1_clk(ram1_clk_pin),
		.ram1_cs(ram1_cs_pin),
		.ram1_di(ram1_di),
		.ram1_do(ram1_do),
		.ram1_do_enable(ram1_do_enable),
	);

endmodule

