/*
 * Usual wiring is:
 *
 *               +------+
 *  !CS       ---| o    |----  +V
 *   DO / D1  ---|      |----  D3 / !RST
 *  !WP / D2  ---|      |----  CLK
 *  GND       ---|      |----  D0 / DI
 *               +------+
// teensy3 pins
#define SPI_CS   10 // white or yellow
#define SPI_MOSI 11 // blue or purple -- to DI / D0
#define SPI_MISO 12 // brown --- from DO / D1
#define SPI_SCLK 13 // green
 */
`default_nettype none



`include "util.v"
`include "uspispy.v"

`ifdef FPGA_ice40
`include "pll_16.v"
`define PICOSOC_MEM ice40up5k_spram
`define PICOSOC_BRAM "firmware.syn.hex"
`elsif FPGA_ecp5
`include "pll_30.v"
`define PICOSOC_MEM picosoc_mem
`define PICOSOC_BRAM "firmware.syn.hex"
`endif

`include "picorv32/ice40up5k_spram.v"
//`include "picorv32/ecp5_spram.v"
`include "picorv32/simpleuart.v"
`include "picorv32/picosoc.v"
`include "picorv32/picorv32.v"
`include "spi_controller.v"
`include "spi_controller_iomem.v"

module top(
`ifdef FPGA_ice40
	output serial_txd,
	input serial_rxd,
	output fpga_spi_cs,
	output led_r,
	output led_g,
	output led_b,

	// left side GPIO, all in bank 0
	// used for the DIO busses
	inout gpio_23, // SPI dio[0]
	inout gpio_25, // SPI dio[1]
	inout gpio_26, // SPI dio[2]
	inout gpio_27, // SPI dio[3]
	inout gpio_32, // RAM0 dio[0]
	//inout gpio_35, // DO NOT USE, special PLL interaction
	inout gpio_31, // not used, broken?
	inout gpio_37, // RAM0 dio[1] / DO
	inout gpio_34, // RAM0 dio[2]
	inout gpio_43, // RAM0 dio[3]
	inout gpio_36, // RAM1 dio[0]
	inout gpio_42, // RAM1 dio[1]
	inout gpio_38, // RAM1 dio[2]
	inout gpio_28, // RAM1 dio[3]

	// right side GPIO, bank 1
	// need to share VCC with FPGA flash, so can't be powered by SPI bus
	// could be used for another PSRAM used for logging.
	inout gpio_12,
	inout gpio_21,
	inout gpio_13,
	inout gpio_19,
	input gpio_18,
	inout gpio_11,
	inout gpio_9,
	inout gpio_6,

	// right side GPIO, bank 2
	// used for SPI_CLK and !CS pins
	inout gpio_44, // SPI clk (global buf 6 for high-fanout)
	inout gpio_4, // SPI !CS
	inout gpio_3, // RAM0 !CS
	inout gpio_48, // RAM0 CLK (to be adjacent to spi clk)
	inout gpio_45, // RAM1 CLK (to be adjacent to spi clk)
	inout gpio_47, // RAM1 !CS
	inout gpio_46, // unused
	inout gpio_2, // unused

`elsif FPGA_ecp5
	input clk_25mhz,
	output [7:0] led,
	inout [27:0] gp,
	inout [27:0] gn,
	input ftdi_txd,
	output ftdi_rxd
`endif
);

`ifdef FPGA_ice40
	assign fpga_spi_cs = 1;
	parameter integer MEM_WORDS = 32768;

	// ice40 has a 48 MHz onboard oscillator
	wire clk_48mhz;
	SB_HFOSC u_hfosc(
		.CLKHFPU(1),
		.CLKHFEN(1),
		.CLKHF(clk_48mhz),
	);

	wire locked;
	wire clk_16mhz;
	pll_16 pll(clk_48mhz, clk_16mhz, locked);
	wire clk = clk_16mhz;

	assign led_b = serial_rxd;
	assign led_g = spi_cs_in; // serial_txd;
	assign led_r = !gpio[7]; // spi_cs_in; // negative logic

	// physical pins for the three SPI ports
	// the clocks are all in the same gpio bank
	// the dio pins are all in the same bank
	wire spi_clk_pin = gpio_44; // global buf 6
	wire spi_cs_pin = gpio_4;
	wire [3:0] spi_data_pins = { gpio_27, gpio_26, gpio_25, gpio_23 };

	wire ram0_clk_pin = gpio_48;
	wire ram0_cs_pin = gpio_3;
	wire [3:0] ram0_data_pins = { gpio_43, gpio_34, gpio_37, gpio_32 };

	wire ram1_clk_pin = gpio_45;
	wire ram1_cs_pin = gpio_47;
	wire [3:0] ram1_data_pins = { gpio_28, gpio_38, gpio_42, gpio_36 };

`elsif FPGA_ecp5
	// the sdram could be used for this instead, but for now EBR it is
	parameter integer MEM_WORDS = 16384;

	// 25 MHz input clock from outside
	wire locked;
	wire clk_30mhz;
	pll_30 pll(clk_25mhz, clk_30mhz, locked);
	wire clk = clk_25mhz;

	assign led[0] = serial_rxd;
	assign led[1] = serial_txd;
	wire serial_rxd = ftdi_txd;
	wire serial_txd = ftdi_rxd;

	// physical pins for the three SPI ports
	wire spi_clk_pin = gp[0];
	wire spi_cs_pin = gp[1];
	wire [3:0] spi_data_pins = gp[5:2];

	wire ram0_clk_pin = gp[6];
	wire ram0_cs_pin = gp[7];
	wire [3:0] ram0_data_pins = gp[11:8];

	wire ram1_clk_pin = gp[12];
	wire ram1_cs_pin = gp[13];
	wire [3:0] ram1_data_pins = gp[17:14];

`endif

	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;
	wire reset = !resetn;

	always @(posedge clk) begin
/*
		if (!locked)
			reset_cnt <= 0;
		else
*/
			reset_cnt <= reset_cnt + !resetn;
	end

	// bidirectional pins for the spi bus data
	wire [3:0] spi_di;
	wire [3:0] spi_do;
	wire [3:0] spi_do_enable;
	wire spi_cs_enable;
	wire spi_cs_in;
	wire spi_cs_out;
	wire spi_clk_in;
	wire spi_clk_out = 0;
	wire spi_clk_enable = 0; // always input for now

	buffer #(.WIDTH(4)) spi_data_buffer(
		.enable(spi_do_enable),
		.pins(spi_data_pins),
		.in(spi_di),
		.out(spi_do)
	);

	buffer #(.PULLUP(1)) spi_cs_buffer (
		.enable(spi_cs_enable),
		.pins(spi_cs_pin),
		.in(spi_cs_in),
		.out(spi_cs_out),
	);

	buffer spi_clk_buffer(
		.pins(spi_clk_pin),
		.enable(spi_clk_enable),
		.in(spi_clk_in),
		.out(spi_clk_out),
	);


	// bidirectional pins for the ram0 data
	wire [3:0] ram0_di;
	reg [3:0] ram0_do;
	reg [3:0] ram0_do_enable;
	buffer #(.WIDTH(4)) ram0_data_buffer(
		.enable(ram0_do_enable),
		.pins(ram0_data_pins),
		.in(ram0_di),
		.out(ram0_do),
	);

	// ram0 cs and clk are always output
	reg ram0_cs_out;
	reg ram0_clk_out;
	buffer #(.WIDTH(2)) ram0_cs_buffer(
		.enable(2'b11),
		.pins({ram0_cs_pin, ram0_clk_pin}),
		.out({ram0_cs_out, ram0_clk_out}),
	);

	// bidirectional pins for the ram1 data
	wire [3:0] ram1_di;
	reg [3:0] ram1_do;
	reg [3:0] ram1_do_enable;
	buffer #(.WIDTH(4)) ram1_data_buffer(
		.enable(ram1_do_enable),
		.pins(ram1_data_pins),
		.in(ram1_di),
		.out(ram1_do),
	);

	// ram1 cs and clk are always output
	reg ram1_cs_out;
	reg ram1_clk_out;
	buffer #(.WIDTH(2)) ram1_cs_buffer(
		.enable(2'b11),
		.pins({ram1_cs_pin, ram1_clk_pin}),
		.out({ram1_cs_out, ram1_clk_out}),
	);


	// spi logical interface for logging and slow command emulation
	wire spi_cmd_strobe;
	wire [7:0] spi_cmd;
	wire [31:0] spi_addr;
	wire [11:0] spi_len;
	wire [7:0] spi_sr;
	reg [7:0] spi_sr_in = 0;
	reg spi_sr_strobe = 0;

	// memory buffer for spi write commands; stores 8-bits at a time in spi_clk domain
	// reads 32-bits at a time in clk domain
	wire spi_write_strobe;
	wire [7:0] spi_write_data;
	wire [7:0] spi_write_addr;
	reg [31:0] spi_write_buffer[0:63];
	reg [31:0] wbuf_rdata;
	reg wbuf_ready;

	always @(posedge spi_clk_in) begin
		// writes are byte at a time
		if (!spi_write_strobe) begin
			// nothing to do
		end else
		case(spi_write_addr[1:0])
		2'b00: spi_write_buffer[spi_write_addr[7:2]][ 7: 0] <= spi_write_data;
		2'b01: spi_write_buffer[spi_write_addr[7:2]][15: 8] <= spi_write_data;
		2'b10: spi_write_buffer[spi_write_addr[7:2]][23:16] <= spi_write_data;
		2'b11: spi_write_buffer[spi_write_addr[7:2]][31:24] <= spi_write_data;
		endcase
	end

	always @(posedge clk) begin
		// reads are 32-bits at a time and always ready (but not always valid...)
		wbuf_rdata <= spi_write_buffer[iomem_addr[7:2]];
		wbuf_ready <= wbuf_sel;
	end

	// record when the reception time of a spi command, in microseconds
	// clk == 16 MHz, so counter/16 == useconds
	reg [27:0] counter = 0;
	reg [23:0] spi_counter = 0;
	always @(posedge clk) begin
		counter <= counter + 1;
		if (spi_cmd_strobe)
			spi_counter <= counter[27:4];
	end

	wire [3:0] uspi_ram0_di = ram0_di;
	wire [3:0] uspi_ram0_do;
	wire [3:0] uspi_ram0_do_enable;
	wire       uspi_ram0_cs;
	wire       uspi_ram0_clk;

	wire [3:0] uspi_ram1_di = ram1_di;
	wire [3:0] uspi_ram1_do;
	wire [3:0] uspi_ram1_do_enable;
	wire       uspi_ram1_cs;
	wire       uspi_ram1_clk;

	uspispy uspi(
		.clk(clk),
		.reset(reset),

		// logical interfaces in clk domain
		.spi_cmd_strobe_out(spi_cmd_strobe),
		.spi_cmd_out(spi_cmd),
		.spi_addr_out(spi_addr),
		.spi_len_out(spi_len),
		.spi_sr(spi_sr),
		.spi_sr_in(spi_sr_in),
		.spi_sr_in_strobe(spi_sr_strobe),

		// memory buffer with write port in spi_clk domain
		.write_data(spi_write_data),
		.write_addr(spi_write_addr),
		.write_strobe(spi_write_strobe),

		// spi bus physical interface
		.spi_clk(spi_clk_in),
		.spi_cs_in(spi_cs_in),
		.spi_cs_out(spi_cs_out),
		.spi_cs_enable(spi_cs_enable),
		.spi_di(spi_di),
		.spi_do(spi_do),
		.spi_do_enable(spi_do_enable),

		// psram physical interfaces
		.ram0_clk(uspi_ram0_clk),
		.ram0_cs(uspi_ram0_cs),
		.ram0_di(uspi_ram0_di),
		.ram0_do(uspi_ram0_do),
		.ram0_do_enable(uspi_ram0_do_enable),

		.ram1_clk(uspi_ram1_clk),
		.ram1_cs(uspi_ram1_cs),
		.ram1_di(uspi_ram1_di),
		.ram1_do(uspi_ram1_do),
		.ram1_do_enable(uspi_ram1_do_enable),
	);

	wire spi0_sel;
	wire [31:0] spi0_rdata;
	wire spi0_ready;

	wire [1:0] spi_controller_sel;
	reg [3:0] spi_controller_di;
	wire [3:0] spi_controller_do;
	wire [3:0] spi_controller_do_enable;
	wire spi_controller_cs;
	wire spi_controller_clk;

	always @(*) begin
		spi_controller_di <= 4'bXXXX;

		if (spi_controller_sel == 2'b00) begin
			spi_controller_di <= ram0_di;
			ram0_clk_out	<= spi_controller_clk;
			ram0_cs_out	<= spi_controller_cs;
			ram0_do		<= spi_controller_do;
			ram0_do_enable	<= spi_controller_do_enable;
		end else begin
			ram0_clk_out	<= uspi_ram0_clk;
			ram0_cs_out	<= uspi_ram0_cs;
			ram0_do		<= uspi_ram0_do;
			ram0_do_enable	<= uspi_ram0_do_enable;
		end

		if (spi_controller_sel == 2'b01) begin
			spi_controller_di <= ram1_di;
			ram1_clk_out	<= spi_controller_clk;
			ram1_cs_out	<= spi_controller_cs;
			ram1_do		<= spi_controller_do;
			ram1_do_enable	<= spi_controller_do_enable;
		end else begin
			ram1_clk_out	<= uspi_ram1_clk;
			ram1_cs_out	<= uspi_ram1_cs;
			ram1_do		<= uspi_ram1_do;
			ram1_do_enable	<= uspi_ram1_do_enable;
		end
	end

	spi_controller_iomem spi_iomem(
		.clk(clk),
		.reset(reset),

		// physical
		.spi_data_in(spi_controller_di),
		.spi_data_out(spi_controller_do),
		.spi_data_enable(spi_controller_do_enable),
		.spi_cs(spi_controller_cs),
		.spi_clk(spi_controller_clk),
		.spi_sel(spi_controller_sel),

		// iomem logical
		.sel(spi0_sel),
		.addr(iomem_addr[7:0]),
		.wstrb(iomem_wstrb),
		.wdata(iomem_wdata),
		.rdata(spi0_rdata),
		.ready(spi0_ready),
	);

	wire gpio_sel;
	reg [31:0] gpio;
	wire gpio_ready = 1;
	wire [31:0] gpio_rdata = gpio;

	wire uspy_sel;
	reg uspy_ready;
	reg [31:0] uspy_rdata;

	wire gpio_sel = iomem_valid && iomem_addr[31:20] == 12'h030;
	wire uspy_sel = iomem_valid && iomem_addr[31:20] == 12'h040;
	wire wbuf_sel = iomem_valid && iomem_addr[31:20] == 12'h041;
	wire spi0_sel = iomem_valid && iomem_addr[31:20] == 12'h050;

	wire        iomem_valid;
	wire [ 3:0] iomem_wstrb;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	reg  [31:0] iomem_rdata;
	reg         iomem_ready;

	always @(*) begin
		if (gpio_sel) begin
			iomem_rdata <= gpio_rdata;
			iomem_ready <= gpio_ready;
		end else
		if (uspy_sel) begin
			iomem_rdata <= uspy_rdata;
			iomem_ready <= uspy_ready;
		end else
		if (wbuf_sel) begin
			iomem_rdata <= wbuf_rdata;
			iomem_ready <= wbuf_ready;
		end else
		if (spi0_sel) begin
			iomem_rdata <= spi0_rdata;
			iomem_ready <= spi0_ready;
		end else
		begin
			iomem_rdata <= 32'h00000000;
			iomem_ready <= 0;
		end
	end

	always @(posedge clk) begin
		uspy_ready <= 0;

		if (!resetn) begin
			gpio <= 0;
		end else
		if (gpio_sel) begin
			if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
			if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
			if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
			if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
		end else
		if (uspy_sel) begin
			spi_sr_strobe <= 0;
			uspy_ready <= 1;
			case(iomem_addr[7:0])
			8'h00: uspy_rdata <= { 4'h0, counter };
			8'h04: uspy_rdata <= { spi_counter, spi_cmd };
			8'h08: uspy_rdata <= { spi_addr };
			8'h0c: uspy_rdata <= { 20'h00000, spi_len };
			8'h10: begin
				uspy_rdata <= { 24'h000000, spi_sr };
				if (iomem_wstrb[0]) begin
					spi_sr_in <= iomem_wdata[7:0];
					spi_sr_strobe <= 1;
				end
			end
			default: uspy_rdata <= 32'hDECAFBAD;
			endcase
		end
	end

	picosoc #(
		.BARREL_SHIFTER(0),
		.ENABLE_MULDIV(0),
		.MEM_WORDS(MEM_WORDS)
	) soc (
		.clk          (clk         ),
		.resetn       (resetn      ),

		.ser_tx       (serial_txd  ),
		.ser_rx       (serial_rxd  ),

		.irq_5        (1'b0        ),
		.irq_6        (1'b0        ),
		.irq_7        (1'b0        ),

		.iomem_valid  (iomem_valid ),
		.iomem_ready  (iomem_ready ),
		.iomem_wstrb  (iomem_wstrb ),
		.iomem_addr   (iomem_addr  ),
		.iomem_wdata  (iomem_wdata ),
		.iomem_rdata  (iomem_rdata )
	);
endmodule

module buffer(
	input [WIDTH-1:0] enable,
	inout [WIDTH-1:0] pins,
	output [WIDTH-1:0] in,
	input [WIDTH-1:0] out
);
	parameter WIDTH = 1;
	parameter PULLUP = 0;

`ifdef FPGA_ice40
	SB_IO #(
		.PIN_TYPE(6'b1010_01), // tristatable outputs
		.PULLUP(PULLUP)
	) buffer[WIDTH-1:0](
		.OUTPUT_ENABLE(enable),
		.PACKAGE_PIN(pins),
		.D_IN_0(in),
		.D_OUT_0(out),
	);
`elsif FPGA_ecp5
	TRELLIS_IO #(
		.DIR("BIDIR")
	) buffer[WIDTH-1:0](
		.T(~enable),
		.B(pins),
		.I(out),
		.O(in)
	);
`else
`error "unknown fpga"
`endif
endmodule
