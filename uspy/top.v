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

`define PICOSOC_MEM ice40up5k_spram
`define PICOSOC_BRAM "firmware.syn.hex"


`include "util.v"
`include "uspispy.v"
`include "pll_16.v"
`include "picorv32/ice40up5k_spram.v"
`include "picorv32/simpleuart.v"
`include "picorv32/picosoc.v"
`include "picorv32/picorv32.v"
`include "spi_controller.v"
`include "spi_controller_iomem.v"

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
	parameter integer MEM_WORDS = 32768;

	wire clk_48mhz;
	SB_HFOSC u_hfosc(
		.CLKHFPU(1),
		.CLKHFEN(1),
		.CLKHF(clk_48mhz),
	);

	wire clk_16mhz;
	wire locked;
	pll_16 pll(clk_48mhz, clk_16mhz, locked);

	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;
	wire reset = !resetn;

	wire clk = clk_16mhz;
	always @(posedge clk_16mhz) begin
		if (!locked)
			reset_cnt <= 0;
		else
			reset_cnt <= reset_cnt + !resetn;
	end

	assign led_b = serial_rxd;
	assign led_g = !spi_cmd_strobe; // serial_txd;
	assign led_r = !gpio[7]; // spi_cs_in; // negative logic

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
	wire spi_clk_in;
	wire spi_clk_out = 0;
	wire spi_clk_enable = 0; // always input for now

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

	SB_IO #(
		.PIN_TYPE(6'b1010_01),
	) spi_clk_buffer(
		.PACKAGE_PIN(spi_clk_pin),
		.OUTPUT_ENABLE(spi_clk_enable),
		.D_IN_0(spi_clk_in),
		.D_OUT_0(spi_clk_out),
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

	// ram0 cs and clk are always output
	wire ram0_cs_out;
	wire ram0_clk_out;
	SB_IO #(
		.PIN_TYPE(6'b1010_01) // tristatable outputs
	) ram0_cs_buffer[1:0] (
		.OUTPUT_ENABLE(2'b11),
		.PACKAGE_PIN({ram0_cs_pin, ram0_clk_pin}),
		.D_OUT_0({ram0_cs_out, ram0_clk_out}),
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

	// ram1 cs and clk are always output
	wire ram1_cs_out;
	wire ram1_clk_out;
	SB_IO #(
		.PIN_TYPE(6'b1010_01) // tristatable outputs
	) ram1_cs_buffer[1:0] (
		.OUTPUT_ENABLE(2'b11),
		.PACKAGE_PIN({ram1_cs_pin, ram1_clk_pin}),
		.D_OUT_0({ram1_cs_out, ram1_clk_out}),
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
	reg [31:0] spi_write_buffer_read;

	always @(posedge spi_clk_in) begin
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
		spi_write_buffer_read <= spi_write_buffer[iomem_addr[7:0]];
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
	wire [3:0] uspi_ram1_di = ram0_di;
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
	wire [3:0] spi_controller_di;  //= spi_controller_sel[0] ? ram1_di : ram0_di;
	wire [3:0] spi_controller_do;  //= spi_controller_sel[0] ? ram1_do : ram0_do;
	wire [3:0] spi_controller_do_enable;  //= spi_controller_sel[0] ? ram1_do_enable : ram0_do_enable;
	wire spi_controller_cs; // = spi_controller_sel[0] ? ram1_cs_out : ram0_cs_out;
	wire spi_controller_clk;  //= spi_controller_sel[0] ? ram1_clk_out : ram0_clk_out;

/*
	always @(*)
	case(spi_controller_sel)
		2'b00: begin
			spi_controller_di = ram0_di;
			ram0_do = spi_controller_do;
			ram0_do_enable = spi_controller_do_enable;
			ram0_cs_out = spi_controller_cs;
			ram0_clk_out = spi_controller_clk;
		end
		2'b01: begin
			spi_controller_di = ram1_di;
			ram1_do = spi_controller_do;
			ram1_do_enable = spi_controller_do_enable;
			ram1_cs_out = spi_controller_cs;
			ram1_clk_out = spi_controller_clk;
		end
	endcase
*/
	assign spi_controller_di =
		spi_controller_sel == 2'b00 ? ram0_di :
		spi_controller_sel == 2'b01 ? ram1_di :
		4'bXXXX;

	assign ram0_cs_out = spi_controller_sel == 2'b00 ? spi_controller_cs : uspi_ram0_cs;
	assign ram1_cs_out = spi_controller_sel == 2'b01 ? spi_controller_cs : uspi_ram1_cs;

	assign ram0_clk_out = spi_controller_sel == 2'b00 ? spi_controller_clk : uspi_ram0_clk;
	assign ram1_clk_out = spi_controller_sel == 2'b01 ? spi_controller_clk : uspi_ram1_clk;

	assign ram0_do = spi_controller_sel == 2'b00 ? spi_controller_do : uspi_ram0_do;
	assign ram1_do = spi_controller_sel == 2'b01 ? spi_controller_do : uspi_ram1_do;

	assign ram0_do_enable = spi_controller_sel == 2'b00 ? spi_controller_do_enable : uspi_ram0_do_enable;
	assign ram1_do_enable = spi_controller_sel == 2'b01 ? spi_controller_do_enable : uspi_ram1_do_enable;

	//assign ram0_di = spi_controller_sel == 2'b00 ? spi_controller_do : uspi_ram0_do;

	spi_controller_iomem spi_iomem(
		.clk(clk),
		.reset(!resetn),
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
	wire spi0_sel = iomem_valid && iomem_addr[31:20] == 12'h050;

	wire        iomem_valid;
	wire [ 3:0] iomem_wstrb;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	wire [31:0] iomem_rdata =
		gpio_sel ? gpio_rdata :
		uspy_sel ? uspy_rdata :
		spi0_sel ? spi0_rdata :
		0;
	wire        iomem_ready =
		gpio_sel ? gpio_ready :
		uspy_sel ? uspy_ready :
		spi0_sel ? spi0_ready :
		0;


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

