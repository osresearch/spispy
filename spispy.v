/*
 * ECP5 flash emulator
 */
`default_nettype none
`include "uart.v"
`include "pll_132.v"
`include "sdram_ctrl.v"
`include "spi_device.v"

module top(
	input clk_25mhz,
	output [7:0] led,
	output wifi_gpio0,
	input ftdi_txd, // from the ftdi chip
	output ftdi_rxd, // to the ftdi chip

	// sdram physical interface
	output [12:0] sdram_a,
	inout [15:0] sdram_d,
	output [1:0] sdram_ba,
	output sdram_clk,
	output sdram_cke,
	output sdram_wen,
	output sdram_csn,
	output [1:0] sdram_dqm,
	output sdram_rasn,
	output sdram_casn,

	// GPIO pins, to be assigned
	inout [27:0] gp,
	inout [27:0] gn
);
	// gpio0 must be tied high to prevent board from rebooting
	assign wifi_gpio0 = 1;

	reg [7:0] led_reg;
	assign led = led_reg;

	// Generate a 132 MHz clock from the 25 MHz reference
	// with a 180 degree out of phase sdram clk
	wire clk_132, clk_132_180, locked, reset = !locked;
	pll_132 pll_132_i(clk_25mhz, clk_132, clk_132_180, locked);
	wire clk = clk_132;
	wire sdram_clk = clk_132_180; // sdram needs to be stable on the rising edge

	// SPI bus is on the J2 positive pins
	wire spi_cs_pin = gp[20];
	wire spi_clk_pin = gp[19];
	wire spi_miso_pin = gp[18];
	wire spi_mosi_pin = gp[17];

	reg spi_cs_enable = 0;
	wire spi_miso_out;
	wire spi_miso_in;
	wire spi_mosi_in;
	wire spi_clk_in;
	wire spi_cs_in;

	wire [7:0] spi_rx_data;
	wire spi_rx_cmd;
	wire spi_rx_strobe;

	TRELLIS_IO #(.DIR("BIDIR")) spi_cs_buf(
		.T(!spi_cs_enable),
		.B(spi_cs_pin),
		.I(1), // always high output
		.O(spi_cs_in),
	);
	TRELLIS_IO #(.DIR("BIDIR")) spi_miso_buf(
		.T(!spi_cs_enable),
		.B(spi_miso_pin),
		.I(spi_miso_out),
		.O(spi_miso_in),
	);
	TRELLIS_IO #(.DIR("INPUT")) spi_mosi_buf(
		.B(spi_mosi_pin),
		.O(spi_mosi_in),
	);
	TRELLIS_IO #(.DIR("INPUT")) spi_clk_buf(
		.B(spi_clk_pin),
		.O(spi_clk_in),
	);

	spi_device spi_i(
		.clk(clk),
		.reset(reset),
		// physical interface, in spi_clk domain
		.spi_cs(spi_cs_enable ? 0 : spi_cs_in),
		.spi_clk(spi_clk_in),
		.spi_miso(spi_miso_out),
		.spi_mosi(spi_mosi_in),
		// byte wise interface, in the local clk
		.spi_rx_data(spi_rx_data),
		.spi_rx_cmd(spi_rx_cmd),
		.spi_rx_strobe(spi_rx_strobe),
		.spi_tx_strobe(0),
		.spi_tx_strobe_immediate(0),
	);

	// oscilloscope debug pins also on jp2
	wire debug_0 = spi_miso_in;
	wire debug_1 = spi_clk_in;
	reg trigger;
	TRELLIS_IO #(.DIR("OUTPUT")) debug0(.B(gp[26]), .I(trigger));
	TRELLIS_IO #(.DIR("OUTPUT")) debug2(.B(gp[27]), .I(debug_0));
	TRELLIS_IO #(.DIR("OUTPUT")) debug3(.B(gp[25]), .I(debug_1));

	// serial port interface for talking to the host system
	// 132 MHz clock / 48 == 3 megabaud
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_rxd_strobe;
	wire [7:0] uart_rxd;

	uart #(
		.DIVISOR(44),
		.FIFO(512),
		.FREESPACE(16),
	) uart_i(
		.clk(clk),
		.reset(reset),
		// physical
		.serial_txd(ftdi_rxd), // fpga --> ftdi
		.serial_rxd(ftdi_txd), // fpga <-- ftdi
		// logical
		.txd(uart_txd),
		.txd_ready(uart_txd_ready),
		.txd_strobe(uart_txd_strobe),
		.rxd(uart_rxd),
		.rxd_strobe(uart_rxd_strobe),
	);

	// sdram logical interface has a 16-bit data interface
	parameter ADDR_WIDTH = 24;
	parameter DATA_WIDTH = 16;
	reg [ADDR_WIDTH-1:0] sd_addr;
	reg [DATA_WIDTH-1:0] sd_wr_data;
	wire [DATA_WIDTH-1:0] sd_rd_data;
	wire [DATA_WIDTH-1:0] sd_rd_data_raw;
	wire sd_rd_ready_raw;
	reg sd_we;
	reg sd_enable;

	wire sd_ack;
	wire sd_idle;

	//assign sdram_clk = clk;
wire	[15:0]	sdram_dq_i;
wire	[15:0]	sdram_dq_o;
wire		sdram_dq_oe;

	// generate an sdram reset controller
	reg sd_refresh_inhibit;
	reg sd_pause_read = 0;
	reg [15:0] sdram_reset_counter;
	wire sdram_reset = sdram_reset_counter != 0;
	always @(posedge clk or posedge reset)
		if (reset)
			sdram_reset_counter <= ~0;
		else
		if (sdram_reset)
			sdram_reset_counter <= sdram_reset_counter - 1;

	genvar i;
	generate
	for(i=0 ; i < DATA_WIDTH ; i=i+1)
		TRELLIS_IO #(.DIR("BIDIR")) sdram_d_buf(
			.T(!sdram_dq_oe),
			.B(sdram_d[i]),
			.I(sdram_dq_o[i]),
			.O(sdram_dq_i[i]),
		);
	endgenerate

////////////////////////////////////////////////////////////////////////
//
// SDRAM Memory Controller
//
////////////////////////////////////////////////////////////////////////

sdram_ctrl #(
	.CLK_FREQ_MHZ			(132),	// sdram_clk freq in MHZ
	.POWERUP_DELAY			(200),	// power up delay in us
	.REFRESH_MS			(32),	// delay between refresh cycles im ms
	.BURST_LENGTH			(1),	// 1 read at a time
	.ROW_WIDTH			(13),	// Row width
	.COL_WIDTH			(9),	// Column width
	.BA_WIDTH			(2),	// Ba width
	.tCAC				(2),	// CAS Latency
	.tRAC				(5),	// RAS Latency
	.tRP				(2),	// Command Period (PRE to ACT)
	.tRC				(7),	// Command Period (REF to REF / ACT to ACT)
	.tMRD				(2)	// Mode Register Set To Command Delay time
)
sdram_ctrl0 (
	// External SDRAM interface
	.ba_o		(sdram_ba),
	.a_o		(sdram_a),
	.cs_n_o		(sdram_csn),
	.ras_o		(sdram_rasn),
	.cas_o		(sdram_casn),
	.we_o		(sdram_wen),
	.dq_i		(sdram_dq_i),
	.dq_o		(sdram_dq_o),
	.dqm_o		(sdram_dqm),
	.dq_oe_o	(sdram_dq_oe),
	.cke_o		(sdram_cke),
	.sdram_clk	(clk), // sdram_clk is the output to the chip
	.sdram_rst	(sdram_reset),

	// logical interface
	.idle_o		(sd_idle),
	.adr_i		(sd_addr),
	.dat_i		(sd_wr_data),
	.dat_raw	(sd_rd_data),
	.sel_i		(2'b11), // always do both bytes
	.acc_i		(sd_enable),
	.ack_raw	(sd_ack),
	.we_i		(sd_we),
	.refresh_inhibit_i(sd_refresh_inhibit),
	.pause_read_i	(sd_pause_read)
);

	reg [24:0] wr_addr;
	reg [24:0] rd_addr;
	reg [4:0] rd_pending;

	reg [10:0] rd_timer;
	reg [7:0] count;
	reg spi_log_this;

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		rd_timer <= rd_timer + 1;
		if (spi_cs_in)
			trigger <= 0;

		if (reset || sdram_reset) begin
			wr_addr <= 0;
			rd_addr <= 0;
			rd_pending <= 0;
			sd_refresh_inhibit <= 0;
		end else
		if (uart_rxd_strobe)
		begin
			led_reg <= uart_rxd;
		end else
		if (spi_rx_strobe)
		begin
			if (spi_rx_cmd) begin
				count <= 0;
				spi_log_this <= uart_txd_ready && spi_rx_data == 8'h03;
			end else begin
				count <= count + 1;
			end

			trigger <= spi_rx_cmd;

			if (uart_txd_ready
			&& (count < 3 || spi_rx_cmd)
			&& spi_log_this)
			begin
				uart_txd <= spi_rx_data;
				uart_txd_strobe <= 1;
			end
		end
	end
endmodule
