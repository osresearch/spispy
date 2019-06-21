/*
 * ECP5 flash emulator
 */
`default_nettype none
`include "uart.v"
`include "pll_132.v"
`include "sdram_ctrl.v"
`include "spi_device.v"
`include "spi_flash.v"
`include "user.v"

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
	inout [27:0] gn,

	// buttons for user io
	input [6:0] btn,
);
	// gpio0 must be tied high to prevent board from rebooting
	assign wifi_gpio0 = 1;

	reg [7:0] led_reg;
	assign led = led_reg;

	// Generate a 132 MHz clock from the 25 MHz reference
	// with a 180 degree out of phase sdram clk
	wire clk_132, clk_132_180, locked, reset = !locked || btn[1];
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

	wire spi_tx_strobe;
	wire [7:0] spi_tx_data;

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
		.spi_tx_data(spi_tx_data),
		.spi_tx_strobe(spi_tx_strobe),
	);

	// flag for when we have timing critical spi transaction
	// there is a race condition with asserting this while the bus is busy
	// so don't mix serial operations with booting the machine right now
	wire spi_critical;

	// these will be written to the serial port
	wire [31:0] spi_log_addr;
	wire [7:0] spi_log_len;
	wire spi_log_strobe;

	// interface to the memory
	wire [31:0] spi_read_addr;
	wire [7:0] spi_read_data;
	wire spi_read_enable;

	spi_flash spi_f(
		.clk(clk),
		.reset(reset),

		// spi bus interface
		.spi_cs(spi_cs_enable ? 0 : spi_cs_in),
		.spi_rx_data(spi_rx_data),
		.spi_rx_cmd(spi_rx_cmd),
		.spi_rx_strobe(spi_rx_strobe),
		.spi_tx_strobe(spi_tx_strobe),
		.spi_tx_data(spi_tx_data),

		// memory interface
		.spi_critical(spi_critical),
		.ram_addr(spi_read_addr),
		.ram_read_enable(spi_read_enable), // when an address has been received
		.ram_read_data(sd_rd_data),
		.ram_read_valid(spi_critical ? sd_ack : 0),

		// logging interface
		.log_strobe(spi_log_strobe),
		.log_addr(spi_log_addr),
		.log_len(spi_log_len),
	);

	// oscilloscope debug pins also on jp2
	wire debug_0 = spi_miso_in;
	wire debug_1 = spi_miso_out;
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
	wire [DATA_WIDTH-1:0] sd_rd_data;

	// the serial user interface has a lower priority access to the sdram
	wire user_sd_we;
	wire user_sd_enable;
	wire [ADDR_WIDTH-1:0] user_sd_addr;
	wire [DATA_WIDTH-1:0] user_sd_wr_data;

	// the spi bus can take control of the ram by asserting the spi_critical
	// signal, which takes over all of the inputs
	// note that right now the sdram is 16-bit wide, but we read only the bottom byte
	// so it is necessary to shift the address by 1
	wire sd_we = spi_critical ? 0 : user_sd_we;
	wire sd_enable = spi_critical ? spi_read_enable : user_sd_enable;
	wire [ADDR_WIDTH-1:0] sd_addr = { spi_critical ? spi_read_addr : user_sd_addr, 1'b0 };
	//wire [ADDR_WIDTH-1:0] sd_addr = spi_critical ? spi_read_addr : user_sd_addr;
	wire [DATA_WIDTH-1:0] sd_wr_data = spi_critical ? 16'hDEAD : user_sd_wr_data;

	wire sd_ack;
	wire sd_idle;

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

	// the dq pins are bidirectional and controlled by the dq_oe signal
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
	//.dat_raw	(sd_rd_data),
	.dat_o	(sd_rd_data),
	.sel_i		(2'b11), // always do both bytes
	.acc_i		(sd_enable),
	//.ack_raw	(sd_ack),
	.ack_o	(sd_ack),
	.we_i		(sd_we),
	.refresh_inhibit_i(sd_refresh_inhibit),
	.pause_read_i	(sd_pause_read)
);

	// Serial port user interface
	wire [7:0] user_txd;
	wire user_txd_strobe;
	wire user_txd_ready = spi_critical ? 0 : uart_txd_ready;

	user_command_parser user(
		.clk(clk),
		.reset(reset),
		// serial port interface
		.uart_rxd(uart_rxd),
		.uart_rxd_strobe(uart_rxd_strobe),
		.uart_txd(user_txd),
		.uart_txd_strobe(user_txd_strobe),
		.uart_txd_ready(user_txd_ready),

		// sdram interface
		.sd_addr(user_sd_addr),
		.sd_we(user_sd_we),
		.sd_enable(user_sd_enable),
		.sd_wr_data(user_sd_wr_data),
		.sd_rd_data(sd_rd_data),
		.sd_ack(spi_critical ? 0 : sd_ack),
		.sd_idle(spi_critical ? 0 : sd_idle),
	);

	reg [2:0] uart_words;
	reg [31:0] uart_word;
	reg user_sd_enable_prev;
	always @(posedge clk) user_sd_enable_prev <= user_sd_enable;
	wire user_sd_enable_rising = user_sd_enable && !user_sd_enable_prev;

	reg [4:0] counter;

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		if (counter == 0)
			trigger <= 0;
		else
			counter <= counter - 1;

		if (reset) begin
			// anything to do?
			counter <= 0;
		end else
		if (uart_rxd_strobe)
		begin
			led_reg <= uart_rxd;
		end else
		if (spi_log_strobe && uart_txd_ready)
		begin
			// a SPI transaction has just occured;
			// write it to the serial port if there is space
			uart_word <= { spi_log_addr[23:0], spi_log_len };
			uart_words <= 4;
			led_reg <= spi_log_addr[11:4];

			counter <= ~0;

			if (spi_log_addr[23:0] == 24'h000010)
				trigger <= 1;
		end else
		if (uart_words != 0 && uart_txd_ready)
		begin
			uart_words <= uart_words - 1;
			uart_txd_strobe <= 1;
			uart_txd <= "0" + uart_words; // uart_word[31:24];
			uart_txd <= uart_word[31:24];
			uart_word <= uart_word << 8;
		end else
		if (user_txd_strobe && uart_txd_ready)
		begin
			uart_txd_strobe <= 1;
			uart_txd <= user_txd;
		end else begin
			led_reg[7] <= spi_critical;
			led_reg[6] <= sd_we;
		end
	end
endmodule
