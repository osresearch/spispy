/*
 * Test the DRAM functionality on the ulx3s board
 */
`default_nettype none
`include "uart.v"
`include "pll_120.v"
`include "pll_132.v"
`include "sdram_ctrl.v"

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
		
);
	// gpio0 must be tied high to prevent board from rebooting
	assign wifi_gpio0 = 1;

	reg [7:0] led_reg;
	assign led = led_reg;

	// Generate a 120 MHz clock from the 25 MHz reference
	// with a 180 degree out of phase sdram clk
	wire clk_120, clk_120_180, locked, reset = !locked;
	//pll_120 pll_120_i(clk_25mhz, clk_120, clk_120_180, locked);
	pll_132 pll_120_i(clk_25mhz, clk_120, clk_120_180, locked);
	wire clk = clk_120;
	wire sdram_clk = clk_120_180; // sdram needs to be stable on the rising edge

	// serial port interface for talking to the host system
	// 120 MHz clock / 40 == 3 megabaud
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_rxd_strobe;
	wire [7:0] uart_rxd;

	uart #(
		//.DIVISOR(40)
		.DIVISOR(44)
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

	// sdram logical interface has the same 16-bit interface
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
	.CLK_FREQ_MHZ			(120),	// sdram_clk freq in MHZ
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

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		rd_timer <= rd_timer + 1;

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
		if (uart_txd_ready
		&& !uart_txd_strobe
		&& sd_idle
		&& !rd_pending
		&& !sd_enable
		&& wr_addr[24] == 1
		) begin
			// turn off the refresh counter and pause after the
			// row has been activated
			sd_refresh_inhibit <= 1;
			//sd_pause_read <= 1;

			// start a read from the same page
			rd_timer <= 0;
			rd_pending <= 1;
			sd_we <= 0;
			sd_enable <= 1;
			sd_addr <= { rd_addr[24:8], 8'hFF };
			led_reg <= rd_addr[7:0];
			uart_txd <= "-"; // replace old value
		end else
		if (rd_pending && rd_pending < 30)
		begin
			// when the first read has finished, turn off the read
			if (sd_ack)
				sd_enable <= 0;
			rd_pending <= rd_pending + 1;
		end else
		if (rd_pending == 30) begin
			// do another read, this time with the bottom bits filled in
			//sd_pause_read <= 0;
			rd_pending <= 31;
			sd_enable <= 1;
			sd_addr[7:0] <= rd_addr[7:0];
			rd_timer <= 0;
		end else
		if (rd_pending == 31 && sd_ack)
		begin
			sd_refresh_inhibit <= 0;
			rd_pending <= 0;
			sd_enable <= 0;
			if (sd_rd_data[7:0] != "A" + rd_addr[4:1])
				uart_txd <= "!";
			else
				uart_txd <= sd_rd_data[7:0];
			//uart_txd <= rd_timer[10:8] ? 8'hFF : rd_timer[7:0];
			uart_txd_strobe <= 1;

			rd_addr <= rd_addr + 2;
		end else
		if (sd_idle && !sd_enable && !rd_pending)
		begin
			// fill the memory with something that tells us where we are
			sd_we <= 1;
			sd_wr_data <= {
				"0" + wr_addr[4:1],
				"A" + wr_addr[4:1]
			}; // { wr_addr[15:12], wr_addr[3:0] };
			//sd_wr_data <= wr_addr[24:1];
			sd_addr <= wr_addr;
			sd_enable <= 1;
			wr_addr <= wr_addr + 2;
		end else
		if (sd_ack) begin
			// write complete
			sd_enable <= 0;
		end
	end
endmodule
