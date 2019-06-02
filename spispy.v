/** \file
 * SPI bus emulator.
 *
 * This uses the onboard DRAM to store a flash image and can provide
 * it over a SPI bus to an external device.
 *
 * This also configures the serial port at 3 Mb/s for the command
 * protocol.
 *
 * Command protocol looks like:
 * FF L0 cmd ...L0 bytes...
 *
 * Commands are:
 * Read:  52 A3 A1 A2 A0
 * Write: 57 A3 A1 A2 A0 ......
 *
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_96.v"
`include "sdram_controller.v"

module top(
	input clk_100mhz,
	inout [7:0] pmod1,

	// SDRAM physical interface
	output [12:0] sdram_addr,
	inout [7:0] sdram_data,
	output [1:0] sdram_bank,
	output sdram_clk,
	output sdram_cke,
	output sdram_we,
	output sdram_cs,
	output sdram_dqm,
	output sdram_ras,
	output sdram_cas
);
	wire locked, clk_96mhz, clk;
	wire reset = !locked;
	pll_96 pll(clk_100mhz, clk_96mhz, locked);
//`define CLK48
`ifdef CLK48
	always @(posedge clk_96mhz) clk <= !clk;
`else
	assign clk = clk_96mhz;
`endif

	// sdram logical interface
	reg [24:0] sd_wr_addr = 0;
	reg [7:0] sd_wr_data;
	reg sd_wr_enable;

	reg [24:0] sd_rd_addr = 0;
	wire [7:0] sd_rd_data;
	reg sd_rd_enable = 0;
	wire sd_rd_ready;

	wire sd_busy;
	assign sdram_clk = clk;

	sdram_controller sdram(
		.clk(clk),
		.rst_n(!reset),

		// physical interface
		.addr(sdram_addr),
		.bank_addr(sdram_bank),
		.data(sdram_data),
		.clock_enable(sdram_cke),
		.cs_n(sdram_cs),
		.ras_n(sdram_ras),
		.cas_n(sdram_cas),
		.we_n(sdram_we),
		.data_mask(sdram_dqm),

		// logical interface
		.wr_addr(sd_wr_addr),
		.wr_enable(sd_wr_enable),
		.wr_data(sd_wr_data),
		.rd_addr(sd_rd_addr),
		.rd_enable(sd_rd_enable),
		.rd_data(sd_rd_data),
		.rd_ready(sd_rd_ready),
		.busy(sd_busy),
	);

	wire serial_txd;
	wire serial_rxd;

	gpio gpio_txd(
		.enable(1), // always on
		.pin(pmod1[0]),
		.out(serial_txd),
	);

	gpio #(.PULLUP(1)) gpio_rxd(
		.enable(0), // always input, with pullup
		.pin(pmod1[1]),
		.in(serial_rxd),
		.out(1),
	);

	// generate a 3 MHz/12 MHz serial clock from the 96 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	wire clk_3mhz, clk_12mhz;
`ifdef CLK48
	divide_by_n #(.N(4)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(16)) div4(clk, reset, clk_3mhz);
`else
	divide_by_n #(.N(8)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(32)) div4(clk, reset, clk_3mhz);
`endif

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

	reg wr_pending;
	reg [31:0] wr_addr;
	reg [7:0] msg_len;

	reg [5:0] mode;
	localparam
		MODE_WAIT	= 6'b000000,
		MODE_LEN	= 6'b000001,
		MODE_CMD	= 6'b000010,
		MODE_RD		= 6'b000011,
		MODE_RD_A3	= 6'b000111,
		MODE_RD_A2	= 6'b000110,
		MODE_RD_A1	= 6'b000101,
		MODE_RD_A0	= 6'b000100,
		MODE_WR		= 6'b001100,
		MODE_WR_A3	= 6'b001011,
		MODE_WR_A2	= 6'b001010,
		MODE_WR_A1	= 6'b001001,
		MODE_WR_A0	= 6'b001000,
		MODE_VERSION	= 6'b111110,
		MODE_INVALID	= 6'b111111;

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		sd_rd_enable <= 0;
		sd_wr_enable <= 0;

		if (reset) begin
			mode <= MODE_WAIT;
			msg_len <= 0;
			wr_pending <= 0;
		end else
		if (uart_rxd_strobe)
		case(mode)
		MODE_WAIT: begin
			if (uart_rxd == 8'hFF) begin
				mode <= MODE_LEN;
				uart_txd <= "@";
				uart_txd_strobe <= 1;
			end else begin
				uart_txd <= "!";
				uart_txd_strobe <= 1;
			end
		end
		MODE_LEN: begin
			msg_len <= uart_rxd;
			uart_txd <= "0" + uart_rxd;
			uart_txd_strobe <= 1;
			mode <= MODE_CMD;
		end
		MODE_CMD: begin
			uart_txd <= uart_rxd;
			uart_txd_strobe <= 1;
			if (uart_rxd == "R") // 8'h52
				mode <= MODE_RD_A3;
			else
			if (uart_rxd == "W") // 8'h57
				mode <= MODE_WR_A3;
			else
			if (uart_rxd == "V") // 8'h56
				mode <= MODE_VERSION;
			else
				mode <= MODE_INVALID;
		end

		// build the read address (only have a 32-MB partial address)
		MODE_RD_A3: begin
			sd_rd_addr[24:24] <= uart_rxd[0:0];
			mode <= MODE_RD_A2;
		end
		MODE_RD_A2: begin
			sd_rd_addr[23:16] <= uart_rxd;
			mode <= MODE_RD_A1;
		end
		MODE_RD_A1: begin
			sd_rd_addr[15: 8] <= uart_rxd;
			mode <= MODE_RD_A0;
		end
		MODE_RD_A0: begin
			sd_rd_addr[ 7: 0] <= uart_rxd;
			mode <= MODE_RD;
		end

		// build the write address
		MODE_WR_A3: begin
			wr_addr[31:24] <= uart_rxd;
			mode <= MODE_WR_A2;
			uart_txd <= "3";
			uart_txd_strobe <= 1;
		end
		MODE_WR_A2: begin
			wr_addr[23:16] <= uart_rxd;
			mode <= MODE_WR_A1;
			uart_txd <= "2";
			uart_txd_strobe <= 1;
		end
		MODE_WR_A1: begin
			wr_addr[15: 8] <= uart_rxd;
			mode <= MODE_WR_A0;
			uart_txd <= "1";
			uart_txd_strobe <= 1;
		end
		MODE_WR_A0: begin
			wr_addr[ 7: 0] <= uart_rxd;
			mode <= MODE_WR;
			uart_txd <= "0";
			uart_txd_strobe <= 1;
		end
		MODE_WR: begin
			// should check that we don't have a pending write
			sd_wr_data <= uart_rxd;
			wr_pending <= 1;
			uart_txd <= "w";
			uart_txd_strobe <= 1;
		end
		//default: begin
		MODE_INVALID: begin
			uart_txd <= mode;
			uart_txd_strobe <= 1;
			mode <= MODE_WAIT;
		end
		endcase
		else
		if (mode == MODE_RD) begin
			if (sd_rd_ready) begin
				// new byte is available to send
				uart_txd <= sd_rd_data;
				uart_txd_strobe <= 1;
				msg_len <= msg_len - 1;
				sd_rd_addr <= sd_rd_addr + 1;

				// if we're done with the read, stop
				if (msg_len == 1) begin
					mode <= MODE_WAIT;
				end
			end else
			if (!sd_busy && !sd_rd_enable && !sd_wr_enable) begin
				// start another read
				sd_rd_enable <= 1;
			end
		end else
		if (mode == MODE_WR && wr_pending) begin
			if (!sd_busy && !sd_rd_enable && !sd_wr_enable) begin
				sd_wr_enable <= 1;
				sd_wr_addr <= wr_addr;
				wr_pending <= 0;
				wr_addr <= wr_addr + 1;
				msg_len <= msg_len - 1;

					uart_txd <= "W";
					uart_txd_strobe <= 1;

				// if we've written all the data to SDRAM
				// go back to waiting
				if (msg_len == 1) begin
					mode <= MODE_WAIT;
				end
			end else begin
				//uart_txd <= "w";
				//uart_txd_strobe <= 1;
			end
		end else
		if (mode == MODE_VERSION) begin
			if (msg_len != 0) begin
				msg_len <= msg_len - 1;
				uart_txd <= "1";
				uart_txd_strobe <= 1;
			end else begin
				mode <= MODE_WAIT;
			end
		end
		begin
			// nothing to do this clock cycle.  relax!
		end
	end
	

	uart_tx_fifo txd(
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
	);

	uart_rx rxd(
		.mclk(clk),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

endmodule
