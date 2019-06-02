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
 * FF CMD [L2 L1 L0 A3 A2 A1 A0 .... ]
 *
 * Command bytes are "R", "W" and "V"
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
	reg [20:0] counter;

	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

	reg wr_pending;
	reg rd_pending;
	reg [31:0] addr; // 32 bits, although most flash chips are 24 bits
	reg [23:0] msg_len; // up to 16 MB at a time

	reg [5:0] cmd_mode;
	reg [5:0] mode;
	localparam
		MODE_WAIT	= 6'b000000,
		MODE_CMD	= 6'b000100,
		MODE_L2		= 6'b000001,
		MODE_L1		= 6'b000010,
		MODE_L0		= 6'b000011,
		MODE_A3		= 6'b010111,
		MODE_A2		= 6'b010110,
		MODE_A1		= 6'b010101,
		MODE_A0		= 6'b010100,
		MODE_RD		= 6'b010011,
		MODE_WR		= 6'b001100,
		MODE_VERSION	= 6'b111110,
		MODE_INVALID	= 6'b111111;

	localparam
		CMD_HDR		= "!",
		CMD_RD		= "R",
		CMD_WR		= "W",
		CMD_VERSION	= "V",
		CMD_INVALID	= 8'hff;

	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		sd_rd_enable <= 0;
		sd_wr_enable <= 0;
		counter <= counter + 1;

		if (reset) begin
			mode <= MODE_WAIT;
			msg_len <= 0;
			wr_pending <= 0;
		end else
		if (uart_rxd_strobe)
		case(mode)
		MODE_WAIT: begin
			rd_pending <= 0;
			wr_pending <= 0;
			msg_len <= 0;
			addr <= 0;

			if (uart_rxd == CMD_HDR) begin
				mode <= MODE_CMD;
			end else begin
				uart_txd <= "!";
				uart_txd_strobe <= 1;
			end
		end
		MODE_L2: begin
			msg_len[23:16] <= uart_rxd;
			mode <= MODE_L1;
		end
		MODE_L1: begin
			msg_len[15: 8] <= uart_rxd;
			mode <= MODE_L0;
		end
		MODE_L0: begin
			msg_len[ 7: 0] <= uart_rxd;
			mode <= MODE_A3;
		end

		// build the address
		MODE_A3: begin
			addr[31:24] <= uart_rxd;
			mode <= MODE_A2;
		end
		MODE_A2: begin
			addr[23:16] <= uart_rxd;
			mode <= MODE_A1;
		end
		MODE_A1: begin
			addr[15: 8] <= uart_rxd;
			mode <= MODE_A0;
		end
		MODE_A0: begin
			addr[ 7: 0] <= uart_rxd;
			mode <= cmd_mode;
		end

		MODE_CMD: begin
			uart_txd <= uart_rxd;
			uart_txd_strobe <= 1;
			case(uart_rxd)
			CMD_RD: begin
				cmd_mode <= MODE_RD;
				mode <= MODE_L2;
			end
			CMD_WR: begin
				cmd_mode <= MODE_WR;
				mode <= MODE_L2;
			end
			CMD_VERSION: begin
				mode <= MODE_VERSION;
				msg_len <= 8;
			end
			default:
				mode <= MODE_INVALID;
			endcase
		end

		MODE_RD: begin
			// they are sending too fast. what should we do?
		end

		MODE_VERSION: begin
			// they are sending too fast. what should we do?
		end

		MODE_WR: begin
			// should check that we don't have a pending write
			sd_wr_data <= uart_rxd;
			wr_pending <= 1;
		end

		default: begin
			uart_txd <= "@";
			uart_txd_strobe <= 1;
			msg_len <= 0;
			mode <= MODE_WAIT;
		end
		endcase
		else
		if (mode == MODE_INVALID) begin
			mode <= MODE_WAIT;
			uart_txd <= "?";
			uart_txd_strobe <= 1;
		end else
		if (mode == MODE_RD) begin
			if (msg_len == 0) begin
				mode <= MODE_WAIT;
			end else
			if (sd_rd_ready && rd_pending) begin
				// new byte is available to send
				uart_txd <= sd_rd_data;
				uart_txd_strobe <= 1;
				msg_len <= msg_len - 1;
				addr <= addr + 1;
				rd_pending <= 0;
			end else
			if (!rd_pending
			&& uart_txd_ready
			&& !uart_txd_strobe
			//&& counter[6:0] == 0
			&& !sd_busy
			&& !sd_rd_enable
			&& !sd_wr_enable)
			begin
				// the SDRAM and UART are ready to
				// start another read
				sd_rd_enable <= 1;
				sd_rd_addr <= addr;
				rd_pending <= 1;
			end
		end else
		if (mode == MODE_WR) begin
			if (msg_len == 0) begin
				mode <= MODE_WAIT;
				uart_txd <= "W";
				uart_txd_strobe <= 1;
			end else
			if (wr_pending
			&& !sd_busy
			&& !sd_rd_enable
			&& !sd_wr_enable)
			begin
				// the SDRAM is ready for a write
				sd_wr_enable <= 1;
				sd_wr_addr <= addr;
				wr_pending <= 0;
				addr <= addr + 1;
				msg_len <= msg_len - 1;
			end
		end else
		if (mode == MODE_VERSION) begin
			if (msg_len == 0)
				mode <= MODE_WAIT;
			if (uart_txd_ready) begin
				msg_len <= msg_len - 1;
				uart_txd <= "1";
				uart_txd_strobe <= 1;
			end
		end
		begin
			// nothing to do this clock cycle.  relax!
		end
	end
	

`undef UART_FIFO
`ifdef UART_FIFO
	uart_tx_fifo #(.NUM(512)) txd(
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready)
	);
`else
	uart_tx txd(
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready)
	);
`endif

	uart_rx rxd(
		.clk(clk),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

endmodule
