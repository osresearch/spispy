`ifndef _user_v_
`define _user_v_
/*
 * Parse user commands on the serial port, generate SDRAM commands.
 *
 */
`include "fifo.v"

module user_command_parser(
	input clk,
	input reset,

	// interface to the outside
	input [7:0] uart_rxd,
	input uart_rxd_strobe,
	output reg [7:0] uart_txd,
	output reg uart_txd_strobe,
	input uart_txd_ready,

	// interface to the sdram
	output reg [ADDR_BITS-1:0] sd_addr,
	output reg [15:0] sd_wr_data,
	output reg [1:0] sd_wr_mask,
	input [15:0] sd_rd_data,
	input sd_ack,
	input sd_idle,
	output reg sd_we,
	output reg sd_enable
);
	parameter ADDR_BITS = 32;

	reg [8:0] counter;

	reg wr_pending;
	reg rd_pending;
	reg [ADDR_BITS-1:0] sd_addr;
	reg [31:0] msg_len; // up to 16 MB at a time

	reg [5:0] cmd_mode;
	reg [5:0] mode;
	localparam
		MODE_WAIT	= 6'b000000,
		MODE_CMD	= 6'b000100,
		MODE_L3		= 6'b100001,
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

	// when we're processing input we are not able to handle
	// any incoming messages on the fifo.
	reg processing;

	// when connected to the USB serial port bytes arrive in
	// batches so it is necessary to store them in a fifo until
	// we're ready to process them.
	wire fifo_rxd_strobe = fifo_rxd_available && !processing;
	wire [7:0] fifo_rxd;
	wire fifo_rxd_available;

	fifo #(.NUM(512)) rx_fifo(
		.clk(clk),
		.reset(reset),
		.write_data(uart_rxd),
		.write_strobe(uart_rxd_strobe),
		.data_available(fifo_rxd_available),
		.read_data(fifo_rxd),
		.read_strobe(fifo_rxd_strobe),
	);


	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		counter <= counter + 1;

		if (reset) begin
			mode <= MODE_WAIT;
			msg_len <= 0;
			wr_pending <= 0;
			rd_pending <= 0;
			processing <= 0;
		end else
		if (fifo_rxd_available && !processing)
		case(mode)
		MODE_WAIT: begin
			sd_we <= 0;
			rd_pending <= 0;
			wr_pending <= 0;
			msg_len <= 0;
			sd_addr <= 0;

			if (fifo_rxd == CMD_HDR) begin
				mode <= MODE_CMD;
			end else begin
				uart_txd <= "!";
				uart_txd <= fifo_rxd;
				uart_txd_strobe <= 1;
			end
		end
		MODE_L3: begin
			msg_len[31:24] <= fifo_rxd;
			mode <= MODE_L2;
		end
		MODE_L2: begin
			msg_len[23:16] <= fifo_rxd;
			mode <= MODE_L1;
		end
		MODE_L1: begin
			msg_len[15: 8] <= fifo_rxd;
			mode <= MODE_L0;
		end
		MODE_L0: begin
			msg_len[ 7: 0] <= fifo_rxd;
			mode <= MODE_A3;
		end

		// build the address
		MODE_A3: begin
			sd_addr[31:24] <= fifo_rxd;
			mode <= MODE_A2;
		end
		MODE_A2: begin
			sd_addr[23:16] <= fifo_rxd;
			mode <= MODE_A1;
		end
		MODE_A1: begin
			sd_addr[15: 8] <= fifo_rxd;
			mode <= MODE_A0;
		end
		MODE_A0: begin
			sd_addr[ 7: 0] <= fifo_rxd;
			mode <= cmd_mode;
		end

		MODE_CMD: begin
			//uart_txd <= fifo_rxd;
			//uart_txd_strobe <= 1;
			case(fifo_rxd)
			CMD_RD: begin
				cmd_mode <= MODE_RD;
				mode <= MODE_L3;
			end
			CMD_WR: begin
				cmd_mode <= MODE_WR;
				mode <= MODE_L3;
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
			// we're processing stuff, don't responsd
			processing <= 1;
		end

		MODE_VERSION: begin
			// we're processing stuff, don't responsd
			processing <= 1;
		end

		MODE_WR: begin
			processing <= 1;

			if (wr_pending) begin
				// if we still have a pending write, do nothing
			end else begin
				// consume the byte and start a new write
				// select which byte we're writing
				if (sd_addr[0]) begin
					sd_wr_data <= { fifo_rxd, 8'h00 };
					sd_wr_mask <= 2'b10;
				end else begin
					sd_wr_data <= { 8'h00, fifo_rxd };
					sd_wr_mask <= 2'b01;
				end
				sd_we <= 1;
				sd_enable <= 1;
				wr_pending <= 1;
			end
		end

		default: begin
			uart_txd <= "@";
			uart_txd_strobe <= 1;
			msg_len <= 0;
			mode <= MODE_WAIT;
			processing <= 0;
		end
		endcase
		else // these are all no bytes are available
		if (mode == MODE_INVALID) begin
			mode <= MODE_WAIT;
			uart_txd <= "?";
			uart_txd_strobe <= 1;
		end else
		if (mode == MODE_RD) begin
			if (msg_len == 0) begin
				mode <= MODE_WAIT;
				processing <= 0;
			end else
			if (sd_ack && rd_pending) begin
				// new byte is available to send
				// we waited until txd_ready was set so that
				// we know that we can send this byte to the serial port
				sd_enable <= 0;
				uart_txd <= sd_addr[0] ? sd_rd_data[15:8] : sd_rd_data[7:0];
				uart_txd_strobe <= 1;
				msg_len <= msg_len - 1;
				sd_addr <= sd_addr + 1;
				rd_pending <= 0;
			end else
			if (!rd_pending
			&& uart_txd_ready
			//&& !uart_txd_strobe
			//&& counter == 0
			&& sd_idle
			&& !sd_enable)
			begin
				// the SDRAM and UART are ready to
				// start another read
				sd_enable <= 1;
				sd_we <= 0;
				rd_pending <= 1;
			end
		end else
		if (mode == MODE_WR) begin
			if (msg_len == 0) begin
				mode <= MODE_WAIT;
				uart_txd <= "w";
				uart_txd_strobe <= 1;
				processing <= 0;
			end else
			if (wr_pending && sd_ack) begin
				// write done, prepare for next byte
				sd_we <= 0;
				sd_enable <= 0;
				wr_pending <= 0;
				sd_addr <= sd_addr + 1;
				msg_len <= msg_len - 1;
				processing <= 0;
			end
		end else
		if (mode == MODE_VERSION) begin
			if (msg_len == 0) begin
				mode <= MODE_WAIT;
				processing <= 0;
			end else
			if (uart_txd_ready)
			//&& !uart_txd_strobe)
			begin
				msg_len <= msg_len - 1;
				uart_txd <= "1";
				uart_txd_strobe <= 1;
			end
		end
		begin
			// nothing to do this clock cycle.  relax!
		end
	end
	
endmodule

`endif
