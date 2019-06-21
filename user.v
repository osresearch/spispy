`ifndef _user_v_
`define _user_v_
/*
 * Parse user commands on the serial port, generate SDRAM commands.
 *
 */

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
	output reg [7:0] sd_wr_data,
	input [7:0] sd_rd_data,
	input sd_ack,
	input sd_idle,
	output reg sd_we,
	output reg sd_enable
);
	parameter ADDR_BITS = 32;

	reg [20:0] counter;

	reg wr_pending;
	reg rd_pending;
	reg [ADDR_BITS-1:0] sd_addr;
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
		counter <= counter + 1;

		if (reset) begin
			mode <= MODE_WAIT;
			msg_len <= 0;
			wr_pending <= 0;
			rd_pending <= 0;
		end else
		if (uart_rxd_strobe)
		case(mode)
		MODE_WAIT: begin
			sd_we <= 0;
			rd_pending <= 0;
			wr_pending <= 0;
			msg_len <= 0;
			sd_addr <= 0;

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
			sd_addr[31:24] <= uart_rxd;
			mode <= MODE_A2;
		end
		MODE_A2: begin
			sd_addr[23:16] <= uart_rxd;
			mode <= MODE_A1;
		end
		MODE_A1: begin
			sd_addr[15: 8] <= uart_rxd;
			mode <= MODE_A0;
		end
		MODE_A0: begin
			sd_addr[ 7: 0] <= uart_rxd;
			mode <= cmd_mode;
		end

		MODE_CMD: begin
			//uart_txd <= uart_rxd;
			//uart_txd_strobe <= 1;
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
			if (wr_pending) begin
				uart_txd_strobe <= 1;
				uart_txd <= "%";
			end
			sd_wr_data <= uart_rxd;
			sd_we <= 1;
			sd_enable <= 1;
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
			if (sd_ack && rd_pending) begin
				// new byte is available to send
				sd_enable <= 0;
				uart_txd <= sd_addr[7:0] + "A"; // sd_rd_data;
				uart_txd <= sd_rd_data;
				uart_txd_strobe <= 1;
				msg_len <= msg_len - 1;
				sd_addr <= sd_addr + 1;
				rd_pending <= 0;
			end else
			if (!rd_pending
			&& uart_txd_ready
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
			end else
			if (wr_pending && sd_ack) begin
				// write done, prepare for next byte
				sd_we <= 0;
				sd_enable <= 0;
				wr_pending <= 0;
				sd_addr <= sd_addr + 1;
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
	
endmodule

`endif
