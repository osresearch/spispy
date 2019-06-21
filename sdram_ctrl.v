/*
 * Copyright (c) 2011, Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 * All rights reserved.
 *
 * Redistribution and use in source and non-source forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in non-source form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
`ifndef _sdram_ctrl_v_
`define _sdram_ctrl_v_

`include "util.v"

module sdram_ctrl #(
	parameter CLK_FREQ_MHZ	= 100,	// sdram_clk freq in MHZ
	parameter POWERUP_DELAY	= 200,	// power up delay in us
	parameter REFRESH_MS	= 64,	// time to wait between refreshes in ms
	parameter BURST_LENGTH	= 8,	// 0, 1, 2, 4 or 8 (0 = full page)
	parameter ROW_WIDTH	= 13,	// Row width
	parameter COL_WIDTH	= 9,	// Column width
	parameter BA_WIDTH	= 2,	// Ba width
	parameter tCAC		= 2,	// CAS Latency
	parameter tRAC		= 5,	// RAS Latency
	parameter tRP		= 2,	// Command Period (PRE to ACT)
	parameter tRC		= 7,	// Command Period (REF to REF / ACT to ACT)
	parameter tMRD		= 2	// Mode Register Set To Command Delay time
)
(
	// SDRAM interface
	input			sdram_rst,
	input			sdram_clk,
	output	[BA_WIDTH-1:0]	ba_o,
	output		[12:0]	a_o,
	output			cs_n_o,
	output			ras_o,
	output			cas_o,
	output			we_o,
	output reg	[15:0]	dq_o,
	output reg	[1:0]	dqm_o,
	input		[15:0]	dq_i,
	output reg		dq_oe_o,
	output			cke_o,

	// Internal interface
	output			idle_o,
	input		[31:0]	adr_i,
	output reg	[31:0]	adr_o,
	input		[15:0]	dat_i,
	output reg	[15:0]	dat_o,
	input		[1:0]	sel_i,
	input			acc_i,
	output reg		ack_o,
	input			we_i,
	input			refresh_inhibit_i, // do not increment or enter refresh
	input			pause_read_i, // do not move past activation
	output			ack_raw,
	output		[15:0]	dat_raw,
);
	// Active Command To Read/Write Command Delay Time
	localparam tRCD = tRAC - tCAC;

	localparam POWERUP_CNT = CLK_FREQ_MHZ*POWERUP_DELAY;
	// refresh should be done for each row every 64 ms => 64e-3/2^ROW_WIDTH
	localparam REFRESH_TIMEOUT = (CLK_FREQ_MHZ*REFRESH_MS*1000)/(1<<ROW_WIDTH);

	// Burst types
	localparam
		SEQUENTIAL  = 1'b0,
		INTERLEAVED = 1'b1;

	// Write burst modes
	localparam
		PROGRAMMED_BL   = 1'b0,
		SINGLE_LOCATION = 1'b1;

	// FSM states
	localparam [3:0]
		INIT_POWERUP  = 4'h0,
		INIT_PRE      = 4'h1,
		INIT_REF      = 4'h2,
		INIT_PGM_MODE = 4'h3,
		IDLE          = 4'h4,
		READ          = 4'h5,
		WRITE         = 4'h6,
		ACTIVATE      = 4'h7,
		PRE           = 4'h8,
		PRE_ALL       = 4'h9,
		REF           = 4'ha;

	// SDRAM commands (a10_oe,a10,ras,cas,we)
	localparam [4:0]
		CMD_NOP      = 5'b10111,
		CMD_BST      = 5'b10110,
		CMD_READ     = 5'b10101,
		CMD_READ_AP  = 5'b11101,
		CMD_WRITE    = 5'b10100,
		CMD_WRITE_AP = 5'b11100,
		CMD_ACT      = 5'b00011,
		CMD_PALL     = 5'b11010,
		CMD_PRE      = 5'b10010,
		CMD_REF      = 5'b00001,
		CMD_SELF     = 5'b00000,
		CMD_MRS      = 5'b10000;

	reg  [3:0]			state;
	reg  [3:0]			next_state;
	reg  [`CLOG2(POWERUP_CNT):0]	cycle_count;
	reg  [`CLOG2(POWERUP_CNT):0]	next_cycle_count;
	reg  [`CLOG2(REFRESH_TIMEOUT):0] refresh_count;
	reg  [3:0]			state_count;
	reg  [ROW_WIDTH-1:0]		row_active[(1<<BA_WIDTH)-1:0];
	reg  [(1<<BA_WIDTH)-1:0]	bank_active;
	reg  [12:0]			a;
	reg  [BA_WIDTH-1:0]		ba;
	reg  [4:0]			cmd;
	reg				we_r;
	wire [2:0]			bl;
	wire				a10;
	wire				a10_oe;
	wire [BA_WIDTH-1:0]		curr_bank;
	wire				curr_bank_active;
	wire [ROW_WIDTH-1:0]		curr_row;
	wire				curr_row_active;

	assign cs_n_o = 1'b0;
	assign cke_o  = 1'b1;
	assign a_o    = a10_oe ? {a[12:11], a10, a[9:0]} : a;
	assign ba_o   = ba;
	assign {a10_oe,a10,ras_o,cas_o,we_o} = cmd;
	assign bl = (BURST_LENGTH == 0) ? 3'b111 :
		    (BURST_LENGTH == 1) ? 3'b000 :
		    (BURST_LENGTH == 2) ? 3'b001 :
		    (BURST_LENGTH == 4) ? 3'b010 :
		    (BURST_LENGTH == 8) ? 3'b011 : 3'b00;
	assign curr_bank = adr_i[(BA_WIDTH+ROW_WIDTH+COL_WIDTH):(ROW_WIDTH+COL_WIDTH+1)];
	assign curr_row = adr_i[(ROW_WIDTH+COL_WIDTH):(COL_WIDTH+1)];
	assign curr_bank_active = bank_active[curr_bank];
	assign curr_row_active = (bank_active[curr_bank] &
				 (row_active[curr_bank] == curr_row));
	assign idle_o = (state == IDLE); // | (state == REF) | (state == PRE_ALL);

	// fast access to the data for low-latency reads, otherwise use the write
	assign ack_raw = (we_i && ack_o) || (state == READ && cycle_count == tCAC);
	assign dat_raw = dq_i;

	always @(posedge sdram_clk) begin
		if (sdram_rst) begin
			dq_oe_o <= 1'b0;
			dq_o <= 0;
			dqm_o <= 2'b11;
			cmd <= CMD_NOP;
			state <= INIT_POWERUP;
			a <= 0;
			ba <= 0;
			ack_o <= 1'b0;
			cycle_count <= 0;
			we_r <= 0;
			bank_active <= 0;
		end else begin
			dq_oe_o <= 1'b0;
			dqm_o <= 2'b11;
			cmd <= CMD_NOP;
			ack_o <= 1'b0;
			if (!refresh_inhibit_i)
				refresh_count <= refresh_count + 1;
			cycle_count <= cycle_count + 1;
			case (state)
			INIT_POWERUP: begin
				if (cycle_count > POWERUP_CNT) begin
					cmd <= CMD_PALL;
					state <= INIT_PRE;
					cycle_count <= 0;
				end
			end

			INIT_PRE: begin
				if (cycle_count > tRP) begin
					cmd <= CMD_REF;
					state <= INIT_REF;
					state_count <= 0;
					cycle_count <= 0;
				end
			end

			INIT_REF: begin
				refresh_count <= 0;
				if (cycle_count > tRC) begin
					cmd <= CMD_REF;
					state_count <= state_count + 1;
					cycle_count <= 0;
				end
				// 8 refresh cycles
				if (state_count == 4'd7 & cycle_count == tRC) begin
					cmd <= CMD_MRS;
					state <= INIT_PGM_MODE;
					ba <= 2'b00;
					a[12:10] <= 0; // Reserved
					a[9] <= SINGLE_LOCATION;
					a[8:7] <= 0; // Standard operation
					a[6:4] <= tCAC;
					a[3] <= SEQUENTIAL;
					a[2:0] <= bl;
					cycle_count <= 0;
				end
			end

			INIT_PGM_MODE: begin
				if (cycle_count > tMRD)
					state <= IDLE;
			end

			IDLE: begin
				cycle_count <= 0;
				ba <= curr_bank;
				if (acc_i & curr_row_active) begin
					a[12:11] <= adr_i[12:11];
					a[9:0] <= adr_i[10:1];
					adr_o <= adr_i;
					if (we_i) begin
						ack_o <= 1'b1;
						dqm_o <= ~sel_i;
						dq_oe_o <= 1'b1;
						dq_o <= dat_i;
						cmd <= CMD_WRITE;
						state <= WRITE;
					end else begin
						dqm_o <= 2'b00;
						cmd <= CMD_READ;
						state <= READ;
					end
				end else
				if (acc_i & curr_bank_active) begin
					cmd <= CMD_PRE;
					state <= PRE;
					we_r <= we_i;
				end else
				if (acc_i) begin
					a <= curr_row;
					cmd <= CMD_ACT;
					state <= ACTIVATE;
					we_r <= we_i;
				end else
				if (!refresh_inhibit_i && refresh_count >= REFRESH_TIMEOUT) begin
					refresh_count <= 0;
					if (|bank_active) begin
						cmd <= CMD_PALL;
						state <= PRE_ALL;
					end else begin
						cmd <= CMD_REF;
						state <= REF;
					end
				end
			end

			REF: begin
				if (cycle_count >= tRC-1)
					state <= IDLE;
			end

			PRE: begin
				if (cycle_count >= tRP-1) begin
					bank_active[curr_bank] <= 1'b0;
					a <= curr_row;
					cmd <= CMD_ACT;
					state <= ACTIVATE;
					cycle_count <= 0;
				end
			end

			PRE_ALL: begin
				if (cycle_count >= tRP-1) begin
					bank_active <= 0;
					cmd <= CMD_REF;
					state <= REF;
					cycle_count <= 0;
				end
			end

			ACTIVATE: begin
				if (cycle_count >= tRCD-1) begin
					bank_active[curr_bank] <= 1'b1;
					row_active[curr_bank] <= curr_row;
					a[12:11] <= adr_i[12:11];
					a[9:0] <= adr_i[10:1];
					adr_o <= adr_i;
					cycle_count <= 0;
					if (we_r) begin
						ack_o <= 1'b1;
						dq_oe_o <= 1'b1;
						dq_o <= dat_i;
						dqm_o <= ~sel_i;
						cmd <= CMD_WRITE;
						state <= WRITE;
					end else
					if (pause_read_i) begin
						// do nothing until released
						cycle_count <= cycle_count;
					end else begin
						dqm_o <= 2'b00;
						cmd <= CMD_READ;
						state <= READ;
					end
				end
			end

			READ: begin
				/* TODO: support for full page burst */
				dqm_o <= 2'b00;
				if (cycle_count == 0) begin
					next_cycle_count <= 0;
					next_state <= IDLE;
				end

				if (cycle_count == tCAC) begin
					ack_o <= 1'b1;
					adr_o <= adr_i;
				end

				if (cycle_count >= tCAC) begin
					dat_o <= dq_i;
				end

				if (cycle_count > tCAC) begin
					if (BURST_LENGTH == 8)
						adr_o[3:1] <= adr_o[3:1] + 3'b1;
					else if (BURST_LENGTH == 4)
						adr_o[2:1] <= adr_o[2:1] + 2'b1;
					else if (BURST_LENGTH == 2)
						adr_o[1] <= adr_o[1] + 1'b1;
				end

				/* dqm has a latency of 2 cycles */
				if (cycle_count >= (BURST_LENGTH-1 + tCAC - 2) &
				    next_state != READ)
					dqm_o <= 2'b11;

				if (cycle_count >= (BURST_LENGTH-1) & next_state == IDLE) begin
					if (acc_i & curr_row_active & !we_i) begin
						dqm_o <= 2'b00;
						ba <= curr_bank;
						a[12:11] <= adr_i[12:11];
						a[9:0] <= adr_i[10:1];
						cmd <= CMD_READ;
						next_cycle_count <= tCAC - (cycle_count - (BURST_LENGTH-1));
						next_state <= READ;
					end else if (acc_i & curr_bank_active) begin
						we_r <= we_i;
						cmd <= CMD_PRE;
						next_cycle_count <= tCAC - (cycle_count - (BURST_LENGTH-1));
						next_state <= PRE;
					end else if (acc_i) begin
						we_r <= we_i;
						a <= curr_row;
						cmd <= CMD_ACT;
						next_cycle_count <= tCAC - (cycle_count - (BURST_LENGTH-1));
						next_state <= ACTIVATE;
					end
				end

				if (cycle_count >= (tCAC + BURST_LENGTH-1)) begin
					cycle_count <= next_cycle_count;
					state <= next_state;
					next_state <= IDLE;
					if (next_state == IDLE) begin
						cycle_count <= 0;
						if (acc_i & curr_row_active & we_i) begin
							ack_o <= 1'b1;
							ba <= curr_bank;
							a[12:11] <= adr_i[12:11];
							a[9:0] <= adr_i[10:1];
							adr_o <= adr_i;
							dq_o <= dat_i;
							dqm_o <= ~sel_i;
							dq_oe_o <= 1'b1;
							cmd <= CMD_WRITE;
						end else if (acc_i & curr_row_active & !we_i) begin
							ba <= curr_bank;
							a[12:11] <= adr_i[12:11];
							a[9:0] <= adr_i[10:1];
							dqm_o <= 2'b00;
							cmd <= CMD_READ;
						end else if (acc_i & curr_bank_active) begin
							we_r <= we_i;
							cmd <= CMD_PRE;
							state <= PRE;
						end else if (acc_i) begin
							we_r <= we_i;
							a <= curr_row;
							cmd <= CMD_ACT;
							state <= ACTIVATE;
						end else begin
							cmd <= CMD_NOP;
							state <= IDLE;
						end
					end
				end
			end

			WRITE: begin
				/* TODO: support for burst writes */
				cycle_count <= 0;
				if (acc_i & curr_row_active & we_i) begin
					ack_o <= 1'b1;
					ba <= curr_bank;
					a[12:11] <= adr_i[12:11];
					a[9:0] <= adr_i[10:1];
					adr_o <= adr_i;
					dq_o <= dat_i;
					dqm_o <= ~sel_i;
					dq_oe_o <= 1'b1;
					cmd <= CMD_WRITE;
				end else if (acc_i & curr_row_active & !we_i) begin
					ba <= curr_bank;
					a[12:11] <= adr_i[12:11];
					a[9:0] <= adr_i[10:1];
					dqm_o <= 2'b00;
					cmd <= CMD_READ;
					state <= READ;
				end else if (acc_i & curr_bank_active) begin
					we_r <= we_i;
					cmd <= CMD_PRE;
					state <= PRE;
				end else if (acc_i) begin
					we_r <= we_i;
					a <= curr_row;
					cmd <= CMD_ACT;
					state <= ACTIVATE;
				end else begin
					cmd <= CMD_NOP;
					state <= IDLE;
				end
			end
			endcase
		end
	end
endmodule

`endif
