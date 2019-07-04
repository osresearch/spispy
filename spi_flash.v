/*
 * SPI Flash device.
 *
 * This implements may of the JEDEC standard SPI flash commands to emulate
 * a common serial flash device.  It is not a very comprehensive design;
 * there is no /HOLD or /WP support, nor are many of the fancy modes implemented.
 *
 * Modern hardware requires SFDP, so it is included.
 */
`ifndef _spi_flash_v_
`define _spi_flash_v_

module spi_flash(
	input	clk,
	input	reset,

	// spi bus interface
	output spi_output_enable,
	input	spi_cs,
	input [7:0] spi_rx_data,
	input	spi_rx_cmd, // when a new transacation is starting
	input	spi_rx_strobe, // when a new byte has been received
	output	spi_tx_strobe, // when we have a new byte to send to thedevice
	output [7:0] spi_tx_data,
	input   [2:0] spi_rx_bit, // which bit is currently being received
	input    spi_rx_bit_strobe, // when a new bit has arrived

	// memory interface
	output	spi_critical, // asserted when we need to lock the memory bus
	output	ram_refresh_inhibit, // asserted when we really need the RAM
	output [31:0] ram_addr,
	output ram_enable,
	output ram_write_enable,
	output reg [1:0] ram_write_mask,
	input [15:0] ram_read_data,
	output [15:0] ram_write_data,
	input ram_data_valid, // when the data is valid

	// logging interface. valid until the next command starts
	output reg [31:0] log_addr,
	output reg [7:0] log_len,
	output reg log_strobe,
	output reg [7:0] errors
);
	parameter VERBOSE_LOGGING = 0;
	parameter SFDP_OFFSET = 8'h01; // above the 16 MB normal memory

	reg spi_tx_strobe;
	reg spi_tx_data;

	reg [31:0] ram_addr;
	reg [15:0] ram_write_data;
	reg ram_enable;
	reg ram_write_enable;
	reg read_complete;
	reg read_immediate_update;

	reg [2:0] spi_count;
	reg spi_rd_cmd;
	reg spi_rd_dummy; // if there is an extra byte before the data

	// as soon as we detect the start of a read command, lock the sdram
	// control for our exclusive access
	reg spi_critical;
	reg spi_output_enable;
	reg ram_refresh_inhibit;

	// synchronize the spi_cs line to our clock
	reg [1:0] spi_cs_prev;
	wire spi_cs_rising = !spi_cs_prev[1] && spi_cs_prev[0];
	wire spi_cs_sync = spi_cs_prev[0];
	always @(posedge clk) spi_cs_prev <= { spi_cs_prev[0], spi_cs };

	reg [7:0] cycles;

	// async page erase
	reg erase_active;
	reg [15:0] erase_len;
	reg [31:0] erase_addr;

	// commands are documented in various SPI flash datasheets,
	// such as https://www.winbond.com/resource-files/w25q256fv_revg1_120214_qpi_website_rev_g.pdf
	localparam SPI_CMD_PP3	= 8'h02;
	localparam SPI_CMD_READ	= 8'h03;
	localparam SPI_CMD_WRDS	= 8'h04;
	localparam SPI_CMD_RDSR	= 8'h05;
	localparam SPI_CMD_WREN	= 8'h06;
	localparam SPI_CMD_ERASE= 8'h20;
	localparam SPI_CMD_SFDP = 8'h5a;
	localparam SPI_CMD_RDID = 8'h9F;

	localparam MODE_IDLE	= 4'h0;
	localparam MODE_READ	= 4'h1;
	localparam MODE_RDID	= 4'h2;
	localparam MODE_ERASE	= 4'h3;
	localparam MODE_WRITE	= 4'h4;

	reg [3:0] spi_mode;
	reg [23:0] spi_jid0 = { 8'hC2, 8'h20, 8'h18 }; // 16 MB flash chip

	reg spi_status_srp0 = 0;
	reg spi_status_tb = 0;
	reg [3:0] spi_status_bp = 0;
	reg spi_status_wrel = 0;
	reg spi_status_busy = 0;

	wire [7:0] spi_status_reg = {
		spi_status_srp0,
		spi_status_tb,
		spi_status_bp[3:0],
		spi_status_wrel,
		spi_status_busy
	};

	// SPI command state machine
	always @(posedge clk)
	begin
		cycles <= cycles + 1;

		log_strobe <= 0;
		spi_tx_strobe <= 0;

		if (reset) begin
			errors <= 0;
			spi_critical <= 0;
			spi_output_enable <= 0;
			ram_refresh_inhibit <= 0;
			ram_enable <= 0;
			read_immediate_update <= 0;
			spi_mode <= 0;
			erase_active <= 0;
			erase_len <= 0;
		end else
		if (spi_cs_sync) begin
			// no longer asserted, release our locks and signal
			// a logging event if we had one
			// if we have an active erase do not release our sd lock
			spi_critical <= erase_active;
			spi_output_enable <= 0;
			spi_count <= 0;
			spi_rd_cmd <= 0;
			spi_mode <= 0;
			ram_refresh_inhibit <= 0;
			read_immediate_update <= 0;

			if (spi_cs_rising && spi_rd_cmd && spi_count == 4)
			begin
				// without verbose logging this is the time
				// to notify that we've had a read transaction
				if (!VERBOSE_LOGGING) // && (spi_mode != MODE_READ))
					log_strobe <= 1;
			end

			// if we are exiting from MODE_WRITE, unset the write latch
			if (spi_mode == MODE_WRITE)
				spi_status_wrel <= 0;

			// if we're in erase mode, keep erasing
			if (erase_active)
			begin
				if (erase_len == 0)
				begin
					// last erase is done.
					spi_critical <= 0;
					erase_active <= 0;

					// unset our status register flags
					errors[7] <= 0;
					spi_status_wrel <= 0;
					spi_status_busy <= 0;
				end else
				if (!ram_enable)
				begin
					// erase the next word
					ram_addr <= erase_addr;
					ram_write_mask <= 2'b11; // both words
					ram_write_data <= 16'hFFFF;
					ram_write_enable <= 1;
					ram_enable <= 1;
				end else
				if (ram_data_valid && ram_write_enable) begin
					// write complete, might have been
					// interleaved with reads, so check that
					// this was a write
					errors[7] <= ~errors[7];
					ram_write_enable <= 0;
					ram_enable <= 0;
					erase_len <= erase_len - 2;
					erase_addr <= erase_addr + 2;
				end
			end else
			if (ram_data_valid) begin
				// someone was doing a read?
				ram_write_enable <= 0;
				ram_enable <= 0;
			end
		end else
		if (spi_rx_cmd)
		begin
			// first byte of a new transaction
			ram_addr <= 0;
			log_addr <= 0;
			log_len <= 0;
			spi_rd_cmd <= 0;

			spi_count <= 1;
			//spi_tx_data <= 8'hF1;
			//spi_tx_strobe <= 1;

			// Anytime a SPI read command starts, assert
			// exclusive access to the SD interface
			case(spi_rx_data)
			SPI_CMD_READ: begin
				// will need to get the RAM
				spi_critical <= 1;
				spi_output_enable <= 1;
				ram_refresh_inhibit <= 1;
				spi_rd_cmd <= 1;
				ram_addr[31:24] <= 0;
				spi_rd_dummy <= 0;
				spi_mode <= MODE_READ;
			end
			SPI_CMD_SFDP: begin
				// will need to get the RAM
				spi_critical <= 1;
				spi_output_enable <= 1;
				ram_refresh_inhibit <= 1;
				spi_rd_cmd <= 1;
				ram_addr[31:24] <= SFDP_OFFSET;
				spi_rd_dummy <= 1;
				spi_mode <= MODE_READ;
			end
			SPI_CMD_WREN: begin
				// write enable
				// TODO: check for write protect
				spi_status_wrel <= 1;
				spi_output_enable <= 1;
			end
			SPI_CMD_WRDS: begin
				// write disable
				spi_status_wrel <= 0;
				spi_output_enable <= 1;
			end
			SPI_CMD_PP3: begin
				// page program 3-byte
				// write enable latch must be set
				// TODO: check for write protect
				// TODO: buffer the incoming data
				if (spi_status_wrel)
				begin
					// will need RAM
					spi_critical <= 1;
					spi_rd_cmd <= 1; // pretend this is a read
					spi_mode <= MODE_WRITE;
					spi_output_enable <= 1;
				end
			end
			SPI_CMD_ERASE: begin
				// erase sector with 3-byte address (ignored)
				// write enable latch must be set
				if (spi_status_wrel)
				begin
					// will need RAM
					spi_critical <= 1;
					spi_rd_cmd <= 1;
					spi_mode <= MODE_ERASE;
					spi_output_enable <= 1;
				end
			end
			SPI_CMD_RDSR: begin
				spi_output_enable <= 1;
				spi_tx_data <= spi_status_reg;
				spi_tx_strobe <= 1;
			end
			SPI_CMD_RDID: begin
				spi_output_enable <= 1;
				spi_tx_data <= spi_jid0[23:16];
				spi_tx_strobe <= 1;
				spi_rd_cmd <= 1; // pretend this is a read
				spi_mode <= MODE_RDID;
			end
			default: begin
				// log the unknown command with a zero address
				log_addr <= { 16'hC0DE, spi_rx_data };
				log_len <= spi_rx_data;
				log_strobe <= 1;
				spi_critical <= 0;
				spi_output_enable <= 0;
				ram_refresh_inhibit <= 0;
				spi_rd_cmd <= 0;
				errors[6] <= 1;
			end
			endcase
		end else
		if (!spi_rd_cmd) begin
			// nothing to do while !CS is active
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 6 && spi_count == 3 && spi_mode == MODE_READ)
		begin
			// special case for the next to last bit on the incoming
			// address.  we have 23 of the 24 bits, which allow us to
			// pre-fetch a 16-bit wide read and choose later which byte.

			// not yet shifted
			ram_addr[7:0] <= { spi_rx_data[6:0], 1'b0 };
			ram_enable <= 1;
			read_complete <= 0;
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 1 && spi_count == 4 && spi_mode == MODE_READ)
		begin
			// normal case, start a fetch for the next byte when
			// we're partial the way done with this one
			// since we know the address will be the next one
			if (ram_enable)
				errors[4] <= 1;

			ram_enable <= 1;
			read_complete <= 0;
		end else
		if (!spi_rx_strobe)
		begin
			// if we're in erase mode, keep erasing
			if (erase_active)
			begin
				if (erase_len == 0)
				begin
					// last erase is done.
					spi_critical <= 0;
					erase_active <= 0;

					// unset our status register flags
					errors[7] <= 0;
					spi_status_wrel <= 0;
					spi_status_busy <= 0;
				end else
				if (!ram_enable)
				begin
					// erase the next word
					ram_addr <= erase_addr;
					ram_write_mask <= 2'b11; // both words
					ram_write_data <= 16'hFFFF;
					ram_write_enable <= 1;
					ram_enable <= 1;
				end else
				if (ram_data_valid && ram_write_enable) begin
					// write complete, might have been
					// interleaved with reads, so check that
					// this was a write
					errors[7] <= ~errors[7];
					ram_write_enable <= 0;
					ram_enable <= 0;
					erase_len <= erase_len - 2;
					erase_addr <= erase_addr + 2;
				end
			end else
			if (ram_data_valid) begin
				// disable the current read or write. the new one
				// will be started at the end of this byte.
				ram_write_enable <= 0;
				ram_enable <= 0;
				read_complete <= 1;

				// Special case if we missed the rising edge
				// and hope that we raced the falling edge
				if (read_immediate_update)
				begin
					spi_tx_strobe <= 1;
					spi_tx_data <= ram_addr[0]
						? ram_read_data[15:8]
						: ram_read_data[7:0];
				end

				read_immediate_update <= 0;
			end
		end else
		if (spi_count == 1)
		begin
			// only support 24-bit reads right now, but use the
			// top of memory for optional data
			ram_addr[23:16] <= spi_rx_data;
			spi_count <= 2;

			// send the jdid id while everything is happening
			if (spi_mode == MODE_RDID)
			begin
				spi_tx_data <= spi_jid0[15: 8];
				spi_tx_strobe <= 1;
			end
		end else
		if (spi_count == 2)
		begin
			ram_addr[15:8] <= spi_rx_data;
			spi_count <= 3;

			// we have enough to start the SDRAM row activation
			// SDRAM should not be busy since we've paused
			// refresh and asserted the priority flag
			// we don't care about the data, so don't do an update
			if (spi_mode == MODE_READ)
			begin
				ram_enable <= 1;
				read_complete <= 0;
				read_immediate_update <= 0;
			end

			// send the jdid id while everything is happening
			if (spi_mode == MODE_RDID)
			begin
				spi_tx_data <= spi_jid0[ 7: 0];
				spi_tx_strobe <= 1;
			end
		end else
		if (spi_count == 3)
		begin
			// fill in the rest of the address for logging
			log_addr <= { ram_addr[31:8], spi_rx_data };
			log_len <= spi_rd_dummy ? -1 : 0;

			// the next read will start one byte higher
			// for a normal read, otherwise on the same byte for
			// one with a dummy read
			if (spi_rd_dummy)
				ram_addr[ 7: 0] <= spi_rx_data; // start this one
			else
			if (spi_mode == MODE_WRITE)
				ram_addr[ 7: 0] <= spi_rx_data - 1; // start one before
			else
				ram_addr[ 7: 0] <= spi_rx_data + 1; // start one after

			spi_count <= 4;

			if (read_complete || ram_data_valid) begin
				// the read has already returned 16-bits of data to
				// us for either byte. choose which one and setup
				// the TX
				read_complete <= 0;
				read_immediate_update <= 0;
				spi_tx_strobe <= 1;
				spi_tx_data <= spi_rx_data[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];

				if (VERBOSE_LOGGING) begin
					log_strobe <= 1;
					log_len <= spi_rx_data[0]
						? ram_read_data[15:8]
						: ram_read_data[7:0];
				end
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
				//errors[6] <= 1;
				if (VERBOSE_LOGGING) begin
					log_addr <= "LOST";
					log_len <= 8'hAF;
					log_strobe <= 1;
				end
			end

			// if we are in erase mode, start the page erasure
			// len should be configurable, always align start address
			// set the busy flag 
			if (spi_mode == MODE_ERASE && spi_status_wrel)
			begin
				spi_status_busy <= 1;
				erase_active <= 1;
				erase_len <= 16'h01000; // 4096 bytes
				erase_addr <= { ram_addr[31:12], 12'b0 };
			end

		end else
		if (spi_count == 4)
		begin
			// increment the spi_len for logging,
			// leave the log_addr alone so it shows the start address
			log_len <= log_len + 1;
			ram_addr[7:0] <= ram_addr[7:0] + 8'h01;

			if (spi_mode == MODE_WRITE) begin
				// write this into the flash
				// todo: should we implement the bad behaviour
				// of flipping the bits?
				ram_enable <= 1;
				ram_write_enable <= 1;

				// note that this is backwards since ram addr
				// is incremented on this same cycle
				if (ram_addr[0]) begin
					ram_write_mask <= 2'b01;
					ram_write_data <= { 8'h0, spi_rx_data };
				end else begin
					ram_write_mask <= 2'b10;
					ram_write_data <= { spi_rx_data, 8'h0 };
				end
			end else
			if (read_complete || ram_data_valid) begin
				// the read has already returned 16-bits of data to
				// us for either byte. choose which one and setup
				// the TX
				read_complete <= 0;
				read_immediate_update <= 0;
				spi_tx_strobe <= 1;
				spi_tx_data <= ram_addr[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];

				log_len <= cycles;
				cycles <= 0;
				if (VERBOSE_LOGGING) begin
					log_len <= ram_addr[0]
						? ram_read_data[15:8]
						: ram_read_data[7:0];
					//log_len <= ram_addr[7:0];
				end
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
				//errors[5] <= 1;
			end

			// our timing window is no longer critical, so it is
			// ok the allow refresh cycles, but we still have
			// the sdram locked with spi_critical
			ram_refresh_inhibit <= 0;

		end else
		begin
			// error! invalid state wtf
			errors[6] <= 1;
			spi_critical <= 0;
			spi_output_enable <= 0;
		end
	end
endmodule

