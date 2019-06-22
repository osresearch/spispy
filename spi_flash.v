/*
 * SPI Flash device.
 *
 */
`ifndef _spi_flash_v_
`define _spi_flash_v_

module spi_flash(
	input	clk,
	input	reset,

	// spi bus interface
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
	output [31:0] ram_addr,
	output ram_read_enable,
	input [15:0] ram_read_data,
	input ram_read_valid, // when the data is valid

	// logging interface. valid until the next command starts
	output reg [31:0] log_addr,
	output reg [7:0] log_len,
	output reg log_strobe,
	output reg [7:0] errors
);
	reg spi_tx_strobe;
	reg spi_tx_data;

	reg [31:0] ram_addr;
	reg ram_read_enable;
	reg read_complete;
	reg read_immediate_update;

	reg [2:0] spi_count;
	reg spi_rd_cmd;

	// as soon as we detect the start of a read command, lock the sdram
	// control for our exclusive access
	reg spi_critical;

	// SPI command state machine
	always @(posedge clk)
	begin
		log_strobe <= 0;
		spi_tx_strobe <= 0;

		if (reset) begin
			errors <= 0;
			spi_critical <= 0;
			ram_read_enable <= 0;
			read_immediate_update <= 0;
		end else
		if (spi_cs) begin
			// no longer asserted, release our locks and signal
			// a logging event if we had one
			spi_critical <= 0;
			spi_count <= 0;
			spi_rd_cmd <= 0;
			ram_read_enable <= 0;
			read_immediate_update <= 0;

			spi_tx_strobe <= 1;
			spi_tx_data <= 8'hFF;

			if (!reset && spi_rd_cmd && spi_count == 4)
			begin
				log_strobe <= 1;
			end
		end else
		if (spi_rx_cmd)
		begin
			// first byte of a new transaction
			ram_addr <= 0;
			log_addr <= 0;
			log_len <= 0;

			spi_count <= 1;
			spi_tx_data <= 8'hF1;
			spi_tx_strobe <= 1;

			// Anytime a SPI read command starts, assert
			// exclusive access to the SD interface
			if (spi_rx_data == 8'h03) begin
				spi_critical <= 1;
				spi_rd_cmd <= 1;
			end else begin
				// log the unknown command with a zero address
				log_len <= spi_rd_cmd;
				log_strobe <= 1;
				spi_critical <= 0;
				spi_rd_cmd <= 0;
			end
		end else
		if (!spi_rd_cmd) begin
			// nothing to do for non-read commands right now
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 6 && spi_count == 3)
		begin
			// special case for the next to last bit on the incoming
			// address.  we have 23 of the 24 bits, which allow us to
			// fetch a 16-bit wide read.

			// not yet shifted
			ram_addr[7:0] <= { spi_rx_data[6:0], 1'b0 };
			ram_read_enable <= 1;
			read_complete <= 0;
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 4 && spi_count == 4)
		begin
			// normal case, start a fetch for the next byte when
			// we're half way done with this one
			// since we know the address will be the next one

			ram_addr[7:0] <= ram_addr[7:0] + 1;
			ram_read_enable <= 1;
			read_complete <= 0;
		end else
		if (!spi_rx_strobe)
		begin
			// so disable the current read. the new one
			// will be started at the end of this byte.
			if (ram_read_valid) begin
				ram_read_enable <= 0;
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
			// only support 24-bit reads
			log_addr[23:16] <= spi_rx_data;
			ram_addr[23:16] <= spi_rx_data;
			spi_count <= 2;
			spi_tx_data <= 8'hF2;
			spi_tx_strobe <= 1;
		end else
		if (spi_count == 2)
		begin
			log_addr[15:7] <= spi_rx_data;
			ram_addr[15:7] <= spi_rx_data;
			spi_count <= 3;

			// we have enough to start the SDRAM activation
			// SDRAM should not be busy since we've paused
			// refresh and asserted the priority flag
			// we don't care about the data, so don't do an update
			ram_read_enable <= 1;
			read_complete <= 0;
			read_immediate_update <= 0;

			spi_tx_data <= 8'hF3;
			spi_tx_strobe <= 1;
		end else
		if (spi_count == 3)
		begin
			// fill in the rest of the address for logging
			log_addr[ 7: 0] <= spi_rx_data;
			ram_addr[ 7: 0] <= spi_rx_data;
			spi_count <= 4;

			if (read_complete || ram_read_valid) begin
				// the read has already returned 16-bits of data to
				// us for either byte. choose which one and setup
				// the TX
				read_complete <= 0;
				read_immediate_update <= 0;
				spi_tx_strobe <= 1;
				spi_tx_data <= spi_rx_data[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
			end
		end else
		if (spi_count == 4)
		begin
			// increment the spi_len for logging,
			// leave the log_addr alone so it shows the start address
			log_len <= log_len + 1;

			if (read_complete || ram_read_valid) begin
				// the read has already returned 16-bits of data to
				// us for either byte. choose which one and setup
				// the TX
				read_complete <= 0;
				read_immediate_update <= 0;
				spi_tx_strobe <= 1;
				spi_tx_data <= ram_addr[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
			end
		end else
		begin
			// error! invalid state wtf
			errors[7] <= 1;
			spi_critical <= 0;
		end
	end
endmodule

