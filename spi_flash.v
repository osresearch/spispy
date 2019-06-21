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

	// memory interface
	output	spi_critical, // asserted when we need to lock the memory bus
	output [31:0] ram_addr,
	output ram_read_enable,
	input [7:0] ram_read_data, // should be 31:0 for quad spi?
	input ram_read_valid, // when the data is valid

	// logging interface. valid until the next command starts
	output reg [31:0] log_addr,
	output reg [7:0] log_len,
	output reg log_strobe
);
	// immediately route incoming ram data to the spi output
	assign spi_tx_strobe = ram_read_valid;
	assign spi_tx_data = ram_read_pending ? ram_read_data : 0;
	reg ram_read_pending;

	reg [31:0] ram_addr;
	reg ram_read_enable;

	reg [2:0] spi_count;
	reg spi_rd_cmd;

	reg spi_critical;

	// SPI command state machine
	always @(posedge clk)
	begin
		log_strobe <= 0;

		if (reset || spi_cs) begin
			// no longer asserted, release our locks and signal
			// a logging event if we had one
			spi_critical <= 0;
			spi_count <= 0;
			ram_read_pending <= 0;
			ram_read_enable <= 0;

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

			// Anytime a SPI read command starts, assert
			// exclusive access to the SD interface
			if (spi_rx_data == 8'h03) begin
				spi_critical <= 1;
				spi_rd_cmd <= 1;
			end else begin
				spi_critical <= 0;
				spi_rd_cmd <= 0;
			end
		end else
		if (!spi_rd_cmd) begin
			// nothing to do for non-read commands right now
		end else
		if (!spi_rx_strobe)
		begin
			// if the RAM has returned a result, then start a new read
			// we are fairly confident that this can't happen
			// when a SPI byte is also incoming since the timings
			// should overlap
			if (spi_rd_cmd && ram_read_valid)
				ram_addr <= ram_addr + 1;

			// the incoming RAM read is wired directly to the output SPI
			ram_read_enable <= 0;
			ram_read_pending <= 0;
		end else
		if (spi_count == 1)
		begin
			// only support 24-bit reads
			log_addr[23:16] <= spi_rx_data;
			ram_addr[23:16] <= spi_rx_data;
			spi_count <= 2;
		end else
		if (spi_count == 2)
		begin
			log_addr[15:7] <= spi_rx_data;
			ram_addr[15:7] <= spi_rx_data;
			spi_count <= 3;

			// we have enough to start the SDRAM activation
			// SDRAM should not be busy since we've paused
			// refresh and asserted the priority flag
			ram_read_enable <= 1;
		end else
		if (spi_count == 3)
		begin
			log_addr[ 7: 0] <= spi_rx_data;
			ram_addr[ 7: 0] <= spi_rx_data;

			// the RAM should be primed and ready for this new
			// read, so start the new read and hope we have a result
			// before the next falling edge
			ram_read_enable <= 1;
			spi_count <= 4;
			ram_read_pending <= 1;

			// we could save another clock cycle by routing the
			// ram_read_enable latch to spi_rx_strobe
		end else
		begin
			// increment the spi_len for logging,
			// leave the spi_addr alone so it shows the start address
			log_len <= log_len + 1;

			// start a new read on the next byte
			ram_addr <= ram_addr + 1;
			ram_read_enable <= 1;
			ram_read_pending <= 1;
		end
	end
endmodule

