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
	output	ram_refresh_inhibit, // asserted when we really need the RAM
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
	reg ram_refresh_inhibit;

	// synchronize the spi_cs line to our clock
	reg [1:0] spi_cs_prev;
	wire spi_cs_rising = !spi_cs_prev[1] && spi_cs_prev[0];
	wire spi_cs_sync = spi_cs_prev[0];
	always @(posedge clk) spi_cs_prev <= { spi_cs_prev[0], spi_cs };

	reg [7:0] cycles;

	// commands are documented in various SPI flash datasheets,
	// such as https://www.winbond.com/resource-files/w25q256fv_revg1_120214_qpi_website_rev_g.pdf
	localparam SPI_CMD_READ	= 8'h03;
	localparam SPI_CMD_WRDS	= 8'h04;
	localparam SPI_CMD_RDSR	= 8'h05;
	localparam SPI_CMD_WREN	= 8'h06;
	localparam SPI_CMD_RDID = 8'h9F;

	reg [23:0] spi_jid0 = { 8'hC2, 8'h20, 8'h17 }; // 8 MB flash chip

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
			ram_refresh_inhibit <= 0;
			ram_read_enable <= 0;
			read_immediate_update <= 0;
		end else
		if (spi_cs_sync) begin
			// no longer asserted, release our locks and signal
			// a logging event if we had one
			spi_critical <= 0;
			spi_count <= 0;
			spi_rd_cmd <= 0;
			ram_refresh_inhibit <= 0;
			ram_read_enable <= 0;
			read_immediate_update <= 0;

			//spi_tx_strobe <= 1;
			//spi_tx_data <= 8'hFF;

			if (spi_cs_rising && spi_rd_cmd && spi_count == 4)
			begin
				//log_strobe <= 1;
			end
		end else
		if (spi_rx_cmd)
		begin
			// first byte of a new transaction
			ram_addr <= 0;
			log_addr <= 0;
			log_len <= 0;

			spi_count <= 1;
			//spi_tx_data <= 8'hF1;
			//spi_tx_strobe <= 1;

			// Anytime a SPI read command starts, assert
			// exclusive access to the SD interface
			case(spi_rx_data)
			SPI_CMD_READ: begin
				spi_critical <= 1;
				ram_refresh_inhibit <= 1;
				spi_rd_cmd <= 1;
			end
			SPI_CMD_WREN: begin
				// write enable. narf
				log_addr <= { 16'hBAD0, spi_rx_data };
				log_strobe <= 1;
				spi_status_wrel <= 1;
				errors[4] <= 1;
			end
			SPI_CMD_WRDS: begin
				// write disable
				spi_status_wrel <= 0;
				log_addr <= { 16'hBAD0, spi_rx_data };
				log_strobe <= 1;
				errors[5] <= 1;
			end
			SPI_CMD_RDSR: begin
				spi_critical <= 1;
				spi_tx_data <= spi_status_reg;
				spi_tx_strobe <= 1;
				spi_rd_cmd <= 1; // pretend this is a read
				log_addr <= { 16'hBAD0, spi_rx_data };
				log_strobe <= 1;
				errors[6] <= 1;
			end
			SPI_CMD_RDID: begin
				spi_critical <= 1;
				spi_tx_data <= spi_jid0[23:16];
				spi_tx_strobe <= 1;
				spi_rd_cmd <= 1; // pretend this is a read
				log_addr <= { 16'hBAD0, spi_rx_data };
				log_strobe <= 1;
				errors[6] <= 1;
			end
			default: begin
				// log the unknown command with a zero address
				log_addr <= { 16'hBAD0, spi_rx_data };
				log_len <= spi_rx_data;
				log_strobe <= 1;
				spi_critical <= 0;
				ram_refresh_inhibit <= 0;
				spi_rd_cmd <= 0;
				errors[7] <= 1;
			end
			endcase
		end else
		if (!spi_rd_cmd) begin
			// nothing to do for non-read commands right now
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 6 && spi_count == 3)
		begin
			// special case for the next to last bit on the incoming
			// address.  we have 23 of the 24 bits, which allow us to
			// pre-fetch a 16-bit wide read and choose later which byte.

			// not yet shifted
			ram_addr[7:0] <= { spi_rx_data[6:0], 1'b0 };
			ram_read_enable <= 1;
			read_complete <= 0;
		end else
		if (spi_rx_bit_strobe && spi_rx_bit == 1 && spi_count == 4)
		begin
			// normal case, start a fetch for the next byte when
			// we're partial the way done with this one
			// since we know the address will be the next one
			if (ram_read_enable)
				errors[4] <= 1;

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

			// send the jdid id while everything is happening
			spi_tx_data <= spi_jid0[15: 8];
			//spi_tx_strobe <= 1;
		end else
		if (spi_count == 2)
		begin
			log_addr[15:7] <= spi_rx_data;
			ram_addr[15:7] <= spi_rx_data;
			spi_count <= 3;

			// we have enough to start the SDRAM row activation
			// SDRAM should not be busy since we've paused
			// refresh and asserted the priority flag
			// we don't care about the data, so don't do an update
			ram_read_enable <= 1;
			read_complete <= 0;
			read_immediate_update <= 0;

			// send the jdid id while everything is happening
			spi_tx_data <= spi_jid0[ 7: 0];
			//spi_tx_strobe <= 1;
		end else
		if (spi_count == 3)
		begin
			// fill in the rest of the address for logging
			log_addr[ 7: 0] <= spi_rx_data;
			ram_addr[ 7: 0] <= spi_rx_data + 1;
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

				log_strobe <= 1;
				log_len <= spi_rx_data[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
				errors[6] <= 1;
				log_addr <= "LOST";
				log_len <= 8'hAF;
				log_strobe <= 1;
			end

		end else
		if (spi_count == 4)
		begin
			// increment the spi_len for logging,
			// leave the log_addr alone so it shows the start address
			log_len <= log_len + 1;
			ram_addr[7:0] <= ram_addr[7:0] + 8'h01;

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

				log_len <= cycles;
				cycles <= 0;
				log_len <= ram_addr[0]
					? ram_read_data[15:8]
					: ram_read_data[7:0];
				//log_len <= ram_addr[7:0];
			end else begin
				// if the read hasn't returned yet, set the
				// immediate update flag and hope it arrives
				// before the falling edge
				read_immediate_update <= 1;
				errors[5] <= 1;
			end

			// our timing window is no longer critical, so it is
			// ok the allow refresh cycles, but we still have
			// the sdram locked with spi_critical
			ram_refresh_inhibit <= 0;
		end else
		begin
			// error! invalid state wtf
			errors[7] <= 1;
			spi_critical <= 0;
		end
	end
endmodule

