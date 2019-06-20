/** \file
 * SPI bus emulator.
 *
 * This uses the onboard DRAM to store a flash image and can provide
 * it over a SPI bus to an external device.
 *
 * This also configures the serial port at 3 Mb/s for the command
 * protocol in user.v.
 *
 * FLCOMP in the IFD should be adjusted to use single SPI at 17 MHz.
 * Offset 0x30.  SPI_FREQUENCY_17MHZ=6 SPI_FREQUENCY_20MHZ=0
 * bit 30 = 0 == no dual fast read support
 * bit 20 = 0 == no fast read support
 * speed offsets 17, 21, 24, 27
 * 0x00 10 00 24
 *
 * 20 17
 * 1 000 00000000000100100
 *
 * minnow board dediprog pinout:
 *             VCC  GND    Black
 *      White  CS   CLK    Green / Orange
 *      Brown  MISO MOSI   Yellow
 *             NC   NC
 */
`default_nettype none
`include "util.v"
`include "uart.v"
`include "gpio.v"
`include "pll_120.v"
`include "pll_96.v"
`include "sdram_controller.v"
`include "user.v"
`include "spi200.v"

module top(
	input clk_100mhz,
	//inout [7:0] pmod1, // serial port
	output pmod1_0,
	input pmod1_1,
	output pmod1_2,
	output pmod1_3,
	output pmod1_4,
	output pmod1_5,
	output pmod1_6,
	inout [7:0] pmod2, // spi flash clip

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
	localparam EMULATE = 0;

	// beaglewire pinouts
	wire serial_txd_pin	= pmod1_0; // pmod1[0];
	wire serial_rxd_pin	= pmod1_1; // pmod1[1];

	wire spi_clk_pin	= pmod2[0];
	wire spi_cs_pin		= pmod2[1];
	wire spi_miso_pin	= pmod2[2];
	wire spi_mosi_pin	= pmod2[3];


	wire locked, clk;
	wire reset = !locked;
`undef CLK120
`ifdef CLK120
	wire clk_120mhz;
	pll_120 pll120(clk_100mhz, clk_120mhz, locked);
	assign clk = clk_120mhz;
`else
	wire clk_96mhz;
	pll_96 pll96(clk_100mhz, clk_96mhz, locked);
	//assign clk = clk_96mhz;
	assign clk = clk_100mhz;
`endif

	parameter ADDR_BITS = 25; // 32 MB SDRAM chip

	// arbitrate between the user commands and the SPI bus when
	// we're in a critcal section.
	reg spi_critical;
	wire [ADDR_BITS-1:0] spi_sd_addr;
	wire [7:0] spi_sd_wr_data = 0;
	reg spi_sd_enable;
	wire spi_sd_we = 0;
	wire spi_sd_busy = sd_busy;
	wire spi_rd_ready = spi_critical ? sd_rd_ready : 0;
	wire spi_sd_refresh_inhibit = spi_critical;

	wire [ADDR_BITS-1:0] user_sd_addr;
	wire [7:0] user_sd_wr_data;
	wire user_sd_enable;
	wire user_sd_we;
	wire user_sd_busy = spi_critical ? 1 : sd_busy;
	wire user_rd_ready = sd_rd_ready;
	wire user_sd_refresh_inhibit;

	// sdram logical interface
	wire sd_pause_cas;
	wire [ADDR_BITS-1:0] sd_addr = spi_critical ? spi_sd_addr : user_sd_addr;
	wire [7:0] sd_wr_data = spi_critical ? spi_sd_wr_data : user_sd_wr_data;
	wire [7:0] sd_rd_data;
	wire [7:0] sd_rd_data_raw;
	wire sd_rd_ready_raw;
	wire sd_we = spi_critical ? spi_sd_we : user_sd_we;
	wire sd_enable = spi_critical ? spi_sd_enable : user_sd_enable;
	wire sd_refresh_inhibit = spi_critical ? spi_sd_refresh_inhibit : user_sd_refresh_inhibit;

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
		.refresh_inhibit(sd_refresh_inhibit),
		.pause_cas(sd_pause_cas),
		.wr_addr(sd_addr),
		.wr_enable(sd_enable & sd_we),
		.wr_data(sd_wr_data),
		.rd_addr(sd_addr),
		.rd_enable(sd_enable & !sd_we),
		.rd_data(sd_rd_data),
		.rd_ready(sd_rd_ready),
		.rd_data_raw(sd_rd_data_raw),
		.rd_ready_raw(sd_rd_ready_raw),
		.busy(sd_busy),
	);

	wire serial_txd;
	wire serial_rxd;

	gpio gpio_txd(
		.enable(1), // always on
		.pin(serial_txd_pin),
		.out(serial_txd),
	);

	gpio #(.PULLUP(1)) gpio_rxd(
		.enable(0), // always input, with pullup
		.pin(serial_rxd_pin),
		.in(serial_rxd),
		.out(1),
	);


	// generate a 3 MHz/12 MHz serial clock from the 96 MHz clock
	// this is the 3 Mb/s maximum supported by the FTDI chip
	// note that some Linux tools can have trouble keeping up with
	// this data rate; xxd -g1 for instance will drop bytes on
	// long bursts (more than 1KB).
	wire clk_3mhz, clk_12mhz;
	//divide_by_n #(.N(8)) div1(clk, reset, clk_12mhz);
	//divide_by_n #(.N(32)) div4(clk, reset, clk_3mhz);

`ifdef CLK120
/*
	// 3 megabaud @ 120 MHz
	divide_by_n #(.N(10)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(40)) div4(clk, reset, clk_3mhz);
*/
	// 1 megabaud @ 120 MHz
	divide_by_n #(.N(30)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(120)) div4(clk, reset, clk_3mhz);
`else
/*
	// 3 megabaud @ 96 MHz
	divide_by_n #(.N(8)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(32)) div4(clk, reset, clk_3mhz);
*/
	// 1 megabaud @ 100 MHz
	divide_by_n #(.N(25)) div1(clk, reset, clk_12mhz);
	divide_by_n #(.N(100)) div4(clk, reset, clk_3mhz);
`endif


	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;
	wire uart_txd_ready;
	reg [7:0] uart_txd;
	reg uart_txd_strobe;

`define UART_FIFO
`ifdef UART_FIFO
	uart_tx_fifo #(
		.NUM(512),
		.FREESPACE(15'h32), // ensure there is always space
	)
`else
	uart_tx
`endif
	txd (
		.clk(clk),
		.reset(reset),
		.baud_x1(clk_3mhz),
		.serial(serial_txd),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe),
		.ready(uart_txd_ready)
	);

	uart_rx rxd(
		.clk(clk),
		.reset(reset),
		.baud_x4(clk_12mhz),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	// SPI flash
	// in controller mode all are outputs except miso
	// in spispy mode all of them are inputs.
	// in emulation mode the cs and miso are in/out based on our mode
	// in toctou mode they can all be in/out
	// cs is driven high to deselect the flash chip
	// and miso is driven with the fpga provided data

	// if we're faking the !CS pin (driving the output high)
	// tell our SPI device that it is still selected.
	reg spi_cs_enable = 0;
	wire fake_spi_cs = spi_cs_enable ? 0 : spi_cs_in;

	reg spi_cs_out = 1; // always high
	reg spi_clk_out = 0;
	reg spi_mosi_out = 0;
	wire spi_miso_out; // controlled by spi_device

	wire spi_cs_in;
	wire spi_clk_in;
	wire spi_mosi_in;
	wire spi_miso_in;

	gpio #(.PULLUP(1)) gpio_spi_cs(
		.enable(spi_cs_enable && EMULATE),
		.pin(spi_cs_pin),
		.in(spi_cs_in),
		.out(spi_cs_out),
	);

	gpio gpio_spi_clk(
		.enable(0), // always input, until we have a reader built in
		.pin(spi_clk_pin),
		.in(spi_clk_in),
		.out(spi_clk_out),
	);

	gpio gpio_spi_mosi(
		.enable(0), // always input, until we have a reader
		.pin(spi_mosi_pin),
		.in(spi_mosi_in),
		.out(spi_mosi_out),
	);

	gpio gpio_spi_miso(
		.enable(spi_cs_enable && EMULATE),
		.pin(spi_miso_pin),
		.in(spi_miso_in),
		.out(spi_miso_out),
	);

/*
	gpio gpio_debug1(
		.enable(1), // always on
		.pin(pmod1[2]),
		.out(spi_cs_pin)
	);
	gpio gpio_debug2(
		.enable(1), // always on
		.pin(pmod1[3]),
		.out(spi_clk_pin),
	);
*/
/*
	reg pmod1_2, pmod1_3; // , pmod1_4, pmod1_5;
	always @(posedge clk) begin
		//pmod1_2 <= spi_rx_strobe; //spi_cs_in;
		pmod1_3 <= spi_critical;
		pmod1_4 <= spi_miso_out;
		pmod1_5 <= spi_miso_in;
	end
*/
	reg trigger = 0;
	assign pmod1_2 = trigger; // spi_cs_in;
	assign pmod1_3 = spi_clk_in;
	assign pmod1_4 = spi_miso_out;
	assign pmod1_5 = spi_miso_in;
	assign pmod1_6 = spi_critical;

	//assign pmod1_2 = spi_cs_in;
	//assign pmod1_3 = spi_clk_in;

	wire spi_rx_cmd;
	wire spi_rx_strobe;
	wire [7:0] spi_rx_data;
	reg [7:0] spi_tx_data = 0;
	reg spi_tx_strobe;
	reg spi_tx_strobe_immediate;

	// for timing reasons the top bit of the incoming SDRAM data is wired
	// directly to the MISO line so that the read request can meet the
	// timing requirements of the SPI bus.
	wire spi_miso_out_clocked;
	reg spi_fast_first_bit;
	assign spi_miso_out =
		spi_fast_first_bit
		? sd_rd_data_raw[7]
		: spi_miso_out_clocked;

	spi_device spi(
		.clk(clk),
		.reset(reset),
		.spi_clk(spi_clk_in),
		.spi_cs(fake_spi_cs),
		.spi_miso(spi_miso_out_clocked),
		.spi_mosi(spi_mosi_in),
		.spi_rx_strobe(spi_rx_strobe),
		.spi_rx_cmd(spi_rx_cmd),
		.spi_rx_data(spi_rx_data),

		.spi_tx_data(spi_tx_data),
		.spi_tx_strobe(spi_tx_strobe),
		.spi_tx_strobe_immediate(spi_tx_strobe_immediate),
	);

	// serial output arbitrator between the user command
	// parser and the spi device decoder
	reg [7:0] spi_data_buffer;
	reg [2:0] spi_data_pending;
	reg [7:0] user_data_buffer;
	reg user_data_pending;
	wire user_txd_strobe;
	wire [7:0] user_txd_data;
	wire user_txd_ready = !user_data_pending;

	reg [2:0] spi_count = 0;
	reg spi_log_this = 0; // don't add a new transaction if there is no space available

	reg [31:0] spi_cmd;
	reg spi_rd_cmd;
	reg spi_first_bit; // has the first bit been read?
	reg spi_first_byte; // are we sending the first byte?
	reg spi_start_read;

	reg [ADDR_BITS-1:0] spi_sd_addr_reg = 0;

	// the lowest byte in the address is hard-wired to the incoming SPI
	// data so that we don't spend a clock-cycle propagating this into the
	// register.  it will be set on the next clock for addition.
	assign spi_sd_addr[24] = 0;
	assign spi_sd_addr[23:16] = spi_sd_addr_reg[23:16];
	assign spi_sd_addr[15: 8] = spi_sd_addr_reg[15:8];
	assign spi_sd_addr[ 7: 0] = spi_first_byte ? spi_rx_data : spi_sd_addr_reg[7:0];
	//assign spi_sd_addr = spi_sd_addr_reg;

	// release the pause on the same cycle as spi_rx_strobe goes high and
	// count is equal to 3.
	assign sd_pause_cas = spi_critical && spi_rd_cmd &&
		( spi_count < 3'h3 || (spi_count == 3'h3 && !spi_rx_strobe) );

	// track how long it has been since the last SPI clock transition
	// this is necessary to detect when the host is no longer reading from us
	// when clk_count == 0, it has been too long
	reg [1:0] spi_clk_sync;
	reg [7:0] spi_clk_count;
	reg [2:0] spi_bit;
	wire spi_clk_rising = !spi_clk_sync[1] && spi_clk_sync[0];
	wire spi_clk_falling = spi_clk_sync[1] && !spi_clk_sync[0];
	reg spi_clk_timeout;
	always @(posedge clk)
	begin
		spi_clk_sync <= { spi_clk_sync[0], spi_clk_in };
		spi_clk_timeout <= 0;

		if (!spi_cs_enable
		|| (spi_clk_sync[1] && !spi_clk_sync[0]))
			spi_clk_count <= 7;
		else
		if (spi_clk_count == 0)
			spi_clk_timeout <= 1;
		else
			spi_clk_count <= spi_clk_count - 1;

		if (fake_spi_cs)
			spi_bit <= 7;
		else
		if (spi_clk_falling)
			spi_bit <= spi_bit - 1;

		// take control of the first bit and hold it
		if (fake_spi_cs)
			spi_fast_first_bit <= 0;
		else
		if (spi_first_bit && !spi_fast_first_bit)
			spi_fast_first_bit <= 1;
		else
		if (spi_fast_first_bit && spi_clk_falling && spi_bit == 7)
			spi_fast_first_bit <= 0;
	end



	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		spi_sd_enable <= 0;
		//spi_tx_strobe <= 0;
		//spi_tx_strobe_immediate <= 0;
		//spi_critical <= !fake_spi_cs;
		trigger <= 0;

		if (spi_clk_timeout) begin
			// too long without a clock, ensure that
			// we do not hold control of the bus.
			spi_cs_enable <= 0;
		end else
		if (fake_spi_cs) begin
			// no longer asserted, release our locks
			spi_critical <= 0;
			//sd_pause_cas <= 0;
			spi_tx_data <= 8'hFF;
			spi_first_byte <= 0;
			spi_first_bit <= 0;
			if (spi_rd_cmd && spi_count == 4)
				spi_data_pending <= 4;
			spi_count <= 0;
		end else
		if (spi_rx_strobe)
		begin
			if (spi_rx_cmd)
			begin
				// if there is no space, then stop logging
				spi_log_this <= uart_txd_ready;
				spi_count <= 1;
				//spi_cmd[31:24] <= spi_rx_data;
				spi_sd_addr_reg[23:0] <= ~0;

				// Anytime a SPI read command starts, assert
				// exclusive access to the SD interface
				if (spi_rx_data == 8'h03) begin
					spi_critical <= 1;
					spi_rd_cmd <= 1;
				end else begin
					spi_critical <= 0;
					spi_rd_cmd <= 0;
				end

				spi_tx_data <= 8'hF0;
				spi_tx_strobe <= ~spi_tx_strobe;
			end else
			if (spi_count == 1) begin
				spi_cmd[31:24] <= spi_rx_data;
				spi_sd_addr_reg[23:16] <= spi_rx_data;
				spi_count <= 2;
				spi_tx_data <= 8'hF2;
				spi_tx_strobe <= ~spi_tx_strobe;
			end else
			if (spi_count == 2) begin
				spi_cmd[23:16] <= spi_rx_data;
				spi_sd_addr_reg[15:8] <= spi_rx_data;
				spi_count <= 3;
				spi_tx_data <= 8'hFE;
				spi_tx_strobe <= ~spi_tx_strobe;

				// we have enough to start the SDRAM activation
				// SDRAM should not be busy since we've paused
				// refresh and asserted the priority flag
				// sd_pause_cas will be asserted by combinatorial
				// logic so that it does not take a cycle to unset
				if (spi_rd_cmd) begin
					//sd_pause_cas <= 1;
					spi_sd_enable <= 1;
					trigger <= (sd_busy ? 1 : 0);

					// signal that we want the address to come
					// direct from spi_rx_data
					spi_first_byte <= 1;

					// let's take over the MISO line
					spi_cs_enable <= 1;
				end
			end else
			if (spi_count == 3) begin
				spi_cmd[15: 8] <= spi_rx_data;
				spi_cmd[ 7: 0] <= 0; // len
				spi_sd_addr_reg[7:0] <= spi_rx_data;
				//spi_cmd <= { 7'h00, spi_sd_addr_reg };
				spi_count <= 4;

				// default to pull the pin high
				// this will show a glitch if the new data is not
				// available on time.
				spi_tx_data <= 8'hFF;
				spi_tx_strobe <= ~spi_tx_strobe;

				// we have the full address to read, release the
				// CAS pause and let it finish the read command
				if (spi_rd_cmd) begin
					spi_first_bit <= 1;

					// pause will be turned off automatically
					//sd_pause_cas <= 0;
				end
			end else begin
				// if sd_first_byte is still set, this is an error
				// it means that the SDRAM never returned a result.
				//trigger <= (spi_first_byte != 0);
				spi_first_byte <= 0;

				// clock out the most recently read SDRAM data
				// it should be ready, since it was started at least
				// 8 SPI clocks ago
				spi_tx_data <= sd_rd_data;
				spi_tx_strobe <= ~spi_tx_strobe;

				// start a new read in case they continue this burst
				//spi_sd_addr_reg <= spi_sd_addr_reg + 1;
				spi_sd_addr_reg <= spi_sd_addr_reg + 1;
				spi_sd_enable <= 1;

				// track the number of bytes read
				//spi_cmd[7:0] <= spi_cmd[7:0] + 1;
			end
		end else
		if (sd_rd_ready_raw && spi_first_bit) begin
			// the sdram has produced data ready for this first byte,
			// clock it to the SPI bus immediately
			trigger <= ( spi_sd_addr[23:0] == 24'h0010);

			spi_cmd[7:0] <= sd_rd_data_raw;
			spi_tx_data <= sd_rd_data_raw;
			spi_tx_strobe_immediate <= ~spi_tx_strobe_immediate;

			// can't start a new read on this cycle (the SDRAM is
			// still busy since this was triggered on the raw output).
			spi_start_read <= 1;
			//trigger <= 1; // (spi_sd_addr == 25'h00000010);
		end else
		if (spi_start_read && !spi_sd_busy) begin
			// start a new read (which will be clocked out when
			// this byte is done)
			spi_sd_addr_reg[7:0] <= spi_sd_addr_reg[7:0] + 1;
			spi_sd_enable <= 1;

			spi_start_read <= 0;
			spi_first_bit <= 0;
		end
		if (spi_data_pending && uart_txd_ready) begin
			if (spi_data_pending == 4)
				uart_txd <= spi_cmd[31:24];
			else
			if (spi_data_pending == 3)
				uart_txd <= spi_cmd[23:16];
			else
			if (spi_data_pending == 2)
				uart_txd <= spi_cmd[15: 8];
			else
			if (spi_data_pending == 1) begin
				uart_txd <= spi_cmd[ 7: 0];
				spi_data_pending <= 0;
			end

			spi_data_pending <= spi_data_pending - 1;
			uart_txd_strobe <= 1;

			// if we have received all zeros, don't output it
			if (spi_cmd == 0) begin
				spi_data_pending <= 0;
				uart_txd_strobe <= 0;
			end
		end

		if (user_txd_strobe) begin
			user_data_buffer <= user_txd_data;
			user_data_pending <= 1;
		end else
		if (user_data_pending && uart_txd_ready && !spi_data_pending) begin
			uart_txd <= user_data_buffer;
			uart_txd_strobe <= 1;
			user_data_pending <= 0;
		end
	end


	// user command parser pulls in data from SPI
	// and from the serial port, drives the SDRAM
	user_command_parser #(
		.ADDR_BITS(ADDR_BITS)
	) parser(
		.clk(clk),
		.reset(reset),
		// serial I/O
		.uart_rxd(uart_rxd),
		.uart_rxd_strobe(uart_rxd_strobe),
		.uart_txd(user_txd_data),
		.uart_txd_strobe(user_txd_strobe),
		.uart_txd_ready(user_txd_ready),
		// SDRAM
		.sd_refresh_inhibit(user_sd_refresh_inhibit),
		.sd_addr(user_sd_addr),
		.sd_wr_data(user_sd_wr_data),
		.sd_rd_data(sd_rd_data),
		.sd_rd_ready(sd_rd_ready),
		.sd_we(user_sd_we),
		.sd_enable(user_sd_enable),
		.sd_busy(user_sd_busy),
	);
endmodule
