`include "usb_fs_pe.v"
`include "usb_serial_ctrl_ep.v"
`include "usb_serial_ep.v"

module usb_serial (
  input  clk_48mhz,
  input  clk,
  input  reset,

  // USB lines in clk_48mhz domain.
  // Split into input vs. output and oe control signal to maintain
  // highest level of compatibility with synthesis tools.
  output usb_p_tx,
  output usb_n_tx,

  input  usb_p_rx,
  input  usb_n_rx,

  output usb_tx_en,

  // serial uart interface, runs at clk domain
  output uart_tx_ready,
  input [7:0] uart_tx_data,
  input uart_tx_strobe,

  output [7:0] uart_rx_data,
  output uart_rx_strobe,

  output host_presence // need to figure this out
);
  wire [6:0] dev_addr;
  wire [7:0] out_ep_data;

  wire ctrl_out_ep_req;
  wire ctrl_out_ep_grant;
  wire ctrl_out_ep_data_avail;
  wire ctrl_out_ep_setup;
  wire ctrl_out_ep_data_get;
  wire ctrl_out_ep_stall;
  wire ctrl_out_ep_acked;

  wire ctrl_in_ep_req;
  wire ctrl_in_ep_grant;
  wire ctrl_in_ep_data_free;
  wire ctrl_in_ep_data_put;
  wire [7:0] ctrl_in_ep_data;
  wire ctrl_in_ep_data_done;
  wire ctrl_in_ep_stall;
  wire ctrl_in_ep_acked;

  wire serial_out_ep_req;
  wire serial_out_ep_grant;
  wire serial_out_ep_data_avail;
  wire serial_out_ep_setup;
  wire serial_out_ep_data_get;
  wire serial_out_ep_stall;
  wire serial_out_ep_acked;

  wire serial_in_ep_req;
  wire serial_in_ep_grant;
  wire serial_in_ep_data_free;
  wire serial_in_ep_data_put;
  wire [7:0] serial_in_ep_data;
  wire serial_in_ep_data_done;
  wire serial_in_ep_stall;
  wire serial_in_ep_acked;

  wire sof_valid;
  wire [10:0] frame_index;

`undef SERIAL
`ifdef SERIAL
  // signal if we have data available
  // note that the "in" and "out" are from the HOST perspective,
  // while "rx" and "tx" are from the FPGA perspective

  // the grant and data available go high one clock cycle before
  // the data is actually available.  this works for receiving data
  wire rx_ready = serial_out_ep_grant && serial_out_ep_data_avail;
  delay #(.DELAY(1)) rx_ready_delay(clk, rx_ready, uart_rx_ready);

  assign serial_out_ep_req = serial_out_ep_data_avail;
  assign serial_out_ep_data_get = serial_out_ep_grant && uart_rx_strobe;
  assign uart_rx_data = out_ep_data;
  assign serial_out_ep_stall = 0;

  // buffer up to 512 bytes of outgoing data (one BRAM)
  reg [7:0] tx_fifo[511:0];
  reg [8:0] tx_read_ptr;
  reg [8:0] tx_write_ptr;
  wire tx_data_available = tx_write_ptr != tx_read_ptr;
  assign uart_tx_ready = (tx_write_ptr+1) != tx_read_ptr;
  assign serial_in_ep_stall = 0;

  delay #(.DELAY(0)) in_req_delay(clk, tx_data_available, serial_in_ep_req);

  wire data_done = !serial_in_ep_data_free || !tx_data_available;
  //delay #(.DELAY(2)) data_done_delay(clk, data_done, serial_in_ep_data_done);
  assign serial_in_ep_data_done = data_done;
  wire in_ready_i = serial_in_ep_data_free && serial_in_ep_grant;
  reg in_ready;
  delay #(.DELAY(0)) in_ready_delay(clk, in_ready_i, in_ready);

  reg data_put;
  //reg serial_in_ep_data_put;
  delay #(.DELAY(0)) data_put_delay(clk, data_put, serial_in_ep_data_put);
  //assign serial_in_ep_data_put = data_put;

  always @(posedge clk) if (!reset)
  begin
	data_put <= 0;

	if (uart_tx_strobe && uart_tx_ready)
	begin
		tx_fifo[tx_write_ptr] <= uart_tx_data;
		tx_write_ptr <= tx_write_ptr + 1;
	end

	if (tx_data_available)
	begin
		data_put <= 1;
		serial_in_ep_data <= tx_fifo[tx_read_ptr];
		if (in_ready)
			tx_read_ptr <= tx_read_ptr + 1;
	end
  end
`else

  usb_serial_ep usb_serial_ep_inst (
    .clk(clk),
    .reset(reset),

    // out endpoint interface 
    .out_ep_req(serial_out_ep_req),
    .out_ep_grant(serial_out_ep_grant),
    .out_ep_data_avail(serial_out_ep_data_avail),
    .out_ep_setup(serial_out_ep_setup),
    .out_ep_data_get(serial_out_ep_data_get),
    .out_ep_data(out_ep_data),
    .out_ep_stall(serial_out_ep_stall),
    .out_ep_acked(serial_out_ep_acked),

    // in endpoint interface 
    .in_ep_req(serial_in_ep_req),
    .in_ep_grant(serial_in_ep_grant),
    .in_ep_data_free(serial_in_ep_data_free),
    .in_ep_data_put(serial_in_ep_data_put),
    .in_ep_data(serial_in_ep_data),
    .in_ep_data_done(serial_in_ep_data_done),
    .in_ep_stall(serial_in_ep_stall),
    .in_ep_acked(serial_in_ep_acked),

    // uart interface
    .uart_tx_ready(uart_tx_ready),
    .uart_tx_data(uart_tx_data),
    .uart_tx_strobe(uart_tx_strobe),
    .uart_rx_data(uart_rx_data),
    .uart_rx_strobe(uart_rx_strobe)
  );
`endif

  usb_serial_ctrl_ep ctrl_ep_inst (
    .clk(clk),
    .reset(reset),
    .dev_addr(dev_addr),

    // out endpoint interface 
    .out_ep_req(ctrl_out_ep_req),
    .out_ep_grant(ctrl_out_ep_grant),
    .out_ep_data_avail(ctrl_out_ep_data_avail),
    .out_ep_setup(ctrl_out_ep_setup),
    .out_ep_data_get(ctrl_out_ep_data_get),
    .out_ep_data(out_ep_data),
    .out_ep_stall(ctrl_out_ep_stall),
    .out_ep_acked(ctrl_out_ep_acked),

    // in endpoint interface 
    .in_ep_req(ctrl_in_ep_req),
    .in_ep_grant(ctrl_in_ep_grant),
    .in_ep_data_free(ctrl_in_ep_data_free),
    .in_ep_data_put(ctrl_in_ep_data_put),
    .in_ep_data(ctrl_in_ep_data),
    .in_ep_data_done(ctrl_in_ep_data_done),
    .in_ep_stall(ctrl_in_ep_stall),
    .in_ep_acked(ctrl_in_ep_acked)
  );

  wire nak_in_ep_grant;
  wire nak_in_ep_data_free;
  wire nak_in_ep_acked;

  usb_fs_pe #(
    .NUM_OUT_EPS(5'd2),
    .NUM_IN_EPS(5'd3)
  ) usb_fs_pe_inst (
    .clk_48mhz(clk_48mhz),
    .clk(clk),
    .reset(reset),

    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en),

    .dev_addr(dev_addr),

    // out endpoint interfaces 
    .out_ep_req({serial_out_ep_req, ctrl_out_ep_req}),
    .out_ep_grant({serial_out_ep_grant, ctrl_out_ep_grant}),
    .out_ep_data_avail({serial_out_ep_data_avail, ctrl_out_ep_data_avail}),
    .out_ep_setup({serial_out_ep_setup, ctrl_out_ep_setup}),
    .out_ep_data_get({serial_out_ep_data_get, ctrl_out_ep_data_get}),
    .out_ep_data(out_ep_data),
    .out_ep_stall({serial_out_ep_stall, ctrl_out_ep_stall}),
    .out_ep_acked({serial_out_ep_acked, ctrl_out_ep_acked}),

    // in endpoint interfaces 
    .in_ep_req({1'b0, serial_in_ep_req, ctrl_in_ep_req}),
    .in_ep_grant({nak_in_ep_grant, serial_in_ep_grant, ctrl_in_ep_grant}),
    .in_ep_data_free({nak_in_ep_data_free, serial_in_ep_data_free, ctrl_in_ep_data_free}),
    .in_ep_data_put({1'b0, serial_in_ep_data_put, ctrl_in_ep_data_put}),
    .in_ep_data({8'b0, serial_in_ep_data[7:0], ctrl_in_ep_data[7:0]}),
    .in_ep_data_done({1'b0, serial_in_ep_data_done, ctrl_in_ep_data_done}),
    .in_ep_stall({1'b0, serial_in_ep_stall, ctrl_in_ep_stall}),
    .in_ep_acked({nak_in_ep_acked, serial_in_ep_acked, ctrl_in_ep_acked}),

    // sof interface
    .sof_valid(sof_valid),
    .frame_index(frame_index)
  );

endmodule
