module usb_serial_ep (
  input clk,
  input reset,


  ////////////////////
  // out endpoint interface 
  ////////////////////
  output out_ep_req,
  input out_ep_grant,
  input out_ep_data_avail,
  input out_ep_setup,
  output out_ep_data_get,
  input [7:0] out_ep_data,
  output out_ep_stall,
  input out_ep_acked,


  ////////////////////
  // in endpoint interface 
  ////////////////////
  output reg in_ep_req = 0,
  input in_ep_grant,
  input in_ep_data_free,
  output reg in_ep_data_put = 0,
  output [7:0] in_ep_data,
  output reg in_ep_data_done = 0,
  output in_ep_stall,
  input in_ep_acked,


  ////////////////////
  // uart interface, runs at clk domain
  ////////////////////
  output uart_tx_ready,
  input [7:0] uart_tx_data,
  input uart_tx_strobe,

  output [7:0] uart_rx_data,
  output uart_rx_strobe
);
  // never stall the USB
  // todo: allow backpressure to stall the serial link
  assign out_ep_stall = 1'b0;
  assign in_ep_stall = 1'b0;

  ////////////////////////////////////////////////////////////////////////////////
  // other glue logic
  ////////////////////////////////////////////////////////////////////////////////

  // "Out" from the Host to the FPGA.
  // new bytes are available from the host when the "out" grant goes high
  // and "out" data available.
  // FPGA will always request data when it is available from the host
  // and always signals that it has read it.
  wire out_data_ready = out_ep_grant && out_ep_data_avail;
  assign out_ep_req = out_ep_data_avail;
  assign out_ep_data_get = out_ep_grant; // always ready

  reg out_data_valid;
  reg uart_rx_ready;
  reg [7:0] uart_rx_data;

  always @(posedge clk) begin
	uart_rx_strobe <= 0;
	out_data_valid <= out_data_ready;

	if (out_data_valid) begin
		uart_rx_data <= out_ep_data;
		uart_rx_strobe <= 1;
	end
  end
  
  // "In" from the FPGA to the Host.
  // when the host says that there is space free in the outbound
  // packet, clock in bytes from the FPGA on the uart_tx_strobe line.

  // the FPGA can send bytes when in "in" grant
  // is high and the "in" data is free.
  wire byte_in_xfr_ready = in_ep_grant && in_ep_data_free;

  reg [7:0] tx_data;
  reg tx_data_valid;
  assign uart_tx_ready = !tx_data_valid;

  always @(posedge clk) begin
	in_ep_data_put <= 0;
	in_ep_req <= 0;
	in_ep_data_done <= 0;
	if (uart_tx_strobe) begin
		tx_data <= uart_tx_data;
		tx_data_valid <= 1;
	end
	if (in_ep_data_free && tx_data_valid) begin
		in_ep_req <= 1;
		in_ep_data_put <= 1;
		in_ep_data_done <= 1;
		in_ep_data <= tx_data;
		tx_data_valid <= 0;
	end
  end

/*
  reg in_ep_req_i = 0;
  always @(posedge clk) in_ep_req_i <= tx_data_valid && in_ep_data_free;
  always @(posedge clk) in_ep_req <= in_ep_req_i;
  //always @(posedge clk) in_ep_data_put <= tx_data_valid;

  // how do we know if we're done?
  // todo: figure out how to packetize the data or use a
  // fifo to keep things fed
  reg in_ep_data_done_i = 1;
  reg in_ep_data_done_q = 0;
  always @(posedge clk) in_ep_data_done_q <= in_ep_data_done_i;
  always @(posedge clk) in_ep_data_done <= in_ep_data_done_q;
*/

endmodule
