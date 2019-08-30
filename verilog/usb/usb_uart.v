`ifndef usb_uart_v
`define usb_uart_v

`include "usb_serial.v"
`include "../util.v"

module usb_uart (
  input  clk,
  input  clk_48mhz,
  input  reset,

  // physical layer
  inout  pin_usbp,
  inout  pin_usbn,
  output pin_pu,

  // uart
  output [7:0] rx_data,
  output rx_strobe,
  input [7:0] tx_data,
  input tx_strobe
);
  wire usb_p_tx;
  wire usb_n_tx;
  wire usb_p_rx;
  wire usb_n_rx;
  wire usb_tx_en;

  wire uart_tx_ready;
  reg uart_tx_strobe;

  wire uart_rx_strobe = rx_strobe;
  wire [7:0] uart_rx_data = rx_data;
  reg [7:0] uart_tx_data;

  wire fifo_data_available;
  wire [7:0] fifo_tx_data = tx_data;
  wire fifo_tx_strobe = tx_strobe;
  wire [7:0] fifo_rx_data;
  reg fifo_rx_strobe;
  
  fifo tx_buffer(
	.clk(clk),
	.reset(reset),
	.data_available(fifo_data_available),
	.write_data(fifo_tx_data),
	.write_strobe(fifo_tx_strobe),
	.read_data(fifo_rx_data),
	.read_strobe(fifo_rx_strobe)
  );

  // if the fifo has any data and the usb serial is ready
  // transmit the data to it and remove it from the fifo
  always @(posedge clk) if (!reset) begin
	if (uart_tx_ready
	&& fifo_data_available
	&& !uart_tx_strobe
	) begin
		uart_tx_data <= fifo_rx_data;
		uart_tx_strobe <= 1;
		fifo_rx_strobe <= 1;
	end else begin
		uart_tx_strobe <= 0;
		fifo_rx_strobe <= 0;
	end
  end

  usb_serial serial(
    .clk_48mhz(clk_48mhz),
    .clk(clk),
    .reset(reset),
    .uart_tx_ready(uart_tx_ready),
    .uart_tx_strobe(uart_tx_strobe),
    .uart_tx_data(uart_tx_data),
    .uart_rx_strobe(uart_rx_strobe),
    .uart_rx_data(uart_rx_data),
    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en)
  );

  assign pin_pu = 1'b1;

  wire usb_p_rx_io;
  wire usb_n_rx_io;
  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_rx_io;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_rx_io;

  SB_IO #(
    .PIN_TYPE(6'b1010_01) // tristatable output
  ) buffer [1:0](
    .OUTPUT_ENABLE(usb_tx_en),
    .PACKAGE_PIN({pin_usbn, pin_usbp}),
    .D_IN_0({usb_n_rx_io, usb_p_rx_io}),
    .D_OUT_0({usb_n_tx, usb_p_tx})
  );
endmodule

`endif
