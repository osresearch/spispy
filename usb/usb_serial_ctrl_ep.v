`include "edge_detect.v"

module usb_serial_ctrl_ep (
  input clk,
  input reset,
  output [6:0] dev_addr,

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
  output in_ep_req,
  input in_ep_grant,
  input in_ep_data_free,
  output in_ep_data_put,
  output [7:0] in_ep_data,
  output in_ep_data_done,
  output reg in_ep_stall,
  input in_ep_acked
);
  

  localparam IDLE = 0;
  localparam SETUP = 1;
  localparam DATA_IN = 2;
  localparam DATA_OUT = 3;
  localparam STATUS_IN = 4;
  localparam STATUS_OUT = 5;
  
  reg [5:0] ctrl_xfr_state = IDLE;
  reg [5:0] ctrl_xfr_state_next;
 
 
 
  reg setup_stage_end;
  reg data_stage_end;
  reg status_stage_end;
  reg send_zero_length_data_pkt;



  // the default control endpoint gets assigned the device address
  reg [6:0] dev_addr_i = 0;
  assign dev_addr = dev_addr_i;

  assign out_ep_req = out_ep_data_avail;
  assign out_ep_data_get = out_ep_data_avail;
  reg out_ep_data_valid = 0;
  always @(posedge clk) out_ep_data_valid <= out_ep_data_avail && out_ep_grant;

  // need to record the setup data
  reg [3:0] setup_data_addr;
  reg [9:0] raw_setup_data [7:0];

  wire [7:0] bmRequestType = raw_setup_data[0];
  wire [7:0] bRequest = raw_setup_data[1];
  wire [15:0] wValue = {raw_setup_data[3][7:0], raw_setup_data[2][7:0]};
  wire [15:0] wIndex = {raw_setup_data[5][7:0], raw_setup_data[4][7:0]};
  wire [15:0] wLength = {raw_setup_data[7][7:0], raw_setup_data[6][7:0]};

  // keep track of new out data start and end
  wire pkt_start;
  wire pkt_end;

  rising_edge_detector detect_pkt_start (
    .clk(clk),
    .in(out_ep_data_avail),
    .out(pkt_start)
  );

  falling_edge_detector detect_pkt_end (
    .clk(clk),
    .in(out_ep_data_avail),
    .out(pkt_end)
  );

  assign out_ep_stall = 1'b0;

  wire setup_pkt_start = pkt_start && out_ep_setup;

  wire has_data_stage = wLength != 16'b0000000000000000;

  wire out_data_stage;
  assign out_data_stage = has_data_stage && !bmRequestType[7];

  wire in_data_stage;
  assign in_data_stage = has_data_stage && bmRequestType[7];

  reg [7:0] bytes_sent = 0;
  reg [6:0] rom_length = 0;

  wire all_data_sent = 
    (bytes_sent >= rom_length) ||
    (bytes_sent >= wLength);

  wire more_data_to_send =
    !all_data_sent;

  wire in_data_transfer_done;

  rising_edge_detector detect_in_data_transfer_done (
    .clk(clk),
    .in(all_data_sent),
    .out(in_data_transfer_done)
  );

  assign in_ep_data_done = (in_data_transfer_done && ctrl_xfr_state == DATA_IN) || send_zero_length_data_pkt;

  assign in_ep_req = ctrl_xfr_state == DATA_IN && more_data_to_send;
  assign in_ep_data_put = ctrl_xfr_state == DATA_IN && more_data_to_send && in_ep_data_free;


  reg [6:0] rom_addr = 0;

  reg save_dev_addr = 0;
  reg [6:0] new_dev_addr = 0;

  ////////////////////////////////////////////////////////////////////////////////
  // control transfer state machine
  ////////////////////////////////////////////////////////////////////////////////


  always @* begin
    setup_stage_end <= 0;
    data_stage_end <= 0;
    status_stage_end <= 0;
    send_zero_length_data_pkt <= 0;

    case (ctrl_xfr_state)
      IDLE : begin
        if (setup_pkt_start) begin
          ctrl_xfr_state_next <= SETUP;
        end else begin
          ctrl_xfr_state_next <= IDLE;
        end
      end

      SETUP : begin
        if (pkt_end) begin
          setup_stage_end <= 1;

          if (in_data_stage) begin
            ctrl_xfr_state_next <= DATA_IN;

          end else if (out_data_stage) begin
            ctrl_xfr_state_next <= DATA_OUT;

          end else begin
            ctrl_xfr_state_next <= STATUS_IN;
            send_zero_length_data_pkt <= 1;
          end

        end else begin
          ctrl_xfr_state_next <= SETUP;
        end
      end

      DATA_IN : begin
	if (in_ep_stall) begin
          ctrl_xfr_state_next <= IDLE;
          data_stage_end <= 1;
          status_stage_end <= 1;

	end else if (in_ep_acked && all_data_sent) begin
          ctrl_xfr_state_next <= STATUS_OUT;
          data_stage_end <= 1;

        end else begin
          ctrl_xfr_state_next <= DATA_IN;
        end
      end

      DATA_OUT : begin
        if (out_ep_acked) begin
          ctrl_xfr_state_next <= STATUS_IN;
          send_zero_length_data_pkt <= 1;
          data_stage_end <= 1;
          
        end else begin
          ctrl_xfr_state_next <= DATA_OUT;
        end
      end

      STATUS_IN : begin
        if (in_ep_acked) begin
          ctrl_xfr_state_next <= IDLE;
          status_stage_end <= 1;
          
        end else begin
          ctrl_xfr_state_next <= STATUS_IN;
        end
      end

      STATUS_OUT: begin
        if (out_ep_acked) begin
          ctrl_xfr_state_next <= IDLE;
          status_stage_end <= 1;
          
        end else begin
          ctrl_xfr_state_next <= STATUS_OUT;
        end
      end

      default begin
        ctrl_xfr_state_next <= IDLE;
      end
    endcase
  end

  always @(posedge clk) begin
    if (reset) begin
      ctrl_xfr_state <= IDLE;
    end else begin
      ctrl_xfr_state <= ctrl_xfr_state_next;
    end
  end

  always @(posedge clk) begin
    in_ep_stall <= 0;

    if (out_ep_setup && out_ep_data_valid) begin
      raw_setup_data[setup_data_addr] <= out_ep_data;
      setup_data_addr <= setup_data_addr + 1;
    end

    if (setup_stage_end) begin
      case (bRequest)
        'h06 : begin
          // GET_DESCRIPTOR
          case (wValue[15:8]) 
            1 : begin
              // DEVICE
              rom_addr    <= 'h00; 
              rom_length  <= 'h12;
            end 

            2 : begin
              // CONFIGURATION
              rom_addr    <= 'h12; 
              rom_length  <= 'h43;
            end 
            
            6 : begin
              // DEVICE_QUALIFIER
              in_ep_stall <= 1;
              rom_addr   <= 'h00;
              rom_length <= 'h00;
            end
            
          endcase
        end

        'h05 : begin
          // SET_ADDRESS
          rom_addr   <= 'h00;
          rom_length <= 'h00;

          // we need to save the address after the status stage ends
          // this is because the status stage token will still be using
          // the old device address
          save_dev_addr <= 1;
          new_dev_addr <= wValue[6:0]; 
        end

        'h09 : begin
          // SET_CONFIGURATION
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h20 : begin
          // SET_LINE_CODING
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h21 : begin
          // GET_LINE_CODING
          rom_addr   <= 'h55;
          rom_length <= 'h07;
        end

        'h22 : begin
          // SET_CONTROL_LINE_STATE
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h23 : begin
          // SEND_BREAK
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        default begin
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end
      endcase
    end

    if (ctrl_xfr_state == DATA_IN && more_data_to_send && in_ep_grant && in_ep_data_free) begin
      rom_addr <= rom_addr + 1;
      bytes_sent <= bytes_sent + 1;
    end

    if (status_stage_end) begin
      setup_data_addr <= 0;      
      bytes_sent <= 0;
      rom_addr <= 0;
      rom_length <= 0;

      if (save_dev_addr) begin
        save_dev_addr <= 0;
        dev_addr_i <= new_dev_addr; 
      end 
    end

    if (reset) begin
      dev_addr_i <= 0;
      setup_data_addr <= 0;
      save_dev_addr <= 0;
    end
  end


  `define CDC_ACM_ENDPOINT 2 
  `define CDC_RX_ENDPOINT 1
  `define CDC_TX_ENDPOINT 1
	
  reg [7:0] ep_rom[255:0];
  assign in_ep_data = ep_rom[rom_addr];

  initial begin
      // device descriptor
      ep_rom['h000] <= 18; // bLength
      ep_rom['h001] <= 1; // bDescriptorType
      ep_rom['h002] <= 'h00; // bcdUSB[0]
      ep_rom['h003] <= 'h02; // bcdUSB[1]
      ep_rom['h004] <= 'h02; // bDeviceClass (Communications Device Class)
      ep_rom['h005] <= 'h00; // bDeviceSubClass (Abstract Control Model)
      ep_rom['h006] <= 'h00; // bDeviceProtocol (No class specific protocol required)
      ep_rom['h007] <= 32; // bMaxPacketSize0

      ep_rom['h008] <= 'h50; // idVendor[0] http://wiki.openmoko.org/wiki/USB_Product_IDs
      ep_rom['h009] <= 'h1d; // idVendor[1]
      ep_rom['h00A] <= 'h30; // idProduct[0]
      ep_rom['h00B] <= 'h61; // idProduct[1]
      
      ep_rom['h00C] <= 0; // bcdDevice[0]
      ep_rom['h00D] <= 0; // bcdDevice[1]
      ep_rom['h00E] <= 0; // iManufacturer
      ep_rom['h00F] <= 0; // iProduct
      ep_rom['h010] <= 0; // iSerialNumber
      ep_rom['h011] <= 1; // bNumConfigurations

      // configuration descriptor
      ep_rom['h012] <= 9; // bLength
      ep_rom['h013] <= 2; // bDescriptorType
      ep_rom['h014] <= (9+9+5+5+4+5+7+9+7+7); // wTotalLength[0] 
      ep_rom['h015] <= 0; // wTotalLength[1]
      ep_rom['h016] <= 2; // bNumInterfaces
      ep_rom['h017] <= 1; // bConfigurationValue
      ep_rom['h018] <= 0; // iConfiguration
      ep_rom['h019] <= 'hC0; // bmAttributes
      ep_rom['h01A] <= 50; // bMaxPower
      
      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      ep_rom['h01B] <= 9; // bLength
      ep_rom['h01C] <= 4; // bDescriptorType
      ep_rom['h01D] <= 0; // bInterfaceNumber
      ep_rom['h01E] <= 0; // bAlternateSetting
      ep_rom['h01F] <= 1; // bNumEndpoints
      ep_rom['h020] <= 2; // bInterfaceClass (Communications Device Class)
      ep_rom['h021] <= 2; // bInterfaceSubClass (Abstract Control Model)
      ep_rom['h022] <= 1; // bInterfaceProtocol (AT Commands: V.250 etc)
      ep_rom['h023] <= 0; // iInterface

      // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
      ep_rom['h024] <= 5;					// bFunctionLength
	    ep_rom['h025] <= 'h24;					// bDescriptorType
	    ep_rom['h026] <= 'h00;					// bDescriptorSubtype
	    ep_rom['h027] <= 'h10; 
      ep_rom['h028] <= 'h01;				// bcdCDC

	    // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
	    ep_rom['h029] <= 5;					// bFunctionLength
	    ep_rom['h02A] <= 'h24;					// bDescriptorType
	    ep_rom['h02B] <= 'h01;					// bDescriptorSubtype
	    ep_rom['h02C] <= 'h00;					// bmCapabilities
	    ep_rom['h02D] <= 1;					// bDataInterface

	    // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
	    ep_rom['h02E] <= 4;					// bFunctionLength
	    ep_rom['h02F] <= 'h24;					// bDescriptorType
	    ep_rom['h030] <= 'h02;					// bDescriptorSubtype
	    ep_rom['h031] <= 'h06;					// bmCapabilities

	    // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
    	ep_rom['h032] <= 5;					// bFunctionLength
    	ep_rom['h033] <= 'h24;					// bDescriptorType
    	ep_rom['h034] <= 'h06;					// bDescriptorSubtype
    	ep_rom['h035] <= 0;					// bMasterInterface
    	ep_rom['h036] <= 1;					// bSlaveInterface0

    	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    	ep_rom['h037] <= 7;					// bLength
    	ep_rom['h038] <= 5;					// bDescriptorType
    	ep_rom['h039] <= `CDC_ACM_ENDPOINT | 'h80;		// bEndpointAddress
    	ep_rom['h03A] <= 'h03;					// bmAttributes (0x03=intr)
    	ep_rom['h03B] <= 8;     // wMaxPacketSize[0]
      ep_rom['h03C] <= 0;			// wMaxPacketSize[1]
    	ep_rom['h03D] <= 64;					// bInterval

    	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
    	ep_rom['h03E] <= 9;					// bLength
    	ep_rom['h03F] <= 4;					// bDescriptorType
    	ep_rom['h040] <= 1;					// bInterfaceNumber
    	ep_rom['h041] <= 0;					// bAlternateSetting
    	ep_rom['h042] <= 2;					// bNumEndpoints
    	ep_rom['h043] <= 'h0A;					// bInterfaceClass
    	ep_rom['h044] <= 'h00;					// bInterfaceSubClass
    	ep_rom['h045] <= 'h00;					// bInterfaceProtocol
    	ep_rom['h046] <= 0;					// iInterface

    	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    	ep_rom['h047] <= 7;					// bLength
    	ep_rom['h048] <= 5;					// bDescriptorType
    	ep_rom['h049] <= `CDC_RX_ENDPOINT;			// bEndpointAddress
    	ep_rom['h04A] <= 'h02;					// bmAttributes (0x02=bulk)
    	ep_rom['h04B] <= 32; // wMaxPacketSize[0]
      ep_rom['h04C] <= 0;				// wMaxPacketSize[1]
    	ep_rom['h04D] <= 0;					// bInterval

    	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    	ep_rom['h04E] <= 7;					// bLength
    	ep_rom['h04F] <= 5;					// bDescriptorType
    	ep_rom['h050] <= `CDC_TX_ENDPOINT | 'h80;			// bEndpointAddress
    	ep_rom['h051] <= 'h02;					// bmAttributes (0x02=bulk)

    	ep_rom['h052] <= 32; // wMaxPacketSize[0]
      ep_rom['h053] <= 0;				// wMaxPacketSize[1]
    	ep_rom['h054] <= 0;				// bInterval

      // LINE_CODING
      ep_rom['h055] <= 'h80; // dwDTERate[0]
      ep_rom['h056] <= 'h25; // dwDTERate[1]
      ep_rom['h057] <= 'h00; // dwDTERate[2]
      ep_rom['h058] <= 'h00; // dwDTERate[3]
      ep_rom['h059] <= 1; // bCharFormat (1 stop bit)
      ep_rom['h05A] <= 0; // bParityType (None)
      ep_rom['h05B] <= 8; // bDataBits (8 bits)

  end

endmodule
