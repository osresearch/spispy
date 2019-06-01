`ifndef _gpio_v_
`define _gpio_v_
/**
 * Bi-directional GPIO for the ice40 chips with SB_IO
 */
module gpio(
	input enable,
	input out,
	output in,
	inout pin
);
	parameter PULLUP = 1'b0;

	SB_IO #(
		.PIN_TYPE(6'b1010_01), // Tristate
		.PULLUP(PULLUP),
	) io(
		.PACKAGE_PIN(pin),
		.OUTPUT_ENABLE(enable),
		.D_OUT_0(out),
		.D_IN_0(in),
	);
endmodule

`endif
