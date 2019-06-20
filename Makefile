all: sdram_demo.bit
all: spispy.bit

sdram_demo.json: pll_132.v
spispy.json: pll_132.v

include Makefile.icestorm
