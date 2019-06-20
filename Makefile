all: sdram_demo.bin
all: spispy.bin

sdram_demo.json: pll_132.v
spispy.json: pll_132.v

include Makefile.icestorm
