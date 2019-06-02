
all: sdram-test.bin
all: spispy.bin
#all: lighthouse-demo.bin

sdram-test.json: pll_96.v
spispy.json: pll_96.v


include Makefile.icestorm
