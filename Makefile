BOARD ?= beaglewire

all: sdram-test.bin
all: spispy.bin
#all: lighthouse-demo.bin

sdram-test.json: pll_96.v
spispy.json: pll_96.v
spispy.json: pll_200.v

# use the BBB as an FPGA programmer since it can directly
# toggle the SPI flash lines.
%.beaglewire: %.bin
	scp $< root@192.168.6.2:
	ssh root@192.168.6.2 ~debian/bridge-cleanup/load_fw/bw-prog.sh $<

include Makefile.icestorm
