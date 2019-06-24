all: sdram_demo.bit
all: spispy.bit

sdram_demo.json: pll_132.v
spispy.json: pll_132.v

flash: spispy.flash
	sleep 0.5
	chmod 666 /dev/ttyUSB0
	-stty -F /dev/ttyUSB0 1:0:1cbd:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0


include Makefile.icestorm
