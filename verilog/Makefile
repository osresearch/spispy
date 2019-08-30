#all: sdram_demo.bit
all: spispy.bit

sdram_demo.json: pll_132.v
spispy.json: pll_132.v

flash: spispy.flash fix-usb

fix-usb:
	sleep 1
	-chmod 666 /dev/ttyUSB*
	-chmod 666 /dev/ttyACM*
	-stty -F /dev/ttyUSB0 1:0:1cbd:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0



include Makefile.icestorm
