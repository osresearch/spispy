all: top.bin

DEVICE-tinyfpga := lp8k
FOOTPRINT-tinyfpga := cm81
PIN_SRC-tinyfpga := ../tinyfpga-bx.pcf

DEVICE-tomu := up5k
FOOTPRINT-tomu := uwg30
PIN_SRC-tomu := ../tomu.pcf

DEVICE := $(DEVICE-$(BOARD))
FOOTPRINT := $(FOOTPRINT-$(BOARD))
PIN_SRC := $(PIN_SRC-$(BOARD))

include ../Makefile.icestorm
