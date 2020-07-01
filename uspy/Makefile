
all: top.bin

TEST-y += test-qspi

top.bin: firmware.hex

include Makefile.icestorm

CROSS ?= /opt/riscv32i/bin/riscv32-unknown-elf-
CFLAGS ?= -g -O3 -Wall

firmware.elf: picorv32/start.o firmware.o

%.o: %.c
	$(CROSS)gcc \
		$(CFLAGS) \
		-march=rv32ic \
		-Wp,-MMD,$(dir $@).$(notdir $@).d \
		-Wp,-MT,$@ \
		-c -o $@ $<
%.o: %.s
	$(CROSS)gcc \
		$(ASFLAGS) \
		-march=rv32ic \
		-Wp,-MMD,$(dir $@).$(notdir $@).d \
		-Wp,-MT,$@ \
		-c -o $@ $<

%.elf:
	$(CROSS)gcc \
		-Wl,-Bstatic \
		-Wl,--strip-debug \
		-Wl,-T,picorv32/sections.lds \
		-ffreestanding \
		-nostdlib \
		-o $@ \
		$^

%.bin: %.elf
	$(CROSS)objcopy -O binary $< $@

%.hex: %.bin
	xxd -g4 -c4 -e $< | cut -d' ' -f2 > $@
