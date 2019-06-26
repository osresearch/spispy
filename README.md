![ULX3S connected to SPI flash with a 8-SOIC clip and debugged with oscilloscope probes](images/header.jpg)

# SPI Spy: Flash emulation

The SPI Spy tool is an open source (both hardware and software)
SPI flash emulation tool.  It an store a flash image in the SDRAM
connected to the FPGA and serve the image to a host CPU over the
SPI bus.

## Platform
The design is currently based on the [ULX3S](https://radiona.org/ulx3s/)
which has an Lattice ECP5-12F FPGA and a 16-bit wide 32 MB SDRAM. It
might be portable to the TinyFPGA-EX or other open source ECP5 boards,
although it uses a custom SDRAM controller to be able to meet the
difficult timing requirements of the SPI flash protocol (described below).

## Supported features
* Single SPI up to 20 MHz clock
* 3-byte addressing (up to 16 MB of flash image)
* Serial port updates to the SDRAM
* Logging flash access patterns
* TOCTOU changes to the flash image based on read patterns

## Not yet supported
* Multiple `!CS` pins
* Dual- and Quad-SPI
* Fast read command
* Erase/Write emulation
* Status registers
* Block protection bits
* BeagleBone Black integration
  * Serial port for console
  * Higher bandwidth programming over GPMC
  * A decent API for TOCTOU

# Wiring

Typical 8-SOIC and 8-DIP flash chips:

```
            +------+
    !CE  ---| o    |----  +V
     SO  ---|      |---- !RST
    !WP  ---|      |----  SCK
    GND  ---|      |----  SI
            +------+
```

# Protocol
![SPI data](images/data.jpg)

The SPI protocol is difficult to emulate without specialized hardware
since it has very demanding timing requirements.  The flash device
has no control over the clock and must be able to respond to a random
read request on the very next clock.  At 20 MHz, the slowest SPI bus
on some Intel PCH chipsets, this is 50ns from receiving the last bit of
the address to having to supply the first bit of the data.

Unfortunately, most microcontroller CPUs aren't able to respond to an
incoming SPI byte on the next SPI cycle due to internal muxes and buses,
so they aren't able to reply in time.  Even if the CPU could do it,
most DRAM memory has a 100ns or longer latency for a random read, so
it won't be able to answer quickly enough.  Additionally, DRAM requires
a refresh cycle that takes it offline during the refresh, which adds a
random latency.

![SDRAM read waveform](images/dram-read.png)

These difficulties can be overcome with an FPGA using a custom DRAM
controller.  The FPGA is able to inhibit refresh cycles during the SPI
critical sections, which reduces the latency jitter, and it can split
the DRAM access into two parts: the "row activation" once 16 of the
24 address bits are known, and then a "column read" of two bytes worth 
of data once 7 of the last 8 bits are known.  The correct byte is selected
once the last bit of the address has been received.

The row activation command requires at least four DRAM clock cycles,
but can be stretched arbitrarily long with a special control signal wired
into the FPGA's sdram controller from the SPI bus interface.  This allows
the FPGA to overlap the activation with the reception of the last bits,
and the final column read requires only two clock cycles when the DRAM
is configured with a CAS latency of two.

Subsequent bytes are "easy" at 20 MHz for single SPI since the full
SDRAM read cycle (7 FPGA clocks) fits into the 8 clocks of the SPI bus
(roughly 24 FPGA clocks). For dual or quad-SPI it will be necessary
to configure a burst mode on the SDRAM controller or allow new column
addresses to be provided dynamically.
