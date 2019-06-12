![BeagleWire connected to SPI flash with oscilloscope probes](images/header.jpg)

# SPI Spy: Flash emulation

The SPI Spy tool is an open source (both hardware and software)
SPI flash emulation tool.  It an store a flash image in the SDRAM
connected to the FPGA and serve the image to a host CPU over the
SPI bus.

## Platform
The design is currently based on the BeagleWire, which has an
iCE40hx8k FPGA and a 32 MB SDRAM.  The BeagleBone Black is used
as an FPGA programmer, but a separate serial adapter is needed.

## Supported features
* Single SPI up to 20 MHz clock
* 3-byte addressing (up to 16 MB of flash image)
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

