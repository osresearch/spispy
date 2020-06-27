# uSpispy

The uSpispy is a low-cost version of the spispy built with an ice40up5k and two
[LY68L6400SLIT QSPI PSRAM](https://lcsc.com/product-detail/RAM_Lyontek-Inc-LY68L6400SLIT_C261881.html)
chips ([datasheet](https://datasheet.lcsc.com/szlcsc/1809140531_Lyontek-Inc-LY68L6400SLIT_C261881.pdf))
instead of the ECP5 and SDRAM.


The commands supported by the PSRAM are directly routed to the appropriate chip:

* 0x03: Read (single cmd, single address, 0 wait cycles, 33 MHz)
* 0x0B: Fast read (single cmd, single address, 8 wait cycles, 84 Mhz)
* 0xEB: Fast read quad (single cmd, quad address, 6 wait cycles, 84 MHz)

The FPGA emulates these commands:

* 0x9F: Read ID
* 0x05: Read status register
* 0x06: Write enable
* 0x04: Write disable
* 0x20: Sector erase (4KB)

Writes are a little more complicated -- the PSRAM acts like RAM with
byte-addressable writes, while the SPI Flash devices expect NOR writes.
The FPGA buffers the writen data so that the PSRAM acts like normal flash.
This requires that the FPGA read the data from the correct PSRAM, `AND` it with
the buffer, and then write that buffer back to the PSRAM.

* 0x02: Write (single cmd, single address, 0 wait cycles, 84 MHz)
* 0x38: Quad write (single cmd, quad address, 0 wait cycles, 84 MHz)
