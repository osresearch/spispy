/*
 *  uspispy fimrware for risc-v
 *
 *  Copyright (C) 2020 Trammell Hudson <hudson@trmm.net>
 *  Portions copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <stdint.h>
#include <stdbool.h>

#  define MEM_TOTAL 0x20000 /* 128 KB */

#define NULL ((void*) 0)
typedef uint32_t size_t;

// a pointer to this is a null pointer, but the compiler does not
// know that because "sram" is a linker symbol from sections.lds.
extern uint32_t sram;

#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)
#define reg_leds (*(volatile uint32_t*)0x03000000)
#define reg_uspi (*(volatile uint32_t*)0x04000000)
#define reg_wbuf (*(volatile uint32_t*)0x04100000)

typedef struct {
	volatile uint32_t counter;
	volatile uint32_t cmd;
	volatile uint32_t addr;
	volatile uint32_t len;
	volatile uint32_t sr;
} uspispy_t;

typedef union {
	volatile uint32_t cr;
	struct {
		volatile uint32_t data:8, mode:8, sel:2, unused:13, idle:1;
	} bytes;
	struct {
		volatile uint32_t data_mode:16, sel_idle:16;
	} words;
} spi_controller_t;


#define uspi ((uspispy_t*)        0x04000000)
#define spi0 ((spi_controller_t*) 0x05000000)

void print_char(char c)
{
	if (c == '\n')
		print_char('\r');
	reg_uart_data = c;
}

void print(const char *p)
{
	while (*p)
		print_char(*(p++));
}

void print_hex(uint32_t v, int digits)
{
	for (int i = 7; i >= 0; i--) {
		char c = "0123456789abcdef"[(v >> (4*i)) & 15];
		if (c == '0' && i >= digits) continue;
		print_char(c);
		digits = i;
	}
}

void print_dec(uint32_t v)
{
	if (v >= 1000) {
		print(">=1000");
		return;
	}

	if      (v >= 900) { print_char('9'); v -= 900; }
	else if (v >= 800) { print_char('8'); v -= 800; }
	else if (v >= 700) { print_char('7'); v -= 700; }
	else if (v >= 600) { print_char('6'); v -= 600; }
	else if (v >= 500) { print_char('5'); v -= 500; }
	else if (v >= 400) { print_char('4'); v -= 400; }
	else if (v >= 300) { print_char('3'); v -= 300; }
	else if (v >= 200) { print_char('2'); v -= 200; }
	else if (v >= 100) { print_char('1'); v -= 100; }

	if      (v >= 90) { print_char('9'); v -= 90; }
	else if (v >= 80) { print_char('8'); v -= 80; }
	else if (v >= 70) { print_char('7'); v -= 70; }
	else if (v >= 60) { print_char('6'); v -= 60; }
	else if (v >= 50) { print_char('5'); v -= 50; }
	else if (v >= 40) { print_char('4'); v -= 40; }
	else if (v >= 30) { print_char('3'); v -= 30; }
	else if (v >= 20) { print_char('2'); v -= 20; }
	else if (v >= 10) { print_char('1'); v -= 10; }

	if      (v >= 9) { print_char('9'); v -= 9; }
	else if (v >= 8) { print_char('8'); v -= 8; }
	else if (v >= 7) { print_char('7'); v -= 7; }
	else if (v >= 6) { print_char('6'); v -= 6; }
	else if (v >= 5) { print_char('5'); v -= 5; }
	else if (v >= 4) { print_char('4'); v -= 4; }
	else if (v >= 3) { print_char('3'); v -= 3; }
	else if (v >= 2) { print_char('2'); v -= 2; }
	else if (v >= 1) { print_char('1'); v -= 1; }
	else print_char('0');
}

static inline uint32_t rdcycle(void)
{
	uint32_t cycles;
	__asm__ volatile ("rdcycle %0" : "=r"(cycles));
	return cycles;
}

static void usleep(uint32_t usec)
{
	const uint32_t cycles = usec * 16;
	const uint32_t start = rdcycle();
	while((rdcycle() - start) < cycles)
		;
}


static void spi_flash(
	const uint8_t cmd,
	uint32_t addr,
	uint32_t len,
	uint32_t sr
)
{
	const bool write_enabled = sr & 2;

	if (cmd == 0x20) {
		// erase command; make sure address is idle and write enabled
		if (len != 0x04 || !write_enabled) {
			uspi->sr = 0;
			return;
		}

		// should take control of the appropriate RAM chip and do stuff
		// then reset the status register
		print("SR=0\n");
		uspi->sr = 0;
		return;
	}

	if (cmd == 0x02) {
		if (len <= 0x04 || !write_enabled) {
			uspi->sr = 0;
			return;
		}

		// write command; address is original address
		// len includes extra 4 bytes
		// should read len-4 bytes from psram, AND it with the buffer
		// and then write it back to the psram
		print("WRITE\n");
		uspi->sr = 0;
	}
}

static inline uint8_t spi_transfer(
	uint8_t tx
)
{
	uint32_t cr;
	while(!spi0->bytes.idle)
		;

	spi0->bytes.data = tx;
	do {
		cr = spi0->cr;
	} while ((cr & 0x80000000) == 0);

	//return (spi0->cr & 0xFF);
	return cr & 0xFF;
}


#define SPI_MODE_NONE	0x91
#define SPI_MODE_SINGLE 0x11
#define SPI_MODE_QUAD_IN 0x40
#define SPI_MODE_QUAD_OUT 0x4F

static inline void spi_cs_mode(uint8_t mode)
{
	// ensure that a transfer has finished before changing the mode
	while(!spi0->bytes.idle)
		;

	spi0->bytes.mode = mode;
}

static void spi_command(
	uint8_t cmd,
	uint8_t * buf,
	size_t len
)
{
	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(cmd);

	for(size_t i = 0 ; i < len ; i++)
		buf[i] = spi_transfer(0x00);

	spi_cs_mode(SPI_MODE_NONE);
}

void psram_erase(
	uint32_t addr,
	size_t len
)
{
	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(0x02);
	spi_transfer((addr >> 16) && 0xFF);
	spi_transfer((addr >>  8) && 0xFF);
	spi_transfer((addr >>  0) && 0xFF);

	for(unsigned i = 0 ; i < len ; i++)
		spi_transfer(0xFF);

	spi_cs_mode(SPI_MODE_NONE);
}

void psram_write(
	uint32_t addr,
	const void * p,
	size_t len
)
{
	const uint8_t * const buf = p;

	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(0x02);

	spi_transfer((addr >> 16) & 0xFF);
	spi_transfer((addr >>  8) & 0xFF);
	spi_transfer((addr >>  0) & 0xFF);

	for(unsigned i = 0 ; i < len ; i++)
		spi_transfer(buf[i]);

	spi_cs_mode(SPI_MODE_NONE);
}

void psram_read(
	uint32_t addr,
	void * p,
	size_t len
)
{
	uint8_t * const buf = p;

	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(0x03);

	spi_transfer((addr >> 16) & 0xFF);
	spi_transfer((addr >>  8) & 0xFF);
	spi_transfer((addr >>  0) & 0xFF);

	for(unsigned i = 0 ; i < len ; i++)
		buf[i] = spi_transfer(0);

	spi_cs_mode(SPI_MODE_NONE);
}

void psram_quad_write(
	uint32_t addr,
	const void * p,
	size_t len
)
{
	const uint8_t * const buf = p;

	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(0x38);

	spi_cs_mode(SPI_MODE_QUAD_OUT);
	spi_transfer((addr >> 16) & 0xFF);
	spi_transfer((addr >>  8) & 0xFF);
	spi_transfer((addr >>  0) & 0xFF);

	for(unsigned i = 0 ; i < len ; i++)
		spi_transfer(buf[i]);

	spi_cs_mode(SPI_MODE_NONE);
}

void psram_quad_read(
	uint32_t addr,
	void * p,
	size_t len
)
{
	uint8_t * const buf = p;

	spi_cs_mode(SPI_MODE_SINGLE);
	spi_transfer(0xEB);

	spi_cs_mode(SPI_MODE_QUAD_OUT);
	spi_transfer((addr >> 16) & 0xFF);
	spi_transfer((addr >>  8) & 0xFF);
	spi_transfer((addr >>  0) & 0xFF);

	// dummy cycles
	spi_cs_mode(SPI_MODE_QUAD_IN);
	spi_transfer(0x00);
	spi_transfer(0x00);
	spi_transfer(0x00);

	for(unsigned i = 0 ; i < len ; i++)
		buf[i] = spi_transfer(i);

	spi_cs_mode(SPI_MODE_NONE);
}

int main(void)
{
	reg_leds = 31;
	//reg_uart_clkdiv = 104; // 115200 baud = 12 MHz / 104
	reg_uart_clkdiv = 139; // 115200 baud = 16 MHz / 139

	// force some clocks with !CS high since the datasheet says so
	spi0->bytes.sel = 0;

	spi_cs_mode(0x90);
	spi_transfer(0x00);
	spi_cs_mode(SPI_MODE_NONE);

	// psram reset enable, followed by reset command
	spi_command(0x66, NULL, 0);
	spi_command(0x99, NULL, 0);

	/* test the spi controller */
	unsigned iter = 0;
	uint8_t buf[16];
	uint8_t buf2[32];

	while(1)
	{
		print("--- ");

		// switch which RAM we're talking to every cycle
		spi0->bytes.sel = iter & 1;

		// 0x9F == RDID
		if (1) {
		uint8_t data[0xC];
		spi_command(0x9F, data, sizeof(data));

		for(int i = 4 ; i < sizeof(data) ; i++)
			print_hex(data[i], 2);

		print_char(' ');
		}
		print_hex(iter, 8);
		print_char(' ');

		// fill the ram with a random sequence and read it back
		for(unsigned i = 0 ; i < sizeof(buf) ; i++)
		{
			buf[i] = i ^ iter;
			buf2[i] = 0;
		}

		psram_erase(iter * sizeof(buf), 4096);
		psram_write(iter * sizeof(buf), buf, sizeof(buf));

		psram_read(iter * sizeof(buf), buf2, sizeof(buf2));
		
		int fail = 0;
		for(unsigned i = 0 ; i < sizeof(buf) ; i++)
		{
			if (buf[i] == buf2[i])
				continue;
			//print_hex(i, 2);
			//print_hex(buf[i], 2);
			//print_hex(buf2[i], 2);
			//print_char(' ');
			fail = 1;
		}

		if (fail) {
			print_char('\n');
			for(unsigned i = 0 ; i < sizeof(buf2) ; i++)
			{
				if ((i & 0x0F) == 0) print_char(' ');
				print_hex(buf2[i], 2);
			}

			// re-read the buffer
			print_char('\n');
			psram_read(iter * sizeof(buf), buf, sizeof(buf));
			for(unsigned i = 0 ; i < sizeof(buf) ; i++)
			{
				if ((i & 0x0F) == 0) print_char(' ');
				print_hex(buf[i], 2);
			}
			print(" FAIL\n");
		} else
			print("pass\n");

		usleep(100000);
		iter++;
	}
/*
	while(1)
	{
		print_char('a');
		reg_leds = ~reg_leds;
	}
*/

	uint32_t last_report = 0;
	uint32_t last_cmd = 0;
	while(1)
	{
		const uint32_t now = rdcycle();
		if (now - last_report > 16000000) {
			last_report = now;
			print_hex(uspi->counter, 8);
			print("---\n");
		}

		const uint32_t cmd = uspi->cmd;
		if (cmd == last_cmd)
			continue;

		const uint32_t addr = uspi->addr;
		const uint32_t len = uspi->len;
		const uint8_t sr = uspi->sr;
		

		print_hex(cmd >> 8, 6);
		print_char(' ');
		print_hex(cmd & 0xFF, 2);
		print_char(' ');
		print_hex(addr, 6);
		print_char(' ');
		print_hex(len, 4);
		print_char(' ');
		print_hex(sr, 2);
		print_char('\n');

		last_cmd = cmd;
		last_report = now;

		spi_flash(cmd & 0xFF, addr, len, sr);
	}
}
