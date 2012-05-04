/*
 * (C) Copyright 2009 SAMSUNG Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 * Heungjun Kim <riverful.kim@samsung.com>
 *
 * based on drivers/serial/s3c64xx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <common.h>
#include <linux/compiler.h>
#include <asm/io.h>
#include <asm/arch/uart.h>
#include <asm/arch/clk.h>
#include <fdtdec.h>
#include <serial.h>
#include "serial_fdt.h"

DECLARE_GLOBAL_DATA_PTR;

#define RX_FIFO_COUNT_MASK	0xff
#define RX_FIFO_FULL_MASK	(1 << 8)
#define TX_FIFO_FULL_MASK	(1 << 24)

/* Information about a serial port */
struct fdt_serial {
	u32 base_addr;   /* address of registers in physical memory */
	u8 port_id;	 /* uart port number */
} config = {
	-1U
};

static inline struct s5p_uart *s5p_get_base_uart(int dev_index)
{
#ifdef CONFIG_OF_SERIAL
	return (struct s5p_uart *)(config.base_addr);
#else
	u32 offset = dev_index * sizeof(struct s5p_uart);
	return (struct s5p_uart *)(samsung_get_base_uart() + offset);
#endif
}

/*
 * The coefficient, used to calculate the baudrate on S5P UARTs is
 * calculated as
 * C = UBRDIV * 16 + number_of_set_bits_in_UDIVSLOT
 * however, section 31.6.11 of the datasheet doesn't recomment using 1 for 1,
 * 3 for 2, ... (2^n - 1) for n, instead, they suggest using these constants:
 */
static const int udivslot[] = {
	0,
	0x0080,
	0x0808,
	0x0888,
	0x2222,
	0x4924,
	0x4a52,
	0x54aa,
	0x5555,
	0xd555,
	0xd5d5,
	0xddd5,
	0xdddd,
	0xdfdd,
	0xdfdf,
	0xffdf,
};

static void serial_setbrg_dev(const int dev_index)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);
	u32 uclk = get_uart_clk(dev_index);
	u32 baudrate = gd->baudrate;
	u32 val;

	val = uclk / baudrate;

	writel(val / 16 - 1, &uart->ubrdiv);

	if (s5p_uart_divslot())
		writew(udivslot[val % 16], &uart->rest.slot);
	else
		writeb(val % 16, &uart->rest.value);
}

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 */
static int serial_init_dev(const int dev_index)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);

	/* enable FIFOs */
	writel(0x1, &uart->ufcon);
	writel(0, &uart->umcon);
	/* 8N1 */
	writel(0x3, &uart->ulcon);
	/* No interrupts, no DMA, pure polling */
	writel(0x245, &uart->ucon);

	serial_setbrg_dev(dev_index);

	return 0;
}

static int serial_err_check(const int dev_index, int op)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);
	unsigned int mask;

	/*
	 * UERSTAT
	 * Break Detect	[3]
	 * Frame Err	[2] : receive operation
	 * Parity Err	[1] : receive operation
	 * Overrun Err	[0] : receive operation
	 */
	if (op)
		mask = 0x8;
	else
		mask = 0xf;

	return readl(&uart->uerstat) & mask;
}

/*
 * Read a single byte from the serial port. Returns 1 on success, 0
 * otherwise. When the function is succesfull, the character read is
 * written into its argument c.
 */
static int serial_getc_dev(const int dev_index)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);

	/* wait for character to arrive */
	while (!(readl(&uart->ufstat) & (RX_FIFO_COUNT_MASK |
					 RX_FIFO_FULL_MASK))) {
		if (serial_err_check(dev_index, 0))
			return 0;
	}

	return (int)(readb(&uart->urxh) & 0xff);
}

/*
 * Output a single byte to the serial port.
 */
static void serial_putc_dev(const char c, const int dev_index)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);

	/* wait for room in the tx FIFO */
	while ((readl(&uart->ufstat) & TX_FIFO_FULL_MASK)) {
		if (serial_err_check(dev_index, 1))
			return;
	}

	writeb(c, &uart->utxh);

	/* If \n, also do \r */
	if (c == '\n')
		serial_putc('\r');
}

/*
 * Test whether a character is in the RX buffer
 */
static int serial_tstc_dev(const int dev_index)
{
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);

	return (int)(readl(&uart->utrstat) & 0x1);
}

static void serial_puts_dev(const char *s, const int dev_index)
{
	while (*s)
		serial_putc_dev(*s++, dev_index);
}

/* Multi serial device functions */
#define DECLARE_S5P_SERIAL_FUNCTIONS(port) \
int s5p_serial##port##_init(void) { return serial_init_dev(port); } \
void s5p_serial##port##_setbrg(void) { serial_setbrg_dev(port); } \
int s5p_serial##port##_getc(void) { return serial_getc_dev(port); } \
int s5p_serial##port##_tstc(void) { return serial_tstc_dev(port); } \
void s5p_serial##port##_putc(const char c) { serial_putc_dev(c, port); } \
void s5p_serial##port##_puts(const char *s) { serial_puts_dev(s, port); }

#define INIT_S5P_SERIAL_STRUCTURE(port, name) { \
	name, \
	s5p_serial##port##_init, \
	NULL, \
	s5p_serial##port##_setbrg, \
	s5p_serial##port##_getc, \
	s5p_serial##port##_tstc, \
	s5p_serial##port##_putc, \
	s5p_serial##port##_puts, }

DECLARE_S5P_SERIAL_FUNCTIONS(0);
struct serial_device s5p_serial0_device =
	INIT_S5P_SERIAL_STRUCTURE(0, "s5pser0");
DECLARE_S5P_SERIAL_FUNCTIONS(1);
struct serial_device s5p_serial1_device =
	INIT_S5P_SERIAL_STRUCTURE(1, "s5pser1");
DECLARE_S5P_SERIAL_FUNCTIONS(2);
struct serial_device s5p_serial2_device =
	INIT_S5P_SERIAL_STRUCTURE(2, "s5pser2");
DECLARE_S5P_SERIAL_FUNCTIONS(3);
struct serial_device s5p_serial3_device =
	INIT_S5P_SERIAL_STRUCTURE(3, "s5pser3");

static struct serial_device *s5p_consoles[] = {
	&s5p_serial0_device, &s5p_serial1_device,
	&s5p_serial2_device, &s5p_serial3_device
};

static struct serial_device *pick_console(int index)
{
	if (index >= ARRAY_SIZE(s5p_consoles)) {
		debug("Unknown console index: %d.\n", index);
		return NULL;
	} else {
		return s5p_consoles[index];
	}
}

#ifdef CONFIG_OF_SERIAL
struct serial_device *serial_s5p_fdt_init(void)
{
	const void *blob = gd->fdt_blob;
	int index = 0;
	int node;

	node = fdtdec_next_alias(blob, "serial", COMPAT_SAMSUNG_EXYNOS5_SERIAL,
			index);
	if (node < 0) {
		debug("\"serial\" alias not found.\n");
		return NULL;
	}

	config.base_addr = fdtdec_get_addr(blob, node, "reg");
	if (config.base_addr == FDT_ADDR_T_NONE) {
		debug("Uart base address not found.\n");
		return NULL;
	}

	config.port_id = fdtdec_get_int(blob, node, "id", -1);

	return pick_console(config.port_id);
}

#else

struct serial_device *default_serial_console(void)
{
	int index;
#if defined(CONFIG_SERIAL0)
	index = 0;
#elif defined(CONFIG_SERIAL1)
	index = 1;
#elif defined(CONFIG_SERIAL2)
	index = 2;
#elif defined(CONFIG_SERIAL3)
	index = 3;
#else
#error "CONFIG_SERIAL? missing."
#endif
	return pick_console(index);
}
#endif
