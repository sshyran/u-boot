/*
 * COM1 NS16550 support
 * originally from linux source (arch/powerpc/boot/ns16550.c)
 * modified to use CONFIG_SYS_ISA_MEM and new defines
 */

#include <config.h>
#include <ns16550.h>
#include <fdtdec.h>
#include <serial.h>
#include "serial_fdt.h"

/* Information obtained about a UART from the FDT */
struct fdt_uart {
	fdt_addr_t reg;	/* address of registers in physical memory */
	int reg_shift;	/* each register is (1 << reg_shift) apart */
	int baudrate;	/* baud rate, will be gd->baudrate if not defined */
	int clock_freq;	/* clock frequency, -1 if not provided */
	int multiplier;	/* divisor multiplier, default 16 */
	int divisor;	/* baud rate divisor, default calculated */
	int enabled;	/* 1 to enable, 0 to disable */
	int interrupt;	/* interrupt line */
	int silent;	/* 1 for silent UART (supresses output by default) */
	int io_mapped;	/* 1 for IO mapped UART, 0 for memory mapped UART */
	enum fdt_compat_id compat; /* our selected driver */
};

struct fdt_uart console_uart = {
	-1U
};

static void uart_calc_divisor(struct fdt_uart *uart)
{
	if (uart->multiplier && uart->baudrate)
		uart->divisor = (uart->clock_freq +
				(uart->baudrate * (uart->multiplier / 2))) /
			(uart->multiplier * uart->baudrate);
}

static int decode_uart_console(const void *blob, struct fdt_uart *uart,
		int default_baudrate)
{
	int node;

	node = fdtdec_find_alias_node(blob, "console");
	if (node < 0)
		return node;
	if (fdt_node_check_compatible(blob, node, "ns16550") != 0)
		return 1;
	uart->reg = fdtdec_get_addr(blob, node, "reg");
	uart->reg_shift = fdtdec_get_int(blob, node, "reg-shift", 2);
	uart->baudrate = fdtdec_get_int(blob, node, "baudrate",
					default_baudrate);
	uart->clock_freq = fdtdec_get_int(blob, node, "clock-frequency", -1);
	uart->multiplier = fdtdec_get_int(blob, node, "multiplier", 16);
	uart->divisor = fdtdec_get_int(blob, node, "divisor", -1);
	uart->enabled = fdtdec_get_is_enabled(blob, node);
	uart->interrupt = fdtdec_get_int(blob, node, "interrupts", -1);
	uart->silent = fdtdec_get_config_int(blob, "silent_console", 0);
	uart->io_mapped = fdtdec_get_int(blob, node, "io-mapped", 0);
	uart->compat = fdtdec_lookup(blob, node);

	/* Calculate divisor if required */
	if ((uart->divisor == -1) && (uart->clock_freq != -1))
		uart_calc_divisor(uart);
	return 0;
}

static int NS16550_fdt_wrap_init(void)
{
	NS16550_init((NS16550_t)console_uart.reg, console_uart.divisor);
#ifdef CONFIG_SILENT_CONSOLE
	/* if the console UART wants to be silent, do this now */
	if (console_uart.silent)
		gd->flags |= GD_FLG_SILENT;
#endif
	return 0;
}

static void NS16550_fdt_wrap_putc(const char c)
{
	if (c == '\n')
		NS16550_fdt_wrap_putc('\r');
	NS16550_putc((NS16550_t)console_uart.reg, c);
}

static void NS16550_fdt_wrap_puts(const char *s)
{
	while (*s)
		NS16550_fdt_wrap_putc(*s++);
}

static void NS16550_fdt_wrap_setbrg(void)
{
	console_uart.baudrate = gd->baudrate;
	uart_calc_divisor(&console_uart);
	NS16550_reinit((NS16550_t)console_uart.reg, console_uart.divisor);
}

static int NS16550_fdt_wrap_getc(void)
{
	return NS16550_getc((NS16550_t)console_uart.reg);
}

static int NS16550_fdt_wrap_tstc(void)
{
	return NS16550_tstc((NS16550_t)console_uart.reg);
}

static struct serial_device console = {
	"ns16550",
	NS16550_fdt_wrap_init,
	NULL,
	NS16550_fdt_wrap_setbrg,
	NS16550_fdt_wrap_getc,
	NS16550_fdt_wrap_tstc,
	NS16550_fdt_wrap_putc,
	NS16550_fdt_wrap_puts
};

struct serial_device *NS16550_fdt_init(void)
{
	if (decode_uart_console(gd->fdt_blob, &console_uart, gd->baudrate))
		return NULL;
#ifdef CONFIG_SYS_NS16550_RUNTIME_MAPPED
	NS16550_is_io_mapped(console_uart.io_mapped);
#endif
	return &console;
}
