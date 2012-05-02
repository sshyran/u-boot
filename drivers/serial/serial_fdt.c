/*
 * Copyright (c) 2011 The Chromium OS Authors.
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * This is a serial port provided by the flattened device tree. It works
 * by selecting a compiled in driver according to the setting in the FDT.
 */

#include <common.h>
#include "serial_fdt.h"
#ifdef CONFIG_SYS_NS16550
#include <ns16550.h>
#endif
#include <serial.h>

DECLARE_GLOBAL_DATA_PTR;

static struct serial_device *console_ptr
	__attribute__((section(".data"))) = NULL;

static struct serial_device *get_console(void)
{
#ifdef CONFIG_S5P
	console_ptr = serial_s5p_fdt_init();
	if (console_ptr)
		return console_ptr;
#endif
#ifdef CONFIG_SYS_NS16550
	console_ptr = NS16550_fdt_init();
	if (console_ptr)
		return console_ptr;
#endif
	return NULL;
}

struct serial_device *serial_fdt_get_console_r(void)
{
	/*
	 * Relocation moves all our function pointers, so we need to set up
	 * things again. This function will only be called once.
	 *
	 * We cannot do the -1 check as in default_serial_console()
	 * because it will be -1 if that function has been ever been called.
	 * However, the function pointers set up by serial_fdt_get_console_f
	 * will be pre-relocation values, so we must re-calculate them.
	 */
	return get_console();
}

struct serial_device *default_serial_console(void)
{
	static int relocated;

	/*
	 * console_ptr needs to be relocated manually because it wasn't set up
	 * by the linker and doesn't have a relocation entry.
	 */
	if (!relocated && (gd->flags & GD_FLG_RELOC)) {
		uintptr_t console_addr;

		/*
		 * If console_ptr is NULL, leave it alone. Otherwise
		 * relocate it.
		 */
		if (console_ptr) {
			console_addr = (uintptr_t)console_ptr;
			console_addr += gd->reloc_off;
			console_ptr = (struct serial_device *)console_addr;
		}
		relocated = 1;
	}
	if (console_ptr)
		return console_ptr;
	else
		return get_console();
}
