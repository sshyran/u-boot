/*
 * Copyright (c) 2013 Google, Inc
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

#include <common.h>
#include <fdtdec.h>
#include <lcd.h>
#include <malloc.h>
#include <asm/sdl.h>
#include <asm/u-boot-sandbox.h>

DECLARE_GLOBAL_DATA_PTR;

enum {
	/* Maximum LCD size we support */
	LCD_MAX_WIDTH		= 1366,
	LCD_MAX_HEIGHT		= 768,
	LCD_MAX_LOG2_BPP	= 4,		/* 2^4 = 16 bpp */
};

vidinfo_t panel_info __attribute__((section(".data")));


void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
}

void lcd_ctrl_init(void *lcdbase)
{
	/*
	 * Allocate memory to keep BMP color conversion map. This is required
	 * for 8 bit BMPs only (hence 256 colors). If malloc fails - keep
	 * going, it is not even clear if displyaing the bitmap will be
	 * required on the way up.
	 */
	panel_info.cmap = malloc(256 * NBITS(panel_info.vl_bpix) / 8);
}

void lcd_enable(void)
{
	if (sandbox_sdl_init_display(panel_info.vl_col, panel_info.vl_row,
				     panel_info.vl_bpix))
		puts("LCD init failed\n");
}

int sandbox_lcd_sdl_early_init(void)
{
	const void *blob = gd->fdt_blob;
	int xres, yres;
	int node;

	node = fdtdec_next_compatible(blob, 0, COMPAT_SANDBOX_LCD_SDL);
	if (node < 0)
		return 0;
	xres = fdtdec_get_int(blob, node, "xres", -1);
	yres = fdtdec_get_int(blob, node, "yres", -1);
	if (xres == -1 || yres == -1)
		return -1;

	panel_info.vl_col = xres;
	panel_info.vl_row = yres;
	panel_info.vl_bpix = 4;

	return 0;
}
