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

#include <common.h>
#include <lcd.h>
#include <fdt_decode.h>
#include <asm/clocks.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/power.h>
#include <asm/arch/pwfm.h>
#include <asm/arch/display.h>
#include <asm/system.h>

DECLARE_GLOBAL_DATA_PTR;

unsigned long timer_last;

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

void *lcd_base;			/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/

short console_col;
short console_row;

vidinfo_t panel_info = {
	/* Insert a value here so that we don't end up in the BSS */
	.vl_col = -1,
};

char lcd_cursor_enabled = 0;	/* set initial value to false */

ushort lcd_cursor_width;
ushort lcd_cursor_height;


static void clk_init(void)
{
	/* TODO: Put this into the FDT when we have clock support there */
	clock_start_periph_pll(PERIPH_ID_3D, CLOCK_ID_MEMORY, CLK_300M);
	clock_start_periph_pll(PERIPH_ID_2D, CLOCK_ID_MEMORY, CLK_300M);
	clock_start_periph_pll(PERIPH_ID_HOST1X, CLOCK_ID_PERIPH, CLK_144M);
	clock_start_periph_pll(PERIPH_ID_DISP1, CLOCK_ID_CGENERAL, CLK_600M);
	clock_start_periph_pll(PERIPH_ID_PWM, CLOCK_ID_SFROM32KHZ, CLK_32768);
}

/*
 * The PINMUX macro is used per board to setup the pinmux configuration.
 */
#define PINMUX(grp, mux, pupd, tri)                   \
        {PINGRP_##grp, PMUX_FUNC_##mux, PMUX_PULL_##pupd, PMUX_TRI_##tri}

struct pingroup_config pinmux_cros_1[] = {
	PINMUX(GPU,   PWM,        NORMAL,    NORMAL),
	PINMUX(LD0,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD1,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD10,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD11,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD12,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD13,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD14,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD15,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD16,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD17,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LD2,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD3,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD4,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD5,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD6,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD7,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD8,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LD9,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LDI,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP0,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP1,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHP2,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LHS,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LM0,   RSVD4,      NORMAL,    NORMAL),
	PINMUX(LPP,   DISPA,      NORMAL,    NORMAL),
	PINMUX(LPW0,  RSVD4,      NORMAL,    NORMAL),
	PINMUX(LPW1,  RSVD4,      NORMAL,    TRISTATE),
	PINMUX(LPW2,  RSVD4,      NORMAL,    NORMAL),
	PINMUX(LSC0,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LSPI,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LVP1,  DISPA,      NORMAL,    NORMAL),
	PINMUX(LVS,   DISPA,      NORMAL,    NORMAL),
	PINMUX(SLXD,  SPDIF,      NORMAL,    NORMAL),
};

/* Initialize the Tegra LCD pinmuxs */
static void init_lcd_pinmux(struct fdt_lcd *config)
{
	/* TODO: put pinmux into the FDT */
	pinmux_config_table(pinmux_cros_1, ARRAY_SIZE(pinmux_cros_1));

	fdt_setup_gpio(&config->panel_vdd);
	fdt_setup_gpio(&config->lvds_shutdown);
	fdt_setup_gpio(&config->backlight_vdd);
	fdt_setup_gpio(&config->backlight_en);

	gpio_set_value(config->panel_vdd.gpio, 1);
	udelay(config->panel_timings[0] * 1000);

	gpio_set_value(config->lvds_shutdown.gpio, 1);
	timer_last = timer_get_us();
}

/* Initialize the Tegra LCD panel and controller */
void init_lcd(struct fdt_lcd *config)
{
	clk_init();
	power_enable_partition(POWERP_3D);
	tegra2_display_register(config);
}

static void init_lcd_pwm(struct fdt_lcd *config)
{
	long pre_delay;

	/*
	 * Compare the current timer valuse with the timer_last to
	 * figure out the pre_delay which is the passed time from
	 * the end of init_lcd_pinmux to now.
	 * If the required delay(timings[1]) is bigger than pre_delay,
	 * we have to do a delay (timings[1] - pre_delay) to make
	 * sure the required delay(timings[1]) has passed.
	 */
	pre_delay = timer_get_us() - timer_last;

	if ((long)(config->panel_timings[1] * 1000) > pre_delay)
		udelay((long)(config->panel_timings[1] * 1000) - pre_delay);

	gpio_set_value(config->backlight_vdd.gpio, 1);
	udelay(config->panel_timings[2] * 1000);

	/* Enable PWM at 15/16 high, divider 1 */
	pwfm_setup(config->pwfm, 1, 0xdf, 1);
	udelay(config->panel_timings[3] * 1000);

	gpio_set_value(config->backlight_en.gpio, 1);
}

void lcd_cursor_size(ushort width, ushort height)
{
	lcd_cursor_width = width;
	lcd_cursor_height = height;
}

void lcd_toggle_cursor(void)
{
	ushort x, y;
	uchar *dest;
	ushort row;

	x = console_col * lcd_cursor_width;
	y = console_row * lcd_cursor_height;
	dest = (uchar *)(lcd_base + y * lcd_line_length + x * (1 << LCD_BPP) /
			8);

	for (row = 0; row < lcd_cursor_height; ++row, dest += lcd_line_length) {
		ushort *d = (ushort *)dest;
		ushort color;
		int i;

		for (i = 0; i < lcd_cursor_width; ++i) {
			color = *d;
			color ^= lcd_color_fg;
			*d = color;
			++d;
		}
	}
}

void lcd_cursor_on(void)
{
	lcd_cursor_enabled = 1;
	lcd_toggle_cursor();
}
void lcd_cursor_off(void)
{
	lcd_cursor_enabled = 0;
	lcd_toggle_cursor();
}

char lcd_is_cursor_enabled(void)
{
	return lcd_cursor_enabled;
}

static void update_panel_size(struct fdt_lcd *config)
{
	panel_info.vl_col = config->width;
	panel_info.vl_row = config->height;
	panel_info.vl_bpix = config->log2_bpp;
}

/*
 *  Main init function called by lcd driver.
 *  Inits and then prints test pattern if required.
 */

void lcd_ctrl_init(void *lcdbase)
{
	struct fdt_lcd config;
	int line_length, size;

	/* get panel details */
	if (fdt_decode_lcd(gd->blob, &config)) {
		printf("No LCD information in device tree\n");
		return;
	}

	/*
	 * The device tree allows for the frame buffer to be specified if
	 * needed, but for now, U-Boot will set this. This may change if
	 * we find that Linux is unable to use the address that U-Boot picks,
	 * and this causes screen flicker.
	 */
	config.frame_buffer = (u32)lcd_base;
	update_panel_size(&config);
	size = lcd_get_size(&line_length),

	/* call board specific hw init */
	init_lcd(&config);

	/* For write-through or cache off, change the LCD memory region */
	if (!(config.cache_type & FDT_LCD_CACHE_WRITE_BACK))
		mmu_set_region_dcache(config.frame_buffer, size,
			config.cache_type & FDT_LCD_CACHE_WRITE_THROUGH ?
				DCACHE_WRITETHROUGH : DCACHE_OFF);

	/* Enable flushing after LCD writes if requested */
	lcd_set_flush_dcache(config.cache_type & FDT_LCD_CACHE_FLUSH);

	debug("LCD frame buffer at %p\n", lcd_base);
}

ulong calc_fbsize(void)
{
	return (panel_info.vl_col * panel_info.vl_row *
		NBITS(panel_info.vl_bpix)) / 8;
}

void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
}

void lcd_enable(void)
{
	struct fdt_lcd config;

	/* get panel details */
	if (fdt_decode_lcd(gd->blob, &config)) {
		printf("No LCD information in device tree\n");
		return;
	}

	/* call board specific hw init for backlight PWM */
	init_lcd_pwm(&config);
}

void lcd_early_init(const void *blob)
{
	struct fdt_lcd config;

	/* get panel details */
	if (!fdt_decode_lcd(gd->blob, &config))
		update_panel_size(&config);
}

int lcd_pinmux_early_init(const void *blob)
{
	struct fdt_lcd config;

	/* get panel details */
	if (fdt_decode_lcd(gd->blob, &config))
		return -1;

	init_lcd_pinmux(&config);
	return 0;
}
