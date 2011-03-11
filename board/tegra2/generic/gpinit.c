/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include <common.h>

#include "../common/lcd/gpinit/gp-dc_reg.h"
#include "../common/lcd/gpinit/gpinit.h"
#include "../common/lcd/gpinit/gp-pinmux.h"
#include "../common/lcd/gpinit/gp-util.h"

struct tegra_gpio_init_table tegra2_gp_gpio_init_table[] = {
	TEGRA_GPIO_INIT_LCD
};

unsigned int tegra2_gp_gpio_offset_tab_len = ARRAY_SIZE(tegra2_gp_gpio_init_table);

struct tegra_clk_init_table tegra2_gp_clk_init_table[] = {
	TEGRA_CLOCK_INIT_LCD
	{ NULL,		NULL,		0,		0},
};

struct tegra_pingroup_config tegra2_gp_pinmux[] = { CONFIG_TEGRA2_PINMUX };

unsigned int tegra2_gp_pinmux_tab_len = ARRAY_SIZE(tegra2_gp_pinmux);

struct resource tegra2_gp_panel_resources[] = { TEGRA_RESOURCE_LCD };

struct tegra_dc_mode tegra2_gp_panel_modes[] = {
	{
		TEGRA_PANEL_MODE
		.h_active = CONFIG_LCD_vl_col,
		.v_active = CONFIG_LCD_vl_row,
	},
};

struct tegra_fb_data tegra2_gp_fb_data = {
	.win            = 0,
	.xres           = CONFIG_LCD_vl_col,
	.yres           = CONFIG_LCD_vl_row,
	.bits_per_pixel = 16,
};
