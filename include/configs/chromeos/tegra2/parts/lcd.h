/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#define CONFIG_LCD
#define CONFIG_TEGRA2_LCD
#define LCD_BPP             LCD_COLOR16
#define LCD_FB_ADDR         0x1C022000   /* FB could be passed from bl */
#define CONFIG_SYS_WHITE_ON_BLACK       /*Console colors*/

#define TEGRA_GPIO_BACKLIGHT_CHROMEOS_1     TEGRA_GPIO_PD4
#define TEGRA_GPIO_LVDS_SHUTDOWN_CHROMEOS_1 TEGRA_GPIO_PB2
#define TEGRA_GPIO_BACKLIGHT_VDD_CHROMEOS_1 TEGRA_GPIO_PW0
#define TEGRA_GPIO_EN_VDD_PNL_CHROMEOS_1    TEGRA_GPIO_PC6

#define TEGRA_GPIO_INIT_LCD_CHROMEOS_1                                  \
        TEGRA_GPIO_INIT(TEGRA_GPIO_BACKLIGHT_CHROMEOS_1,     true)      \
        TEGRA_GPIO_INIT(TEGRA_GPIO_LVDS_SHUTDOWN_CHROMEOS_1, true)      \
        TEGRA_GPIO_INIT(TEGRA_GPIO_BACKLIGHT_VDD_CHROMEOS_1, false)     \
        TEGRA_GPIO_INIT(TEGRA_GPIO_EN_VDD_PNL_CHROMEOS_1,    true)

#define TEGRA_CLOCK_INIT_LCD_CHROMEOS_1                         \
        TEGRA_CLOCK("3d",     "pll_m",   300000000, true)       \
        TEGRA_CLOCK("2d",     "pll_m",   300000000, true)       \
        TEGRA_CLOCK("host1x", "pll_p",   144000000, true)       \
        TEGRA_CLOCK("disp1",  "pll_p",   216000000, true)       \
        TEGRA_CLOCK("pwm",    "clk_32k", 32768,     true)

#define TEGRA_RESOURCE_LCD_CHROMEOS_1                                   \
        TEGRA_RESOURCE("irq",   0x69,               1)                  \
        TEGRA_RESOURCE("regs",  TEGRA_DISPLAY_BASE, TEGRA_DISPLAY_SIZE) \
        TEGRA_RESOURCE("fbmem", LCD_FB_ADDR,        0x4000000)          \
        TEGRA_RESOURCE("pwm",   TEGRA_PWFM2_BASE,   TEGRA_PWFM2_SIZE)

#define TEGRA_PANEL_MODE_CHROMEOS_1                     \
        TEGRA_PANEL(pclk,          62200000)            \
        TEGRA_PANEL(h_ref_to_sync, 11)                  \
        TEGRA_PANEL(v_ref_to_sync, 1)                   \
        TEGRA_PANEL(h_sync_width,  58)                  \
        TEGRA_PANEL(v_sync_width,  4)                   \
        TEGRA_PANEL(h_back_porch,  58)                  \
        TEGRA_PANEL(v_back_porch,  4)                   \
        TEGRA_PANEL(h_front_porch, 58)                  \
        TEGRA_PANEL(v_front_porch, 4)
