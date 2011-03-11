/*
 * Copyright 2011, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#define CONFIG_LCD_vl_col      1024
#define CONFIG_LCD_vl_row      600

#define TEGRA_GPIO_BACKLIGHT            TEGRA_GPIO_PB5
#define TEGRA_GPIO_LVDS_SHUTDOWN        TEGRA_GPIO_PB2
#define TEGRA_GPIO_BACKLIGHT_VDD        TEGRA_GPIO_PW0
#define TEGRA_GPIO_EN_VDD_PNL           TEGRA_GPIO_PC6

#define TEGRA_GPIO_INIT_LCD                                      \
        TEGRA_GPIO_INIT(TEGRA_GPIO_BACKLIGHT,     true)          \
        TEGRA_GPIO_INIT(TEGRA_GPIO_LVDS_SHUTDOWN, true)          \
        TEGRA_GPIO_INIT(TEGRA_GPIO_BACKLIGHT_VDD, false)         \
        TEGRA_GPIO_INIT(TEGRA_GPIO_EN_VDD_PNL,    true)

#define TEGRA_CLOCK_INIT_LCD                                    \
        TEGRA_CLOCK("host1x", "pll_p",   166000000, true)       \
        TEGRA_CLOCK("disp1",  "pll_p",   216000000, true)       \
        TEGRA_CLOCK("2d",     "pll_m",   266400000, true)       \
        TEGRA_CLOCK("3d",     "pll_m",   266400000, true)       \
        TEGRA_CLOCK("pwm",    "clk_32k", 32768,     true)

#define TEGRA_RESOURCE_LCD                                              \
        TEGRA_RESOURCE("irq",   0x69,               1)                  \
        TEGRA_RESOURCE("regs",  TEGRA_DISPLAY_BASE, TEGRA_DISPLAY_SIZE) \
        TEGRA_RESOURCE("fbmem", LCD_FB_ADDR,        0x4000000)          \
        TEGRA_RESOURCE("pwm",   TEGRA_PWFM0_BASE,   TEGRA_PWFM0_SIZE)

#define TEGRA_PANEL_MODE                                \
        TEGRA_PANEL(pclk,          79500000)            \
        TEGRA_PANEL(h_ref_to_sync, 4)                   \
        TEGRA_PANEL(v_ref_to_sync, 2)                   \
        TEGRA_PANEL(h_sync_width,  136)                 \
        TEGRA_PANEL(v_sync_width,  4)                   \
        TEGRA_PANEL(h_back_porch,  138)                 \
        TEGRA_PANEL(v_back_porch,  21)                  \
        TEGRA_PANEL(h_front_porch, 34)                  \
        TEGRA_PANEL(v_front_porch, 4)
