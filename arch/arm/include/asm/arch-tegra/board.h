/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _TEGRA_BOARD_H_
#define _TEGRA_BOARD_H_

/* Set up pinmux to make UART usable */
void gpio_early_init_uart(void);

/* Set up early UART output */
void board_init_uart_f(void);

/* Set up any early GPIOs the board might need for proper operation */
void gpio_early_init(void);  /* overrideable GPIO config        */

/*
 * Hooks to allow boards to set up the pinmux for a specific function.
 * Has to be implemented in the board files as we don't yet support pinmux
 * setup from FTD. If a board file does not implement one of those functions
 * an empty stub function will be called.
 */

void pin_mux_usb(void);      /* overrideable USB pinmux setup     */
void pin_mux_spi(void);      /* overrideable SPI pinmux setup     */
void pin_mux_nand(void);     /* overrideable NAND pinmux setup    */
void pin_mux_display(void);  /* overrideable DISPLAY pinmux setup */

/* Enable needed power rails */
void board_vreg_init(void);

int board_get_revision(void);

/* Currently only Venice2 and derivitives use a board ID */
#define VENICE2_ID		0	/* 0000 from board ID straps */
#define NORRIN_ERS_ID		4	/* 0100 from board ID straps */
#define NORRIN_PIX_ID		8	/* 1000 from board ID straps */
#define NORRIN_FFD_ID		9	/* 1001 from board ID straps */

#endif
