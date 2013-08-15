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

/* Tegra124 high-level function multiplexing */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/funcmux.h>
#include <asm/arch/pinmux.h>

int funcmux_select(enum periph_id id, int config)
{
	int bad_config = config != FUNCMUX_DEFAULT;

	switch (id) {
	case PERIPH_ID_UART4:
		switch (config) {
		case FUNCMUX_UART4_GPIO: /* TXD,RXD,CTS,RTS */
			pinmux_set_func(PINGRP_GPIO_PJ7, PMUX_FUNC_UARTD);
			pinmux_set_func(PINGRP_GPIO_PB0, PMUX_FUNC_UARTD);
			pinmux_set_func(PINGRP_GPIO_PB1, PMUX_FUNC_UARTD);
			pinmux_set_func(PINGRP_GPIO_PK7, PMUX_FUNC_UARTD);

			pinmux_set_io(PINGRP_GPIO_PJ7, PMUX_PIN_OUTPUT);
			pinmux_set_io(PINGRP_GPIO_PB0, PMUX_PIN_INPUT);
			pinmux_set_io(PINGRP_GPIO_PB1, PMUX_PIN_INPUT);
			pinmux_set_io(PINGRP_GPIO_PK7, PMUX_PIN_OUTPUT);

			pinmux_tristate_disable(PINGRP_GPIO_PJ7);
			pinmux_tristate_disable(PINGRP_GPIO_PB0);
			pinmux_tristate_disable(PINGRP_GPIO_PB1);
			pinmux_tristate_disable(PINGRP_GPIO_PK7);
			break;
		}
		break;

	case PERIPH_ID_UART1:
		switch (config) {
		case FUNCMUX_UART1_KBC:
			pinmux_set_func(PINGRP_KB_ROW9, PMUX_FUNC_UARTA);
			pinmux_set_func(PINGRP_KB_ROW10, PMUX_FUNC_UARTA);

			pinmux_set_io(PINGRP_KB_ROW9, PMUX_PIN_OUTPUT);
			pinmux_set_io(PINGRP_KB_ROW10, PMUX_PIN_INPUT);

			pinmux_tristate_disable(PINGRP_KB_ROW9);
			pinmux_tristate_disable(PINGRP_KB_ROW10);
			break;
		}
		break;

	/* Add other periph IDs here as needed */

	default:
		debug("%s: invalid periph_id %d", __func__, id);
		return -1;
	}

	if (bad_config) {
		debug("%s: invalid config %d for periph_id %d", __func__,
		      config, id);
		return -1;
	}
	return 0;
}
