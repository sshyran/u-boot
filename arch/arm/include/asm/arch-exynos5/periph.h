/*
 * (C) Copyright 2012 The Chromium Authors
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
 *
 */

#ifndef __EXYNOS_PERIPH_H
#define __EXYNOS_PERIPH_H

#ifndef __ASSEMBLY__

/*
 * Peripherals requiring clock/pinmux configuration. List will
 * grow with support for more devices getting added.
 *
 * At present the order is arbitrary - we may be able to take advantage
 * of some orthogonality later.
 */
enum periph_id {
	PERIPH_ID_UART0,
	PERIPH_ID_UART1,
	PERIPH_ID_UART2,
	PERIPH_ID_UART3,
	PERIPH_ID_SDMMC0,
	PERIPH_ID_SDMMC1,
	PERIPH_ID_SDMMC2,
	PERIPH_ID_SDMMC3,
	PERIPH_ID_SDMMC4,
	PERIPH_ID_SROMC,
	PERIPH_ID_SPI0,
	PERIPH_ID_SPI1,
	PERIPH_ID_SPI2,
	PERIPH_ID_SPI3,
	PERIPH_ID_SPI4,
	PERIPH_ID_LCD,
	PERIPH_ID_BACKLIGHT,
	PERIPH_ID_I2C0,
	PERIPH_ID_I2C1,
	PERIPH_ID_I2C2,
	PERIPH_ID_I2C3,
	PERIPH_ID_I2C4,
	PERIPH_ID_I2C5,
	PERIPH_ID_I2C6,
	PERIPH_ID_I2C7,

	PERIPH_ID_COUNT,
	PERIPH_ID_NONE = -1,
};
#endif /* __ASSEMBLY__ */

#endif
