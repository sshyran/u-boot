/*
 *  (C) Copyright 2010,2011
 *  NVIDIA Corporation <www.nvidia.com>
 * Portions Copyright (c) 2011 The Chromium OS Authors
 *
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

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/sizes.h>

#define CONFIG_TEGRA2_LP0

#define CONFIG_SPI_UART_SWITCH

/* High-level configuration options */
#define TEGRA2_SYSMEM		"mem=384M@0M nvmem=128M@384M mem=512M@512M"
#define V_PROMPT		"Tegra2 # "

#include "tegra2-common.h"

#ifndef CONFIG_OF_CONTROL
/* Things in here are defined by the device tree now. Let it grow! */

#define CONFIG_TEGRA2_BOARD_STRING	"NVIDIA Seaboard"

#define CONFIG_TEGRA2_ENABLE_UARTD
#define CONFIG_SYS_NS16550_COM1		NV_PA_APB_UARTD_BASE

/* Seaboard SPI activity corrupts the first UART */
#define CONFIG_SPI_CORRUPTS_UART	NV_PA_APB_UARTD_BASE
#define CONFIG_SPI_CORRUPTS_UART_NR	3

/* On Seaboard: GPIO_PI3 = Port I = 8, bit = 3 */
#define UART_DISABLE_GPIO	GPIO_PI3

/* To select the order in which U-Boot sees USB ports */
#define CONFIG_TEGRA2_USB0      NV_PA_USB3_BASE
#define CONFIG_TEGRA2_USB1      NV_PA_USB1_BASE
#define CONFIG_TEGRA2_USB2      0
#define CONFIG_TEGRA2_USB3      0

/* Put USB1 in host mode */
#define CONFIG_TEGRA2_USB1_HOST
#define CONFIG_MACH_TYPE	MACH_TYPE_SEABOARD

/* Keyboard scan matrix configuration */
#define CONFIG_TEGRA2_KBC_PLAIN_KEYCODES {			\
	  0,    0,  'w',  's',  'a',  'z',    0,    KEY_FN,	\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	'5',  '4',  'r',  'e',  'f',  'd',  'x',    0,		\
	'7',  '6',  't',  'h',  'g',  'v',  'c',  ' ',		\
	'9',  '8',  'u',  'y',  'j',  'n',  'b', '\\',		\
	'-',  '0',  'o',  'i',  'l',  'k',  ',',  'm',		\
	  0,  '=',  ']', '\r',    0,    0,    0,    0,		\
	  0,    0,    0,    0, KEY_SHIFT, KEY_SHIFT,    0,    0,	\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	'[',  'p', '\'',  ';',  '/',  '.',    0,    0,		\
	  0,    0, 0x08,  '3',  '2',    0,    0,    0,		\
	  0, 0x7F,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,  'q',    0,    0,  '1',    0,		\
	0x1B,  '`',   0, 0x09,    0,    0,    0,    0		\
}

#define CONFIG_TEGRA2_KBC_SHIFT_KEYCODES {			\
	  0,    0,  'W',  'S',  'A',  'Z',    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	'%',  '$',  'R',  'E',  'F',  'D',  'X',    0,		\
	'&',  '^',  'T',  'H',  'G',  'V',  'C',  ' ',		\
	'(',  '*',  'U',  'Y',  'J',  'N',  'B',  '|',		\
	'_',  ')',  'O',  'I',  'L',  'K',  ',',  'M',		\
	  0,  '+',  '}', '\r',    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,    0,    0,    0,    0,    0,		\
	'{',  'P',  '"',  ':',  '?',  '>',    0,    0,		\
	  0,    0, 0x08,  '#',  '@',    0,    0,    0,		\
	  0, 0x7F,    0,    0,    0,    0,    0,    0,		\
	  0,    0,    0,  'Q',    0,    0,  '!',    0,		\
	0x1B, '~',    0, 0x09,    0,    0,    0,    0		\
}

#define CONFIG_TEGRA2_KBC_FUNCTION_KEYCODES {			\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	'7',     0,    0,    0,    0,    0,    0,    0,		\
	'9',   '8',  '4',    0,  '1',    0,    0,    0,		\
	  0,   '/',  '6',  '5',  '3',  '2',    0,  '0',		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,  '\'',    0,  '-',  '+',  '.',    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,    0,    0,    0,    0,		\
	  0,     0,    0,    0,  '?',    0,    0,    0		\
}

/* pin-mux settings for seaboard */
#define CONFIG_I2CP_PIN_MUX		1
#define CONFIG_I2C1_PIN_MUX		1
#define CONFIG_I2C2_PIN_MUX		2
#define CONFIG_I2C3_PIN_MUX		1

#endif /* CONFIG_OF_CONTROL not defined ^^^^^^^ */

#define CONFIG_TEGRA2_KEYBOARD
#define CONFIG_KEYBOARD

#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_STD_DEVICES_SETTINGS	"stdin=serial,tegra-kbc\0" \
					"stdout=serial,lcd\0" \
					"stderr=serial,lcd\0"

#define CONFIG_SYS_BOARD_ODMDATA	0x300d8011 /* lp1, 1GB */

/* GPIO */
#define CONFIG_TEGRA2_GPIO
#define CONFIG_CMD_TEGRA2_GPIO_INFO

/* SPI */
#define CONFIG_TEGRA2_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define CONFIG_CMD_SPI
#define CONFIG_CMD_SF

/* I2C */
#define CONFIG_TEGRA2_I2C
#define CONFIG_SYS_I2C_INIT_BOARD
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		4
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_CMD_I2C

/* TPM */
#define CONFIG_INFINEON_TPM_I2C
#define CONFIG_INFINEON_TPM_I2C_BUS	2
#define CONFIG_TPM_SLB9635_I2C
#define CONFIG_TPM_I2C_BURST_LIMITATION	3

/* SD/MMC */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_TEGRA2_MMC
#define CONFIG_CMD_MMC

#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT

/* Environment in SPI */
#define CONFIG_ENV_IS_IN_SPI_FLASH

#define CONFIG_ENV_SECT_SIZE    CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET       (SZ_4M - CONFIG_ENV_SECT_SIZE)

/*
 *  LCDC configuration
 */
#define CONFIG_LCD
#define CONFIG_VIDEO_TEGRA2

/* TODO: This needs to be configurable at run-time */
#define LCD_BPP             LCD_COLOR16
#define CONFIG_SYS_WHITE_ON_BLACK       /*Console colors*/

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_EXTRA_ENV_SETTINGS_COMMON \
	"board=seaboard\0" \

#define CONFIG_DEFAULT_DEVICE_TREE "tegra2-seaboard"

#endif /* __CONFIG_H */
