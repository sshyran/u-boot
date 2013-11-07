/*
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __CHROMEOS_VENICE2_CONFIG_H
#define __CHROMEOS_VENICE2_CONFIG_H

#define CONFIG_EXTRA_BOOTARGS \
	"console=ttyS0,115200n8 " \
	"noinitrd " \
	"usbcore.old_scheme_first=1 \0 "

/* Add chromeos specific only for non spl build */
#ifndef CONFIG_SPL_BUILD

/* Support vboot flag reading from GPIO hardware */
#define CONFIG_CHROMEOS_GPIO_FLAG

/* Support vboot flag reading from EC */
#define CONFIG_CHROMEOS_CROS_EC_FLAG

#define CONFIG_DIRECT_BOOTARGS ""
#define CONFIG_STD_DEVICES_SETTINGS ""
#include <configs/chromeos.h>

#define CONFIG_PHYSMEM
#define CONFIG_TPM
#define CONFIG_INFINEON_TPM_I2C
#define CONFIG_CMD_TIME

#define CONFIG_CROS_EC			/* CROS_EC protocol */
#define CONFIG_CROS_EC_SPI		/* Support CROS_EC over SPI */
#define CONFIG_CMD_CROS_EC

/* Enable keyboard */
#define CONFIG_CROS_EC_KEYB		/* CROS_EC keyboard input */
#define CONFIG_KEYBOARD

#endif
#include <configs/venice2.h>

/* High-level configuration options */
#ifdef V_PROMPT
#undef V_PROMPT
#endif
#define V_PROMPT		"Tegra124 (ChromeOS Venice2) # "

#ifdef CONFIG_TEGRA_BOARD_STRING
#undef CONFIG_TEGRA_BOARD_STRING
#endif
#define CONFIG_TEGRA_BOARD_STRING	"NVIDIA ChromeOS Venice2"

#define CONFIG_DEVICE_TREE_LIST		"tegra124-venice2"

#endif /* __CHROMEOS_VENICE2_CONFIG_H */
