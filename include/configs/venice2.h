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

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/sizes.h>

#include "tegra124-common.h"

/* Use memory controller SDRAM size instead of ODMDATA */
#define CONFIG_TEGRA_USE_EMC_DRAM_SIZE

/* Enable fdt support for Venice2. Flash the image in u-boot-dtb.bin */
#define CONFIG_DEFAULT_DEVICE_TREE	tegra124-venice2
#define CONFIG_OF_CONTROL
#define CONFIG_OF_SEPARATE
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_OF_SPI
#define CONFIG_OF_SPI_FLASH

/* High-level configuration options */
#define V_PROMPT			"Tegra124 (Venice2) # "
#define CONFIG_TEGRA_BOARD_STRING	"NVIDIA Venice2"

/* Board-specific serial config */
#define CONFIG_SERIAL_MULTI
#define CONFIG_TEGRA_ENABLE_UARTA
#define CONFIG_SYS_NS16550_COM1		NV_PA_APB_UARTA_BASE

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_ARCH_EARLY_INIT_R

/* I2C */
#define CONFIG_TEGRA_I2C
#define CONFIG_SYS_I2C_INIT_BOARD
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		TEGRA_I2C_NUM_CONTROLLERS
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_CMD_I2C

/* SD/MMC */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_TEGRA_MMC
#define CONFIG_CMD_MMC

/* Environment in eMMC, at the end of 2nd "boot sector" */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_MMC_ENV_PART		2
#define CONFIG_ENV_OFFSET		((4096 * 1024) - CONFIG_ENV_SIZE)

/* SPI */
#define CONFIG_TEGRA114_SPI		/* Compatible w/T114 SPI */
#define CONFIG_TEGRA114_SPI_CTRLS	6
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SF_DEFAULT_MODE         SPI_MODE_0
#define CONFIG_SF_DEFAULT_SPEED        24000000
#define CONFIG_CMD_SPI
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_SIZE          (4 << 20)

/* Backlight enable GPIO. TBD - move to DT when LCD driver added */
#define DSI_PANEL_BL_EN_GPIO		GPIO_PH2
/* Touchscreen GPIO, TS_SHDN_L */
#define TS_SHDN_L_GPIO			GPIO_PK1

/* USB Host support */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_TEGRA
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_USB

/* USB networking support */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX

/* General networking support */
#define CONFIG_CMD_NET
#define CONFIG_CMD_DHCP

/* FIT image support */
#define CONFIG_FIT
#define CONFIG_FIT_BEST_MATCH

/* USB keyboard */
#define CONFIG_USB_KEYBOARD

/* TPM */
#define CONFIG_INFINEON_TPM_I2C
#define CONFIG_CMD_TPM
#define CONFIG_TPM
#define CONFIG_TPM_TIS_I2C

#include "tegra-common-post.h"

#endif /* __CONFIG_H */
