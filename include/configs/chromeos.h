/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef __configs_chromeos_h__
#define __configs_chromeos_h__

/*
 * This config file defines platform-independent settings that a verified boot
 * firmware must have.
 */

/* Enable verified boot */
#define CONFIG_CHROMEOS

/* Enable test codes */
#ifdef VBOOT_DEBUG
#define CONFIG_CHROMEOS_TEST
#endif /* VBOOT_DEBUG */

/* Enable graphics display */
#define CONFIG_LCD_BMP_RLE8
#define CONFIG_LZMA
#define CONFIG_SPLASH_SCREEN

/*
 * Use the fdt to decide whether to load the environment early in start-up
 * (even before we decide if we're entering developer mode).
 */
#define CONFIG_OF_LOAD_ENVIRONMENT

/*
 * Enable this feature to embed crossystem data into device tree before booting
 * the kernel.
 */
/*
 * TODO If x86 firmware does not embedding crossystem data in a device tree (and
 * pass the tree to kernel) but uses ACPI or whatever instead, move this to
 * chromeos_tegra2_twostop.h.
 */
#define CONFIG_OF_UPDATE_FDT_BEFORE_BOOT

/*
 * We use fixed memory address for the parameters --- this should be simpler
 * than atags or a register holding the address of the parameters. Besides,
 * Chrome OS kernel is loaded to a fixed location, we could simply use this
 * location as our anchor for the location of the parameters.
 */
/*
 * Layout: first, the kernel buffer, then the crossystem data (and the
 * VbSharedData), and finally, the GBB.
 */
/* For security reason, Chrome OS kernel must be loaded to specific location. */
#define CHROMEOS_KERNEL_LOADADDR	0x00100000
#define CHROMEOS_KERNEL_BUFSIZE		0x00800000
#define CROSSYSTEM_DATA_ADDRESS \
	(CHROMEOS_KERNEL_LOADADDR + CHROMEOS_KERNEL_BUFSIZE)
#define CROSSYSTEM_DATA_MAXSIZE		0x8000
#define GBB_ADDRESS \
	(CROSSYSTEM_DATA_ADDRESS + CROSSYSTEM_DATA_MAXSIZE)

/*
 * This is the default kernel command line to a Chrome OS kernel. An ending
 * space character helps us concatenate more arguments.
 */
#define CHROMEOS_BOOTARGS "cros_secure " CONFIG_BOOTARGS " "

#endif /* __configs_chromeos_h__ */
