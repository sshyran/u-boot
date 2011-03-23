/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#ifndef __SEABOARD_FIRMWARE_LAYOUT_H__

/*
 * Use a smaller firmware image layout for Seaboard because it has
 * only 16MBit (=2MB) of SPI Flash.
 */

#define CONFIG_FIRMWARE_SIZE		0x00200000 /* 2 MB */

/* -- Region: Read-only ----------------------------------------------------- */

/* ---- Section: Read-only firmware ----------------------------------------- */

#define CONFIG_OFFSET_RO_SECTION	0x00000000
#define CONFIG_LENGTH_RO_SECTION	0x000f0000

#define CONFIG_OFFSET_BOOT_STUB		0x00000000
#define CONFIG_LENGTH_BOOT_STUB		0x00088000

#define CONFIG_OFFSET_RECOVERY		0x00088000
#define CONFIG_LENGTH_RECOVERY		0x00040000

#define CONFIG_OFFSET_RO_DATA		0x000c8000
#define CONFIG_LENGTH_RO_DATA		0x00008000

#define CONFIG_OFFSET_FMAP		0x000c8000
#define CONFIG_LENGTH_FMAP		0x00000400

#define CONFIG_OFFSET_GBB		0x000d0000
#define CONFIG_LENGTH_GBB		0x00020000

/* ---- Section: Vital-product data (VPD) ----------------------------------- */

#define CONFIG_OFFSET_RO_VPD		0x000f0000
#define CONFIG_LENGTH_RO_VPD		0x00010000

/* -- Region: Writable ------------------------------------------------------ */

/* ---- Section: Rewritable slot A ------------------------------------------ */

#define CONFIG_OFFSET_RW_SECTION_A	0x00100000
#define CONFIG_LENGTH_RW_SECTION_A	0x00078000

#define CONFIG_OFFSET_VBLOCK_A		0x00100000
#define CONFIG_LENGTH_VBLOCK_A		0x00010000

#define CONFIG_OFFSET_FW_MAIN_A		0x00110000
#define CONFIG_LENGTH_FW_MAIN_A		0x00068000

#define CONFIG_OFFSET_DATA_A		0x00100000
#define CONFIG_LENGTH_DATA_A		0x00000000

/* ---- Section: Rewritable slot B ------------------------------------------ */

#define CONFIG_OFFSET_RW_SECTION_B	0x00178000
#define CONFIG_LENGTH_RW_SECTION_B	0x00078000

#define CONFIG_OFFSET_VBLOCK_B		0x00178000
#define CONFIG_LENGTH_VBLOCK_B		0x00010000

#define CONFIG_OFFSET_FW_MAIN_B		0x00188000
#define CONFIG_LENGTH_FW_MAIN_B		0x00068000

#define CONFIG_OFFSET_DATA_B		0x00178000
#define CONFIG_LENGTH_DATA_B		0x00000000

/* ---- Section: Rewritable VPD --------------------------------------------- */

#define CONFIG_OFFSET_RW_VPD		0x001f0000
#define CONFIG_LENGTH_RW_VPD		0x00008000

/* ---- Section: Rewritable shared ------------------------------------------ */

#define CONFIG_OFFSET_RW_SHARED		0x001f8000
#define CONFIG_LENGTH_RW_SHARED		0x00008000

#define CONFIG_OFFSET_DEV_CFG		0x001f8000
#define CONFIG_LENGTH_DEV_CFG		0x00004000

#define CONFIG_OFFSET_SHARED_DATA	0x001fc000
#define CONFIG_LENGTH_SHARED_DATA	0x00003000

/* where are the meanings of these documented? Add a comment/link here */
#define CONFIG_OFFSET_ENV		0x001ff000
#define CONFIG_LENGTH_ENV		0x00001000

#endif /* __SEABOARD_FIRMWARE_LAYOUT_H__ */
