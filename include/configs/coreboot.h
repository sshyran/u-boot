/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * (C) Copyright 2008
 * Graeme Russ, graeme.russ@gmail.com.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/ibmpc.h>
/*
 * board/config.h - configuration options, board specific
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_X86
#define CONFIG_SYS_COREBOOT
#undef CONFIG_SHOW_BOOT_PROGRESS
#define CONFIG_LAST_STAGE_INIT


/*-----------------------------------------------------------------------
 * Watchdog Configuration
 * NOTE: If CONFIG_HW_WATCHDOG is NOT defined, the watchdog jumper on the
 * bottom (processor) board MUST be removed!
 */
#undef CONFIG_WATCHDOG
#define CONFIG_HW_WATCHDOG

/*-----------------------------------------------------------------------
 * Real Time Clock Configuration
 */
#define CONFIG_RTC_MC146818
#define CONFIG_SYS_ISA_IO_BASE_ADDRESS	0
#define CONFIG_SYS_ISA_IO	CONFIG_SYS_ISA_IO_BASE_ADDRESS

/*-----------------------------------------------------------------------
 * Serial Configuration
 */
#define CONFIG_SERIAL_MULTI
#define CONFIG_BAUDRATE			9600
#define CONFIG_SYS_BAUDRATE_TABLE	{300, 600, 1200, 2400, 4800, \
					 9600, 19200, 38400, 115200}
#define CONFIG_SYS_NS16550_REG_SIZE	1
#define CONFIG_SYS_OXPCIE952

#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_STD_DEVICES_SETTINGS     "stdin=vga,serial\0" \
                                        "stdout=vga,serial\0" \
                                        "stderr=vga,serial\0"

/* max. 1 IDE bus	*/
#define CONFIG_SYS_IDE_MAXBUS		1
/* max. 1 drive per IDE bus */
#define CONFIG_SYS_IDE_MAXDEVICE	(CONFIG_SYS_IDE_MAXBUS * 1)

#define CONFIG_SYS_ATA_BASE_ADDR	CONFIG_SYS_ISA_IO_BASE_ADDRESS
#define CONFIG_SYS_ATA_IDE0_OFFSET	0x01f0
#define CONFIG_SYS_ATA_IDE1_OFFSET	0x0170
#define CONFIG_SYS_ATA_DATA_OFFSET	0
#define CONFIG_SYS_ATA_REG_OFFSET	0
#define CONFIG_SYS_ATA_ALT_OFFSET	0x200


#define CONFIG_SUPPORT_VFAT
/************************************************************
 * ATAPI support (experimental)
 ************************************************************/
#define CONFIG_ATAPI

/************************************************************
 * DISK Partition support
 ************************************************************/
#define CONFIG_EFI_PARTITION
#define CONFIG_DOS_PARTITION
#define CONFIG_MAC_PARTITION
#define CONFIG_ISO_PARTITION		/* Experimental */


/*-----------------------------------------------------------------------
 * Video Configuration
 */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_COREBOOT
#define CONFIG_VIDEO_SW_CURSOR
#define VIDEO_FB_16BPP_WORD_SWAP
#define CONFIG_I8042_KBD
#define CONFIG_CFB_CONSOLE

/*-----------------------------------------------------------------------
 * Device tree configuration.
 */
#define CONFIG_OF_LIBFDT

/*-----------------------------------------------------------------------
 * Command line configuration.
 */
#include <config_cmd_default.h>

#define CONFIG_CMD_BDI
#define CONFIG_CMD_BOOTD
#define CONFIG_CMD_CONSOLE
#define CONFIG_CMD_DATE
#define CONFIG_CMD_ECHO
#undef CONFIG_CMD_FLASH
#define CONFIG_CMD_FPGA
#define CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#define CONFIG_CMD_IRQ
#define CONFIG_CMD_ITEST
#define CONFIG_CMD_LOADB
#define CONFIG_CMD_LOADS
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_MISC
#define CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#define CONFIG_CMD_PCI
#define CONFIG_CMD_PING
#define CONFIG_CMD_RUN
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_SETGETDCR
#define CONFIG_CMD_SOURCE
#define CONFIG_CMD_XIMG
#define CONFIG_CMD_IDE
#define CONFIG_CMD_CBFS
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_USB

#define CONFIG_BOOTDELAY			-1
#undef  CONFIG_BOOTARGS

#define CONFIG_BOOTCOMMAND			"run set_bootargs; "\
						"fatload ${devtype} ${devnum}:c 3000000 syslinux/vmlinuz.a; "\
						"zboot 3000000; "

#define CONFIG_EXTRA_ENV_SETTINGS       	"devtype=ide\0"\
						"devnum=0\0"\
						"devname=sda\0"\
						CONFIG_STD_DEVICES_SETTINGS \
						"set_bootargs=setenv bootargs "\
							"console=uart8250,mmio,0xe0401000,115200n8 "\
							"root=/dev/${devname}3 "\
							"init=/sbin/init "\
							"i915.modeset=1 "\
							"rootwait "\
							"ro "\
							"cros_legacy\0"\
						"usb_boot=usb start;"\
							"setenv devtype usb;"\
							"setenv devnum 1;"\
							"setenv devname sdb;"\
							"run bootcmd\0" \
						"mmc_boot=usb start;"\
							"setenv devtype usb;"\
							"setenv devnum 0;"\
							"setenv devname sdb;"\
							"run bootcmd\0" \
						""


#if defined(CONFIG_CMD_KGDB)
#define CONFIG_KGDB_BAUDRATE			115200
#define CONFIG_KGDB_SER_INDEX			2
#endif

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT			"boot > "
#define CONFIG_SYS_CBSIZE			256
#define CONFIG_SYS_PBSIZE			(CONFIG_SYS_CBSIZE + \
						 sizeof(CONFIG_SYS_PROMPT) + \
						 16)
#define CONFIG_SYS_MAXARGS			16
#define CONFIG_SYS_BARGSIZE			CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START		0x00100000
#define CONFIG_SYS_MEMTEST_END			0x01000000
#define CONFIG_SYS_LOAD_ADDR			0x100000
#define CONFIG_SYS_HZ				1000

#define CONFIG_NO_RESET_CODE
#define CONFIG_NO_REALMODE_CODE
#define CONFIG_ZBOOT_32

/*-----------------------------------------------------------------------
 * SDRAM Configuration
 */
#define CONFIG_NR_DRAM_BANKS			4

/* CONFIG_SYS_SDRAM_DRCTMCTL Overrides the following*/
#undef CONFIG_SYS_SDRAM_PRECHARGE_DELAY
#undef CONFIG_SYS_SDRAM_RAS_CAS_DELAY
#undef CONFIG_SYS_SDRAM_CAS_LATENCY_2T
#undef CONFIG_SYS_SDRAM_CAS_LATENCY_3T

/*-----------------------------------------------------------------------
 * CPU Features
 */

#define CONFIG_SYS_GENERIC_TIMER
#define CONFIG_SYS_PCAT_INTERRUPTS
#define CONFIG_SYS_NUM_IRQS			16

/*-----------------------------------------------------------------------
 * Memory organization:
 * 32kB Stack
 * 16kB Cache-As-RAM @ 0x19200000
 * 256kB Monitor
 * (128kB + Environment Sector Size) malloc pool
 */
#define CONFIG_SYS_STACK_SIZE			(32 * 1024)
#define CONFIG_SYS_INIT_SP_ADDR		(256 * 1024 + 16 * 1024)
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_MONITOR_LEN			(256 * 1024)
#define CONFIG_SYS_MALLOC_LEN			(0x20000 + 128 * 1024)
/* Address of temporary Global Data */
#define CONFIG_SYS_INIT_GD_ADDR		(256 * 1024)


/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

/*-----------------------------------------------------------------------
 * FLASH configuration
 */
#define CONFIG_SYS_NO_FLASH
#undef CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_MAX_FLASH_SECT		1
#define CONFIG_SYS_MAX_FLASH_BANKS		1

/*-----------------------------------------------------------------------
 * Environment configuration
 */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			0x01000

/*-----------------------------------------------------------------------
 * PCI configuration
 */
#define CONFIG_PCI

/*-----------------------------------------------------------------------
 * USB configuration
 */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_PCI
#define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS     8
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX

/*-----------------------------------------------------------------------
 * Network device support
 */
#define CONFIG_NET_MULTI
/* this will have to be enabled to get the bitmaps to work...
#define CONFIG_LCD	1
*/

#define CONFIG_CHROMEOS_HWID	"COREBOOT U-BOOT 0000"
#define CONFIG_OFFSET_GBB	0x000d0000
#define CONFIG_LENGTH_GBB	0x00020000
#define CONFIG_OFFSET_FMAP	0x001a0000
#define CONFIG_LENGTH_FMAP	0x00000400
#define CONFIG_LCD_vl_col	1366
#define CONFIG_LCD_vl_row	768

#endif	/* __CONFIG_H */
