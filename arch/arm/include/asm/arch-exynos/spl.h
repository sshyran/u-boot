/*
 * Copyright (c) 2012 The Chromium OS Authors.
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

#ifndef __ASM_ARCH_EXYNOS_SPL_H__
#define __ASM_ARCH_EXYNOS_SPL_H__

#include <asm/arch-exynos/dmc.h>

enum boot_mode {
	/*
	 * Assign the OM pin values for respective boot modes.
	 * Exynos4 does not support spi boot and the mmc boot OM
	 * pin values are the same across Exynos4 and Exynos5.
	 */
	BOOT_MODE_MMC = 4,
	BOOT_MODE_EMMC = 8,	/* EMMC4.4 */
	BOOT_MODE_SERIAL = 20,
	/* Boot based on Operating Mode pin settings */
	BOOT_MODE_OM = 32,
	BOOT_MODE_USB,	/* Boot using USB download */
};

#define SPL_SIGNATURE	0xdeadbeef

#ifndef __ASSEMBLY__
/* Parameters of early board initialization in SPL */
struct spl_machine_param {
	/* Add fields as and when required */
	u32		signature;	/* SPL_SIGNATURE */
	u32		version;	/* Version number */
	u32		size;		/* Size of block */
	/**
	 * Parameters we expect, in order, terminated with \0. Each parameter
	 * is a single character representing one 32-bit word in this
	 * structure.
	 *
	 * Valid characters in this string are:
	 *
	 * Code		Name
	 * v		mem_iv_size
	 * m		mem_type
	 * S		uboot_start
	 * o		uboot_offset
	 * u		uboot_size
	 * b		boot_source
	 * f		frequency_mhz (memory frequency in MHz)
	 * a		ARM clock frequency in MHz
	 * s		serial base address
	 * i		i2c base address for early access (meant for PMIC)
	 * r		board rev GPIO numbers used to read board revision
	 *			(lower halfword=tit 0, upper=tit 1)
	 * R		more board rev GPIO numbers used to read board revision
	 *			(lower halfword=tit 2, upper=tit 3)
	 * M		Memory Manufacturer name
	 * w		Bad Wake GPIO number
	 * W		Write protect firmware GPIO
	 * j		Jump to RW SPL
	 * A		RW SPL IRAM start
	 * U		RW SPL size
	 * d		Skip SDRAM init
	 * \0		termination
	 */
	char		params[20];	/* Length must be word-aligned */
	u32		mem_iv_size;	/* Memory channel interleaving size */
	enum ddr_mode	mem_type;	/* Type of on-board memory */
	u32		uboot_start;	/* U-Boot start address */
	u32		uboot_offset;	/* Offset of U-Boot in boot media */
	/*
	 * U-boot size - The iROM mmc copy function used by the SPL takes a
	 * block count paramter to describe the u-boot size unlike the spi
	 * boot copy function which just uses the u-boot size directly. Align
	 * the u-boot size to block size (512 bytes) when populating the SPL
	 * table only for mmc boot.
	 */
	u32		uboot_size;
	enum boot_mode	boot_source;	/* Boot device */
	unsigned	frequency_mhz;	/* Frequency of memory in MHz */
	unsigned	arm_freq_mhz;	/* ARM Frequency in MHz */
	u32		serial_base;	/* Serial base address */
	u32		i2c_base;	/* i2c base address */
	u32		board_rev_gpios[2];	/* Board revision GPIOs: r,R */
	enum mem_manuf	mem_manuf;	/* Memory Manufacturer */
	u32		bad_wake_gpio;	/* If high at wake time disallow wake */
	u32		write_protect_gpio;	/* Firmware write protect */

	/*
	 * 1=jump to RW SPL on resume, 0=don't.
	 * This will only be set for the RO SPL and only if
	 * early-firmware-selection is in use. In the case of a resume/wakeup,
	 * RO SPL will see this flag and know that it should not do any init
	 * itself, but should just jump to RW SPL. This is to provide a
	 * fully-updatable resume path on Exynos. The RW SPL is already ready
	 * and waiting in IRAM, we just need to jump to it.
	 */
	u32		jump_to_rw_spl;
	u32		rw_spl_start;	/* RW SPL start address */
	u32		rw_spl_size;	/* RW SPL size */
	u32		skip_sdram_init;	/* Don't init SDRAM */
} __attribute__((__packed__));
#endif

/**
 * Validate signature and return a pointer to the parameter table.  If the
 * signature is invalid, call panic() and never return.
 *
 * @return pointer to the parameter table if signature matched or never return.
 */
struct spl_machine_param *spl_get_machine_params(void);

#endif /* __ASM_ARCH_EXYNOS_SPL_H__ */
