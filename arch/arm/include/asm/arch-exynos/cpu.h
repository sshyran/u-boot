/*
 * (C) Copyright 2010 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
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

#ifndef _EXYNOS_COMMON_CPU_H
#define _EXYNOS_COMMON_CPU_H

#define DEVICE_NOT_AVAILABLE		0

#define EXYNOS_PRO_ID			0x10000000

/* Address of address of function that copys data from SD or MMC */
#define EXYNOS_COPY_MMC_FNPTR_ADDR	0x02020030

/* Address of address of function that copys data from eMMC4.4 */
#define EXYNOS_COPY_EMMC_FNPTR_ADDR	0x02020044

/* Address of address of function that copys data from SPI */
#define EXYNOS_COPY_SPI_FNPTR_ADDR	0x02020058

/* Address of address of function that copys data through USB */
#define EXYNOS_COPY_USB_FNPTR_ADDR	0x02020070

/* Boot mode values */
#define EXYNOS_USB_SECONDARY_BOOT	0xfeed0002

#define EXYNOS_IRAM_SECONDARY_BASE	0x02020018

#ifndef __ASSEMBLY__
#include <asm/io.h>

enum boot_mode {
	/*
	 * Assign the OM pin values for respective boot modes.
	 * Exynos4 does not support spi boot and the mmc boot OM
	 * pin values are the same across Exynos4 and Exynos5.
	 */
	BOOT_MODE_MMC = 4,
	BOOT_MODE_EMMC = 8,
	BOOT_MODE_SERIAL = 20,
	/* Boot based on Operating Mode pin settings */
	BOOT_MODE_OM = 32,
};

/**
 * Get the U-boot size for SPL copy functions
 *
 * @return size of U-Boot code/data that needs to be loaded by the SPL stage
 */
unsigned int exynos_get_uboot_size(void);

/**
 * Get the boot device containing BL1, BL2 (SPL) and U-boot
 *
 * @return boot device
 */
enum boot_mode exynos_get_boot_device(void);

/* CPU detection macros */
extern unsigned int s5p_cpu_id;
extern unsigned int s5p_cpu_rev;

static inline int s5p_get_cpu_rev(void)
{
	return s5p_cpu_rev;
}

static inline void s5p_set_cpu_id(void)
{
	s5p_cpu_id = readl(EXYNOS_PRO_ID);
	s5p_cpu_id = (0xC000 | ((s5p_cpu_id & 0x00FFF000) >> 12));

	/*
	 * 0xC200: EXYNOS4210 EVT0
	 * 0xC210: EXYNOS4210 EVT1
	 */
	if (s5p_cpu_id == 0xC200) {
		s5p_cpu_id |= 0x10;
		s5p_cpu_rev = 0;
	} else if (s5p_cpu_id == 0xC210) {
		s5p_cpu_rev = 1;
	}
}

#define IS_SAMSUNG_TYPE(type, id)			\
static inline int cpu_is_##type(void)			\
{							\
	return s5p_cpu_id == id ? 1 : 0;		\
}

IS_SAMSUNG_TYPE(exynos4, 0xc210)
IS_SAMSUNG_TYPE(exynos5, 0xc520)

#endif

#endif	/* _EXYNOS_COMMON_CPU_H */
