/*
 * Copyright (C) 2011 Samsung Electronics
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

#include <common.h>
#include <config.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk.h>

#define OM_STAT		(0x1f << 1)

/**
 * Copy data from SD or MMC device to RAM.
 *
 * @param offset	Block offset of the data
 * @param nblock	Number of blocks
 * @param dst		Destination address
 * @return 1 = True or 0 = False
 */
typedef u32 (*mmc_copy_func_t)(u32 offset, u32 nblock, u32 dst);

/**
 * Copy data from eMMC device to RAM.
 *
 * @param nblock	Number of blocks
 * @param dst		Destination address
 * @return 1 = True or 0 = False
 */
typedef u32 (*emmc_copy_func_t)(u32 offset, u32 dst);

/**
 * Copy data from SPI flash to RAM.
 *
 * @param offset	Block offset of the data
 * @param nblock	Number of blocks
 * @param dst		Destination address
 * @return 1 = True or 0 = False
 */
typedef u32 (*spi_copy_func_t)(u32 offset, u32 nblock, u32 dst);


/**
 * Copy data through USB.
 *
 * @return 1 = True or 0 = False
 */
typedef u32 (*usb_copy_func_t)(void);

/**
 * Change the mshc controller clock divider
 * to clock mshc at 40Mhz.
 * This is needed to support emmc boot.
 */
static int emmc_divider_change()
{
	struct exynos5_clock *clk =
		(struct exynos5_clock *)samsung_get_base_clock();
	unsigned int addr;
	unsigned int div_mmc;

	addr = (unsigned int)&clk->div_fsys3;
	div_mmc = readl(addr) & ~0xff0f;
	div_mmc |= (1 << 8) | 9;
	writel(div_mmc, addr);
}

/*
 * Set/clear program flow prediction and return the previous state.
 */
static int config_branch_prediction(int set_cr_z)
{
	unsigned int cr;

	/* System Control Register: 11th bit Z Branch prediction enable */
	cr = get_cr();
	set_cr(set_cr_z ? cr | CR_Z : cr & ~CR_Z);

	return cr & CR_Z;
}

/* Copy U-Boot image to RAM */
static void copy_uboot_to_ram(void)
{
	unsigned int sec_boot_check;
	unsigned int uboot_size;
	int is_cr_z_set;
	enum boot_mode boot_source;
	mmc_copy_func_t mmc_copy;
	emmc_copy_func_t emmc_copy;

#if defined(CONFIG_EXYNOS_SPI_BOOT)
	spi_copy_func_t spi_copy;
#endif
	usb_copy_func_t usb_copy;

	uboot_size = exynos_get_uboot_size();
	boot_source = exynos_get_boot_device();

	/* Read iRAM location to check for secondary USB boot mode */
	sec_boot_check = readl(EXYNOS_IRAM_SECONDARY_BASE);
	if (sec_boot_check == EXYNOS_USB_SECONDARY_BOOT) {
		/*
		 * iROM needs program flow prediction to be disabled
		 * before copy from USB device to RAM
		 */
		is_cr_z_set = config_branch_prediction(0);
		usb_copy = *(usb_copy_func_t *)EXYNOS_COPY_USB_FNPTR_ADDR;
		usb_copy();
		config_branch_prediction(is_cr_z_set);
		return;
	}

	if (boot_source == BOOT_MODE_OM)
		boot_source = readl(EXYNOS_POWER_BASE) & OM_STAT;
	switch (boot_source) {
#if defined(CONFIG_EXYNOS_SPI_BOOT)
	case BOOT_MODE_SERIAL:
		spi_copy = *(spi_copy_func_t *)EXYNOS_COPY_SPI_FNPTR_ADDR;
		spi_copy(SPI_FLASH_UBOOT_POS, uboot_size,
				CONFIG_SYS_TEXT_BASE);
		break;
#endif
	case BOOT_MODE_MMC:
		mmc_copy = *(mmc_copy_func_t *)EXYNOS_COPY_MMC_FNPTR_ADDR;
		assert(!(uboot_size & 511));
		mmc_copy(BL2_START_OFFSET, uboot_size / 512,
				CONFIG_SYS_TEXT_BASE);
		break;
	case BOOT_MODE_EMMC:
		emmc_divider_change();
		emmc_copy = *(emmc_copy_func_t *)EXYNOS_COPY_EMMC_FNPTR_ADDR;
		emmc_copy(CONFIG_EMMC_UBOOT_BLKCNT, CONFIG_SYS_TEXT_BASE);
		break;
	default:
		/* TODO: Call panic() here */
		debug("Invalid boot mode selection\n");
		break;
	}
}

void board_init_f(unsigned long bootflag)
{
	__attribute__((noreturn)) void (*uboot)(void);

	copy_uboot_to_ram();

	/* Jump to U-Boot image */
	uboot = (void *)CONFIG_SYS_TEXT_BASE;
	uboot();
	/* Never returns Here */
}

/* Place Holders */
void board_init_r(gd_t *id, ulong dest_addr)
{
	/* Function attribute is no-return */
	/* This Function never executes */
	while (1)
		;
}

void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3) {}
