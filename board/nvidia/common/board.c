/*
 *  (C) Copyright 2010,2011
 *  NVIDIA Corporation <www.nvidia.com>
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
#include <ns16550.h>
#include <linux/compiler.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#ifdef CONFIG_LCD
#include <asm/arch/display.h>
#endif
#include <asm/arch/funcmux.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/pmu.h>
#ifdef CONFIG_PWM_TEGRA
#include <asm/arch/pwm.h>
#endif
#include <asm/arch/tegra.h>
#include <asm/arch-tegra/board.h>
#include <asm/arch/clk_rst.h>
#include <asm/arch/pmc.h>
#include <asm/arch-tegra/sys_proto.h>
#include <asm/arch-tegra/uart.h>
#ifdef CONFIG_TEGRA_LP0
#include <asm/arch/warmboot.h>
#endif
#ifdef CONFIG_TEGRA_CLOCK_SCALING
#include <asm/arch/emc.h>
#endif
#ifdef CONFIG_TEGRA_MMC
#include <asm/arch-tegra/tegra_mmc.h>
#include <asm/arch-tegra/mmc.h>
#endif
#ifdef CONFIG_USB_EHCI_TEGRA
#include <asm/arch/usb.h>
#endif
#include <asm/gpio.h>
#include <i2c.h>
#include <spi.h>
#include <cros_ec.h>
#include "emc.h"
#include <malloc.h>

DECLARE_GLOBAL_DATA_PTR;

const struct tegra_sysinfo sysinfo = {
	CONFIG_TEGRA_BOARD_STRING
};

#ifndef CONFIG_SPL_BUILD
/*
 * Routine: timer_init
 * Description: init the timestamp and lastinc value
 */
int timer_init(void)
{
	return 0;
}
#endif

void __pin_mux_usb(void)
{
}

void pin_mux_usb(void) __attribute__((weak, alias("__pin_mux_usb")));

void __pin_mux_spi(void)
{
}

void pin_mux_spi(void) __attribute__((weak, alias("__pin_mux_spi")));

void __gpio_early_init_uart(void)
{
}

void gpio_early_init_uart(void)
__attribute__((weak, alias("__gpio_early_init_uart")));

/* TODO(twarren@nvidia.com): Create this only in boards that have NAND */
void __pin_mux_nand(void)
{
#if defined(CONFIG_TEGRA_NAND)
	funcmux_select(PERIPH_ID_NDFLASH, FUNCMUX_DEFAULT);
#endif
}

void pin_mux_nand(void) __attribute__((weak, alias("__pin_mux_nand")));

void __pin_mux_display(void)
{
}

void pin_mux_display(void) __attribute__((weak, alias("__pin_mux_display")));

/*
 * Routine: power_det_init
 * Description: turn off power detects
 */
static void power_det_init(void)
{
#if defined(CONFIG_TEGRA20)
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	/* turn off power detects */
	writel(0, &pmc->pmc_pwr_det_latch);
	writel(0, &pmc->pmc_pwr_det);
#endif
}

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	__maybe_unused int err;

	/* Do clocks and UART first so that printf() works */
	clock_init();
	clock_verify();

#ifdef CONFIG_FDT_SPI
	pin_mux_spi();
	spi_init();
#endif

#ifdef CONFIG_PWM_TEGRA
	if (pwm_init(gd->fdt_blob))
		debug("%s: Failed to init pwm\n", __func__);
#endif
#ifdef CONFIG_LCD
	pin_mux_display();
	tegra_lcd_check_next_stage(gd->fdt_blob, 0);
#endif
	/* boot param addr */
	gd->bd->bi_boot_params = (NV_PA_SDRAM_BASE + 0x100);

	power_det_init();
#ifdef CONFIG_TEGRA_I2C
#ifndef CONFIG_SYS_I2C_INIT_BOARD
#error "You must define CONFIG_SYS_I2C_INIT_BOARD to use i2c on Nvidia boards"
#endif
	i2c_init_board();
# ifdef CONFIG_TEGRA_PMU
	if (pmu_set_nominal())
		debug("Failed to select nominal voltages\n");
#  ifdef CONFIG_TEGRA_CLOCK_SCALING
	err = board_emc_init();
	if (err)
		debug("Memory controller init failed: %d\n", err);
#  endif
# endif /* CONFIG_TEGRA_PMU */
#endif /* CONFIG_TEGRA_I2C */

#if defined(CONFIG_TEGRA114) || defined(CONFIG_TEGRA124)
	/* Enable needed power rails. TBD: Move to kernel or driver init. */
	board_vreg_init();
#endif
#ifdef CONFIG_USB_EHCI_TEGRA
	pin_mux_usb();
	board_usb_init(gd->fdt_blob);
#endif
#ifdef CONFIG_LCD
	tegra_lcd_check_next_stage(gd->fdt_blob, 0);
#endif

#ifdef CONFIG_TEGRA_NAND
	pin_mux_nand();
#endif

#ifdef CONFIG_TEGRA_LP0
	/* save Sdram params to PMC 2, 4, and 24 for WB0 */
	warmboot_save_sdram_params();
#endif

	return 0;
}

#ifdef CONFIG_BOARD_EARLY_INIT_F
static void __gpio_early_init(void)
{
}

void gpio_early_init(void) __attribute__((weak, alias("__gpio_early_init")));

int board_early_init_f(void)
{
#if !defined(CONFIG_TEGRA20)
	pinmux_init();
#endif
	board_init_uart_f();

	/* Initialize periph GPIOs */
	gpio_early_init();
	gpio_early_init_uart();
#ifdef CONFIG_LCD
	tegra_lcd_early_init(gd->fdt_blob);
#endif

	return 0;
}
#endif	/* EARLY_INIT */

int board_late_init(void)
{
#ifdef CONFIG_LCD
	/* Make sure we finish initing the LCD */
	tegra_lcd_check_next_stage(gd->fdt_blob, 1);
#endif
	stdio_print_current_devices();

#ifdef CONFIG_TEGRA_LP0
#ifdef CONFIG_RESERVE_TEGRA_LP0
	/* TEGRA_LP0_ADDR is gd->arch.tegra_lp0_addr in this case */
	TEGRA_LP0_ADDR = (unsigned long)memalign(TEGRA_LP0_ALIGN,
						 TEGRA_LP0_SIZE);
#endif
	/* prepare the WB code to LP0 location */
	warmboot_prepare_code(TEGRA_LP0_ADDR, TEGRA_LP0_SIZE);
#endif

#ifdef CONFIG_CROS_EC
	if (cros_ec_get_error()) {
		/* Force console on */
		gd->flags &= ~GD_FLG_SILENT;

		printf("cros-ec communications failure %d\n",
		       cros_ec_get_error());
		puts("\nPlease reset with Power+Refresh\n\n");
		panic("Cannot init cros-ec device");
		return -1;
	}
#endif
	return 0;
}

#if defined(CONFIG_TEGRA_MMC)
void __pin_mux_mmc(void)
{
}

void pin_mux_mmc(void) __attribute__((weak, alias("__pin_mux_mmc")));

/* this is a weak define that we are overriding */
int board_mmc_init(bd_t *bd)
{
	debug("%s called\n", __func__);

	/* Enable muxes, etc. for SDMMC controllers */
	pin_mux_mmc();

	debug("%s: init MMC\n", __func__);
	tegra_mmc_init();

	return 0;
}

void pad_init_mmc(struct mmc_host *host)
{
#if defined(CONFIG_TEGRA30)
	enum periph_id id = host->mmc_id;
	u32 val;

	debug("%s: sdmmc address = %08x, id = %d\n", __func__,
		(unsigned int)host->reg, id);

	/* Set the pad drive strength for SDMMC1 or 3 only */
	if (id != PERIPH_ID_SDMMC1 && id != PERIPH_ID_SDMMC3) {
		debug("%s: settings are only valid for SDMMC1/SDMMC3!\n",
			__func__);
		return;
	}

	val = readl(&host->reg->sdmemcmppadctl);
	val &= 0xFFFFFFF0;
	val |= MEMCOMP_PADCTRL_VREF;
	writel(val, &host->reg->sdmemcmppadctl);

	val = readl(&host->reg->autocalcfg);
	val &= 0xFFFF0000;
	val |= AUTO_CAL_PU_OFFSET | AUTO_CAL_PD_OFFSET | AUTO_CAL_ENABLED;
	writel(val, &host->reg->autocalcfg);
#endif	/* T30 */
}
#endif	/* MMC */

#if defined(CONFIG_OF_BOARD_SETUP)
/*
 * ft_board_setup_lp0_vec() is to add "nvidia,lp0-vec" property to pmc node to
 * kernel's device tree. Also, add the TEGRA_LP0_ADDR/_SIZE area to memreserve.
 *
 * "nvidia,lp0-vec" property contains TEGRA_LP0_ADDR and TEGRA_LP0_SIZE.
 * If there are problems setting the lp0-vec property, we stil return with 0 so
 * we can continue to boot kernel.
 */
static int ft_board_setup_lp0_vec(void *blob, bd_t *bd)
{
	static const char fdt_subnode_name[] = "pmc";
	static const char prop_name[] = "nvidia,lp0-vec";
	int pmc;
	int err;

	pmc = fdt_subnode_offset(blob, 0, fdt_subnode_name);
	if (pmc < 0) {
		printf("%s: failed to get %s node\n", __func__,
		       fdt_subnode_name);
		return 0;
	}

	if (TEGRA_LP0_ADDR) {
		/* add "nvidia,lp0-vec = <%addr %length>" to pmc node */
		err = fdt_setprop_u32(blob, pmc, prop_name, TEGRA_LP0_ADDR);
		if (err < 0) {
			printf("%s: failed to setprop; err=%d\n", __func__,
			       err);
			/* error: delete the property from the tree */
			fdt_delprop(blob, pmc, prop_name);
			return 0;
		}

		err = fdt_appendprop_u32(blob, pmc, prop_name, TEGRA_LP0_SIZE);
		if (err < 0) {
			printf("%s: failed to appendprop; err=%d\n", __func__,
			       err);
			/* error: delete the property from the tree */
			fdt_delprop(blob, pmc, prop_name);
			return 0;
		}

		/* add "memreserve" to reserve TEGRA_LP0_ADDR area */
		fdt_add_mem_rsv(blob, (uint64_t)TEGRA_LP0_ADDR,
				(uint64_t)TEGRA_LP0_SIZE);
	}

	return 0;
}

#ifdef CONFIG_TEGRA_LP0
int ft_system_setup(void *blob, bd_t *bd)
{
	return ft_board_setup_lp0_vec(blob, bd);
}
#endif

__weak int ft_board_setup(void *blob, bd_t *bd)
{
	return ft_system_setup(blob, bd);
}
#endif

int arch_early_init_r(void)
{
#ifdef CONFIG_CROS_EC
	if (cros_ec_board_init()) {
		printf("%s: Failed to init EC\n", __func__);
		return 0;
	}
#endif
	return 0;
}

int board_get_revision(void)
{
	int rev = -1;

#if defined(CONFIG_TEGRA124)
	int board_id;

	gpio_request(GPIO_PQ3, "BD_ID0");
	gpio_direction_input(GPIO_PQ3);
	gpio_request(GPIO_PT1, "BD_ID1");
	gpio_direction_input(GPIO_PT1);
	gpio_request(GPIO_PX1, "BD_ID2");
	gpio_direction_input(GPIO_PX1);
	gpio_request(GPIO_PX4, "BD_ID3");
	gpio_direction_input(GPIO_PX4);

	/* Use GPIOs to read BOARD ID straps 0-3 and return an ID */
	board_id = gpio_get_value(GPIO_PQ3);
	board_id |= (gpio_get_value(GPIO_PT1) << 1);
	board_id |= (gpio_get_value(GPIO_PX1) << 2);
	board_id |= (gpio_get_value(GPIO_PX4) << 3);

	debug("%s: Board ID from GPIOs is 0x%04X\n", __func__, board_id);

	switch (board_id) {
	case VENICE2_ID:
		rev = 0;
		break;
	case NORRIN_ERS_ID:
	case NORRIN_PIX_ID:
	case NORRIN_FFD_ID:
		rev = 1;
		break;
	}

#endif
	return rev;
}
