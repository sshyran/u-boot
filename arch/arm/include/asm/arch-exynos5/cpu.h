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

#ifndef _EXYNOS5_CPU_H
#define _EXYNOS5_CPU_H

#include <asm/arch-exynos/cpu.h>

/* EXYNOS5 */
#define EXYNOS5_GPIO_PART4_BASE		0x03860000
#define EXYNOS5_PRO_ID			0x10000000
#define EXYNOS5_CLOCK_BASE		0x10010000
#define EXYNOS5_POWER_BASE		0x10040000
#define EXYNOS5_SWRESET			0x10040400
#define EXYNOS5_SYSREG_BASE		0x10050000
#define EXYNOS5_TZPC1_DECPROT1SET	0x10110810
#define EXYNOS5_WATCHDOG_BASE		0x101D0000
#define EXYNOS5_DMC_PHY0_BASE		0x10C00000
#define EXYNOS5_DMC_PHY1_BASE		0x10C10000
#define EXYNOS5_GPIO_PART3_BASE		0x10D10000
#define EXYNOS5_DMC_CTRL_BASE		0x10DD0000
#define EXYNOS5_GPIO_PART1_BASE		0x11400000
#define EXYNOS5_USB_HOST_EHCI_BASE	0x12110000
#define EXYNOS5_USBPHY_BASE		0x12130000
#define EXYNOS5_USBOTG_BASE		0x12140000
#define EXYNOS5_MMC_BASE		0x12200000
#define EXYNOS5_MSHC_BASE		0x12200000
#define EXYNOS5_SROMC_BASE		0x12250000
#define EXYNOS5_UART_BASE		0x12C00000
#define EXYNOS5_I2C_BASE		0x12C60000
#define EXYNOS5_SPI_BASE		0x12D20000
#define EXYNOS5_PWMTIMER_BASE		0x12DD0000
#define EXYNOS5_SPI_ISP_BASE		0x131A0000
#define EXYNOS5_GPIO_PART2_BASE		0x13400000
#define EXYNOS5_FIMD_BASE		0x14400000
#define EXYNOS5_DISP1_CTRL_BASE		0x14420000
#define EXYNOS5_MIPI_DSI1_BASE		0x14500000

#define EXYNOS5_ADC_BASE		DEVICE_NOT_AVAILABLE
#define EXYNOS5_MODEM_BASE		DEVICE_NOT_AVAILABLE

/* Compatibility defines */
#define EXYNOS_POWER_BASE		EXYNOS5_POWER_BASE

#ifndef __ASSEMBLY__

#define SAMSUNG_BASE(device, base)				\
static inline unsigned int samsung_get_base_##device(void)	\
{								\
	return cpu_is_exynos5() ? EXYNOS5_##base : 0;		\
}

SAMSUNG_BASE(adc, ADC_BASE)
SAMSUNG_BASE(clock, CLOCK_BASE)
SAMSUNG_BASE(dsim, MIPI_DSI1_BASE)
SAMSUNG_BASE(disp_ctrl, DISP1_CTRL_BASE)
SAMSUNG_BASE(fimd, FIMD_BASE)
SAMSUNG_BASE(gpio_part1, GPIO_PART1_BASE)
SAMSUNG_BASE(gpio_part2, GPIO_PART2_BASE)
SAMSUNG_BASE(gpio_part3, GPIO_PART3_BASE)
SAMSUNG_BASE(gpio_part4, GPIO_PART4_BASE)
SAMSUNG_BASE(i2c, I2C_BASE)
SAMSUNG_BASE(pro_id, PRO_ID)
SAMSUNG_BASE(mmc, MMC_BASE)
SAMSUNG_BASE(modem, MODEM_BASE)
SAMSUNG_BASE(mshci, MSHC_BASE)
SAMSUNG_BASE(sromc, SROMC_BASE)
SAMSUNG_BASE(swreset, SWRESET)
SAMSUNG_BASE(sysreg, SYSREG_BASE)
SAMSUNG_BASE(timer, PWMTIMER_BASE)
SAMSUNG_BASE(uart, UART_BASE)
SAMSUNG_BASE(usb_phy, USBPHY_BASE)
SAMSUNG_BASE(usb_otg, USBOTG_BASE)
SAMSUNG_BASE(watchdog, WATCHDOG_BASE)
SAMSUNG_BASE(power, POWER_BASE)
SAMSUNG_BASE(spi, SPI_BASE)
SAMSUNG_BASE(spi_isp, SPI_ISP_BASE)
#endif

#define EXYNOS5_SPI_NUM_CONTROLLERS	5

#endif	/* _EXYNOS5_CPU_H */
