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

#include <common.h>
#include <asm-generic/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/gp_padctrl.h>
#include <asm/arch/pinmux.h>
#include "pinmux-config-venice2.h"
#include <i2c.h>

/* TODO(twarren@nvidia.com): Move to device tree */
#define PMU_I2C_ADDRESS		0x40		/* AS3722 PMU */

/*
 * NOTE: On Venice2, the AS3728 PMIC is controlled by the AS3722, and
 * isn't directly controlled from U-Boot.
 */

/*
 * Routine: pinmux_init
 * Description: Do individual peripheral pinmux configs
 */
void pinmux_init(void)
{
	pinmux_config_table(tegra124_pinmux_set_nontristate,
		ARRAY_SIZE(tegra124_pinmux_set_nontristate));

	pinmux_config_table(tegra124_pinmux_common,
		ARRAY_SIZE(tegra124_pinmux_common));

	pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	/* Initialize any non-default pad configs (APB_MISC_GP regs) */
	padgrp_config_table(venice2_padctrl, ARRAY_SIZE(venice2_padctrl));
}

/* TODO(twarren@nvidia.com): Move to pmic infrastructure (pmic_common_init) */

/* Writes val to reg @ chip address pmu */
void i2c_write_pmic(uchar pmu, uchar reg, uchar val)
{
	uchar data_buffer[1];
	int ret;

	data_buffer[0] = val;

	ret = i2c_write(pmu, reg, 1, data_buffer, 1);
	if (ret) {
		printf("%s: PMU i2c_write %02X<-%02X returned %d\n",
			__func__, reg, data_buffer[0], ret);
	}
}

/* Reads reg @ chip address pmu */
void i2c_read_pmic(uchar pmu, uchar reg, uchar *val)
{
	int ret;

	ret = i2c_read(pmu, reg, 1, val, 1);
	if (ret) {
		printf("%s: PMU i2c_read %02X returned %d\n",
			__func__, reg, ret);
	}
}

#if defined(CONFIG_TEGRA_MMC)
/*
 * Do I2C/PMU writes to bring up SD card bus power
 *
 */
void board_sdmmc_voltage_init(void)
{
#if defined(VENICE2_LATE_PMIC_INIT)
	/*
	 * TODO(twarren@nvidia.com):
	 *  Find out why writing the SDMMC LDO this late hangs the CPU
	 *  Maybe it's the re-write of the enable bit? (already set by OTP)
	 */
	uchar val;

	int ret = i2c_set_bus_num(0);	/* PMU is on bus 0 */
	if (ret) {
		printf("%s: Unable to select PMIC bus [%d]!\n", __func__, ret);
		return;
	}

	/* AS3722: LDO6VOLTAGE (reg 0x16) = 3.3V (0.8v+vsel*25mV)*/
	/* AS3722: LDO6CTRL (bit 6, reg 0x4E) = Active */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x16, 0x64);
	i2c_read_pmic(PMU_I2C_ADDRESS, 0x4E, &val);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x4E, (val | (1 << 6)));
#endif
}

/*
 * Routine: pin_mux_mmc
 * Description: setup the MMC muxes, power rails, etc.
 */
void pin_mux_mmc(void)
{
	/*
	 * NOTE: We don't do mmc-specific pin muxes here.
	 * They were done globally in pinmux_init().
	 */

	/* Bring up the SDIO3 power rail */
	board_sdmmc_voltage_init();
}
#endif /* MMC */

void board_vreg_init(void)
{
	int ret = i2c_set_bus_num(0);	/* PMU is on bus 0 */

	if (ret) {
		printf("%s: Unable to select PMIC bus [%d]!\n", __func__, ret);
		return;
	}

#if defined(VENICE2_LATE_PMIC_INIT)
	/*
	 * Set and enable 1V2_GEN_VDD for VDDIO_HSIC, AVDD_DSI_CSI
	 *   LDO2VOLTAGE (reg 0x12) = 1.2v (0.8v+vsel*25mV)
	 *   LDO2CTRL (bit 2, reg 0x4E) = Active
	 */

	/*
	 * TODO(twarren@nvidia.com):
	 *  Find out why writing the LDO this late hangs the CPU
	 *  Maybe it's the re-write of the enable bit? (already set by OTP)
	 */
	uchar val;

	i2c_write_pmic(PMU_I2C_ADDRESS, 0x12, 0x10);
	i2c_read_pmic(PMU_I2C_ADDRESS, 0x4E, &val);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x4E, (val | (1 << 2)));
#endif
	/* Enable LCD backlight, s/b GPIO_PH2, LCD_BL_EN */
	gpio_request(DSI_PANEL_BL_EN_GPIO, "lcd_bl_en");
	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);

	/* Enable touchscreen */
	gpio_request(TS_SHDN_L_GPIO, "ts_shdn_l");
	gpio_direction_output(TS_SHDN_L_GPIO, 1);
}
