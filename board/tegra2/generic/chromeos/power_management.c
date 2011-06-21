/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

/* Implementation of per-board power management function */

#include <common.h>
#include <i2c.h>
#include <asm/arch/gpio.h>
#include "../lcd/gpinit/gpinit.h"

#include <chromeos/common.h>
#include <chromeos/power_management.h>

#define PREFIX "power_management: "

void cros_reboot(void)
{
	/*
	 * Pulling this gpio down should reboot the main processor and TPM
	 * chip together (on certain boards).
	 */
	tg2_gpio_direction_output(TEGRA_GPIO_PORT(TEGRA_GPIO_PG3),
			TEGRA_GPIO_BIT(TEGRA_GPIO_PG3), 1);
	tg2_gpio_set_value(TEGRA_GPIO_PORT(TEGRA_GPIO_PG3),
			TEGRA_GPIO_BIT(TEGRA_GPIO_PG3), 0);

	VBDEBUG(PREFIX "fall back to cold reboot\n");
	cold_reboot();
}

#define PMIC_I2C_BUS		0x00
#define PMIC_I2C_DEVICE_ADDRESS	0x34
#define TPS6586X_SUPPLYENE	0x14

/* This function never returns */
void cold_reboot(void)
{
	uint8_t byte;

	if (i2c_set_bus_num(PMIC_I2C_BUS)) {
		VBDEBUG(PREFIX "i2c_set_bus_num fail\n");
		goto FATAL;
	}

	if (i2c_read(PMIC_I2C_DEVICE_ADDRESS, TPS6586X_SUPPLYENE, 1,
				&byte, sizeof(byte))) {
		VBDEBUG(PREFIX "i2c_read fail\n");
		goto FATAL;
	}

	/* Set TPS6586X_SUPPLYENE bit0 to 1 */
	byte |= 1;

	if (i2c_write(PMIC_I2C_DEVICE_ADDRESS, TPS6586X_SUPPLYENE, 1,
				&byte, sizeof(byte))) {
		VBDEBUG(PREFIX "i2c_write fail\n");
		goto FATAL;
	}

	/* The PMIC will reboot the whole system after 10 ms */
	udelay(100);

FATAL:
	/* The final solution of doing a cold reboot */
	printf("Please press cold reboot button\n");
	while (1);
}
