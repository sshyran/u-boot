/*
 * Copyright (C) 2012 Samsung Electronics
 * Alim Akhtar <alim.akhtar@samsung.com>
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
#include <i2c.h>
#include "max77686.h"

/*
 * Max77686 parameters values
 * see max77686.h for parameters details
 */
struct max77686_para max77686_param[] = {/*{regnum, vol_addr, vol_bitpos,
	vol_bitmask, reg_enaddr, reg_enbitpos, reg_enbitmask, reg_enbiton,
	reg_enbitoff, vol_min, vol_div}*/
	{PMIC_BUCK1, 0x11, 0x0, 0x3F, 0x10, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_BUCK2, 0x14, 0x0, 0xFF, 0x12, 0x4, 0x3, 0x1, 0x0, 600, 12500},
	{PMIC_BUCK3, 0x1E, 0x0, 0xFF, 0x1C, 0x4, 0x3, 0x1, 0x0, 6000, 125000},
	{PMIC_BUCK4, 0x28, 0x0, 0xFF, 0x26, 0x4, 0x3, 0x1, 0x0, 600, 12500},
	{PMIC_BUCK5, 0x31, 0x0, 0x3F, 0x30, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_BUCK6, 0x33, 0x0, 0x3F, 0x32, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_BUCK7, 0x35, 0x0, 0x3F, 0x34, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_BUCK8, 0x37, 0x0, 0x3F, 0x36, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_BUCK9, 0x39, 0x0, 0x3F, 0x38, 0x0, 0x3, 0x3, 0x0, 750, 50000},
	{PMIC_LDO1,  0x40, 0x0, 0x3F, 0x40, 0x6, 0x3, 0x3, 0x0, 800, 25000},
	{PMIC_LDO2,  0x41, 0x0, 0x3F, 0x41, 0x6, 0x3, 0x1, 0x0, 800, 25000},
	{PMIC_LDO3,  0x42, 0x0, 0x3F, 0x42, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO4,  0x43, 0x0, 0x3F, 0x43, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO5,  0x44, 0x0, 0x3F, 0x44, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO6,  0x45, 0x0, 0x3F, 0x45, 0x6, 0x3, 0x1, 0x0, 800, 25000},
	{PMIC_LDO7,  0x46, 0x0, 0x3F, 0x46, 0x6, 0x3, 0x1, 0x0, 800, 25000},
	{PMIC_LDO8,  0x47, 0x0, 0x3F, 0x47, 0x6, 0x3, 0x1, 0x0, 800, 25000},
	{PMIC_LDO9,  0x48, 0x0, 0x3F, 0x48, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO10, 0x49, 0x0, 0x3F, 0x49, 0x6, 0x3, 0x1, 0x0, 800, 50000},
	{PMIC_LDO11, 0x4A, 0x0, 0x3F, 0x4A, 0x6, 0x3, 0x1, 0x0, 800, 50000},
	{PMIC_LDO12, 0x4B, 0x0, 0x3F, 0x4B, 0x6, 0x3, 0x1, 0x0, 800, 50000},
	{PMIC_LDO13, 0x4C, 0x0, 0x3F, 0x4C, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO14, 0x4D, 0x0, 0x3F, 0x4D, 0x6, 0x3, 0x1, 0x0, 800, 50000},
	{PMIC_LDO15, 0x4E, 0x0, 0x3F, 0x4E, 0x6, 0x3, 0x1, 0x0, 800, 25000},
	{PMIC_LDO16, 0x4F, 0x0, 0x3F, 0x4F, 0x6, 0x3, 0x1, 0x0, 800, 50000},
	{PMIC_LDO17, 0x50, 0x0, 0x3F, 0x50, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO18, 0x51, 0x0, 0x3F, 0x51, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO19, 0x52, 0x0, 0x3F, 0x52, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO20, 0x53, 0x0, 0x3F, 0x53, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO21, 0x54, 0x0, 0x3F, 0x54, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO22, 0x55, 0x0, 0x3F, 0x55, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO23, 0x56, 0x0, 0x3F, 0x56, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO24, 0x57, 0x0, 0x3F, 0x57, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO25, 0x58, 0x0, 0x3F, 0x58, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_LDO26, 0x59, 0x0, 0x3F, 0x59, 0x6, 0x3, 0x3, 0x0, 800, 50000},
	{PMIC_EN32KHZ_CP, 0x0, 0x0, 0x0, 0x7F, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0},
};

/*
 * Write a value to a register
 *
 * @param chip_addr	i2c addr for max77686
 * @param reg		reg number to write
 * @param val		value to be written
 *
 */
static inline int max77686_i2c_write(unsigned char chip_addr,
					unsigned int reg, unsigned char val)
{
	return i2c_write(chip_addr, reg, 1, &val, 1);
}

/*
 * Read a value from a register
 *
 * @param chip_addr	i2c addr for max77686
 * @param reg		reg number to write
 * @param val		value to be written
 *
 */
static inline int max77686_i2c_read(unsigned char chip_addr,
					unsigned int reg, unsigned char *val)
{
	return i2c_read(chip_addr, reg, 1, val, 1);
}

/*
 * Enable the max77686 register
 *
 * @param reg		register number of buck/ldo to be enabled
 * @param enable	enable or disable bit
 *
 *			REG_DISABLE = 0,
			needed to set the buck/ldo enable bit OFF
 * @return		Return 0 if ok, else -1
 */
int max77686_enablereg(enum max77686_regnum reg, int enable)
{
	struct max77686_para *pmic;
	unsigned char read_data;
	int ret;

	pmic = &max77686_param[reg];

	ret = max77686_i2c_read(MAX77686_I2C_ADDR, pmic->vol_addr, &read_data);
	if (ret != 0) {
		debug("max77686 i2c read failed.\n");
		return -1;
	}

	if (enable == REG_DISABLE) {
		clrbits_8(&read_data,
				pmic->reg_enbitmask << pmic->reg_enbitpos);
	} else {
		clrsetbits_8(&read_data,
				pmic->reg_enbitmask << pmic->reg_enbitpos,
				pmic->reg_enbiton << pmic->reg_enbitpos);
	}

	ret = max77686_i2c_write(MAX77686_I2C_ADDR,
				 pmic->reg_enaddr, read_data);
	if (ret != 0) {
		debug("max77686 i2c write failed.\n");
		return -1;
	}

	return 0;
}

/* Set the required voltage level of pmic
 *
 * @param reg		register number of buck/ldo to be set
 * @param volt		voltage level to be set
 * @param enable	enable or disable bit
 *
 * @return		Return 0 if ok, else -1
 */
int max77686_volsetting(enum max77686_regnum reg, unsigned int volt, int enable)
{
	struct max77686_para *pmic;
	unsigned char read_data;
	unsigned int vol_level = 0;
	int ret;

	pmic = &max77686_param[reg];

	if (pmic->vol_addr == 0) {
		debug("not a voltage register.\n");
		return -1;
	}

	ret = max77686_i2c_read(MAX77686_I2C_ADDR, pmic->vol_addr, &read_data);
	if (ret != 0) {
		debug("max77686 i2c read failed.\n");
		return -1;
	}

	if (volt - pmic->vol_min > 0)
		vol_level = ((volt - pmic->vol_min) * 1000) / pmic->vol_div;

	clrsetbits_8(&read_data, pmic->vol_bitmask << pmic->vol_bitpos,
			vol_level << pmic->vol_bitpos);

	ret = max77686_i2c_write(MAX77686_I2C_ADDR, pmic->vol_addr, read_data);
	if (ret != 0) {
		debug("max77686 i2c write failed.\n");
		return -1;
	}

	ret = max77686_enablereg(reg, enable);
	if (ret != 0) {
		debug("Failed to enable buck/ldo.\n");
		return -1;
	}

	return 0;
}

/**
 * Initialize the pmic voltages to power up the system
 * This also calls i2c_init so that we can program the pmic
 *
 * REG_ENABLE = 0, needed to set the buck/ldo enable bit ON
 *
 * @return	Return 0 if ok, else -1
 */
int power_init(void)
{
	int error = 0;

	/* init the i2c so that we can program pmic chip */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	error = max77686_volsetting(PMIC_BUCK2, CONFIG_VDD_ARM, REG_ENABLE);
	error |= max77686_volsetting(PMIC_BUCK3, CONFIG_VDD_INT, REG_ENABLE);
	error |= max77686_volsetting(PMIC_BUCK4, CONFIG_VDD_G3D, REG_ENABLE);
	error |= max77686_volsetting(PMIC_BUCK1, CONFIG_VDD_MIF, REG_ENABLE);
	error |= max77686_volsetting(PMIC_LDO2, CONFIG_VDD_LDO2, REG_ENABLE);
	error |= max77686_volsetting(PMIC_LDO3, CONFIG_VDD_LDO3, REG_ENABLE);
	error |= max77686_volsetting(PMIC_LDO5, CONFIG_VDD_LDO5, REG_ENABLE);
	error |= max77686_volsetting(PMIC_LDO10, CONFIG_VDD_LDO10, REG_ENABLE);
	if (error != 0)
		debug("power init failed\n");

	return error;
}
