/*
 * (C) Copyright 2009 Samsung Electronics
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
 */

#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>

#define CON_MASK(x)		(0xf << ((x) << 2))
#define CON_SFR(x, v)		((v) << ((x) << 2))

#define DAT_MASK(x)		(0x1 << (x))
#define DAT_SET(x)		(0x1 << (x))

#define PULL_MASK(x)		(0x3 << ((x) << 1))
#define PULL_MODE(x, v)		((v) << ((x) << 1))

#define DRV_MASK(x)		(0x3 << ((x) << 1))
#define DRV_SET(x, m)		((m) << ((x) << 1))
#define RATE_MASK(x)		(0x1 << (x + 16))
#define RATE_SET(x)		(0x1 << (x + 16))

static struct s5p_gpio_bank *gpio_get_bank(unsigned gpio)
{
	int bank_offset;

	if (gpio < GPIO_MAX_PORT_PART_1) {
		if (gpio > GPIO_Y67)
			gpio = gpio + (0x980 * GPIO_PER_BANK);
		bank_offset = gpio / GPIO_PER_BANK;
		return EXYNOS5_GPIO_PART1_BASE + (bank_offset *
						sizeof(struct s5p_gpio_bank));
	} else if (gpio < GPIO_MAX_PORT_PART_1_2) {
		bank_offset = (gpio - GPIO_MAX_PORT_PART_1) / GPIO_PER_BANK;
		return EXYNOS5_GPIO_PART2_BASE + (bank_offset *
						sizeof(struct s5p_gpio_bank));
	} else if (gpio < GPIO_MAX_PORT_PART_1_2_3) {
		if (gpio > GPIO_V37)
			gpio = gpio + (0x20 * GPIO_PER_BANK);
		bank_offset = (gpio - GPIO_MAX_PORT_PART_1_2) / GPIO_PER_BANK;
		return EXYNOS5_GPIO_PART3_BASE + (bank_offset *
						sizeof(struct s5p_gpio_bank));
	}

	return NULL;
}

void s5p_gpio_cfg_pin(struct s5p_gpio_bank *bank, int gpio, int cfg)
{
	unsigned int value;

	value = readl(&bank->con);
	value &= ~CON_MASK(gpio);
	value |= CON_SFR(gpio, cfg);
	writel(value, &bank->con);
}

void s5p_gpio_direction_output(struct s5p_gpio_bank *bank, int gpio, int en)
{
	unsigned int value;

	s5p_gpio_cfg_pin(bank, gpio, GPIO_OUTPUT);

	value = readl(&bank->dat);
	value &= ~DAT_MASK(gpio);
	if (en)
		value |= DAT_SET(gpio);
	writel(value, &bank->dat);
}

void s5p_gpio_direction_input(struct s5p_gpio_bank *bank, int gpio)
{
	s5p_gpio_cfg_pin(bank, gpio, GPIO_INPUT);
}

void s5p_gpio_set_value(struct s5p_gpio_bank *bank, int gpio, int en)
{
	unsigned int value;

	value = readl(&bank->dat);
	value &= ~DAT_MASK(gpio);
	if (en)
		value |= DAT_SET(gpio);
	writel(value, &bank->dat);
}

unsigned int s5p_gpio_get_value(struct s5p_gpio_bank *bank, int gpio)
{
	unsigned int value;

	value = readl(&bank->dat);
	return !!(value & DAT_MASK(gpio));
}

void s5p_gpio_set_pull(struct s5p_gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = readl(&bank->pull);
	value &= ~PULL_MASK(gpio);

	switch (mode) {
	case GPIO_PULL_DOWN:
	case GPIO_PULL_UP:
		value |= PULL_MODE(gpio, mode);
		break;
	default:
		break;
	}

	writel(value, &bank->pull);
}

void s5p_gpio_set_drv(struct s5p_gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = readl(&bank->drv);
	value &= ~DRV_MASK(gpio);

	switch (mode) {
	case GPIO_DRV_1X:
	case GPIO_DRV_2X:
	case GPIO_DRV_3X:
	case GPIO_DRV_4X:
		value |= DRV_SET(gpio, mode);
		break;
	default:
		return;
	}

	writel(value, &bank->drv);
}

void s5p_gpio_set_rate(struct s5p_gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = readl(&bank->drv);
	value &= ~RATE_MASK(gpio);

	switch (mode) {
	case GPIO_DRV_FAST:
	case GPIO_DRV_SLOW:
		value |= RATE_SET(gpio);
		break;
	default:
		return;
	}

	writel(value, &bank->drv);
}

struct s5p_gpio_bank *s5p_gpio_get_bank(unsigned gpio)
{
	int bank = gpio / GPIO_PER_BANK;
	bank *= sizeof(struct s5p_gpio_bank);

	return (struct s5p_gpio_bank *) (s5p_gpio_base(gpio) + bank);
}

int s5p_gpio_get_pin(unsigned gpio)
{
	return gpio % GPIO_PER_BANK;
}

/* Common GPIO API */

void gpio_cfg_pin(int gpio, int cfg)
{
	unsigned int value;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	value = readl(&bank->con);
	value &= ~CON_MASK(GPIO_BIT(gpio));
	value |= CON_SFR(GPIO_BIT(gpio), cfg);
	writel(value, &bank->con);
}

void gpio_set_pull(int gpio, int mode)
{
	unsigned int value;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	value = readl(&bank->pull);
	value &= ~PULL_MASK(GPIO_BIT(gpio));

	switch (mode) {
	case GPIO_PULL_DOWN:
	case GPIO_PULL_UP:
		value |= PULL_MODE(GPIO_BIT(gpio), mode);
		break;
	default:
		break;
	}

	writel(value, &bank->pull);
}

void gpio_set_drv(int gpio, int mode)
{
	unsigned int value;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	value = readl(&bank->drv);
	value &= ~DRV_MASK(GPIO_BIT(gpio));

	switch (mode) {
	case GPIO_DRV_1X:
	case GPIO_DRV_2X:
	case GPIO_DRV_3X:
	case GPIO_DRV_4X:
		value |= DRV_SET(GPIO_BIT(gpio), mode);
		break;
	default:
		return;
	}

	writel(value, &bank->drv);
}

void gpio_set_rate(int gpio, int mode)
{
	unsigned int value;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	value = readl(&bank->drv);
	value &= ~RATE_MASK(GPIO_BIT(gpio));

	switch (mode) {
	case GPIO_DRV_FAST:
	case GPIO_DRV_SLOW:
		value |= RATE_SET(GPIO_BIT(gpio));
		break;
	default:
		return;
	}

	writel(value, &bank->drv);
}

int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

int gpio_free(unsigned gpio)
{
	return 0;
}

int gpio_direction_input(unsigned gpio)
{
	gpio_cfg_pin(gpio, GPIO_INPUT);

	return 0;
}

int gpio_direction_output(unsigned gpio, int value)
{
	unsigned int val;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	gpio_cfg_pin(gpio, GPIO_OUTPUT);

	val = readl(&bank->dat);
	val &= ~DAT_MASK(GPIO_BIT(gpio));
	if (value)
		val |= DAT_SET(GPIO_BIT(gpio));
	writel(val, &bank->dat);

	return 0;
}

int gpio_get_value(unsigned gpio)
{
	unsigned int value;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	value = readl(&bank->dat);
	return !!(value & DAT_MASK(GPIO_BIT(gpio)));
}

int gpio_set_value(unsigned gpio, int value)
{
	unsigned int val;
	struct s5p_gpio_bank *bank = gpio_get_bank(gpio);

	val = readl(&bank->dat);
	val &= ~DAT_MASK(GPIO_BIT(gpio));
	if (value)
		val |= DAT_SET(GPIO_BIT(gpio));
	writel(val, &bank->dat);

	return 0;
}
