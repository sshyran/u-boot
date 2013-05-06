/*
 * Copyright (c) 2013 The Chromium OS Authors.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <fdtdec.h>
#include <i2c.h>
#include <rtc.h>
#include <asm/arch-exynos/spl.h>

DECLARE_GLOBAL_DATA_PTR;

enum {
	MAX77802_RTCINT = 0xc0,
	MAX77802_RTCINTM = 0xc1,
	MAX77802_RTCCNTLM = 0xc2,
	MAX77802_RTCCNTL = 0xc3,
	MAX77802_RTCUPDATE0 = 0xc4,
	/* Both of these values are in the same register. */
	MAX77802_RTCSMPL = 0xc6,
	MAX77802_RTCWTSR = 0xc6,

	MAX77802_RTCSEC = 0xc7,
	MAX77802_RTCMIN = 0xc8,
	MAX77802_RTCHOUR = 0xc9,
	MAX77802_RTCDAY = 0xca,
	MAX77802_RTCMONTH = 0xcb,
	MAX77802_RTCYEAR = 0xcc,
	MAX77802_RTCDATE = 0xcd,

	MAX77802_RTCAE1 = 0xce,

	MAX77802_RTCSECA1 = 0xcf,
	MAX77802_RTCMINA1 = 0xd0,
	MAX77802_RTCHOURA1 = 0xd1,
	MAX77802_RTCDAYA1 = 0xd2,
	MAX77802_RTCMONTHA1 = 0xd3,
	MAX77802_RTCYEARA1 = 0xd4,
	MAX77802_RTCDATEA1 = 0xd5,

	MAX77802_RTCAE2 = 0xd6,

	MAX77802_RTCSECA2 = 0xd7,
	MAX77802_RTCMINA2 = 0xd8,
	MAX77802_RTCHOURA2 = 0xd9,
	MAX77802_RTCDAYA2 = 0xda,
	MAX77802_RTCMONTHA2 = 0xdb,
	MAX77802_RTCYEARA2 = 0xdc,
	MAX77802_RTCDATEA2 = 0xdd
};

enum {
	MAX77802_RTCCNTLM_BCDM = 1 << 0,
	MAX77802_RTCCNTLM_HRMODEM = 1 << 1
};

enum {
	MAX77802_RTCCNTL_BCD = 1 << 0,
	MAX77802_RTCCNTL_HRMODE = 1 << 1
};

enum {
	MAX77802_RTCUPDATE0_UDR = 1 << 0,
	MAX77802_RTCUPDATE0_FREEZE_SEC = 1 << 2,
	MAX77802_RTCUPDATE0_RTCWAKE = 1 << 3,
	MAX77802_RTCUPDATE0_RBUDR = 1 << 4
};

enum {
	MAX77802_RTCHOUR_AMPM = 1 << 6
};

enum {
	DATA_SECOND,
	DATA_MINUTE,
	DATA_HOUR,
	DATA_WEEKDAY,
	DATA_MONTH,
	DATA_YEAR,
	DATA_DATE,

	DATA_SIZE
};

static int init_done __attribute__((section(".data")));
static int i2c_bus __attribute__((section(".data")));
static unsigned int i2c_addr __attribute__((section(".data")));

static int max77802_rtc_write(unsigned int reg, uint8_t val)
{
	return i2c_write(i2c_addr, reg, 1, &val, 1);
}

static int max77802_rtc_read(unsigned int reg, uint8_t *val)
{
	return i2c_read(i2c_addr, reg, 1, val, 1);
}

static int rtc_init(void)
{
#ifdef CONFIG_SPL_BUILD
	struct spl_machine_param *params = spl_get_machine_params();
#else
	int node, parent;
#endif

	if (init_done)
		return 0;

#ifdef CONFIG_SPL_BUILD
	if (params->rtc_type != SPL_RTC_TYPE_MAX77802)
		/* Not the right type of RTC. */
		return -1;
	i2c_bus = CONFIG_SPL_MAX77802_BUS;
	i2c_addr = CONFIG_SPL_MAX77802_ADDR;
#else
	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0,
					     "maxim,max77802-pmic");
	if (node < 0) {
		printf("PMIC: Error %s. No node for %s in device tree\n",
		       fdt_strerror(node), "maxim,max77802-pmic");
		return node;
	}
	parent = fdt_parent_offset(gd->fdt_blob, node);
	if (parent < 0) {
		printf("%s: Cannot find node parent\n", __func__);
		return -1;
	}

	i2c_bus = i2c_get_bus_num_fdt(parent);
	if (i2c_bus < 0) {
		printf("%s: Cannot find I2C bus\n", __func__);
		return -1;
	}
	i2c_addr = fdtdec_get_int(gd->fdt_blob, node, "reg", 9);
#endif

	i2c_set_bus_num(i2c_bus);

	if (max77802_rtc_write(MAX77802_RTCCNTLM,
			       MAX77802_RTCCNTLM_BCDM |
			       MAX77802_RTCCNTLM_HRMODEM)) {
#ifndef CONFIG_SPL_BUILD
		printf("%s: Failed to set rtccntlm.\n", __func__);
#endif
		return -1;
	}

	if (max77802_rtc_write(MAX77802_RTCCNTL,
			       MAX77802_RTCCNTL_HRMODE)) {
#ifndef CONFIG_SPL_BUILD
		printf("%s: Failed to set rtccntl.\n", __func__);
#endif
		return -1;
	}

	init_done = 1;

	return 0;
}

int rtc_get(struct rtc_time *tm)
{
	int old_bus = i2c_get_bus_num();
	uint8_t data[DATA_SIZE];
	uint8_t update0;

	i2c_set_bus_num(i2c_bus);
	if (rtc_init()) {
		i2c_set_bus_num(old_bus);
		return -1;
	}

	if (max77802_rtc_read(MAX77802_RTCUPDATE0, &update0) ||
	    max77802_rtc_write(MAX77802_RTCUPDATE0,
				   update0 | MAX77802_RTCUPDATE0_RBUDR)) {
		printf("%s: Failed to access rtcupdate0.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	do {
		if (max77802_rtc_read(MAX77802_RTCUPDATE0, &update0)) {
			printf("%s: Failed to access rtcupdate0.\n", __func__);
			i2c_set_bus_num(old_bus);
			return -1;
		}
	} while (update0 & MAX77802_RTCUPDATE0_RBUDR);

	if (i2c_read(i2c_addr, MAX77802_RTCSEC, 1, data, DATA_SIZE)) {
		printf("%s: Failed to read from the RTC.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	tm->tm_sec = data[DATA_SECOND];
	tm->tm_min = data[DATA_MINUTE];
	tm->tm_hour = data[DATA_HOUR] & 0x3f;
	tm->tm_wday = __builtin_ctz(data[DATA_WEEKDAY]);
	tm->tm_mday = data[DATA_DATE];
	tm->tm_mon = data[DATA_MONTH];
	tm->tm_year = data[DATA_YEAR];

	if (tm->tm_year < 70)
		tm->tm_year += 2000;
	else
		tm->tm_year += 1900;

	i2c_set_bus_num(old_bus);
	return 0;
}

int rtc_set(struct rtc_time *tm)
{
	int old_bus = i2c_get_bus_num();
	uint8_t data[DATA_SIZE];
	uint8_t update0;

	i2c_set_bus_num(i2c_bus);
	if (rtc_init()) {
		printf("%s: Failed to initialize the RTC.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	memset(data, 0, sizeof(data));
	data[DATA_SECOND] = tm->tm_sec;
	data[DATA_MINUTE] = tm->tm_min;
	data[DATA_HOUR] = tm->tm_hour;
	data[DATA_WEEKDAY] = 0x1 << tm->tm_wday;
	data[DATA_DATE] = tm->tm_mday;
	data[DATA_MONTH] = tm->tm_mon;
	data[DATA_YEAR] = tm->tm_year % 100;

	if (tm->tm_hour > 12)
		data[DATA_HOUR] |= MAX77802_RTCHOUR_AMPM;

	if (i2c_write(i2c_addr, MAX77802_RTCSEC, 1, data, DATA_SIZE)) {
		printf("%s: Failed to set data registers.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	if (max77802_rtc_read(MAX77802_RTCUPDATE0, &update0) ||
	    max77802_rtc_write(MAX77802_RTCUPDATE0,
				   update0 | MAX77802_RTCUPDATE0_UDR)) {
		printf("%s: Failed to access rtcupdate0.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	do {
		if (max77802_rtc_read(MAX77802_RTCUPDATE0, &update0)) {
			printf("%s: Failed to access rtcupdate0.\n", __func__);
			i2c_set_bus_num(old_bus);
			return -1;
		}
	} while (update0 & MAX77802_RTCUPDATE0_UDR);

	i2c_set_bus_num(old_bus);
	return 0;
}

void rtc_reset(void)
{
	struct rtc_time tm;

	init_done = 0;

	memset(&tm, 0, sizeof(tm));
	rtc_set(&tm);
}
