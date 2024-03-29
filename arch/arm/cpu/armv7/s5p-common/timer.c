/*
 * Copyright (C) 2009 Samsung Electronics
 * Heungjun Kim <riverful.kim@samsung.com>
 * Inki Dae <inki.dae@samsung.com>
 * Minkyu Kang <mk7.kang@samsung.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/pwm.h>
#include <asm/arch/clk.h>
#include <pwm.h>

DECLARE_GLOBAL_DATA_PTR;

/* macro to read the 16 bit timer */
static inline struct s5p_timer *s5p_get_base_timer(void)
{
	return (struct s5p_timer *)samsung_get_base_timer();
}

/**
 * Read the countdown timer.
 *
 * This operates at 1MHz and counts downwards. It will wrap about every
 * hour (2^32 microseconds).
 *
 * @return current value of timer
 */
static unsigned long timer_get_us_down(void)
{
	struct s5p_timer *const timer = s5p_get_base_timer();

	return readl(&timer->tcnto4);
}

int timer_init(void)
{
	/* PWM Timer 4 */
	pwm_init(4, MUX_DIV_4, 0);
	pwm_config(4, 0, 0);
	pwm_enable(4);

	/* Use this as the current monotonic time in us */
	gd->timer_reset_value = 0;

	/* Use this as the last timer value we saw */
	gd->lastinc = timer_get_us_down();
	return 0;
}

/*
 * timer without interrupts
 */
unsigned long get_timer(unsigned long base)
{
	ulong now = timer_get_us_down();

	/*
	 * Increment the time by the amount elapsed since the last read.
	 * The timer may have wrapped around, but it makes no difference to
	 * our arithmetic here.
	 */
	gd->timer_reset_value += gd->lastinc - now;
	gd->lastinc = now;

	/* Divide by 1000 to convert from us to ms */
	return gd->timer_reset_value / 1000 - base;
}

unsigned long timer_get_us(void)
{
	static unsigned long base_time_us;

	struct s5p_timer *const timer =
		(struct s5p_timer *)samsung_get_base_timer();
	unsigned long now_downward_us = readl(&timer->tcnto4);

	if (!base_time_us)
		base_time_us = now_downward_us;

	/* Note that this timer counts downward. */
	return base_time_us - now_downward_us;
}

/* delay x useconds */
void __udelay(unsigned long usec)
{
	unsigned long count_value;

	count_value = timer_get_us_down();
	while ((int)(count_value - timer_get_us_down()) < (int)usec)
		;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
unsigned long get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}
