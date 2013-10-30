/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Portions Copyright (C) 2012 Samsung Electronics
 * R. Chadrasekar <rcsekar@samsung.com>
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

#ifndef __RT5640_H__
#define __RT5640_H__

/* Sources for SYSCLK - use with set_dai_sysclk() */
#define RT5640_SYSCLK_MCLK	0
#define RT5640_SYSCLK_PLL	1

/*  Available audio interface ports in RT5640 codec */
enum en_audio_interface {
	 RT5640_AIF1 = 1,
	 RT5640_AIF2,
};

#define RT5640_ID			0x000C	/* s/b 0004 in spec !? */

/* rt5640 family devices */
enum rt5640_type {
	RT5640 = 0,
};

/*
 * initialise RT5640 sound codec device for the given configuration
 *
 * @param blob			FDT node for codec values
 * @param aif_id		enum value of codec interface port in which
 *				soc i2s is connected
 * @param sampling_rate		Sampling rate ranges between from 8khz to 96khz
 * @param mclk_freq		Master clock frequency.
 * @param bits_per_sample	bits per Sample can be 16 or 24
 * @param channels		Number of channnels, maximum 2
 *
 * @returns -1 for error and 0 for success.
 */
int rt5640_init(const void *blob, enum en_audio_interface aif_id,
			int sampling_rate, int mclk_freq,
			int bits_per_sample, unsigned int channels);
#endif /*__RT5640_H__ */
