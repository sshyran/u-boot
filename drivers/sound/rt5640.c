/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Portions Copyright (C) 2012 Samsung Electronics
 * R. Chandrasekar <rcsekar@samsung.com>
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

/* Tom Warren - adapted from the WM8994.c codec source in drivers/sound */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clk_rst.h>
#include <asm/gpio.h>
#include <div64.h>
#include <fdtdec.h>
#include <i2c.h>
#include <asm/arch-tegra/tegra_i2c.h>
#include <i2s.h>
#include <sound.h>
#include <asm/arch/sound.h>
#include "rt5640.h"
#include "rt5640_regs.h"

/* codec private data */
struct rt5640_priv {
	enum rt5640_type type;		/* codec type of RealTek */
	int revision;			/* Revision */
	int sysclk;			/* System clock frequency in Hz  */
	int mclk;			/* master clock frequency in Hz */
	int sysclk_src;			/* source of system click (MCLK, PLL) */
};

static struct rt5640_priv g_rt5640_info;
static unsigned char g_rt5640_i2c_dev_addr;
static struct sound_codec_info g_codec_info;

/* Codec registers that are in the spec (1 = exists) */
static int __maybe_unused reg_map[256] = {
	/* 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F */
	    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,	/* 00-0F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0,	/* 10-1F */
	    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1,	/* 20-2F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0,	/* 30-3F */
	    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,	/* 40-4F */
	    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 50-5F */
	    0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0,	/* 60-6F */
	    1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 70-7F */
	    1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1,	/* 80-8F */
	    0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 90-9F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* A0-AF */
	    1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,	/* B0-BF */
	    1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1,	/* C0-CF */
	    1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,	/* D0-DF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* E0-EF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1,	/* F0-FF */
};

/* Private codec registers that are in the spec (1 = exists) */
static int __maybe_unused pr_reg_map[256] = {
	/* 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 00-0F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10-1F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 20-2F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0,	/* 30-3F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 40-4F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 50-5F */
	    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,	/* 60-6F */
	    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 70-7F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 80-8F */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 90-9F */
	    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,	/* A0-AF */
	    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* B0-BF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* C0-CF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* D0-DF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* E0-EF */
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* F0-FF */
};

struct rt5640_init_reg {
	u8 reg;
	u16 val;
};

static struct rt5640_init_reg init_list[] = {
	{RT5640_GEN_CTRL1,	0x3401},
	{RT5640_DEPOP_M1,	0x0019},
	{RT5640_DEPOP_M2,	0x3100},
	{RT5640_ADDA_CLK1,	0x1114},
	{RT5640_MICBIAS,	0x3030},
	{RT5640_PRIV_INDEX,	0x003d},
	{RT5640_PRIV_DATA,	0x2600},
	{RT5640_CLS_D_OUT,	0xa000},
	{RT5640_PRIV_INDEX,	0x001c},
	{RT5640_PRIV_DATA,	0x0D21},
	{RT5640_PRIV_INDEX,	0x001b},
	{RT5640_PRIV_DATA,	0x0000},
	{RT5640_PRIV_INDEX,	0x0012},
	{RT5640_PRIV_DATA,	0x0aa8},
	{RT5640_PRIV_INDEX,	0x0014},
	{RT5640_PRIV_DATA,	0x0aaa},
	{RT5640_PRIV_INDEX,	0x0020},
	{RT5640_PRIV_DATA,	0x6110},
	{RT5640_PRIV_INDEX,	0x0021},
	{RT5640_PRIV_DATA,	0xe0e0},
	{RT5640_PRIV_INDEX,	0x0023},
	{RT5640_PRIV_DATA,	0x1804},
	{RT5640_PWR_DIG1,	0x9801},
	{RT5640_PWR_MIXER,	0xF000},
	{RT5640_PWR_VOL,	0xCC00},
	{RT5640_PWR_ANLG1,	0xECD8},
};
#define RT5640_INIT_REG_LEN ARRAY_SIZE(init_list)

/*
 * Initialize I2C for rt5640
 *
 * @param bus no	i2c bus number in which rt5640 is connected
 */
static void rt5640_i2c_init(int bus_no)
{
	debug("%s: entry, bus_no = %d\n", __func__, bus_no);

	i2c_set_bus_num(bus_no);
	/* TODO(twarren@nvidia.com): Should get this from DT file! */
	gpio_request(GPIO_PV3, "RT5640");
	gpio_direction_output(GPIO_PV3, 1);
	udelay(1000000);
}

/*
 * Writes value to a device register through i2c
 *
 * @param reg	reg number to be write
 * @param data	data to be written to the above registor
 *
 * @return	int value 1 for change, 0 for no change or negative error code.
 */
static int rt5640_i2c_write(unsigned int reg, unsigned short data)
{
	unsigned char val[2];

	val[0] = (unsigned char)((data >> 8) & 0xff);
	val[1] = (unsigned char)(data & 0xff);
	debug("%s reg : 0x%04X, Data : 0x%04X\n", __func__, reg, data);
	return i2c_write(g_rt5640_i2c_dev_addr, reg, 1, val, 2);
}

/*
 * Read a value from a device register through i2c
 *
 * @param reg	reg number to be read
 * @param data	address of read data to be stored
 *
 * @return	int value 0 for success, -1 in case of error.
 */
static unsigned int rt5640_i2c_read(unsigned int reg , unsigned short *data)
{
	unsigned char val[2];
	int ret;

	ret = i2c_read(g_rt5640_i2c_dev_addr, reg, 1, val, 2);
	debug("%s reg : 0x%04X, Data : 0x%02X%02X\n", __func__, reg,
	      val[1], val[0]);
	if (ret != 0) {
		debug("%s: Error while reading register %#04x\n",
		      __func__, reg);
		return -1;
	}

	*data = (val[0] << 8) | val[1];

	return 0;
}

/*
 * update device register bits through i2c
 *
 * @param reg	codec register
 * @param mask	register mask
 * @param value	new value
 *
 * @return int value 1 if change in the register value,
 * 0 for no change or negative error code.
 */
static int rt5640_update_bits(unsigned int reg, unsigned short mask,
						unsigned short value)
{
	int change , ret = 0;
	unsigned short old, new;

	debug("%s: entry. reg, mask, value = 0x%04X, 0x%04X, 0x%04X\n",
	      __func__, reg, mask, value);

	if (rt5640_i2c_read(reg, &old) != 0)
		return -1;
	new = (old & ~mask) | (value & mask);
	change  = (old != new);
	if (change)
		ret = rt5640_i2c_write(reg, new);
	if (ret < 0)
		return ret;

	return change;
}

/* Initialize codec regs w/static/base values */
void rt5640_reg_init(void)
{
	int i;

	for (i = 0; i < RT5640_INIT_REG_LEN; i++)
		rt5640_i2c_write(init_list[i].reg, init_list[i].val);
}

/*
 * Sets i2s set format
 *
 * @param aif_id	Interface ID
 * @param fmt		i2S format
 *
 * @return -1 for error and 0  Success.
 */
int rt5640_set_fmt(int aif_id, unsigned int fmt)
{
	unsigned int reg_val = 0;

	debug("%s: entry, aif_id =%d, fmt = 0x%04x\n",
	      __func__, aif_id, fmt);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		debug("%s: Format is CBM_CFM\n", __func__);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		debug("%s: Format is CBS_CFS\n", __func__);
		reg_val |= RT5640_I2S_MS_S;
		break;
	default:
		return -1;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		debug("%s: Format is NB_NF\n", __func__);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		debug("%s: Format is IB_NF\n", __func__);
		reg_val |= RT5640_I2S_BP_INV;
		break;
	default:
		return -1;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		debug("%s: Format is I2S\n", __func__);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		debug("%s: Format is LJ\n", __func__);
		reg_val |= RT5640_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		debug("%s: Format is DSP_A\n", __func__);
		reg_val |= RT5640_I2S_DF_PCM_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		debug("%s: Format is DSP_B\n", __func__);
		reg_val  |= RT5640_I2S_DF_PCM_B;
		break;
	default:
		return -1;
	}

	if (aif_id == 1) {
		rt5640_update_bits(RT5640_I2S1_SDP,
				   RT5640_I2S_MS_MASK | RT5640_I2S_BP_MASK |
				   RT5640_I2S_DF_MASK, reg_val);
	}

	return 0;
}

/*
 * Configures Audio interface Clock
 *
 * @param rt5640	rt5640 information pointer
 * @param aif		Audio Interface ID
 *
 * @return		0 = success.
 */
static int configure_aif_clock(struct rt5640_priv *rt5640, int aif)
{
	debug("%s: entry, aif = %d\n", __func__, aif);

	/* ADC/DAC clock control 1: I2S clock pre-divider = 1 */
	rt5640_update_bits(RT5640_ADDA_CLK1,
			   RT5640_I2S_PD1_MASK, RT5640_I2S_PD1_1);
	return 0;
}

/*
 * Configures Audio interface for the given frequency
 *
 * @param rt5640	rt5640 information
 * @param aif_id	Audio Interface
 * @param clk_id	Input Clock ID
 * @param freq		Sampling frequency in Hz
 *
 * @return	int value 0 for success, -1 in case of error.
 */
static int rt5640_set_sysclk(struct rt5640_priv *rt5640, int aif_id,
				int clk_id, unsigned int freq)
{
	unsigned int reg = 0;
	int ret;

	debug("%s: entry, aif_id = %d\n", __func__, aif_id);
	debug("clk_id = %d, freq = %d\n", clk_id, freq);

	/* If the AIF is already set up, just return */
	if (freq == rt5640->sysclk && clk_id == rt5640->sysclk_src)
		return 0;

	switch (clk_id) {
	case RT5640_SYSCLK_MCLK:
		reg |= RT5640_SCLK_SRC_MCLK;
		break;
	case RT5640_SYSCLK_PLL:
		reg |= RT5640_SCLK_SRC_PLL1;
		break;
	default:
		printf("%s: Invalid clock id (%d)\n", __func__, clk_id);
		return -1;
	}

	rt5640_update_bits(RT5640_GLB_CLK, RT5640_SCLK_SRC_MASK, reg);
	rt5640->sysclk = freq;
	rt5640->sysclk_src = clk_id;

	ret = configure_aif_clock(rt5640, aif_id);
	if (ret < 0) {
		debug("%s: codec register access error\n", __func__);
		return -1;
	}

	debug("%s: Sysclk is %dHz and clock id is %d\n",
	      __func__, freq, clk_id);

	return 0;
}

/*
 * Initializes Volume for SPK and HP path
 *
 * @return	int value 0 for success, -1 in case of error.
 *
 */
static int rt5640_init_volume(void)
{
	int ret;

	debug("%s: entry\n", __func__);

	/* Unmute SPKOUT */
	ret = rt5640_update_bits(RT5640_SPK_VOL,
			RT5640_L_MUTE | RT5640_R_MUTE, 0);
	ret |= rt5640_update_bits(RT5640_SPK_VOL,
			RT5640_VOL_L_MUTE | RT5640_VOL_R_MUTE,	0);

	/* Unmute HP, set to max volume */
	ret |= rt5640_i2c_write(RT5640_HP_VOL, 0x0000);

	/* Unmute stereo DAC */
	ret |= rt5640_update_bits(RT5640_STO_DAC_MIXER,
			RT5640_M_DAC_L1 | RT5640_M_DAC_R1, 0);

	/* Unmute DAC mixer */
	ret |= rt5640_update_bits(RT5640_DIG_MIXER,
			RT5640_M_STO_L_DAC_L | RT5640_M_STO_R_DAC_R, 0);

	/* Unmute HPO mixer for DAC1 */
	ret |= rt5640_update_bits(RT5640_HPO_MIXER,
			RT5640_M_DAC1_HM, 0);

	/* Unmute DAC1 to SPKO mixer L/R */
	ret |= rt5640_update_bits(RT5640_SPO_L_MIXER,
			RT5640_M_DAC_L1_SPM_L, 0);
	ret |= rt5640_update_bits(RT5640_SPO_R_MIXER,
			RT5640_M_DAC_R1_SPM_R, 0);

	/* Unmute DAC1 to OUT mixer L/R */
	ret |= rt5640_update_bits(RT5640_OUT_L3_MIXER,
			RT5640_M_DAC_L1_OM_L, 0);
	ret |= rt5640_update_bits(RT5640_OUT_R3_MIXER,
			RT5640_M_DAC_R1_OM_R, 0);

	if (ret < 0) {
		debug("%s: codec register access error\n", __func__);
		return -1;
	}

	return 0;
}

/*
 * Initiliaze rt5640 codec device
 *
 * @param rt5640	rt5640 information
 *
 * @returns -1 for error  and 0 Success.
 */
static int rt5640_device_init(struct rt5640_priv *rt5640)
{
	const char *devname;
	unsigned short reg_data;
	int ret;

	debug("%s called\n", __func__);

	rt5640_i2c_write(RT5640_RESET, 0);		/* SW Reset */

	ret = rt5640_i2c_read(RT5640_RESET, &reg_data);
	if (ret < 0) {
		debug("%s: Failed to read ID register, ret = %d\n",
		      __func__, ret);
		goto err;
	}

	if (reg_data == RT5640_ID) {
		devname = "rt5640";
		debug("Device registered as type %d\n", rt5640->type);
		rt5640->type = RT5640;
	} else {
		printf("Device is not a rt5640, ID is %x\n", reg_data);
		ret = -1;
		goto err;
	}

	ret = rt5640_i2c_read(RT5640_VENDOR_ID2, &reg_data);
	if (ret < 0) {
		printf("%s: Failed to read vendor ID2 register, ret = %d\n",
		       __func__, ret);
		goto err;
	}

	rt5640->revision = reg_data;
	debug("%s revision = %04X\n", devname, rt5640->revision);

	/* Do static init (slam) of codec regs */
	rt5640_reg_init();

	ret |= rt5640_init_volume();
	if (ret < 0)
		goto err;

#if defined(DEBUG_REGS)
	int i;

	/* Read/Dump MX-00 thru MX-FF, as per reg_map */
	for (i = 0; i < 0x100; i++) {
		if (reg_map[i]) {
			rt5640_i2c_read(i, &reg_data);
			printf("%s: codec reg %02Xh == 0x%04X\n", __func__,
			       i, reg_data);
		}
	}

	/* Read/Dump PR-00 thru PR-FF, as per pr_reg_map */
	for (i = 0; i < 0x100; i++) {
		if (pr_reg_map[i]) {
			rt5640_i2c_write(RT5640_PRIV_INDEX, i);
			rt5640_i2c_read(RT5640_PRIV_DATA, &reg_data);
			printf("%s: codec private reg %02Xh == 0x%04X\n",
			       __func__, i, reg_data);
		}
	}
#endif	/* DEBUG */
	debug("%s: Codec chip init ok\n", __func__);
	return 0;
err:
	debug("%s: Codec chip init error\n", __func__);
	return -1;
}

/*
 * Gets fdt values for rt5640 config parameters
 *
 * @param pcodec_info	codec information structure
 * @param blob		FDT blob
 *
 * @return		int value 0 for success, -1 in case of error.
 */
static int get_codec_values(struct sound_codec_info *pcodec_info,
			const void *blob)
{
	int error = 0;
	enum fdt_compat_id compat;
	int node;
	int parent;

	debug("%s entry\n", __func__);

	/* Get the node from FDT for codec */
	node = fdtdec_next_compatible(blob, 0, COMPAT_REALTEK_RT5640_CODEC);
	if (node <= 0) {
		debug("Tegra sound: No node for codec in device tree\n");
		debug("node = %d\n", node);
		return -1;
	}

	parent = fdt_parent_offset(blob, node);
	if (parent < 0) {
		debug("%s: Cannot find node parent\n", __func__);
		return -1;
	}

	compat = fdtdec_lookup(blob, parent);
	switch (compat) {
	case COMPAT_NVIDIA_TEGRA114_I2C:
		pcodec_info->i2c_bus = i2c_get_bus_num_fdt(parent);
		error |= pcodec_info->i2c_bus;
		debug("i2c bus = %d\n", pcodec_info->i2c_bus);
		pcodec_info->i2c_dev_addr = fdtdec_get_int(blob, node,
							"reg", 0);
		error |= pcodec_info->i2c_dev_addr;
		debug("i2c dev addr = %02X\n", pcodec_info->i2c_dev_addr);
		break;
	default:
		debug("%s: Unknown compat id %d\n", __func__, compat);
		return -1;
	}

	pcodec_info->codec_type = CODEC_RT_5640;

	if (error == -1) {
		debug("fail to get rt5640 codec node properties\n");
		return -1;
	}

	return 0;
}

/* rt5640 Device Initialisation */
int rt5640_init(const void *blob, enum en_audio_interface aif_id,
			int sampling_rate, int mclk_freq,
			int bits_per_sample, unsigned int channels)
{
	int ret = 0;
	struct sound_codec_info *pcodec_info = &g_codec_info;

	debug("%s entry\n", __func__);
	debug("aif_id = %d, mclk_freq = %d\n", aif_id, mclk_freq);
	debug("sample rate: %d, BPS: %d, channels: %d\n",
	      sampling_rate, bits_per_sample, channels);

	/* Get the codec Values */
	if (get_codec_values(pcodec_info, blob) < 0) {
		debug("FDT Codec values failed\n");
		return -1;
	}

	/* store the device address for later use */
	g_rt5640_i2c_dev_addr = pcodec_info->i2c_dev_addr;
	rt5640_i2c_init(pcodec_info->i2c_bus);

	if (pcodec_info->codec_type == CODEC_RT_5640) {
		g_rt5640_info.type = RT5640;
	} else {
		debug("%s: Codec id [%d] not defined\n", __func__,
		      pcodec_info->codec_type);
		return -1;
	}

	ret = rt5640_device_init(&g_rt5640_info);
	if (ret < 0) {
		debug("%s: rt5640 codec chip init failed\n", __func__);
		return ret;
	}

	ret =  rt5640_set_sysclk(&g_rt5640_info, aif_id, RT5640_SYSCLK_MCLK,
							mclk_freq);
	if (ret < 0) {
		debug("%s: rt5640 codec set sys clock failed\n", __func__);
		return ret;
	}

	if (ret == 0) {
		debug("%s calling rt5640_set_fmt()\n", __func__);
		ret = rt5640_set_fmt(aif_id, SND_SOC_DAIFMT_I2S |
						SND_SOC_DAIFMT_NB_NF |
						SND_SOC_DAIFMT_CBS_CFS);
	}

	return ret;
}
