/*
 * (C) Copyright 2006 Detlev Zundel, dzu@denx.de
 * (C) Copyright 2006 DENX Software Engineering
 * (C) Copyright 2011 NVIDIA Corporation <www.nvidia.com>
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
#include <asm/io.h>
#include <nand.h>
#include <asm/arch/gpio.h>
#include <asm/arch/nvcommon.h>
#include "../board.h"
#include "tegra2_nand.h"

#define NAND_PIO_CMD_TIMEOUT_MS	10
#ifndef CONFIG_NAND_CLK_DIVISOR_DDDDDDDH
 /* Set clock divisor
  * 7 bits of D and 1 bit of H
  * divisor= (DDDDDDD + 1) + (H x 0.5)
  * clock = original clock / divisor
  * 6 means /4  */
 #define CONFIG_NAND_CLK_DIVISOR_DDDDDDDH	6
#endif
static int byte_count;

static struct nand_ecclayout nand_soft_eccoob = {
	.oobfree = {
		{.offset = 40, .length = 24 }}
};

/**
 * tegra2_nand_read_byte - [DEFAULT] read one byte from the chip
 * @mtd:	MTD device structure
 *
 * Default read function for 8bit bus-width
 */
static uint8_t tegra2_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int dword_read;

	dword_read = readl(chip->IO_ADDR_R + NAND_RESP_0);
	dword_read = dword_read >> (8 * byte_count);
	byte_count ++;
	return (uint8_t) dword_read;
}

/**
 * tegra2_nand_write_buf - [DEFAULT] write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 8bit bus-width
 */
static void tegra2_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
	int len)
{
	int i, j, l, l2;
	struct nand_chip *chip = mtd->priv;

	for (i = 0; i < len/4; i++)
	{
		l = ((int *)buf)[i];
		writel(l, chip->IO_ADDR_W+ NAND_RESP_0);
		writel(NAND_CMD_GO+ NAND_CMD_PIO+ NAND_CMD_TX+
			(NAND_CMD_TRANS_SIZE_BYTES4<<NAND_CMD_TRANS_SIZE_SHIFT)
			+ NAND_CMD_A_VALID+ NAND_CMD_CE0,
			chip->IO_ADDR_W+ NAND_COMMAND_0);

		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during write_buf\n");
	}
	if ((len % 4) != 0)
	{
		l = 0;
		for (j=0; j<(len %4); j++)
		{
			l2 = (int) buf[i*4+j];
			l |= (l2<< (8*j));
		}
		writel(l, chip->IO_ADDR_W+ NAND_RESP_0);
		writel(NAND_CMD_GO+ NAND_CMD_PIO+NAND_CMD_TX+
			(((len % 4)-1)<<NAND_CMD_TRANS_SIZE_SHIFT)+
			NAND_CMD_A_VALID+ NAND_CMD_CE0,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during write_buf\n");
	}
}

/**
 * tegra2_nand_read_buf - [DEFAULT] read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit bus-width
 */
static void tegra2_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i, j, l;
	struct nand_chip *chip = mtd->priv;
	int *buf_dword;

	buf_dword = (int *) buf;
	for (i = 0; i < len/4; i++)
	{
		writel(NAND_CMD_GO+ NAND_CMD_PIO+ NAND_CMD_RX+
			(NAND_CMD_TRANS_SIZE_BYTES4<<NAND_CMD_TRANS_SIZE_SHIFT)
			+ NAND_CMD_A_VALID+ NAND_CMD_CE0,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during read_buf\n");
		l = readl(chip->IO_ADDR_R+ NAND_RESP_0);
		buf_dword[i] = l;
	}
	if ((len % 4) != 0)
	{
		writel(NAND_CMD_GO+ NAND_CMD_PIO+NAND_CMD_RX+
			(((len % 4)-1)<<NAND_CMD_TRANS_SIZE_SHIFT)+
			NAND_CMD_A_VALID+ NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during read_buf\n");
		l = readl(chip->IO_ADDR_R+ NAND_RESP_0);
		for (j=0; j<(len %4); j++)
		{
			buf[i*4+j] = (char) (l>>(8*j));
		}
	}
}

/*
 * = 1 - Command completed
 *   0 - Timeout
 */
static int tegra2_nand_waitfor_cmd_completion(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	int	i;

	for (i=0; i< NAND_PIO_CMD_TIMEOUT_MS * 1000; i++)
	{
		if (!(readl(this->IO_ADDR_R + NAND_CMD_REG1_0) & NAND_CMD_GO))
		{
			if (readl(this->IO_ADDR_R + NAND_STATUS_0) &
				NAND_STATUS_RBSY0)
				break;
		}
		udelay(1);
	}
	if (i== NAND_PIO_CMD_TIMEOUT_MS * 1000)
		return 0;
	return 1;
}

/*
 * = 1 - ready
 *   0 - not ready
 */
static int tegra2_nand_dev_ready(struct mtd_info *mtd)
{
	register struct nand_chip *chip = mtd->priv;
	int	RegVal;

	RegVal= readl(chip->IO_ADDR_R + NAND_STATUS_0);
	if (RegVal & NAND_STATUS_RBSY0)
		return 1;
	else
		return 0;
}

/*
 *	hardware specific access to control-lines
 */
static void tegra2_nand_hwcontrol(struct mtd_info *mtd, int cmd,
	unsigned int ctrl)
{
}

/**
 * tegra2_nand_command - [DEFAULT] Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 */
static void tegra2_nand_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/*
	 * Write out the command to the device.
	 */
	if (mtd->writesize < 2048) {
		/* Only command NAND_CMD_RESET or NAND_CMD_READID will come
		 * here before mtd->writesize is initialized, we don't have
		 * any action here because page size of NAND HY27UF084G2B
		 * is 2048 bytes and mtd->writesize will be 2048 after
		 * initialized. */
	}
	else
	{
		/* Emulate NAND_CMD_READOOB */
		if (command == NAND_CMD_READOOB)
		{
			column += mtd->writesize;
			command = NAND_CMD_READ0;
		}

		if (column != -1 || page_addr != -1)
		{
			/* Serially input address */
			if (column != -1)
			{
				/* Adjust columns for 16 bit buswidth */
				if (chip->options & NAND_BUSWIDTH_16)
					column >>= 1;
			}
		}
	}

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {
	case NAND_CMD_READID:
		writel(NAND_CMD_READID, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_ALE+ NAND_CMD_PIO+
			NAND_CMD_RX+
			(NAND_CMD_TRANS_SIZE_BYTES4<<NAND_CMD_TRANS_SIZE_SHIFT)
			+ NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		byte_count = 0;
		break;
	case NAND_CMD_READ0:
		writel(NAND_CMD_READ0, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_READSTART, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel((page_addr <<16)+ (column & 0xFFFF),
			chip->IO_ADDR_W+ NAND_ADDR_REG1_0);
		writel(page_addr >>16, chip->IO_ADDR_W + NAND_ADDR_REG2_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_ALE+ NAND_CMD_PIO+
			NAND_CMD_SEC_CMD+ NAND_CMD_CE0+ NAND_CMD_ALE_BYTES5,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		byte_count = 0;
		break;
	case NAND_CMD_SEQIN:
		writel(NAND_CMD_SEQIN, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_PAGEPROG, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel((page_addr <<16)+ (column & 0xFFFF),
			chip->IO_ADDR_W+ NAND_ADDR_REG1_0);
		writel(page_addr >>16, chip->IO_ADDR_W + NAND_ADDR_REG2_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_ALE+ NAND_CMD_PIO+
			NAND_CMD_SEC_CMD+ NAND_CMD_AFT_DAT+ NAND_CMD_CE0+
			NAND_CMD_ALE_BYTES5,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		break;
	case NAND_CMD_PAGEPROG:
		writel(NAND_CMD_PAGEPROG, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO+NAND_CMD_CLE+NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		break;
	case NAND_CMD_ERASE1:
		writel(NAND_CMD_ERASE1, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_ERASE2, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel(page_addr, chip->IO_ADDR_W + NAND_ADDR_REG1_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_ALE+
			NAND_CMD_SEC_CMD+ NAND_CMD_CE0+ NAND_CMD_ALE_BYTES3,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		break;
	case NAND_CMD_RNDOUT:
		writel(NAND_CMD_RNDOUT, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_RNDOUTSTART, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel((column & 0xFFFF), chip->IO_ADDR_W + NAND_ADDR_REG1_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_ALE+ NAND_CMD_PIO+
			NAND_CMD_SEC_CMD+ NAND_CMD_CE0+ NAND_CMD_ALE_BYTES2,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		break;
	case NAND_CMD_ERASE2:
		return;
	case NAND_CMD_STATUS:
		writel(NAND_CMD_STATUS, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO+ NAND_CMD_CLE+ NAND_CMD_PIO+ NAND_CMD_RX+
			(NAND_CMD_TRANS_SIZE_BYTES1<<NAND_CMD_TRANS_SIZE_SHIFT)
			+NAND_CMD_CE0,
			chip->IO_ADDR_W+ NAND_COMMAND_0);
		byte_count = 0;
		break;

	case NAND_CMD_RESET:
		writel(NAND_CMD_RESET, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO+NAND_CMD_CLE+NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		break;
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready)
		{
			udelay(chip->chip_delay);
			return;
		}
	}
	if (!tegra2_nand_waitfor_cmd_completion(mtd))
		printf("Command 0x%02X timeout\n", command);
}

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - cmd_ctrl: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccm.ode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int board_nand_init(struct nand_chip *nand)
{
	int	RegVal;

	/* Assert RESET to NAND controller */
	RegVal = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	RegVal |= (1 <<
		CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_NDFLASH_RST_SHIFT);
	writel(RegVal, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);

	/* enable clock to NAND controller */
	RegVal = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
	RegVal |= (1 <<
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_NDFLASH_SHIFT);
	writel(RegVal, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);

	RegVal = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	RegVal &=
	~CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_DIVISOR_MASK;
	RegVal |= CONFIG_NAND_CLK_DIVISOR_DDDDDDDH;
	writel(RegVal, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	udelay(1);

	/* Set clock source as PLLP_OUT0 */
	RegVal = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	RegVal &= ~(
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_MASK <<
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_SHIFT);
	RegVal |=
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_PLLP_OUT0;
	writel(RegVal, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	udelay(2);

	/* Deassert RESET to NAND controller */
	RegVal = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	RegVal &= ~(1 <<
	CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_NDFLASH_RST_SHIFT);
	writel(RegVal, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);

	/* pinmux ATC_SEL uses NAND */
	RegVal = readl(NV_ADDRESS_MAP_APB_MISC_BASE +
		APB_MISC_PP_PIN_MUX_CTL_A_0);
	RegVal &= ~(APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_DEFAULT_MASK
		<< APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_SHIFT);
	RegVal |= (APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_NAND
		<< APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_SHIFT);
	writel(RegVal, NV_ADDRESS_MAP_APB_MISC_BASE +
		APB_MISC_PP_PIN_MUX_CTL_A_0);

	RegVal = NAND_CONFIG_BUS_WIDTH_8BIT + NAND_CONFIG_PAGE_SIZE_2048;
	writel(RegVal, NAND_BASE + NAND_CONFIG_0);

	/* Frequence output of PLLP_OUT0 is set by BOOTROM to 216MHz
	 * to CLK_RST_CONTROLLER_PLLP_BASE_0,
	 * 216MHz / divisor 4 = 54MHZ
	 * 1 clock = 18.5 ns */
	/* Set timing for NAND device, defined in tegra2_nand.h.
	 * If not defined, then use timing that was set by BOOTROM. */
#ifdef CONFIG_TEGRA2_NAND_TIMING
	writel(CONFIG_TEGRA2_NAND_TIMING, NAND_BASE + NAND_TIMING_0);
#endif
#ifdef CONFIG_TEGRA2_NAND_TIMING2
	writel(CONFIG_TEGRA2_NAND_TIMING2, NAND_BASE + NAND_TIMING2_0);
#endif
#if (LINUX_MACH_TYPE == MACH_TYPE_SEABOARD)
	/* GPIO port H bit 3, H.03, GMI_AD11->MFG_MODE_R, */
	tg2_gpio_direction_output(7, 3, 1);
#endif
#if (LINUX_MACH_TYPE == MACH_TYPE_HARMONY)
	/* GPIO port C bit 7, C.07, GMI_WP->NAND_WP */
	tg2_gpio_direction_output(2, 7, 1);
#endif
	nand->cmd_ctrl = tegra2_nand_hwcontrol;
	nand->dev_ready  = tegra2_nand_dev_ready;
	nand->ecc.mode = NAND_ECC_NONE;
	nand->ecc.layout = &nand_soft_eccoob;
	nand->options = LP_OPTIONS;
	nand->cmdfunc = tegra2_nand_command;
	nand->read_byte = tegra2_nand_read_byte;
	nand->read_buf = tegra2_nand_read_buf;
	nand->write_buf = tegra2_nand_write_buf;
	return 0;
}

