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
#include <asm/errno.h>

#ifndef CONFIG_TEGRA2_NAND_PAGE_DATA_BYTES
#define CONFIG_TEGRA2_NAND_PAGE_DATA_BYTES		2048
#endif

#ifndef CONFIG_TEGRA2_NAND_PAGE_SPARE_BYTES
#define CONFIG_TEGRA2_NAND_PAGE_SPARE_BYTES		64
#endif

#ifndef CONFIG_TEGRA2_NAND_SKIPPED_SPARE_BYTES
#define CONFIG_TEGRA2_NAND_SKIPPED_SPARE_BYTES		4
#endif

#ifndef CONFIG_TEGRA2_NAND_RS_DATA_ECC_BYTES
#define CONFIG_TEGRA2_NAND_RS_DATA_ECC_BYTES		36
#endif

#ifndef CONFIG_TEGRA2_NAND_TAG_BYTES
#define CONFIG_TEGRA2_NAND_TAG_BYTES			20
#endif

#ifndef CONFIG_TEGRA2_NAND_TAG_ECC_BYTES
#define CONFIG_TEGRA2_NAND_TAG_ECC_BYTES		4
#endif

#ifndef CONFIG_TEGRA2_NAND_PAGE_SIZE
#define CONFIG_TEGRA2_NAND_PAGE_SIZE	NAND_CONFIG_PAGE_SIZE_2048
#endif

#ifndef CONFIG_TEGRA2_NAND_BUS_WIDTH
#define CONFIG_TEGRA2_NAND_BUS_WIDTH	NAND_CONFIG_BUS_WIDTH_8BIT
#endif

#define NAND_PAGE_DATA_BYTES		CONFIG_TEGRA2_NAND_PAGE_DATA_BYTES
#define NAND_PAGE_SPARE_BYTES		CONFIG_TEGRA2_NAND_PAGE_SPARE_BYTES
#define NAND_SKIPPED_SPARE_BYTES	CONFIG_TEGRA2_NAND_SKIPPED_SPARE_BYTES
#define NAND_RS_DATA_ECC_BYTES		CONFIG_TEGRA2_NAND_RS_DATA_ECC_BYTES
#define NAND_TAG_BYTES			CONFIG_TEGRA2_NAND_TAG_BYTES
#define NAND_TAG_ECC_BYTES		CONFIG_TEGRA2_NAND_TAG_ECC_BYTES

#define NAND_CMD_TIMEOUT_MS		10

#ifndef CONFIG_NAND_CLK_DIVISOR_DDDDDDDH
/*
 * Set clock divisor
 * 7 bits of D and 1 bit of H
 * divisor= (DDDDDDD + 1) + (H x 0.5)
 * clock = original clock / divisor
 * 6 means /4
 */
#define CONFIG_NAND_CLK_DIVISOR_DDDDDDDH	6
#endif

static int byte_count;

static struct nand_ecclayout tegra_eccoob = {
	.eccbytes = NAND_RS_DATA_ECC_BYTES + NAND_TAG_ECC_BYTES,
	.eccpos = {
		4,  5,  6,  7,  8,  9,  10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21,
		22, 23, 24, 25, 26, 27, 28, 29, 30,
		31, 32, 33, 34, 35, 36, 37, 38, 39,
		60, 61, 62, 63,
	},
	.oobavail = NAND_TAG_BYTES,
	.oobfree = {
		{ .offset = (NAND_SKIPPED_SPARE_BYTES+NAND_RS_DATA_ECC_BYTES),
		  .length = NAND_TAG_BYTES,
		},
	},
};

enum {
	ECC_OK,
	ECC_TAG_ERROR = 1 << 0,
	ECC_DATA_ERROR = 1 << 1
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

	for (i = 0; i < len/4; i++) {
		l = ((int *)buf)[i];
		writel(l, chip->IO_ADDR_W + NAND_RESP_0);
		writel(NAND_CMD_GO | NAND_CMD_PIO | NAND_CMD_TX |
			(NAND_CMD_TRANS_SIZE_BYTES4 <<
				NAND_CMD_TRANS_SIZE_SHIFT)
			| NAND_CMD_A_VALID | NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);

		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during write_buf\n");
	}
	if ((len % 4) != 0) {
		l = 0;
		for (j = 0; j < (len % 4); j++) {
			l2 = (int) buf[i*4+j];
			l |= (l2<< (8*j));
		}
		writel(l, chip->IO_ADDR_W + NAND_RESP_0);
		writel(NAND_CMD_GO | NAND_CMD_PIO | NAND_CMD_TX |
			(((len % 4)-1) << NAND_CMD_TRANS_SIZE_SHIFT) |
			NAND_CMD_A_VALID | NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
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
	for (i = 0; i < len/4; i++) {
		writel(NAND_CMD_GO | NAND_CMD_PIO | NAND_CMD_RX |
			(NAND_CMD_TRANS_SIZE_BYTES4 <<
				NAND_CMD_TRANS_SIZE_SHIFT)
			| NAND_CMD_A_VALID | NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during read_buf\n");
		l = readl(chip->IO_ADDR_R + NAND_RESP_0);
		buf_dword[i] = l;
	}
	if ((len % 4) != 0) {
		writel(NAND_CMD_GO | NAND_CMD_PIO | NAND_CMD_RX |
			(((len % 4)-1) << NAND_CMD_TRANS_SIZE_SHIFT) |
			NAND_CMD_A_VALID | NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		if (!tegra2_nand_waitfor_cmd_completion(mtd))
			printf("Command timeout during read_buf\n");
		l = readl(chip->IO_ADDR_R + NAND_RESP_0);
		for (j = 0; j < (len % 4); j++)
			buf[i*4+j] = (char) (l>>(8*j));
	}
}

/*
 * @return:
 *	1 - Command completed
 *	0 - Timeout
 */
static int tegra2_nand_waitfor_cmd_completion(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	int	i;
	u32 reg_val;

	for (i = 0; i < NAND_CMD_TIMEOUT_MS * 1000; i++) {
		if (!(readl(this->IO_ADDR_R + NAND_COMMAND_0) & NAND_CMD_GO)) {
			if ((readl(this->IO_ADDR_R + NAND_STATUS_0) &
				NAND_STATUS_RBSY0) &&
				(readl(this->IO_ADDR_R + NAND_ISR_0) &
				NAND_ISR_IS_CMD_DONE)) {
				reg_val = readl(this->IO_ADDR_R +
					NAND_DMA_MST_CTRL_0);
				/*
				 * If NAND_DMA_MST_CTRL_EN_A_ENABLE or
				 * NAND_DMA_MST_CTRL_EN_B_ENABLE is set,
				 * that means DMA engine is running, then we
				 * have to wait until
				 * NAND_DMA_MST_CTRL_IS_DMA_DONE
				 * is cleared for DMA transfer completion.
				 */
				if (reg_val & (NAND_DMA_MST_CTRL_EN_A_ENABLE |
					NAND_DMA_MST_CTRL_EN_B_ENABLE)) {
					if (reg_val &
						NAND_DMA_MST_CTRL_IS_DMA_DONE)
						return 1;
				} else
					return 1;
			}
		}
		udelay(1);
	}
	return 0;
}

/*
 * @return:
 *	1 - ready
 *	0 - not ready
 */
static int tegra2_nand_dev_ready(struct mtd_info *mtd)
{
	register struct nand_chip *chip = mtd->priv;
	int	reg_val;

	reg_val = readl(chip->IO_ADDR_R + NAND_STATUS_0);
	if (reg_val & NAND_STATUS_RBSY0)
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

static void tegra2_nand_clear_interrupt_status(struct nand_chip *chip)
{
	u32 reg_val;

	/* Clear interrupt status */
	reg_val = readl(chip->IO_ADDR_R + NAND_ISR_0);
	writel(reg_val, chip->IO_ADDR_W + NAND_ISR_0);
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
		/*
		 * Only command NAND_CMD_RESET or NAND_CMD_READID will come
		 * here before mtd->writesize is initialized, we don't have
		 * any action here because page size of NAND HY27UF084G2B
		 * is 2048 bytes and mtd->writesize will be 2048 after
		 * initialized.
		 */
	} else {
		/* Emulate NAND_CMD_READOOB */
		if (command == NAND_CMD_READOOB) {
			column += mtd->writesize;
			command = NAND_CMD_READ0;
		}

		if (column != -1 || page_addr != -1) {
			/* Serially input address */
			if (column != -1) {
				/* Adjust columns for 16 bit buswidth */
				if (chip->options & NAND_BUSWIDTH_16)
					column >>= 1;
			}
		}
	}

	tegra2_nand_clear_interrupt_status(chip);

	/* Stop DMA engine, clear DMA completion status */
	writel(NAND_DMA_MST_CTRL_EN_A_DISABLE
		| NAND_DMA_MST_CTRL_EN_B_DISABLE
		| NAND_DMA_MST_CTRL_IS_DMA_DONE,
		chip->IO_ADDR_W + NAND_DMA_MST_CTRL_0);

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {
	case NAND_CMD_READID:
		writel(NAND_CMD_READID, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO | NAND_CMD_CLE | NAND_CMD_ALE | NAND_CMD_PIO
			| NAND_CMD_RX |
			(NAND_CMD_TRANS_SIZE_BYTES4<<NAND_CMD_TRANS_SIZE_SHIFT)
			| NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		byte_count = 0;
		break;
	case NAND_CMD_READ0:
		writel(NAND_CMD_READ0, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_READSTART, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel((page_addr << 16) | (column & 0xFFFF),
			chip->IO_ADDR_W + NAND_ADDR_REG1_0);
		writel(page_addr >> 16, chip->IO_ADDR_W + NAND_ADDR_REG2_0);
		return;
	case NAND_CMD_SEQIN:
		writel(NAND_CMD_SEQIN, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_PAGEPROG, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel((page_addr << 16) | (column & 0xFFFF),
			chip->IO_ADDR_W + NAND_ADDR_REG1_0);
		writel(page_addr >> 16,
			chip->IO_ADDR_W + NAND_ADDR_REG2_0);
		return;
	case NAND_CMD_PAGEPROG:
		return;
	case NAND_CMD_ERASE1:
		writel(NAND_CMD_ERASE1, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_ERASE2, chip->IO_ADDR_W + NAND_CMD_REG2_0);
		writel(page_addr, chip->IO_ADDR_W + NAND_ADDR_REG1_0);
		writel(NAND_CMD_GO | NAND_CMD_CLE | NAND_CMD_ALE |
			NAND_CMD_SEC_CMD | NAND_CMD_CE0 | NAND_CMD_ALE_BYTES3,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		break;
	case NAND_CMD_RNDOUT:
		return;
	case NAND_CMD_ERASE2:
		return;
	case NAND_CMD_STATUS:
		writel(NAND_CMD_STATUS, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO | NAND_CMD_CLE | NAND_CMD_PIO | NAND_CMD_RX
			| (NAND_CMD_TRANS_SIZE_BYTES1 <<
				NAND_CMD_TRANS_SIZE_SHIFT)
			| NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		byte_count = 0;
		break;
	case NAND_CMD_RESET:
		writel(NAND_CMD_RESET, chip->IO_ADDR_W + NAND_CMD_REG1_0);
		writel(NAND_CMD_GO | NAND_CMD_CLE | NAND_CMD_CE0,
			chip->IO_ADDR_W + NAND_COMMAND_0);
		break;
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}
	if (!tegra2_nand_waitfor_cmd_completion(mtd))
		printf("Command 0x%02X timeout\n", command);
}

/*
 * After a DMA transfer for read, we call this function to see whether there
 * is any uncorrectable error on the pointed data buffer or oob buffer.
 *
 * @databuf:	data buffer
 * @a_len:	data buffer length
 * @oobbuf:	oob buffer
 * @b_len:	oob buffer length
 * @return:
 *	ECC_OK - no ECC error or correctable ECC error
 *	ECC_TAG_ERROR - uncorrectable tag ECC error
 *	ECC_DATA_ERROR - uncorrectable data ECC error
 *	ECC_DATA_ERROR + ECC_TAG_ERROR - uncorrectable data+tag ECC error
 */
static int check_ecc_error(struct nand_chip *chip, u8 *datbuf, int a_len,
	u8 *oobbuf, int b_len)
{
	int i;
	int return_val = ECC_OK;
	u32 reg_val;

	if (!(readl(chip->IO_ADDR_R + NAND_CONFIG_0) & NAND_ISR_IS_ECC_ERR))
		return ECC_OK;

	reg_val = readl(chip->IO_ADDR_R + NAND_DEC_STATUS_0);
	if ((reg_val & NAND_DEC_STATUS_A_ECC_FAIL) && datbuf) {
		reg_val = readl(chip->IO_ADDR_R + NAND_BCH_DEC_STATUS_BUF_0);
		/*
		 * If uncorrectable error occurs on data area, then see whether
		 * they are all FF. If all are FF, it's a blank page.
		 * Not error.
		 */
		if (reg_val & NAND_BCH_DEC_STATUS_FAIL_SEC_FLAG_MASK) {
			for (i = 0; i < a_len; i++)
				if (datbuf[i] != 0xFF) {
					return_val |= ECC_DATA_ERROR;
					break;
				}
		}
	}

	if ((reg_val & NAND_DEC_STATUS_B_ECC_FAIL) && oobbuf) {
		reg_val = readl(chip->IO_ADDR_R + NAND_BCH_DEC_STATUS_BUF_0);
		/*
		 * If uncorrectable error occurs on tag area, then see whether
		 * they are all FF. If all are FF, it's a blank page.
		 * Not error.
		 */
		if (reg_val & NAND_BCH_DEC_STATUS_FAIL_TAG_MASK) {
			for (i = 0; i < b_len; i++)
				if (oobbuf[i] != 0xFF) {
					return_val |= ECC_TAG_ERROR;
					break;
				}
		}
	}

	return return_val;
}


/**
 * tegra_nand_rw_page - page read/write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 * @page:	page number
 * @with_ecc:	1 to enable ECC, 0 to disable ECC
 * @is_writing:	0 for read, 1 for write
 * @return:	0 when successfully completed
 *		-EIO when command timeout
 */
static int tegra_nand_rw_page(struct mtd_info *mtd, struct nand_chip *chip,
	uint8_t *buf, int page, int with_ecc, int is_writing)
{
	u32 reg_val;
	int tag_size;
	struct nand_oobfree *free = chip->ecc.layout->oobfree;
	char tag_buf[NAND_TAG_BYTES + NAND_TAG_ECC_BYTES + 4];
	char *tag_ptr;

	/* Need to be 4-byte aligned */
	tag_ptr = tag_buf;
	tag_ptr += 3;
	tag_ptr = (char *) ((u32) tag_ptr & ~3);

	/* Stop command */
	writel(0, chip->IO_ADDR_W + NAND_COMMAND_0);

	/* Stop DMA engine and clear DMA completion status */
	writel(NAND_DMA_MST_CTRL_GO_DISABLE
		| NAND_DMA_MST_CTRL_IS_DMA_DONE,
		chip->IO_ADDR_W + NAND_DMA_MST_CTRL_0);
	writel((1<<chip->page_shift)-1,
		chip->IO_ADDR_W + NAND_DMA_CFG_A_0);
	writel((u32) buf, chip->IO_ADDR_W + NAND_DATA_BLOCK_PTR_0);

	if (with_ecc) {
		writel((u32) tag_ptr, chip->IO_ADDR_W + NAND_TAG_PTR_0);
		if (is_writing) {
			memcpy(tag_ptr, chip->oob_poi + free->offset,
				NAND_TAG_BYTES + NAND_TAG_ECC_BYTES);
		}
	} else
		writel((u32) chip->oob_poi, chip->IO_ADDR_W + NAND_TAG_PTR_0);

	/* Set ECC selection, configure ECC settings */
	reg_val = CONFIG_TEGRA2_NAND_PAGE_SIZE | CONFIG_TEGRA2_NAND_BUS_WIDTH;

	if (with_ecc) {
		tag_size = NAND_TAG_BYTES + NAND_TAG_ECC_BYTES;
		reg_val |= (NAND_CONFIG_SKIP_SPARE_SEL_4
			| NAND_CONFIG_SKIP_SPARE_ENABLE
			| NAND_CONFIG_HW_ECC_CORRECTION_ENABLE
			| NAND_CONFIG_ECC_EN_TAG_DISABLE
			| NAND_CONFIG_HW_ECC_SEL_RS
			| NAND_CONFIG_HW_ECC_ENABLE
			| NAND_CONFIG_TVAL4
			| (tag_size - 1));

		if (!is_writing)
			tag_size += NAND_SKIPPED_SPARE_BYTES;
	} else {
		tag_size = mtd->oobsize;
		reg_val |= (NAND_CONFIG_SKIP_SPARE_DISABLE
			| NAND_CONFIG_HW_ECC_CORRECTION_DISABLE
			| NAND_CONFIG_ECC_EN_TAG_DISABLE
			| NAND_CONFIG_HW_ECC_DISABLE
			| (tag_size - 1));
	}
	writel(reg_val, chip->IO_ADDR_W + NAND_CONFIG_0);

	writel(NAND_BCH_CONFIG_BCH_ECC_DISABLE,
		chip->IO_ADDR_W + NAND_BCH_CONFIG_0);

	writel(tag_size - 1,
		chip->IO_ADDR_W + NAND_DMA_CFG_B_0);

	tegra2_nand_clear_interrupt_status(chip);

	reg_val = NAND_CMD_CLE | NAND_CMD_ALE
		| NAND_CMD_SEC_CMD
		| (NAND_CMD_ALE_BYTES5 << NAND_CMD_ALE_BYTE_SIZE_SHIFT)
		| NAND_CMD_A_VALID
		| NAND_CMD_B_VALID
		| (NAND_CMD_TRANS_SIZE_BYTES_PAGE_SIZE_SEL <<
		NAND_CMD_TRANS_SIZE_SHIFT)
		| NAND_CMD_CE0;
	if (!is_writing)
		reg_val |= (NAND_CMD_AFT_DAT_DISABLE | NAND_CMD_RX);
	else
		reg_val |= (NAND_CMD_AFT_DAT_ENABLE | NAND_CMD_TX);
	writel(reg_val, chip->IO_ADDR_W + NAND_COMMAND_0);

	/* Setup DMA engine */
	reg_val = NAND_DMA_MST_CTRL_GO_ENABLE
		| NAND_DMA_MST_CTRL_BURST_8WORDS
		| NAND_DMA_MST_CTRL_EN_A_ENABLE
		| NAND_DMA_MST_CTRL_EN_B_ENABLE;

	if (!is_writing)
		reg_val |= NAND_DMA_MST_CTRL_DIR_READ;
	else
		reg_val |= NAND_DMA_MST_CTRL_DIR_WRITE;
	writel(reg_val, chip->IO_ADDR_W + NAND_DMA_MST_CTRL_0);

	/* Send out command to device */
	reg_val = readl(chip->IO_ADDR_R + NAND_COMMAND_0);
	reg_val |= NAND_CMD_GO;
	writel(reg_val, chip->IO_ADDR_W + NAND_COMMAND_0);

	if (!tegra2_nand_waitfor_cmd_completion(mtd)) {
		if (!is_writing)
			printf("Read Page 0x%X timeout ", page);
		else
			printf("Write Page 0x%X timeout ", page);
		if (with_ecc)
			printf("with ECC");
		else
			printf("without ECC");
		printf("\n");
		return -EIO;
	}

	if (with_ecc && !is_writing) {
			memcpy(chip->oob_poi, tag_ptr,
				NAND_SKIPPED_SPARE_BYTES);
			memcpy(chip->oob_poi + free->offset,
				tag_ptr + NAND_SKIPPED_SPARE_BYTES,
				NAND_TAG_BYTES);
			reg_val = (u32) check_ecc_error(chip, (u8 *) buf,
				1<<chip->page_shift,
				(u8 *) (tag_ptr + NAND_SKIPPED_SPARE_BYTES),
				NAND_TAG_BYTES);
			if (reg_val & ECC_TAG_ERROR)
				printf("Read Page 0x%X tag ECC error\n", page);
			if (reg_val & ECC_DATA_ERROR)
				printf("Read Page 0x%X data ECC error\n",
					page);
			if (reg_val & (ECC_DATA_ERROR | ECC_TAG_ERROR))
				return -EIO;
	}
	return 0;
}

/**
 * tegra_nand_read_page_hwecc - hardware ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 * @return:	0 when successfully completed
 *		-EIO when command timeout
 */
static int tegra_nand_read_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int page)
{
	return tegra_nand_rw_page(mtd, chip, buf, page, 1, 0);
}

/**
 * tegra_nand_write_page_hwecc - hardware ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void tegra_nand_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf)
{
	int page;

	page = (readl(chip->IO_ADDR_R + NAND_ADDR_REG1_0) >> 16) |
		(readl(chip->IO_ADDR_R + NAND_ADDR_REG2_0) << 16);

	tegra_nand_rw_page(mtd, chip, (uint8_t *) buf, page, 1, 1);
}


/**
 * tegra_nand_read_page_raw - read raw page data without ecc
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 * @return:	0 when successfully completed
 *		-EINVAL when chip->oob_poi is not double-word aligned
 *		-EIO when command timeout
 */
static int tegra_nand_read_page_raw(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int page)
{
	return tegra_nand_rw_page(mtd, chip, buf, page, 0, 0);
}

/**
 * tegra_nand_write_page_raw - raw page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void tegra_nand_write_page_raw(struct mtd_info *mtd,
		struct nand_chip *chip,	const uint8_t *buf)
{
	int page;

	page = (readl(chip->IO_ADDR_R + NAND_ADDR_REG1_0) >> 16) |
		(readl(chip->IO_ADDR_R + NAND_ADDR_REG2_0) << 16);

	tegra_nand_rw_page(mtd, chip, (uint8_t *) buf, page, 0, 1);
}

/**
 * tegra_nand_rw_oob - OOB data read/write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @with_ecc:	1 to enable ECC, 0 to disable ECC
 * @is_writing:	0 for read, 1 for write
 * @return:	0 when successfully completed
 *		-EINVAL when chip->oob_poi is not double-word aligned
 *		-EIO when command timeout
 */
static int tegra_nand_rw_oob(struct mtd_info *mtd, struct nand_chip *chip,
	int page, int with_ecc, int is_writing)
{
	u32 reg_val;
	int tag_size;
	struct nand_oobfree *free = chip->ecc.layout->oobfree;

	if (((int) chip->oob_poi) & 0x03)
		return -EINVAL;

	/* Stop command */
	writel(0, chip->IO_ADDR_W + NAND_COMMAND_0);

	/* Stop DMA engine and clear DMA completion status */
	writel(NAND_DMA_MST_CTRL_GO_DISABLE
		| NAND_DMA_MST_CTRL_IS_DMA_DONE,
		chip->IO_ADDR_W + NAND_DMA_MST_CTRL_0);

	writel((u32) chip->oob_poi, chip->IO_ADDR_W + NAND_TAG_PTR_0);

	/* Set ECC selection */
	reg_val = CONFIG_TEGRA2_NAND_PAGE_SIZE | CONFIG_TEGRA2_NAND_BUS_WIDTH;

	tag_size = mtd->oobsize;
	if (with_ecc) {
		reg_val |= (NAND_CONFIG_SKIP_SPARE_DISABLE
			| NAND_CONFIG_HW_ECC_CORRECTION_DISABLE
			| NAND_CONFIG_HW_ECC_DISABLE
			| NAND_CONFIG_ECC_EN_TAG_ENABLE);
	} else {
		reg_val |= (NAND_CONFIG_SKIP_SPARE_DISABLE
			| NAND_CONFIG_HW_ECC_CORRECTION_DISABLE
			| NAND_CONFIG_HW_ECC_DISABLE
			| NAND_CONFIG_ECC_EN_TAG_DISABLE);
	}
	reg_val |= (tag_size - 1);
	writel(reg_val, chip->IO_ADDR_W + NAND_CONFIG_0);

	writel(NAND_BCH_CONFIG_BCH_ECC_DISABLE,
		chip->IO_ADDR_W + NAND_BCH_CONFIG_0);

	if (is_writing && with_ecc)
		tag_size -= NAND_TAG_ECC_BYTES;

	writel(tag_size - 1, chip->IO_ADDR_W + NAND_DMA_CFG_B_0);

	tegra2_nand_clear_interrupt_status(chip);

	reg_val = NAND_CMD_CLE | NAND_CMD_ALE
		| NAND_CMD_SEC_CMD
		| (NAND_CMD_ALE_BYTES5 << NAND_CMD_ALE_BYTE_SIZE_SHIFT)
		| NAND_CMD_B_VALID
		| NAND_CMD_CE0;
	if (!is_writing)
		reg_val |= (NAND_CMD_AFT_DAT_DISABLE | NAND_CMD_RX);
	else
		reg_val |= (NAND_CMD_AFT_DAT_ENABLE | NAND_CMD_TX);
	writel(reg_val, chip->IO_ADDR_W + NAND_COMMAND_0);

	/* Setup DMA engine */
	reg_val = NAND_DMA_MST_CTRL_GO_ENABLE
		| NAND_DMA_MST_CTRL_BURST_8WORDS
		| NAND_DMA_MST_CTRL_EN_B_ENABLE;
	if (!is_writing)
		reg_val |= NAND_DMA_MST_CTRL_DIR_READ;
	else
		reg_val |= NAND_DMA_MST_CTRL_DIR_WRITE;

	writel(reg_val, chip->IO_ADDR_W + NAND_DMA_MST_CTRL_0);

	/* Send out command to device */
	reg_val = readl(chip->IO_ADDR_R + NAND_COMMAND_0);
	reg_val |= NAND_CMD_GO;
	writel(reg_val, chip->IO_ADDR_W + NAND_COMMAND_0);

	if (!tegra2_nand_waitfor_cmd_completion(mtd)) {
		if (!is_writing)
			printf("Read OOB of Page 0x%X timeout\n", page);
		else
			printf("Write OOB of Page 0x%X timeout\n", page);
		return -EIO;
	}

	if (with_ecc && !is_writing) {
		reg_val = (u32) check_ecc_error(chip, 0, 0,
			(u8 *) (chip->oob_poi + free->offset), NAND_TAG_BYTES);
		if (reg_val & ECC_TAG_ERROR)
			printf("Read OOB of Page 0x%X tag ECC error\n", page);
	}
	return 0;
}

/**
 * tegra_nand_read_oob - OOB data read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
static int tegra_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
	int page, int sndcmd)
{
	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	tegra_nand_rw_oob(mtd, chip, page, 0, 0);
	return sndcmd;
}

/**
 * tegra_nand_write_oob - OOB data write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to write
 */
static int tegra_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
	int page)
{
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);

	return tegra_nand_rw_oob(mtd, chip, page, 0, 1);
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
	int	reg_val;

	/* Assert RESET to NAND controller */
	reg_val = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	reg_val |= (1 <<
		CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_NDFLASH_RST_SHIFT);
	writel(reg_val, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);

	/* Enable clock to NAND controller */
	reg_val = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
	reg_val |= (1 <<
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_NDFLASH_SHIFT);
	writel(reg_val, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);

	reg_val = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	reg_val &=
	~CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_DIVISOR_MASK;
	reg_val |= CONFIG_NAND_CLK_DIVISOR_DDDDDDDH;
	writel(reg_val, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	udelay(1);

	/* Set clock source as PLLP_OUT0 */
	reg_val = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	reg_val &= ~(
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_MASK <<
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_SHIFT);
	reg_val |=
	CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_PLLP_OUT0;
	writel(reg_val, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
	udelay(2);

	/* Deassert RESET to NAND controller */
	reg_val = readl(NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	reg_val &= ~(1 <<
	CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_NDFLASH_RST_SHIFT);
	writel(reg_val, NV_ADDRESS_MAP_CLK_RST_BASE +
		CLK_RST_CONTROLLER_RST_DEVICES_L_0);

	/* Pinmux ATC_SEL uses NAND */
	reg_val = readl(NV_ADDRESS_MAP_APB_MISC_BASE +
		APB_MISC_PP_PIN_MUX_CTL_A_0);
	reg_val &= ~(APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_DEFAULT_MASK
		<< APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_SHIFT);
	reg_val |= (APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_NAND
		<< APB_MISC_PP_PIN_MUX_CTL_A_0_ATC_SEL_SHIFT);
	writel(reg_val, NV_ADDRESS_MAP_APB_MISC_BASE +
		APB_MISC_PP_PIN_MUX_CTL_A_0);

	reg_val = CONFIG_TEGRA2_NAND_PAGE_SIZE | CONFIG_TEGRA2_NAND_BUS_WIDTH;
	writel(reg_val, NAND_BASE + NAND_CONFIG_0);

	/*
	 * Frequence output of PLLP_OUT0 is set by BOOTROM to 216MHz
	 * to CLK_RST_CONTROLLER_PLLP_BASE_0,
	 * 216MHz / divisor 4 = 54MHZ
	 * 1 clock = 18.5 ns
	 * Set timing for NAND device, defined in tegra2_nand.h.
	 * If not defined, then use timing that was set by BOOTROM.
	 */
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

	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.layout = &tegra_eccoob;
	nand->ecc.size = NAND_PAGE_DATA_BYTES;
	nand->ecc.bytes = NAND_PAGE_SPARE_BYTES;

	nand->options = LP_OPTIONS;
	nand->cmdfunc = tegra2_nand_command;
	nand->read_byte = tegra2_nand_read_byte;
	nand->read_buf = tegra2_nand_read_buf;
	nand->write_buf = tegra2_nand_write_buf;
	nand->ecc.read_page = tegra_nand_read_page_hwecc;
	nand->ecc.write_page = tegra_nand_write_page_hwecc;
	nand->ecc.read_page_raw = tegra_nand_read_page_raw;
	nand->ecc.write_page_raw = tegra_nand_write_page_raw;
	nand->ecc.read_oob = tegra_nand_read_oob;
	nand->ecc.write_oob = tegra_nand_write_oob;
	return 0;
}
