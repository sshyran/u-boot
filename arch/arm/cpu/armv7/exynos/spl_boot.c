/*
 * Copyright (C) 2012 Samsung Electronics
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
#include <config.h>
#include <elog.h>
#include <errno.h>
#include <mmc.h>
#include <dwmmc.h>
#include <dwmmc_simple.h>
#include <asm/gpio.h>
#include <asm/sections.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk.h>
#include <asm/arch/cpu.h>
#include <asm/arch/dmc.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/periph.h>
#include <asm/arch/power.h>
#include <asm/arch/setup.h>
#include <asm/arch/spi.h>
#include <asm/arch/spl.h>
#include "clock_init.h"

DECLARE_GLOBAL_DATA_PTR;

/* Index into irom ptr table */
enum index {
	MMC_INDEX,
	EMMC44_INDEX,
	EMMC44_END_INDEX,
	SPI_INDEX,
	USB_INDEX,
};

struct wake_event {
	struct event_header header;
	uint8_t sleep_type;
	uint8_t checksum;
} __packed;

/* IROM Function Pointers Table */
static const u32 irom_ptr_table[] = {
	[MMC_INDEX] = 0x02020030,	/* iROM Function Pointer-SDMMC boot */
	[EMMC44_INDEX] = 0x02020044,	/* iROM Function Pointer-EMMC4.4 boot*/
	[EMMC44_END_INDEX] = 0x02020048,/* iROM Function Pointer
						-EMMC4.4 end boot operation */
	[SPI_INDEX] = 0x02020058,	/* iROM Function Pointer-SPI boot */
	[USB_INDEX] = 0x02020070,	/* iROM Function Pointer-USB boot*/
	};

static uint32_t uboot_load_offset __attribute__ ((section(".data")));
static bool running_from_uboot __attribute__ ((section(".data")));
static uint8_t running_firmware __attribute__ ((section(".data")));

void *get_irom_func(int index)
{
	return (void *)*(u32 *)irom_ptr_table[index];
}

/*
 * Set/clear program flow prediction and return the previous state.
 */
static int config_branch_prediction(int set_cr_z)
{
	unsigned int cr;

	/* System Control Register: 11th bit Z Branch prediction enable */
	cr = get_cr();
	set_cr(set_cr_z ? cr | CR_Z : cr & ~CR_Z);

	return cr & CR_Z;
}

/*
 * Prepare a `fake' dram description so that the mmu can be set up, then
 * enable dcache.
 */
static void spl_enable_dcache(void)
{
	bd_t bd;

	memset(&bd, '\0', sizeof(bd));

	/* First bank is IRAM */
	bd.bi_dram[0].start = CONFIG_PHY_IRAM_BASE;
	bd.bi_dram[0].size = CONFIG_IRAM_TOP - CONFIG_PHY_IRAM_BASE;

	/* Second is all of DRAM - 2GB for now */
	bd.bi_dram[1].start = CONFIG_SYS_SDRAM_BASE;
	bd.bi_dram[1].size = 2UL << 30;

	gd->bd = &bd;
	gd->arch.tlb_addr = CONFIG_PHY_IRAM_TLB_BASE;
	dcache_enable();
	gd->bd = NULL;
}

#if defined(CONFIG_ELOG) || defined(CONFIG_EXYNOS_FAST_SPI_BOOT)
/**
 * Dedicated function for receiving data over SPI in 4 byte chunnks.
 *
 * @param regs	pointer to SPI controller register block
 * @parma todo	how many bytes to receive
 * @param dinp	pointer to the buffer for received data, 4 bytes aligned
 */
static void spi_fast_rx(struct exynos_spi *regs, int todo, uint32_t *dinp)
{
	unsigned rx_lvl, tx_lvl;
	unsigned tx_bytes;

	/*
	 * Just in case, round up the counter. Let's not worry about
	 * occasional overwrites of the tail of the buffer.
	 */
	todo = (todo + 3) & ~3;
	tx_bytes = todo;

	while (todo) {
		uint32_t spi_sts;

		spi_sts = readl(&regs->spi_sts);
		rx_lvl = (spi_sts >> 15) & 0x7f;
		tx_lvl = (spi_sts >> 6) & 0x7f;

		/*
		 * Even though the FIFO is at least 64 bytes in size (256
		 * bytes on port 0 on 5420), let's not feed more that 32 bytes
		 * at a time - with larger batches occasional RX erors creep
		 * in.
		 */
		while ((tx_lvl < 32) && tx_bytes) {
			writel(~0, &regs->tx_data);
			tx_lvl += 4;
			tx_bytes -= 4;
		}

		while ((rx_lvl >= 4) && todo) {
			*dinp++ = readl(&regs->rx_data);
			todo -= 4;
			rx_lvl -= 4;
		}
	}
}



/**
 * Send and receive data over SPI. This doesn't handle bus initialization or
 * cleanup.
 *
 * @param regs		pointer to SPI controller register block
 * @parma todo		how many bytes to send or receive
 * @param dinp		pointer to the buffer for received data, or NULL
 * @param doutp		pointer to the buffer of data to transmit, or NULL
 * @param word_chunks	whether to send the data in byte or word chunks
 */
static void spi_rx_tx(struct exynos_spi *regs, int todo, void *dinp,
		      void const *doutp, int word_chunks)
{
	uint8_t *rxp = (uint8_t *)dinp;
	uint8_t *txp = (uint8_t *)doutp;
	int rx_lvl, tx_lvl;
	uint out_bytes, in_bytes;
	int chunk_size;

	in_bytes = todo;
	out_bytes = todo;

	if (word_chunks)
		chunk_size = 4;
	else
		chunk_size = 1;

	setbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);
	writel(((todo + chunk_size - 1) / chunk_size) | SPI_PACKET_CNT_EN,
	       &regs->pkt_cnt);

	if (word_chunks && !doutp && !((unsigned)dinp & 3)) {
		/* This is the case for fast receive implementation. */
		spi_fast_rx(regs, todo, dinp);
		return;
	}

	while (in_bytes || out_bytes) {
		uint32_t spi_sts;
		int temp = 0xffffffff;

		spi_sts = readl(&regs->spi_sts);
		rx_lvl = ((spi_sts >> 15) & 0x7f);
		tx_lvl = ((spi_sts >> 6) & 0x7f);
		while (tx_lvl < 32 && out_bytes) {
			if (doutp) {
				memcpy(&temp, txp, chunk_size);
				txp += chunk_size;
			}
			writel(temp, &regs->tx_data);
			out_bytes -= chunk_size;
			tx_lvl += chunk_size;
		}
		while (rx_lvl >= chunk_size && in_bytes) {
			temp = readl(&regs->rx_data);
			if (dinp) {
				memcpy(rxp, &temp, chunk_size);
				rxp += chunk_size;
			}
			in_bytes -= chunk_size;
			rx_lvl -= chunk_size;
		}
	}
}

/**
 * Prepare the spi controller for a transaction
 *
 * @param regs	  spi controller register structure
 */
static void exynos_spi_init(struct exynos_spi *regs, int word_chunks)
{
	set_spi_clk(PERIPH_ID_SPI1, 50000000); /* set spi clock to 50Mhz */
	/* set the spi1 GPIO */
	exynos_pinmux_config(PERIPH_ID_SPI1, PINMUX_FLAG_NONE);

	/* set pktcnt and enable it */
	writel(4 | SPI_PACKET_CNT_EN, &regs->pkt_cnt);
	/* set FB_CLK_SEL */
	writel(SPI_FB_DELAY_180, &regs->fb_clk);
	/* set CH_WIDTH and BUS_WIDTH */
	clrbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_MASK |
				      SPI_MODE_BUS_WIDTH_MASK);
	if (word_chunks)
		setbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_WORD |
					      SPI_MODE_BUS_WIDTH_WORD);
	else
		setbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_BYTE |
					      SPI_MODE_BUS_WIDTH_BYTE);
	clrbits_le32(&regs->ch_cfg, SPI_CH_CPOL_L); /* CPOL: active high */

	/* clear rx and tx channel if set priveously */
	clrbits_le32(&regs->ch_cfg, SPI_RX_CH_ON | SPI_TX_CH_ON);

	setbits_le32(&regs->swap_cfg, SPI_RX_SWAP_EN |
		SPI_RX_BYTE_SWAP | SPI_RX_HWORD_SWAP);

	/* do a soft reset */
	setbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);

	/* now set rx and tx channel ON */
	setbits_le32(&regs->ch_cfg, SPI_RX_CH_ON | SPI_TX_CH_ON | SPI_CH_HS_EN);
	clrbits_le32(&regs->cs_reg, SPI_SLAVE_SIG_INACT); /* CS low */
}
/**
 * Shut down the spi controller after a transaction
 *
 * @param regs	  spi controller register structure
 */
static void exynos_spi_finish(struct exynos_spi *regs)
{
	setbits_le32(&regs->cs_reg, SPI_SLAVE_SIG_INACT);/* make the CS high */

	/*
	 * Let put controller mode to BYTE as
	 * SPI driver does not support WORD mode yet
	 */
	clrbits_le32(&regs->mode_cfg, SPI_MODE_CH_WIDTH_WORD |
					SPI_MODE_BUS_WIDTH_WORD);
	writel(0, &regs->swap_cfg);

	/*
	 * Flush spi tx, rx fifos and reset the SPI controller
	 * and clear rx/tx channel
	 */
	clrsetbits_le32(&regs->ch_cfg, SPI_CH_HS_EN, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_CH_RST);
	clrbits_le32(&regs->ch_cfg, SPI_TX_CH_ON | SPI_RX_CH_ON);
}
#endif /* defined(CONFIG_ELOG) || defined(CONFIG_EXYNOS_FAST_SPI_BOOT) */

#ifdef CONFIG_EXYNOS_FAST_SPI_BOOT
/**
 * Read data from spi flash
 *
 * @param offset	offset to read from
 * @parma size	  size of data to read, must be a divisible by 4
 * @param addr	  address to read data into
 */
static void exynos_spi_read(unsigned int offset, unsigned int size,
			    uintptr_t addr)
{
	int upto, todo;
	int i;
	struct exynos_spi *regs = (struct exynos_spi *)CONFIG_ENV_SPI_BASE;

	exynos_spi_init(regs, 1);

	/* Send read instruction (0x3h) followed by a 24 bit addr */
	writel((SF_READ_DATA_CMD << 24) | (offset & 0xffffff), &regs->tx_data);

	/* waiting for TX done */
	while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE))
		;

	for (upto = 0, i = 0; upto < size; upto += todo, i++) {
		todo = min(size - upto, (1 << 15));
		spi_rx_tx(regs, todo, (void *)(addr + (i << 15)), NULL, 1);
	}

	exynos_spi_finish(regs);
}
#endif  /* CONFIG_EXYNOS_FAST_SPI_BOOT */

#ifdef CONFIG_ELOG
/**
 * Write data into spi flash
 *
 * @param offset	offset to write to
 * @parma size	  size of data to write
 * @param addr	  address of data to write
 */
#define SPI_PAGE_SIZE (1 << 8)
static void exynos_spi_write(unsigned int offset, unsigned int size,
			     uintptr_t addr)
{
	struct exynos_spi *regs = (struct exynos_spi *)CONFIG_ENV_SPI_BASE;
	uint8_t *data = (uint8_t *)addr;

	while (size) {
		int block_size, page_offset;

		page_offset = offset % SPI_PAGE_SIZE;
		if ((page_offset + size) > SPI_PAGE_SIZE)
			block_size = SPI_PAGE_SIZE - page_offset;
		else
			block_size = size;

		/* Send a write enable command */
		exynos_spi_init(regs, 0);
		writel(SF_WRITE_ENABLE_CMD, &regs->tx_data);
		while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE))
			;
		exynos_spi_finish(regs);
		exynos_spi_init(regs, 0);

		/* Send a write command followed by a 24 bit addr */
		writel(SF_WRITE_DATA_CMD, &regs->tx_data);
		writel(offset >> 16, &regs->tx_data);
		writel(offset >> 8, &regs->tx_data);
		writel(offset >> 0, &regs->tx_data);

		/* Write out a block. */
		spi_rx_tx(regs, block_size, NULL, data, 0);

		/* waiting for TX done */
		while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE))
			;

		exynos_spi_finish(regs);

		data += block_size;
		offset += block_size;
		size -= block_size;
	}
}

/**
 * Small helper for exynos_find_end_of_log.
 *
 * @param regs	  SPI controller registers
 * @param data	  A buffer for the data
 * @param offset	The offset to read from, which is updated
 */
static void exynos_spi_get4bytes(struct exynos_spi *regs, void *data,
				 int *offset)
{
	spi_rx_tx(regs, 4, data, NULL, 1);
	*offset += 4;
}

/* Scan for the end of the event log coming in over SPI. */
static int exynos_find_end_of_log(void)
{
	struct exynos_spi *regs = (struct exynos_spi *)CONFIG_ENV_SPI_BASE;
	struct elog_header eheader;

	uint8_t bytes[4];
	int offset = 0;
	int log_offset;

	exynos_spi_init(regs, 1);

	/* Send read instruction (0x3h) followed by a 24 bit addr */
	writel((SF_READ_DATA_CMD << 24) | CONFIG_ELOG_OFFSET, &regs->tx_data);

	/* waiting for TX done */
	while (!(readl(&regs->spi_sts) & SPI_ST_TX_DONE))
		;

	spi_rx_tx(regs, sizeof(eheader), &eheader, NULL, 1);
	offset += sizeof(eheader);

	if (eheader.magic != ELOG_SIGNATURE) {
		/* Bad event log signature. */
		log_offset = -1;
	} else {
		/* Get to the start of the first event */
		log_offset = eheader.header_size;
	}

	/* Traverse the log looking for a free space. */
	while (log_offset >= 0) {
		uint8_t type;
		uint8_t event_length;

		while (log_offset >= offset)
			exynos_spi_get4bytes(regs, bytes, &offset);

		/* the type is the first byte */
		type = bytes[log_offset % 4];

		/* if we found the end, stop looking */
		if (type == ELOG_TYPE_EOL)
			break;

		/* load more bytes if the size is in the next chunk */
		if (log_offset + 1 >= offset)
			exynos_spi_get4bytes(regs, bytes, &offset);

		/* scan past this event */
		event_length = bytes[(log_offset + 1) % 4];
		if (!event_length) {
			/* Something is broken, bail out. */
			log_offset = -1;
			break;
		}

		log_offset += event_length;
		if (log_offset >= CONFIG_ELOG_SIZE)
			/* Reached the end of the log. */
			log_offset = -1;
	}

	exynos_spi_finish(regs);
	return log_offset;
}

/* Put a wake event in the event log. */
static void exynos_log_wake_event(void)
{
	int offset;
	struct wake_event event;
	uint8_t sleep_type = 3;

	offset = exynos_find_end_of_log();

	if (offset < 0)
		return;
	/* offset is relative to the log, we need relative to flash */
	offset += CONFIG_ELOG_OFFSET;

	elog_prepare_event(&event, ELOG_TYPE_FW_WAKE, &sleep_type, 1);
	exynos_spi_write(offset, sizeof(event), (uintptr_t)&event);
}
#endif /* CONFIG_ELOG */

#ifdef CONFIG_SPL_MMC_BOOT_WP
/**
 * Assert eMMC boot partition write protection
 *
 * Initialize the eMMC and send a command to assert power-on write
 * protection of eMMC boot partition 1.
 *
 * Additionally, apply power-on protection of the boot configuration,
 * to prevent the boot partition from being switched to one that is
 * not write protected.
 *
 * NOTE: This function uses features specific to eMMC 4.5.  Behavior on
 * other versions is undefined.
 *
 * @return 0 on success, -ve on error
 */
static int spl_set_boot_wp(void)
{
	struct dwmci_host host;
	struct mmc_cmd cmd;
	u8 reg;
	int ret;

	/* Configure mmc pins */
	ret = exynos_pinmux_config(DWMMC_SIMPLE_PERIPH_ID,
				   DWMMC_SIMPLE_PINMUX_FLAGS);
	if (ret)
		return ret;

	/* Initialize dwmci peripheral */
	ret = dwmci_simple_init(&host);
	if (ret)
		return ret;

	/* Startup mmc */
	ret = dwmci_simple_startup(&host);
	if (ret)
		return ret;

	/*
	 * Write protect boot partition 1 only,
	 * Only supported on eMMC 4.5, eMMC 4.41 behavior is undefined.
	 */
	reg = EXT_CSD_BOOT_WP_PWR_WP_EN | EXT_CSD_BOOT_WP_PART_SELECT |
		EXT_CSD_BOOT_WP_PWR_SEL_PART1;

	/* Set power-on write protect on boot partitions */
	cmd.cmdidx = MMC_CMD_SWITCH;
	cmd.resp_type = MMC_RSP_R1b;
	cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
				 (EXT_CSD_BOOT_WP << 16) |
				 (reg << 8);
	ret = dwmci_simple_send_cmd(&host, &cmd);
	if (ret)
		return ret;

	/* Set power-on protection of boot configuration */
	cmd.cmdidx = MMC_CMD_SWITCH;
	cmd.resp_type = MMC_RSP_R1b;
	cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
				 (EXT_CSD_BOOT_CONFIG_PROT << 16) |
				 (EXT_CSD_BOOT_CONFIG_PROT_PWR << 8);

	return dwmci_simple_send_cmd(&host, &cmd);
}

/**
 * Initialize the write protect GPIO to an input and floating.
 *
 * It will take a while for the input value to settle after this
 * initialization.
 */
static void spl_boot_wp_init(void)
{
	struct spl_machine_param *param = spl_get_machine_params();

	/* Invalid value */
	if (param->write_protect_gpio == 0xffffffff)
		return;

	gpio_direction_input(param->write_protect_gpio);
	gpio_set_pull(param->write_protect_gpio, 0);
}

/**
 * Check if write protection should be asserted, if so, assert it.
 *
 * @return 0 on success, -ve on error
 */
static int check_and_set_wp(void)
{
	struct spl_machine_param *param = spl_get_machine_params();

	/* Invalid value */
	if (param->write_protect_gpio == 0xffffffff)
		return 0;

	/* Active low WP input */
	if (!gpio_get_value(param->write_protect_gpio))
		return spl_set_boot_wp();

	return 0;
}
#endif

/*
 * Based on boot mode determine if dcache should be enabled.
 */
static int spl_use_dcache(enum boot_mode bootmode)
{
	return (bootmode != BOOT_MODE_USB);
}

enum boot_mode copy_uboot_to_ram(ulong uboot_addr, ulong uboot_size,
				 enum boot_mode bootmode, ulong uboot_offset,
				 bool enable_debug)
{
	int is_cr_z_set;
	unsigned int sec_boot_check;
#ifndef CONFIG_EXYNOS_FAST_SPI_BOOT
	u32 (*spi_copy)(u32 offset, u32 nblock, u32 dst);
#endif
	u32 (*copy_bl2)(u32 offset, u32 nblock, u32 dst);
	u32 (*copy_bl2_from_emmc)(u32 nblock, u32 dst);
	void (*end_bootop_from_emmc)(void);
	u32 (*usb_copy)(void);

	/* Read iRAM location to check for secondary USB boot mode */
	if (bootmode == BOOT_MODE_OM) {
		sec_boot_check = readl(EXYNOS_IRAM_SECONDARY_BASE);
		if (sec_boot_check == EXYNOS_USB_SECONDARY_BOOT)
			bootmode = BOOT_MODE_USB;
		else
			bootmode = readl(EXYNOS5_POWER_BASE) & OM_STAT;
	}

	if (spl_use_dcache(bootmode))
		spl_enable_dcache();

	if (enable_debug)
		puts("\n");
	switch (bootmode) {
	case BOOT_MODE_SERIAL:
#ifdef CONFIG_EXYNOS_FAST_SPI_BOOT
		if (enable_debug)
			puts("SPI fast...");
		/* let us our own function to copy u-boot from SF */
		exynos_spi_read(uboot_offset, uboot_size, uboot_addr);
#else
		if (enable_debug)
			puts("SPI iROM...");
		spi_copy = get_irom_func(SPI_INDEX);
		spi_copy(uboot_offset, uboot_size, uboot_addr);
#endif
		break;
	case BOOT_MODE_MMC:
		if (enable_debug)
			puts("MMC...");
		copy_bl2 = get_irom_func(MMC_INDEX);
		copy_bl2(CONFIG_UBOOT_OFFSET / MMC_MAX_BLOCK_LEN,
			 DIV_ROUND_UP(uboot_size, MMC_MAX_BLOCK_LEN),
			 uboot_addr);
		break;
	case BOOT_MODE_EMMC:
		if (enable_debug)
			puts("eMMC...");
#ifdef CONFIG_SPL_MMC_BOOT_WP
		/*
		 * GPIOs on the Exynos 5250 default to pulled down.  It will
		 * take a while for the GPIO to settle after being changed, so
		 * initialize it before copying U-Boot, giving it that time to
		 * settle.
		 */
		spl_boot_wp_init();
#endif
		/* Set the FSYS1 clock divisor value for EMMC boot */
		emmc_boot_clk_div_set();

		copy_bl2_from_emmc = get_irom_func(EMMC44_INDEX);
		end_bootop_from_emmc = get_irom_func(EMMC44_END_INDEX);

		copy_bl2_from_emmc(DIV_ROUND_UP(uboot_size, MMC_MAX_BLOCK_LEN),
				   uboot_addr);
		end_bootop_from_emmc();

#ifdef CONFIG_SPL_MMC_BOOT_WP
		/* Only set eMMC write protection if booting from eMMC */
		if (check_and_set_wp()) {
			/*
			* TODO(mpratt@chromium.org):
			* Reboot or retry if write protection fails to apply.
			*/
			printf("eMMC write-protect failed");
			hang();
		}
#endif
		break;
	case BOOT_MODE_USB:
		if (enable_debug)
			puts("USB...");
		/*
		 * iROM needs program flow prediction to be disabled
		 * before copy from USB device to RAM
		 */
		is_cr_z_set = config_branch_prediction(0);
		usb_copy = get_irom_func(USB_INDEX);
		usb_copy();
		config_branch_prediction(is_cr_z_set);
		break;
	default:
		puts("Invalid boot mode");
		hang();
		break;
	}

	if (spl_use_dcache(bootmode))
		flush_dcache_range(CONFIG_SYS_TEXT_BASE,
				   CONFIG_SYS_TEXT_BASE + uboot_size);
	if (enable_debug) {
		printf("loaded from offset %lx to %lx, size %lx", uboot_offset,
		       uboot_addr, uboot_size);
	}

	return bootmode;
}

void memzero(void *s, size_t n)
{
	char *ptr = s;
	size_t i;

	for (i = 0; i < n; i++)
		*ptr++ = '\0';
}

/**
 * Set up the U-Boot global_data pointer
 *
 * This sets the address of the global data, and sets up basic values.
 *
 * @param gdp   Value to give to gd
 */
static void setup_global_data(void)
{
	memzero((void *)gd, sizeof(gd_t));
	gd->flags |= GD_FLG_RELOC;
	gd->baudrate = CONFIG_BAUDRATE;
	gd->have_console = 1;
}

/**
 * Reset the CPU if the wakeup was not permitted.
 *
 * On some boards we need to look at a special GPIO to ensure that the wakeup
 * from sleep was valid.  If the wakeup is not valid we need to go through a
 * full reset.
 */
static void reset_if_invalid_wakeup(void)
{
	struct spl_machine_param *param = spl_get_machine_params();
	const u32 gpio = param->bad_wake_gpio;
	int is_bad_wake;

	/* We're a bad wakeup if the gpio was defined and was high */
	is_bad_wake = ((gpio != 0xffffffff) && gpio_get_value(gpio));

	if (is_bad_wake) {
		power_reset();

		/*
		 * We don't expect to get here, but it's better to loop
		 * if some bug in U-Boot makes the reset not happen.
		 */
		while (1)
			;
	}
}

__weak void board_process_wakeup(void)
{
}

void board_init_f(unsigned long bootflag)
{
	struct spl_machine_param *param;
	__attribute__((noreturn)) void (*uboot)(uint32_t marker);
	enum boot_mode boot_mode;
	bool enable_debug;
	uint32_t offset;
	int actions;

	param = spl_get_machine_params();
	enable_debug = param->spl_debug;
	boot_mode = param->boot_source;
	setup_global_data();
	arch_cpu_init();

	actions = lowlevel_select_actions();

	/*
	 * TODO(sjg@chromium.org): Support skipping the SDRAM init in RO SPL
	 * if (param->skip_sdram_init)
	 *	actions &= ~DO_MEM_INIT;
	 */

	/*
	 * We allow only serial init in RW SPL since all the clocks, etc.
	 * should be done by RO SPL.
	 * TODO(sjg@chromium.org): We could init SDRAM only in RW SPL
	 * but this may involve some work on the memory init code.
	 * Need to review this before release.
	 */
	if (running_from_uboot)
		actions &= DO_UART;

	/* Allow board-dependent wakeup processing */
	if (actions & DO_WAKEUP)
		board_process_wakeup();

	lowlevel_do_init(actions);
	if (actions & DO_WAKEUP) {
		reset_if_invalid_wakeup();
#ifdef CONFIG_ELOG
		exynos_log_wake_event();
#endif
		power_exit_wakeup();
	}

	offset = running_from_uboot ? uboot_load_offset : param->uboot_offset;
	copy_uboot_to_ram(param->uboot_start, param->uboot_size, boot_mode,
			  offset, enable_debug);

#ifdef CONFIG_SPL_VERIFY
	if (!board_image_verify(running_firmware, param->uboot_start,
				param->uboot_size, enable_debug)) {
		if (enable_debug)
			puts(", verified");
	} else {
		if (enable_debug)
			puts(", skip verify");
	}
#endif

	/* Jump to U-Boot image */
	if (enable_debug)
		puts(", jump\n");
	uboot = map_sysmem(param->uboot_start, param->uboot_size);
	(*uboot)(running_from_uboot ? SPL_RUNNING_FROM_UBOOT : 0);
	/* Never returns Here */
}

/* Place Holders */
void board_init_r(gd_t *id, ulong dest_addr)
{
	/* Function attribute is no-return */
	/* This Function never executes */
	while (1)
		;
}

void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3)
{
	running_from_uboot = (r0 == SPL_RUNNING_FROM_UBOOT);
	if (running_from_uboot) {
		running_firmware = r1;
		uboot_load_offset = r2;
	}
}

#ifdef CONFIG_SPL_SERIAL_SUPPORT
void puts(const char *s)
{
	serial_puts(s);
}

int printf(const char *fmt, ...)
{
	va_list args;
	int i;
	/* We won't be able to print strings longer then this. */
	char printf_buffer[150];

	va_start(args, fmt);
	i = vsnprintf(printf_buffer, sizeof(printf_buffer), fmt, args);
	va_end(args);

	puts(printf_buffer);

	return i;
}

#else

int printf(const char *fmt, ...)
{
	return 0;
}
#endif

struct spl_hash *spl_get_hash(void)
{
#ifdef CONFIG_VAR_SIZE_SPL
	void *hdr_base;

	/* Find the variable-size-SPL header */
	hdr_base = (void *)_start - CONFIG_SPL_HEADER_SIZE;
	return spl_extract_hash(hdr_base);
#else
	return NULL;
#endif
}
