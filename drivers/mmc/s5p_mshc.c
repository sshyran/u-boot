/*
 * (C) Copyright 2012 Samsung Electronics Co. Ltd
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
#include <fdtdec.h>
#include <mmc.h>
#include <asm/arch/clk.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mshc.h>

/* Support just the one mshc host */
struct mmc mshci_dev;
struct mshci_host mshci_host;

/* Struct to hold mshci register and bus width */
struct fdt_mshci {
	struct s5p_mshci *reg;	/* address of registers in physical memory */
	int width;		/* bus width  */
};

/* reset the card interface unit */
static int mshci_reset_ciu(struct mshci_host *host)
{
	unsigned int ier;
	ulong	start;

	ier = readl(&host->reg->ctrl);
	ier |= CTRL_RESET;

	writel(ier, &host->reg->ctrl);

	start = get_timer(0);

	while (readl(&host->reg->ctrl) & CTRL_RESET) {
		if (get_timer(start) > TIMEOUT_MS) {
			debug("card interface reset failed\n");
			return -1;
		}
	}
}

static int mshci_reset_fifo(struct mshci_host *host)
{
	unsigned int ier;
	ulong start;

	ier = readl(&host->reg->ctrl);
	ier |= FIFO_RESET;

	writel(ier, &host->reg->ctrl);

	start = get_timer(0);
	while (readl(&host->reg->ctrl) & FIFO_RESET) {
		if (get_timer(start) > TIMEOUT_MS) {
			debug("fifo reset failed\n");
			return -1;
		}
	}
}

static int mshci_reset_dma(struct mshci_host *host)
{
	unsigned int ier;
	ulong start;

	ier = readl(&host->reg->ctrl);
	ier |= DMA_RESET;

	writel(ier, &host->reg->ctrl);

	start = get_timer(0);
	while (readl(&host->reg->ctrl) & DMA_RESET) {
		if (get_timer(start) > TIMEOUT_MS) {
			debug("dma reset failed\n");
			return -1;
		}
	}
	return 0;
}

static int mshci_reset_all(struct mshci_host *host)
{
	int ret;
	ulong start;

	/*
	 * before we reset ciu check the DATA0 line. if it is low and
	 * we resets the ciu then we might see some errors
	 */
	start = get_timer(0);
	while (readl(&host->reg->status) & DATA_BUSY) {
		if (get_timer(start) > TIMEOUT_MS) {
			debug("Controller did not release"
				"data0 before ciu reset\n");
			return -1;
		}
	}

	ret = mshci_reset_ciu(host);
	if (ret < 0) {
		debug("Fail to reset card.\n");
		return -1;
	}

	ret = mshci_reset_fifo(host);
	if (ret < 0) {
		debug("Fail to reset fifo.\n");
		return -1;
	}

	ret = mshci_reset_dma(host);
	if (ret < 0) {
		debug("Fail to reset dma.\n");
		return -1;
	}

	return 0;
}

static void mshci_set_mdma_desc(u8 *desc_vir, u8 *desc_phy,
		unsigned int des0, unsigned int des1, unsigned int des2)
{
	struct mshci_idmac *desc = (struct mshci_idmac *)desc_vir;

	desc->des0 = des0;
	desc->des1 = des1;
	desc->des2 = des2;
	desc->des3 = (unsigned int)desc_phy + sizeof(struct mshci_idmac);
}

static void mshci_prepare_data(struct mshci_host *host, struct mmc_data *data)
{
	unsigned int i, ret;
	unsigned int data_cnt;
	unsigned int des_flag;
	unsigned int blksz;
	static struct mshci_idmac idmac_desc[0x10000];
	/* TODO: check we really need this big array? */

	struct mshci_idmac *pdesc_dmac;
	ret = mshci_reset_fifo(host);
	if (ret < 0) {
		debug("Fail to reset FIFO\n");
		return -1;
	}

	pdesc_dmac = idmac_desc;
	blksz = data->blocksize;
	data_cnt = data->blocks;

	for  (i = 0;; i++) {
		des_flag = (MSHCI_IDMAC_OWN | MSHCI_IDMAC_CH);
		des_flag |= (i == 0) ? MSHCI_IDMAC_FS : 0;
		if (data_cnt <= 8) {
			des_flag |= MSHCI_IDMAC_LD;
			mshci_set_mdma_desc((u8 *)pdesc_dmac,
			(u8 *)virt_to_phys(pdesc_dmac),
			des_flag, blksz * data_cnt,
			(unsigned int)(virt_to_phys(data->dest)) +
			(unsigned int)(i * 0x1000));
			break;
		}
		/* max transfer size is 4KB per descriptor */
		mshci_set_mdma_desc((u8 *)pdesc_dmac,
			(u8 *)virt_to_phys(pdesc_dmac),
			des_flag, blksz * 8,
			virt_to_phys(data->dest) +
			(unsigned int)(i * 0x1000));

		data_cnt -= 8;
		pdesc_dmac++;
	}

	writel((unsigned int)virt_to_phys(idmac_desc), &host->reg->dbaddr);

	/* enable the Internal DMA Controller */
	setbits_le32(&host->reg->ctrl,
			readl(&host->reg->ctrl) | ENABLE_IDMAC | DMA_ENABLE);
	setbits_le32(&host->reg->bmod,
			readl(&host->reg->bmod) | BMOD_IDMAC_ENABLE |
				BMOD_IDMAC_FB);

	writel(data->blocksize, &host->reg->blksiz);
	writel(data->blocksize * data->blocks, &host->reg->bytcnt);
}

static int mshci_set_transfer_mode(struct mshci_host *host,
	struct mmc_data *data)
{
	int mode = CMD_DATA_EXP_BIT;

	if (data->blocks > 1)
		mode |= CMD_SENT_AUTO_STOP_BIT;
	if (data->flags & MMC_DATA_WRITE)
		mode |= CMD_RW_BIT;

	return mode;
}

/*
 * Sends a command out on the bus.
 *
 * @param mmc	mmc device
 * @param cmd	mmc_cmd to be sent on bud
 * @param data	mmc data to be send (optional)
 *
 * @return	return 0 if ok, else -1
 */
static int
s5p_mshci_send_command(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
	struct mshci_host *host = mmc->priv;

	int flags = 0, i;
	unsigned int mask;
	unsigned int timeout = 1000;
	ulong start;

	/*
	 * We shouldn't wait for data inihibit for stop commands, even
	 * though they might use busy signaling
	 */
	start = get_timer(0);
	while (readl(&host->reg->status) & DATA_BUSY) {
		if (get_timer(start) > COMMAND_TIMEOUT) {
			debug("timeout on data busy\n");
			return -1;
		}
	}

	if (readl(&host->reg->rintsts)) {
		if ((readl(&host->reg->rintsts) &
				(INTMSK_CDONE | INTMSK_ACD)) == 0)
			debug("there are pending interrupts 0x%x\n",
				readl(&host->reg->rintsts));
	}
	/* It clears all pending interrupts before sending a command*/
	writel(INTMSK_ALL, &host->reg->rintsts);

	if (data)
		mshci_prepare_data(host, data);

	writel(cmd->cmdarg, &host->reg->cmdarg);

	if (data)
		flags = mshci_set_transfer_mode(host, data);

	if ((cmd->resp_type & MMC_RSP_136) && (cmd->resp_type & MMC_RSP_BUSY))
		/* this is out of SD spec */
		return -1;

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		flags |= CMD_RESP_EXP_BIT;
		if (cmd->resp_type & MMC_RSP_136)
			flags |= CMD_RESP_LENGTH_BIT;
	}

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= CMD_CHECK_CRC_BIT;
	flags |= (cmd->cmdidx | CMD_STRT_BIT | CMD_USE_HOLD_REG |
			CMD_WAIT_PRV_DAT_BIT);

	mask = readl(&host->reg->cmd);
	if (mask & CMD_STRT_BIT)
		debug("cmd busy, current cmd: %d", cmd->cmdidx);

	writel(flags, &host->reg->cmd);
	/* wait for command complete by busy waiting. */
	for (i = 0; i < COMMAND_TIMEOUT; i++) {
		mask = readl(&host->reg->rintsts);
		if (mask & INTMSK_CDONE) {
			if (!data)
				writel(mask, &host->reg->rintsts);
			break;
		}
	}
	/* timeout for command complete. */
	if (COMMAND_TIMEOUT == i) {
		debug("timeout waiting for status update\n");
		return TIMEOUT;
	}

	if (mask & INTMSK_RTO) {
		if (((cmd->cmdidx == 8 || cmd->cmdidx == 41 ||
			cmd->cmdidx == 55)) == 0) {
			debug("response timeout error: 0x%x cmd: %d\n",
				mask, cmd->cmdidx);
		}
			return TIMEOUT;
	} else if (mask & INTMSK_RE) {
		debug("response error: 0x%x cmd: %d\n", mask, cmd->cmdidx);
		return -1;
	}
	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
				cmd->response[0] = readl(&host->reg->resp3);
				cmd->response[1] = readl(&host->reg->resp2);
				cmd->response[2] = readl(&host->reg->resp1);
				cmd->response[3] = readl(&host->reg->resp0);
		} else {
			cmd->response[0] = readl(&host->reg->resp0);
			debug("\tcmd->response[0]: 0x%08x\n", cmd->response[0]);
		}
	}

	if (data) {
		while (!(mask & (DATA_ERR | DATA_TOUT | INTMSK_DTO)))
			mask = readl(&host->reg->rintsts);
		writel(mask, &host->reg->rintsts);
		if (mask & (DATA_ERR | DATA_TOUT)) {
			debug("error during transfer: 0x%x\n", mask);
			/* make sure disable IDMAC and IDMAC_Interrupts */
			writel((readl(&host->reg->ctrl) &
			~(DMA_ENABLE | ENABLE_IDMAC)), &host->reg->ctrl);
			/* mask all interrupt source of IDMAC */
			writel(0, &host->reg->idinten);
			return -1;
		} else if (mask & INTMSK_DTO) {
			debug("mshci dma interrupt end\n");
		} else {
			debug("unexpected condition 0x%x\n", mask);
		}
		/* make sure disable IDMAC and IDMAC_Interrupts */
		writel((readl(&host->reg->ctrl) & ~(DMA_ENABLE | ENABLE_IDMAC)),
				&host->reg->ctrl);
		/* mask all interrupt source of IDMAC */
		writel(0, &host->reg->idinten);
	}
	udelay(100); /* TODO: please check why we need this */
	return 0;
}

/*
 * ON/OFF host controller clock
 *
 * @param host		pointer to mshci_host
 * @param val		to enable/disable clock
 */
static void mshci_clock_onoff(struct mshci_host *host, int val)
{

	if (val) {
		writel(CLK_ENABLE, &host->reg->clkena);
		writel(0, &host->reg->cmd);
		writel(CMD_ONLY_CLK, &host->reg->cmd);
			readl(&host->reg->cmd) & CMD_STRT_BIT;
	} else {
		writel(CLK_DISABLE, &host->reg->clkena);
		writel(0, &host->reg->cmd);
		writel(CMD_ONLY_CLK, &host->reg->cmd);
			readl(&host->reg->cmd) & CMD_STRT_BIT;
	}
}

#define MAX_EMMC_CLOCK	(40000000) /* Limit mshc clock to 40Mhz */

/*
 * change host controller clock
 *
 * @param host		pointer to mshci_host
 * @param clock		request clock
 */
static void mshci_change_clock(struct mshci_host *host, uint clock)
{
	int div;
	u32 mpll_clock;
	u32 sclk_mshc;

	if (clock == host->clock)
		return;

	/* If Input clock is higher than maximum mshc clock */
	if (clock > MAX_EMMC_CLOCK) {
		debug("Input clock is too high\n");
		clock = MAX_EMMC_CLOCK;
	}

	/* disable the clock before changing it */
	mshci_clock_onoff(host, CLK_DISABLE);

	sclk_mshc = get_mshc_clk_div();

	/* clkdiv */
	for (div = 1 ; div <= 0xFF; div++) {
		if (((sclk_mshc / 2) / (2*div)) <= clock) {
			writel(div, &host->reg->clkdiv);
			break;
		}
	}

	writel(div, &host->reg->clkdiv);
	writel(0, &host->reg->cmd);
	writel(CMD_ONLY_CLK, &host->reg->cmd);

	readl(&host->reg->cmd) & CMD_STRT_BIT;
	writel(readl(&host->reg->cmd) & (~CMD_SEND_CLK_ONLY),
					&host->reg->cmd);

	mshci_clock_onoff(host, CLK_ENABLE);
	host->clock = clock;

out:
	host->clock = clock;
	return;
}

/*
 * Set ios for host controller clock
 *
 * This sets the card bus width and clksel
 */
static void s5p_mshci_set_ios(struct mmc *mmc)
{
	struct mshci_host *host = mmc->priv;
	u32 mode, ddr, sdr;

	debug("bus_width: %x, clock: %d\n", mmc->bus_width, mmc->clock);

	if (mmc->clock > 0)
		mshci_change_clock(host, mmc->clock);

	if (mmc->bus_width == 8)
		writel(PORT0_CARD_WIDTH8, &host->reg->ctype);
	else if (mmc->bus_width == 4)
		writel(PORT0_CARD_WIDTH4, &host->reg->ctype);
	else
		writel(PORT0_CARD_WIDTH1, &host->reg->ctype);

	writel(0x00020001, &host->reg->clksel);
}

/*
 * Fifo init for host controller
 */
static void mshci_fifo_init(struct mshci_host *host)
{
	int fifo_val, fifo_depth, fifo_threshold;

	fifo_val = readl(&host->reg->fifoth);

	fifo_depth = 0x80;
	fifo_threshold = fifo_depth / 2;

	fifo_val &= ~(RX_WMARK | TX_WMARK | MSIZE_MASK);
	fifo_val |= (fifo_threshold | (fifo_threshold << 16) | MSIZE_8);
	writel(fifo_val, &host->reg->fifoth);
}


static void mshci_init(struct mshci_host *host)
{
	/* power on the card */
	writel(POWER_ENABLE, &host->reg->pwren);

	mshci_reset_all(host);
	mshci_fifo_init(host);

	/* clear all pending interrupts */
	writel(INTMSK_ALL, &host->reg->rintsts);

	/* interrupts are not used, disable all */
	writel(0, &host->reg->intmask);
}

static int s5p_mphci_init(struct mmc *mmc)
{
	struct mshci_host *host = (struct mshci_host *)mmc->priv;
	unsigned int ier;

	mshci_init(host);

	/* enumerate at 400KHz */
	mshci_change_clock(host, 400000);

	/* set auto stop command */
	ier = readl(&host->reg->ctrl);
	ier |= SEND_AS_CCSD;
	writel(ier, &host->reg->ctrl);

	/* set 1bit card mode */
	writel(PORT0_CARD_WIDTH1, &host->reg->ctype);

	writel(0xfffff, &host->reg->debnce);

	/* set bus mode register for IDMAC */
	writel(BMOD_IDMAC_RESET, &host->reg->bmod);

	writel(0x0, &host->reg->idinten);

	/* set the max timeout for data and response */
	writel(TMOUT_MAX, &host->reg->tmout);

	return 0;
}

static int s5p_mshci_initialize(int bus_width, struct s5p_mshci *reg)
{
	struct mmc *mmc;
	u32 chip_version, main_rev, sub_rev;

	mmc = &mshci_dev;

	sprintf(mmc->name, "S5P MSHC");

	mmc->priv = &mshci_host;
	mmc->send_cmd = s5p_mshci_send_command;
	mmc->set_ios = s5p_mshci_set_ios;
	mmc->init = s5p_mphci_init;

	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC;

	if (bus_width == 8)
		mmc->host_caps |= MMC_MODE_8BIT;
	else
		mmc->host_caps |= MMC_MODE_4BIT;

	mmc->f_min = 400000;
	mmc->f_max = 40000000;

	mshci_host.clock = 0;
	mshci_host.reg =  reg;
	mmc->b_max = 1;
	mmc_register(mmc);

	return 0;
}

#ifdef CONFIG_OF_CONTROL
int fdtdec_decode_mshci(const void *blob, struct fdt_mshci *config)
{
	int node;

	node = fdtdec_next_compatible(blob, 0, COMPAT_SAMSUNG_EXYNOS5_MSHC);
	if (node < 0)
		return node;

	config->width = fdtdec_get_int(blob, node,
				"samsung,mshci-bus-width", 8);

	config->reg = (struct s5p_mshci *)fdtdec_get_addr(blob, node, "reg");
	if ((fdt_addr_t)config->reg == FDT_ADDR_T_NONE)
		return -FDT_ERR_NOTFOUND;

	return 0;
}
#else
int fdtdec_decode_mshci(const void *blob, struct fdt_mshci *config)
{
	return -EINVAL;
}
#endif

int s5p_mshci_init(const void *blob)
{
	struct fdt_mshci config;
	struct s5p_mshci *base_addr =
			(struct s5p_mshci *)(samsung_get_base_mshci());

	if (!blob) {
		if (fdtdec_decode_mshci(blob, &config)) {
			debug("mshc configuration failed\n");
			return -1;
		}
		return s5p_mshci_initialize(config.width, config.reg);
	}
	return s5p_mshci_initialize(CONFIG_MSHC_BUS_WIDTH, base_addr);
}
