/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <common.h>
#include <asm/io.h>
#include <asm/arch/ahb.h>
#include <asm/arch/clock.h>
#include <asm/arch/flow.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/tegra.h>
#include <asm/arch/clk_rst.h>
#include <asm/arch/pmc.h>
#include <asm/arch-tegra/ap.h>
#include "../tegra-common/cpu.h"

/* Tegra124-specific CPU init code */

static void enable_cpu_power_rail(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	debug("enable_cpu_power_rail entry\n");

	/* un-tristate PWR_I2C SCL/SDA, rest of the defaults are correct */
	pinmux_tristate_disable(PINGRP_PWR_I2C_SCL);
	pinmux_tristate_disable(PINGRP_PWR_I2C_SDA);

	pmic_enable_cpu_vdd();

	/*
	 * Set CPUPWRGOOD_TIMER - APB clock is 1/2 of SCLK (102MHz),
	 * set it for 5ms as per SysEng (102MHz/5mS = 510000 (7C830h).
	 */
	writel(0x7C830, &pmc->pmc_cpupwrgood_timer);

	/* Set polarity to 0 (normal) and enable CPUPWRREQ_OE */
	clrbits_le32(&pmc->pmc_cntrl, CPUPWRREQ_POL);
	setbits_le32(&pmc->pmc_cntrl, CPUPWRREQ_OE);
}

static void enable_cpu_clocks(void)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 reg;

	debug("enable_cpu_clocks entry\n");

	/* Wait for PLL-X to lock */
	do {
		reg = readl(&clkrst->crc_pll_simple[SIMPLE_PLLX].pll_base);
		debug("%s: PLLX base = 0x%08X\n", __func__, reg);
	} while ((reg & (1 << 27)) == 0);

	debug("%s: PLLX locked, delay for stable clocks\n", __func__);
	/* Wait until all clocks are stable */
	udelay(PLL_STABILIZATION_DELAY);

	debug("%s: Setting CCLK_BURST and DIVIDER\n", __func__);
	writel(CCLK_BURST_POLICY, &clkrst->crc_cclk_brst_pol);
	writel(SUPER_CCLK_DIVIDER, &clkrst->crc_super_cclk_div);

	debug("%s: Enabling clock to all CPUs\n", __func__);
	/* Enable the clock to all CPUs */
	reg = readl(&clkrst->crc_clk_cpu_cmplx_clr);
	reg |= (CLR_CPU3_CLK_STP + CLR_CPU2_CLK_STP);
	reg |= CLR_CPU1_CLK_STP;
	writel((reg | CLR_CPU0_CLK_STP), &clkrst->crc_clk_cpu_cmplx_clr);

	debug("%s: Enabling main CPU complex clocks\n", __func__);
	/* Always enable the main CPU complex clocks */
	clock_enable(PERIPH_ID_CPU);
	clock_enable(PERIPH_ID_CPULP);
	clock_enable(PERIPH_ID_CPUG);

	debug("%s: Done\n", __func__);
}

static void remove_cpu_resets(void)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 reg;

	debug("remove_cpu_resets entry\n");

	/* Take the slow and fast partitions out of reset */
	reg = CLR_NONCPURESET;
	writel(reg, &clkrst->crc_rst_cpulp_cmplx_clr);
	writel(reg, &clkrst->crc_rst_cpug_cmplx_clr);

	/* Clear the SW-controlled reset of the slow cluster */
	reg = (CLR_CPURESET0 + CLR_DBGRESET0 + CLR_CORERESET0 + CLR_CXRESET0);
	reg |= (CLR_L2RESET + CLR_PRESETDBG);
	writel(reg, &clkrst->crc_rst_cpulp_cmplx_clr);

	/* Clear the SW-controlled reset of the fast cluster */
	reg = (CLR_CPURESET0 + CLR_DBGRESET0 + CLR_CORERESET0 + CLR_CXRESET0);
	reg |= (CLR_CPURESET1 + CLR_DBGRESET1 + CLR_CORERESET1 + CLR_CXRESET1);
	reg |= (CLR_CPURESET2 + CLR_DBGRESET2 + CLR_CORERESET2 + CLR_CXRESET2);
	reg |= (CLR_CPURESET3 + CLR_DBGRESET3 + CLR_CORERESET3 + CLR_CXRESET3);
	reg |= (CLR_L2RESET + CLR_PRESETDBG);
	writel(reg, &clkrst->crc_rst_cpug_cmplx_clr);
}

static void t1x4_init_mem_ctlr(void)
{
	struct ahb_ctlr *ahbctlr = (struct ahb_ctlr *)NV_PA_AHB_BASE;
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	debug("%s entry\n", __func__);

	/* Disable any further writes to the address map config register */
	setbits_le32(&pmc->pmc_sec_disable, AMAP_WRITE_ON);

	/* Set up the AHB Mem Gizmo for split writes and fast rearb */
	clrbits_le32(&ahbctlr->gizmo_ahb_mem, GIZ_DONT_SPLIT_AHB_WR);
	setbits_le32(&ahbctlr->gizmo_ahb_mem,
		    (GIZ_ENABLE_SPLIT + GIZ_ENB_FAST_REARB));

	/* Start USB AHB write requests immediately */
	setbits_le32(&ahbctlr->gizmo_usb, GIZ_USB_IMMEDIATE);

	debug("%s exit\n", __func__);
}

/**
 * The T1x4 requires some special clock initialization, including setting up
 * the DVC I2C, turning on MSELECT and selecting the G CPU cluster
 */
void t1x4_init_clocks(void)
{
	struct flow_ctlr *flow = (struct flow_ctlr *)NV_PA_FLOW_BASE;
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	struct clk_rst_ctlr *clkrst =
			(struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 val;

	debug("t1x4_init_clocks entry\n");

	/* Set active CPU cluster to G */
	clrbits_le32(&flow->cluster_control, 1);

	/* Change the oscillator drive strength */
	val = readl(&clkrst->crc_osc_ctrl);
	val &= ~OSC_XOFS_MASK;
	val |= (OSC_DRIVE_STRENGTH << OSC_XOFS_SHIFT);
	writel(val, &clkrst->crc_osc_ctrl);

	/* Update same value in PMC_OSC_EDPD_OVER XOFS field for warmboot */
	val = readl(&pmc->pmc_osc_edpd_over);
	val &= ~PMC_XOFS_MASK;
	val |= (OSC_DRIVE_STRENGTH << PMC_XOFS_SHIFT);
	writel(val, &pmc->pmc_osc_edpd_over);

	/* Set HOLD_CKE_LOW_EN to 1 */
	setbits_le32(&pmc->pmc_cntrl2, HOLD_CKE_LOW_EN);

	debug("Setting up PLLX\n");
	init_pllx();

	val = (1 << CLK_SYS_RATE_AHB_RATE_SHIFT);
	writel(val, &clkrst->crc_clk_sys_rate);

	/* Enable clocks to required peripherals. TBD - minimize this list */
	debug("Enabling clocks\n");

	clock_set_enable(PERIPH_ID_CACHE2, 1);
	clock_set_enable(PERIPH_ID_GPIO, 1);
	clock_set_enable(PERIPH_ID_TMR, 1);
	clock_set_enable(PERIPH_ID_CPU, 1);
	clock_set_enable(PERIPH_ID_EMC, 1);
	clock_set_enable(PERIPH_ID_I2C5, 1);
	clock_set_enable(PERIPH_ID_APBDMA, 1);
	clock_set_enable(PERIPH_ID_MEM, 1);
	clock_set_enable(PERIPH_ID_CORESIGHT, 1);
	clock_set_enable(PERIPH_ID_MSELECT, 1);
	clock_set_enable(PERIPH_ID_DVFS, 1);
	/* HACK for audio - must enable all toys under AHUB! */
	clock_set_enable(PERIPH_ID_AUDIO, 1);
	clock_set_enable(PERIPH_ID_APBIF, 1);
	clock_set_enable(PERIPH_ID_I2S0, 1);
	clock_set_enable(PERIPH_ID_I2S1, 1);
	clock_set_enable(PERIPH_ID_I2S2, 1);
	clock_set_enable(PERIPH_ID_I2S3, 1);
	clock_set_enable(PERIPH_ID_I2S4, 1);
	clock_set_enable(PERIPH_ID_DAM0, 1);
	clock_set_enable(PERIPH_ID_DAM1, 1);
	clock_set_enable(PERIPH_ID_DAM2, 1);
	clock_set_enable(PERIPH_ID_AMX0, 1);
	clock_set_enable(PERIPH_ID_AMX1, 1);
	clock_set_enable(PERIPH_ID_ADX0, 1);
	clock_set_enable(PERIPH_ID_ADX1, 1);
	clock_set_enable(PERIPH_ID_SPDIF, 1);
	/*
	 * Hack this for T124, as these PERIPH_IDs are in the X regs,
	 * which isn't working right yet in the clock_set_ code.
	 * TODO(twarren@nvidia.com): Fix clock_set_enable for X bits.
	 */
	val = readl(&clkrst->crc_clk_out_enb_x);
	val |= (1 << 25) | (1 << 20);	/* enable AMX1, ADX1 */
	writel(val, &clkrst->crc_clk_out_enb_x);

	/*
	 * Set MSELECT clock source as PLLP (00), and ask for a clock
	 * divider that would set the MSELECT clock at 102MHz for a
	 * PLLP base of 408MHz.
	 */
	clock_ll_set_source_divisor(PERIPH_ID_MSELECT, 0,
		CLK_DIVIDER(NVBL_PLLP_KHZ, 102000));

	/* Give clock time to stabilize */
	udelay(IO_STABILIZATION_DELAY);

	/* I2C5 (DVC) gets CLK_M and a divisor of 17 */
	clock_ll_set_source_divisor(PERIPH_ID_I2C5, 3, 16);

	/* Give clock time to stabilize */
	udelay(IO_STABILIZATION_DELAY);

	/* Take required peripherals out of reset */
	debug("Taking periphs out of reset\n");
	reset_set_enable(PERIPH_ID_CACHE2, 0);
	reset_set_enable(PERIPH_ID_GPIO, 0);
	reset_set_enable(PERIPH_ID_TMR, 0);
	reset_set_enable(PERIPH_ID_COP, 0);
	reset_set_enable(PERIPH_ID_EMC, 0);
	reset_set_enable(PERIPH_ID_I2C5, 0);
	reset_set_enable(PERIPH_ID_APBDMA, 0);
	reset_set_enable(PERIPH_ID_MEM, 0);
	reset_set_enable(PERIPH_ID_CORESIGHT, 0);
	reset_set_enable(PERIPH_ID_MSELECT, 0);
	reset_set_enable(PERIPH_ID_DVFS, 0);
	/* HACK for audio */
	reset_set_enable(PERIPH_ID_AUDIO, 0);
	reset_set_enable(PERIPH_ID_APBIF, 0);
	reset_set_enable(PERIPH_ID_I2S0, 0);
	reset_set_enable(PERIPH_ID_I2S1, 0);
	reset_set_enable(PERIPH_ID_I2S2, 0);
	reset_set_enable(PERIPH_ID_I2S3, 0);
	reset_set_enable(PERIPH_ID_I2S4, 0);
	reset_set_enable(PERIPH_ID_DAM0, 0);
	reset_set_enable(PERIPH_ID_DAM1, 0);
	reset_set_enable(PERIPH_ID_DAM2, 0);
	reset_set_enable(PERIPH_ID_AMX0, 0);
	reset_set_enable(PERIPH_ID_AMX1, 0);
	reset_set_enable(PERIPH_ID_ADX0, 0);
	reset_set_enable(PERIPH_ID_ADX1, 0);
	reset_set_enable(PERIPH_ID_SPDIF, 0);
	/*
	 * Hack this for T124, as these PERIPH_IDs are in the X regs,
	 * which isn't working right yet in the reset_set_ code.
	 * TODO(twarren@nvidia.com): Fix reset_set_enable for X bits.
	 */
	val = readl(&clkrst->crc_rst_devices_x);
	/* clear reset for AFC0-5, AMX1, and ADX1 */
	val &= ~((0xFC << 24) | (1 << 25) | (1 << 20));
	writel(val, &clkrst->crc_rst_devices_x);

	debug("t1x4_init_clocks exit\n");
}

static int is_partition_powered(u32 mask)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	/* Get power gate status */
	reg = readl(&pmc->pmc_pwrgate_status);
	return (reg & mask) == mask;
}

static void power_partition(u32 status, u32 partid)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	debug("%s: status = %08X, part ID = %08X\n", __func__, status, partid);
	/* Is the partition already on? */
	if (!is_partition_powered(status)) {
		/* No, toggle the partition power state (OFF -> ON) */
		debug("power_partition, toggling state\n");
		clrbits_le32(&pmc->pmc_pwrgate_toggle, 0x1F);
		setbits_le32(&pmc->pmc_pwrgate_toggle, partid);
		setbits_le32(&pmc->pmc_pwrgate_toggle, START_CP);

		/* Wait for the power to come up */
		while (!is_partition_powered(status))
			;

		/* Give I/O signals time to stabilize */
		udelay(IO_STABILIZATION_DELAY);
	}
}

void powerup_cpus(void)
{
	debug("powerup_cpus entry\n");

	/* We boot to the fast cluster */
	debug("powerup_cpus entry: G cluster\n");

	/* Power up the fast cluster rail partition */
	debug("powerup_cpus: CRAIL\n");
	power_partition(CRAIL, CRAILID);

	/* Power up the fast cluster non-CPU partition */
	debug("powerup_cpus: C0NC\n");
	power_partition(C0NC, C0NCID);

	/* Power up the fast cluster CPU0 partition */
	debug("powerup_cpus: CE0\n");
	power_partition(CE0, CE0ID);

	debug("powerup_cpus: done\n");
}

void start_cpu(u32 reset_vector)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	debug("start_cpu entry, reset_vector = %x\n", reset_vector);

	t1x4_init_clocks();

	t1x4_init_mem_ctlr();

	/* Set power-gating timer multiplier */
	clrbits_le32(&pmc->pmc_pwrgate_timer_mult, TIMER_MULT_MASK);
	setbits_le32(&pmc->pmc_pwrgate_timer_mult, MULT_8);

	/* Enable VDD_CPU */
	enable_cpu_power_rail();

	/* Get the CPU(s) running */
	enable_cpu_clocks();

	/* Enable CoreSight */
	clock_enable_coresight();

	/* Take CPU(s) out of reset */
	remove_cpu_resets();

	/* Set the entry point for CPU execution from reset */
	writel(reset_vector, EXCEP_VECTOR_CPU_RESET_VECTOR);

	/* If the CPU(s) don't already have power, power 'em up */
	powerup_cpus();
	debug("start_cpu exit, should continue @ reset_vector\n");
}

/*
 * On poweron, AVP clock source (also called system clock) is set to PLLP_out0
 * with frequency set at 1MHz. Before initializing PLLP, we need to move the
 * system clock's source to CLK_M temporarily. And then switch it to PLLP_out4
 * (204MHz) at a later time.
 */
void set_avp_clock_to_clkm(void)
{
	struct clk_rst_ctlr *clkrst =
			(struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 val;

	val = (SCLK_SOURCE_CLKM << SCLK_SWAKEUP_FIQ_SOURCE_SHIFT) |
		(SCLK_SOURCE_CLKM << SCLK_SWAKEUP_IRQ_SOURCE_SHIFT) |
		(SCLK_SOURCE_CLKM << SCLK_SWAKEUP_RUN_SOURCE_SHIFT) |
		(SCLK_SOURCE_CLKM << SCLK_SWAKEUP_IDLE_SOURCE_SHIFT) |
		(SCLK_SYS_STATE_RUN << SCLK_SYS_STATE_SHIFT);
	writel(val, &clkrst->crc_sclk_brst_pol);
	/* Wait 2-3us for the clock to flush thru the logic as per the TRM */
	udelay(3);
}
