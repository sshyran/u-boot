/*
* (C) Copyright 2010-2011
* NVIDIA Corporation <www.nvidia.com>
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

#include "ap20.h"
#include <asm/io.h>
#include <asm/arch/tegra2.h>
#include <asm/arch/bitfield.h>
#include <asm/arch/clk_rst.h>
#include <asm/arch/clock.h>
#include <asm/arch/pmc.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/scu.h>
#include <common.h>
#include <asm/arch/warmboot.h>
#include "../../../../../board/nvidia/common/board.h"

struct clk_pll_table {
	u16		n;
	u16		m;
	u8		p;
	u8		cpcon;
};

/* ~0=uninitialized/unknown, 0=false, 1=true */
uint32_t is_tegra2_processor_reset = 0xffffffff;

/*
 * Timing tables for each SOC for all four oscillator options.
 */
static struct clk_pll_table tegra_pll_x_table[TEGRA_SOC_COUNT]
						[CLOCK_OSC_FREQ_COUNT] = {
	/* T20: 1 GHz */
	{{ 1000, 13, 0, 12},	/* OSC 13M */
	 { 625,  12, 0, 8},	/* OSC 19.2M */
	 { 1000, 12, 0, 12},	/* OSC 12M */
	 { 1000, 26, 0, 12},	/* OSC 26M */
	},

	/* T25: 1.2 GHz */
	{{ 923, 10, 0, 12},
	 { 750, 12, 0,  8},
	 { 600,  6, 0, 12},
	 { 600, 13, 0, 12},
	},
};

int ap20_cpu_is_cortexa9(void)
{
	u32 id = readb(NV_PA_PG_UP_BASE + PG_UP_TAG_0);
	return id == (PG_UP_TAG_0_PID_CPU & 0xff);
}

static int pllx_set_rate(u32 divn, u32 divm, u32 divp, u32 cpcon)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	struct clk_pll *pll = &clkrst->crc_pll[CLOCK_ID_XCPU];
	u32 reg;

	/* If PLLX is already enabled, just return */
	if (bf_readl(PLL_ENABLE, &pll->pll_base))
		return 0;

	/* Set BYPASS, m, n and p to PLLX_BASE */
	reg = bf_pack(PLL_BYPASS, 1) | bf_pack(PLL_DIVM, divm);
	reg |= bf_pack(PLL_DIVN, divn) | bf_pack(PLL_DIVP, divp);
	writel(reg, &pll->pll_base);

	/* Set cpcon to PLLX_MISC */
	reg = bf_pack(PLL_CPCON, cpcon);
	writel(reg, &pll->pll_misc);

	/* Enable PLLX */
	reg = readl(&pll->pll_base);
	reg |= bf_pack(PLL_ENABLE, 1);

	/* Disable BYPASS */
	reg &= ~bf_mask(PLL_BYPASS);
	writel(reg, &pll->pll_base);

	return 0;
}

static void init_pllx(void)
{
	int chip_type;
	enum clock_osc_freq osc;
	struct clk_pll_table *sel;

	/* get chip type. If unknown, assign to T20 */
	chip_type = tegra_get_chip_type();
	if (chip_type == TEGRA_SOC_UNKNOWN)
		chip_type = TEGRA_SOC_T20;

	/* get osc freq */
	osc = clock_get_osc_freq();

	/* set pllx */
	sel = &tegra_pll_x_table[chip_type][osc];
	pllx_set_rate(sel->n, sel->m, sel->p, sel->cpcon);
}

static void enable_cpu_clock(int enable)
{
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;
	u32 clk;

	/*
	 * NOTE:
	 * Regardless of whether the request is to enable or disable the CPU
	 * clock, every processor in the CPU complex except the master (CPU 0)
	 * will have it's clock stopped because the AVP only talks to the
	 * master. The AVP does not know (nor does it need to know) that there
	 * are multiple processors in the CPU complex.
	 */

	if (enable) {
		/* Initialize PLLX */
		init_pllx();

		/* Wait until all clocks are stable */
		udelay(PLL_STABILIZATION_DELAY);

		writel(CCLK_BURST_POLICY, &clkrst->crc_cclk_brst_pol);
		writel(SUPER_CCLK_DIVIDER, &clkrst->crc_super_cclk_div);
	}

	/*
	 * Read the register containing the individual CPU clock enables and
	 * always stop the clock to CPU 1.
	 */
	clk = readl(&clkrst->crc_clk_cpu_cmplx);
	clk |= bf_pack(CPU1_CLK_STP, 1);

	/* Stop/Unstop the CPU clock */
	bf_update(CPU0_CLK_STP, clk, enable == 0);
	writel(clk, &clkrst->crc_clk_cpu_cmplx);

	clock_enable(PERIPH_ID_CPU);
}

static int is_cpu_powered(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	return (readl(&pmc->pmc_pwrgate_status) & CPU_PWRED) ? 1 : 0;
}

static void remove_cpu_io_clamps(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	/* Remove the clamps on the CPU I/O signals */
	reg = readl(&pmc->pmc_remove_clamping);
	reg |= CPU_CLMP;
	writel(reg, &pmc->pmc_remove_clamping);

	/* Give I/O signals time to stabilize */
	udelay(IO_STABILIZATION_DELAY);
}

static void powerup_cpu(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;
	int timeout = IO_STABILIZATION_DELAY;

	if (!is_cpu_powered()) {
		/* Toggle the CPU power state (OFF -> ON) */
		reg = readl(&pmc->pmc_pwrgate_toggle);
		reg &= PARTID_CP;
		reg |= START_CP;
		writel(reg, &pmc->pmc_pwrgate_toggle);

		/* Wait for the power to come up */
		while (!is_cpu_powered()) {
			if (timeout-- == 0)
				printf("CPU failed to power up!\n");
			else
				udelay(10);
		}

		/*
		 * Remove the I/O clamps from CPU power partition.
		 * Recommended only on a Warm boot, if the CPU partition gets
		 * power gated. Shouldn't cause any harm when called after a
		 * cold boot according to HW, probably just redundant.
		 */
		remove_cpu_io_clamps();
	}
}

static void enable_cpu_power_rail(void)
{
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	u32 reg;

	reg = readl(&pmc->pmc_cntrl);
	reg |= CPUPWRREQ_OE;
	writel(reg, &pmc->pmc_cntrl);

	/*
	 * The TI PMU65861C needs a 3.75ms delay between enabling
	 * the power rail and enabling the CPU clock.  This delay
	 * between SM1EN and SM1 is for switching time + the ramp
	 * up of the voltage to the CPU (VDD_CPU from PMU). We use 0xf00 as
	 * is is ARM-friendly (can fit in a single ARMv4T mov immmediate
	 * instruction).
	 */
	udelay(3840);
}

static void reset_A9_cpu(int reset)
{
	/*
	* NOTE:  Regardless of whether the request is to hold the CPU in reset
	*        or take it out of reset, every processor in the CPU complex
	*        except the master (CPU 0) will be held in reset because the
	*        AVP only talks to the master. The AVP does not know that there
	*        are multiple processors in the CPU complex.
	*/

	/* Hold CPU 1 in reset, and CPU 0 if asked */
	reset_cmplx_set_enable(1, crc_rst_cpu | crc_rst_de | crc_rst_debug, 1);
	reset_cmplx_set_enable(0, crc_rst_cpu | crc_rst_de | crc_rst_debug,
			       reset);

	/* Enable/Disable master CPU reset */
	reset_set_enable(PERIPH_ID_CPU, reset);
}

static void clock_enable_coresight(int enable)
{
	u32 rst, src;

	clock_set_enable(PERIPH_ID_CORESIGHT, enable);
	reset_set_enable(PERIPH_ID_CORESIGHT, !enable);

	if (enable) {
		/*
		 * Put CoreSight on PLLP_OUT0 (216 MHz) and divide it down by
		 *  1.5, giving an effective frequency of 144MHz.
		 * Set PLLP_OUT0 [bits31:30 = 00], and use a 7.1 divisor
		 *  (bits 7:0), so 00000001b == 1.5 (n+1 + .5)
		 */
		src = CLK_DIVIDER(NVBL_PLLP_KHZ, 144000);
		clock_ll_set_source_divisor(PERIPH_ID_CSI, 0, src);

		/* Unlock the CPU CoreSight interfaces */
		rst = 0xC5ACCE55;
		writel(rst, CSITE_CPU_DBG0_LAR);
		writel(rst, CSITE_CPU_DBG1_LAR);
	}
}

void start_cpu(u32 reset_vector)
{
	/* Enable VDD_CPU */
	enable_cpu_power_rail();

	/* Hold the CPUs in reset */
	reset_A9_cpu(1);

	/* Disable the CPU clock */
	enable_cpu_clock(0);

	/* Enable CoreSight */
	clock_enable_coresight(1);

	/*
	 * Set the entry point for CPU execution from reset,
	 *  if it's a non-zero value.
	 */
	if (reset_vector)
		writel(reset_vector, EXCEP_VECTOR_CPU_RESET_VECTOR);

	/* Enable the CPU clock */
	enable_cpu_clock(1);

	/* If the CPU doesn't already have power, power it up */
	powerup_cpu();

	/* Take the CPU out of reset */
	reset_A9_cpu(0);
}


void halt_avp(void)
{
	for (;;) {
		writel((HALT_COP_EVENT_JTAG | HALT_COP_EVENT_IRQ_1 \
			| HALT_COP_EVENT_FIQ_1 | (FLOW_MODE_STOP<<29)),
			FLOW_CTLR_HALT_COP_EVENTS);
	}
}

void enable_scu(void)
{
	struct scu_ctlr *scu = (struct scu_ctlr *)NV_PA_ARM_PERIPHBASE;
	u32 reg;

	/* If SCU already setup/enabled, return */
	if (readl(&scu->scu_ctrl) & SCU_CTRL_ENABLE)
		return;

	/* Invalidate all ways for all processors */
	writel(0xFFFF, &scu->scu_inv_all);

	/* Enable SCU - bit 0 */
	reg = readl(&scu->scu_ctrl);
	reg |= SCU_CTRL_ENABLE;
	writel(reg, &scu->scu_ctrl);
}

void init_pmc_scratch(void)
{
	struct pmc_ctlr *const pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;
	int i;

	/* SCRATCH0 is initialized by the boot ROM and shouldn't be cleared */
	for (i = 0; i < 23; i++)
		writel(0, &pmc->pmc_scratch1+i);

	/* ODMDATA is for kernel use to determine RAM size, LP config, etc. */
	writel(CONFIG_SYS_BOARD_ODMDATA, &pmc->pmc_scratch20);

#ifdef CONFIG_TEGRA2_LP0
	/* save Sdram params to PMC 2, 4, and 24 for WB0 */
	warmboot_save_sdram_params();
#endif
}

void tegra_start(void)
{
	struct pmux_tri_ctlr *pmt = (struct pmux_tri_ctlr *)NV_PA_APB_MISC_BASE;

	/* If we are the AVP, start up the first Cortex-A9 */
	if (!ap20_cpu_is_cortexa9()) {
		extern void _start(void);

		/* enable JTAG */
		writel(0xC0, &pmt->pmt_cfg_ctl);

		/*
		* If we are ARM7 - give it a different stack. We are about to
		* start up the A9 which will want to use this one.
		*/
		asm volatile("ldr	sp, =%c0\n"
			: : "i"(AVP_EARLY_BOOT_STACK_LIMIT));

		start_cpu((u32)_start);
		halt_avp();
		/* not reached */
	}

	/* Init PMC scratch memory */
	init_pmc_scratch();

	enable_scu();

	/* enable SMP mode and FW for CPU0, by writing to Auxiliary Ctl reg */
	asm volatile(
		"mrc	p15, 0, r0, c1, c0, 1\n"
		"orr	r0, r0, #0x41\n"
		"mcr	p15, 0, r0, c1, c0, 1\n");

	/* FIXME: should have ap20's L2 disabled too? */

	/* Init is_tegra2_processor_reset */
	is_tegra2_processor_reset = check_is_tegra2_processor_reset();
}

