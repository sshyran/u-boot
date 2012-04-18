/*
 * DDR3 mem setup file for SMDK5250 board based on EXYNOS5
 *
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

#include <config.h>
#include <asm/io.h>
#include <asm/arch/dmc.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>

#include "clock_init.h"
#include "setup.h"

#if 0
static void reset_phy_ctrl(void)
{
	struct exynos5_clock *clk = (struct exynos5_clock *)EXYNOS5_CLOCK_BASE;

	writel(LPDDR3PHY_CTRL_PHY_RESET_OFF, &clk->lpddr3phy_ctrl);
	writel(LPDDR3PHY_CTRL_PHY_RESET, &clk->lpddr3phy_ctrl);
}

static void config_ctrl_dll_on(unsigned int state,
			struct exynos5_phy_control *phy0_ctrl,
			struct exynos5_phy_control *phy1_ctrl)
{
	unsigned int val, tmp;

	val = readl(&phy0_ctrl->phy_con13);

	/* Reading ctrl_lock_value[8:2] */
	val &= (NR_DELAY_CELL_COARSE_LOCK_MASK <<
		NR_DELAY_CELL_COARSE_LOCK_OFFSET);

	/* Aligning 'val' to match 'ctrl_force' offset of PHY_CON12 */
	val >>= CTRL_CLOCK_OFFSET;

	/* Setting the PHY_CON12 register */
	tmp = PHY_CON12_RESET_VAL;
	CONFIG_CTRL_DLL_ON(tmp, state);

	/* Writing 'val' in the 'ctrl_force' offset of PHY_CON12 */
	tmp |= val;
	writel(tmp, &phy0_ctrl->phy_con12);

	val = readl(&phy1_ctrl->phy_con13);

	/* Reading ctrl_lock_value[8:2] */
	val &= (NR_DELAY_CELL_COARSE_LOCK_MASK <<
		NR_DELAY_CELL_COARSE_LOCK_OFFSET);

	/* Aligning 'val' to match 'ctrl_force' offset of PHY_CON12 */
	val >>= CTRL_CLOCK_OFFSET;

	/* Setting the PHY_CON12 register */
	tmp = PHY_CON12_RESET_VAL;
	CONFIG_CTRL_DLL_ON(tmp, state);

	/* Writing 'val' in the 'ctrl_force' offset of PHY_CON12 */
	tmp |= val;
	writel(tmp, &phy1_ctrl->phy_con12);
}
#endif

void mem_ctrl_init(void)
{
	struct exynos5_phy_control *phy0_ctrl, *phy1_ctrl;
	struct exynos5_dmc *dmc;
	struct exynos5_clock *clk = (struct exynos5_clock *)EXYNOS5_CLOCK_BASE;

	phy0_ctrl = (struct exynos5_phy_control *)EXYNOS5_DMC_PHY0_BASE;
	phy1_ctrl = (struct exynos5_phy_control *)EXYNOS5_DMC_PHY1_BASE;
	dmc = (struct exynos5_dmc *)EXYNOS5_DMC_CTRL_BASE;

	writel(0x0, &clk->src_cdrex);
	writel(0x01000051, &clk->div_cdrex);
	writel(MPLL_CON1_VAL, &clk->mpll_con1);
	writel(0x80c80300, &clk->mpll_con0);

	sdelay(30000);
	writel(0x100, &clk->src_core1);
	sdelay(30000);

	writel(0x0, &clk->lpddr3phy_ctrl);
	sdelay(1000);
	writel(0x1, &clk->lpddr3phy_ctrl);
	sdelay(1000);

	writel(0x6db, &phy0_ctrl->phy_con39);
	writel(0x6db, &phy1_ctrl->phy_con39);

	writel(0x80b, &phy0_ctrl->phy_con42);
	writel(0x80b, &phy1_ctrl->phy_con42);

	writel(0x0e240304, &phy0_ctrl->phy_con16);
	writel(0x0e240304, &phy1_ctrl->phy_con16);

	writel(0x0e240304, &phy0_ctrl->phy_con16);
	writel(0x0e240304, &phy1_ctrl->phy_con16);

	writel(0x0e240306, &phy0_ctrl->phy_con16);
	writel(0x0e240306, &phy1_ctrl->phy_con16);

	writel(0x0e240304, &phy0_ctrl->phy_con16);
	writel(0x0e240304, &phy1_ctrl->phy_con16);

	writel(0xf, &phy0_ctrl->phy_con14);
	writel(0xf, &phy1_ctrl->phy_con14);

	writel((0x1FFF0000|(0x3<<12)), &dmc->concontrol);

	writel(0xe0000008, &dmc->phycontrol0);
	writel(0xe0000000, &dmc->phycontrol0);

	writel(0x08080808, &phy0_ctrl->phy_con4);
	writel(0x08080808, &phy1_ctrl->phy_con4);

	writel(0x08080808, &phy0_ctrl->phy_con6);
	writel(0x08080808, &phy1_ctrl->phy_con6);

	writel(0x8, &phy0_ctrl->phy_con10);
	writel(0x8, &phy1_ctrl->phy_con10);

	writel(0x10100030, &phy0_ctrl->phy_con12);
	writel(0x10100030, &phy1_ctrl->phy_con12);

	sdelay(1000);

	writel(0x10100070, &phy0_ctrl->phy_con12);
	writel(0x10100070, &phy1_ctrl->phy_con12);

	sdelay(2000);

	writel(0xe0000008, &dmc->phycontrol0);
	writel(0xe0000000, &dmc->phycontrol0);
	writel(0xe0000008, &dmc->phycontrol0);
	writel(0xe0000000, &dmc->phycontrol0);

	writel((0x0FFF0000|(0x3<<12)), &dmc->concontrol);

	writel(0x3, &dmc->ivcontrol);

	writel(0x00001333, &dmc->memconfig0);

	writel(0x00001333, &dmc->memconfig1);

	writel(0x00400780, &dmc->membaseconfig0);

	writel(0x00800780, &dmc->membaseconfig1);

	writel(0xFF000000, &dmc->prechconfig);

	writel(0xFFFF00FF, &dmc->pwrdnconfig);

	writel(0x000000bb, &dmc->timingref);

	writel(0x8C36650E, &dmc->timingrow);

	writel(0x3630580B, &dmc->timingdata);

	writel(0x41000A44, &dmc->timingpower);

	writel(0x01000000, &dmc->directcmd);
	sdelay(100000);

	writel(0x01100000, &dmc->directcmd);
	sdelay(100000);

	writel(0x11000000, &dmc->directcmd);
	sdelay(100000);

	writel(0x11100000, &dmc->directcmd);
	sdelay(100000);

	writel(0x07000000, &dmc->directcmd);
	sdelay(100000);

	writel((0x00020000|0x18), &dmc->directcmd);
	sdelay(100000);

	writel(0x00030000, &dmc->directcmd);
	sdelay(100000);

	writel((0x00010000|0x42), &dmc->directcmd);
	sdelay(100000);

	writel((0x00000000|0xD70), &dmc->directcmd);
	sdelay(100000);

	writel(0x0a000000, &dmc->directcmd);
	sdelay(100000);

	writel(0x17000000, &dmc->directcmd);
	sdelay(100000);

	writel((0x10020000|0x18), &dmc->directcmd);
	sdelay(100000);

	writel(0x10030000, &dmc->directcmd);
	sdelay(100000);

	writel((0x10010000|0x42), &dmc->directcmd);
	sdelay(100000);

	writel((0x10000000|0xD70), &dmc->directcmd);
	sdelay(100000);

	writel(0x1a000000, &dmc->directcmd);
	sdelay(100000);

	writel(0x00302620, &dmc->memcontrol);
	writel((0x0FFF0020|(0x3<<12)), &dmc->concontrol);

}
#if 0
/*==========old one==========-*/

	writel(, &dmc->directcmd);
	sdelay(100000);

	writel(0x1fff2100, &dmc->concontrol);

	writel(0x80000000, &clk->lpddr3phy_ctrl);

	writel(0x1, &phy0_ctrl->phy_con24);
	writel(0x1, &phy1_ctrl->phy_con24);

	writel(0xf, &phy0_ctrl->phy_con14);
	writel(0xf, &phy1_ctrl->phy_con14);

	writel(0x7107f, &phy0_ctrl->phy_con26);
	writel(0x7107f, &phy1_ctrl->phy_con26);

	writel(0x17021A00, &phy0_ctrl->phy_con0);
	writel(0x8080304, &phy0_ctrl->phy_con16);

	writel(0x17021A00, &phy1_ctrl->phy_con0);
	writel(0x8080304, &phy1_ctrl->phy_con16);

	writel(0xe0C0304, &phy0_ctrl->phy_con16);
	writel(0xe0C0306, &phy0_ctrl->phy_con16);
	do {
		val = readl(&phy0_ctrl->phy_con17);
	} while ((val & 0x1) != 1);
	writel(0xe080304, &phy0_ctrl->phy_con16);

	writel(0xe0C0304, &phy1_ctrl->phy_con16);
	writel(0xe0C0306, &phy1_ctrl->phy_con16);
	do {
		val = readl(&phy1_ctrl->phy_con17);
	} while ((val & 0x1) != 1);
	writel(0xe080304, &phy1_ctrl->phy_con16);

	writel(0x0fff2100, &dmc->concontrol);
	writel(0x1fff2100, &dmc->concontrol);
	sdelay(10000);
	writel(0x0fff2100, &dmc->concontrol);

	writel(0x312700, &dmc->memcontrol);
	writel(0x4007c0, &dmc->membaseconfig0);
	writel(0x8007c0, &dmc->membaseconfig1);
	writel(0x1323, &dmc->memconfig0);
	writel(0x1323, &dmc->memconfig0);
	writel(0x7, &dmc->ivcontrol);
	writel(0x5d, &dmc->timingref);
	writel(0x34498692, &dmc->timingrow);
	writel(0x3631D60C, &dmc->timingdata);
	writel(0x50380336, &dmc->timingpower);

	writel(0xfff, &dmc->qoscontrol0);
	writel(0xfff, &dmc->qoscontrol1);
	writel(0xfff, &dmc->qoscontrol2);
	writel(0xfff, &dmc->qoscontrol3);
	writel(0xfff, &dmc->qoscontrol4);
	writel(0xfff, &dmc->qoscontrol5);
	writel(0xfff, &dmc->qoscontrol6);
	writel(0xfff, &dmc->qoscontrol7);
	writel(0xfff, &dmc->qoscontrol8);
	writel(0xfff, &dmc->qoscontrol9);
	writel(0xfff, &dmc->qoscontrol10);
	writel(0xfff, &dmc->qoscontrol11);
	writel(0xfff, &dmc->qoscontrol12);
	writel(0xfff, &dmc->qoscontrol13);
	writel(0xfff, &dmc->qoscontrol14);
	writel(0x0, &dmc->qoscontrol15);

	writel(0x7f7f7f7f, &phy0_ctrl->phy_con4);
	writel(0x7f7f7f7f, &phy1_ctrl->phy_con4);

	writel(0x7f7f7f7f, &phy0_ctrl->phy_con6);
	writel(0x7f7f7f7f, &phy1_ctrl->phy_con6);

	writel(0x7f, &phy0_ctrl->phy_con10);
	writel(0x7f, &phy1_ctrl->phy_con10);

	writel(0x10107f70, &phy0_ctrl->phy_con12);
	writel(0x10107f70, &phy1_ctrl->phy_con12);

	writel(0x10107f50, &phy0_ctrl->phy_con12);
	writel(0x10107f50, &phy1_ctrl->phy_con12);

	writel(0x8, &dmc->phycontrol0);
	writel(0x0, &dmc->phycontrol0);

	/* Channel0 - Chip0 */
	writel(0x7000000, &dmc->directcmd);
	sdelay(1000);
	writel(0x71c00, &dmc->directcmd);
	sdelay(1000);
	writel(0x10bfc, &dmc->directcmd);
	sdelay(1000);
	writel(0x50c, &dmc->directcmd);
	sdelay(1000);
	writel(0x868, &dmc->directcmd);
	sdelay(1000);
	writel(0xc04, &dmc->directcmd);
	sdelay(1000);

	/* Channel0 - Chip1 */
	writel(0x7100000, &dmc->directcmd);
	sdelay(1000);
	writel(0x171c00, &dmc->directcmd);
	sdelay(1000);
	writel(0x110bfc, &dmc->directcmd);
	sdelay(1000);
	writel(0x10050c, &dmc->directcmd);
	sdelay(1000);
	writel(0x100868, &dmc->directcmd);
	sdelay(1000);
	writel(0x100c04, &dmc->directcmd);
	sdelay(1000);

	/* Channel1 - Chip0 */
	writel(0x17000000, &dmc->directcmd);
	sdelay(1000);
	writel(0x10071c00, &dmc->directcmd);
	sdelay(1000);
	writel(0x10010bfc, &dmc->directcmd);
	sdelay(1000);
	writel(0x1000050c, &dmc->directcmd);
	sdelay(1000);
	writel(0x10000868, &dmc->directcmd);
	sdelay(1000);
	writel(0x10000c04, &dmc->directcmd);
	sdelay(1000);

	/* Channel1 - Chip0 */
	writel(0x17100000, &dmc->directcmd);
	sdelay(1000);
	writel(0x10171c00, &dmc->directcmd);
	sdelay(1000);
	writel(0x10110bfc, &dmc->directcmd);
	sdelay(1000);
	writel(0x1010050c, &dmc->directcmd);
	sdelay(1000);
	writel(0x10100868, &dmc->directcmd);
	sdelay(1000);
	writel(0x10100c04, &dmc->directcmd);
	sdelay(1000);

	writel(0x0, &clk->src_cdrex);
	do {
		val = readl(&clk->mux_stat_cdrex);
	} while(val != 0x00211111);

	writel(0x01000051, &clk->div_cdrex);
	do {
		val = readl(&clk->div_stat_cdrex);
	} while(val != 0);

	clrbits_le32(&clk->src_core1, MUX_MPLL_SEL_MASK);
	do {
		val = readl(&clk->mux_stat_core1);
	} while(val != MUX_MPLL_SEL_MASK);

	writel(MPLL_CON1_VAL, &clk->mpll_con1);
	writel(MPLL_CON0_VAL, &clk->mpll_con0);
	sdelay(0x10000);

	clrbits_le32(&clk->src_core1, MUX_MPLL_SEL_MASK);
	do {
		val = readl(&clk->mux_stat_core1);
	} while(val != MPLL_SEL_MOUT_MPLLFOUT);

	writel(0x08080808, &phy0_ctrl->phy_con4);
	writel(0x08080808, &phy1_ctrl->phy_con4);

	writel(0x08080808, &phy0_ctrl->phy_con6);
	writel(0x08080808, &phy1_ctrl->phy_con6);

	writel(0x08, &phy0_ctrl->phy_con10);
	writel(0x08, &phy1_ctrl->phy_con10);

	writel(0x10107f70, &phy0_ctrl->phy_con12);
	sdelay(1000);
	writel(0x10107f30, &phy0_ctrl->phy_con12);
	writel(0x10107f70, &phy0_ctrl->phy_con12);
	sdelay(1000);

	writel(0x10107f70, &phy1_ctrl->phy_con12);
	sdelay(1000);
	writel(0x10107f30, &phy1_ctrl->phy_con12);
	writel(0x10107f70, &phy1_ctrl->phy_con12);
	sdelay(1000);

	writel(0x1fff2100, &dmc->concontrol);
	do {
		val = readl(&dmc->phystatus);
	} while ((val & 0xc) != 0xc);
	writel(0x0fff2100, &dmc->concontrol);

	writel(0x8, &dmc->phycontrol0);
	writel(0x0, &dmc->phycontrol0);

	writel(0x17021A00, &phy0_ctrl->phy_con0);
	writel(0xe080304, &phy0_ctrl->phy_con16);

	writel(0x17021A00, &phy1_ctrl->phy_con0);
	writel(0xe080304, &phy1_ctrl->phy_con16);

	writel(0x1000000, &dmc->directcmd);
	writel(0x1100000, &dmc->directcmd);
	writel(0x11000000, &dmc->directcmd);
	writel(0x11100000, &dmc->directcmd);

	writel(0x312710, &dmc->memcontrol);
	writel(0xff000000, &dmc->prechconfig);
	writel(0xffff00ff, &dmc->pwrdnconfig);
	val = readl(&dmc->memcontrol);
	val |= 0x20;
	writel(val, &dmc->memcontrol);
	writel(0xffff00ff, &dmc->pwrdnconfig);
	val = readl(&dmc->memcontrol);
	val |= 0x2;
	writel(val, &dmc->memcontrol);

	setbits_le32(&dmc->memcontrol, 0x1);

	writel(0xffff2108, &dmc->concontrol);
	writel(0x1fff2128, &dmc->concontrol);
#endif
