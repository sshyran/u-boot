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

void mem_ctrl_init(void)
{
	struct exynos5_phy_control *phy0_ctrl, *phy1_ctrl;
	struct exynos5_dmc *dmc;
	struct exynos5_clock *clk = (struct exynos5_clock *)EXYNOS5_CLOCK_BASE;
	unsigned int val;

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

	writel(0xb6d, &phy0_ctrl->phy_con39);
	writel(0xb6d, &phy1_ctrl->phy_con39);

	writel(0x80b, &phy0_ctrl->phy_con42);
	writel(0x80b, &phy1_ctrl->phy_con42);

	writel(0xD2C0304, &phy0_ctrl->phy_con16);
	writel(0xD2C0304, &phy1_ctrl->phy_con16);

	writel(0xD2C0304, &phy0_ctrl->phy_con16);
	writel(0xD2C0304, &phy1_ctrl->phy_con16);

	writel(0xD2C0306, &phy0_ctrl->phy_con16);
	writel(0xD2C0306, &phy1_ctrl->phy_con16);

	do {
		val = readl(0x10c00048);
	} while (val & 0x1 != 1);

	do {
		val = readl(0x10c10048);
	} while (val & 0x1 != 1);

	writel(0xD2C0304, &phy0_ctrl->phy_con16);
	writel(0xD2C0304, &phy1_ctrl->phy_con16);

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

	writel(0x7, &dmc->ivcontrol);

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

	writel((0x00010000|0x0), &dmc->directcmd);
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

	writel((0x10010000|0x0), &dmc->directcmd);
	sdelay(100000);

	writel((0x10000000|0xD70), &dmc->directcmd);
	sdelay(100000);

	writel(0x1a000000, &dmc->directcmd);
	sdelay(100000);

	writel(0x17020a40, 0x10C00000);
	writel(0x17020a40, 0x10C10000);
	writel(0x17024a40, 0x10C00000);
	writel(0x17024a40, 0x10C10000);
	writel(0x10044, 0x10C00008);
	writel(0x10044, 0x10C10008);
	writel(0x17026a40, 0x10C00000);
	writel(0x17026a40, 0x10C10000);
	writel(0x10101a50, 0x10C00030);
	writel(0x10101a50, 0x10C10030);
	writel(0x1010044, 0x10C00008);
	writel(0x1010044, 0x10C10008);
	writel(0x17026b40, 0x10C00000);
	writel(0x17026b40, 0x10C10000);
	writel(0x9010100, 0x10C00004);
	writel(0x9010100, 0x10C10004);
	writel(0x00000001, 0x10DD00f8);
	sdelay(2000);

	do {
		val = readl(0x10DD0040);
	} while (val & 0xC000 != 0xC000);

	writel(0x00000000, 0x10DD00f8);
	writel(0x00000000, 0x10C00038);
	writel(0x00000000, 0x10C10038);
	writel(0x10101a70, 0x10C00030);
	writel(0x10101a70, 0x10C10030);
	sdelay(1000);

	writel(0xe0000008, 0x10DD0018);
	writel(0xe0000000, 0x10DD0018);
	writel(0x01000000, 0x10DD0010);
	sdelay(1000);

	writel(0x01100000, 0x10DD0010);
	sdelay(1000);

	writel(0x11000000, 0x10DD0010);
	sdelay(1000);

	writel(0x11100000, 0x10DD0010);

	writel(0x00302620, &dmc->memcontrol);
	writel((0x0FFF0020|(0x3<<12)), &dmc->concontrol);
}
