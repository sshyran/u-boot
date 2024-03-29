/*
 * Machine Specific Values for SMDK5250 board based on Exynos5
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

#ifndef _SMDK5250_SETUP_H
#define _SMDK5250_SETUP_H

#include <config.h>
#include <version.h>
#include <asm/arch/cpu.h>
#include <asm/arch/dmc.h>

/* GPIO Offsets for UART: GPIO Contol Register */
#define EXYNOS5_GPIO_A0_CON_OFFSET	0x0
#define EXYNOS5_GPIO_A1_CON_OFFSET	0x20

/* TZPC : Register Offsets */
#define TZPC0_BASE		0x10100000
#define TZPC1_BASE		0x10110000
#define TZPC2_BASE		0x10120000
#define TZPC3_BASE		0x10130000
#define TZPC4_BASE		0x10140000
#define TZPC5_BASE		0x10150000
#define TZPC6_BASE		0x10160000
#define TZPC7_BASE		0x10170000
#define TZPC8_BASE		0x10180000
#define TZPC9_BASE		0x10190000

/* CLK_SRC_CPU */
/* 0 = MOUTAPLL, 1 = SCLKMPLL */
#define MUX_HPM_SEL		0
#define MUX_CPU_SEL		0
#define MUX_APLL_SEL		1
#define CLK_SRC_CPU_VAL		((MUX_HPM_SEL << 20) \
				| (MUX_CPU_SEL << 16) \
				| (MUX_APLL_SEL))

/* CLK_DIV_CPU0 */
#define ARM2_RATIO		0x0
#define APLL_RATIO		0x1
#define PCLK_DBG_RATIO		0x1
#define ATB_RATIO		0x4
#define PERIPH_RATIO		0x7
#define ACP_RATIO		0x7
#define CPUD_RATIO		0x2
#define ARM_RATIO		0x0
#define CLK_DIV_CPU0_VAL	((ARM2_RATIO << 28) \
				| (APLL_RATIO << 24) \
				| (PCLK_DBG_RATIO << 20) \
				| (ATB_RATIO << 16) \
				| (PERIPH_RATIO << 12) \
				| (ACP_RATIO << 8) \
				| (CPUD_RATIO << 4) \
				| (ARM_RATIO))

/* CLK_DIV_CPU1 */
#define HPM_RATIO		0x4
#define COPY_RATIO		0x0
#define CLK_DIV_CPU1_VAL	((HPM_RATIO << 4) \
				| (COPY_RATIO))

#define APLL_MDIV		0x7D
#define APLL_PDIV		0x3
#define APLL_SDIV		0x0

#define MPLL_MDIV		0x64
#define MPLL_PDIV		0x3
#define MPLL_SDIV		0x0

#define CPLL_MDIV		0x96
#define CPLL_PDIV		0x4
#define CPLL_SDIV		0x0

/* APLL_CON1 */
#define APLL_CON1_VAL	(0x00203800)

/* MPLL_CON1 */
#define MPLL_CON1_VAL	(0x00203800)

/* CPLL_CON1 */
#define CPLL_CON1_VAL	(0x00203800)

#define EPLL_MDIV	0x60
#define EPLL_PDIV	0x3
#define EPLL_SDIV	0x3

#define EPLL_CON1_VAL	0x00000000
#define EPLL_CON2_VAL	0x00000080

#define VPLL_MDIV	0x96
#define VPLL_PDIV	0x3
#define VPLL_SDIV	0x2

#define VPLL_CON1_VAL	0x00000000
#define VPLL_CON2_VAL	0x00000080

#define BPLL_CON1_VAL	0x00203800

/* Set PLL */
#define set_pll(mdiv, pdiv, sdiv)	(1<<31 | mdiv<<16 | pdiv<<8 | sdiv)

#define APLL_CON0_VAL	set_pll(APLL_MDIV, APLL_PDIV, APLL_SDIV)
#define MPLL_CON0_VAL	set_pll(MPLL_MDIV, MPLL_PDIV, MPLL_SDIV)
#define CPLL_CON0_VAL	set_pll(CPLL_MDIV, CPLL_PDIV, CPLL_SDIV)
#define EPLL_CON0_VAL	set_pll(EPLL_MDIV, EPLL_PDIV, EPLL_SDIV)
#define VPLL_CON0_VAL	set_pll(VPLL_MDIV, VPLL_PDIV, VPLL_SDIV)

/* CLK_SRC_CORE0 */
#define CLK_SRC_CORE0_VAL	0x00060000

/* CLK_SRC_CORE1 */
#define CLK_SRC_CORE1_VAL	0x100

/* CLK_DIV_CORE0 */
#define CLK_DIV_CORE0_VAL	0x120000

/* CLK_DIV_CORE1 */
#define CLK_DIV_CORE1_VAL	0x07070700

/* CLK_SRC_CDREX */
#define CLK_SRC_CDREX_INIT_VAL	0x1
#define CLK_SRC_CDREX_VAL	0x11111

/* CLK_DIV_CDREX */
#define CLK_DIV_CDREX_INIT_VAL	0x71771111

#define MCLK_CDREX2_RATIO	0x0
#define ACLK_EFCON_RATIO	0x1
#define MCLK_DPHY_RATIO		0x0
#define MCLK_CDREX_RATIO	0x0
#define ACLK_C2C_200_RATIO	0x1
#define C2C_CLK_400_RATIO	0x1
#define ACLK_CDREX_RATIO	0x1

#define MCLK_EFPHY_RATIO	0x4
#define CLK_DIV_CDREX2_VAL	MCLK_EFPHY_RATIO

/* CLK_DIV_ACP */
#define CLK_DIV_ACP_VAL		0x12

/* CLK_SRC_TOP0 */
#define MUX_ACLK_300_GSCL_SEL		0x1
#define MUX_ACLK_300_GSCL_MID_SEL	0x0
#define MUX_ACLK_400_SEL		0x0
#define MUX_ACLK_333_SEL		0x0
#define MUX_ACLK_300_DISP1_SEL		0x1
#define MUX_ACLK_300_DISP1_MID_SEL	0x0
#define MUX_ACLK_200_SEL		0x0
#define MUX_ACLK_166_SEL		0x0
#define CLK_SRC_TOP0_VAL	((MUX_ACLK_300_GSCL_SEL << 25) \
				| (MUX_ACLK_300_GSCL_MID_SEL << 24) \
				| (MUX_ACLK_400_SEL << 20) \
				| (MUX_ACLK_333_SEL << 16) \
				| (MUX_ACLK_300_DISP1_SEL << 15) \
				| (MUX_ACLK_300_DISP1_MID_SEL << 14)	\
				| (MUX_ACLK_200_SEL << 12) \
				| (MUX_ACLK_166_SEL << 8))

/* CLK_SRC_TOP1 */
#define MUX_ACLK_400_ISP_SEL		0x0
#define MUX_ACLK_400_IOP_SEL		0x0
#define MUX_ACLK_MIPI_HSI_TXBASE_SEL	0x0
#define CLK_SRC_TOP1_VAL		((MUX_ACLK_400_ISP_SEL << 24) \
					|(MUX_ACLK_400_IOP_SEL << 20) \
					|(MUX_ACLK_MIPI_HSI_TXBASE_SEL << 16))

/* CLK_SRC_TOP2 */
#define MUX_BPLL_USER_SEL	0x1
#define MUX_MPLL_USER_SEL	0x1
#define MUX_VPLL_SEL		0x1
#define MUX_EPLL_SEL		0x0
#define MUX_CPLL_SEL		0x0
#define VPLLSRC_SEL		0x0
#define CLK_SRC_TOP2_VAL	((MUX_BPLL_USER_SEL << 24) \
				| (MUX_MPLL_USER_SEL << 20) \
				| (MUX_VPLL_SEL << 16) \
				| (MUX_EPLL_SEL << 12) \
				| (MUX_CPLL_SEL << 8) \
				| (VPLLSRC_SEL))
/* CLK_SRC_TOP3 */
#define MUX_ACLK_333_SUB_SEL		0x1
#define MUX_ACLK_400_SUB_SEL		0x1
#define MUX_ACLK_266_ISP_SUB_SEL	0x1
#define MUX_ACLK_266_GPS_SUB_SEL	0x1
#define MUX_ACLK_300_GSCL_SUB_SEL	0x1
#define MUX_ACLK_266_GSCL_SUB_SEL	0x1
#define MUX_ACLK_300_DISP1_SUB_SEL	0x1
#define MUX_ACLK_200_DISP1_SUB_SEL	0x1
#define CLK_SRC_TOP3_VAL		((MUX_ACLK_333_SUB_SEL << 24) \
					| (MUX_ACLK_400_SUB_SEL << 20) \
					| (MUX_ACLK_266_ISP_SUB_SEL << 16) \
					| (MUX_ACLK_266_GPS_SUB_SEL << 12) \
					| (MUX_ACLK_300_GSCL_SUB_SEL << 10) \
					| (MUX_ACLK_266_GSCL_SUB_SEL << 8) \
					| (MUX_ACLK_300_DISP1_SUB_SEL << 6) \
					| (MUX_ACLK_200_DISP1_SUB_SEL << 4))

/* CLK_DIV_TOP0 */
#define ACLK_300_RATIO		0x0
#define ACLK_400_RATIO		0x3
#define ACLK_333_RATIO		0x2
#define ACLK_266_RATIO		0x2
#define ACLK_200_RATIO		0x3
#define ACLK_166_RATIO		0x5
#define ACLK_133_RATIO		0x1
#define ACLK_66_RATIO		0x5
#define CLK_DIV_TOP0_VAL	((ACLK_300_RATIO << 28) \
				| (ACLK_400_RATIO << 24) \
				| (ACLK_333_RATIO << 20) \
				| (ACLK_266_RATIO << 16) \
				| (ACLK_200_RATIO << 12) \
				| (ACLK_166_RATIO << 8) \
				| (ACLK_133_RATIO << 4) \
				| (ACLK_66_RATIO))

/* CLK_DIV_TOP1 */
#define ACLK_MIPI_HSI_TX_BASE_RATIO	0x3
#define ACLK_66_PRE_RATIO	0x1
#define ACLK_400_ISP_RATIO	0x1
#define ACLK_400_IOP_RATIO	0x1
#define ACLK_300_GSCL_RATIO	0x0
#define ACLK_266_GPS_RATIO	0x7

#define CLK_DIV_TOP1_VAL	((ACLK_MIPI_HSI_TX_BASE_RATIO << 28) \
				| (ACLK_66_PRE_RATIO << 24) \
				| (ACLK_400_ISP_RATIO << 20) \
				| (ACLK_400_IOP_RATIO << 16) \
				| (ACLK_300_GSCL_RATIO << 12) \
				| (ACLK_266_GPS_RATIO << 8))

/* APLL_LOCK */
#define APLL_LOCK_VAL		(0x3E8)
/* MPLL_LOCK */
#define MPLL_LOCK_VAL		(0x2F1)
/* CPLL_LOCK */
#define CPLL_LOCK_VAL		(0x3E8)
/* EPLL_LOCK */
#define EPLL_LOCK_VAL		(0x2321)
/* VPLL_LOCK */
#define VPLL_LOCK_VAL		(0x2321)
/* BPLL_LOCK */
#define BPLL_LOCK_VAL		(0x3E8)

/* CLK_SRC_PERIC0 */
/* SRC_CLOCK = SCLK_MPLL */
#define PWM_SEL			0
#define UART4_SEL		6
#define UART3_SEL		6
#define UART2_SEL		6
#define UART1_SEL		6
#define UART0_SEL		6
#define CLK_SRC_PERIC0_VAL	((PWM_SEL << 24) \
				| (UART4_SEL << 16) \
				| (UART3_SEL << 12) \
				| (UART2_SEL << 8) \
				| (UART1_SEL << 4) \
				| (UART0_SEL << 0))

/* CLK_SRC_PERIC1 */
/* SRC_CLOCK = SCLK_MPLL */
#define SPI0_SEL		6
#define SPI1_SEL		6
#define SPI2_SEL		6
#define CLK_SRC_PERIC1_VAL	((SPI2_SEL << 24) \
				| (SPI1_SEL << 20) \
				| (SPI0_SEL << 16))

/* SCLK_SRC_ISP - set SPI0/1 to 6 = SCLK_MPLL_USER */
#define SPI0_ISP_SEL		6
#define SPI1_ISP_SEL		6
#define SCLK_SRC_ISP_VAL	(SPI1_ISP_SEL << 4) \
				| (SPI0_ISP_SEL << 0)

/* SCLK_DIV_ISP - set SPI0/1 to 0xf = divide by 16 */
#define SPI0_ISP_RATIO		0xf
#define SPI1_ISP_RATIO		0xf
#define SCLK_DIV_ISP_VAL	(SPI1_ISP_RATIO << 12) \
				| (SPI0_ISP_RATIO << 0)

#define CLK_SRC_FSYS_VAL	0x66666
#define CLK_DIV_FSYS0_VAL	0x0BB00000
#define CLK_DIV_FSYS1_VAL	0x000f000f
#define CLK_DIV_FSYS2_VAL	0x020f020f
#define CLK_DIV_FSYS3_VAL	0x000f

/* CLK_DIV_PERIC0 */
#define UART5_RATIO		8
#define UART4_RATIO		8
#define UART3_RATIO		8
#define UART2_RATIO		8
#define UART1_RATIO		8
#define UART0_RATIO		8
#define CLK_DIV_PERIC0_VAL	((UART4_RATIO << 16) \
				| (UART3_RATIO << 12) \
				| (UART2_RATIO << 8) \
				| (UART1_RATIO << 4) \
				| (UART0_RATIO << 0))

/* CLK_DIV_PERIC1 */
#define SPI1_RATIO		0xf
#define SPI0_RATIO		0xf
#define SPI1_SUB_RATIO		0x0
#define SPI0_SUB_RATIO		0x0
#define CLK_DIV_PERIC1_VAL	((SPI1_SUB_RATIO << 24) \
				| ((SPI1_RATIO << 16) \
				| (SPI0_SUB_RATIO << 8) \
				| (SPI0_RATIO << 0)))

/* CLK_DIV_PERIC2 */
#define SPI2_RATIO		0xf
#define SPI2_SUB_RATIO		0x0
#define CLK_DIV_PERIC2_VAL	((SPI2_SUB_RATIO << 8) \
				| (SPI2_RATIO << 0))

/* CLK_DIV_PERIC3 */
#define PWM_RATIO		8
#define CLK_DIV_PERIC3_VAL	(PWM_RATIO << 0)

/* CLK_SRC_LEX */
#define CLK_SRC_LEX_VAL		0x0

/* CLK_DIV_LEX */
#define CLK_DIV_LEX_VAL		0x10

/* CLK_DIV_R0X */
#define CLK_DIV_R0X_VAL		0x10

/* CLK_DIV_L0X */
#define CLK_DIV_R1X_VAL		0x10

/* CLK_DIV_ISP0 */
#define CLK_DIV_ISP0_VAL	0x31

/* CLK_DIV_ISP1 */
#define CLK_DIV_ISP1_VAL	0x0

/* CLK_DIV_ISP2 */
#define CLK_DIV_ISP2_VAL	0x1

/* CLK_SRC_DISP1_0 */
#define CLK_SRC_DISP1_0_VAL	0x6

#define MPLL_DEC	(MPLL_MDIV * MPLL_MDIV / (MPLL_PDIV * 2^(MPLL_SDIV-1)))

/*
 * TZPC Register Value :
 * R0SIZE: 0x0 : Size of secured ram
 */
#define R0SIZE			0x0

/*
 * TZPC Decode Protection Register Value :
 * DECPROTXSET: 0xFF : Set Decode region to non-secure
 */
#define DECPROTXSET		0xFF

#define LPDDR3PHY_CTRL_PHY_RESET	(1 << 0)
#define LPDDR3PHY_CTRL_PHY_RESET_OFF	(0 << 0)

/*ZQ Configurations */
#define PHY_CON16_RESET_VAL	0x08000304

#define ZQ_CLK_DIV_EN		(1 << 18)
#define ZQ_MANUAL_STR		(1 << 1)

/* Direct Command */
#define DIRECT_CMD_NOP			0x07000000
#define DIRECT_CMD_PALL			0x01000000
#define DIRECT_CMD_ZQINIT		0x0a000000
#define DIRECT_CMD_CHANNEL_SHIFT	28
#define DIRECT_CMD_CHIP_SHIFT		20

/* DMC PHY Control0 register */
#define PHY_CONTROL0_RESET_VAL	0x0
#define MEM_TERM_EN	(1 << 31)	/* Termination enable for memory */
#define PHY_TERM_EN	(1 << 30)	/* Termination enable for PHY */
#define CTRL_SHGATE	(1 << 29)	/* Duration of DQS gating signal */
#define FP_RSYNC	(1 << 3)	/* Force DLL resyncronization */

/* MDLL control */
#define PHY_CON12_RESET_VAL		0x10100070

#define NR_DELAY_CELL_COARSE_LOCK_OFFSET	10
#define NR_DELAY_CELL_COARSE_LOCK_MASK		0x7F
#define CTRL_CLOCK_OFFSET			2

/* PHY Control */
#define PHY_CON0_RESET_VAL	0x17020A40
#define BYTE_RDLVL_EN		(1 << 13)
#define CTRL_ATGATE		(1 << 6)
/*
 * TODO(clchiou): Remove these SET_ functions and put the actual code in
 * the C file.
 */
#define SET_CTRL_DDR_MODE(x, y)		(x = (x & ~(0x3 << 11)) | y << 11)

#define PHY_CON1_RESET_VAL	0x9210000

#define DDR3_ADDR		0x0208

#define PHY_CON2_RESET_VAL	0x00010004
#define RDLVL_EN		(1 << 25)
#define RDDSKEW_CLEAR		(1 << 13)

#define RDLVL_CONFIG_RESET_VAL	0x0
#define CTRL_RDLVL_DATA_EN	(1 << 1)
#define LPDDR2_ADDR		0x00000208

/* MEMCONTROL register bit fields */
#define DMC_MEMCONTROL_CLK_STOP_DISABLE	(0 << 0)
#define DMC_MEMCONTROL_DPWRDN_DISABLE	(0 << 1)
#define DMC_MEMCONTROL_DPWRDN_ACTIVE_PRECHARGE	(0 << 2)
#define DMC_MEMCONTROL_TP_DISABLE	(0 << 4)
#define DMC_MEMCONTROL_DSREF_DISABLE	(0 << 5)
#define DMC_MEMCONTROL_ADD_LAT_PALL_CYCLE(x)	(x << 6)

#define DMC_MEMCONTROL_MEM_TYPE_LPDDR3	(7 << 8)
#define DMC_MEMCONTROL_MEM_TYPE_DDR3	(6 << 8)
#define DMC_MEMCONTROL_MEM_TYPE_LPDDR2	(5 << 8)

#define DMC_MEMCONTROL_MEM_WIDTH_32BIT	(2 << 12)

#define DMC_MEMCONTROL_NUM_CHIP_1	(0 << 16)
#define DMC_MEMCONTROL_NUM_CHIP_2	(1 << 16)

#define DMC_MEMCONTROL_BL_8		(3 << 20)
#define DMC_MEMCONTROL_BL_4		(2 << 20)

#define DMC_MEMCONTROL_PZQ_DISABLE	(0 << 24)

#define DMC_MEMCONTROL_MRR_BYTE_7_0	(0 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_15_8	(1 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_23_16	(2 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_31_24	(3 << 25)

/* MEMCONFIG0 register bit fields */
#define DMC_MEMCONFIGx_CHIP_MAP_INTERLEAVED	(1 << 12)
#define DMC_MEMCONFIGx_CHIP_COL_10		(3 << 8)
#define DMC_MEMCONFIGx_CHIP_ROW_14		(2 << 4)
#define DMC_MEMCONFIGx_CHIP_ROW_15		(3 << 4)
#define DMC_MEMCONFIGx_CHIP_BANK_8		(3 << 0)


#define DMC_MEMBASECONFIGx_CHIP_BASE(x)		(x << 16)
#define DMC_MEMBASECONFIGx_CHIP_MASK(x)		(x << 0)
#define DMC_MEMBASECONFIG_VAL(x)	(	\
	DMC_MEMBASECONFIGx_CHIP_BASE(x) |	\
	DMC_MEMBASECONFIGx_CHIP_MASK(0x780)	\
)

#define DMC_MEMBASECONFIG0_VAL	DMC_MEMBASECONFIG_VAL(0x40)
#define DMC_MEMBASECONFIG1_VAL	DMC_MEMBASECONFIG_VAL(0x80)

#define DMC_PRECHCONFIG_VAL		0xFF000000
#define DMC_PWRDNCONFIG_VAL		0xFFFF00FF

#define SET_DQS_OFFSET_VAL	0x7F7F7F7F
#define SET_DQ_OFFSET_VAL	0x7F7F7F7F
#define SET_DEBUG_OFFSET_VAL	0x7F

#define RESET_DQS_OFFSET_VAL	0x08080808
#define RESET_DQ_OFFSET_VAL	0x08080808
#define RESET_DEBUG_OFFSET_VAL	0x8

#define CTRL_PULLD_DQ		(0x0F << 8)
#define CTRL_PULLD_DQS		(0x0F << 0)

#define DMC_CONCONTROL_RESET_VAL	0x0FFF0000
#define DFI_INIT_START		(1 << 28)
#define EMPTY			(1 << 8)
#define AREF_EN			(1 << 5)

#define DFI_INIT_COMPLETE_CHO	(1 << 2)
#define DFI_INIT_COMPLETE_CH1	(1 << 3)

#define RDLVL_COMPLETE_CHO	(1 << 14)
#define RDLVL_COMPLETE_CH1	(1 << 15)

#define CLK_STOP_EN	(1 << 0)
#define DPWRDN_EN	(1 << 1)
#define DSREF_EN	(1 << 5)

struct mem_timings;

/* Functions common between LPDDR2 and DDR3 */
void sdelay(unsigned long);
void mem_ctrl_init(void);
/*
 * Memory variant specific initialization code
 *
 * @param mem		Memory timings for this memory type.
 * @param mem_iv_size	Memory interleaving size is a configurable parameter
 *			which the DMC uses to decide how to split a memory
 *			chunk into smaller chunks to support concurrent
 *			accesses; may vary across boards.
 */
void ddr3_mem_ctrl_init(struct mem_timings *mem, unsigned long mem_iv_size);
void lpddr2_mem_ctrl_init(struct mem_timings *mem, unsigned long mem_iv_size);

void system_clock_init(void);
void mem_clk_setup(void);
void tzpc_init(void);
/*
 * Configure ZQ I/O interface
 *
 * @param mem		Memory timings for this memory type.
 * @param phy0_ctrl	Pointer to struct containing PHY0 control reg
 * @param phy1_ctrl	Pointer to struct containing PHY1 control reg
 */
void dmc_config_zq(struct mem_timings *mem,
		   struct exynos5_phy_control *phy0_ctrl,
		   struct exynos5_phy_control *phy1_ctrl);

/*
 * Send NOP and MRS/EMRS Direct commands
 *
 * @param mem		Memory timings for this memory type.
 * @param dmc		Pointer to struct of DMC registers
 */
void dmc_config_mrs(struct mem_timings *mem, struct exynos5_dmc *dmc);

/*
 * Send PALL Direct commands
 *
 * @param mem		Memory timings for this memory type.
 * @param dmc		Pointer to struct of DMC registers
 */
void dmc_config_prech(struct mem_timings *mem, struct exynos5_dmc *dmc);

/*
 * Configure the memconfig and membaseconfig registers
 *
 * @param mem		Memory timings for this memory type.
 * @param exynos5_dmc	Pointer to struct of DMC registers
 */
void dmc_config_memory(struct mem_timings *mem, struct exynos5_dmc *dmc);

/* Set the PS-Hold drive value */
void ps_hold_setup(void);
/*
 * Reset the DLL. This function is common between DDR3 and LPDDR2.
 * However, the reset value is different. So we are passing a flag
 * ddr_mode to distinguish between LPDDR2 and DDR3.
 *
 * @param exynos5_dmc	Pointer to struct of DMC registers
 * @param ddr_mode	Type of DDR memory
 */
void update_reset_dll(struct exynos5_dmc *, enum ddr_mode);
#endif
