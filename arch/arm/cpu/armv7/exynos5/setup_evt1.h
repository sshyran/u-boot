/*
 * (C) Copyright 2011 Samsung Electronics Co. Ltd
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _VAL_SMDK5250_H
#define _VAL_SMDK5250_H

#include <config.h>
#include <version.h>

#include <asm/arch/cpu.h>

#if defined(CONFIG_CLK_ARM_600_APLL_600)

#define APLL_MDIV	0xC8
#define APLL_PDIV	0x4
#define APLL_SDIV	0x1

#elif defined(CONFIG_CLK_ARM_800_APLL_800)

#define APLL_MDIV	0x64
#define APLL_PDIV	0x3
#define APLL_SDIV	0x0

#elif defined(CONFIG_CLK_ARM_1000_APLL_1000)

#define APLL_MDIV	0x7D
#define APLL_PDIV	0x3
#define APLL_SDIV	0x0

#elif defined(CONFIG_CLK_ARM_1200_APLL_1200)

#define APLL_MDIV	0x96
#define APLL_PDIV	0x3
#define APLL_SDIV	0x0

#elif defined(CONFIG_CLK_ARM_1400_APLL_1400)

#define APLL_MDIV	0xAF
#define APLL_PDIV	0x3
#define APLL_SDIV	0x0

#elif defined(CONFIG_CLK_ARM_1700_APLL_1700)

#define APLL_MDIV	0x1A9
#define APLL_PDIV	0x6
#define APLL_SDIV	0x0
#endif

/*#define MPLL_MDIV	0x64 */
/*#define MPLL_PDIV	0x3 */
/*#define MPLL_SDIV	0x0 */

#define MPLL_MDIV	0xC8
#define MPLL_PDIV	0x6
#define MPLL_SDIV	0x1

#define CPLL_MDIV	0xDE
#define CPLL_PDIV	0x4
#define CPLL_SDIV	0x2

#define GPLL_MDIV	0x215
#define GPLL_PDIV	0xC
#define GPLL_SDIV	0x1

/* APLL_CON1	*/
#define APLL_CON1_VAL	(0x00203800)

/* MPLL_CON1	*/
#define MPLL_CON1_VAL   (0x00203800)

/* CPLL_CON1	*/
#define CPLL_CON1_VAL	(0x00203800)

/* GPLL_CON1	*/
#define GPLL_CON1_VAL	(0x00203800)

#define EPLL_MDIV	0x60
#define EPLL_PDIV	0x3
#define EPLL_SDIV	0x3

#define EPLL_CON1_VAL	0x00000000
#define EPLL_CON2_VAL	0x00000080

#define VPLL_MDIV	0x96
#define VPLL_PDIV	0x3
#define VPLL_SDIV	0x2

/* VPLL_CON1, CON2	*/
#define VPLL_CON1_VAL	0x00000000
#define VPLL_CON2_VAL	0x00000080

#define BPLL_MDIV	0x64
#define BPLL_PDIV	0x3
#define BPLL_SDIV	0x0

/* BPLL_CON1	*/
#define BPLL_CON1_VAL	0x00203800

/* Set PLL */
#define set_pll(mdiv, pdiv, sdiv)	(1<<31 | mdiv<<16 | pdiv<<8 | sdiv)

#define APLL_CON0_VAL	set_pll(APLL_MDIV, APLL_PDIV, APLL_SDIV)
#define MPLL_CON0_VAL	set_pll(MPLL_MDIV, MPLL_PDIV, MPLL_SDIV)
#define CPLL_CON0_VAL	set_pll(CPLL_MDIV, CPLL_PDIV, CPLL_SDIV)
#define GPLL_CON0_VAL	set_pll(GPLL_MDIV, GPLL_PDIV, GPLL_SDIV)
#define EPLL_CON0_VAL	set_pll(EPLL_MDIV, EPLL_PDIV, EPLL_SDIV)
#define VPLL_CON0_VAL	set_pll(VPLL_MDIV, VPLL_PDIV, VPLL_SDIV)
#define BPLL_CON0_VAL	set_pll(BPLL_MDIV, BPLL_PDIV, BPLL_SDIV)

/* CLK_SRC_CPU	*/
/* 0 = MOUTAPLL,  1 = SCLKMPLL	*/
#define MUX_HPM_SEL             0
#define MUX_CPU_SEL             0
#define MUX_APLL_SEL            1

#define CLK_SRC_CPU_VAL	((MUX_HPM_SEL << 20)    \
				| (MUX_CPU_SEL << 16)  \
				| (MUX_APLL_SEL))

/* CLK_DIV_CPU0	*/
#if defined(CONFIG_CLK_ARM_600_APLL_600)
#define ARM2_RATIO              0x0
#define APLL_RATIO              0x1
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x2
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x1
#define ARM_RATIO               0x0

#elif defined(CONFIG_CLK_ARM_800_APLL_800)

#define ARM2_RATIO              0x0
#define APLL_RATIO              0x1
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x3
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x2
#define ARM_RATIO               0x0

#elif defined(CONFIG_CLK_ARM_1000_APLL_1000)

#define ARM2_RATIO              0x0
#define APLL_RATIO              0x1
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x4
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x2
#define ARM_RATIO               0x0

#elif defined(CONFIG_CLK_ARM_1200_APLL_1200)

#define ARM2_RATIO              0x0
#define APLL_RATIO              0x3
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x5
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x3
#define ARM_RATIO               0x0

#elif defined(CONFIG_CLK_ARM_1400_APLL_1400)

#define ARM2_RATIO              0x0
#define APLL_RATIO              0x3
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x6
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x3
#define ARM_RATIO               0x0

#elif defined(CONFIG_CLK_ARM_1700_APLL_1700)

#define ARM2_RATIO              0x0
#define APLL_RATIO              0x3
#define PCLK_DBG_RATIO          0x1
#define ATB_RATIO               0x6
#define PERIPH_RATIO            0x7
#define ACP_RATIO               0x7
#define CPUD_RATIO              0x3
#define ARM_RATIO               0x0
#endif

/* CLK_DIV_CPU0_VAL */
#define CLK_DIV_CPU0_VAL	((ARM2_RATIO << 28)             \
				| (APLL_RATIO << 24)            \
				| (PCLK_DBG_RATIO << 20)        \
				| (ATB_RATIO << 16)             \
				| (PERIPH_RATIO << 12)          \
				| (ACP_RATIO << 8)              \
				| (CPUD_RATIO << 4)             \
				| (ARM_RATIO))


/* CLK_FSYS */
#define CLK_SRC_FSYS0_VAL              0x66666
#define CLK_DIV_FSYS0_VAL	       0x0BB00000

/* CLK_DIV_CPU1	*/
#define HPM_RATIO               0x2
#define COPY_RATIO              0x0

/* CLK_DIV_CPU1 = 0x00000003 */
#define CLK_DIV_CPU1_VAL        ((HPM_RATIO << 4)		\
				| (COPY_RATIO))

/* CLK_SRC_CORE0 */
#define CLK_SRC_CORE0_VAL       0x00000000

/* CLK_SRC_CORE1 */
#define CLK_SRC_CORE1_VAL       0x100

/* CLK_DIV_CORE0 */
#define CLK_DIV_CORE0_VAL       0x00120000

/* CLK_DIV_CORE1 */
#define CLK_DIV_CORE1_VAL       0x07070700

/* CLK_DIV_SYSRGT */
#define CLK_DIV_SYSRGT_VAL      0x00000111

/* CLK_DIV_ACP */
#define CLK_DIV_ACP_VAL         0x12

/* CLK_DIV_SYSLFT */
#define CLK_DIV_SYSLFT_VAL      0x00000311

/* CLK_SRC_CDREX */
#define CLK_SRC_CDREX_VAL       0x1

/* CLK_DIV_CDREX */
#define MCLK_DPHY_RATIO	0x1
#define MCLK_CDREX_RATIO	0x1
#define ACLK_C2C_200_RATIO	0x1
#define C2C_CLK_400_RATIO	0x1
#define PCLK_CDREX_RATIO	0x1
#define ACLK_CDREX_RATIO	0x1

#if defined(CONFIG_LPDDR3)
#define CLK_DIV_CDREX_VAL	0x71720071

#elif defined(CONFIG_DDR3)
#define CLK_DIV_CDREX_VAL	((MCLK_DPHY_RATIO << 20)        \
                                | (MCLK_CDREX_RATIO << 16)      \
				| (ACLK_C2C_200_RATIO << 12)	\
				| (C2C_CLK_400_RATIO << 8)	\
				| (PCLK_CDREX_RATIO << 4)	\
				| (ACLK_CDREX_RATIO))
#endif
/* CLK_SRC_TOP0	*/
#define MUX_ACLK_300_GSCL_SEL           0x0
#define MUX_ACLK_300_GSCL_MID_SEL       0x0
#define MUX_ACLK_400_G3D_MID_SEL        0x0
#define MUX_ACLK_333_SEL	        0x0
#define MUX_ACLK_300_DISP1_SEL	        0x0
#define MUX_ACLK_300_DISP1_MID_SEL      0x0
#define MUX_ACLK_200_SEL	        0x0
#define MUX_ACLK_166_SEL	        0x0
#define CLK_SRC_TOP0_VAL	((MUX_ACLK_300_GSCL_SEL  << 25)		\
				| (MUX_ACLK_300_GSCL_MID_SEL << 24)	\
				| (MUX_ACLK_400_G3D_MID_SEL << 20)	\
				| (MUX_ACLK_333_SEL << 16)		\
				| (MUX_ACLK_300_DISP1_SEL << 15)	\
				| (MUX_ACLK_300_DISP1_MID_SEL << 14)	\
				| (MUX_ACLK_200_SEL << 12)		\
				| (MUX_ACLK_166_SEL << 8))

/* CLK_SRC_TOP1	*/
#define MUX_ACLK_400_G3D_SEL            0x1
#define MUX_ACLK_400_ISP_SEL            0x0
#define MUX_ACLK_400_IOP_SEL            0x0
#define MUX_ACLK_MIPI_HSI_TXBASE_SEL    0x0
#define MUX_ACLK_300_GSCL_MID1_SEL      0x0
#define MUX_ACLK_300_DISP1_MID1_SEL     0x0
#define CLK_SRC_TOP1_VAL	((MUX_ACLK_400_G3D_SEL << 28)           \
				|(MUX_ACLK_400_ISP_SEL << 24)           \
				|(MUX_ACLK_400_IOP_SEL << 20)           \
				|(MUX_ACLK_MIPI_HSI_TXBASE_SEL << 16)   \
				|(MUX_ACLK_300_GSCL_MID1_SEL << 12)     \
				|(MUX_ACLK_300_DISP1_MID1_SEL << 8))

/* CLK_SRC_TOP2 */
#define MUX_GPLL_SEL                    0x1
#define MUX_BPLL_USER_SEL               0x0
#define MUX_MPLL_USER_SEL               0x0
#define MUX_VPLL_SEL                    0x1
#define MUX_EPLL_SEL                    0x1
#define MUX_CPLL_SEL                    0x1
#define VPLLSRC_SEL                     0x0
#define CLK_SRC_TOP2_VAL	((MUX_GPLL_SEL << 28)	        \
				| (MUX_BPLL_USER_SEL << 24)	\
				| (MUX_MPLL_USER_SEL << 20)	\
				| (MUX_VPLL_SEL << 16)	        \
				| (MUX_EPLL_SEL << 12)	        \
				| (MUX_CPLL_SEL << 8)           \
				| (VPLLSRC_SEL))
/* CLK_SRC_TOP3 */
#define MUX_ACLK_333_SUB_SEL            0x1
#define MUX_ACLK_400_SUB_SEL            0x1
#define MUX_ACLK_266_ISP_SUB_SEL        0x1
#define MUX_ACLK_266_GPS_SUB_SEL        0x0
#define MUX_ACLK_300_GSCL_SUB_SEL       0x1
#define MUX_ACLK_266_GSCL_SUB_SEL       0x1
#define MUX_ACLK_300_DISP1_SUB_SEL      0x1
#define MUX_ACLK_200_DISP1_SUB_SEL      0x1
#define CLK_SRC_TOP3_VAL	((MUX_ACLK_333_SUB_SEL << 24)	        \
				| (MUX_ACLK_400_SUB_SEL << 20)	        \
				| (MUX_ACLK_266_ISP_SUB_SEL << 16)	\
				| (MUX_ACLK_266_GPS_SUB_SEL << 12)      \
				| (MUX_ACLK_300_GSCL_SUB_SEL << 10)     \
				| (MUX_ACLK_266_GSCL_SUB_SEL << 8)      \
				| (MUX_ACLK_300_DISP1_SUB_SEL << 6)     \
				| (MUX_ACLK_200_DISP1_SUB_SEL << 4))

/* CLK_DIV_TOP0	*/
#define ACLK_300_DISP1_RATIO	0x2
#define ACLK_400_G3D_RATIO	0x0
#define ACLK_333_RATIO		0x0
#define ACLK_266_RATIO		0x2
#define ACLK_200_RATIO		0x3
#define ACLK_166_RATIO		0x1
#define ACLK_133_RATIO		0x1
#define ACLK_66_RATIO		0x5

#define CLK_DIV_TOP0_VAL	((ACLK_300_DISP1_RATIO << 28)	\
				| (ACLK_400_G3D_RATIO << 24)	\
				| (ACLK_333_RATIO  << 20)	\
				| (ACLK_266_RATIO << 16)	\
				| (ACLK_200_RATIO << 12)	\
				| (ACLK_166_RATIO << 8)		\
				| (ACLK_133_RATIO << 4)		\
				| (ACLK_66_RATIO))

/* CLK_DIV_TOP1	*/
#define ACLK_MIPI_HSI_TX_BASE_RATIO     0x3
#define ACLK_66_PRE_RATIO               0x1
#define ACLK_400_ISP_RATIO              0x1
#define ACLK_400_IOP_RATIO              0x1
#define ACLK_300_GSCL_RATIO             0x2

#define CLK_DIV_TOP1_VAL	((ACLK_MIPI_HSI_TX_BASE_RATIO << 28)	\
				| (ACLK_66_PRE_RATIO << 24)		\
				| (ACLK_400_ISP_RATIO  << 20)		\
				| (ACLK_400_IOP_RATIO << 16)		\
				| (ACLK_300_GSCL_RATIO << 12))

/* APLL_LOCK	*/
#define APLL_LOCK_VAL	(0x546)
/* MPLL_LOCK	*/
#define MPLL_LOCK_VAL	(0x546)
/* CPLL_LOCK	*/
#define CPLL_LOCK_VAL	(0x546)
/* GPLL_LOCK	*/
#define GPLL_LOCK_VAL	(0x546)
/* EPLL_LOCK	*/
#define EPLL_LOCK_VAL	(0x3A98)
/* VPLL_LOCK	*/
#define VPLL_LOCK_VAL	(0x3A98)
/* BPLL_LOCK	*/
#define BPLL_LOCK_VAL	(0x546)

/* CLK_SRC_PERIC0 */
#define PWM_SEL		0
#define UART3_SEL	6
#define UART2_SEL	6
#define UART1_SEL	6
#define UART0_SEL	6
/* SRC_CLOCK = SCLK_MPLL */
#define CLK_SRC_PERIC0_VAL	((PWM_SEL << 24)        \
				| (UART3_SEL << 12)     \
				| (UART2_SEL << 8)       \
				| (UART1_SEL << 4)      \
				| (UART0_SEL))

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

/* CLK_DIV_PERIL0	*/
#define UART5_RATIO	7
#define UART4_RATIO	7
#define UART3_RATIO	7
#define UART2_RATIO	7
#define UART1_RATIO	7
#define UART0_RATIO	7

#define CLK_DIV_PERIC0_VAL	((UART3_RATIO << 12)    \
				| (UART2_RATIO << 8)    \
				| (UART1_RATIO << 4)    \
				| (UART0_RATIO))
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

/* CLK_SRC_LEX */
#define CLK_SRC_LEX_VAL         0x0

/* CLK_DIV_LEX */
#define CLK_DIV_LEX_VAL         0x10

/* CLK_DIV_R0X */
#define CLK_DIV_R0X_VAL         0x10

/* CLK_DIV_L0X */
#define CLK_DIV_R1X_VAL         0x10

/* CLK_DIV_ISP0 */
#define CLK_DIV_ISP0_VAL        0x31

/* CLK_DIV_ISP1 */
#define CLK_DIV_ISP1_VAL        0x0

/* CLK_DIV_ISP2 */
#define CLK_DIV_ISP2_VAL        0x1

#define MPLL_DEC	(MPLL_MDIV * MPLL_MDIV / (MPLL_PDIV * 2^(MPLL_SDIV-1)))

#define SCLK_UART	MPLL_DEC / (UART1_RATIO+1)

#define UART_UBRDIV_VAL	0x35    /* (SCLK_UART/(115200*16) -1) */
#define UART_UDIVSLOT_VAL	0x4
	/*((((SCLK_UART*10/(115200*16) -10))%10)*16/10)*/
void system_clock_init(void);
#endif
