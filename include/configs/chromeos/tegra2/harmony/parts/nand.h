/*
 *  (C) Copyright 2010
 *  NVIDIA Corporation <www.nvidia.com>
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

/* Set clock divisor
 * 7 bits of D and 1 bit of H
 * divisor= (DDDDDDD + 1) + (H x 0.5)
 * clock = original clock / divisor
 * 6 means /4  */
#define CONFIG_NAND_CLK_DIVISOR_DDDDDDDH       6

/* For HYNIX HY27UF4G2B
 * Frequence output of PLLP_OUT0 is set by BOOTROM to 216MHz
 * to CLK_RST_CONTROLLER_PLLP_BASE_0,
 * 216MHz / divisor 4 = 54MHZ
 * 1 clock = 18.5 ns = NAND_CLK_PERIOD
 * TRP_RESP_CNT=n, max(tRP, tREA)= max(12ns, 20ns)= 20ns for non-EDO mode
 *   bit 31-28=n=1, generated timing= (n+1) * NAND_CLK_PERIOD= (1+1)* 18.5
 * TWB_CNT bit 27-24=n, tWB = 100ns = (n+1)* 18.5, so n= 5 (bit 27-24)
 * similar way for other fields, please refer to reference manual
 */
/* Value to be set to NAND_TIMING_0 register, address=70008014h */
#define CONFIG_TEGRA2_NAND_TIMING      0x15040001
/* Value to be set to NAND_TIMING2_0 register, address=7000801Ch */
#define CONFIG_TEGRA2_NAND_TIMING2     0x01
