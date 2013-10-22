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
#include <asm/errno.h>
#include <asm/arch-tegra/ap.h>
#include <asm/arch/clk_rst.h>
#include <asm/arch/clock.h>
#include <asm/arch/pmc.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/tegra.h>
#include <asm/arch/fuse.h>
#include <asm/arch/gp_padctrl.h>
#include <asm/arch/warmboot.h>
#include <asm/arch/sdram_param.h>

enum field_type {
	TYPE_SDRAM,
	TYPE_PLLM,
	TYPE_CONST,
	TYPE_COUNT,
};

#define sdram_offset(member) offsetof(struct sdram_params, member)
#define pmc_offset(member) offsetof(struct pmc_ctlr, member)
#define pllm_offset(member) offsetof(struct clk_pllm, member)

struct encode_fields {
	u32 src_offset;
	u32 op1_mask;
	u32 op1_shift;
	u32 op2_mask;
	u32 op2_shift;
	u32 dst_offset;
	u32 dst_shift;
};

/*
 * encode: set dst_off.dst_bit to 1 if (src_off.op1_bits > src_off.op2_bits)
 *         else set dst_off.dst_bit to 0.
 */
#define encode(src_off, op1_bits, op2_bits, dst_off, dst_bit)		\
	{								\
		.src_offset = src_off,					\
		.op1_mask = 0xfffffffful >>				\
			(31 - ((1 ? op1_bits) - (0 ? op1_bits))),	\
		.op1_shift = 0 ? op1_bits,				\
		.op2_mask = 0xfffffffful >>				\
			(31 - ((1 ? op2_bits) - (0 ? op2_bits))),	\
		.op2_shift = 0 ? op2_bits,				\
		.dst_offset = dst_off,					\
		.dst_shift = dst_bit,					\
	}

#define e(src, op1, op2, dst, dbit)					\
	encode(sdram_offset(src), op1, op2, sdram_offset(dst), dbit)

struct encode_fields encode_list[] = {
	e(emc_swizzle_rank0_byte0, 26:24, 30:28, swizzle_rank_byte_encode, 0),
	e(emc_swizzle_rank0_byte1, 26:24, 30:28, swizzle_rank_byte_encode, 1),
	e(emc_swizzle_rank0_byte2, 26:24, 30:28, swizzle_rank_byte_encode, 2),
	e(emc_swizzle_rank0_byte3, 26:24, 30:28, swizzle_rank_byte_encode, 3),
	e(emc_swizzle_rank1_byte0, 26:24, 30:28, swizzle_rank_byte_encode, 4),
	e(emc_swizzle_rank1_byte1, 26:24, 30:28, swizzle_rank_byte_encode, 5),
	e(emc_swizzle_rank1_byte2, 26:24, 30:28, swizzle_rank_byte_encode, 6),
	e(emc_swizzle_rank1_byte3, 26:24, 30:28, swizzle_rank_byte_encode, 7),
};

struct pack_fields {
	enum field_type src_type;
	u32 src_offset;		/* for TYPE_CONST, this field is a constant */
	u32 src_mask;
	u32 src_shift;
	enum field_type dst_type;
	u32 dst_offset;
	u32 dst_mask;
	u32 dst_shift;
};

/*
 * pack: a macro to copy from bits in src to bits in dst.
 *
 * For example,
 *   pack(TYPE_SDRAM, sdram_offset(emc_clock_source), 7:0,
 *	  pmc_offset(pmc_scratch6), 15:8)
 *     is to:
 *         copy bits 7:0 of sdram_offset.emc_clock_source to
 *         bits 15:8 of pmc_scratch6 register.
 *
 *   The first parameter of pack determines the type of src:
 *     TYPE_SDRAM: src is of SDRAM parameter,
 *     TYPE_PLLM:  src is of PLLM,
 *     TYPE_CONST: src is a constant.
 */
#define pack(_src_type, src_off, src_bits, dst_off, dst_bits)		\
	{								\
		.src_type = _src_type,					\
		.src_offset = src_off,					\
		.src_mask = 0xfffffffful >>				\
			(31 - ((1 ? src_bits) - (0 ? src_bits))),	\
		.src_shift = 0 ? src_bits,				\
		.dst_offset = dst_off,					\
		.dst_mask = 0xfffffffful >>				\
			(31 - ((1 ? dst_bits) - (0 ? dst_bits))),	\
		.dst_shift = 0 ? dst_bits,				\
	}

#define s(offset, src, scratch, dst)	\
	pack(TYPE_SDRAM, sdram_offset(offset), src, pmc_offset(scratch), dst)

#define m(offset, src, scratch, dst)	\
	pack(TYPE_PLLM, pllm_offset(offset), src, pmc_offset(scratch), dst)

#define c(const, src, scratch, dst)	\
	pack(TYPE_CONST, const, src, pmc_offset(scratch), dst)

struct pack_fields pack_list_1[] = {
	s(emc_clock_source, 7:0, pmc_scratch6, 15:8),
	s(emc_clock_source, 31:29, pmc_scratch6, 18:16),
	s(emc_clock_source, 26:26, pmc_scratch6, 19:19),
	s(emc_odt_write, 5:0, pmc_scratch6, 25:20),
	s(emc_odt_write, 11:8, pmc_scratch6, 29:26),
	s(emc_odt_write, 30:30, pmc_scratch6, 30:30),
	s(emc_odt_write, 31:31, pmc_scratch6, 31:31),
	s(emc_xm2dqpadctrl2, 18:16, pmc_scratch7, 22:20),
	s(emc_xm2dqpadctrl2, 22:20, pmc_scratch7, 25:23),
	s(emc_xm2dqpadctrl2, 26:24, pmc_scratch7, 28:26),
	s(emc_xm2dqpadctrl2, 30:28, pmc_scratch7, 31:29),
	s(emc_xm2dqpadctrl3, 18:16, pmc_scratch8, 22:20),
	s(emc_xm2dqpadctrl3, 22:20, pmc_scratch8, 25:23),
	s(emc_xm2dqpadctrl3, 26:24, pmc_scratch8, 28:26),
	s(emc_xm2dqpadctrl3, 30:28, pmc_scratch8, 31:29),
	s(emc_txsrdll, 11:0, pmc_scratch9, 31:20),
	c(0, 31:0, pmc_scratch10, 31:0),
	s(emc_dsr_vttgen_drv, 5:0, pmc_scratch10, 25:20),
	s(emc_dsr_vttgen_drv, 18:16, pmc_scratch10, 28:26),
	s(emc_dsr_vttgen_drv, 26:24, pmc_scratch10, 31:29),
	s(emc_fbio_spare, 31:24, pmc_scratch11, 7:0),
	s(emc_fbio_spare, 23:16, pmc_scratch11, 15:8),
	s(emc_fbio_spare, 15:8, pmc_scratch11, 23:16),
	s(emc_fbio_spare, 7:0, pmc_scratch11, 31:24),
	s(emc_cfg_rsv, 31:0, pmc_scratch12, 31:0),
	s(emc_cdb_cntl2, 31:0, pmc_scratch13, 31:0),
	s(mc_emem_arb_da_turns, 31:0, pmc_scratch14, 31:0),
	s(emc_cfg_dig_dll, 0:0, pmc_scratch17, 0:0),
	s(emc_cfg_dig_dll, 25:2, pmc_scratch17, 24:1),
	s(emc_cfg_dig_dll, 31:27, pmc_scratch17, 29:25),
	s(emc_cdb_cntl1, 29:0, pmc_scratch18, 29:0),
	s(mc_emem_arb_misc0, 14:0, pmc_scratch19, 14:0),
	s(mc_emem_arb_misc0, 30:16, pmc_scratch19, 29:15),
	s(emc_xm2dqspadctrl, 4:0, pmc_scratch22, 4:0),
	s(emc_xm2dqspadctrl, 12:8, pmc_scratch22, 9:5),
	s(emc_xm2dqspadctrl, 31:14, pmc_scratch22, 27:10),
	s(emc_rdr, 3:0, pmc_scratch22, 31:28),
	s(emc_xm2dqpadctrl, 31:4, pmc_scratch23, 27:0),
	s(emc_rext, 3:0, pmc_scratch23, 31:28),
	s(emc_xm2comppadctrl, 16:0, pmc_scratch24, 16:0),
	s(emc_xm2comppadctrl, 24:20, pmc_scratch24, 21:17),
	s(emc_xm2comppadctrl, 27:27, pmc_scratch24, 22:22),
	s(emc_xm2comppadctrl, 31:28, pmc_scratch24, 26:23),
	s(emc_r2w, 4:0, pmc_scratch24, 31:27),
	s(emc_cfg, 9:1, pmc_scratch25, 8:0),
	s(emc_cfg, 26:16, pmc_scratch25, 19:9),
	s(emc_cfg, 31:28, pmc_scratch25, 23:20),
	s(emc_xm2vttgenpadctrl, 0:0, pmc_scratch25, 24:24),
	s(emc_xm2vttgenpadctrl, 2:2, pmc_scratch25, 25:25),
	s(emc_xm2vttgenpadctrl, 18:16, pmc_scratch25, 28:26),
	s(emc_xm2vttgenpadctrl, 26:24, pmc_scratch25, 31:29),
	s(emc_zcal_interval, 23:10, pmc_scratch26, 13:0),
	s(emc_zcal_interval, 9:0, pmc_scratch26, 23:14),
	s(emc_sel_dpd_ctrl, 5:2, pmc_scratch26, 27:24),
	s(emc_sel_dpd_ctrl, 8:8, pmc_scratch26, 28:28),
	s(emc_sel_dpd_ctrl, 18:16, pmc_scratch26, 31:29),
	s(emc_xm2vttgenpadctrl3, 22:0, pmc_scratch27, 22:0),
	s(emc_xm2vttgenpadctrl3, 24:24, pmc_scratch27, 23:23),
	s(emc_swizzle_rank0_byte_cfg, 1:0, pmc_scratch27, 25:24),
	s(emc_swizzle_rank0_byte_cfg, 5:4, pmc_scratch27, 27:26),
	s(emc_swizzle_rank0_byte_cfg, 9:8, pmc_scratch27, 29:28),
	s(emc_swizzle_rank0_byte_cfg, 13:12, pmc_scratch27, 31:30),
	s(emc_xm2clkpadctrl2, 5:0, pmc_scratch28, 5:0),
	s(emc_xm2clkpadctrl2, 13:8, pmc_scratch28, 11:6),
	s(emc_xm2clkpadctrl2, 20:16, pmc_scratch28, 16:12),
	s(emc_xm2clkpadctrl2, 23:23, pmc_scratch28, 17:17),
	s(emc_xm2clkpadctrl2, 28:24, pmc_scratch28, 22:18),
	s(emc_xm2clkpadctrl2, 31:31, pmc_scratch28, 23:23),
	s(emc_swizzle_rank1_byte_cfg, 1:0, pmc_scratch28, 25:24),
	s(emc_swizzle_rank1_byte_cfg, 5:4, pmc_scratch28, 27:26),
	s(emc_swizzle_rank1_byte_cfg, 9:8, pmc_scratch28, 29:28),
	s(emc_swizzle_rank1_byte_cfg, 13:12, pmc_scratch28, 31:30),
	s(mc_emem_arb_da_covers, 23:0, pmc_scratch29, 23:0),
	s(mc_emem_arb_rsv, 7:0, pmc_scratch29, 31:24),
	s(emc_auto_cal_config, 4:0, pmc_scratch30, 4:0),
	s(emc_auto_cal_config, 12:8, pmc_scratch30, 9:5),
	s(emc_auto_cal_config, 18:16, pmc_scratch30, 12:10),
	s(emc_auto_cal_config, 25:20, pmc_scratch30, 18:13),
	s(emc_auto_cal_config, 31:28, pmc_scratch30, 22:19),
	s(emc_rfc, 8:0, pmc_scratch30, 31:23),
	s(emc_xm2dqspadctrl2, 21:0, pmc_scratch31, 21:0),
	s(emc_xm2dqspadctrl2, 24:24, pmc_scratch31, 22:22),
	s(emc_ar2pden, 8:0, pmc_scratch31, 31:23),
	s(emc_xm2clkpadctrl, 0:0, pmc_scratch32, 0:0),
	s(emc_xm2clkpadctrl, 4:2, pmc_scratch32, 3:1),
	s(emc_xm2clkpadctrl, 7:7, pmc_scratch32, 4:4),
	s(emc_xm2clkpadctrl, 31:14, pmc_scratch32, 22:5),
	s(emc_rfc_slr, 8:0, pmc_scratch32, 31:23),
	s(emc_xm2dqspadctrl3, 0:0, pmc_scratch33, 0:0),
	s(emc_xm2dqspadctrl3, 5:5, pmc_scratch33, 1:1),
	s(emc_xm2dqspadctrl3, 12:8, pmc_scratch33, 6:2),
	s(emc_xm2dqspadctrl3, 18:14, pmc_scratch33, 11:7),
	s(emc_xm2dqspadctrl3, 24:20, pmc_scratch33, 16:12),
	s(emc_xm2dqspadctrl3, 30:26, pmc_scratch33, 21:17),
	s(emc_txsr, 9:0, pmc_scratch33, 31:22),
	s(mc_emem_arb_cfg, 8:0, pmc_scratch40, 8:0),
	s(mc_emem_arb_cfg, 20:16, pmc_scratch40, 13:9),
	s(mc_emem_arb_cfg, 27:24, pmc_scratch40, 17:14),
	s(mc_emem_arb_cfg, 31:28, pmc_scratch40, 21:18),
	s(emc_mc2emcq, 2:0, pmc_scratch40, 24:22),
	s(emc_mc2emcq, 10:8, pmc_scratch40, 27:25),
	s(emc_mc2emcq, 27:24, pmc_scratch40, 31:28),
	s(emc_auto_cal_interval, 20:0, pmc_scratch42, 20:0),
	s(mc_emem_arb_outstanding_req, 8:0, pmc_scratch42, 29:21),
	s(mc_emem_arb_outstanding_req, 31:30, pmc_scratch42, 31:30),
	s(emc_mrs_wait_cnt2, 9:0, pmc_scratch44, 9:0),
	s(emc_mrs_wait_cnt2, 25:16, pmc_scratch44, 19:10),
	s(emc_txdsrvttgen, 11:0, pmc_scratch44, 31:20),
	s(emc_mrs_wait_cnt, 9:0, pmc_scratch45, 9:0),
	s(emc_mrs_wait_cnt, 25:16, pmc_scratch45, 19:10),
	s(emc_cfg_pipe, 1:0, pmc_scratch45, 21:20),
	s(emc_cfg_pipe, 9:4, pmc_scratch45, 27:22),
	s(emc_cfg_pipe, 15:12, pmc_scratch45, 31:28),
	s(emc_xm2dqspadctrl4, 22:18, pmc_scratch46, 4:0),
	s(emc_xm2dqspadctrl4, 16:12, pmc_scratch46, 9:5),
	s(emc_xm2dqspadctrl4, 10:6, pmc_scratch46, 14:10),
	s(emc_xm2dqspadctrl4, 4:0, pmc_scratch46, 19:15),
	s(emc_zcal_wait_cnt, 9:0, pmc_scratch46, 29:20),
	s(emc_xm2dqspadctrl5, 22:18, pmc_scratch47, 4:0),
	s(emc_xm2dqspadctrl5, 16:12, pmc_scratch47, 9:5),
	s(emc_xm2dqspadctrl5, 10:6, pmc_scratch47, 14:10),
	s(emc_xm2dqspadctrl5, 4:0, pmc_scratch47, 19:15),
	s(emc_xm2vttgenpadctrl2, 5:0, pmc_scratch47, 25:20),
	s(emc_xm2vttgenpadctrl2, 31:28, pmc_scratch47, 29:26),
	s(emc_xm2dqspadctrl6, 12:8, pmc_scratch48, 4:0),
	s(emc_xm2dqspadctrl6, 18:14, pmc_scratch48, 9:5),
	s(emc_xm2dqspadctrl6, 24:20, pmc_scratch48, 14:10),
	s(emc_xm2dqspadctrl6, 30:26, pmc_scratch48, 19:15),
	s(emc_auto_cal_config3, 4:0, pmc_scratch48, 24:20),
	s(emc_auto_cal_config3, 12:8, pmc_scratch48, 29:25),
	s(emc_fbio_cfg5, 1:0, pmc_scratch48, 31:30),
	s(emc_dll_xform_quse8, 4:0, pmc_scratch50, 4:0),
	s(emc_dll_xform_quse8, 22:8, pmc_scratch50, 19:5),
	s(mc_emem_arb_ring1_throttle, 4:0, pmc_scratch50, 24:20),
	s(mc_emem_arb_ring1_throttle, 20:16, pmc_scratch50, 29:25),
	s(emc_fbio_cfg5, 3:2, pmc_scratch50, 31:30),
	s(emc_dll_xform_quse9, 4:0, pmc_scratch51, 4:0),
	s(emc_dll_xform_quse9, 22:8, pmc_scratch51, 19:5),
	s(emc_ctt_term_ctrl, 2:0, pmc_scratch51, 22:20),
	s(emc_ctt_term_ctrl, 12:8, pmc_scratch51, 27:23),
	s(emc_ctt_term_ctrl, 31:31, pmc_scratch51, 28:28),
	s(emc_fbio_cfg6, 2:0, pmc_scratch51, 31:29),
	s(emc_dll_xform_quse10, 4:0, pmc_scratch56, 4:0),
	s(emc_dll_xform_quse10, 22:8, pmc_scratch56, 19:5),
	s(emc_xm2cmdpadctrl, 10:3, pmc_scratch56, 27:20),
	s(emc_xm2cmdpadctrl, 28:28, pmc_scratch56, 28:28),
	s(emc_puterm_adj, 1:0, pmc_scratch56, 30:29),
	s(emc_puterm_adj, 7:7, pmc_scratch56, 31:31),
	s(emc_dll_xform_quse11, 4:0, pmc_scratch57, 4:0),
	s(emc_dll_xform_quse11, 22:8, pmc_scratch57, 19:5),
	s(emc_wdv, 3:0, pmc_scratch57, 31:28),
	s(emc_dll_xform_quse12, 4:0, pmc_scratch58, 4:0),
	s(emc_dll_xform_quse12, 22:8, pmc_scratch58, 19:5),
	s(emc_burst_refresh_num, 3:0, pmc_scratch58, 31:28),
	s(emc_dll_xform_quse13, 4:0, pmc_scratch59, 4:0),
	s(emc_dll_xform_quse13, 22:8, pmc_scratch59, 19:5),
	s(emc_wext, 3:0, pmc_scratch59, 31:28),
	s(emc_dll_xform_quse14, 4:0, pmc_scratch60, 4:0),
	s(emc_dll_xform_quse14, 22:8, pmc_scratch60, 19:5),
	s(emc_clken_override, 3:1, pmc_scratch60, 30:28),
	s(emc_clken_override, 6:6, pmc_scratch60, 31:31),
	s(emc_dll_xform_quse15, 4:0, pmc_scratch61, 4:0),
	s(emc_dll_xform_quse15, 22:8, pmc_scratch61, 19:5),
	s(emc_r2r, 3:0, pmc_scratch61, 31:28),
	s(emc_dll_xform_dq4, 4:0, pmc_scratch62, 4:0),
	s(emc_dll_xform_dq4, 22:8, pmc_scratch62, 19:5),
	s(emc_rc, 6:0, pmc_scratch62, 26:20),
	s(emc_w2r, 4:0, pmc_scratch62, 31:27),
	s(emc_dll_xform_dq5, 4:0, pmc_scratch63, 4:0),
	s(emc_dll_xform_dq5, 22:8, pmc_scratch63, 19:5),
	s(emc_tfaw, 6:0, pmc_scratch63, 26:20),
	s(emc_r2p, 4:0, pmc_scratch63, 31:27),
	s(emc_dll_xform_dq6, 4:0, pmc_scratch64, 4:0),
	s(emc_dll_xform_dq6, 22:8, pmc_scratch64, 19:5),
	s(emc_dli_trim_txdqs0, 6:0, pmc_scratch64, 26:20),
	s(emc_qsafe, 4:0, pmc_scratch64, 31:27),
	s(emc_dll_xform_dq7, 4:0, pmc_scratch65, 4:0),
	s(emc_dll_xform_dq7, 22:8, pmc_scratch65, 19:5),
	s(emc_dli_trim_txdqs1, 6:0, pmc_scratch65, 26:20),
	s(emc_tclkstable, 4:0, pmc_scratch65, 31:27),
	s(emc_auto_cal_config2, 4:0, pmc_scratch66, 4:0),
	s(emc_auto_cal_config2, 12:8, pmc_scratch66, 9:5),
	s(emc_auto_cal_config2, 20:16, pmc_scratch66, 14:10),
	s(emc_auto_cal_config2, 28:24, pmc_scratch66, 19:15),
	s(emc_dli_trim_txdqs2, 6:0, pmc_scratch66, 26:20),
	s(emc_tclkstop, 4:0, pmc_scratch66, 31:27),
	s(mc_emem_arb_misc1, 1:0, pmc_scratch67, 1:0),
	s(mc_emem_arb_misc1, 12:4, pmc_scratch67, 10:2),
	s(mc_emem_arb_misc1, 25:21, pmc_scratch67, 15:11),
	s(mc_emem_arb_misc1, 31:28, pmc_scratch67, 19:16),
	s(emc_dli_trim_txdqs3, 6:0, pmc_scratch67, 26:20),
	s(emc_einput_duration, 4:0, pmc_scratch67, 31:27),
	s(emc_zcal_mrw_cmd, 7:0, pmc_scratch68, 7:0),
	s(emc_zcal_mrw_cmd, 23:16, pmc_scratch68, 15:8),
	s(emc_zcal_mrw_cmd, 31:30, pmc_scratch68, 17:16),
	s(emc_trefbw, 13:0, pmc_scratch68, 31:18),
	s(emc_xm2cmdpadctrl2, 31:14, pmc_scratch69, 17:0),
	s(emc_dli_trim_txdqs4, 6:0, pmc_scratch69, 24:18),
	s(emc_dli_trim_txdqs5, 6:0, pmc_scratch69, 31:25),
	s(emc_xm2cmdpadctrl3, 31:14, pmc_scratch70, 17:0),
	s(emc_dli_trim_txdqs6, 6:0, pmc_scratch70, 24:18),
	s(emc_dli_trim_txdqs7, 6:0, pmc_scratch70, 31:25),
	s(emc_xm2cmdpadctrl5, 2:0, pmc_scratch71, 2:0),
	s(emc_xm2cmdpadctrl5, 6:4, pmc_scratch71, 5:3),
	s(emc_xm2cmdpadctrl5, 10:8, pmc_scratch71, 8:6),
	s(emc_xm2cmdpadctrl5, 14:12, pmc_scratch71, 11:9),
	s(emc_xm2cmdpadctrl5, 18:16, pmc_scratch71, 14:12),
	s(emc_xm2cmdpadctrl5, 22:20, pmc_scratch71, 17:15),
	s(emc_dli_trim_txdqs8, 6:0, pmc_scratch71, 24:18),
	s(emc_dli_trim_txdqs9, 6:0, pmc_scratch71, 31:25),
	s(emc_cdb_cntl3, 17:0, pmc_scratch72, 17:0),
	s(emc_dli_trim_txdqs10, 6:0, pmc_scratch72, 24:18),
	s(emc_dli_trim_txdqs11, 6:0, pmc_scratch72, 31:25),
	s(emc_swizzle_rank0_byte0, 2:0, pmc_scratch73, 2:0),
	s(emc_swizzle_rank0_byte0, 6:4, pmc_scratch73, 5:3),
	s(emc_swizzle_rank0_byte0, 10:8, pmc_scratch73, 8:6),
	s(emc_swizzle_rank0_byte0, 14:12, pmc_scratch73, 11:9),
	s(emc_swizzle_rank0_byte0, 18:16, pmc_scratch73, 14:12),
	s(emc_swizzle_rank0_byte0, 22:20, pmc_scratch73, 17:15),
	s(emc_dli_trim_txdqs12, 6:0, pmc_scratch73, 24:18),
	s(emc_dli_trim_txdqs13, 6:0, pmc_scratch73, 31:25),
	s(emc_swizzle_rank0_byte1, 2:0, pmc_scratch74, 2:0),
	s(emc_swizzle_rank0_byte1, 6:4, pmc_scratch74, 5:3),
	s(emc_swizzle_rank0_byte1, 10:8, pmc_scratch74, 8:6),
	s(emc_swizzle_rank0_byte1, 14:12, pmc_scratch74, 11:9),
	s(emc_swizzle_rank0_byte1, 18:16, pmc_scratch74, 14:12),
	s(emc_swizzle_rank0_byte1, 22:20, pmc_scratch74, 17:15),
	s(emc_dli_trim_txdqs14, 6:0, pmc_scratch74, 24:18),
	s(emc_dli_trim_txdqs15, 6:0, pmc_scratch74, 31:25),
	s(emc_swizzle_rank0_byte2, 2:0, pmc_scratch75, 2:0),
	s(emc_swizzle_rank0_byte2, 6:4, pmc_scratch75, 5:3),
	s(emc_swizzle_rank0_byte2, 10:8, pmc_scratch75, 8:6),
	s(emc_swizzle_rank0_byte2, 14:12, pmc_scratch75, 11:9),
	s(emc_swizzle_rank0_byte2, 18:16, pmc_scratch75, 14:12),
	s(emc_swizzle_rank0_byte2, 22:20, pmc_scratch75, 17:15),
	s(mc_emem_arb_timing_rp, 6:0, pmc_scratch75, 24:18),
	s(mc_emem_arb_timing_rc, 6:0, pmc_scratch75, 31:25),
	s(emc_swizzle_rank0_byte3, 2:0, pmc_scratch76, 2:0),
	s(emc_swizzle_rank0_byte3, 6:4, pmc_scratch76, 5:3),
	s(emc_swizzle_rank0_byte3, 10:8, pmc_scratch76, 8:6),
	s(emc_swizzle_rank0_byte3, 14:12, pmc_scratch76, 11:9),
	s(emc_swizzle_rank0_byte3, 18:16, pmc_scratch76, 14:12),
	s(emc_swizzle_rank0_byte3, 22:20, pmc_scratch76, 17:15),
	s(mc_emem_arb_timing_faw, 6:0, pmc_scratch76, 24:18),
	s(mc_emem_arb_timing_wap2pre, 6:0, pmc_scratch76, 31:25),
	s(emc_swizzle_rank1_byte0, 2:0, pmc_scratch77, 2:0),
	s(emc_swizzle_rank1_byte0, 6:4, pmc_scratch77, 5:3),
	s(emc_swizzle_rank1_byte0, 10:8, pmc_scratch77, 8:6),
	s(emc_swizzle_rank1_byte0, 14:12, pmc_scratch77, 11:9),
	s(emc_swizzle_rank1_byte0, 18:16, pmc_scratch77, 14:12),
	s(emc_swizzle_rank1_byte0, 22:20, pmc_scratch77, 17:15),
	s(emc_ras, 5:0, pmc_scratch77, 23:18),
	s(emc_rp, 5:0, pmc_scratch77, 29:24),
	s(emc_cfg2, 9:8, pmc_scratch77, 31:30),
	s(emc_swizzle_rank1_byte1, 2:0, pmc_scratch78, 2:0),
	s(emc_swizzle_rank1_byte1, 6:4, pmc_scratch78, 5:3),
	s(emc_swizzle_rank1_byte1, 10:8, pmc_scratch78, 8:6),
	s(emc_swizzle_rank1_byte1, 14:12, pmc_scratch78, 11:9),
	s(emc_swizzle_rank1_byte1, 18:16, pmc_scratch78, 14:12),
	s(emc_swizzle_rank1_byte1, 22:20, pmc_scratch78, 17:15),
	s(emc_w2p, 5:0, pmc_scratch78, 23:18),
	s(emc_rd_rcd, 5:0, pmc_scratch78, 29:24),
	s(emc_cfg2, 27:26, pmc_scratch78, 31:30),
	s(emc_swizzle_rank1_byte2, 2:0, pmc_scratch79, 2:0),
	s(emc_swizzle_rank1_byte2, 6:4, pmc_scratch79, 5:3),
	s(emc_swizzle_rank1_byte2, 10:8, pmc_scratch79, 8:6),
	s(emc_swizzle_rank1_byte2, 14:12, pmc_scratch79, 11:9),
	s(emc_swizzle_rank1_byte2, 18:16, pmc_scratch79, 14:12),
	s(emc_swizzle_rank1_byte2, 22:20, pmc_scratch79, 17:15),
	s(emc_wr_rcd, 5:0, pmc_scratch79, 23:18),
	s(emc_quse, 5:0, pmc_scratch79, 29:24),
	s(emc_fbio_cfg5, 4:4, pmc_scratch79, 31:31),
	s(emc_swizzle_rank1_byte3, 2:0, pmc_scratch80, 2:0),
	s(emc_swizzle_rank1_byte3, 6:4, pmc_scratch80, 5:3),
	s(emc_swizzle_rank1_byte3, 10:8, pmc_scratch80, 8:6),
	s(emc_swizzle_rank1_byte3, 14:12, pmc_scratch80, 11:9),
	s(emc_swizzle_rank1_byte3, 18:16, pmc_scratch80, 14:12),
	s(emc_swizzle_rank1_byte3, 22:20, pmc_scratch80, 17:15),
	s(emc_qrst, 5:0, pmc_scratch80, 23:18),
	s(emc_rdv, 5:0, pmc_scratch80, 29:24),
	s(emc_fbio_cfg5, 6:5, pmc_scratch80, 31:30),
	s(emc_dyn_self_ref_control, 15:0, pmc_scratch81, 15:0),
	s(emc_dyn_self_ref_control, 31:31, pmc_scratch81, 16:16),
	s(emc_pdex2wr, 5:0, pmc_scratch81, 22:17),
	s(emc_pdex2rd, 5:0, pmc_scratch81, 28:23),
	s(emc_refresh, 5:0, pmc_scratch82, 5:0),
	s(emc_refresh, 15:6, pmc_scratch82, 15:6),
	s(emc_cmdq, 4:0, pmc_scratch82, 20:16),
	s(emc_cmdq, 10:8, pmc_scratch82, 23:21),
	s(emc_cmdq, 14:12, pmc_scratch82, 26:24),
	s(emc_cmdq, 28:24, pmc_scratch82, 31:27),
	s(emc_acpd_control, 15:0, pmc_scratch83, 15:0),
	s(emc_cfg_dig_dll_period, 15:0, pmc_scratch83, 31:16),
	s(emc_dll_xform_dqs0, 4:0, pmc_scratch84, 4:0),
	s(emc_dll_xform_dqs0, 22:12, pmc_scratch84, 15:5),
	s(emc_dll_xform_dqs1, 4:0, pmc_scratch84, 20:16),
	s(emc_dll_xform_dqs1, 22:12, pmc_scratch84, 31:21),
	s(emc_dll_xform_dqs2, 4:0, pmc_scratch85, 4:0),
	s(emc_dll_xform_dqs2, 22:12, pmc_scratch85, 15:5),
	s(emc_dll_xform_dqs3, 4:0, pmc_scratch85, 20:16),
	s(emc_dll_xform_dqs3, 22:12, pmc_scratch85, 31:21),
	s(emc_dll_xform_dqs4, 4:0, pmc_scratch86, 4:0),
	s(emc_dll_xform_dqs4, 22:12, pmc_scratch86, 15:5),
	s(emc_dll_xform_dqs5, 4:0, pmc_scratch86, 20:16),
	s(emc_dll_xform_dqs5, 22:12, pmc_scratch86, 31:21),
	s(emc_dll_xform_dqs6, 4:0, pmc_scratch87, 4:0),
	s(emc_dll_xform_dqs6, 22:12, pmc_scratch87, 15:5),
	s(emc_dll_xform_dqs7, 4:0, pmc_scratch87, 20:16),
	s(emc_dll_xform_dqs7, 22:12, pmc_scratch87, 31:21),
	s(emc_dll_xform_dqs8, 4:0, pmc_scratch88, 4:0),
	s(emc_dll_xform_dqs8, 22:12, pmc_scratch88, 15:5),
	s(emc_dll_xform_dqs9, 4:0, pmc_scratch88, 20:16),
	s(emc_dll_xform_dqs9, 22:12, pmc_scratch88, 31:21),
	s(emc_dll_xform_dqs10, 4:0, pmc_scratch89, 4:0),
	s(emc_dll_xform_dqs10, 22:12, pmc_scratch89, 15:5),
	s(emc_dll_xform_dqs11, 4:0, pmc_scratch89, 20:16),
	s(emc_dll_xform_dqs11, 22:12, pmc_scratch89, 31:21),
	s(emc_dll_xform_dqs12, 4:0, pmc_scratch90, 4:0),
	s(emc_dll_xform_dqs12, 22:12, pmc_scratch90, 15:5),
	s(emc_dll_xform_dqs13, 4:0, pmc_scratch90, 20:16),
	s(emc_dll_xform_dqs13, 22:12, pmc_scratch90, 31:21),
	s(emc_dll_xform_dqs14, 4:0, pmc_scratch91, 4:0),
	s(emc_dll_xform_dqs14, 22:12, pmc_scratch91, 15:5),
	s(emc_dll_xform_dqs15, 4:0, pmc_scratch91, 20:16),
	s(emc_dll_xform_dqs15, 22:12, pmc_scratch91, 31:21),
	s(emc_dll_xform_quse0, 4:0, pmc_scratch92, 4:0),
	s(emc_dll_xform_quse0, 22:12, pmc_scratch92, 15:5),
	s(emc_dll_xform_quse1, 4:0, pmc_scratch92, 20:16),
	s(emc_dll_xform_quse1, 22:12, pmc_scratch92, 31:21),
	s(emc_dll_xform_quse2, 4:0, pmc_scratch93, 4:0),
	s(emc_dll_xform_quse2, 22:12, pmc_scratch93, 15:5),
	s(emc_dll_xform_quse3, 4:0, pmc_scratch93, 20:16),
	s(emc_dll_xform_quse3, 22:12, pmc_scratch93, 31:21),
	s(emc_dll_xform_quse4, 4:0, pmc_scratch94, 4:0),
	s(emc_dll_xform_quse4, 22:12, pmc_scratch94, 15:5),
	s(emc_dll_xform_quse5, 4:0, pmc_scratch94, 20:16),
	s(emc_dll_xform_quse5, 22:12, pmc_scratch94, 31:21),
	s(emc_dll_xform_quse6, 4:0, pmc_scratch95, 4:0),
	s(emc_dll_xform_quse6, 22:12, pmc_scratch95, 15:5),
	s(emc_dll_xform_quse7, 4:0, pmc_scratch95, 20:16),
	s(emc_dll_xform_quse7, 22:12, pmc_scratch95, 31:21),
	s(emc_dll_xform_dq0, 4:0, pmc_scratch96, 4:0),
	s(emc_dll_xform_dq0, 22:12, pmc_scratch96, 15:5),
	s(emc_dll_xform_dq1, 4:0, pmc_scratch96, 20:16),
	s(emc_dll_xform_dq1, 22:12, pmc_scratch96, 31:21),
	s(emc_dll_xform_dq2, 4:0, pmc_scratch97, 4:0),
	s(emc_dll_xform_dq2, 22:12, pmc_scratch97, 15:5),
	s(emc_dll_xform_dq3, 4:0, pmc_scratch97, 20:16),
	s(emc_dll_xform_dq3, 22:12, pmc_scratch97, 31:21),
	s(emc_pre_refresh_req_cnt, 15:0, pmc_scratch98, 15:0),
	s(emc_dll_xform_addr0, 4:0, pmc_scratch98, 20:16),
	s(emc_dll_xform_addr0, 22:12, pmc_scratch98, 31:21),
	s(emc_dll_xform_addr1, 4:0, pmc_scratch99, 4:0),
	s(emc_dll_xform_addr1, 22:12, pmc_scratch99, 15:5),
	s(emc_dll_xform_addr2, 4:0, pmc_scratch99, 20:16),
	s(emc_dll_xform_addr2, 22:12, pmc_scratch99, 31:21),
	s(emc_dll_xform_addr3, 4:0, pmc_scratch100, 4:0),
	s(emc_dll_xform_addr3, 22:12, pmc_scratch100, 15:5),
	s(emc_dll_xform_addr4, 4:0, pmc_scratch100, 20:16),
	s(emc_dll_xform_addr4, 22:12, pmc_scratch100, 31:21),
	s(emc_dll_xform_addr5, 4:0, pmc_scratch101, 4:0),
	s(emc_dll_xform_addr5, 22:12, pmc_scratch101, 15:5),
	s(emc_pchg2pden, 5:0, pmc_scratch102, 5:0),
	s(emc_act2pden, 5:0, pmc_scratch102, 11:6),
	s(emc_rw2pden, 5:0, pmc_scratch102, 17:12),
	s(emc_tcke, 5:0, pmc_scratch102, 23:18),
	s(emc_trpab, 5:0, pmc_scratch102, 29:24),
	s(emc_fbio_cfg5, 8:7, pmc_scratch102, 31:30),
	s(emc_ctt, 5:0, pmc_scratch103, 5:0),
	s(emc_einput, 5:0, pmc_scratch103, 11:6),
	s(emc_puterm_extra, 21:16, pmc_scratch103, 17:12),
	s(emc_tckesr, 5:0, pmc_scratch103, 23:18),
	s(emc_tpd, 5:0, pmc_scratch103, 29:24),
	s(emc_fbio_cfg5, 10:9, pmc_scratch103, 31:30),
	s(emc_rdv_mask, 5:0, pmc_scratch104, 5:0),
	s(emc_xm2cmdpadctrl4, 0:0, pmc_scratch104, 6:6),
	s(emc_xm2cmdpadctrl4, 2:2, pmc_scratch104, 7:7),
	s(emc_xm2cmdpadctrl4, 4:4, pmc_scratch104, 8:8),
	s(emc_xm2cmdpadctrl4, 6:6, pmc_scratch104, 9:9),
	s(emc_xm2cmdpadctrl4, 8:8, pmc_scratch104, 10:10),
	s(emc_xm2cmdpadctrl4, 10:10, pmc_scratch104, 11:11),
	s(emc_qpop, 5:0, pmc_scratch104, 17:12),
	s(mc_emem_arb_timing_rcd, 5:0, pmc_scratch104, 23:18),
	s(mc_emem_arb_timing_ras, 5:0, pmc_scratch104, 29:24),
	s(emc_fbio_cfg5, 12:11, pmc_scratch104, 31:30),
	s(mc_emem_arb_timing_rap2pre, 5:0, pmc_scratch105, 5:0),
	s(mc_emem_arb_timing_r2w, 5:0, pmc_scratch105, 11:6),
	s(mc_emem_arb_timing_w2r, 5:0, pmc_scratch105, 17:12),
	s(emc_ibdly, 4:0, pmc_scratch105, 22:18),
	s(mc_emem_arb_timing_r2r, 4:0, pmc_scratch105, 27:23),
	s(emc_w2w, 3:0, pmc_scratch105, 31:28),
	s(mc_emem_arb_timing_w2w, 4:0, pmc_scratch106, 4:0),
	s(mc_emem_arb_override, 27:27, pmc_scratch106, 5:5),
	s(mc_emem_arb_override, 26:26, pmc_scratch106, 6:6),
	s(mc_emem_arb_override, 16:16, pmc_scratch106, 7:7),
	s(mc_emem_arb_override, 10:10, pmc_scratch106, 8:8),
	s(mc_emem_arb_override, 4:4, pmc_scratch106, 9:9),
	s(emc_wdv_mask, 3:0, pmc_scratch106, 13:10),
	s(emc_ctt_duration, 3:0, pmc_scratch106, 17:14),
	s(emc_quse_width, 3:0, pmc_scratch106, 21:18),
	s(emc_puterm_width, 3:0, pmc_scratch106, 25:22),
	s(emc_bgbias_ctl0, 3:0, pmc_scratch106, 29:26),
	s(emc_fbio_cfg5, 25:24, pmc_scratch106, 31:30),
	s(mc_emem_arb_timing_rrd, 3:0, pmc_scratch107, 3:0),
	s(emc_fbio_cfg5, 23:20, pmc_scratch107, 10:7),
	s(emc_fbio_cfg5, 15:13, pmc_scratch107, 13:11),
	s(emc_cfg2, 5:3, pmc_scratch107, 16:14),
	s(emc_fbio_cfg5, 26:26, pmc_scratch107, 17:17),
	s(emc_fbio_cfg5, 28:28, pmc_scratch107, 18:18),
	s(emc_cfg2, 2:0, pmc_scratch107, 21:19),
	s(emc_cfg2, 7:6, pmc_scratch107, 23:22),
	s(emc_cfg2, 15:10, pmc_scratch107, 29:24),
	s(emc_cfg2, 23:22, pmc_scratch107, 31:30),
	s(emc_cfg2, 25:24, pmc_scratch108, 1:0),
	s(emc_cfg2, 31:28, pmc_scratch108, 5:2),
	s(bootrom_patch_data, 31:0, pmc_scratch15, 31:0),
	s(bootrom_patch_control, 31:0, pmc_scratch16, 31:0),
	s(emc_dev_select, 1:0, pmc_scratch17, 31:30),
	s(emc_zcal_warmcoldboot_enables, 1:0, pmc_scratch18, 31:30),
	s(emc_cfg_dig_dll_period_warmboot, 1:0, pmc_scratch19, 31:30),
	s(emc_warmboot_extra_modereg_write_en, 0:0, pmc_scratch46, 30:30),
	s(mc_clken_override_all_warmboot, 0:0, pmc_scratch46, 31:31),
	s(emc_clken_override_all_warmboot, 0:0, pmc_scratch47, 30:30),
	s(emc_mrs_warmboot_enable, 0:0, pmc_scratch47, 31:31),
	s(emc_timing_control_wait, 7:0, pmc_scratch57, 27:20),
	s(emc_zcal_warmboot_wait, 7:0, pmc_scratch58, 27:20),
	s(emc_auto_cal_wait, 7:0, pmc_scratch59, 27:20),
	s(warmboot_wait, 7:0, pmc_scratch60, 27:20),
	s(emc_pin_program_wait, 7:0, pmc_scratch61, 27:20),
	s(ahb_arb_xbar_ctrl_mem_init_done, 0:0, pmc_scratch79, 30:30),
	s(emc_extra_refresh_num, 2:0, pmc_scratch81, 31:29),
	s(swizzle_rank_byte_encode, 15:0, pmc_scratch101, 31:16),
	s(memory_type, 2:0, pmc_scratch107, 6:4),
};

struct pack_fields pack_list_ddr3[] = {
	s(emc_mrs, 13:0, pmc_scratch5, 13:0),
	s(emc_emrs, 13:0, pmc_scratch5, 27:14),
	s(emc_mrs, 21:20, pmc_scratch5, 29:28),
	s(emc_mrs, 31:30, pmc_scratch5, 31:30),
	s(emc_emrs2, 13:0, pmc_scratch7, 13:0),
	s(emc_emrs, 21:20, pmc_scratch7, 15:14),
	s(emc_emrs, 31:30, pmc_scratch7, 17:16),
	s(emc_emrs2, 21:20, pmc_scratch7, 19:18),
	s(emc_emrs3, 13:0, pmc_scratch8, 13:0),
	s(emc_emrs2, 31:30, pmc_scratch8, 15:14),
	s(emc_emrs3, 21:20, pmc_scratch8, 17:16),
	s(emc_emrs3, 31:30, pmc_scratch8, 19:18),
	s(emc_warmboot_mrs_extra, 13:0, pmc_scratch9, 13:0),
	s(emc_warmboot_mrs_extra, 31:30, pmc_scratch9, 15:14),
	s(emc_warmboot_mrs_extra, 21:20, pmc_scratch9, 17:16),
	s(emc_zq_cal_ddr3_warmboot, 31:30, pmc_scratch9, 19:18),
	s(emc_mrs, 27:26, pmc_scratch10, 1:0),
	s(emc_emrs, 27:26, pmc_scratch10, 3:2),
	s(emc_emrs2, 27:26, pmc_scratch10, 5:4),
	s(emc_emrs3, 27:26, pmc_scratch10, 7:6),
	s(emc_warmboot_mrs_extra, 27:27, pmc_scratch10, 8:8),
	s(emc_warmboot_mrs_extra, 26:26, pmc_scratch10, 9:9),
	s(emc_zq_cal_ddr3_warmboot, 0:0, pmc_scratch10, 10:10),
	s(emc_zq_cal_ddr3_warmboot, 4:4, pmc_scratch10, 11:11),
	c(0, 31:0, pmc_scratch116, 31:0),
	c(0, 31:0, pmc_scratch117, 31:0),
};

struct pack_fields pack_list_lpddr2[] = {
	s(emc_mrw_lpddr2_zcal_warmboot, 23:16, pmc_scratch5, 7:0),
	s(emc_mrw_lpddr2_zcal_warmboot, 7:0, pmc_scratch5, 15:8),
	s(emc_warmboot_mrw_extra, 23:16, pmc_scratch5, 23:16),
	s(emc_warmboot_mrw_extra, 7:0, pmc_scratch5, 31:24),
	s(emc_mrw_lpddr2_zcal_warmboot, 31:30, pmc_scratch6, 1:0),
	s(emc_warmboot_mrw_extra, 31:30, pmc_scratch6, 3:2),
	s(emc_mrw_lpddr2_zcal_warmboot, 27:26, pmc_scratch6, 5:4),
	s(emc_warmboot_mrw_extra, 27:26, pmc_scratch6, 7:6),
	s(emc_mrw1, 7:0, pmc_scratch7, 7:0),
	s(emc_mrw1, 23:16, pmc_scratch7, 15:8),
	s(emc_mrw1, 27:26, pmc_scratch7, 17:16),
	s(emc_mrw1, 31:30, pmc_scratch7, 19:18),
	s(emc_mrw2, 7:0, pmc_scratch8, 7:0),
	s(emc_mrw2, 23:16, pmc_scratch8, 15:8),
	s(emc_mrw2, 27:26, pmc_scratch8, 17:16),
	s(emc_mrw2, 31:30, pmc_scratch8, 19:18),
	s(emc_mrw3, 7:0, pmc_scratch9, 7:0),
	s(emc_mrw3, 23:16, pmc_scratch9, 15:8),
	s(emc_mrw3, 27:26, pmc_scratch9, 17:16),
	s(emc_mrw3, 31:30, pmc_scratch9, 19:18),
	s(emc_mrw4, 7:0, pmc_scratch10, 7:0),
	s(emc_mrw4, 23:16, pmc_scratch10, 15:8),
	s(emc_mrw4, 27:26, pmc_scratch10, 17:16),
	s(emc_mrw4, 31:30, pmc_scratch10, 19:18),
};

struct pack_fields pack_list_2[] = {
	s(mc_video_protect_gpu_override0, 31:0, pmc_secure_scratch8, 31:0),
	s(mc_video_protect_vpr_override, 3:0, pmc_secure_scratch9, 3:0),
	s(mc_video_protect_vpr_override, 11:6, pmc_secure_scratch9, 9:4),
	s(mc_video_protect_vpr_override, 23:14, pmc_secure_scratch9, 19:10),
	s(mc_video_protect_vpr_override, 26:26, pmc_secure_scratch9, 20:20),
	s(mc_video_protect_vpr_override, 31:29, pmc_secure_scratch9, 23:21),
	s(emc_fbio_cfg5, 19:16, pmc_secure_scratch9, 27:24),
	s(mc_display_snap_ring, 1:0, pmc_secure_scratch9, 29:28),
	s(mc_display_snap_ring, 31:31, pmc_secure_scratch9, 30:30),
	s(emc_adr_cfg, 0:0, pmc_secure_scratch9, 31:31),
	s(mc_video_protect_gpu_override1, 15:0, pmc_secure_scratch10, 15:0),
	s(mc_emem_adr_cfg_bank_mask0, 15:0, pmc_secure_scratch10, 31:16),
	s(mc_emem_adr_cfg_bank_mask1, 15:0, pmc_secure_scratch11, 15:0),
	s(mc_emem_adr_cfg_bank_mask2, 15:0, pmc_secure_scratch11, 31:16),
	s(mc_emem_cfg, 13:0, pmc_secure_scratch12, 13:0),
	s(mc_emem_cfg, 31:31, pmc_secure_scratch12, 14:14),
	s(mc_video_protect_bom, 31:20, pmc_secure_scratch12, 26:15),
	s(mc_video_protect_vpr_override1, 1:0, pmc_secure_scratch12, 28:27),
	s(mc_video_protect_vpr_override1, 4:4, pmc_secure_scratch12, 29:29),
	s(mc_video_protect_bom_adr_hi, 1:0, pmc_secure_scratch12, 31:30),
	s(mc_video_protect_size_mb, 11:0, pmc_secure_scratch13, 11:0),
	s(mc_sec_carveout_bom, 31:20, pmc_secure_scratch13, 23:12),
	s(mc_emem_adr_cfg_bank_swizzle3, 2:0, pmc_secure_scratch13, 26:24),
	s(mc_video_protect_write_access, 1:0, pmc_secure_scratch13, 28:27),
	s(mc_sec_carveout_adr_hi, 1:0, pmc_secure_scratch13, 30:29),
	s(mc_emem_adr_cfg, 0:0, pmc_secure_scratch13, 31:31),
	s(mc_sec_carveout_size_mb, 11:0, pmc_secure_scratch14, 11:0),
	s(mc_mts_carveout_bom, 31:20, pmc_secure_scratch14, 23:12),
	s(mc_mts_carveout_adr_hi, 1:0, pmc_secure_scratch14, 25:24),
	s(mc_sec_carveout_protect_write_access, 0:0, pmc_secure_scratch14, 26:26),
	s(mc_mts_carveout_reg_ctrl, 0:0, pmc_secure_scratch14, 27:27),
	s(mc_mts_carveout_size_mb, 11:0, pmc_secure_scratch15, 11:0),
	s(mc_emem_adr_cfg_dev0, 2:0, pmc_secure_scratch15, 14:12),
	s(mc_emem_adr_cfg_dev0, 9:8, pmc_secure_scratch15, 16:15),
	s(mc_emem_adr_cfg_dev0, 19:16, pmc_secure_scratch15, 20:17),
	s(mc_emem_adr_cfg_dev1, 2:0, pmc_secure_scratch15, 23:21),
	s(mc_emem_adr_cfg_dev1, 9:8, pmc_secure_scratch15, 25:24),
	s(mc_emem_adr_cfg_dev1, 19:16, pmc_secure_scratch15, 29:26),
	c(0x1555555, 25:0, pmc_sec_disable2, 25:0),
	c(0xff, 7:0, pmc_sec_disable, 19:12),
	c(0, 31:0, pmc_scratch2, 31:0),
	m(pllm_base, 15:0, pmc_scratch2, 15:0),
	m(pllm_base, 20:20, pmc_scratch2, 16:16),
	m(pllm_misc2, 2:0, pmc_scratch2, 19:17),
	c(0, 31:0, pmc_scratch35, 31:0),
	m(pllm_misc1, 23:0, pmc_scratch35, 23:0),
	m(pllm_misc1, 30:28, pmc_scratch35, 30:28),
	c(0, 31:0, pmc_scratch3, 31:0),
	s(pllm_input_divider, 7:0, pmc_scratch3, 7:0),
	c(0x3e, 7:0, pmc_scratch3, 15:8),
	c(0, 3:0, pmc_scratch3, 19:16),
	s(pllm_kvco, 0:0, pmc_scratch3, 20:20),
	s(pllm_kcp, 1:0, pmc_scratch3, 22:21),
	c(0, 31:0, pmc_scratch36, 31:0),
	s(pllm_setup_control, 23:0, pmc_scratch36, 23:0),
	c(0, 31:0, pmc_scratch4, 31:0),
	s(pllm_stable_time, 9:0, pmc_scratch4, 9:0),
	s(pllm_stable_time, 9:0, pmc_scratch4, 19:10),
	s(pllm_select_div2, 0:0, pmc_pllm_wb0_override2, 27:27),
	c(1, 0:0, pmc_pllp_wb0_override, 11:11),
};

void do_encode(struct encode_fields *encode, struct sdram_params *sdram)
{
	u32 val, op1, op2;

	val = readl((char *)sdram + encode->src_offset);
	debug("%s: sdram[%#x] => %#x\n", __func__, encode->src_offset, val);
	op1 = val >> encode->op1_shift;
	op1 &= encode->op1_mask;
	op2 = val >> encode->op2_shift;
	op2 &= encode->op2_mask;

	val = readl((char *)sdram + encode->dst_offset);
	val &= ~(1 << encode->dst_shift);
	if (op1 > op2)
		val |= (1 << encode->dst_shift);
	debug("%s: sdram[%#x] <= %#x\n", __func__, encode->dst_offset, val);
	writel(val, (char *)sdram + encode->dst_offset);
}

void do_encode_list(struct encode_fields *encode_list,  u32 num_list,
		       struct sdram_params *sdram)
{
	struct encode_fields *encode;
	u32 i;

	for (i = 0, encode = encode_list; i < num_list; ++i, ++encode)
		do_encode(encode, sdram);
}

void do_pack(struct pack_fields *pack, struct sdram_params *sdram)
{
	u32 val, offset, type, reg;
	struct clk_rst_ctlr *clkrst = (struct clk_rst_ctlr *)NV_PA_CLK_RST_BASE;

	offset = pack->src_offset;
	type = pack->src_type;

	switch (type) {
	case TYPE_SDRAM:
		val = readl((char *)sdram + offset);
		break;
	case TYPE_PLLM:
		val = readl((char *)&clkrst->crc_pll[CLOCK_ID_MEMORY]
			+ offset);
		break;
	case TYPE_CONST:
		val = offset;
		break;
	default:
		debug("src_type (%u) is not supported\n", type);
		return;
	}

	val >>= pack->src_shift;
	val &= pack->src_mask;
	val <<= pack->dst_shift;

	reg = readl(NV_PA_PMC_BASE + pack->dst_offset);
	reg &= ~(pack->dst_mask << pack->dst_shift);
	reg |= val;
	writel(reg, NV_PA_PMC_BASE + pack->dst_offset);
}

void do_pack_list(struct pack_fields *pack_list, u32 num_list,
		   struct sdram_params *sdram)
{
	struct pack_fields *pack;
	u32 i;

	for (pack = pack_list, i = 0; i < num_list; ++i, ++pack)
		do_pack(pack, sdram);
}

int warmboot_save_sdram_params(void)
{
	u32 ram_code;
	struct sdram_params sdram;
	struct pmc_ctlr *pmc = (struct pmc_ctlr *)NV_PA_PMC_BASE;

	/* get ram code that is used as index to array _params in BCT */
	ram_code = readl(&pmc->pmc_strap_opt_a) >> STRAP_OPT_A_RAM_CODE_SHIFT;
	ram_code &= 3;		/* get last 2 bits */

	memcpy(&sdram,
	       (char *)((struct sdram_params *)SDRAM_PARAMS_BASE + ram_code),
	       sizeof(sdram));

	/* encode BIT6_GT_BIT7 bits in sdram.swizzle_rank_byte_encode */
	do_encode_list(encode_list, ARRAY_SIZE(encode_list), &sdram);

	do_pack_list(pack_list_1, ARRAY_SIZE(pack_list_1), &sdram);

	if (sdram.memory_type == MEMORY_TYPE_LPDDR2)
		do_pack_list(pack_list_lpddr2, ARRAY_SIZE(pack_list_lpddr2),
			     &sdram);
	else if (sdram.memory_type == MEMORY_TYPE_DDR3)
		do_pack_list(pack_list_ddr3, ARRAY_SIZE(pack_list_ddr3),
			     &sdram);

	do_pack_list(pack_list_2, ARRAY_SIZE(pack_list_2), &sdram);

	return 0;
}

/*
 * NOTE: If more than one of the following is enabled, only one of them will
 *	 actually be used. RANDOM takes precedence over PATTERN and ZERO, and
 *	 PATTERN takes precedence overy ZERO.
 *
 *	 RANDOM_AES_BLOCK_IS_PATTERN is to define a 32-bit PATTERN.
 */
#undef RANDOM_AES_BLOCK_IS_RANDOM	/* to randomize the header */
#undef RANDOM_AES_BLOCK_IS_PATTERN	/* to patternize the header */
#define RANDOM_AES_BLOCK_IS_ZERO	/* to clear the header */

static u32 get_major_version(void)
{
	u32 major_id;
	struct apb_misc_gp_ctlr *gp =
		(struct apb_misc_gp_ctlr *)NV_PA_APB_MISC_GP_BASE;

	major_id = (readl(&gp->hidrev) & HIDREV_MAJORPREV_MASK) >>
			HIDREV_MAJORPREV_SHIFT;
	return major_id;
}

static int is_production_mode_fuse_set(struct fuse_regs *fuse)
{
	return readl(&fuse->production_mode);
}

static int is_odm_production_mode_fuse_set(struct fuse_regs *fuse)
{
	return readl(&fuse->security_mode);
}

static int is_failure_analysis_mode(struct fuse_regs *fuse)
{
	return readl(&fuse->fa);
}

static int is_odm_production_mode(void)
{
	struct fuse_regs *fuse = (struct fuse_regs *)NV_PA_FUSE_BASE;

	if (!is_failure_analysis_mode(fuse) &&
	    is_odm_production_mode_fuse_set(fuse))
		return 1;
	else
		return 0;
}

static int is_production_mode(void)
{
	struct fuse_regs *fuse = (struct fuse_regs *)NV_PA_FUSE_BASE;

	if (get_major_version() == 0)
		return 1;

	if (!is_failure_analysis_mode(fuse) &&
	    is_production_mode_fuse_set(fuse) &&
	    !is_odm_production_mode_fuse_set(fuse))
		return 1;
	else
		return 0;
}

static enum fuse_operating_mode fuse_get_operation_mode(void)
{
	u32 chip_id;
	struct apb_misc_gp_ctlr *gp =
		(struct apb_misc_gp_ctlr *)NV_PA_APB_MISC_GP_BASE;

	chip_id = readl(&gp->hidrev);
	chip_id = (chip_id & HIDREV_CHIPID_MASK) >> HIDREV_CHIPID_SHIFT;
	if (chip_id == CHIPID_TEGRA124) {
		if (is_odm_production_mode()) {
			printf("!! odm_production_mode is not supported !!\n");
			return MODE_UNDEFINED;
		} else {
			if (is_production_mode())
				return MODE_PRODUCTION;
			else
				return MODE_PREPRODUCTION;
		}
	}
	return MODE_UNDEFINED;
}

#if defined(RANDOM_AES_BLOCK_IS_RANDOM)
/* Currently, this routine returns a 32-bit all 0 seed. */
static u32 query_random_seed(void)
{
	return 0;
}
#endif

static void determine_crypto_options(int *is_encrypted, int *is_signed,
				     int *use_zero_key)
{
	switch (fuse_get_operation_mode()) {
	case MODE_PREPRODUCTION:
	case MODE_PRODUCTION:
		*is_encrypted = 0;
		*is_signed = 1;
		*use_zero_key = 1;
		break;
	case MODE_UNDEFINED:
	default:
		*is_encrypted = 0;
		*is_signed = 0;
		*use_zero_key  = 0;
		break;
	}
}

static int sign_wb_code(u32 start, u32 length, int use_zero_key)
{
	int err;
	u8 *source;		/* Pointer to source */
	u8 *hash;

	/* Calculate AES block parameters. */
	source = (u8 *)(start + offsetof(struct wb_header, random_aes_block));
	length -= offsetof(struct wb_header, random_aes_block);
	hash = (u8 *)(start + offsetof(struct wb_header, hash_signature.hash));
	err = sign_data_block(source, length, hash);

	return err;
}

int warmboot_prepare_code(u32 seg_address, u32 seg_length)
{
	int err = 0;
	u32 length;			/* length of the signed/encrypt code */
	struct wb_header *dst_header;	/* Pointer to dest WB header */
	int is_encrypted;		/* Segment is encrypted */
	int is_signed;			/* Segment is signed */
	int use_zero_key;		/* Use key of all zeros */

	/* Determine crypto options. */
	determine_crypto_options(&is_encrypted, &is_signed, &use_zero_key);

	/* Get the actual code limits. */
	length = roundup(((u32)wb_end - (u32)wb_start), 16);

	/*
	 * The region specified by seg_address must not be in IRAM and must be
	 * nonzero in length.
	 */
	if ((seg_length == 0) || (seg_address == 0) ||
	    ((seg_address >= NV_PA_BASE_SRAM) &&
	     (seg_address < (NV_PA_BASE_SRAM + NV_PA_BASE_SRAM_SIZE)))) {
		err = -EFAULT;
		goto fail;
	}

	/* Things must be 16-byte aligned. */
	if ((seg_length & (TEGRA_LP0_ALIGN - 1)) ||
	    (seg_address & (TEGRA_LP0_ALIGN - 1))) {
		err = -EINVAL;
		goto fail;
	}

	/* Will the code fit? (destination includes wb_header + wb code) */
	if (seg_length < (length + sizeof(struct wb_header))) {
		err = -EINVAL;
		goto fail;
	}

	dst_header = (struct wb_header *)seg_address;
	memset((char *)dst_header, 0, sizeof(struct wb_header));

	/* Populate the random_aes_block as requested. */
	{
		u32 *aes_block = (u32 *)&(dst_header->random_aes_block);
		u32 *end = (u32 *)(((u32)aes_block) +
				   sizeof(dst_header->random_aes_block));

		do {
#if defined(RANDOM_AES_BLOCK_IS_RANDOM)
			*aes_block++ = query_random_seed();
#elif defined(RANDOM_AES_BLOCK_IS_PATTERN)
			*aes_block++ = RANDOM_AES_BLOCK_IS_PATTERN;
#elif defined(RANDOM_AES_BLOCK_IS_ZERO)
			*aes_block++ = 0;
#else
			printf("None of RANDOM_AES_BLOCK_IS_XXX is defined; ");
			printf("Default to pattern 0.\n");
			*aes_block++ = 0;
#endif
		} while (aes_block < end);
	}

	/* Populate the header. */
	dst_header->length_in_secure = length + sizeof(struct wb_header);
	dst_header->length_secure = length + sizeof(struct wb_header);
	dst_header->destination = NV_WB_RUN_ADDRESS;
	dst_header->entry_point = NV_WB_RUN_ADDRESS;
	dst_header->code_length = length;

	if (is_encrypted) {
		printf("!!!! Encryption is not supported !!!!\n");
		dst_header->length_in_secure = 0;
		err = -EACCES;
		goto fail;
	} else {
		/* copy the wb code directly following dst_header. */
		memcpy((char *)(dst_header+1), (char *)wb_start, length);
	}

	if (is_signed)
		err = sign_wb_code(seg_address, dst_header->length_in_secure,
				   use_zero_key);

fail:
	if (err)
		printf("WB code not copied to LP0 location! (error=%d)\n", err);

	return err;
}

/*
 * Delete all substrings which begin with 'str2' from 'str1' string.
 *
 * The substring to be deleted begins with the 'str2' till the space char. that
 * seperates this substring and the next substring. If there is no space after
 * the substring, only the substring is deleted.
 */
static void str_delete(char *str1, char *str2)
{
	char *found;
	char *next;

	while (1) {
		/* find a substring which begins with 'str2' in 'str1' */
		found = strstr(str1, str2);
		if (found == NULL)
			return;

		/* find the next substring, separated by a space char. */
		next = strchr(found, ' ');
		if (next) {
			/* found a next substring, replace the 'str2' with it */
			strcpy(found, next + 1);
		} else {
			/* no next substring, terminate the 'str1' and return */
			*(found - 1) = '\0';
			return;
		}
	}
}

/*
 * Append "lp0_vec=<len>@<addr>" substring to extra_bootargs env. variable.
 *
 * If there are "lp0_vec=XXX" substrings in the extra_bootargs variable, they
 * are deleted first, then "lp0_vec=XXX" substring is added.
 */
int warmboot_set_lp0_vec(u32 seg_address, u32 seg_length)
{
	char *args;
	int len;

	args = getenv("extra_bootargs");

	/* delete all "lp0_vec=XXX " occurrences */
	str_delete(args, "lp0_vec=");

	len = strlen(args);

	char new_args[len + 32];

	strcpy(new_args, args);

	/* append the "lp0_vec=" to the end of extra_bootargs */
	sprintf(&new_args[len], " lp0_vec=%#x@%#x", seg_length, seg_address);
	setenv("extra_bootargs", new_args);

	return 0;
}
