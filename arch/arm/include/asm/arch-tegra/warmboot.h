/*
 * (C) Copyright 2010 - 2013
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

#ifndef _WARM_BOOT_H_
#define _WARM_BOOT_H_

#include <asm/arch/sdram_param.h>

#define STRAP_OPT_A_RAM_CODE_SHIFT	4
#define STRAP_OPT_A_RAM_CODE_MASK	(0xf << STRAP_OPT_A_RAM_CODE_SHIFT)

/* Defines the supported operating modes */
enum fuse_operating_mode {
	MODE_PREPRODUCTION = 2,
	MODE_PRODUCTION = 3,
	MODE_UNDEFINED,
};

/* Defines the CMAC-AES-128 hash length in 32 bit words. (128 bits = 4 words) */
enum {
	HASH_LENGTH = 4
};

/* Defines the storage for a hash value (128 bits) */
struct hash {
	u32 hash[HASH_LENGTH];
};

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


/**
 * Save warmboot memory settings for a later resume
 *
 * @return 0 if ok, -1 on error
 */
int warmboot_save_sdram_params(void);

int warmboot_prepare_code(u32 seg_address, u32 seg_length);
int sign_data_block(u8 *source, u32 length, u8 *signature);
void wb_start(void);	/* Start of WB assembly code */
void wb_end(void);	/* End of WB assembly code */

/* Common routines for T114 and T124 SOCs */

/*
 * t1x4_wb_save_sdram_params():
 * save sdram parameters to scratch registers so sdram parameters can be
 * restored when system resumes from LP0
 */
int t1x4_wb_save_sdram_params(struct sdram_params *sdram);

/*
 * t1x4_wb_prepare_code():
 * prepare WB code, which will be executed by AVP when system resumes from LP0
 */
int t1x4_wb_prepare_code(u32 tegra_id, u32 seg_address, u32 seg_length);
#endif	/* _WARM_BOOT_H_ */
