/*
 * Copyright (C) 2013 Samsung Electronics
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

#ifndef __ASM_ARM_ARCH_ACE_H_
#define __ASM_ARM_ARCH_ACE_H_

#define ACE_SHA_TYPE_SHA1		1
#define ACE_SHA_TYPE_SHA256		2

/**
 * Computes hash value of input pbuf using ACE
 *
 * @param in_addr	A pointer to the input buffer
 * @param buflen	Byte length of input buffer
 * @param out_addr	A pointer to the output buffer. When complete
 *			32 bytes are copied to pout[0]...pout[31]. Thus, a user
 *			should allocate at least 32 bytes at pOut in advance.
 * @param hash_type SHA1 or SHA256
 *
 * @return		0 on Success, -1 on Failure (Timeout)
 */
int ace_sha_hash_digest(const uchar *in_addr, uint buflen,
			uchar * out_addr, uint hash_type);

#endif
