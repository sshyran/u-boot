/*
 * Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#include <common.h>
#include <asm/arch/spl.h>

struct spl_hash *spl_extract_hash(void *hdr_base)
{
	struct spl_var_size_header *hdr;
	struct spl_hash *hash;

	hdr = (struct spl_var_size_header *)hdr_base;
	if (!hdr->hash_offset)
		return NULL;

	/* Find the hash block- we only support SHA256 */
	hash = (struct spl_hash *)(hdr_base + hdr->hash_offset);
	if (hash->signature != SPL_HASH_SIGNATURE ||
	    hash->version != SPL_HASH_VERSION ||
	    hash->size < sizeof(*hash) ||
	    strncmp(hash->algo, "sha256", 6))
		return NULL;

	return hash;
}
