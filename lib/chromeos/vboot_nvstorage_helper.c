/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#include <common.h>
#include <chromeos/vboot_nvstorage_helper.h>

/*
 * TODO It should averagely distributed erase/write operation to entire flash
 * memory section allocated for VBNVCONTEXT to increase maximal lifetime.
 *
 * But since VbNvContext gets written infrequently enough, this is likely
 * an overkill.
 */

#define PREFIX "vboot_nvstorage_helper: "

int read_nvcontext(firmware_storage_t *file, VbNvContext *nvcxt)
{
	if (firmware_storage_read(file,
			CONFIG_OFFSET_VBNVCONTEXT, VBNV_BLOCK_SIZE,
			nvcxt->raw)) {
		debug(PREFIX "read_nvcontext fail\n");
		return -1;
	}

	if (VbNvSetup(nvcxt)) {
		debug(PREFIX "setup nvcontext fail\n");
		return -1;
	}

	return 0;
}

int write_nvcontext(firmware_storage_t *file, VbNvContext *nvcxt)
{
	if (firmware_storage_write(file,
			CONFIG_OFFSET_VBNVCONTEXT, VBNV_BLOCK_SIZE,
			nvcxt->raw)) {
		debug(PREFIX "write_nvcontext fail\n");
		return -1;
	}

	return 0;
}
