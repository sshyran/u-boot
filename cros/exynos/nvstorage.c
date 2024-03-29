/*
 * Copyright (c) 2012 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

#include <common.h>
#include <cros/common.h>
#include <vboot_api.h>

VbError_t VbExNvStorageRead(uint8_t* buf)
{
	/* TODO(chromium-os:28077) Implement VbExNvStorageRead */
	return VBERROR_SUCCESS;
}

VbError_t VbExNvStorageWrite(const uint8_t* buf)
{
	/* TODO(chromium-os:28077) Implement VbExNvStorageWrite */
	return VBERROR_SUCCESS;
}
