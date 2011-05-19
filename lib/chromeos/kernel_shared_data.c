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
#include <chromeos/common.h>
#include <chromeos/gpio.h>
#include <chromeos/kernel_shared_data.h>

/* This is used to keep u-boot and kernel in sync */
#define SHARED_MEM_VERSION 1
#define SHARED_MEM_SIGNATURE "CHROMEOS"


KernelSharedDataType *get_kernel_shared_data(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	return (KernelSharedDataType *)(
		gd->bd->bi_dram[CONFIG_NR_DRAM_BANKS-1].start +
		gd->bd->bi_dram[CONFIG_NR_DRAM_BANKS-1].size - SZ_1M);
}

void initialize_kernel_shared_data(void)
{
	KernelSharedDataType *sd = get_kernel_shared_data();

	memset(sd, '\0', sizeof(*sd));

	strncpy((char*) sd->signature, SHARED_MEM_SIGNATURE,
		sizeof(sd->signature));

	sd->version = SHARED_MEM_VERSION;

	sd->write_protect_sw = is_firmware_write_protect_gpio_asserted();
	sd->recovery_sw = is_recovery_mode_gpio_asserted();
	sd->developer_sw = is_developer_mode_gpio_asserted();

	/*
	 * these two will be overwritten by next stages or show up as is in
	 * legacy mode
	 */
	strncpy((char*)sd->fwid, version_string, sizeof(sd->fwid));
	strncpy((char*)sd->frid, version_string, sizeof(sd->frid));

	sd->vbnv[0] = 0;
	sd->vbnv[1] = VBNV_BLOCK_SIZE;

	sd->total_size = sizeof(*sd);
}
