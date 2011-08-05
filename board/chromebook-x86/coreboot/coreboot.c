/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * (C) Copyright 2008
 * Graeme Russ, graeme.russ@gmail.com.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/u-boot-i386.h>
#include <cbfs.h>
#include <common.h>
#include <config.h>
#include <flash.h>
#include <malloc.h>
#include <netdev.h>
#include <asm/ic/coreboot/tables.h>
#include <asm/ic/coreboot/sysinfo.h>
#include <chromeos/power_management.h>

#ifdef CONFIG_HW_WATCHDOG
#include <watchdog.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

unsigned long monitor_flash_len = CONFIG_SYS_MONITOR_LEN;


/*
 * Miscellaneous platform dependent initializations
 */
int cpu_init_f(void)
{
	int ret = get_coreboot_info(&lib_sysinfo);
	if (ret != 0) {
		printf("Failed to parse coreboot tables.\n");
	}
	return ret;
}

int board_early_init_f(void)
{
	return 0;
}

int board_early_init_r(void)
{
#if defined CONFIG_CMD_CBFS && defined CONFIG_OF_CONTROL
	CbfsFile file;
	void *dtb;
	u32 size;

	file_cbfs_init(0xffffffff);
	if (file_cbfs_result != CBFS_SUCCESS) {
		printf("%s.\n", file_cbfs_error());
		goto cbfs_failed;
	}
	file = file_cbfs_find("u-boot.dtb");
	if (!file) {
		if (file_cbfs_result != CBFS_FILE_NOT_FOUND)
			printf("%s.\n", file_cbfs_error());
		goto cbfs_failed;
	}
	size = file_cbfs_size(file);
	if (file_cbfs_result != CBFS_SUCCESS) {
		printf("%s.\n", file_cbfs_error());
		goto cbfs_failed;
	}
	dtb = malloc(size);
	if (!dtb) {
		printf("Bad allocation!\n");
		goto cbfs_failed;
	}
	if (size != file_cbfs_read(file, dtb, size)) {
		free(dtb);
		printf("%s.\n", file_cbfs_error());
		goto cbfs_failed;
	}
	gd->blob = dtb;
cbfs_failed:
#endif /* CONFIG_CMD_CBFS && CONFIG_OF_CONTROL */
	return 0;
}

void show_boot_progress(int val)
{
}


int last_stage_init(void)
{
	return 0;
}

#ifndef CONFIG_SYS_NO_FLASH
ulong board_flash_get_legacy(ulong base, int banknum, flash_info_t *info)
{
	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	return pci_eth_init(bis);
}

void setup_pcat_compatibility()
{
}

#ifdef CONFIG_HW_WATCHDOG
void hw_watchdog_reset(void)
{
}
#endif

int do_coldboot(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	cold_reboot();
	return (0);
}

U_BOOT_CMD(coldboot, 1, 1, do_coldboot, "Initiate a cold reboot.", "");

int do_poweroff(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	power_off();
	return (0);
}

U_BOOT_CMD(poweroff, 1, 1, do_poweroff, "Switch off power", "");

