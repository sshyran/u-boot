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
#include <malloc.h>
#ifdef CONFIG_MMC
#include <mmc.h>
#endif
#include <part.h>
#ifdef CONFIG_USB_STORAGE
#include <usb.h>
#endif
#include <chromeos/common.h>
#include <chromeos/os_storage.h>
#include <linux/string.h> /* for strcmp */

#include <boot_device.h>

#define PREFIX "os_storage: "

/*
 * MMC dev number of SD card
 * TODO(waihong): Find this number via FDT?
 */
#define MMC_DEV_NUM_SD 1

#define BACKUP_LBA_OFFSET 0x20

static struct {
	block_dev_desc_t *dev_desc;
	ulong offset, limit;
} bootdev_config = {
	.dev_desc = NULL,
	.offset = 0u,
	.limit = 0u
};

block_dev_desc_t *get_bootdev(void)
{
	return bootdev_config.dev_desc;
}

int get_device_number(void)
{
	return bootdev_config.dev_desc->dev;
}

ulong get_offset(void)
{
	return bootdev_config.offset;
}

ulong get_limit(void)
{
	return bootdev_config.limit;
}

uint64_t get_bytes_per_lba(void)
{
	if (!bootdev_config.dev_desc) {
		VBDEBUG(PREFIX "get_bytes_per_lba: not dev_desc set\n");
		return ~0ULL;
	}

	return (uint64_t) bootdev_config.dev_desc->blksz;
}

uint64_t get_ending_lba(void)
{
	uint8_t static_buf[512];
	uint8_t *buf = NULL;
	uint64_t ret = ~0ULL, bytes_per_lba = ~0ULL;

	bytes_per_lba = get_bytes_per_lba();
	if (bytes_per_lba == ~0ULL) {
		VBDEBUG(PREFIX "get_ending_lba: get bytes_per_lba fail\n");
		goto EXIT;
	}

	if (bytes_per_lba > sizeof(static_buf))
		buf = malloc(bytes_per_lba);
	else
		buf = static_buf;

	if (BootDeviceReadLBA(1, 1, buf)) {
		VBDEBUG(PREFIX "get_ending_lba: read primary GPT header fail\n");
		goto EXIT;
	}

	ret = *(uint64_t*) (buf + BACKUP_LBA_OFFSET);

EXIT:
	if (buf != static_buf)
		free(buf);

	return ret;
}

/* TODO(clchiou): This will be deprecated when we fix crosbug:14022 */
static void setup_envvar(char *ifname, int dev)
{
	char buf[32];

	setenv("devtype", ifname);

	if (!strcmp(ifname, "usb")) {
		setenv("devname", "sda");
	} else { /* assert ifname == "mmc" */
		sprintf(buf, "mmcblk%dp", dev);
		setenv("devname", buf);
	}

	sprintf(buf, "%d", dev);
	setenv("devnum", buf);
}

int set_bootdev(char *ifname, int dev, int part)
{
	disk_partition_t part_info;

	if ((bootdev_config.dev_desc = get_dev(ifname, dev)) == NULL) {
		VBDEBUG(PREFIX "block device not supported\n");
		goto cleanup;
	}

	if (part == 0) {
		bootdev_config.offset = 0;
		bootdev_config.limit = bootdev_config.dev_desc->lba;
		setup_envvar(ifname, dev);
		return 0;
	}

	if (get_partition_info(bootdev_config.dev_desc, part, &part_info)) {
		VBDEBUG(PREFIX "cannot find partition\n");
		goto cleanup;
	}

	bootdev_config.offset = part_info.start;
	bootdev_config.limit = part_info.size;
	setup_envvar(ifname, dev);
	return 0;

cleanup:
	bootdev_config.dev_desc = NULL;
	bootdev_config.offset = 0;
	bootdev_config.limit = 0;

	return 1;
}

int BootDeviceReadLBA(uint64_t lba_start, uint64_t lba_count, void *buffer)
{
	block_dev_desc_t *dev_desc;

	if ((dev_desc = bootdev_config.dev_desc) == NULL)
		return 1; /* No boot device configured */

	if (lba_start + lba_count > bootdev_config.limit)
		return 1; /* read out of range */

	if (dev_desc->block_read(dev_desc->dev,
				bootdev_config.offset + lba_start, lba_count,
				buffer) < 0)
		return 1; /* error reading blocks */

	return 0;
}

int BootDeviceWriteLBA(uint64_t lba_start, uint64_t lba_count,
		const void *buffer)
{
	block_dev_desc_t *dev_desc;

	if ((dev_desc = bootdev_config.dev_desc) == NULL)
		return 1; /* No boot device configured */

	if (lba_start + lba_count > bootdev_config.limit)
		return 1; /* read out of range */

	if (dev_desc->block_write(dev_desc->dev,
				bootdev_config.offset + lba_start, lba_count,
				buffer) < 0)
		return 1; /* error reading blocks */

	return 0;
}

#ifdef CONFIG_MMC
int is_mmc_storage_present(int mmc_device_number)
{
	if (mmc_device_number >= MMC_DEV_INSTANCES) {
		VBDEBUG(PREFIX "%d >= MMC_DEV_INSTANCES\n", mmc_device_number);
		return 0;
	}

	/* TODO(waihong): Better way to probe MMC than restart it */
	return initialize_mmc_device(mmc_device_number) == 0;
}
#else
int is_mmc_storage_present(int mmc_device_number)
{
	printf("MMC storage is not enabled\n");
	return 0;
}
#endif

#ifdef CONFIG_USB_STORAGE
int is_usb_storage_present(int usb_controller_number)
{
	if (usb_controller_number >= CONFIG_USB_CONTROLLER_INSTANCES) {
		VBDEBUG(PREFIX "%d >= CONFIG_USB_CONTROLLER_INSTANCES\n",
				usb_controller_number);
		return 0;
	}

	/* TODO(waihong): Better way to probe USB than restart it */
	usb_stop();

#ifdef CONFIG_TEGRA2
	extern int USB_EHCI_TEGRA_BASE_ADDR;
	extern int USB_base_addr[];
	if (!USB_base_addr[usb_controller_number]) {
		VBDEBUG(PREFIX "unknown USB controller: %d\n",
				usb_controller_number);
		return 0;
	}
	USB_EHCI_TEGRA_BASE_ADDR = USB_base_addr[usb_controller_number];
#endif /* CONFIG_TEGRA2 */

	if (usb_init() >= 0) {
		/* Scanning bus for storage devices, mode = 1. */
		return usb_stor_scan(1) == 0;
	}

	return 1;
}
#else
int is_usb_storage_present(int usb_controller_number)
{
	printf("USB storage is not enabled\n");
	return 0;
}
#endif /* CONFIG_USB_STORAGE */

int is_any_storage_device_plugged(int boot_probed_device)
{
	int bus;

#ifdef CONFIG_MMC
	if (is_mmc_storage_present(MMC_DEV_NUM_SD)) {
		if (boot_probed_device == NOT_BOOT_PROBED_DEVICE ||
				!set_bootdev("mmc", MMC_DEV_NUM_SD, 0))
			return 1;
	}
#else
	VBDEBUG(PREFIX "MMC storage is not enabled\n");
#endif

#ifdef CONFIG_USB_STORAGE
	for (bus = 0; bus < CONFIG_USB_CONTROLLER_INSTANCES; bus ++) {
		if (is_usb_storage_present(bus))
			if (boot_probed_device == NOT_BOOT_PROBED_DEVICE ||
					!set_bootdev("usb", 0, 0))
				return 1;
	}
#else
	VBDEBUG(PREFIX "USB storage is not enabled\n");
#endif

	return 0;
}
