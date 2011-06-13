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
#include <chromeos/firmware_storage.h>

#if defined CONFIG_CHROMEOS_DEFAULT_FIRMWARE_STORAGE_SPI && \
	defined CONFIG_CHROMEOS_DEFAULT_FIRMWARE_STORAGE_NAND
#error "You can only set one as your default storage"
#endif

#if defined CONFIG_CHROMEOS_DEFAULT_FIRMWARE_STORAGE_NAND
#	ifndef CONFIG_CHROMEOS_FIRMWARE_STORAGE_NAND
#	error "You set NAND as default device, but did not define NAND storage"
#	endif
int (*const firmware_storage_init)(firmware_storage_t *file) =
		firmware_storage_init_nand;

#elif defined CONFIG_CHROMEOS_DEFAULT_FIRMWARE_STORAGE_SPI
#	ifndef CONFIG_CHROMEOS_FIRMWARE_STORAGE_SPI
#	error "You set SPI as default device, but did not define SPI storage"
#	endif
int (*const firmware_storage_init)(firmware_storage_t *file) =
		firmware_storage_init_spi;

#else
#error "No default firmware storage device are set"
#endif

#define PREFIX "firmware_storage_init: "

#ifndef CONFIG_CHROMEOS_FIRMWARE_STORAGE_NAND
int firmware_storage_init_nand(firmware_storage_t *file)
{
	VBDEBUG(PREFIX "NAND is not supported\n");
	return -1;
}
#endif

#ifndef CONFIG_CHROMEOS_FIRMWARE_STORAGE_SPI
int firmware_storage_init_spi(firmware_storage_t *file)
{
	VBDEBUG(PREFIX "SPI is not supported\n");
	return -1;
}
#endif

#ifndef CONFIG_CHROMEOS_FIRMWARE_STORAGE_RAM
int firmware_storage_init_ram(firmware_storage_t *file, void *beg, void *end)
{
	VBDEBUG(PREFIX "RAM is not supported\n");
	return -1;
}
#endif

#undef PREFIX
#define PREFIX "firmware_storage_read: "

int firmware_storage_read(firmware_storage_t *file,
		const off_t offset, const size_t count, void *buf)
{
	ssize_t size;
	size_t remain;

	if (file->seek(file->context, offset, SEEK_SET) < 0) {
		VBDEBUG(PREFIX "seek to address 0x%08x fail\n", offset);
		return -1;
	}

	size = 0;
	remain = count;
	while (remain > 0 &&
			(size = file->read(file->context, buf, remain)) > 0) {
		remain -= size;
		buf += size;
	}

	if (size < 0) {
		VBDEBUG(PREFIX "an error occur when read firmware: %ld\n", size);
		return -1;
	}

	if (remain > 0) {
		VBDEBUG(PREFIX "cannot read all data: %ld\n", remain);
		return -1;
	}

	return 0;
}

#undef PREFIX
#define PREFIX "firmware_storage_write: "

int firmware_storage_write(firmware_storage_t *file,
		const off_t offset, const size_t count, const void *buf)
{
	ssize_t size;
	size_t remain;

	if (file->seek(file->context, offset, SEEK_SET) < 0) {
		VBDEBUG(PREFIX "seek to address 0x%08x fail\n", offset);
		return -1;
	}

	size = 0;
	remain = count;
	while (remain > 0 &&
			(size = file->write(file->context, buf, remain)) > 0) {
		remain -= size;
		buf += size;
	}

	if (size < 0) {
		VBDEBUG(PREFIX "an error occur when write firmware: %ld\n", size);
		return -1;
	}

	if (remain > 0) {
		VBDEBUG(PREFIX "cannot write all data: %ld\n", remain);
		return -1;
	}

	return 0;
}
