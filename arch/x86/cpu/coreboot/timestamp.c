/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2011 The ChromiumOS Authors.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA, 02110-1301 USA
 */

#include <common.h>
#include <coreboot_timestamp.h>
#include <asm/arch-coreboot/sysinfo.h>

struct timestamp_entry {
	uint32_t	entry_id;
	uint64_t	entry_stamp;
} __attribute__((packed));

struct timestamp_table {
	uint64_t	base_time;
	uint32_t	max_entries;
	uint32_t	num_entries;
	struct timestamp_entry entries[0]; /* Variable number of entries */
} __attribute__((packed));

static struct timestamp_table *ts_table  __attribute__((section(".data")));

void timestamp_init(void)
{
	ts_table = lib_sysinfo.tstamp_table;
	set_base_timer_value(ts_table->base_time);
	timestamp_add_now(TS_U_BOOT_INITTED);
}

void timestamp_add(enum timestamp_id id, uint64_t ts_time)
{
	struct timestamp_entry *tse;

	if (!ts_table || (ts_table->num_entries == ts_table->max_entries))
		return;

	tse = &ts_table->entries[ts_table->num_entries++];
	tse->entry_id = id;
	tse->entry_stamp = ts_time - ts_table->base_time;
}

void timestamp_add_now(enum timestamp_id id)
{
	timestamp_add(id, rdtsc());
}
