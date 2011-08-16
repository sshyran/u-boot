/*
 * Copyright (c) 2011, Google Inc. All rights reserved.
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


/*
 * This module records the progress of boot and arbitrary commands, and
 * permits accurate timestamping of each. The records can optionally be
 * passed to kernel in the ATAGs
 */

#include <common.h>

DECLARE_GLOBAL_DATA_PTR;

struct bootstage_record {
	uint32_t time_us;
	const char *name;
};

static struct bootstage_record record[BOOTSTAGE_COUNT] = {{1, "avoid_bss"}};

uint32_t bootstage_mark(enum bootstage_id id, const char *name)
{
	struct bootstage_record *rec = &record[id];

	/* Only record the first event for each */
	if (!rec->name) {
		rec->time_us = (uint32_t)timer_get_us();
		rec->name = name;
	}
	return rec->time_us;
}

static void print_time(unsigned long us_time)
{
	char str[12], *s;
	int grab = 3;

	/* We don't seem to have %'d in U-Boot */
	sprintf(str, "%9ld", us_time);
	for (s = str; *s; s += grab) {
		if (s != str)
			putc(s[-1] != ' ' ? ',' : ' ');
		printf("%.*s", grab, s);
		grab = 3;
	}
}

static uint32_t print_time_record(enum bootstage_id id,
			struct bootstage_record *rec, uint32_t prev)
{
	print_time(rec->time_us);
	print_time(rec->time_us - prev);
	if (rec->name)
		printf("  %s\n", rec->name);
	else
		printf("  id=%d\n", id);
	return rec->time_us;
}

static int h_compare_record(const void *r1, const void *r2)
{
	const struct bootstage_record *rec1 = r1, *rec2 = r2;

	return rec1->time_us - rec2->time_us;
}

void bootstage_report(void)
{
	struct bootstage_record *rec = record;
	int id;
	uint32_t prev;
	u32 flags = gd->flags;

	/* Force this information to display even on silent console */
	gd->flags &= ~GD_FLG_SILENT;
	puts("Timer summary in microseconds:\n");
	printf("%11s%11s  %s\n", "Mark", "Elapsed", "Stage");

	/* Fake the first record - we could get it from early boot */
	rec->name = "reset";
	rec->time_us = 0;
	prev = print_time_record(BOOTSTAGE_AWAKE, rec, 0);

	/* Sort records by increasing time */
	qsort(record, ARRAY_SIZE(record), sizeof(*rec), h_compare_record);

	for (id = 0; id < BOOTSTAGE_COUNT; id++, rec++) {
		if (rec->time_us != 0)
			prev = print_time_record(id, rec, prev);
	}
	if (flags & GD_FLG_SILENT)
		gd->flags |= GD_FLG_SILENT;
}

