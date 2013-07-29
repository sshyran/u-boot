/*
 * Copyright (c) 2013 Google, Inc
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

#include <common.h>
#include <asm/arch/sound.h>
#include <asm/sdl.h>

int sound_play(uint32_t msec, uint32_t frequency)
{
	sandbox_sdl_sound_start(frequency);
	mdelay(msec);
	sandbox_sdl_sound_stop();

	return 0;
}

int sound_init(const void *blob)
{
	return sandbox_sdl_sound_init();
}
