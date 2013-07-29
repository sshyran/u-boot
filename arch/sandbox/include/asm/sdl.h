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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __SANDBOX_SDL_H
#define __SANDBOX_SDL_H

int sandbox_sdl_init_display(int width, int height, int log2_bpp);
int sandbox_sdl_sync(void *lcd_base);

int sandbox_sdl_scan_keys(int key[], int max_keys);
int sandbox_sdl_key_pressed(int keycode);

int sandbox_sdl_sound_start(uint frequency);
int sandbox_sdl_sound_stop(void);
int sandbox_sdl_sound_init(void);

#endif
