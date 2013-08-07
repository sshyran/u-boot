/*
 * Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

void __def_board_i2c_release_bus(int node)
{
}

void board_i2c_release_bus(int node)
	__attribute__((weak, alias("__def_board_i2c_release_bus")));

int __def_board_i2c_claim_bus(int node)
{
	return 0;
}

int board_i2c_claim_bus(int node)
	__attribute__((weak, alias("__def_board_i2c_claim_bus")));
