/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef __configs_chromeos_tegra2_twostop_h__
#define __configs_chromeos_tegra2_twostop_h__

/* So far all our tegra2-based boards share the seaboard config. */
#include <configs/seaboard.h>

#include "chromeos.h"

/* We initialize Chrome OS -specific GPIOs here */
#define CONFIG_MISC_INIT_R

/* Store the VbNvContext in the first block of the disk. */
#define CHROMEOS_VBNVCONTEXT_LBA	0

#endif /* __configs_chromeos_tegra2_twostop_h__ */
