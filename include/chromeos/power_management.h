/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 */

/* power management interface for Chrome OS verified boot */

#ifndef CHROMEOS_POWER_MANAGEMENT_H_
#define CHROMEOS_POWER_MANAGEMENT_H_

/**
 * This reboots the main processor and TPM chip together (we can do that on
 * certain boards). Any kernel crashlog or DRAM content should be preserved.
 *
 * If it fails, it proceeds to a cold reboot. We do not proceed to a warm
 * reboot (which only reboots the main processor but leave TPM chip untouched).
 *
 * The problem of a warm reboot is that the verified boot cannot be tolerant of
 * TPM in an uninitialized or locked state (a workaround was provided, but will
 * be retired anytime; see crosbug.com/15759).
 *
 * Compared to the problem of a warm reboot (failed verified boot), the problem
 * of a cold reboot (you lose crashlog sometimes) seems to be lighter. And so we
 * decide that cros_reboot should fall into a cold reboot if it fails to reboot
 * the main processor and TPM chip together.
 */
void cros_reboot(void);

/**
 * This reboots the whole system (main processor, DRAM, TPM, etc.) by pulling
 * PMIC. Any kernel crashlog or DRAM content will be lost.
 */
void cold_reboot(void);

#endif /* CHROMEOS_POWER_MANAGEMENT_H_ */
