/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* this macro enable tlcl stub to call real hardware functions */
#define CONFIG_HARDWARE_TPM

/* config for infineon prototype i2c tpm chip */
#define CONFIG_INFINEON_TPM_I2C
#define CONFIG_INFINEON_TPM_I2C_BUS		2

/* auto detect all following chips selected below */
#define CONFIG_TPM_SLB9635_I2C
/* #define CONFIG_TPM_SLB9635_I2C_V03 */

/* limit burst write to tpm chip on i2c */
#define CONFIG_TPM_I2C_BURST_LIMITATION		3
