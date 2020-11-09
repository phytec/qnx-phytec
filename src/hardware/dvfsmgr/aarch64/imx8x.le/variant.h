/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */


#ifndef VARIANT_H
#define VARIANT_H

#define iMX8_SOC

#define SW_CPULOAD													// Do software CPU load calculation macro

#define MX8X_CPU_NUMBER_MAX						8					// Max CPU core number
#define MX8X_SC_DECVICE_DEFUALT					"/dev/sc"			// System controller device
#define MX8X_CL_DECVICE_DEFUALT					"/proc/1/as"		// path of CPU load process

#define MX8X_SC_DELAY							10					// Delay time for System controller call
#define MX8X_SC_RETRY							10					// Max retry for System controller call
#define MX8X_TEMP_READ_CNT						10					// Max samples for getting correct CPU temprature

#define MX8X_MAX_SAFE_VOLT						1100000				// This should match the configuration table
#define MAX_SAFE_VOLT							MX8X_MAX_SAFE_VOLT

#define MX8X_TEMP_ERROR							-1000

#define MX8X_IDEL_START							0
#define MX8X_IDEL_END							1
#define MX8X_IDEL_DIFF							2

#define CHIP_STRING_SIZE						32

enum imx_chip_type_list {
	IMX_CHIP_TYPE_QUAD_MAX = 0x01,
	IMX_CHIP_TYPE_QUAD_X_PLUS = 0x02,
	IMX_CHIP_TYPE_DUAL_X_PLUS = 0x03,
	IMX_CHIP_TYPE_UNKNOWN,
};

enum imx_chip_rev_list {
	IMX_CHIP_REV_A = 0x00,
	IMX_CHIP_REV_B = 0x01,
	IMX_CHIP_REV_UNKNOWN,
};

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/dvfsmgr/aarch64/imx8x.le/variant.h $ $Rev: 847526 $")
#endif
