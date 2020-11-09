/*
 * $QNXLicenseC:
 * Copyright 2019, QNX Software Systems.
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

#ifndef	__AARCH64_EXYNOSAUTO9_H_INCLUDED
#define	__AARCH64_EXYNOSAUTO9_H_INCLUDED

/*
 * System Memory Map
 */

/*
 * UART
 */
#define EXYNOSAUTO9_UART0_BASE                      0x10300000
#define EXYNOSAUTO9_UART_SIZE                       0x1000
#define EXYNOSAUTO9_UART0_IRQ                       377

/*
 * ENET
 */
#define EXYNOSAUTO9_ENET0_BASE                      0x17D80000
#define EXYNOSAUTO9_ENET1_BASE                      0x17D90000
#define EXYNOSAUTO9_ENET_SIZE                       0x2000
#define EXYNOSAUTO9_ENET0_IRQ                       239
#define EXYNOSAUTO9_ENET1_IRQ                       244


#define EXYNOSAUTO9_GPIO_FSYS2                      0x17C30000
#define EXYNOSAUTO9_GPF2_CON                        (EXYNOSAUTO9_GPIO_FSYS2 + 0x0000)
#define EXYNOSAUTO9_GPF3_CON                        (EXYNOSAUTO9_GPIO_FSYS2 + 0x0020)
#define EXYNOSAUTO9_GPF4_CON                        (EXYNOSAUTO9_GPIO_FSYS2 + 0x0040)
#define EXYNOSAUTO9_GPF5_CON                        (EXYNOSAUTO9_GPIO_FSYS2 + 0x0060)
#define EXYNOSAUTO9_GPF6_CON                        (EXYNOSAUTO9_GPIO_FSYS2 + 0x0080)

#define EXYNOSAUTO9_GPIO_FSYS1                      0x17060000
#define EXYNOSAUTO9_GPF8_CON                        (EXYNOSAUTO9_GPIO_FSYS1 + 0x0000)
#define EXYNOSAUTO9_GPF8_PUD                        (EXYNOSAUTO9_GPIO_FSYS1 + 0x0008)
#define EXYNOSAUTO9_GPF8_DRV                        (EXYNOSAUTO9_GPIO_FSYS1 + 0x000C)


#endif	/* __AARCH64_EXYNOSAUTO9_H_INCLUDED */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/lib/public/aarch64/exynosauto9.h $ $Rev: 888192 $")
#endif
