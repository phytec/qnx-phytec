/*
 * $QNXLicenseC:
 * Copyright 2018 QNX Software Systems.
 * Copyright 2018 NXP
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

#include "startup.h"
#include <hw/nxp/imx8/sci/sci.h>
#include "board.h"
#include "imx_startup.h"

/* Startup global data */
extern imx_startup_data_t startup_data;

/**
 * Calculate the core frequency (in MHz).
 *
 * @return Core frequency (in MHz)
 */
uint32_t aarch64_cpuspeed(void)
{
    uint64_t            part_num;
    sc_pm_clock_rate_t  rate;

    /* Get core part number information */
    part_num = ((aa64_sr_rd64(midr_el1) & 0xFFF0) >> 4) ;

    /* Get core frequency */
    switch (part_num) {
    case 0x0D04:
        /* Cortex-A35 */
        (void)sc_pm_get_clock_rate(startup_data.ipc, SC_R_A35, SC_PM_CLK_CPU, &rate);
        break;
    case 0x0D03:
        /* Cortex-A53 */
        (void)sc_pm_get_clock_rate(startup_data.ipc, SC_R_A53, SC_PM_CLK_CPU, &rate);
        break;
    case 0x0D08:
        /* Cortex-A72 */
        (void)sc_pm_get_clock_rate(startup_data.ipc, SC_R_A72, SC_PM_CLK_CPU, &rate);
        break;
    default:
        rate = 0;
        break;
    }

    return (uint32_t)(rate / 1000000);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/aarch64_cpuspeed.c $ $Rev: 869196 $")
#endif
