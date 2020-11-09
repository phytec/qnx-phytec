/*
 * $QNXLicenseC:
 * Copyright 2013, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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


#include <sim_sdmmc.h>

/**
 * Board specific interface
 *
 * @file       sdmmc/aarch64/mx8x.le/sim_bs.c
 * @addtogroup sdmmc_bs
 * @{
 */

/**
 * Board specific arguments.
 *
 * @param hba       HBA pointer.
 * @param options   Board specific options.
 *
 * @return EOK always.
 */
int sim_bs_args(SIM_HBA *hba, char *options)
{
    SIM_SDMMC_EXT   *ext;
    char            *value;
    int              opt;
    static char     *opts[] = {
                        "dname",
                        NULL
                    };

    ext = (SIM_SDMMC_EXT *)hba->ext;

    while (options && *options != '\0' ) {
        if ((opt = getsubopt( &options, opts, &value)) == -1) {
            /* Nothing to do */
        }
        switch (opt) {
            case 0:
                ext->eflags |= SDMMC_EFLAG_DEVNAME;
                if (value && !strncmp(value, "compat", strlen("compat"))) {
                    ext->eflags |= SDMMC_EFLAG_BS;
                }
                break;
            default:
                break;
        }
    }

    return (EOK);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devb/sdmmc/aarch64/mx8x.le/sim_bs.c $ $Rev: 854898 $")
#endif
