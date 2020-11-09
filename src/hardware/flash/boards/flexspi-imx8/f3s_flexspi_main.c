/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
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

#include "f3s_flexspi.h"
#include "f3s_flexspi_s26hs.h"

/**
 * @file       flexspi-imx8/f3s_flexspi_main.c
 * @addtogroup ffs3
 * @{
 */
extern char *drv_opts;

/**
 * This is the main function for the FLEXSPI f3s flash file system.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 *
 * @return Execution status of main function.
 */
int main(int argc, char **argv)
{
    int                  error;
    static f3s_service_t service[] = {
        {
            sizeof(f3s_service_t),
            f3s_flexspi_open,
            f3s_flexspi_page,
            f3s_flexspi_status,
            f3s_flexspi_close
        },
        {
            0, 0, 0, 0, 0   /* Mandatory last entry */
        }
    };

#if MTD_VER == 2
    static f3s_flash_v2_t flash[] = {
#ifndef IMX_FLEXSPI_SUPPORT_S26HS
        {
            sizeof(f3s_flash_v2_t),
            f3s_flexspi_ident,     /* Identification */
            f3s_sram_reset,        /* Common Reset */

            NULL,                  /* v1 Read                  */
            NULL,                  /* v1 Write                 */
            NULL,                  /* v1 Erase                 */
            NULL,                  /* v1 Suspend               */
            NULL,                  /* v1 Resume                */
            NULL,                  /* v1 Sync                  */

            f3s_flexspi_read,      /* v2 Read                  */
            f3s_flexspi_write,     /* v2 Write                 */
            f3s_flexspi_erase,     /* v2 Erase                 */
            NULL,                  /* v2 Suspend               */
            NULL,                  /* v2 Resume                */
            f3s_flexspi_sync,      /* v2 Sync                  */
            NULL,                  /* v2 Islock                */
            NULL,                  /* v2 Lock                  */
            NULL,                  /* v2 Unlock                */
            NULL                   /* v2 Unlockall             */
        },
#else
        /* NOTE: f3s_flexspi_ident_s26hs() reopens and reconfigures the device for hyperflash
         * use. This means any entries after this entry must ensure their ident function can
         * work with the new settings, and/or they should also reopen/reconfigure the device.
         */
        {
            sizeof(f3s_flash_v2_t),
            f3s_flexspi_ident_s26hs, /* Identification */
            f3s_sram_reset,          /* Common Reset */

            NULL,                    /* v1 Read                  */
            NULL,                    /* v1 Write                 */
            NULL,                    /* v1 Erase                 */
            NULL,                    /* v1 Suspend               */
            NULL,                    /* v1 Resume                */
            NULL,                    /* v1 Sync                  */

            f3s_flexspi_read_s26hs,  /* v2 Read                  */
            f3s_flexspi_write_s26hs, /* v2 Write                 */
            f3s_flexspi_erase_s26hs, /* v2 Erase                 */
            NULL,                    /* v2 Suspend               */
            NULL,                    /* v2 Resume                */
            f3s_flexspi_sync_s26hs,  /* v2 Sync                  */
            NULL,                    /* v2 Islock                */
            NULL,                    /* v2 Lock                  */
            NULL,                    /* v2 Unlock                */
            NULL                     /* v2 Unlockall             */
        },
#endif
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 /* Mandatory last entry */
        }
    };
#else
#error "MTD version must be 2"
#endif

    /* Save driver options */
    if ((strstr(argv[argc - 1], "base") != NULL) || (strstr(argv[argc - 1], "irq") != NULL) ||
        (strstr(argv[argc - 1], "page_size") != NULL) || (strstr(argv[argc - 1], "size") != NULL)) {
        drv_opts = argv[--argc];
    }

    /* Initialize f3s */
    f3s_init(argc, argv, (f3s_flash_t *)flash);

    /* Start f3s */
    error = f3s_start(service, (f3s_flash_t *)flash);

    return error;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_main.c $ $Rev: 909225 $")
#endif
