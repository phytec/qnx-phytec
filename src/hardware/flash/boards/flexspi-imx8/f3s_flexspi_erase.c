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
#ifdef DEBUG
    #include <sys/slog.h>
    #include <sys/slogcodes.h>
#endif

/**
 * @file       flexspi-imx8/f3s_flexspi_erase.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the erase callout for FLEXSPI serial NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Memory offset.
 *
 * @retval EOK    Everything is OK.
 * @retval ERANGE The offset is out of bounds.
 * @retval EIO    Previous erase fail.
 */
int f3s_flexspi_erase(__attribute__((unused)) f3s_dbase_t *dbase, f3s_access_t *access,
                    __attribute__((unused)) uint32_t flags, uint32_t offset)
{
    int rc;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x", pthread_self(), __func__, __LINE__, offset);
#endif
    if (access->service->page(&access->socket, 0, offset, NULL) == NULL) {
        return (ERANGE);
    }
    rc = sector_erase((imx_fspi_t *) access->socket.socket_handle, offset);
    if (rc < 0) {
        return (EIO);
    }

    return (EOK);
}

/** @}*/


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_erase.c $ $Rev: 893539 $")
#endif
