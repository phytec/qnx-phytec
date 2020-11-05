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


#include "imx_fc_flexspi.h"
#include "f3s_flexspi.h"

/**
 * @file       flexspi-imx8/f3s_flexspi_sync.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the sync callout for FLEXSPI serial NOR flash driver. Called together with erase
 * function to check erase progress.
 *
 * @param dbase       Flash Services Database.
 * @param access      Access Super Structure.
 * @param flags       Flags.
 * @param text_offset Memory offset.
 *
 * @retval EOK    Everything is fine. Memory is ready.
 * @retval ERANGE Offset out of bounds.
 * @retval EIO    This return code currently not supported.
 * @retval EAGAIN WIP flag is set. Erase still in progress.
 */
int32_t f3s_flexspi_sync(__attribute__((unused)) f3s_dbase_t *dbase, f3s_access_t *access,
                        __attribute__((unused)) uint32_t flags, uint32_t text_offset)
{
    int rc;
    imx_fspi_t *dev = (imx_fspi_t *) access->socket.socket_handle;

    if (access->service->page(&access->socket, 0, text_offset, NULL) == NULL) {
        return (ERANGE);
    }
    out32(dev->vbase + IMX_FLEXSPI_IPCR0, text_offset); /* Write address to controller */
    rc = iswriting(dev);
    if (-1 == rc) {
        return (EIO);
    }
    if (1 == rc) {
        /* Write in progress */
        return (EAGAIN);
    }
    return (EOK);
}

/** @} */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_sync.c $ $Rev: 893539 $")
#endif
