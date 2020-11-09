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

#include <sys/slog.h>
#include <sys/slogcodes.h>

#include "f3s_flexspi.h"

/**
 * @file       flexspi-imx8/f3s_flexspi_write.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the write callout for FLEXSPI serial NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Offset where to write data.
 * @param size   Size of data.
 * @param buffer Buffer to write to memory.
 *
 * @return Size of written data if everything is fine. EIO otherwise.
 */
int32_t f3s_flexspi_write(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                          uint32_t offset, int32_t size, uint8_t *buffer)
{
    int     rc,i;
    int     nbytes;
    int32_t xfer;
    imx_fspi_t *dev = (imx_fspi_t *) access->socket.socket_handle;
    uint8_t *verify=dev->verify; /* Buffer for verification */

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x size=%d", pthread_self(), __func__, __LINE__,
          offset, size);
#endif
    if (access->service->page(&access->socket, 0, offset, &size) == NULL) {
        return -1;
    }
    rc = page_program(dev, offset, size, buffer);
    if (-1 == rc) {
        errno = EIO;
        return -1;
    }
    if (!(flags & F3S_VERIFY_WRITE)) {
        return rc;
    }
    /* Verify data was written correctly */
    size = rc;
    xfer = rc;

    while (xfer) {
        nbytes = min(dev->page_size, xfer);

        rc = read_from(dev, offset, nbytes, verify);
        if (-1 == rc) {
            errno = EIO;
            return -1;
        }

        if (memcmp(verify, buffer, rc)) {
            for (i = 0; i < rc; i++) {
                slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "EXPECTED %d val: 0x%x", i, buffer[i]);
                slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "READ     %d val: 0x%x", i, verify[i]);
            }
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "(devf  t%d::%s:%d) program verify error\n"
                  "between offset 0x%x and 0x%x, size = %d", pthread_self(),
                  __func__, __LINE__, offset, offset + rc, rc);
            errno = EIO;
            return -1;
        }
        xfer -= rc;
        offset += rc;
        buffer += rc;
    }
    /* Verification successful */
    return size;
}

/** @} */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_write.c $ $Rev: 876662 $")
#endif
