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
 * @file       flexspi-imx8/f3s_flexspi_read.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the read callout for FLEXSPI serial NOR flash driver.
 *
 * @param dbase       Flash Services Database.
 * @param access      Access Super Structure
 * @param flags       Flags.
 * @param text_offset Offset of memory to read data.
 * @param buffer_size Buffer size.
 * @param buffer      Pointer to buffer.
 *
 * @return Returns number of bytes read or -1 in case of an error.
 */
int32_t f3s_flexspi_read(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                         uint32_t text_offset, int32_t buffer_size, uint8_t *buffer)
{
    int rc;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x", pthread_self(), __func__, __LINE__, text_offset);
#endif

    /* Check if offset does not fit in array */
    if (text_offset >= access->socket.window_size) {
        errno = ERANGE;
        return -1;
    }
    /* Ensure that offset + size is not out of bounds */
    buffer_size = min(buffer_size, access->socket.window_size - text_offset);
    rc = read_from((imx_fspi_t *) access->socket.socket_handle, text_offset, buffer_size, buffer);
    if (-1 == rc) {
        errno = EIO;
        return -1;
    }

    return rc; /* Return number of bytes read */
}

/** @} */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_read.c $ $Rev: 842484 $")
#endif
