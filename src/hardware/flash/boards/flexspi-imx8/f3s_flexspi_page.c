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

/**
 * @file       flexspi-imx8/f3s_flexspi_page.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the page callout for QSPI serial NOR flash driver.
 *
 * @param socket Socket Services Info.
 * @param flags  Flags.
 * @param offset Memory offset.
 * @param size   Size of data.
 *
 * @retval ~0 every time the offset is within bounds.
 * @retval NULL otherwise.
 */
uint8_t * f3s_flexspi_page(f3s_socket_t *socket, uint32_t flags, uint32_t offset, int32_t *size)
{
    /* Check if offset does not fit in array */
    if (offset >= socket->window_size) {
        errno = ERANGE;
        return NULL;
    }

    /* Always zero since there is only 1 window for this device */
    socket->window_offset = 0;

    /* Ensure that offset + size is not out of bounds */
    if (size) {
        *size = min(*size, socket->window_size - offset);
    }

    /* Memory pointers are not applicable to this device
     * so we just return ~0 every time the offset is within bounds */
    return (uint8_t*) ~0;
}

/** @} */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_page.c $ $Rev: 839864 $")
#endif
