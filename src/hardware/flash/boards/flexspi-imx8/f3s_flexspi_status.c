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
 * @file       flexspi-imx8/f3s_flexspi_status.c
 * @addtogroup ffs3
 * @{
 */

/**
 * This is the status callout for FLEXSPI serial NOR flash driver.
 *
 * @param socket Socket Services Info.
 * @param flags  Flags.
 *
 * @return EOK always.
 */
int32_t f3s_flexspi_status(__attribute__((unused)) f3s_socket_t *socket, __attribute__((unused)) uint32_t flags)
{
    /* Everything is fine */
    return EOK;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_status.c $ $Rev: 893539 $")
#endif
