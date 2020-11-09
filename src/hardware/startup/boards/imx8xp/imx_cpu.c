/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * Copyright 2018, QNX Software Systems.
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
#include "imx_startup.h"

#define CHIP_STRING_SIZE 30

/**
 * Get chip revision code.
 *
 * @param ipc IPC handle.
 *
 * @return  Chip revision code.
 */
uint32_t imx_get_chip_rev(sc_ipc_t ipc)
{
    uint32_t chip_rev = 0;
    sc_err_t sc_status;

    sc_status = sc_misc_get_control(ipc, SC_R_SYSTEM, SC_C_ID, &chip_rev); /* Get chip revision */
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Get chip revision failed.\n", __FUNCTION__);
    }
    return ((chip_rev >> 5) & 0x0F);
}

/**
 * Get chip type code.
 *
 * @param ipc IPC handle.
 *
 * @return  Chip type code.
 */
uint32_t imx_get_chip_type(sc_ipc_t ipc)
{
    uint32_t chip_type = 0;
    sc_err_t sc_status;

    sc_status = sc_misc_get_control(ipc, SC_R_SYSTEM, SC_C_ID, &chip_type); /* Get chip type */
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Get chip type failed.\n", __FUNCTION__);
    }
    return (chip_type & 0x1F);
}

/**
 * Print chip information: Type, revision.
 *
 * @param ipc IPC handle.
 */
void print_chip_info(sc_ipc_t ipc)
{
    uint32_t chip_type = imx_get_chip_type(ipc);
    uint32_t chip_rev = imx_get_chip_rev(ipc);

    char chip_type_str[CHIP_STRING_SIZE];
    char chip_rev_str[CHIP_STRING_SIZE];

    switch (chip_type) {
        case IMX_CHIP_TYPE_QUAD_MAX:
            strcpy(chip_type_str, "QuadMax");
            break;
        case IMX_CHIP_TYPE_QUAD_X_PLUS:
            strcpy(chip_type_str, "QuadXPlus");
            break;
        case IMX_CHIP_TYPE_DUAL_X_PLUS:
            strcpy(chip_type_str, "DualXPlus");
            break;
        default:
            strcpy(chip_type_str, "Unknown Variant");
            break;
    }
    switch (chip_rev) {
        case IMX_CHIP_REV_A:
            strcpy(chip_rev_str, "A");
            break;
        case IMX_CHIP_REV_B:
            strcpy(chip_rev_str, "B");
            break;
        default:
            strcpy(chip_rev_str, "Unknown Revision");
            break;
    }
    kprintf("Detected i.MX8%s, revision %s\n", chip_type_str, chip_rev_str);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/imx_cpu.c $ $Rev: 862292 $")
#endif
