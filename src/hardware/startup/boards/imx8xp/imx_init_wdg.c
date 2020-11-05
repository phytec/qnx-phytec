/*
 * $QNXLicenseC:
 * Copyright 2011,2012, 2018  QNX Software Systems.
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

#include <stdint.h>
#include <hw/nxp/imx8/sci/sci.h>
#include "startup.h"
#include "board.h"
#include "imx_startup.h"
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
#include "aarch64/mx8x_smc_call.h"
#endif

/* The watchdog timeout value should be specified in board.h.  Set default value to 7500 seconds. */
#if !defined(WDOG_TIMEOUT)
    #define IMX_WDOG_TIMEOUT 7500
#endif

//#define IMX_WDOG_DEBUG 1U

/**
 * Enable Watchdog.
 *
 * @param ipc IPC handle
 */
void imx_wdg_enable(sc_ipc_t ipc)
{
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    imx_smc_status_t smc_status;
#else
    sc_err_t sc_status;
    sc_rm_pt_t pt_tz;
#ifdef IMX_WDOG_DEBUG
    sc_timer_wdog_time_t timeout;
    sc_timer_wdog_time_t max_timeout;
    sc_timer_wdog_time_t remaining_time;
#endif
#endif

#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    /* Set WDOG reset action type */
    smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_SRTC, IMX_FSL_SIP_SRTC_SET_WDOG_ACT, SC_TIMER_WDOG_ACTION_BOARD, 0x00, 0x00);
    if (smc_status != PSCI_SUCCESS) {
        crash("WDOG set action failed. ERR_CODE: %d\n", smc_status);
    }

    /* Set default WDOG timeout */
    smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_SRTC, IMX_FSL_SIP_SRTC_SET_TIMEOUT_WDOG, IMX_WDOG_TIMEOUT, 0x00, 0x00);
    if (smc_status != PSCI_SUCCESS) {
        crash("WDOG timeout settings failed. ERR_CODE: %d\n", smc_status);
    }

    /* Start WDOG timer */
    smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_SRTC, IMX_FSL_SIP_SRTC_START_WDOG, false, 0x00, 0x00);
    if (smc_status != PSCI_SUCCESS) {
        crash("WDOG start failed. ERR_CODE: %d\n", smc_status);
    }
#else
    sc_status = sc_rm_get_partition(ipc, &pt_tz);
    if (sc_status != SC_ERR_NONE) {
        crash("WDOG get partition failed. ERR_CODE: %d\n", sc_status);
    }

    sc_status = sc_timer_set_wdog_action(ipc, pt_tz, SC_TIMER_WDOG_ACTION_BOARD);
    if (sc_status != SC_ERR_NONE) {
        crash("WDOG set action failed. ERR_CODE: %d\n", sc_status);
    }

    /* Set default WDOG timeout */
    sc_status = sc_timer_set_wdog_timeout(ipc, IMX_WDOG_TIMEOUT);
    if (sc_status != SC_ERR_NONE) {
        crash("WDOG timeout settings failed. ERR_CODE: %d\n", sc_status);
    }

    /* Start WDOG timer */
    sc_status =  sc_timer_start_wdog(ipc, false);
    if (sc_status != SC_ERR_NONE) {
        crash("WDOG start failed. ERR_CODE: %d\n", sc_status);
    }
#ifdef IMX_WDOG_DEBUG
    sc_status = sc_timer_get_wdog_status(ipc, &timeout, &max_timeout, &remaining_time);
    if (sc_status != SC_ERR_NONE) {
        crash("WDOG status read failed. ERR_CODE: %d\n", sc_status);
    }

    else {
        kprintf("Watchdog status: \n");
        kprintf("   Max timeout: %d\n", max_timeout);
        kprintf("   Current timeout: %d\n", timeout);
        kprintf("   Remaining time until timeout: %d\n", remaining_time);
    }
#endif
#endif

}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_init_wdg.c $ $Rev: 891625 $")
#endif
