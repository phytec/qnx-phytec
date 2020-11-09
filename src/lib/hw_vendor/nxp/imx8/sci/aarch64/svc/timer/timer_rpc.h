/*
* $QNXLicenseC:
* Copyright 2017-2019 NXP
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

/*==========================================================================*/
/*!
 * @file
 *
 * Header file for the TIMER RPC implementation.
 *
 * @addtogroup TIMER_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_TIMER_RPC_H
#define SC_TIMER_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC TIMER function calls
 */
/*@{*/
#define TIMER_FUNC_UNKNOWN 0 /*!< Unknown function */
#define TIMER_FUNC_SET_WDOG_TIMEOUT 1U /*!< Index for sc_timer_set_wdog_timeout() RPC call */
#define TIMER_FUNC_SET_WDOG_PRE_TIMEOUT 12U /*!< Index for sc_timer_set_wdog_pre_timeout() RPC call */
#define TIMER_FUNC_SET_WDOG_WINDOW 19U /*!< Index for sc_timer_set_wdog_window() RPC call */
#define TIMER_FUNC_START_WDOG 2U /*!< Index for sc_timer_start_wdog() RPC call */
#define TIMER_FUNC_STOP_WDOG 3U /*!< Index for sc_timer_stop_wdog() RPC call */
#define TIMER_FUNC_PING_WDOG 4U /*!< Index for sc_timer_ping_wdog() RPC call */
#define TIMER_FUNC_GET_WDOG_STATUS 5U /*!< Index for sc_timer_get_wdog_status() RPC call */
#define TIMER_FUNC_PT_GET_WDOG_STATUS 13U /*!< Index for sc_timer_pt_get_wdog_status() RPC call */
#define TIMER_FUNC_SET_WDOG_ACTION 10U /*!< Index for sc_timer_set_wdog_action() RPC call */
#define TIMER_FUNC_SET_RTC_TIME 6U /*!< Index for sc_timer_set_rtc_time() RPC call */
#define TIMER_FUNC_GET_RTC_TIME 7U /*!< Index for sc_timer_get_rtc_time() RPC call */
#define TIMER_FUNC_GET_RTC_SEC1970 9U /*!< Index for sc_timer_get_rtc_sec1970() RPC call */
#define TIMER_FUNC_SET_RTC_ALARM 8U /*!< Index for sc_timer_set_rtc_alarm() RPC call */
#define TIMER_FUNC_SET_RTC_PERIODIC_ALARM 14U /*!< Index for sc_timer_set_rtc_periodic_alarm() RPC call */
#define TIMER_FUNC_CANCEL_RTC_ALARM 15U /*!< Index for sc_timer_cancel_rtc_alarm() RPC call */
#define TIMER_FUNC_SET_RTC_CALB 11U /*!< Index for sc_timer_set_rtc_calb() RPC call */
#define TIMER_FUNC_SET_SYSCTR_ALARM 16U /*!< Index for sc_timer_set_sysctr_alarm() RPC call */
#define TIMER_FUNC_SET_SYSCTR_PERIODIC_ALARM 17U /*!< Index for sc_timer_set_sysctr_periodic_alarm() RPC call */
#define TIMER_FUNC_CANCEL_SYSCTR_ALARM 18U /*!< Index for sc_timer_cancel_sysctr_alarm() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming TIMER RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void timer_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_TIMER_RPC_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/timer/timer_rpc.h $ $Rev: 904590 $")
#endif
