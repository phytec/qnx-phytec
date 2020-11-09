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
 * Header file for the PM RPC implementation.
 *
 * @addtogroup PM_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_PM_RPC_H
#define SC_PM_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC PM function calls
 */
/*@{*/
#define PM_FUNC_UNKNOWN 0 /*!< Unknown function */
#define PM_FUNC_SET_SYS_POWER_MODE 19U /*!< Index for sc_pm_set_sys_power_mode() RPC call */
#define PM_FUNC_SET_PARTITION_POWER_MODE 1U /*!< Index for sc_pm_set_partition_power_mode() RPC call */
#define PM_FUNC_GET_SYS_POWER_MODE 2U /*!< Index for sc_pm_get_sys_power_mode() RPC call */
#define PM_FUNC_PARTITION_WAKE 28U /*!< Index for sc_pm_partition_wake() RPC call */
#define PM_FUNC_SET_RESOURCE_POWER_MODE 3U /*!< Index for sc_pm_set_resource_power_mode() RPC call */
#define PM_FUNC_SET_RESOURCE_POWER_MODE_ALL 22U /*!< Index for sc_pm_set_resource_power_mode_all() RPC call */
#define PM_FUNC_GET_RESOURCE_POWER_MODE 4U /*!< Index for sc_pm_get_resource_power_mode() RPC call */
#define PM_FUNC_REQ_LOW_POWER_MODE 16U /*!< Index for sc_pm_req_low_power_mode() RPC call */
#define PM_FUNC_REQ_CPU_LOW_POWER_MODE 20U /*!< Index for sc_pm_req_cpu_low_power_mode() RPC call */
#define PM_FUNC_SET_CPU_RESUME_ADDR 17U /*!< Index for sc_pm_set_cpu_resume_addr() RPC call */
#define PM_FUNC_SET_CPU_RESUME 21U /*!< Index for sc_pm_set_cpu_resume() RPC call */
#define PM_FUNC_REQ_SYS_IF_POWER_MODE 18U /*!< Index for sc_pm_req_sys_if_power_mode() RPC call */
#define PM_FUNC_SET_CLOCK_RATE 5U /*!< Index for sc_pm_set_clock_rate() RPC call */
#define PM_FUNC_GET_CLOCK_RATE 6U /*!< Index for sc_pm_get_clock_rate() RPC call */
#define PM_FUNC_CLOCK_ENABLE 7U /*!< Index for sc_pm_clock_enable() RPC call */
#define PM_FUNC_SET_CLOCK_PARENT 14U /*!< Index for sc_pm_set_clock_parent() RPC call */
#define PM_FUNC_GET_CLOCK_PARENT 15U /*!< Index for sc_pm_get_clock_parent() RPC call */
#define PM_FUNC_RESET 13U /*!< Index for sc_pm_reset() RPC call */
#define PM_FUNC_RESET_REASON 10U /*!< Index for sc_pm_reset_reason() RPC call */
#define PM_FUNC_GET_RESET_PART 26U /*!< Index for sc_pm_get_reset_part() RPC call */
#define PM_FUNC_BOOT 8U /*!< Index for sc_pm_boot() RPC call */
#define PM_FUNC_SET_BOOT_PARM 27U /*!< Index for sc_pm_set_boot_parm() RPC call */
#define PM_FUNC_REBOOT 9U /*!< Index for sc_pm_reboot() RPC call */
#define PM_FUNC_REBOOT_PARTITION 12U /*!< Index for sc_pm_reboot_partition() RPC call */
#define PM_FUNC_REBOOT_CONTINUE 25U /*!< Index for sc_pm_reboot_continue() RPC call */
#define PM_FUNC_CPU_START 11U /*!< Index for sc_pm_cpu_start() RPC call */
#define PM_FUNC_CPU_RESET 23U /*!< Index for sc_pm_cpu_reset() RPC call */
#define PM_FUNC_RESOURCE_RESET 29U /*!< Index for sc_pm_resource_reset() RPC call */
#define PM_FUNC_IS_PARTITION_STARTED 24U /*!< Index for sc_pm_is_partition_started() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming PM RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void pm_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_PM_RPC_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/pm/pm_rpc.h $ $Rev: 904590 $")
#endif
