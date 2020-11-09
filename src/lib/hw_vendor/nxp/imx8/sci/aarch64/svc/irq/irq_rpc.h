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
 * Header file for the IRQ RPC implementation.
 *
 * @addtogroup IRQ_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_IRQ_RPC_H
#define SC_IRQ_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC IRQ function calls
 */
/*@{*/
#define IRQ_FUNC_UNKNOWN 0 /*!< Unknown function */
#define IRQ_FUNC_ENABLE 1U /*!< Index for sc_irq_enable() RPC call */
#define IRQ_FUNC_STATUS 2U /*!< Index for sc_irq_status() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming IRQ RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void irq_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_IRQ_RPC_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/irq/irq_rpc.h $ $Rev: 904590 $")
#endif
