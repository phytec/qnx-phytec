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
 * File containing client-side RPC functions for the IRQ service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup IRQ_SVC
 * @{
 */
/*==========================================================================*/

/* Includes */

#include <hw/nxp/imx8/sci/sci_types.h>
#include <hw/nxp/imx8/sci/svc/rm/api.h>
#include <hw/nxp/imx8/sci/svc/irq/api.h>
#include "rpc.h"
#include "svc/irq/irq_rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_irq_enable(sc_ipc_t ipc, sc_rsrc_t resource, sc_irq_group_t group,
    uint32_t mask, sc_bool_t enable)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_IRQ);
    RPC_FUNC(&msg) = U8(IRQ_FUNC_ENABLE);

    RPC_U32(&msg, 0U) = U32(mask);
    RPC_U16(&msg, 4U) = U16(resource);
    RPC_U8(&msg, 6U) = U8(group);
    RPC_U8(&msg, 7U) = B2U8(enable);

    sc_call_rpc(ipc, &msg, SC_FALSE);

    err = (sc_err_t) RPC_R8(&msg);

    return err;
}

sc_err_t sc_irq_status(sc_ipc_t ipc, sc_rsrc_t resource, sc_irq_group_t group,
    uint32_t *status)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_IRQ);
    RPC_FUNC(&msg) = U8(IRQ_FUNC_STATUS);

    RPC_U16(&msg, 0U) = U16(resource);
    RPC_U8(&msg, 2U) = U8(group);

    sc_call_rpc(ipc, &msg, SC_FALSE);

    err = (sc_err_t) RPC_R8(&msg);

    if (status != NULL)
    {
        *status = (uint32_t) RPC_U32(&msg, 0U);
    }

    return err;
}

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/irq/irq_rpc_clnt.c $ $Rev: 904595 $")
#endif
