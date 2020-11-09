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
 * Header file for the PAD RPC implementation.
 *
 * @addtogroup PAD_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_PAD_RPC_H
#define SC_PAD_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC PAD function calls
 */
/*@{*/
#define PAD_FUNC_UNKNOWN 0 /*!< Unknown function */
#define PAD_FUNC_SET_MUX 1U /*!< Index for sc_pad_set_mux() RPC call */
#define PAD_FUNC_GET_MUX 6U /*!< Index for sc_pad_get_mux() RPC call */
#define PAD_FUNC_SET_GP 2U /*!< Index for sc_pad_set_gp() RPC call */
#define PAD_FUNC_GET_GP 7U /*!< Index for sc_pad_get_gp() RPC call */
#define PAD_FUNC_SET_WAKEUP 4U /*!< Index for sc_pad_set_wakeup() RPC call */
#define PAD_FUNC_GET_WAKEUP 9U /*!< Index for sc_pad_get_wakeup() RPC call */
#define PAD_FUNC_SET_ALL 5U /*!< Index for sc_pad_set_all() RPC call */
#define PAD_FUNC_GET_ALL 10U /*!< Index for sc_pad_get_all() RPC call */
#define PAD_FUNC_SET 15U /*!< Index for sc_pad_set() RPC call */
#define PAD_FUNC_GET 16U /*!< Index for sc_pad_get() RPC call */
#define PAD_FUNC_SET_GP_28FDSOI 11U /*!< Index for sc_pad_set_gp_28fdsoi() RPC call */
#define PAD_FUNC_GET_GP_28FDSOI 12U /*!< Index for sc_pad_get_gp_28fdsoi() RPC call */
#define PAD_FUNC_SET_GP_28FDSOI_HSIC 3U /*!< Index for sc_pad_set_gp_28fdsoi_hsic() RPC call */
#define PAD_FUNC_GET_GP_28FDSOI_HSIC 8U /*!< Index for sc_pad_get_gp_28fdsoi_hsic() RPC call */
#define PAD_FUNC_SET_GP_28FDSOI_COMP 13U /*!< Index for sc_pad_set_gp_28fdsoi_comp() RPC call */
#define PAD_FUNC_GET_GP_28FDSOI_COMP 14U /*!< Index for sc_pad_get_gp_28fdsoi_comp() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming PAD RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void pad_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_PAD_RPC_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/pad/pad_rpc.h $ $Rev: 904590 $")
#endif
