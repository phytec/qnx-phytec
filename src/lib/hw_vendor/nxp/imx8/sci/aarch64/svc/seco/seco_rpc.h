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
 * Header file for the SECO RPC implementation.
 *
 * @addtogroup SECO_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_SECO_RPC_H
#define SC_SECO_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC SECO function calls
 */
/*@{*/
#define SECO_FUNC_UNKNOWN 0 /*!< Unknown function */
#define SECO_FUNC_IMAGE_LOAD 1U /*!< Index for sc_seco_image_load() RPC call */
#define SECO_FUNC_AUTHENTICATE 2U /*!< Index for sc_seco_authenticate() RPC call */
#define SECO_FUNC_ENH_AUTHENTICATE 24U /*!< Index for sc_seco_enh_authenticate() RPC call */
#define SECO_FUNC_FORWARD_LIFECYCLE 3U /*!< Index for sc_seco_forward_lifecycle() RPC call */
#define SECO_FUNC_RETURN_LIFECYCLE 4U /*!< Index for sc_seco_return_lifecycle() RPC call */
#define SECO_FUNC_COMMIT 5U /*!< Index for sc_seco_commit() RPC call */
#define SECO_FUNC_ATTEST_MODE 6U /*!< Index for sc_seco_attest_mode() RPC call */
#define SECO_FUNC_ATTEST 7U /*!< Index for sc_seco_attest() RPC call */
#define SECO_FUNC_GET_ATTEST_PKEY 8U /*!< Index for sc_seco_get_attest_pkey() RPC call */
#define SECO_FUNC_GET_ATTEST_SIGN 9U /*!< Index for sc_seco_get_attest_sign() RPC call */
#define SECO_FUNC_ATTEST_VERIFY 10U /*!< Index for sc_seco_attest_verify() RPC call */
#define SECO_FUNC_GEN_KEY_BLOB 11U /*!< Index for sc_seco_gen_key_blob() RPC call */
#define SECO_FUNC_LOAD_KEY 12U /*!< Index for sc_seco_load_key() RPC call */
#define SECO_FUNC_GET_MP_KEY 13U /*!< Index for sc_seco_get_mp_key() RPC call */
#define SECO_FUNC_UPDATE_MPMR 14U /*!< Index for sc_seco_update_mpmr() RPC call */
#define SECO_FUNC_GET_MP_SIGN 15U /*!< Index for sc_seco_get_mp_sign() RPC call */
#define SECO_FUNC_BUILD_INFO 16U /*!< Index for sc_seco_build_info() RPC call */
#define SECO_FUNC_CHIP_INFO 17U /*!< Index for sc_seco_chip_info() RPC call */
#define SECO_FUNC_ENABLE_DEBUG 18U /*!< Index for sc_seco_enable_debug() RPC call */
#define SECO_FUNC_GET_EVENT 19U /*!< Index for sc_seco_get_event() RPC call */
#define SECO_FUNC_FUSE_WRITE 20U /*!< Index for sc_seco_fuse_write() RPC call */
#define SECO_FUNC_PATCH 21U /*!< Index for sc_seco_patch() RPC call */
#define SECO_FUNC_START_RNG 22U /*!< Index for sc_seco_start_rng() RPC call */
#define SECO_FUNC_SAB_MSG 23U /*!< Index for sc_seco_sab_msg() RPC call */
#define SECO_FUNC_SECVIO_ENABLE 25U /*!< Index for sc_seco_secvio_enable() RPC call */
#define SECO_FUNC_SECVIO_CONFIG 26U /*!< Index for sc_seco_secvio_config() RPC call */
#define SECO_FUNC_SECVIO_DGO_CONFIG 27U /*!< Index for sc_seco_secvio_dgo_config() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming SECO RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void seco_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_SECO_RPC_H */

/**@}*/


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/seco/seco_rpc.h $ $Rev: 904595 $")
#endif
