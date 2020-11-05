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
 * Header file for the MISC RPC implementation.
 *
 * @addtogroup MISC_SVC
 * @{
 */
/*==========================================================================*/

#ifndef SC_MISC_RPC_H
#define SC_MISC_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC MISC function calls
 */
/*@{*/
#define MISC_FUNC_UNKNOWN 0 /*!< Unknown function */
#define MISC_FUNC_SET_CONTROL 1U /*!< Index for sc_misc_set_control() RPC call */
#define MISC_FUNC_GET_CONTROL 2U /*!< Index for sc_misc_get_control() RPC call */
#define MISC_FUNC_SET_MAX_DMA_GROUP 4U /*!< Index for sc_misc_set_max_dma_group() RPC call */
#define MISC_FUNC_SET_DMA_GROUP 5U /*!< Index for sc_misc_set_dma_group() RPC call */
#define MISC_FUNC_DEBUG_OUT 10U /*!< Index for sc_misc_debug_out() RPC call */
#define MISC_FUNC_WAVEFORM_CAPTURE 6U /*!< Index for sc_misc_waveform_capture() RPC call */
#define MISC_FUNC_BUILD_INFO 15U /*!< Index for sc_misc_build_info() RPC call */
#define MISC_FUNC_API_VER 35U /*!< Index for sc_misc_api_ver() RPC call */
#define MISC_FUNC_UNIQUE_ID 19U /*!< Index for sc_misc_unique_id() RPC call */
#define MISC_FUNC_SET_ARI 3U /*!< Index for sc_misc_set_ari() RPC call */
#define MISC_FUNC_BOOT_STATUS 7U /*!< Index for sc_misc_boot_status() RPC call */
#define MISC_FUNC_BOOT_DONE 14U /*!< Index for sc_misc_boot_done() RPC call */
#define MISC_FUNC_OTP_FUSE_READ 11U /*!< Index for sc_misc_otp_fuse_read() RPC call */
#define MISC_FUNC_OTP_FUSE_WRITE 17U /*!< Index for sc_misc_otp_fuse_write() RPC call */
#define MISC_FUNC_SET_TEMP 12U /*!< Index for sc_misc_set_temp() RPC call */
#define MISC_FUNC_GET_TEMP 13U /*!< Index for sc_misc_get_temp() RPC call */
#define MISC_FUNC_GET_BOOT_DEV 16U /*!< Index for sc_misc_get_boot_dev() RPC call */
#define MISC_FUNC_GET_BOOT_TYPE 33U /*!< Index for sc_misc_get_boot_type() RPC call */
#define MISC_FUNC_GET_BOOT_CONTAINER 36U /*!< Index for sc_misc_get_boot_container() RPC call */
#define MISC_FUNC_GET_BUTTON_STATUS 18U /*!< Index for sc_misc_get_button_status() RPC call */
#define MISC_FUNC_ROMPATCH_CHECKSUM 26U /*!< Index for sc_misc_rompatch_checksum() RPC call */
#define MISC_FUNC_BOARD_IOCTL 34U /*!< Index for sc_misc_board_ioctl() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming MISC RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     mu          MU message came from
 * @param[in]     msg         pointer to RPC message
 */
void misc_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_MISC_RPC_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/svc/misc/misc_rpc.h $ $Rev: 904595 $")
#endif
