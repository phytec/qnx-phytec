/*
* $QNXLicenseC:
* Copyright 2017-2018 NXP
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
 * Header file for the IPC implementation.
 */
/*==========================================================================*/

#ifndef SC_IPC_H
#define SC_IPC_H

/* Includes */

#include <hw/nxp/imx8/sci/sci_types.h>

/* Defines */

/* Types */

/* Functions */

/*!
 * This function opens an IPC channel.
 *
 * @param[out]    ipc         return pointer for ipc handle
 * @param[in]     id          id of channel to open
 *
 * @return Returns an error code (SC_ERR_NONE = success, SC_ERR_IPC
 *         otherwise).
 *
 * The \a id parameter is implementation specific. Could be an MU
 * address, pointer to a driver path, channel index, etc.
 */
sc_err_t sc_ipc_open(sc_ipc_t *ipc, sc_ipc_id_t id);

/*!
 * This function closes an IPC channel.
 *
 * @param[in]     ipc         id of channel to close
 */
void sc_ipc_close(sc_ipc_t ipc);

/*!
 * This function reads a message from an IPC channel.
 *
 * @param[in]     ipc         id of channel read from
 * @param[out]    data        pointer to message buffer to read
 *
 * This function will block if no message is available to be read.
 */
void sc_ipc_read(sc_ipc_t ipc, void *data);

/*!
 * This function writes a message to an IPC channel.
 *
 * @param[in]     ipc         id of channel to write to
 * @param[in]     data        pointer to message buffer to write
 *
 * This function will block if the outgoing buffer is full.
 */
void sc_ipc_write(sc_ipc_t ipc, const void *data);

#endif /* SC_IPC_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/hw_vendor/nxp/imx8/sci/public/hw/nxp/imx8/sci/ipc.h $ $Rev: 869197 $")
#endif
