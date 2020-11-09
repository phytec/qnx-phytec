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
 * Header file containing includes to system headers and porting types.
 */
/*==========================================================================*/

#ifndef SC_SCFW_H
#define SC_SCFW_H

/* Includes */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef DEBUG
    #include <stdio.h>
#endif
#include <sys/types.h>

/*!
 * This type is used to declare a handle for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef paddr_t sc_ipc_t;

/*!
 * This type is used to declare an ID for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef paddr_t sc_ipc_id_t;

#endif /* SC_SCFW_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/public/hw/nxp/imx8/sci/scfw.h $ $Rev: 862289 $")
#endif
