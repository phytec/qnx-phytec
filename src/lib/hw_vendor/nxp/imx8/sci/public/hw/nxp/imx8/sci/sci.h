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

#ifndef _SC_SCI_H
#define _SC_SCI_H

/* Defines */

/* Includes */

#include <hw/nxp/imx8/sci/sci_types.h>
#include <hw/nxp/imx8/sci/ipc.h>
#include <hw/nxp/imx8/sci/svc/irq/api.h>
#include <hw/nxp/imx8/sci/svc/misc/api.h>
#include <hw/nxp/imx8/sci/svc/pad/api.h>
#include <hw/nxp/imx8/sci/svc/pm/api.h>
#include <hw/nxp/imx8/sci/svc/rm/api.h>
#include <hw/nxp/imx8/sci/svc/seco/api.h>
#include <hw/nxp/imx8/sci/svc/timer/api.h>

/* Types */

/* Functions */
char * sc_status2str(sc_err_t status);
const char * sc_rsrc2str(sc_rsrc_t resource);

#endif /* _SC_SCI_H */

/**@}*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/hw_vendor/nxp/imx8/sci/public/hw/nxp/imx8/sci/sci.h $ $Rev: 904595 $")
#endif
