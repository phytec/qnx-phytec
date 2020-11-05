/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
 * Copyright 2017 NXP
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


#include "proto.h"

int
_spi_close_ocb(resmgr_context_t *ctp, void *reserved, spi_ocb_t *ocb)
{
	SPIDEV		*drvhdl = (SPIDEV *)ocb->hdr.attr;

	/*
	 * Unlock All the devices belong to this ocb
	 */
	while (_spi_unlock_dev(ctp, SPI_DEV_ID_NONE, ocb) == EOK)
		;

	/*
	 * Notify clients to unblock
	 */
	iofunc_notify_trigger(drvhdl->notify, INT_MAX, IOFUNC_NOTIFY_INPUT);
	iofunc_notify_trigger(drvhdl->notify, INT_MAX, IOFUNC_NOTIFY_OUTPUT);
	iofunc_notify_trigger(drvhdl->notify, INT_MAX, IOFUNC_NOTIFY_OBAND);

	return iofunc_close_ocb_default(ctp, reserved, &ocb->hdr);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/slave/_spi_close_ocb.c $ $Rev: 855721 $")
#endif
