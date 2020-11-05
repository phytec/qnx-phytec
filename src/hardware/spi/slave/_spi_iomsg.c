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
_spi_iomsg(resmgr_context_t *ctp, io_msg_t *msg, spi_ocb_t *ocb)
{
	spi_msg_t	*spimsg = (spi_msg_t *)msg;

	if (((spimsg->msg_hdr.i.combine_len < sizeof(spi_msg_t))) || ((spimsg->msg_hdr.i.mgrid != _IOMGR_SPI)))
        return ENOSYS;
	if ((spimsg->msg_hdr.i.subtype >= _SPI_IOMSG_READ) && (spimsg->msg_hdr.i.subtype <= _SPI_IOMSG_DMAXCHANGE)) {
		int err;

		if ((err =_spi_lock_check(ctp, spimsg->device, ocb)) != EOK)
			return err;

		switch (spimsg->msg_hdr.i.subtype) {
			case _SPI_IOMSG_READ:
				perror("Unsupported iomsg_read function");
				break;
			case _SPI_IOMSG_WRITE:
				perror("Unsupported iomsg_write function");
				break;
			case _SPI_IOMSG_EXCHANGE:
				perror("Unsupported iomsg_exchange function");
				break;
			case _SPI_IOMSG_CMDREAD:
				perror("Unsupported iomsg_cmdread function");
				break;
			case _SPI_IOMSG_DMAXCHANGE:
				perror("Unsupported iomsg_dmaxchange function");
				break;
		}
	}

	return EINVAL;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/slave/_spi_iomsg.c $ $Rev: 853252 $")
#endif
