/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

#ifndef IMX_QSPI_HC_H_
#define IMX_QSPI_HC_H_

#include <stdint.h>

#define MAX_BURST_RX                            128
#define IMX_QSPI_AMBA_BASE_ADDRESS              0x60000000U

#define FLASH_PAGE_SIZE                         256               /**< Page size in Bytes */
#define FLASH_SECTOR_SIZE                       256               /**< Number of pages in one sector */
#define FLASH_TOTAL_SIZE                        1024              /**< Number of sectors */
//
#define TOTAL_SIZE_BYTES                        (FLASH_TOTAL_SIZE * FLASH_SECTOR_SIZE * FLASH_PAGE_SIZE)

/* API */
void qspi_init(void);
int qspi_read(int offset, uint8_t *buffer, unsigned int size);

#endif /* IMX_QSPI_HC_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/private/imx_qspi_ipl.h $ $Rev: 886103 $")
#endif
