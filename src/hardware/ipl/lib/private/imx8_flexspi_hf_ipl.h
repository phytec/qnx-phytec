/*
 * $QNXLicenseC:
 * Copyright 2020, QNX Software Systems.
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

#ifndef _IMX8_FLEXSPI_HF_IPL_H
#define _IMX8_FLEXSPI_HF_IPL_H

#include <stdint.h>
//#include <aarch64/imx8_common/imx_flexspi.h>

#define FLEXSPI_MAX_BURST_RX                    128

typedef struct _flexspi_conf_hf {
    char           *name;               /* Name/description of device */
    unsigned        rx_fifo_size;       /* Receive FIFO size */
    unsigned        pads;               /* PADs number */
    unsigned        smpl;               /* Sample clock source */
    unsigned        mid;                /* Manufacturer ID */
    unsigned        did;                /* Device ID */
    unsigned        size;               /* Flash size in bytes */
    unsigned        col_width;          /* Column bits */
    unsigned char   word_addr;          /* Is word addressable: 0/1 */
} flexspi_conf_hf_t;

typedef struct _flexspi_hf {
    unsigned        pbase;              /* Base address */
    flexspi_conf_hf_t  *conf;
    unsigned        verbose;            /* Verbose level */
} flexspi_hf_t;

extern int imx_flexspi_init_hf(flexspi_hf_t *flexspi, unsigned base, int verbose);
extern int imx_flexspi_read_hf(flexspi_hf_t *flexspi, unsigned offset, unsigned long buffer, unsigned size);

#endif /* _IMX8_FLEXSPI_HF_IPL_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/private/imx8_flexspi_hf_ipl.h $ $Rev: 910001 $")
#endif
