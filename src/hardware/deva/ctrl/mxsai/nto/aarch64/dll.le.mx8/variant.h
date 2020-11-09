/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
 * Copyright 2018-2019 NXP
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

#ifndef VARIANT_H_
#define VARIANT_H_

#include <aarch64/imx8_common/imx_sai.h>
#include <aarch64/imx8_common/imx_edma.h>
#include <aarch64/imx8_common/imx_edma_requests.h>

#define IMX_EDMA                        1
#define IMX_SAI_VERSION                 IMX_SAI_VERSION_QM_QXP
#define IMX_SAI_DEFAULT_CLK_MODE        IMX_SAI_MASTER
#define IMX_SAI_DEFAULT_CLK             24576000
#define IMX_SAI_FIFO_WATERMARK          16
#define IMX_SAI_DEFAULT_SAMPLE_SIZE     2
#define IMX_SAI_SAMPLE_RATE_MIN         8000
#define IMX_SAI_SAMPLE_RATE_MAX         48000
#define IMX_SAI_SAMPLE_RATES            SND_PCM_RATE_8000,\
                                        SND_PCM_RATE_16000,\
                                        SND_PCM_RATE_32000,\
                                        SND_PCM_RATE_48000

#define I2C_BUS_NUMBER                  0
#define I2C_SLAVE_ADDR                  0x48
#define AIF_TX_VOICES                   "2"
#define AIF_RX_VOICES                   "2"

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxsai/nto/aarch64/dll.le.mx8/variant.h $ $Rev: 905343 $")
#endif
