/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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

#include <hw/hwinfo_imx8x.h>

#define IMX_ESAI_MAX_FRAGSIZE           64 * 1024

#define IMX_EDMA                        1
#define ESAI_COUNT                      2
#define QXP_ESAI_COUNT                  1
/* Contains ESAI instance number or ESAI base address. */
#define IMX_ESAI_BASE                   0
#define ESAI_MAIN_CLK_SRC               MAIN_CLK_SRC_XTAL
#define ESAI_MAIN_CLK_FREQ              (24576000)
#define ESAI_CLK_MODE                   ESAI_CLK_SLAVE
#define SAMPLE_RATE_MIN                 48000
#define SAMPLE_RATE_MAX                 48000
#define AIF_TX_VOICES                   "8"
#define AIF_RX_VOICES                   "2:2"
#define SAMPLE_SIZE                     2
#define ESAI_PROTOCOL                   PROTOCOL_I2S
#define ESAI_NUM_SLOTS                  2
#define SDO_PIN_MAP                     "0:1:2:3"
#define SDI_PIN_MAP                     "0:1"

/** ESAI ETDR Register offset */
#define IMX_ESAI_ETDR_OFFSET           0x00
/** ESAI ERDR Register offset */
#define IMX_ESAI_ERDR_OFFSET           0x04

#define I2C_BUS_NUMBER                  0
#define I2C_SLAVE_ADDR                  0x48

#define CODEC_PLAYBACK_UNMUTE   codec_on(mx, ADO_PCM_CHANNEL_PLAYBACK);
#define CODEC_PLAYBACK_MUTE     codec_off(mx, ADO_PCM_CHANNEL_PLAYBACK);
#define CODEC_CAPTURE_UNMUTE    codec_on(mx, ADO_PCM_CHANNEL_CAPTURE);
#define CODEC_CAPTURE_MUTE      codec_off(mx, ADO_PCM_CHANNEL_CAPTURE);

enum imx_chip_type_list {
    IMX_CHIP_TYPE_QUAD_MAX = 0x01,
    IMX_CHIP_TYPE_QUAD_X_PLUS = 0x02,
    IMX_CHIP_TYPE_DUAL_X_PLUS = 0x03,
};

typedef struct chip_info {
    uint32_t type;
    uint32_t rev;
} chip_info_t;

void codec_on(HW_CONTEXT_T *hwc, int channel);
void codec_off(HW_CONTEXT_T *hwc, int channel);

int codec_mixer(ado_card_t * card, HW_CONTEXT_T * imx, ado_pcm_t *pcm);
void codec_set_default_group(HW_CONTEXT_T *hwc, ado_pcm_t *pcm, int channel, int index);

#endif /* VARIANT_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/deva/ctrl/mxesai/nto/aarch64/dll.le.mx8/variant.h $ $Rev: 882864 $")
#endif
