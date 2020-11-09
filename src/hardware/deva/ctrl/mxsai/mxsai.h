/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
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

#ifndef MXSAI_DLL_H_
#define MXSAI_DLL_H_

/*
 *  Definitions for i.MX SAI audio controller
 */
struct imx_card;
struct imx_stream;
#define  HW_CONTEXT_T                      struct imx_card
#define  PCM_SUBCHN_CONTEXT_T              struct imx_stream

#include <audio_driver.h>
#include <mixer/i2s_codec_dll.h>
#include <hw/dma.h>
#include <string.h>
#include <proto.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>
#include <variant.h>
#include <hw/hwinfo_imx8x.h>

#define MAX_NSLOTS          32

/** IMX stream DMA data. */
typedef struct imx_stream_dma {
    void            *chn;                           /**< DMA channel for transfer. */
    dma_addr_t      *buf;                           /**< DMA buffer allocated by the driver. */
    struct sigevent event;                          /**< DMA event associated with SAI. */
    void            *pulse;                         /**< DMA pulse handler for pulse receive. */
} imx_stream_dma_t;

/** IMX SAI mode. */
typedef enum imx_sai_mode {
    IMX_SAI_MASTER,                                 /**< SAI device is master and ext. codec is slave. */
    IMX_SAI_SLAVE,                                  /**< SAI device is slave and ext. codec is master. */
} imx_sai_mode_t;

/** IMX SAI protocol. */
typedef enum imx_sai_protocol {
    IMX_SAI_PROTOCOL_I2S,                           /**< I2S protocol. */
    IMX_SAI_PROTOCOL_PCM,                           /**< PCM protocol. */
} imx_sai_protocol_t;

/** IMX SAI RX TX synchronization mode. */
typedef enum imx_sai_sync_mode {
    IMX_SAI_SYNC_RX_WITH_TX = 0,                    /**< Rx uses Tx frame sync and bit clock. */
    IMX_SAI_ASYNC = 0xff                            /**< Rx and Tx are asynchronous. */
} imx_sai_sync_mode_t;

/** IMX SAI VERSION **/
typedef enum imx_sai_version {
    IMX_SAI_VERSION_QM_QXP = 0                      /** Support for i.MX8QM and i.MX8QXP using EDMA. */
} imx_sai_version_t;

/** IMX SAI transfer configuration structure. */
typedef struct imx_sai_xfer_config {
    imx_sai_protocol_t protocol;                    /**< SAI protocol. */
    int32_t            sample_rate;                 /**< SAI sample rate in Hz. */
    uint32_t           sample_rate_min;             /**< Lowest supported sample rate in Hz. */
    uint32_t           sample_rate_max;             /**< Highest supported sample rate in Hz. */
    int32_t            sample_size;                 /**< Sample size. 2 or 4 bytes*/
    int32_t            conf_slot_size;              /**< Configured slot size, i.e. word width, number of bits per word.*/
    int32_t            curr_slot_size;              /**< Current used slot size.*/
    int32_t            nslots;                      /**< Number of slots. */
    uint8_t            clk_pol;                     /**< Bit clock polarity. */
    uint8_t            sync_pol;                    /**< Bit clock synchronization polarity. */
    uint8_t            bit_delay;                   /**< Bit clock delay 0,1. Allows early frame sync. */
    uint8_t            sync_len;                    /**< Frame sync len. */
    uint8_t            msel;                        /**< MCLK select. See chip specific information in RM. */
    uint32_t           fifo_size;                   /**< FIFO size */
} imx_sai_xfer_config_t;

/** IMX stream data structure. */
struct imx_stream {
    ado_pcm_subchn_t *subchn;                       /**< Pointer to subchannel structure. */
    ado_pcm_cap_t    pcm_caps;                      /**< Data structure of capabilities of a PCM device. */
    ado_pcm_hw_t     pcm_funcs;                     /**< Data structure of callbacks for PCM devices. */
    ado_pcm_config_t *config;                       /**< Data structure that describes the configuration
                                                         of a PCM subchannel. */
    int              active;                        /**< Stream active or not. */
    imx_stream_dma_t dma;                           /**< DMA related data. */
    uint32_t         voices;                        /**< Number of voices. */
    uint32_t         pcm_offset;
};

typedef struct imx_stream imx_stream_t;

struct imx_audio_interface
{
    ado_pcm_t    *pcm;
    imx_stream_t play_strm;
    imx_stream_t cap_strm;
};

typedef struct imx_audio_interface imx_aif_t;

/** IMX Card data structure. */
struct imx_card {
    uint32_t              base;                     /**< SAI Peripheral base address. */
    ado_mutex_t           lock;                     /**< Mutex for hardware and common data lock. */
    ado_card_t            *card;
    ado_mixer_t           *mixer;                   /**< Mixer data. */
    imx_aif_t             *aif;                     /**< Audio Interfaces. */
    uint32_t              num_tx_aif;
    uint32_t              num_rx_aif;
    imx_sai_rxtx_t        *tx_reg;                  /**< SAI Peripheral register structure */
    imx_sai_rxtx_t        *rx_reg;                  /**< SAI Peripheral register structure */
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
    imx_sai_ver_reg_t     *id_reg;
#endif
    imx_sai_xfer_config_t tx_cfg;                   /**< SAI transfer config for Tx and Rx. */
    imx_sai_xfer_config_t rx_cfg;                   /**< SAI transfer config for Tx and Rx. */
    dma_functions_t       dmafuncs;                 /**< DMA functions. */
    uint32_t              tx_dma_chnl_type;         /**< TX DMA channel type. */
    uint32_t              rx_dma_chnl_type;         /**< RX DMA channel type. */
    uint32_t              tx_frag_size;
    uint32_t              rx_frag_size;
    volatile int32_t      playback_frag_index;
    volatile int32_t      capture_frag_index;
    ado_pcm_dmabuf_t      capture_dmabuf;
    ado_pcm_dmabuf_t      playback_dmabuf;
    int32_t               i2c_dev;                  /**< i2c device instance number. */
    int32_t               i2c_addr;                 /**< i2c address. */
    uint32_t              sys_clk;                  /**< System clock frequency in Hz. */
    imx_sai_sync_mode_t   xfer_sync_mode;           /**< SAI TX RX xfer synchronization. */
    uint32_t              clk_mode;                 /**< SAI Master/Slave mode. */
    int16_t               tx_fifo_watermark;
    int16_t               rx_fifo_watermark;
    ado_mixer_dll_codec_callbacks_t callbacks;      /**< ctrl -> mixer callbacks */
};
typedef struct imx_card imx_t;

void codec_on(HW_CONTEXT_T *hwc, int channel);
void codec_off(HW_CONTEXT_T *hwc, int channel);
void codec_set_rate(HW_CONTEXT_T *hwc, uint32_t rate, int channel);
void codec_set_default_group(HW_CONTEXT_T *hwc, ado_pcm_t *pcm, int channel, int index);
int codec_mixer(ado_card_t * card, HW_CONTEXT_T * imx, ado_pcm_t *pcm);

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxsai/mxsai.h $ $Rev: 905343 $")
#endif

