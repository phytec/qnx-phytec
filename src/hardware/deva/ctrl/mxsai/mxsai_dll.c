/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017, QNX Software Systems.
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

/*
 *
 *    mxsai_dll.c
 *      The primary interface into the mxsai DLL.
 */

#include "mxsai.h"

#ifndef IMX_SAI_VERSION
    #define IMX_SAI_VERSION IMX_SAI_VERSION_QM_QXP
#endif

#define IMX_SAI_WAIT_TIMEOUT    10000U
#define IMX_SAI_MAX_FIFO_SIZE   0xFFFF

#define MIN_WATERMARK           16
#define MAX_WATERMARK           32
#define IMX_DMA_ADDR_FLAG      (DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE)

/**
 * This function returns the number of active capture channels
 */
uint32_t
num_active_capture_interfaces(HW_CONTEXT_T * imx)
{
    uint32_t num; /* number of capture interfaces */
    uint32_t idx = 0;

    num = 0;
    for(idx=0; idx < imx->num_rx_aif; idx++)
    {
        if(imx->aif[idx].cap_strm.active == 1)
            num++;
    }
    ado_error_fmt("active capture streams %d", num);
    return num;
}

/**
 * This function returns the number of active playback channels
 */
uint32_t
num_active_playback_interfaces(HW_CONTEXT_T * imx)
{
    uint32_t num; /* number of playback interfaces */
    uint32_t idx = 0;

    num = 0;
    for(idx=0; idx < imx->num_tx_aif; idx++)
    {
        if(imx->aif[idx].play_strm.active == 1)
            num++;
    }
    ado_error_fmt("active play streams %d", num);
    return num;
}

static int calculate_fifo_watermark(ado_pcm_config_t *config, imx_sai_xfer_config_t *cfg)
{
    int watermark = IMX_SAI_FIFO_WATERMARK * cfg->sample_size;
#if IMX_EDMA
    /* NOTE: The fragment size (DMA transfer size) must be an even multiple of the FIFO watermark
     */
    for (watermark = MAX_WATERMARK; watermark >= MIN_WATERMARK; watermark--)
    {
        if (config->mode.block.frag_size % watermark == 0 && !((config->mode.block.frag_size / watermark) & 0x1)) {
            if ((cfg->fifo_size % watermark == 0) && (cfg->fifo_size > watermark)) {
                if ((cfg->fifo_size % IMX_SAI_FIFO_WATERMARK == 0) && (cfg->fifo_size > IMX_SAI_FIFO_WATERMARK)) {
                    break;  /* We found a suitable watermark */
                }
            }
        }
    }
    if (watermark < MIN_WATERMARK)
    {
        watermark = MAX_WATERMARK;
        ado_error_fmt("MX_SAI: WARNING - Suitable watermark could not be found, defaulting to %d bytes", watermark);
    }

    /* Fragment size has to be an even multiple of the FIFO watermark
     * Note: Just check the LSB to see if frag_size is an even multiple of the FIFO watermark or not.
     */
    if ((config->mode.block.frag_size % watermark != 0) || ((config->mode.block.frag_size / watermark) & 0x1))
    {
        int frame_size = ado_pcm_format_byte_width(config->format.format) * config->format.voices;
        int alignment = ado_lcm(watermark * 2, frame_size);    /* Multiple by 2 to make it an even multiple */

        ado_error_fmt("MX_SAI: WARNING - Aligning fragment (%d) to be an even multiple of the FIFO watermark (%d) and the frame size (%d) - Alignment = %d",
                      config->mode.block.frag_size, watermark, frame_size, alignment);

        if (config->mode.block.frag_size <= alignment) {
            config->mode.block.frag_size = alignment;
        } else {
            config->mode.block.frag_size = config->mode.block.frag_size / alignment * alignment;
        }

        /* Update total DMA buffer size based on new fragment size */
        config->dmabuf.size = config->mode.block.frags_total * config->mode.block.frag_size;
    }
    ado_debug(DB_LVL_DRIVER, "MX_SAI: FIFO watermark = %d, Fragment size = %d", watermark, config->mode.block.frag_size);
#endif
    return (watermark);
}

/*
 * This function starts tx transmission.
 */
static int32_t imx_sai_start_tx(HW_CONTEXT_T *imx)
{
    uint32_t i;
    int rtn = EOK;

    if (imx->tx_reg == NULL) {
        ado_error_fmt("imx->tx_reg uninitialized");
        return -1;
    }

    /* Reset FIFO
     * In SYNC mode, TX is enabled when RX is enabled. Resetting the TX FIFO while TX is already enabled
     * can lead to channel swap, so we only reset the TX FIFO if TX is not enabled.
     * */
    if ((imx->tx_reg->csr & IMX_SAI_CSR_EN_MASK) != (uint32_t)IMX_SAI_CSR_EN_MASK) {
        imx->tx_reg->csr |= IMX_SAI_CSR_FR_MASK;
        nanospin_ns(1000);
        imx->tx_reg->csr &= ~((uint32_t)(IMX_SAI_CSR_FR_MASK));
    }
    /* Set fifo water mark (in unit of samples)*/
    imx->tx_reg->cr1 = imx->tx_fifo_watermark / imx->tx_cfg.sample_size;
    if (imx->dmafuncs.xfer_start(imx->aif[0].play_strm.dma.chn) == -1) {
        ado_error_fmt("Audio DMA Start failed (%s)", strerror(rtn));
        return -1;
    }
    /* Give DMA time to fill the FIFO to the watermark
     * Note: DMA request will not be asserted if we enable to soon
     * Note: Documentation describes the FRF bit backwards, bit is 0 when the watermark has been reached
     */
    for (i = 0; (imx->tx_reg->csr & IMX_SAI_CSR_FRF_MASK) && i < IMX_SAI_WAIT_TIMEOUT; i++)
    {
        nanospin_ns(200);
    }
    /* Report an error if DMA didn't fill the FIFO to the watermark on time, but still keep running */
    if (i >= IMX_SAI_WAIT_TIMEOUT) {
        rtn = ETIME;
        ado_error_fmt("Failed to reach TX FIFO watermark (0x%x)", imx->tx_reg->csr & IMX_SAI_CSR_FRF_MASK);
    }

    /* Clear FIFO error */
    if (imx->tx_reg->csr & IMX_SAI_CSR_FEF_MASK) {
        imx->tx_reg->csr |= IMX_SAI_CSR_FEF_MASK;
    }
    /* Enable TX if it is not enabled*/
    if ((imx->tx_reg->csr & IMX_SAI_CSR_EN_MASK) != (uint32_t)IMX_SAI_CSR_EN_MASK) {
        imx->tx_reg->csr |= (IMX_SAI_CSR_EN_MASK);
    }

    /* Un-mute playback after serializer is enabled to minimize pop */
    codec_on(imx, ADO_PCM_CHANNEL_PLAYBACK);

    return (rtn);
}

/*
 * This function is used when more than 1 capture pcm devices created. It is called from
 * rx interrupt handler (or pulse) to split the hardware stream into multiple pcm streams
 *
 * Since this function is forced inline, when called with a const literal sampleSize (i.e. sizeof(int16_t))
 * the compiler can optimize away any branches based on the sampleSize.
 */
static inline void __attribute__ ((__always_inline__))
imx_dmacapture_split(HW_CONTEXT_T *imx, void *srcDMAAddr, const size_t size, const size_t sampleSize)
{
    uint32_t cnt = 0, slot_cnt;
    int32_t hw_sample_offset = 0, voice = 0;
    int16_t *buf16;
    int32_t *buf32;
    int16_t *dmaptr16 = srcDMAAddr;
    int32_t *dmaptr32 = srcDMAAddr;

    if((NULL == srcDMAAddr) || (0 == size))
    {
        ado_error_fmt("Invalid data (0x%lx) or size (%d)", (paddr_t)srcDMAAddr, size);
        return;
    }

    /* Invalidate cache */
    msync(srcDMAAddr, size, MS_INVALIDATE);

    while ((hw_sample_offset * sampleSize) < size) {
        for (cnt = 0, slot_cnt = 0; cnt < imx->num_rx_aif; cnt++ ) {
            if (imx->aif[cnt].cap_strm.active) {
                if ((imx->aif[cnt].cap_strm.pcm_offset * sampleSize ) >= imx->aif[cnt].cap_strm.config->dmabuf.size) {
                    imx->aif[cnt].cap_strm.pcm_offset = 0;
                }

                if (sampleSize == sizeof(int16_t)) {
                    buf16 = (int16_t*) imx->aif[cnt].cap_strm.config->dmabuf.addr + imx->aif[cnt].cap_strm.pcm_offset;
                } else {
                    buf32 = (int32_t*) imx->aif[cnt].cap_strm.config->dmabuf.addr + imx->aif[cnt].cap_strm.pcm_offset;
                }
                for (voice = 0; voice < imx->aif[cnt].cap_strm.voices; voice++) {
                    if (sampleSize == sizeof(int16_t)) {
                        buf16[voice] = dmaptr16[hw_sample_offset + voice];
                    } else {
                        buf32[voice] = dmaptr32[hw_sample_offset + voice];
                    }
                }
                /* Bump the pcm buffer offset */
                imx->aif[cnt].cap_strm.pcm_offset += imx->aif[cnt].cap_strm.voices;
                if (((imx->aif[cnt].cap_strm.pcm_offset * sampleSize) %
                     ado_pcm_dma_int_size(imx->aif[cnt].cap_strm.config)) == 0)
                {
                    /* Signal to io-audio (DMA transfer was completed) */
                    dma_interrupt(imx->aif[cnt].cap_strm.subchn);
                }
            }
            slot_cnt += imx->aif[cnt].cap_strm.voices;
            hw_sample_offset += imx->aif[cnt].cap_strm.voices;
        }
        hw_sample_offset += (imx->rx_cfg.nslots - slot_cnt);
    }
}

/* Since this function is forced inline, when called with a const literal sampleSize (i.e. sizeof(int16_t))
 * the compiler can optimize away any branches based on the sampleSize.
 */
static inline void __attribute__ ((__always_inline__))
imx_dmaplayback_combine_internal(HW_CONTEXT_T *imx, void *dstDMAAddr, const size_t size, bool calldmainterrupt, const size_t sampleSize)
{

    uint32_t cnt = 0, slot_cnt;
    int32_t hw_sample_offset = 0, voice = 0;
    int16_t *buf16;
    int32_t *buf32;
    int16_t *dmaptr16 = dstDMAAddr;
    int32_t *dmaptr32 = dstDMAAddr;

    if((NULL == dstDMAAddr) || (0 == size))
    {
        ado_error_fmt("invalid data (0x%x) or size (%d)", dstDMAAddr, size);
        return;
    }

    /* combine multiple streams from clients to one data line for dma tranfer to hardware
     * One frame on hardware is:
     * cfg.nslots * cfg.sample_size
     * bit clock is always fixed at the maximum supported sample rate
    */
    while ((hw_sample_offset * imx->tx_cfg.sample_size) < size)  {
        for (cnt = 0, slot_cnt = 0; cnt < imx->num_tx_aif; cnt++ ) {
            if (imx->aif[cnt].play_strm.active) {
                if ((imx->aif[cnt].play_strm.pcm_offset * sampleSize ) >= imx->aif[cnt].play_strm.config->dmabuf.size)
                    imx->aif[cnt].play_strm.pcm_offset = 0;
                if (sampleSize == sizeof(int16_t)) {
                    buf16 = (int16_t*) imx->aif[cnt].play_strm.config->dmabuf.addr + imx->aif[cnt].play_strm.pcm_offset;
                } else {
                    buf32 = (int32_t*) imx->aif[cnt].play_strm.config->dmabuf.addr + imx->aif[cnt].play_strm.pcm_offset;
                }
                for (voice = 0; voice < imx->aif[cnt].play_strm.voices; voice++) {
                    if (sampleSize == sizeof(int16_t)) {
                        dmaptr16[hw_sample_offset + voice] = buf16[voice];
                    } else {
                        dmaptr32[hw_sample_offset + voice] = buf32[voice];
                    }
                }
                /* Bump the pcm buffer offset */
                imx->aif[cnt].play_strm.pcm_offset += imx->aif[cnt].play_strm.voices;
                if (((imx->aif[cnt].play_strm.pcm_offset * sampleSize) %
                     ado_pcm_dma_int_size(imx->aif[cnt].play_strm.config)) == 0)
                {
                    // Signal to io-audio (DMA transfer was completed)
                    if (calldmainterrupt)
                        dma_interrupt(imx->aif[cnt].play_strm.subchn);
                }
            }
            else
            {
                /* Silence fill the inactive TDM slots */
                for (voice = 0; voice < imx->aif[cnt].play_strm.voices; voice++) {
                    if (sampleSize == sizeof(int16_t)) {
                         dmaptr16[hw_sample_offset + voice] = 0x0;
                    } else {
                         dmaptr32[hw_sample_offset + voice] = 0x0;
                    }
                }
            }
            slot_cnt += imx->aif[cnt].play_strm.voices;
            hw_sample_offset += imx->aif[cnt].play_strm.voices;
        }
        /* Pad the remaining slots/words if the combined number of voices/channels accross
         * all of our playback interfaces is less then configured nslots
         */
        for (;slot_cnt < imx->tx_cfg.nslots; slot_cnt++)
        {
            if (sampleSize == sizeof(int16_t)) {
                dmaptr16[hw_sample_offset++] = 0x0;
            } else {
                dmaptr32[hw_sample_offset++] = 0x0;
            }
        }
    }

    /* Flush cache */
    msync(dstDMAAddr, size, MS_SYNC);

}

/*
 * This function is used when more than 1 playback pcm device is created. It is called from the
 * tx interrupt handler (or pulse) and trigger function to combine multiple pcm streams into 1
 * hardware TDM stream.
 */
static void
imx_dmaplayback_combine (HW_CONTEXT_T *imx, void *dstDMAAddr, const size_t size, bool calldmainterrupt)
{
    if (imx->tx_cfg.sample_size == sizeof(int16_t)) {
        imx_dmaplayback_combine_internal(imx, dstDMAAddr, size, calldmainterrupt, sizeof(int16_t));
    } else {
        imx_dmaplayback_combine_internal(imx, dstDMAAddr, size, calldmainterrupt, sizeof(int32_t));
    }
}
/*
 * TX pulse handler (for playback).
 *
 * Sends a signal that the current fragment of a subchannel has been completed
 * by the DMA engine.
 */
static void imx_play_pulse_hdlr(HW_CONTEXT_T *imx, struct sigevent *event)
{
#if IMX_EDMA
    int status = EOK;

    if ( (status = imx->dmafuncs.xfer_complete(imx->aif[0].play_strm.dma.chn)) != EOK)
    {
        int cnt;

        ado_error_fmt("DMA failure - 0x%x", status);
        for (cnt = 0; cnt < imx->num_tx_aif; cnt++ )
        {
            if (imx->aif[cnt].play_strm.subchn)
            {
                ado_pcm_error(imx->aif[cnt].play_strm.subchn, ADO_PCM_STATUS_ERROR);
            }
        }
    }
    else
#endif
    {
        if ( imx->num_tx_aif == 1) {
            if (imx->aif[0].play_strm.subchn) {
                dma_interrupt(imx->aif[0].play_strm.subchn);
            }
        } else {
            /* We use a ping-ping buffer for DMA, so divide buffer size by 2 to get the frag size */
            const size_t size = imx->playback_dmabuf.size / 2;

            if (imx->playback_frag_index >= 2)
                imx->playback_frag_index = 0;

            imx_dmaplayback_combine(imx, (void*)(&imx->playback_dmabuf.addr[imx->playback_frag_index * size]), size, true);
            imx->playback_frag_index++;
        }
    }
}

/*
 * Pulse handler (for capture).
 *
 * Sends a signal that the current fragment of a subchannel has been completed
 * by the DMA engine.
 */
static void imx_cap_pulse_hdlr(HW_CONTEXT_T * imx, struct sigevent *event)
{
#if IMX_EDMA
    int status = EOK;

    if ( (status = imx->dmafuncs.xfer_complete(imx->aif[0].cap_strm.dma.chn)) != EOK)
    {
        int cnt;

        ado_error_fmt("DMA failure - 0x%x", status);
        for (cnt = 0; cnt < imx->num_rx_aif; cnt++ )
        {
            if (imx->aif[cnt].cap_strm.subchn)
            {
                ado_pcm_error(imx->aif[cnt].cap_strm.subchn, ADO_PCM_STATUS_ERROR);
            }
        }
    }
    else
#endif
    {
        if ( imx->num_rx_aif == 1) {
            if (imx->aif[0].cap_strm.subchn) {
                dma_interrupt(imx->aif[0].cap_strm.subchn);
            }
        } else {
            /* We use a ping-ping buffer from DMA, so divide buffer size by 2 to get the frag size */
            const size_t size = imx->capture_dmabuf.size / 2;

            if (imx->capture_frag_index >= 2)
                imx->capture_frag_index = 0;

            if (imx->rx_cfg.sample_size == sizeof(int16_t)) {
                imx_dmacapture_split(imx, (void*)
                                    (&imx->capture_dmabuf.addr[imx->capture_frag_index * size]), size, sizeof(int16_t));
            } else {
                imx_dmacapture_split(imx, (void *)
                                    (&imx->capture_dmabuf.addr[imx->capture_frag_index * size]), size, sizeof(int32_t));
            }
            imx->capture_frag_index++;
        }
    }
}

/*
 * Gets IMX SAI capabilities.
 *
 * This function is used to return to the client the capabilities of the device
 * at this instant. When the device was created, its static capabilities were
 * passed in as an argument; however, if a number of subchannels are already
 * running, the device may no longer have the ability to support those capabilities.
 */
int32_t imx_capabilities(HW_CONTEXT_T *imx, ado_pcm_t *pcm, snd_pcm_channel_info_t *info)
{
    uint32_t  i;
    int chn_avail = 1;

    if (imx->num_tx_aif && (info->channel == SND_PCM_CHANNEL_PLAYBACK)) {
        info->fragment_align = imx->tx_cfg.sample_size * imx->tx_cfg.nslots;
        for (i = 0; i < imx->num_tx_aif; i++) {
            if (pcm == imx->aif[i].pcm && imx->aif[i].play_strm.subchn) {
                chn_avail = 0;
                break;
            }
        }
        if (chn_avail && (imx->clk_mode == IMX_SAI_MASTER)
            && (imx->tx_cfg.sample_rate_min != imx->tx_cfg.sample_rate_max) ) {
            ado_mutex_lock(&imx->lock);
            if (num_active_playback_interfaces(imx)) {
                /*
                 * Since we only have one data line, sequential playback stream rate
                 * should be locked by first playback stream
                 */
                info->min_rate = info->max_rate = imx->tx_cfg.sample_rate;
                info->rates = ado_pcm_rate2flag(imx->tx_cfg.sample_rate);
            } else if (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX) {
                /* Playback and Capture are Rate locked, so adjust rate capabilities
                 * if the other side has been acquired.
                 */
                if (num_active_capture_interfaces(imx)) {
                    info->min_rate = info->max_rate = imx->rx_cfg.sample_rate;
                    info->rates = ado_pcm_rate2flag(imx->rx_cfg.sample_rate);
                }
            }
            ado_mutex_unlock(&imx->lock);
        }
    } else if (imx->num_rx_aif && (info->channel == SND_PCM_CHANNEL_CAPTURE)) {
        info->fragment_align = imx->rx_cfg.sample_size * imx->rx_cfg.nslots;
        for (i = 0; i < imx->num_rx_aif; i++) {
            if (pcm == imx->aif[i].pcm && imx->aif[i].cap_strm.subchn) {
                chn_avail = 0;
                break;
            }
        }
        if (chn_avail && (imx->clk_mode == IMX_SAI_MASTER)
            && (imx->rx_cfg.sample_rate_min != imx->rx_cfg.sample_rate_max) ) {
            ado_mutex_lock(&imx->lock);
            if (num_active_capture_interfaces(imx)) {
                /*
                 * Since we only have one data line, sequential capture stream rate
                 * should be locked by first capture stream
                 */
                info->min_rate = info->max_rate = imx->rx_cfg.sample_rate;
                info->rates = ado_pcm_rate2flag(imx->rx_cfg.sample_rate);
            } else if (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX) {
                /* Playback and Capture are Rate locked, so adjust rate capabilities
                 * if the other side has been acquired.
                 */
                if (num_active_playback_interfaces(imx)) {
                    info->min_rate = info->max_rate = imx->tx_cfg.sample_rate;
                    info->rates = ado_pcm_rate2flag(imx->tx_cfg.sample_rate);
                }
            }
            ado_mutex_unlock(&imx->lock);
        }
    } else {
        chn_avail = 0;
    }
    /* There are no available channels */
    if (chn_avail == 0) {
        ado_error_fmt("No available channels for channel %d", info->channel);
        info->formats = 0;
        info->rates = 0;
        info->min_rate = 0;
        info->max_rate = 0;
        info->min_voices = 0;
        info->max_voices = 0;
        info->min_fragment_size = 0;
        info->max_fragment_size = 0;
    }
    return (EOK);
}

/*
 * Sets SAI clock rate according to required sample rate etc.
 */
static int imx_sai_set_clock_rate(HW_CONTEXT_T *imx, int rate, int channel)
{
    uint32_t i = 0;
    imx_sai_rxtx_t *reg;
    imx_sai_xfer_config_t *cfg;
    uint32_t active_streams;

    if (channel == ADO_PCM_CHANNEL_PLAYBACK) {
        if (imx->tx_reg == NULL) {
            ado_error_fmt("imx->tx_reg uninitialized");
            return -1;
        }
        reg = imx->tx_reg;
        cfg = &imx->tx_cfg;
        active_streams = num_active_playback_interfaces(imx);
    } else {
        if (imx->rx_reg == NULL) {
            ado_error_fmt("imx->rx_reg uninitialized");
            return -1;
        }
        reg = imx->rx_reg;
        cfg = &imx->rx_cfg;
        active_streams = num_active_capture_interfaces(imx);
    }

    ado_mutex_lock(&imx->lock);
    /* Disable Tx and Rx */
    if ((num_active_playback_interfaces(imx) || num_active_capture_interfaces(imx)) && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
        /* In synchronous mode disable both Tx, Rx */
        imx->tx_reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
        imx->rx_reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
        /* Wait for frame complete */
        while (((imx->tx_reg->csr & IMX_SAI_CSR_EN_MASK)
                || (imx->rx_reg->csr & IMX_SAI_CSR_EN_MASK)) && i < IMX_SAI_WAIT_TIMEOUT) {
            i++;
            ado_mutex_unlock(&imx->lock);
            usleep(100);
            ado_mutex_lock(&imx->lock);
        }
    } else {
        if (active_streams) {
            reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
            /* Wait for frame complete */
            while ((reg->csr & IMX_SAI_CSR_EN_MASK) && i < IMX_SAI_WAIT_TIMEOUT) {
                i++;
                ado_mutex_unlock(&imx->lock);
                usleep(100);
                ado_mutex_lock(&imx->lock);
            }
        }
    }

    if (imx->clk_mode == IMX_SAI_MASTER) {
        /* Configure as Master */
        uint32_t pm;
        uint32_t f_bit_clk;
        uint32_t remainder;

        cfg->curr_slot_size = cfg->conf_slot_size;
        f_bit_clk = rate * cfg->nslots * cfg->curr_slot_size;

        remainder = imx->sys_clk % (2 * f_bit_clk);
        if (remainder) {
            /* System clock may not be even multiple times of bit clock so that clock divider value is not correct.
             * If slot_size is bigger than sample size, we drop slot size back to sample size and calculate a new
             * bit clock according to sample size. If the system clock can support this new bit block, we
             * re-initialize slot_size related registers to use sample size as slot size.
             */
            f_bit_clk = rate * cfg->nslots * cfg->sample_size * _BITS_BYTE;
            remainder = imx->sys_clk % (2 * f_bit_clk);
            if ( remainder == 0 ) {
                cfg->curr_slot_size = cfg->sample_size * _BITS_BYTE;
            } else {
                ado_error_fmt("channel %d: system clock %u is not even multiple times of bit clock %u", channel, imx->sys_clk , f_bit_clk);
                return EINVAL;
            }
        }

        ado_debug(DB_LVL_DRIVER, "channel %d: sample rate %d: current slot size is %d, configured slot size is %d, sample size is %d, bit clock is %u, system clock is %u",
                channel, rate, cfg->curr_slot_size, cfg->conf_slot_size, cfg->sample_size * _BITS_BYTE, f_bit_clk, imx->sys_clk);

        pm = imx->sys_clk / (2 * f_bit_clk) - 1;

        if (pm > 0xFF) {
            ado_error_fmt("Wrong MCLK to bit clock divider for sample rate %u", rate);
            return EINVAL;
        }

        /* Slot size, i.e, word width. From iMx8QM document, word width of less than 8 bits is not supported.*/
        if ((cfg->curr_slot_size < 8) || (cfg->curr_slot_size > 32)) {
            ado_error_fmt("channel %d: Wrong slot size : %u", channel, cfg->curr_slot_size);
            return EINVAL;
        }

        /* Configure bit clock */
        if (imx->num_tx_aif && imx->num_rx_aif && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
            /* Frame sync length */
            if (imx->tx_cfg.sync_len > cfg->curr_slot_size ) {
                imx->tx_cfg.sync_len = cfg->curr_slot_size;
                // set sync length
                imx->tx_reg->cr4 &= ~IMX_SAI_CR4_SYWD_MASK;
                imx->tx_reg->cr4 |= ((imx->tx_cfg.sync_len - 1) << IMX_SAI_CR4_SYWD_SHIFT) & IMX_SAI_CR4_SYWD_MASK;
            }
            if (imx->rx_cfg.sync_len > cfg->curr_slot_size) {
                imx->rx_cfg.sync_len = cfg->curr_slot_size;
                // set sync length
                imx->rx_reg->cr4 &= ~IMX_SAI_CR4_SYWD_MASK;
                imx->rx_reg->cr4 |= ((imx->rx_cfg.sync_len - 1) << IMX_SAI_CR4_SYWD_SHIFT) & IMX_SAI_CR4_SYWD_MASK;
            }

            imx->tx_reg->cr2 &= ~0xFF;
            imx->tx_reg->cr2 |= pm;
            imx->rx_reg->cr2 &= ~0xFF;
            imx->rx_reg->cr2 |= pm;
            imx->tx_reg->cr2 |= IMX_SAI_CR2_BCD_MASK;
            imx->tx_reg->cr4 |= IMX_SAI_CR4_FSD_MASK;
            imx->rx_reg->cr2 |= IMX_SAI_CR2_BCD_MASK;
            imx->rx_reg->cr4 |= IMX_SAI_CR4_FSD_MASK;
        } else {
            if (cfg->sync_len > cfg->curr_slot_size) {
                cfg->sync_len = cfg->curr_slot_size;
                reg->cr4 &= ~IMX_SAI_CR4_SYWD_MASK;
                reg->cr4 |= ((cfg->sync_len - 1) << IMX_SAI_CR4_SYWD_SHIFT) & IMX_SAI_CR4_SYWD_MASK;
            }

            reg->cr2 &= ~0xFF;
            reg->cr2 |= pm;
            reg->cr2 |= IMX_SAI_CR2_BCD_MASK;
            reg->cr4 |= IMX_SAI_CR4_FSD_MASK;
        }
    } else {
        /* Configure as Slave */
        if (imx->num_tx_aif && imx->num_rx_aif && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
            imx->tx_reg->cr2 &= ~IMX_SAI_CR2_BCD_MASK;
            imx->tx_reg->cr4 &= ~IMX_SAI_CR4_FSD_MASK;
            imx->rx_reg->cr2 &= ~IMX_SAI_CR2_BCD_MASK;
            imx->rx_reg->cr4 &= ~IMX_SAI_CR4_FSD_MASK;

        } else {
            reg->cr2 &= ~IMX_SAI_CR2_BCD_MASK;
            reg->cr4 &= ~IMX_SAI_CR4_FSD_MASK;
        }
    }

    if (imx->num_tx_aif && imx->num_rx_aif && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
        imx->tx_cfg.curr_slot_size = cfg->curr_slot_size;
        imx->rx_cfg.curr_slot_size = cfg->curr_slot_size;

        imx->tx_reg->cr5 &= ~IMX_SAI_CR5_WNW_MASK;
        imx->tx_reg->cr5 &= ~IMX_SAI_CR5_W0W_MASK;
        imx->tx_reg->cr5 |= ((imx->tx_cfg.curr_slot_size - 1) << IMX_SAI_CR5_WNW_SHIFT)
                            & (IMX_SAI_CR5_WNW_MASK);
        imx->tx_reg->cr5 |= ((imx->tx_cfg.curr_slot_size - 1) << IMX_SAI_CR5_W0W_SHIFT)
                            & (IMX_SAI_CR5_W0W_MASK);
        imx->rx_reg->cr5 &= ~IMX_SAI_CR5_WNW_MASK;
        imx->rx_reg->cr5 &= ~IMX_SAI_CR5_W0W_MASK;
        imx->rx_reg->cr5 |= ((imx->rx_cfg.curr_slot_size - 1) << IMX_SAI_CR5_WNW_SHIFT)
                            & (IMX_SAI_CR5_WNW_MASK);
        imx->rx_reg->cr5 |= ((imx->rx_cfg.curr_slot_size - 1) << IMX_SAI_CR5_W0W_SHIFT)
                            & (IMX_SAI_CR5_W0W_MASK);
    } else {
        reg->cr5 &= ~IMX_SAI_CR5_WNW_MASK;
        reg->cr5 &= ~IMX_SAI_CR5_W0W_MASK;
        reg->cr5 |= ((cfg->curr_slot_size - 1) << IMX_SAI_CR5_WNW_SHIFT) & (IMX_SAI_CR5_WNW_MASK);
        reg->cr5 |= ((cfg->curr_slot_size - 1) << IMX_SAI_CR5_W0W_SHIFT) & (IMX_SAI_CR5_W0W_MASK);
    }

    /* Enable Tx and Rx */
    if (imx->num_tx_aif && imx->num_rx_aif && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
        codec_set_rate(imx, rate, ADO_PCM_CHANNEL_PLAYBACK);
        /* If Rx stream is running in sync mode - re-enable both Tx and Rx */
        if (num_active_capture_interfaces(imx)) {
            imx->rx_reg->csr |= (IMX_SAI_CSR_EN_MASK);
            imx->tx_reg->csr |= (IMX_SAI_CSR_EN_MASK);

        } else if (num_active_playback_interfaces(imx)) {
            /* Enable only Tx if stream is running */
            imx->tx_reg->csr |= (IMX_SAI_CSR_EN_MASK);
        }
    } else {
        codec_set_rate(imx, rate, channel);
        /* Asynchronous mode */
        if (active_streams) {
            reg->csr |= (IMX_SAI_CSR_EN_MASK);
        }
    }

    /* Store sample rate */
    if (imx->num_tx_aif && imx->num_rx_aif && (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX)) {
        imx->tx_cfg.sample_rate = rate;
        imx->rx_cfg.sample_rate = rate;
    } else {
        cfg->sample_rate = rate;
    }

    ado_mutex_unlock(&imx->lock);
    return (EOK);
}

/*
 * Called when a client attempts to open a playback PCM stream.
 */
int32_t imx_playback_aquire(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T **pc, ado_pcm_config_t *config,
                            ado_pcm_subchn_t *subchn, uint32_t *why_failed)
{
    uint32_t i;
    imx_aif_t *aif = NULL;

    ado_mutex_lock(&imx->lock);

    for ((*pc) = NULL, i = 0; i < imx->num_tx_aif; i++)
    {
        if (ado_pcm_subchn_is_channel(subchn, imx->aif[i].pcm, ADO_PCM_CHANNEL_PLAYBACK))
        {
            aif = &imx->aif[i];
            *pc = &aif->play_strm;
            break;
        }
    }

    if ((*pc) == NULL || (*pc)->subchn || aif == NULL)
    {
        *why_failed = SND_PCM_PARAMS_NO_CHANNEL;
        ado_mutex_unlock(&imx->lock);
        return EAGAIN;
    }

    (*pc)->active = 0;

    /* If multiple rates supported check for rate switch */
    if ((imx->clk_mode == IMX_SAI_MASTER)
        && imx->tx_cfg.sample_rate_min != imx->tx_cfg.sample_rate_max) {

        /* playback rate is locked by active playback or
         * active capture when they are in synchronous mode.
         */
        if (config->format.rate != imx->tx_cfg.sample_rate) {
            if ((num_active_playback_interfaces(imx) == 0) && ((num_active_capture_interfaces(imx) == 0) || imx->xfer_sync_mode == IMX_SAI_ASYNC)) {
                ado_mutex_unlock(&imx->lock);
                imx_sai_set_clock_rate(imx, config->format.rate, ADO_PCM_CHANNEL_PLAYBACK);
                ado_mutex_lock(&imx->lock);
            } else {
                /* In case there is request on different sample rate but we can't set it because
                 * Tx is running in synchronous mode or other TX is running -> report EBUSY
                 */
                ado_mutex_unlock(&imx->lock);
                return (EBUSY);
            }
        }
    }

    /* Allocate DMA buffer */
    config->dmabuf.flags = ADO_BUF_CACHE;
    /* If num_tx_aif == 1, then we DMA right into the pcm buffer so make it DMA safe */
    if (imx->num_tx_aif == 1)
    {
        config->dmabuf.flags |= ADO_SHM_DMA_SAFE;
        imx->tx_fifo_watermark = calculate_fifo_watermark(config, &imx->tx_cfg);
    }

    if (ado_pcm_buf_alloc(config, config->dmabuf.size, config->dmabuf.flags) == NULL) {
        ado_mutex_unlock(&imx->lock);
        return (errno);
    }
    /* Only setup the DMA tranfer if there is only 1 playback interface/device.
     * If multiple playback devices then a ping-pong DMA buffer will be allocated and initialized in ctrl_init().
     */
    if (imx->num_tx_aif == 1)
    {
        dma_transfer_t tinfo;
#if IMX_EDMA
        dma_addr_t dst_addr;
#endif
        int frag_idx;

        /* Initialize DMA info structure */
        memset(&tinfo, 0, sizeof(tinfo));
        /* Higher layer defines number of fragments in DMA buffer so we will copy these
         * fragments into SAI by DMA */
        tinfo.src_addrs = ado_calloc(1, config->mode.block.frags_total * sizeof(dma_addr_t));
        if (tinfo.src_addrs == NULL) {
            ado_pcm_buf_free(config);
            ado_mutex_unlock(&imx->lock);
            return -1;
        }
#if IMX_EDMA
        /* eDMA driver specific initialization */
        tinfo.dst_addrs = &dst_addr;
        tinfo.dst_fragments = 1;
        /* Initialize dst_addr to physical address of TX register */
        tinfo.dst_addrs->paddr = imx->base + IMX_SAI_TDR_OFFSET;
        tinfo.dst_addrs->len = imx->tx_cfg.sample_size;
        /* Register address cannot be incremented */
        tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT;
#endif
        for (frag_idx = 0; frag_idx < config->mode.block.frags_total; frag_idx++) {
            /* Initialize source address of every fragment */
            tinfo.src_addrs[frag_idx].paddr = config->dmabuf.phys_addr + (frag_idx * config->mode.block.frag_size);
            tinfo.src_addrs[frag_idx].len = config->mode.block.frag_size;
        }
        tinfo.src_fragments = config->mode.block.frags_total;
        tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
        /* xfer_unit_size depends on Audio sample size */
        tinfo.xfer_unit_size = (imx->tx_cfg.sample_size == 2) ? 16 : 32;
#if IMX_EDMA
        tinfo.xfer_bytes = imx->tx_fifo_watermark;
#endif
        imx->dmafuncs.setup_xfer((*pc)->dma.chn, &tinfo);

        ado_free(tinfo.src_addrs);
    }

    /* Store subchannel and config */
    (*pc)->subchn = subchn;
    (*pc)->config = config;
    ado_mutex_unlock(&imx->lock);
    return (EOK);
}

/**
 * Called by upper layer when client closes its connection to the device.
 */
int32_t imx_playback_release(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T *pc, ado_pcm_config_t *config)
{
    ado_mutex_lock(&imx->lock);
    /* Clear data */
    ado_pcm_buf_free(config);
    pc->subchn = NULL;
    pc->active = 0;
    ado_mutex_unlock(&imx->lock);
    return (EOK);
}

/**
 * Called when a client attempts to open a capture PCM stream.
 */
int32_t imx_capture_aquire(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T **pc, ado_pcm_config_t *config,
                           ado_pcm_subchn_t *subchn, uint32_t *why_failed)
{
    uint32_t i;
    imx_aif_t *aif = NULL;

    ado_mutex_lock(&imx->lock);
    for ((*pc) = NULL, i = 0; i < imx->num_rx_aif; i++)
    {
        if (ado_pcm_subchn_is_channel(subchn, imx->aif[i].pcm, ADO_PCM_CHANNEL_CAPTURE))
        {
            aif = &imx->aif[i];
            *pc = &aif->cap_strm;
            break;
        }
    }

    if ((*pc) == NULL || (*pc)->subchn || aif == NULL)
    {
        *why_failed = SND_PCM_PARAMS_NO_CHANNEL;
        ado_mutex_unlock(&imx->lock);
        return EAGAIN;
    }

    (*pc)->active = 0;

    /* If multiple rates supported check for rate switch */
    if ((imx->clk_mode == IMX_SAI_MASTER)
        && imx->rx_cfg.sample_rate_min != imx->rx_cfg.sample_rate_max) {

        /* capture rate is locked by active capture or active playback
         * when they are in synchronous mode.
         */
        if (config->format.rate != imx->rx_cfg.sample_rate) {
            if ((num_active_capture_interfaces(imx) == 0) && ((num_active_playback_interfaces(imx) == 0) || imx->xfer_sync_mode == IMX_SAI_ASYNC)) {
                ado_mutex_unlock(&imx->lock);
                imx_sai_set_clock_rate(imx, config->format.rate, ADO_PCM_CHANNEL_CAPTURE);
                ado_mutex_lock(&imx->lock);
            } else {
                /* In case there is request on different sample rate but we can't set it because
                 * Rx is running in synchronous mode or other RX is running -> report EBUSY
                 */
                ado_mutex_unlock(&imx->lock);
                return (EBUSY);
            }
        }
    }

    config->dmabuf.flags = ADO_BUF_CACHE;
    /* If num_rx_aif == 1, then we DMA right into the pcm buffer so make it DMA safe */
    if (imx->num_rx_aif == 1) {
        config->dmabuf.flags |= ADO_SHM_DMA_SAFE;
        imx->rx_fifo_watermark = calculate_fifo_watermark(config, &imx->rx_cfg);
    }
    /* Allocate DMA buffer */
    if (ado_pcm_buf_alloc(config, config->dmabuf.size, config->dmabuf.flags) == NULL) {
        ado_mutex_unlock(&imx->lock);
        return (errno);
    }
    /* Only setup the DMA tranfer if there is only 1 capture interface/device.
     * If multiple capture devices then a ping-pong DMA buffer will be allocated and initialized in ctrl_init().
     */
    if (imx->num_rx_aif == 1) {
        dma_transfer_t tinfo;
#if IMX_EDMA
        dma_addr_t src_addr;
#endif
        int frag_idx;

        memset(&tinfo, 0, sizeof(tinfo));
        /* We will copy segments from SAI to buffer by DMA */
        tinfo.dst_addrs = ado_calloc(1, config->mode.block.frags_total * sizeof(dma_addr_t));

        if (tinfo.dst_addrs == NULL) {
            ado_error_fmt("Insufficient memory");
            ado_pcm_buf_free(config);
            ado_mutex_unlock(&imx->lock);
            return (errno);
        }
#if IMX_EDMA
        tinfo.src_addrs = &src_addr;
        tinfo.src_fragments = 1;
        tinfo.src_addrs->paddr = imx->base + IMX_SAI_RDR_OFFSET;
        tinfo.src_addrs->len = imx->rx_cfg.sample_size;
        /* Register address cannot be incremented */
        tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT;
#endif
        for (frag_idx = 0; frag_idx < config->mode.block.frags_total; frag_idx++) {
            /* Initialize destination address of every fragment */
            tinfo.dst_addrs[frag_idx].paddr = config->dmabuf.phys_addr + (frag_idx * config->mode.block.frag_size);
            tinfo.dst_addrs[frag_idx].len = config->mode.block.frag_size;
        }
        tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
        tinfo.dst_fragments = config->mode.block.frags_total;
        /* xfer_unit_size depends on sample_size */
        tinfo.xfer_unit_size = (imx->rx_cfg.sample_size == 2) ? 16 : 32;
#if IMX_EDMA
        tinfo.xfer_bytes = imx->rx_fifo_watermark;
#endif
        imx->dmafuncs.setup_xfer((*pc)->dma.chn, &tinfo);
        ado_free(tinfo.dst_addrs);
    }
    /* Store subchannel and config */
    (*pc)->subchn = subchn;
    (*pc)->config = config;
    ado_mutex_unlock(&imx->lock);
    return (EOK);
}

/**
 * Called by upper layer when client closes its connection to the device.
 */
int32_t imx_capture_release(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T *pc, ado_pcm_config_t *config)
{
    ado_mutex_lock(&imx->lock);
    /* Clear data */
    ado_pcm_buf_free(config);
    pc->subchn = NULL;
    pc->active = 0;
    ado_mutex_unlock(&imx->lock);
    return (EOK);
}

/**
 * Called by upper layer to prepare hardware before it's started up.
 */
int32_t
imx_prepare (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * pc, ado_pcm_config_t * config)
{
    pc->pcm_offset = 0;
    return (EOK);
}

/**
 * Called by upper layer to start or stop playback subchannel.
 */
int32_t imx_playback_trigger(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T *pc, uint32_t cmd)
{
    int32_t rtn = EOK;

    ado_mutex_lock(&imx->lock);
    /* Check command type */
    if (cmd == ADO_PCM_TRIGGER_GO) {
        if (num_active_playback_interfaces(imx) == 0) {
            imx->playback_frag_index = 0;
            if (imx->num_tx_aif > 1)
            {
                /* We use a ping-ping buffer for DMA, so divide buffer size by 2 to get the frag size */
                const size_t size = imx->playback_dmabuf.size / 2;

                /* Combine PCM data from the various PCM playback devices into the DMA buffer,
                 * Fill the entire DMA buffer (both fragments), we will backfill the DMA buffer
                 * on the interrupt completion interrupt/event
                 *
                 * Note: We cannot call dma_interrupt more than once from the trigger since the sw_mix
                 *       processing cannot complete until trigger function completes.
                 */
                imx_dmaplayback_combine(imx, (void*)(&imx->playback_dmabuf.addr[0]), size, false);
                imx_dmaplayback_combine(imx, (void*)(&imx->playback_dmabuf.addr[size]), size, true);
            }
            rtn = imx_sai_start_tx(imx);
            if(rtn != EOK)
            {
                ado_error_fmt("Couldn't start tx:%s", strerror(rtn));
            }
        }
        pc->active = 1;
    } else {
        if (num_active_playback_interfaces(imx) == 1) {
            /* Disable TX.
             * In Tx Rx synchronous mode disable only if Rx is not running.
             */
            /* Clear FIFO error */
            if (imx->tx_reg->csr & IMX_SAI_CSR_FEF_MASK) {
                ado_error_fmt("FIFO error detected. TCSR = 0x%x", imx->tx_reg->csr);
                imx->tx_reg->csr |= IMX_SAI_CSR_FEF_MASK;
            }
            if ((num_active_capture_interfaces(imx) == 0) || imx->xfer_sync_mode == IMX_SAI_ASYNC) {
                imx->tx_reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
            }
            if (imx->dmafuncs.xfer_abort(imx->aif[0].play_strm.dma.chn) == -1) {
                rtn = errno;
                ado_error_fmt("Audio DMA Stop failed (%s)", strerror(rtn));
            }
            /* Mute playback */
            codec_off(imx, ADO_PCM_CHANNEL_PLAYBACK);
        }
        pc->active = 0;
    }
    ado_mutex_unlock(&imx->lock);
    return (rtn);
}

/**
 * Called by upper layer to start or stop capture subchannel.
 */
int32_t imx_capture_trigger(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T *pc, uint32_t cmd)
{
    int rtn = EOK;

    ado_mutex_lock(&imx->lock);
    /* Check command type */
    if (cmd == ADO_PCM_TRIGGER_GO) {
        if (num_active_capture_interfaces(imx) == 0) {
            imx->capture_frag_index = 0;
            /* Reset FIFO */
            imx->rx_reg->csr |= IMX_SAI_CSR_FR_MASK;

            /* Set fifo water mark (in unit of samples)*/
            imx->rx_reg->cr1 = imx->rx_fifo_watermark / imx->rx_cfg.sample_size;

            /* Clear errors */
            if (imx->rx_reg->csr & IMX_SAI_CSR_FEF_MASK) {
                imx->rx_reg->csr |= IMX_SAI_CSR_FEF_MASK;
            }
            if (imx->rx_reg->csr & IMX_SAI_CSR_SEF_MASK) {
                imx->rx_reg->csr |= IMX_SAI_CSR_SEF_MASK;
            }
            /* Include the 7th bit of the watermark to distinguish between empty and full
             * when the 6 LSBs of both the read and write FIFO pointers are the same
             */
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
            if (((imx->rx_reg->fr[0] >> 16) != (imx->rx_reg->fr[0] & 0x7F))) {
#else
            if (((imx->rx_reg->fr >> 16) != (imx->rx_reg->fr & 0x7F))) {
#endif
                ado_error_fmt("AUDIO RX FIFO not empty");
            }
            if (imx->dmafuncs.xfer_start(imx->aif[0].cap_strm.dma.chn) == -1) {
                rtn = errno;
                ado_error_fmt("Audio DMA Start failed (%s)", strerror(rtn));
            }
            /* Enable receiver */
            imx->rx_reg->csr |= (IMX_SAI_CSR_EN_MASK);
            /* Enable also Tx in case Rx is synchronized with Tx */
            if (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX) {
                if (imx->num_tx_aif &&
                    ((imx->tx_reg->csr & IMX_SAI_CSR_EN_MASK) != (uint32_t)IMX_SAI_CSR_EN_MASK)) {
                    /* Reset TX FIFO */
                    imx->tx_reg->csr |= IMX_SAI_CSR_FR_MASK;
                    nanospin_ns(1000);
                    imx->tx_reg->csr &= ~(IMX_SAI_CSR_FR_MASK);
                    /* Enable TX*/
                    imx->tx_reg->csr |= (IMX_SAI_CSR_EN_MASK);
                }
            }
            /* Un-mute capture */
            codec_on(imx, ADO_PCM_CHANNEL_CAPTURE);
        }
        pc->active = 1;
    } else {
        if (num_active_capture_interfaces(imx) == 1) {
            /* Disable receiver */
            imx->rx_reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
            /* Disable also Tx in case Rx is synchronized with Tx */
            if (imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX) {
                /* Disable transmitter if not playing */
                if (num_active_playback_interfaces(imx) == 0) {
                    imx->tx_reg->csr &= ~(IMX_SAI_CSR_EN_MASK);
                }
            }
            if (imx->dmafuncs.xfer_abort(imx->aif[0].cap_strm.dma.chn) == -1) {
                rtn = errno;
                ado_error_fmt("Audio DMA Stop failed (%s)", strerror(rtn));
            }
            /* Mute capture */
            codec_off(imx, ADO_PCM_CHANNEL_CAPTURE);
        }
        pc->active = 0;
    }
    ado_mutex_unlock(&imx->lock);
    return (rtn);
}

/*
 * No position function as we are unable to get the transfer count of the
 * current DMA operation from the DMA microcode because bytes_left() function
 * in edma library doesn't support DMA_MODE_FLAG_REPEAT mode. The resolution
 * of the positional information returned to the client will be limited to the
 * fragment size.
 *
 * If we get new DMA microcode that supports this and the dma library's
 * bytes_left() function is updated to use this info to return the actual
 * bytes left, uncomment the below function and the function pointer
 * assignments in ctrl_init().
 */

#if 0
static uint32_t
imx_position(HW_CONTEXT_T *imx, PCM_SUBCHN_CONTEXT_T *pc, ado_pcm_config_t *config, uint32_t *hw_buffer_level)
{
    uint32_t pos = 0;

    ado_mutex_lock(&imx->lock);
    if (imx->strm[IMX_SAI_TX] && pc->pcm.subchn == imx->strm[IMX_SAI_TX]->pcm.subchn) {
        pos = (imx->strm[IMX_SAI_TX]->pcm.pcm_cur_frag * ado_pcm_dma_int_size(config)) +
            (ado_pcm_dma_int_size(config) -
            imx->dmafuncs.bytes_left(imx->strm[IMX_SAI_TX]->dma.chn));
    }
    if (imx->strm[IMX_SAI_RX] && pc->pcm.subchn == imx->strm[IMX_SAI_RX]->pcm.subchn) {
        pos = (imx->strm[IMX_SAI_RX]->pcm.pcm_cur_frag * ado_pcm_dma_int_size(config)) +
            ado_pcm_dma_int_size(config) -
            imx->dmafuncs.bytes_left(imx->strm[IMX_SAI_RX]->dma.chn);
    }

    ado_mutex_unlock(&imx->lock);

    return (pos);
}
#endif

static int reset_regs (HW_CONTEXT_T *imx, int channel)
{
    imx_sai_rxtx_t *reg;
    imx_sai_xfer_config_t *cfg;

    if (channel == ADO_PCM_CHANNEL_PLAYBACK) {
        reg = imx->tx_reg;
        cfg = &imx->tx_cfg;
    } else {
        reg = imx->rx_reg;
        cfg = &imx->rx_cfg;
    }
    /* Software reset and FIFO reset */
    reg->csr = IMX_SAI_CSR_SR_MASK | IMX_SAI_CSR_FR_MASK;
    nanospin_ns(100);
    /* Calculate FIFO size */
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
    cfg->fifo_size = 1 << ((imx->id_reg->param >> 8) & 0xF);
#else
    reg->cr1 = IMX_SAI_MAX_FIFO_SIZE;
    cfg->fifo_size = reg->cr1 + 1;
    if (cfg->fifo_size > IMX_SAI_MAX_FIFO_SIZE) {
        ado_error_fmt("Unable to calculate FIFO size %u", cfg->fifo_size);
    }
#endif
    if (cfg->fifo_size <= IMX_SAI_FIFO_WATERMARK) {
        ado_error("Watermark >= FIFO size");
        return -1;
    }
    /* Reset registers */
    reg->csr = 0;
    reg->cr1 = 0;
    reg->cr2 = 0;
    reg->cr3 = 0;
#if IMX_SAI_CONTINUE_ON_FEF
    reg->cr4 = IMX_SAI_CR4_FCONT_MASK;
#else
    reg->cr4 = 0;
#endif
    reg->cr5 = 0;

    /* Mask all the tx/rx time slots */
    reg->mr = 0xFFFFFFFF;

    /* Synchronous clock setting */
    if ((channel == ADO_PCM_CHANNEL_CAPTURE) && imx->xfer_sync_mode == IMX_SAI_SYNC_RX_WITH_TX) {
        imx->rx_reg->cr2 |= (1 << IMX_SAI_CR2_SYNC_SHIFT);
    }

    if (!cfg->clk_pol) {
        /* Data is clocked out on falling edge of bit clock */
        reg->cr2 |= (IMX_SAI_CR2_BCP_MASK);
    }

    /* Frame sync length */
    if (cfg->sync_len > 0 && cfg->sync_len <= 32) {
        reg->cr4 |= ((cfg->sync_len - 1) << IMX_SAI_CR4_SYWD_SHIFT) & IMX_SAI_CR4_SYWD_MASK;
    } else {
        ado_error_fmt("Wrong frame sync length : %u", cfg->sync_len);
    }

    if (!cfg->sync_pol) {
        /* Frame sync is active low  */
        reg->cr4 |= IMX_SAI_CR4_FSP_MASK;
    }

    if (cfg->bit_delay) {
        /* 0-bit delay - early frame sync */
        reg->cr4 |= IMX_SAI_CR4_FSE_MASK;
    }

    /* Words per frame */
    if ((cfg->nslots > 0 && cfg->nslots <= 32)) {

        reg->cr4 |= ((cfg->nslots - 1) << IMX_SAI_CR4_FRSZ_SHIFT) & IMX_SAI_CR4_FRSZ_MASK;

    } else {
        ado_error_fmt("Wrong words per frame : %u", cfg->nslots);
        return -1;
    }

    /* MSB first */
    reg->cr4 |= IMX_SAI_CR4_MF_MASK;
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
    if (imx->tx_cfg.protocol == IMX_SAI_PROTOCOL_I2S) {
        if (channel == ADO_PCM_CHANNEL_PLAYBACK) {
            /* Output mode, transmit data pins are never tri-stated and will output zero when slots are
               masked or channels are disabled. */
            reg->cr4 |= IMX_SAI_CR4_CHMOD_MASK;
        }
    }
#endif
    /* Configures the shift register bit index to use as the first bit transmitted of each word in the frame.
     * For MSB first, configures the bit index of the MSB bit of the valid sample in the 32bit shift register,
     * i.e., if sample size is 16 then bit 15 is the MSB bit, bits 31-16 are ignored.
     */
    reg->cr5 |= (cfg->sample_size * _BITS_BYTE - 1) << IMX_SAI_CR5_FBT_SHIFT;

    /* Enable channel */
    reg->cr3 |= IMX_SAI_CR3_CE_MASK;

    /* Enable DMA TX/RX Event */
    reg->csr |= IMX_SAI_CSR_FRDE_MASK;

    reg->cr2 = ((reg->cr2) & ~(IMX_SAI_CR2_MSEL_MASK)) | (cfg->msel << IMX_SAI_CR2_MSEL_SHIFT);
    return 0;
}
/*
 * Initializes SAI hardware.
 */
static int imx_sai_init(HW_CONTEXT_T *imx)
{
    uint32_t i;
    uint32_t tmp, tmp1;

    ado_mutex_lock(&imx->lock);
    /* Initialize both Tx and Rx */
    if (imx->num_tx_aif) {
        if (reset_regs(imx, ADO_PCM_CHANNEL_PLAYBACK) == -1) {
            ado_mutex_unlock(&imx->lock);
            return -1;
        }
    }
    if (imx->num_rx_aif) {
        if (reset_regs(imx, ADO_PCM_CHANNEL_CAPTURE) == -1) {
            ado_mutex_unlock(&imx->lock);
            return -1;
        }
    }

    /* Enable time slots according to number of voices of each device */
    tmp = 0;
    tmp1 = 0;
    for (i = 0; i < imx->num_tx_aif; i++) {
        if (imx->aif[0].play_strm.voices == 32) {
            tmp1 = 0xFFFFFFFF;
            break;
        } else {
            tmp = 2;
            uint32_t j;
            /* Enable/unmask time slot*/
            for (j = 1; j < imx->aif[i].play_strm.voices; j++) {
                tmp *= 2;
            }
            tmp--;
            tmp1 |= (tmp << (i * imx->aif[i].play_strm.voices));
            ado_debug(DB_LVL_DRIVER, "tx mr tmp1 is 0x%x", tmp1);
        }
    }
    if (imx->num_tx_aif) {
        imx->tx_reg->mr &= ~(tmp1);
    }

    tmp = 0;
    tmp1 = 0;
    for (i = 0; i < imx->num_rx_aif; i++) {
        if (imx->aif[0].cap_strm.voices == 32) {
            tmp1 = 0xFFFFFFFF;
            break;
        } else {
            tmp = 2;
            uint32_t j;
            /* Enable/unmask time slot*/
            for (j = 1; j < imx->aif[i].cap_strm.voices; j++) {
                tmp *= 2;
            }
            tmp--;
            tmp1 |= (tmp << (i * imx->aif[i].cap_strm.voices));
            ado_debug(DB_LVL_DRIVER, "rx mr tmp1 is 0x%x", tmp1);
        }
    }
    if (imx->num_rx_aif) {
        imx->rx_reg->mr &= ~(tmp1);
    }

    /* imx_sai_set_clock_rate() will enable the SAI for us */
    if (imx->num_tx_aif) {
        if (imx_sai_set_clock_rate(imx, imx->tx_cfg.sample_rate_max, ADO_PCM_CHANNEL_PLAYBACK) != EOK) {
            ado_mutex_unlock(&imx->lock);
            return -1;
        }
        /* Enable bit clock  */
        imx->tx_reg->csr |= (IMX_SAI_CSR_BCE_MASK);
    }
    if (imx->num_rx_aif) {
        if((imx->num_tx_aif == 0) || (imx->xfer_sync_mode == IMX_SAI_ASYNC)) {
            if (imx_sai_set_clock_rate(imx, imx->rx_cfg.sample_rate_max, ADO_PCM_CHANNEL_CAPTURE) != EOK) {
                ado_mutex_unlock(&imx->lock);
                return -1;
            }
        }
        /* Enable bit clock  */
        imx->rx_reg->csr |= (IMX_SAI_CSR_BCE_MASK);
    }

    ado_mutex_unlock(&imx->lock);
    return EOK;
}

ado_dll_version_t ctrl_version;
/*
 * Initializes SAI driver library version.
 */
void ctrl_version(int *major, int *minor, char *date)
{
    *major = ADO_MAJOR_VERSION;
    *minor = 1;
    date = __DATE__;
    /* Parameter is not used, suppress unused argument warning */
    (void)date;
}

/*
 * This function configures the various protocol specific flags.
 */
static void imx_sai_config_default_protocol_flags(imx_sai_xfer_config_t *config, imx_txrx_index_t index)
{
    switch (config->protocol) {
        /* Note: If the value is not 0xFF then it was overriden on the
         *       command line so don't set it based on the protocol flag
         */
        case IMX_SAI_PROTOCOL_PCM:
            if (config->clk_pol == 0xFF)
                config->clk_pol = 1; /* On rising edge */
            if (config->bit_delay == 0xFF)
                config->bit_delay = 1; /* 1 bit delay */
            if (config->sync_pol == 0xFF)
                config->sync_pol = 1; /* Active high frame sync */
            break;
        case IMX_SAI_PROTOCOL_I2S:
            if (config->clk_pol == 0xFF)
            {
                if (index == IMX_SAI_TX) {
                    config->clk_pol = 0;    /* TX on falling edge */
                } else {
                    config->clk_pol = 1;    /* RX on rising edge */
                }
            }
            if (config->bit_delay == 0xFF)
                config->bit_delay = 1; /* 1 bit delay */
            if (config->sync_pol == 0xFF)
                config->sync_pol = 0; /* Active low frame sync */
            break;
        default:
            ado_error_fmt("Unknown SAI protocol");
            break;
    }
}

/*
 * Obtain base address and DMA requests from hwinfo table
 */
int query_hwi_device(HW_CONTEXT_T *imx, unsigned unit)
{
    uint32_t zero = 0;
    /* Getting the SAI Base addresss from the Hwinfo Section if available */
    unsigned hwi_off = hwi_find_device(IMX_HWI_SAI, unit);

    if (hwi_off != HWI_NULL_OFF) {
        hwi_tag *tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
        if (tag) {
            imx->base = tag->location.base;
        }
        /*The first dma channel in hwi tag is for rx*/
        tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &zero);
        if (imx->num_rx_aif) {
            if (tag) {
                imx->rx_dma_chnl_type = tag->dma.chnl;
            } else {
                ado_error_fmt("DMA requests not found in hwinfo table");
                return -1;
            }
        }
        tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &zero);
        if (imx->num_tx_aif) {
            if (tag) {
                imx->tx_dma_chnl_type = tag->dma.chnl;
            } else {
                ado_error_fmt("DMA requests not found in hwinfo table");
                return -1;
            }
        }
        return 0;
    }
    return -1;
}

static bool is_supported_rate(uint32_t rate)
{
    uint32_t ratelist[] = { IMX_SAI_SAMPLE_RATES };
    int i;
#define NUM_RATES  (sizeof(ratelist) / sizeof(ratelist[0]))
    if (ado_pcm_rate2flag(rate) == 0) {
        return false;
    }
    for (i = 0; i < NUM_RATES; i++) {
        if (ado_pcm_rate2flag(rate) == ratelist[i]) {
            return true;
        }
    }
    return false;
}

/*
 * Parse command line parameters.
 */
static int imx_parse_commandline(HW_CONTEXT_T *imx, char *args)
{
    char *value;
    char *tx_voices = NULL;
    char *rx_voices = NULL;
    char * imx_opts[] = {
    #define OPT_SAIBASE         0
        "saibase",
    #define OPT_TCHNTYPE        1
        "tx_dma_chnl_type",
    #define OPT_RCHNTYPE        2
        "rx_dma_chnl_type",
    #define OPT_TX_RATE         3
        "tx_rate",
    #define OPT_RX_RATE         4
        "rx_rate",
    #define OPT_CLK_MODE        5
        "clk_mode",
    #define OPT_I2C_BUS         6
        "i2c_bus",
    #define OPT_I2C_ADDR        7
        "i2c_addr",
    #define OPT_SYS_CLK         8
        "sys_clk",
    #define OPT_TX_VOICES       9
        "tx_voices",
    #define OPT_RX_VOICES       10
        "rx_voices",
    #define OPT_TX_SAMPLE_SIZE  11
        "tx_sample_size",
    #define OPT_RX_SAMPLE_SIZE  12
        "rx_sample_size",
    #define OPT_TX_SLOT_SIZE    13
        "tx_slot_size",
    #define OPT_RX_SLOT_SIZE    14
        "rx_slot_size",
    #define OPT_TX_PROTOCOL     15
        "tx_protocol",
    #define OPT_RX_PROTOCOL     16
        "rx_protocol",
    #define OPT_TX_CLK_POL      17
        "tx_clk_pol",
    #define OPT_RX_CLK_POL      18
        "rx_clk_pol",
    #define OPT_TX_SYNC_POL     19
        "tx_sync_pol",
    #define OPT_RX_SYNC_POL     20
        "rx_sync_pol",
    #define OPT_TX_BIT_DELAY    21
        "tx_bit_delay",
    #define OPT_RX_BIT_DELAY    22
        "rx_bit_delay",
    #define OPT_TX_SYNC_SIZE    23
        "tx_sync_size",
    #define OPT_RX_SYNC_SIZE    24
        "rx_sync_size",
    #define OPT_RX_FRAG_SIZE    25
        "rx_frag_size",
    #define OPT_TX_FRAG_SIZE    26
        "tx_frag_size",
    #define OPT_TX_NSLOTS       27
        "tx_nslots",
    #define OPT_RX_NSLOTS       28
        "rx_nslots",
    #define OPT_TX_MSEL         29
        "tx_msel",
    #define OPT_RX_MSEL         30
        "rx_msel",
    #define OPT_SYNC_MODE       31
        "sync_mode",
        NULL
    };
    uint32_t optbase;
    imx_sai_xfer_config_t txconfig = {0}, rxconfig = {0};
    uint32_t tx_dma_chntype, rx_dma_chntype;
    uint32_t override_txdmachntype = 0, override_rxdmachntype = 0;

    txconfig.msel = rxconfig.msel = 0;
    txconfig.sample_rate_min = rxconfig.sample_rate_min = IMX_SAI_SAMPLE_RATE_MIN;
    txconfig.sample_rate_max = rxconfig.sample_rate_max = IMX_SAI_SAMPLE_RATE_MAX;
    txconfig.sample_size = rxconfig.sample_size = IMX_SAI_DEFAULT_SAMPLE_SIZE;
    txconfig.nslots = rxconfig.nslots = 2;
    /*
     * slot_size is for data alignment on the I2S/TDM lines, i.e., how the hardware shift the data
     * out relative to the frame sync and bit clock.
     */
    txconfig.conf_slot_size = rxconfig.conf_slot_size = 0;
#if defined(IMX_SAI_PROTOCOL_PCM)
    txconfig.protocol = rxconfig.protocol = IMX_SAI_PROTOCOL_PCM;
#else
    txconfig.protocol = rxconfig.protocol = IMX_SAI_PROTOCOL_I2S;
#endif
    /* Initialize to 0xFF so imx_sai_config_default_protocol_flags()
     * can determine if a command line override was applied
     */
    txconfig.clk_pol = rxconfig.clk_pol = 0xFF;
    txconfig.sync_pol = rxconfig.sync_pol = 0xFF;
    txconfig.bit_delay = rxconfig.bit_delay = 0xFF;

    optbase = 0;
#if IMX_EDMA
    tx_dma_chntype = IMX_DMA_REQ_SAI0_TX;
    rx_dma_chntype = IMX_DMA_REQ_SAI0_RX;
#endif
    imx->sys_clk = IMX_SAI_DEFAULT_CLK;
    imx->clk_mode = IMX_SAI_DEFAULT_CLK_MODE;
#if defined(I2C_SLAVE_ADDR)
    imx->i2c_addr = I2C_SLAVE_ADDR;
#else
    imx->i2c_addr = -1;
#endif
#if defined(I2C_BUS_NUMBER)
    imx->i2c_dev = I2C_BUS_NUMBER;
#else
    imx->i2c_dev = -1;
#endif
    imx->xfer_sync_mode = IMX_SAI_SYNC_RX_WITH_TX;
    imx->rx_frag_size = imx->tx_frag_size = 4 * 1024;

    while (*args != '\0') {
        switch (getsubopt(&args, imx_opts, &value)) {
            case OPT_SAIBASE:
                optbase = strtoul(value, NULL, 0);
                break;
            case OPT_TCHNTYPE:
                tx_dma_chntype = strtol(value, NULL, 0);
                override_txdmachntype = 1;
                break;
            case OPT_RCHNTYPE:
                rx_dma_chntype = strtol(value, NULL, 0);
                override_rxdmachntype = 1;
                break;
            case OPT_TX_RATE:
                {
                    char *value2;

                    txconfig.sample_rate_min = txconfig.sample_rate_max = strtoul(value, 0, 0);

                    if (is_supported_rate(txconfig.sample_rate_min) == false) {
                        ado_error_fmt("Invalid min tx sample rate - %d", txconfig.sample_rate_min);
                        goto cleanup_fail1;
                    }
                    if ((value2 = strchr(value, ':')) != NULL) {
                        txconfig.sample_rate_max = strtoul(value2 + 1, 0, 0);

                        if ((txconfig.sample_rate_max < txconfig.sample_rate_min) || (is_supported_rate(txconfig.sample_rate_max) == false)) {
                            ado_error_fmt("Invalid max tx sample rate - %d", txconfig.sample_rate_max);
                            goto cleanup_fail1;
                        }
                    }
                }
                break;
            case OPT_RX_RATE:
                {
                    char *value2;

                    rxconfig.sample_rate_min = rxconfig.sample_rate_max = strtoul(value, 0, 0);

                    if (is_supported_rate(rxconfig.sample_rate_min) == false) {
                        ado_error_fmt("Invalid min rx sample rate - %d", rxconfig.sample_rate_min);
                        goto cleanup_fail1;
                    }
                    if ((value2 = strchr(value, ':')) != NULL) {
                        rxconfig.sample_rate_max = strtoul(value2 + 1, 0, 0);

                        if ((rxconfig.sample_rate_max < rxconfig.sample_rate_min) || (is_supported_rate(rxconfig.sample_rate_max) == false)) {
                            ado_error_fmt("Invalid max rx sample rate - %d", rxconfig.sample_rate_max);
                            goto cleanup_fail1;
                        }
                    }
                }
                break;
            case OPT_CLK_MODE:
                if (value && *value != '\0') {
                    if (strcmp(value, "master") == 0) {
                        imx->clk_mode = IMX_SAI_MASTER;
                        ado_debug (DB_LVL_DRIVER, "Audio clock mode = Master");
                    } else if (strcmp(value, "slave") == 0) {
                        imx->clk_mode = IMX_SAI_SLAVE;
                        ado_debug (DB_LVL_DRIVER, "Audio clock mode = Slave");
                    }
                }
                break;
            case OPT_I2C_BUS:
                imx->i2c_dev = strtol(value, NULL, 0);
                break;
            case OPT_I2C_ADDR:
                imx->i2c_addr = strtol(value, NULL, 0);
                break;
            case OPT_SYS_CLK:
                imx->sys_clk = strtol(value, NULL, 0);
                break;
            case OPT_TX_VOICES:
                if (value != NULL)
                    tx_voices = strdup(value);
                break;
            case OPT_RX_VOICES:
                if (value != NULL)
                    rx_voices = strdup(value);
                break;
            case OPT_TX_SAMPLE_SIZE:
                txconfig.sample_size = strtoul(value, NULL, 0);
                switch (txconfig.sample_size) {
                    case 2:
                    case 16:
                        txconfig.sample_size = 2;
                        break;
                    case 3:
                    case 24:
                        txconfig.sample_size = 3;
                        break;
                    case 4:
                    case 32:
                        txconfig.sample_size = 4;
                        break;
                    default:
                        ado_error_fmt("Invalid tx sample size %d", txconfig.sample_size);
                        return EINVAL;
                }
                break;
            case OPT_RX_SAMPLE_SIZE:
                rxconfig.sample_size = strtoul(value, NULL, 0);

                switch (rxconfig.sample_size) {
                    case 2:
                    case 16:
                        rxconfig.sample_size = 2;
                        break;
                    case 3:
                    case 24:
                        rxconfig.sample_size = 3;
                        break;
                    case 4:
                    case 32:
                        rxconfig.sample_size = 4;
                        break;
                    default:
                        ado_error_fmt("Invalid rx sample size %d", rxconfig.sample_size);
                        return EINVAL;
                }
                break;
            case OPT_TX_SLOT_SIZE:
                txconfig.conf_slot_size = strtoul(value, NULL, 0);
                if (txconfig.conf_slot_size < 8 || txconfig.conf_slot_size > 32) {
                        ado_error_fmt("Invalid tx slot size %d", txconfig.conf_slot_size);
                        goto cleanup_fail1;
                }
                break;
            case OPT_RX_SLOT_SIZE:
                rxconfig.conf_slot_size = strtoul(value, NULL, 0);
                if (rxconfig.conf_slot_size < 8 || rxconfig.conf_slot_size > 32) {
                        ado_error_fmt("Invalid rx slot size %d", rxconfig.conf_slot_size);
                        goto cleanup_fail1;
                }
                break;
            case OPT_TX_PROTOCOL:
                if (value && *value != 0) {
                    if (strcmp(value, "i2s") == 0) {
                        txconfig.protocol = IMX_SAI_PROTOCOL_I2S;
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol = I2S");
                    } else if (strcmp(value, "pcm") == 0) {
                        txconfig.protocol = IMX_SAI_PROTOCOL_PCM;
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol = PCM");
                    } else {
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol not supported");
                        goto cleanup_fail1;
                    }
                }
                break;
            case OPT_RX_PROTOCOL:
                if (value && *value != 0) {
                    if (strcmp(value, "i2s") == 0) {
                        rxconfig.protocol = IMX_SAI_PROTOCOL_I2S;
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol = I2S");
                    } else if (strcmp(value, "pcm") == 0) {
                        rxconfig.protocol = IMX_SAI_PROTOCOL_PCM;
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol = PCM");
                    } else {
                        ado_debug(DB_LVL_DRIVER, "Audio Protocol not supported");
                        goto cleanup_fail1;
                    }
                }
                break;
            case OPT_TX_CLK_POL:
                txconfig.clk_pol = atoi(value);
                if (txconfig.clk_pol > 1) {
                    ado_error_fmt("Invalid xclk polarity mode");
                    goto cleanup_fail1;
                }
                break;
            case OPT_RX_CLK_POL:
                rxconfig.clk_pol = atoi(value);
                if (rxconfig.clk_pol > 1) {
                    ado_error_fmt("Invalid rclk polarity mode");
                    goto cleanup_fail1;
                }
                break;
            case OPT_TX_SYNC_POL:
                txconfig.sync_pol = atoi(value);
                if (txconfig.sync_pol > 1) {
                    ado_error_fmt("Invalid sync_pol value");
                    goto cleanup_fail1;
                }
                break;
            case OPT_RX_SYNC_POL:
                rxconfig.sync_pol = atoi(value);
                if (rxconfig.sync_pol > 1) {
                    ado_error_fmt("Invalid sync_pol value");
                    goto cleanup_fail1;
                }
                break;
            case OPT_TX_BIT_DELAY:
                txconfig.bit_delay = atoi(value);
                if (txconfig.bit_delay > 1) {
                    ado_error_fmt("Invalid bit_delay value (0 or 1)");
                    goto cleanup_fail1;
                }
                break;
            case OPT_RX_BIT_DELAY:
                rxconfig.bit_delay = atoi(value);
                if (rxconfig.bit_delay > 1) {
                    ado_error_fmt("Invalid bit_delay value (0 or 1)");
                    goto cleanup_fail1;
                }
                break;
            case OPT_TX_SYNC_SIZE:
                if (value && *value != 0) {
                    if (strcmp(value, "bit") == 0) {
                        txconfig.sync_len = 1;
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync lenght = bit");
                    } else if (strcmp(value, "word") == 0) {
                        txconfig.sync_len = txconfig.conf_slot_size;
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync length = word");
                    } else {
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync length not supported");
                        goto cleanup_fail1;
                    }
                }
                break;
            case OPT_RX_SYNC_SIZE:
                if (value && *value != 0) {
                    if (strcmp(value, "bit") == 0) {
                        rxconfig.sync_len = 1;
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync lenght = bit");
                    } else if (strcmp(value, "word") == 0) {
                        rxconfig.sync_len = rxconfig.conf_slot_size;
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync length = word");
                    } else {
                        ado_debug(DB_LVL_DRIVER, "Audio Frame sync length not supported");
                        goto cleanup_fail1;
                    }
                }
                break;
            case OPT_RX_FRAG_SIZE:
                if (value && *value != '\0')
                    imx->rx_frag_size = strtol (value, NULL, 0);
                break;
            case OPT_TX_FRAG_SIZE:
                if (value && *value != '\0')
                    imx->tx_frag_size = strtol (value, NULL, 0);
                break;
            case OPT_TX_NSLOTS:
                if (value && *value != 0) {
                    txconfig.nslots = atoi(value);
                }
                break;
            case OPT_RX_NSLOTS:
                if (value && *value != 0) {
                    rxconfig.nslots = atoi(value);
                }
                break;
            case OPT_TX_MSEL:
                if (value && *value != 0) {
                    txconfig.msel = atoi(value);
                    ado_debug(DB_LVL_DRIVER, "MSEL = %u", txconfig.msel);
                }
                break;
            case OPT_RX_MSEL:
                if (value && *value != 0) {
                    rxconfig.msel = atoi(value);
                    ado_debug(DB_LVL_DRIVER, "MSEL = %u", rxconfig.msel);
                }
                break;
            case OPT_SYNC_MODE:
                if (value && *value != 0) {
                    if (strcmp(value, "async") == 0) {
                        imx->xfer_sync_mode = IMX_SAI_ASYNC;
                        ado_debug(DB_LVL_DRIVER, "xfer_sync_mode = IMX_SAI_ASYNC");
                    } else if (strcmp(value, "sync") == 0) {
                        imx->xfer_sync_mode = IMX_SAI_SYNC_RX_WITH_TX;
                        ado_debug(DB_LVL_DRIVER, "xfer_sync_mode = IMX_SAI_SYNC_RX_WITH_TX");
                    }

                }
                break;
            default:
                break;
        }
    }
    {
        char *value2;
        uint32_t idx;
        vuint32_t rx_cnt, tx_cnt;
        int num_voices;

        /* Count number of TX interfaces */
        value = (tx_voices == NULL) ? AIF_TX_VOICES : tx_voices;
        for (idx = 0, tx_cnt = 1; idx < strlen(value); (value[idx] == ':') ? tx_cnt++ : 0, idx++);
        /* Count number of RX interfaces */
        value2 = (rx_voices == NULL) ? AIF_RX_VOICES : rx_voices;
        for (idx = 0, rx_cnt = 1; idx < strlen(value2); (value2[idx] == ':') ? rx_cnt++ : 0, idx++);
        /* Allocate Audio interfaces */
        imx->num_tx_aif = strtoul(value, NULL, 0) == 0 ? 0 : tx_cnt;
        imx->num_rx_aif = strtoul(value2, NULL, 0) == 0 ? 0 : rx_cnt;
        if ((imx->aif = (imx_aif_t *) ado_calloc (max(imx->num_tx_aif, imx->num_rx_aif), sizeof (imx_aif_t))) == NULL)
        {
            ado_error_fmt("Unable to allocate aif for mxsai (%s)", strerror (errno));
            goto cleanup_fail1;
        }

        value = (tx_voices == NULL) ? AIF_TX_VOICES : tx_voices;

        imx->aif[0].play_strm.voices = strtoul(value, 0, 0);
        if (imx->aif[0].play_strm.voices > 32)
        {
            ado_error_fmt("Invalid tx[%d] channels %d", 0, imx->aif[0].play_strm.voices);
            goto cleanup_fail;
        }

        idx = 1;    /* Skip first character since we handled it above */
        while (idx < tx_cnt && (value2 = strchr(value, ':')) != NULL)
        {
            imx->aif[idx++].play_strm.voices = strtoul(value2 + 1, 0, 0);
            if (imx->aif[idx - 1].play_strm.voices > 32)
            {
                ado_error_fmt("Invalid tx channels %d", idx - 1,  imx->aif[idx - 1].play_strm.voices);
                goto cleanup_fail;
            }
            value = value2+1;
        }

        num_voices = 0;
        for (idx = 0; idx < tx_cnt; idx++)
        {
            num_voices += imx->aif[idx].play_strm.voices;
        }

        if (txconfig.nslots > MAX_NSLOTS || txconfig.nslots < num_voices) {
            ado_error_fmt("wrong tx slots %d, maximum tx slots is %d, number of total voices is %d ", txconfig.nslots, MAX_NSLOTS, num_voices);
            goto cleanup_fail;
        }

        value = (rx_voices == NULL) ? AIF_RX_VOICES : rx_voices;

        imx->aif[0].cap_strm.voices = strtoul(value, 0, 0);
        if (imx->aif[0].cap_strm.voices > 32)
        {
            ado_debug(DB_LVL_DRIVER, "Invalid rx[%d] channels %d", 0, imx->aif[0].cap_strm.voices);
            goto cleanup_fail;
        }

        idx = 1;    /* Skip first character since we handled it above */
        while (idx < rx_cnt && (value2 = strchr(value, ':')) != NULL)
        {
            imx->aif[idx++].cap_strm.voices = strtoul(value2 + 1, 0, 0);
            if (imx->aif[idx - 1].cap_strm.voices > 32)
            {
                ado_debug(DB_LVL_DRIVER, "Invalid rx channels %d", idx - 1,  imx->aif[idx - 1].cap_strm.voices);
                goto cleanup_fail;
            }
            value = value2+1;
        }

        num_voices = 0;
        for (idx = 0; idx < rx_cnt; idx++)
        {
            num_voices += imx->aif[idx].cap_strm.voices;
        }

        if (rxconfig.nslots > MAX_NSLOTS || rxconfig.nslots < num_voices) {
            ado_error_fmt("wrong rx slots %d, maximum rx slots is %d, number of total voices is %d ", rxconfig.nslots, MAX_NSLOTS, num_voices);
            goto cleanup_fail;
        }

        /* Free strdup'd strings */
        if (tx_voices != NULL) {
            free(tx_voices);
            tx_voices = NULL;
        }
        if (rx_voices != NULL) {
            free(rx_voices);
            rx_voices = NULL;
        }
    }
#if IMX_EDMA
    if ((imx->num_tx_aif) && (tx_dma_chntype == IMX_DMA_REQ_NONE)) {
#else
    {
#endif
        ado_error_fmt("Number of TX aif is %d with no DMA Request", imx->num_tx_aif);
        goto cleanup_fail;
    }
#if IMX_EDMA
    if ((imx->num_rx_aif) && (rx_dma_chntype == IMX_DMA_REQ_NONE)) {
#else
    {
#endif
        ado_error_fmt("Number of RX aif is %d with no DMA Request", imx->num_rx_aif);
        goto cleanup_fail;
    }

    if (imx->num_tx_aif == 0 && imx->num_rx_aif == 0 ) {
        ado_error_fmt("no Tx and Rx interfaces");
        goto cleanup_fail;
    }

    if ((imx->num_tx_aif == 0 && imx->num_rx_aif != 0) ||
        (imx->num_rx_aif == 0 && imx->num_tx_aif != 0)) {
        imx->xfer_sync_mode = IMX_SAI_ASYNC;
    }

    if (optbase <= 0xFF) {
        // query syspage
        if (query_hwi_device(imx, optbase) == -1) {
            goto cleanup_fail;
        }
    } else {
        imx->base = optbase;
    }

    if (imx->num_tx_aif) {
        if (override_txdmachntype) {
            imx->tx_dma_chnl_type = tx_dma_chntype;
        }
        if (txconfig.conf_slot_size == 0) {
            // slot size is not set in command line, default to sample size
            txconfig.conf_slot_size = txconfig.sample_size * _BITS_BYTE;
        } else if (txconfig.conf_slot_size < txconfig.sample_size * _BITS_BYTE) {
            ado_error_fmt("tx config slot size is smaller than sample size");
            goto cleanup_fail;
        }
        /* If sync_len is not set default to the slot_size (word) */
        if (txconfig.sync_len == 0) {
            txconfig.sync_len = txconfig.conf_slot_size;
        }
        imx_sai_config_default_protocol_flags(&txconfig, IMX_SAI_TX);
        memcpy(&imx->tx_cfg, &txconfig, sizeof(imx_sai_xfer_config_t));
    }

    if (imx->num_rx_aif) {
        if (override_rxdmachntype) {
            imx->rx_dma_chnl_type = rx_dma_chntype;
        }
        if (rxconfig.conf_slot_size == 0) {
            // slot size is not set in command line, default to sample size
            rxconfig.conf_slot_size = rxconfig.sample_size * _BITS_BYTE;
        } else if (rxconfig.conf_slot_size < rxconfig.sample_size * _BITS_BYTE) {
            ado_error_fmt("rx config slot size is smaller than sample size");
            goto cleanup_fail;
        }
        /* If sync_len is not set default to the slot_size (word) */
        if (rxconfig.sync_len == 0) {
            rxconfig.sync_len = rxconfig.conf_slot_size;
        }
        imx_sai_config_default_protocol_flags(&rxconfig, IMX_SAI_RX);
        memcpy(&imx->rx_cfg, &rxconfig, sizeof(imx_sai_xfer_config_t));
    }

    ado_debug(DB_LVL_DRIVER, "num_tx_aif %d, num_rx_aif %d, tx_cfg->nslots %d, rx_cfg->nslots %d", imx->num_tx_aif, imx->num_rx_aif,
              imx->tx_cfg.nslots, imx->rx_cfg.nslots);
    return EOK;
cleanup_fail:
    if (imx->aif != NULL)
        ado_free(imx->aif);
cleanup_fail1:
    if (tx_voices != NULL)
        free(tx_voices);
    if (rx_voices != NULL)
        free(rx_voices);
    return EINVAL;
}

ado_ctrl_dll_init_t ctrl_init;
/**
 * Entry point of IMX SAI audio driver library.
 *
 * Called when io-audio loads IMX SAI HW DLL.
 *
 * @param hw_context Pointer which store IMX SAI hardware context structure.
 * @param card       Pointer to an internal card structure.
 * @param args       Any command-line arguments.
 *
 * @return Execution status.
 */
int ctrl_init(HW_CONTEXT_T **hw_context, ado_card_t *card, char *args)
{
    uint32_t i, cnt;
    imx_t *imx;
    uint32_t rate;
#if IMX_EDMA
    char str[100] = { 0 };
#endif
    dma_driver_info_t dma_info;
    uint32_t ratelist[] = { IMX_SAI_SAMPLE_RATES };
    char pcm_name[_POSIX_NAME_MAX];
    char pcm_name2[_POSIX_NAME_MAX];
#define NUM_RATES  (sizeof(ratelist) / sizeof(ratelist[0]))

    ado_debug(DB_LVL_DRIVER, "CTRL_DLL_INIT: MX SAI");

#ifndef IMX_EDMA
    /* There are other i.MX8 variants that suport SDMA, but we
     * do not support those platforms.  If your are expecting a
     * DMA other than EDMA we should exit */
    ado_error_fmt("EDMA is the only supported DMA", strerror(errno));
    return -1;
#endif

    if ((imx = (imx_t *) ado_calloc(1, sizeof(imx_t))) == NULL) {
        ado_error_fmt("Unable to allocate memory for imx (%s)", strerror(errno));
        return -1;
    }

    /* Initialize pointer to HW context */
    *hw_context = imx;
    imx->card = card;

    if (imx_parse_commandline(imx, args) != EOK) {
        ado_free(imx);
        return -1;
    }
    /* Initialize file descriptor */
    ado_card_set_shortname(card, "MX_SAI");
    ado_card_set_longname(card, "NXP i.MX SAI", imx->base);

    ado_debug(DB_LVL_DRIVER, "Using SAI Base 0x%x", imx->base);

    /* Get DMA functions */
    if (get_dmafuncs(&imx->dmafuncs, sizeof(dma_functions_t)) == -1) {
        ado_error_fmt("Failed to get DMA lib functions");
        if (imx->aif) {
            ado_free(imx->aif);
        }
        ado_free(imx);
        return -1;
    }

    if (imx->dmafuncs.init(NULL) == -1) {
        ado_error_fmt("DMA init failed");
        if (imx->aif) {
            ado_free(imx->aif);
        }
        ado_free(imx);
        return -1;
    }
    imx->dmafuncs.driver_info(&dma_info);

    ado_mutex_init(&imx->lock);

    if (imx->num_tx_aif) {
        /* This pulse occurs on every segment complete */
        my_attach_pulse(&imx->aif[0].play_strm.dma.pulse, &imx->aif[0].play_strm.dma.event, imx_play_pulse_hdlr, imx);
#if IMX_EDMA
        /* Attach DMA channel for Tx */
        strcpy(str, "tcd_reload=1");
        imx->aif[0].play_strm.dma.chn = imx->dmafuncs.channel_attach_smmu(str, &imx->aif[0].play_strm.dma.event,
                                                                      &imx->tx_dma_chnl_type, dma_info.max_priority,
                                                                      DMA_ATTACH_EVENT_PER_SEGMENT | DMA_ATTACH_PROCESS, ado_card_smmu(card));
#endif
        if (imx->aif[0].play_strm.dma.chn == NULL) {
            ado_error_fmt("EDMA playback channel attach failed");
            my_detach_pulse(&imx->aif[0].play_strm.dma.pulse);
            ctrl_destroy(imx);
            return -1;
        }
    }
    if (imx->num_rx_aif) {
        /* This pulse occurs on every segment complete */
        my_attach_pulse(&imx->aif[0].cap_strm.dma.pulse, &imx->aif[0].cap_strm.dma.event, imx_cap_pulse_hdlr, imx);
#if IMX_EDMA
        /* Attach DMA channel for Rx */
        strcpy(str, "tcd_reload=1");
        imx->aif[0].cap_strm.dma.chn = imx->dmafuncs.channel_attach_smmu(str, &imx->aif[0].cap_strm.dma.event,
                                                                         &imx->rx_dma_chnl_type, dma_info.max_priority,
                                                                         DMA_ATTACH_EVENT_PER_SEGMENT | DMA_ATTACH_PROCESS, ado_card_smmu(card));
#endif
        if (imx->aif[0].cap_strm.dma.chn == NULL) {
            ado_error_fmt("EDMA capture channel attach failed");
            my_detach_pulse(&imx->aif[0].cap_strm.dma.pulse);
            ctrl_destroy(imx);
            return -1;
        }
    }

    /* MMAP peripheral registers
     * Note: We map the registers after the DMA attachments for SMMU support
     */
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
    imx->id_reg = ado_device_mmap(imx->base, sizeof(*imx->id_reg));
    if (!imx->id_reg) {
        ctrl_destroy(imx);
        return -1;
    }
#endif

    if (imx->num_rx_aif) {
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
        imx->rx_reg = ado_device_mmap(imx->base + IMX_SAI_RXREGS_OFFSET, sizeof(imx_sai_rxtx_t));
#else
        imx->rx_reg = ado_device_mmap(imx->base + sizeof(imx_sai_rxtx_t), sizeof(imx_sai_rxtx_t));
#endif
        if (imx->rx_reg == MAP_FAILED) {
            ado_error_fmt("Unable to mmap SAI RX (%s)", strerror(errno));
            ctrl_destroy(imx);
            return -1;
        }
    }
    if (imx->num_tx_aif) {
#if (IMX_SAI_VERSION != IMX_SAI_VERSION_QM_QXP)
        imx->tx_reg = ado_device_mmap(imx->base + IMX_SAI_TXREGS_OFFSET, sizeof(imx_sai_rxtx_t));
#else
        imx->tx_reg = ado_device_mmap(imx->base, sizeof(imx_sai_rxtx_t));
#endif
        if (imx->tx_reg == MAP_FAILED) {
            ado_error_fmt("Unable to mmap SAI TX (%s)", strerror(errno));
            ctrl_destroy(imx);
            return -1;
        }
    }

    if (imx_sai_init(imx) == -1) {
        ado_error_fmt("SAI register initialization failed");
        ctrl_destroy(imx);
        return -1;
    }

    /* If multiple rx audio interfaces then allocate a ping-pong DMA buffer for capture.
     * We will DMA into this buffer and copy the TDM slot data out of this buffer into the
     * appropriate client buffers that are allocated in the aquire function.
     */
    if(imx->num_rx_aif > 1)
    {
        dma_transfer_t tinfo;
        int frag_idx = 0;
        int dma_xfer_size = imx->rx_frag_size;
        ado_pcm_config_t config;
#if IMX_EDMA
        dma_addr_t addr;
#endif
        /* Align the ping and pong buffers to the audio frame size */
        int alignment = imx->rx_cfg.sample_size * imx->rx_cfg.nslots;
        if (global_options.sw_mixer_ms > 0)
        {
            dma_xfer_size = global_options.sw_mixer_ms * alignment * imx->rx_cfg.sample_rate_max / 1000;
        }
        else if (global_options.sw_mixer_samples > 0)
        {
            dma_xfer_size = global_options.sw_mixer_samples * alignment;
        }
        else
        {
            dma_xfer_size = imx->rx_frag_size / alignment * alignment;
        }
        /* Two fragments for a ping-pong buffer */
        config.mode.block.frags_total = 2;

        config.mode.block.frag_size = dma_xfer_size;
        config.format.voices = imx->rx_cfg.nslots;
        switch (imx->rx_cfg.sample_size) {
            case 2:
                config.format.format = SND_PCM_SFMT_S16_LE;
                break;
            case 3:
                config.format.format = SND_PCM_SFMT_S24_4_LE;
                break;
            case 4:
                config.format.format = SND_PCM_SFMT_S32_LE;
                break;
            default:
                ado_error_fmt("Invalid rx sample size for format assignment %d", imx->rx_cfg.sample_size);
                ctrl_destroy(imx);
                return -1;
        }
        imx->rx_fifo_watermark = calculate_fifo_watermark(&config, &imx->rx_cfg);
        /* Update dmabuf.size based on possible changes done by calculate_fifo_watermark() */
        imx->capture_dmabuf.size = config.mode.block.frag_size * config.mode.block.frags_total;

        imx->capture_dmabuf.phys_addr = 0; /* Ensure phys address is 0 so we map anonymous memory */
        imx->capture_dmabuf.flags = ADO_BUF_CACHE;
        if ((imx->capture_dmabuf.addr = ado_mmap_phys (imx->card, imx->capture_dmabuf.size,
                                                       imx->capture_dmabuf.flags, &imx->capture_dmabuf.phys_addr)) == MAP_FAILED)
        {
            ado_error_fmt("MX SAI: Unable to allocate DMA buffer for capture pseudo devices");
            ctrl_destroy(imx);
            return -1;
        }
        /* Setup the DMA transfer */
        memset(&tinfo, 0, sizeof(tinfo));

        /* Allocate descriptor List */
        tinfo.dst_addrs = ado_calloc( config.mode.block.frags_total, sizeof(dma_addr_t));
        if(NULL == tinfo.dst_addrs)
        {
            ado_error_fmt("MX SAI: insufficient memory");
            ctrl_destroy(imx);
            return -1;
        }
#if IMX_EDMA
        tinfo.src_addrs = &addr;
        tinfo.src_fragments = 1;
        /* Initialize src_addrs to physical address of RX register or ASRC IN register */
        tinfo.src_addrs->paddr = imx->base + IMX_SAI_RDR_OFFSET;
        tinfo.src_addrs->len = imx->rx_cfg.sample_size;//4U;//imx->rx_cfg.sample_size;
        /* Register address cannot be incremented */
        tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT;
#endif
        tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
#if IMX_EDMA
        tinfo.xfer_bytes = imx->rx_fifo_watermark;
#endif
        for (frag_idx = 0; frag_idx < config.mode.block.frags_total; frag_idx++)
        {
            tinfo.dst_addrs[frag_idx].paddr = imx->capture_dmabuf.phys_addr + (frag_idx * config.mode.block.frag_size);
            tinfo.dst_addrs[frag_idx].len = config.mode.block.frag_size;
        }
        tinfo.dst_fragments = config.mode.block.frags_total;
        tinfo.xfer_unit_size = (imx->rx_cfg.sample_size == 2) ? 16 : 32;
        imx->dmafuncs.setup_xfer(imx->aif[0].cap_strm.dma.chn, &tinfo);
        ado_free(tinfo.dst_addrs);
    }

    /* If multiple tx audio interfaces then allocate a ping-pong DMA buffer for playback.
     * We will DMA from this buffer to the hardware and copy the TDM slot data to the
     * appropriate client buffers that are allocated in the aquire function.
     */
    if(imx->num_tx_aif > 1)
    {
        dma_transfer_t tinfo;
        int frag_idx = 0;
        int dma_xfer_size;
        ado_pcm_config_t config;
#if IMX_EDMA
        dma_addr_t addr;
#endif
        /* Align the ping and pong buffers to the audio frame size */
        int alignment = imx->tx_cfg.sample_size * imx->tx_cfg.nslots;
        if (global_options.sw_mixer_ms > 0)
        {
            dma_xfer_size = global_options.sw_mixer_ms * alignment * imx->tx_cfg.sample_rate_max / 1000;
        }
        else if (global_options.sw_mixer_samples > 0)
        {
            dma_xfer_size = global_options.sw_mixer_samples * alignment;
        }
        else
        {
            dma_xfer_size = imx->tx_frag_size / alignment * alignment;
        }
        /* Two fragments for a ping-pong buffer */
        config.mode.block.frags_total = 2;

        config.mode.block.frag_size = dma_xfer_size;
        config.format.voices = imx->tx_cfg.nslots;
        switch (imx->tx_cfg.sample_size) {
            case 2:
                config.format.format = SND_PCM_SFMT_S16_LE;
                break;
            case 3:
                config.format.format = SND_PCM_SFMT_S24_4_LE;
                break;
            case 4:
                config.format.format = SND_PCM_SFMT_S32_LE;
                break;
            default:
                ado_error_fmt("Invalid tx sample size for format assignment %d", imx->tx_cfg.sample_size);
                ctrl_destroy(imx);
                return -1;
        }
        imx->tx_fifo_watermark = calculate_fifo_watermark(&config, &imx->tx_cfg);
        /* Update dmabuf.size based on possible changes done by calculate_fifo_watermark() */
        imx->playback_dmabuf.size = config.mode.block.frag_size * config.mode.block.frags_total;

        imx->playback_dmabuf.phys_addr = 0; /* Ensure phys address is 0 so we map anonymous memory */
        imx->playback_dmabuf.flags = ADO_BUF_CACHE;
        if ((imx->playback_dmabuf.addr = ado_mmap_phys (imx->card, imx->playback_dmabuf.size,
                                                       imx->playback_dmabuf.flags, &imx->playback_dmabuf.phys_addr)) == MAP_FAILED)
        {
            ado_error_fmt("MX SAI: Unable to allocate DMA buffer for playback pseudo devices");
            ctrl_destroy(imx);
            return -1;
        }
        /* Setup the DMA transfer */
        memset(&tinfo, 0, sizeof(tinfo));

        /* Allocate descriptor List */
        tinfo.src_addrs = ado_calloc( config.mode.block.frags_total, sizeof(dma_addr_t));
        if(NULL == tinfo.src_addrs)
        {
            ado_error_fmt("MX SAI: insufficient memory");
            ctrl_destroy(imx);
            return -1;
        }
        for (frag_idx = 0; frag_idx < config.mode.block.frags_total; frag_idx++)
        {
            tinfo.src_addrs[frag_idx].paddr = imx->playback_dmabuf.phys_addr + (frag_idx * config.mode.block.frag_size);
            tinfo.src_addrs[frag_idx].len = config.mode.block.frag_size;
        }
#if IMX_EDMA
        /* eDMA driver specific initialization */
        /* Allocate dst_addr for TX reg */
        tinfo.dst_addrs = &addr;
        tinfo.dst_fragments = 1;
        /* Initialize dst_addr to physical address of TX register or ASRC IN register */
        tinfo.dst_addrs->paddr = imx->base + IMX_SAI_TDR_OFFSET;
        tinfo.dst_addrs->len = imx->tx_cfg.sample_size;
        /* Register address cannot be incremented */
        tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT;
#endif
        /* Continuously repeat this DMA transfer */
        tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
#if IMX_EDMA
        tinfo.xfer_bytes = imx->tx_fifo_watermark;
#endif
        tinfo.src_fragments = config.mode.block.frags_total;
        tinfo.xfer_unit_size = (imx->tx_cfg.sample_size == 2) ? 16 : 32;
        imx->dmafuncs.setup_xfer(imx->aif[0].play_strm.dma.chn, &tinfo);
        ado_free(tinfo.src_addrs);
    }

    if (imx->num_tx_aif) {
        imx->aif[0].play_strm.pcm_caps.chn_flags = SND_PCM_CHNINFO_BLOCK | SND_PCM_CHNINFO_STREAM
                                                | SND_PCM_CHNINFO_INTERLEAVE | SND_PCM_CHNINFO_BLOCK_TRANSFER | SND_PCM_CHNINFO_MMAP
                                                | SND_PCM_CHNINFO_MMAP_VALID;
        switch (imx->tx_cfg.sample_size) {
            case 2:
                imx->aif[0].play_strm.pcm_caps.formats = SND_PCM_FMT_S16_LE;
                break;
            case 3:
                imx->aif[0].play_strm.pcm_caps.formats = SND_PCM_FMT_S24_4_LE;
                break;
            case 4:
                imx->aif[0].play_strm.pcm_caps.formats = SND_PCM_FMT_S32_LE;
                break;
            default:
                ado_error_fmt("Invalid tx sample size for PCM capability format assignment %d", imx->tx_cfg.sample_size);
                ctrl_destroy(imx);
                return -1;
        }

        for (i = 0; i < NUM_RATES; i++) {
            rate = ado_pcm_flag2rate(ratelist[i]);
            if (rate >= imx->tx_cfg.sample_rate_min && rate <= imx->tx_cfg.sample_rate_max) {
                imx->aif[0].play_strm.pcm_caps.rates |= ratelist[i];
            }
        }

        imx->aif[0].play_strm.pcm_caps.min_rate = imx->tx_cfg.sample_rate_min;
        imx->aif[0].play_strm.pcm_caps.max_rate = imx->tx_cfg.sample_rate_max;
        imx->aif[0].play_strm.pcm_caps.min_voices = imx->aif[0].play_strm.voices;
        imx->aif[0].play_strm.pcm_caps.max_voices = imx->aif[0].play_strm.voices;
        imx->aif[0].play_strm.pcm_caps.min_fragsize = 64;
#ifdef IMX_SAI_MAX_FRAGSIZE
        imx->aif[0].play_strm.pcm_caps.max_fragsize = IMX_SAI_MAX_FRAGSIZE;
#else
        imx->aif[0].play_strm.pcm_caps.max_fragsize = 64 * 1024;
#endif
#if IMX_EDMA
        /* Limit the number of fragments to the maximum allowed DMA descriptors */
        imx->aif[0].play_strm.pcm_caps.max_frags = MAX_DESCRIPTORS;
#endif
        imx->aif[0].play_strm.pcm_funcs.capabilities = imx_capabilities;
        imx->aif[0].play_strm.pcm_funcs.aquire = imx_playback_aquire;
        imx->aif[0].play_strm.pcm_funcs.release = imx_playback_release;
        imx->aif[0].play_strm.pcm_funcs.prepare = imx_prepare;
        imx->aif[0].play_strm.pcm_funcs.trigger = imx_playback_trigger;
        for (cnt = 1; cnt < imx->num_tx_aif; cnt++)
        {
            memcpy (&imx->aif[cnt].play_strm.pcm_caps, &imx->aif[0].play_strm.pcm_caps,
                sizeof (imx->aif[cnt].play_strm.pcm_caps));
            imx->aif[cnt].play_strm.pcm_caps.min_voices = imx->aif[cnt].play_strm.voices;
            imx->aif[cnt].play_strm.pcm_caps.max_voices = imx->aif[cnt].play_strm.voices;
        }
    }
    if (imx->num_rx_aif) {

        imx->aif[0].cap_strm.pcm_caps.chn_flags = SND_PCM_CHNINFO_BLOCK | SND_PCM_CHNINFO_STREAM
                                               | SND_PCM_CHNINFO_INTERLEAVE | SND_PCM_CHNINFO_BLOCK_TRANSFER | SND_PCM_CHNINFO_MMAP
                                               | SND_PCM_CHNINFO_MMAP_VALID;
        switch (imx->rx_cfg.sample_size) {
            case 2:
                imx->aif[0].cap_strm.pcm_caps.formats = SND_PCM_FMT_S16_LE;
                break;
            case 3:
                imx->aif[0].cap_strm.pcm_caps.formats = SND_PCM_FMT_S24_4_LE;
                break;
            case 4:
                imx->aif[0].cap_strm.pcm_caps.formats = SND_PCM_FMT_S32_LE;
                break;
            default:
                ado_error_fmt("Invalid rx sample size for PCM capability format assignment %d", imx->rx_cfg.sample_size);
                ctrl_destroy(imx);
                return -1;
        }

        for (i = 0; i < NUM_RATES; i++) {
            rate = ado_pcm_flag2rate(ratelist[i]);
            if (rate >= imx->rx_cfg.sample_rate_min && rate <= imx->rx_cfg.sample_rate_max) {
                imx->aif[0].cap_strm.pcm_caps.rates |= ratelist[i];
            }
        }

        imx->aif[0].cap_strm.pcm_caps.min_rate = imx->rx_cfg.sample_rate_min;
        imx->aif[0].cap_strm.pcm_caps.max_rate = imx->rx_cfg.sample_rate_max;
        imx->aif[0].cap_strm.pcm_caps.min_voices = imx->aif[0].cap_strm.voices;
        imx->aif[0].cap_strm.pcm_caps.max_voices = imx->aif[0].cap_strm.voices;
        imx->aif[0].cap_strm.pcm_caps.min_fragsize = 64;
#ifdef IMX_SAI_MAX_FRAGSIZE
        imx->aif[0].cap_strm.pcm_caps.max_fragsize = IMX_SAI_MAX_FRAGSIZE;
#else
        imx->aif[0].cap_strm.pcm_caps.max_fragsize = 64 * 1024;
#endif
#if IMX_EDMA
        /* Limit the number of fragments to the maximum allowed DMA descriptors */
        imx->aif[0].cap_strm.pcm_caps.max_frags = MAX_DESCRIPTORS;
#endif
        imx->aif[0].cap_strm.pcm_funcs.capabilities = imx_capabilities;
        imx->aif[0].cap_strm.pcm_funcs.aquire = imx_capture_aquire;
        imx->aif[0].cap_strm.pcm_funcs.release = imx_capture_release;
        imx->aif[0].cap_strm.pcm_funcs.prepare = imx_prepare;
        imx->aif[0].cap_strm.pcm_funcs.trigger = imx_capture_trigger;
        for (cnt = 1; cnt < imx->num_rx_aif; cnt++)
        {
            memcpy (&imx->aif[cnt].cap_strm.pcm_caps, &imx->aif[0].cap_strm.pcm_caps,
                sizeof (imx->aif[cnt].cap_strm.pcm_caps));
            imx->aif[cnt].cap_strm.pcm_caps.min_voices = imx->aif[cnt].cap_strm.voices;
            imx->aif[cnt].cap_strm.pcm_caps.max_voices = imx->aif[cnt].cap_strm.voices;
        }
    }
    for (cnt = 0; cnt < max(imx->num_tx_aif, imx->num_rx_aif); cnt++) {
        snprintf(pcm_name, _POSIX_NAME_MAX, "mxsai PCM %d", cnt);
        snprintf(pcm_name2, _POSIX_NAME_MAX, "mxsai-%d", cnt);
        /* Only create one mixer device for the card */
        if (cnt == 0) {
            if (codec_mixer(card, imx, imx->aif[cnt].pcm)) {
                ado_error_fmt("MX SAI: Unable to create codec mixer");
                ctrl_destroy(imx);
                return -1;
            }
            /* Initialize codec to be muted, will be un-muted from the trigger callback */
            codec_off(imx, ADO_PCM_CHANNEL_PLAYBACK);
            codec_off(imx, ADO_PCM_CHANNEL_CAPTURE);
        }

        if (ado_pcm_create(card, pcm_name, 0, pcm_name2,
                           cnt < imx->num_tx_aif ? 1 : 0,
                           cnt < imx->num_tx_aif ? &imx->aif[cnt].play_strm.pcm_caps : NULL,
                           cnt < imx->num_tx_aif ? &imx->aif[0].play_strm.pcm_funcs : NULL,
                           cnt < imx->num_rx_aif ? 1 : 0,
                           cnt < imx->num_rx_aif ? &imx->aif[cnt].cap_strm.pcm_caps : NULL,
                           cnt < imx->num_rx_aif ? &imx->aif[0].cap_strm.pcm_funcs : NULL,
                           imx->mixer, &imx->aif[cnt].pcm)) {
            ado_error_fmt("MXSAI: Unable to create pcm devices (%s)", strerror(errno));
            ctrl_destroy(imx);
            return -1;
        }
        ado_debug(DB_LVL_DRIVER, "MX SAI: PCM%d -> TX voices = %d, RX voices = %d, max_frag = %d", cnt,
                  imx->aif[cnt].play_strm.voices, imx->aif[cnt].cap_strm.voices,  imx->aif[cnt].play_strm.pcm_caps.max_fragsize);
        if (cnt < imx->num_tx_aif) {
            codec_set_default_group(imx, imx->aif[cnt].pcm, ADO_PCM_CHANNEL_PLAYBACK, cnt);
        }
        if (cnt < imx->num_rx_aif) {
            codec_set_default_group(imx, imx->aif[cnt].pcm, ADO_PCM_CHANNEL_CAPTURE, cnt);
        }
    }
    return 0;
}

ado_ctrl_dll_destroy_t ctrl_destroy;
/**
 * Entry point called when card is unmounted.
 *
 * @param imx IMX SAI hardware context structure.
 *
 * @return Execution status.
 */
int ctrl_destroy(HW_CONTEXT_T *imx)
{
    ado_debug(DB_LVL_DRIVER, "CTRL_DLL_DESTROY: MX");

    if (imx->playback_dmabuf.addr != NULL) {
        /* Free global playback transfer buffer */
        ado_munmap_phys (imx->card, imx->playback_dmabuf.addr, imx->playback_dmabuf.size);
    }
    if (imx->capture_dmabuf.addr != NULL) {
        /* Free global capture transfer buffer */
        ado_munmap_phys (imx->card, imx->capture_dmabuf.addr, imx->capture_dmabuf.size);
    }

    if (imx->num_rx_aif) {
        if (imx->aif[0].cap_strm.dma.chn ) {
            imx->dmafuncs.channel_release(imx->aif[0].cap_strm.dma.chn);
            my_detach_pulse(&imx->aif[0].cap_strm.dma.pulse);
        }
    }
    if (imx->num_tx_aif) {
        if (imx->aif[0].play_strm.dma.chn ) {
            imx->dmafuncs.channel_release(imx->aif[0].play_strm.dma.chn);
            my_detach_pulse(&imx->aif[0].play_strm.dma.pulse);
        }
    }

    imx->dmafuncs.fini();
    ado_mutex_destroy(&imx->lock);
    if (imx->rx_reg) {
        ado_device_munmap(imx->rx_reg, sizeof(imx_sai_rxtx_t));
    }
    if (imx->tx_reg) {
        ado_device_munmap(imx->tx_reg, sizeof(imx_sai_rxtx_t));
    }
    if (imx->aif)
        ado_free(imx->aif);
    ado_free(imx);
    return 0;
}



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxsai/mxsai_dll.c $ $Rev: 906157 $")
#endif
