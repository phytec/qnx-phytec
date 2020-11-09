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


#include "mxesai.h"

static bool is_active ( HW_CONTEXT_T *hwc, int channel)
{
    /* Active state is currently only tracked for capture */
    if (channel == ADO_PCM_CHANNEL_CAPTURE)
        return (hwc->aif->cap_strm.active ? true : false);

    return (false);
}

static const snd_pcm_chmap_t *
get_chmap( HW_CONTEXT_T *hwc, int channel, int index)
{
    if (channel == ADO_PCM_CHANNEL_PLAYBACK)
    {
        if ((uint32_t)index < hwc->num_tx_aif)
            return (hwc->aif[index].play_strm.pcm_caps.chmap);
        else
            return (NULL);
    }
    else
    {
        if ((uint32_t)index < hwc->num_rx_aif)
            return (hwc->aif[index].cap_strm.pcm_caps.chmap);
        else
            return (NULL);
    }
}

void codec_on(HW_CONTEXT_T *hwc, int channel)
{
    if(hwc->callbacks.codec_on)
        hwc->callbacks.codec_on(hwc->mixer, channel);
    return;
}
void codec_off(HW_CONTEXT_T *hwc, int channel)
{
    if(hwc->callbacks.codec_off)
        hwc->callbacks.codec_off(hwc->mixer, channel);
    return;
}

void
codec_set_default_group( HW_CONTEXT_T *hwc, ado_pcm_t *pcm, int channel, int index )
{
    if (hwc->callbacks.codec_set_default_group) {
        hwc->callbacks.codec_set_default_group(pcm, hwc->mixer, channel, index);
    }
}

int codec_mixer(ado_card_t * card, HW_CONTEXT_T * imx, ado_pcm_t *pcm)
{
    int status = EOK;
    int idx = 0;
    ado_mixer_dll_codec_params_t params = {0};

    params.hw_context = imx;
    params.is_active = is_active;
    params.get_chmap = get_chmap;

    /* Proivde the total number of tx_voices across all interface */
    for(idx = 0; idx < imx->num_tx_aif; idx++)
    {
        params.tx_voices += imx->aif[idx].play_strm.voices;
    }
    /* Proivde the total number of rx_voices across all interface */
    for(idx = 0; idx < imx->num_rx_aif; idx++)
    {
        params.rx_voices += imx->aif[idx].cap_strm.voices;
    }

    params.i2c_dev = imx->i2c_bus;
    params.i2c_addr = imx->i2c_addr;
    params.clk_master = (imx->clk_mode == ESAI_CLK_SLAVE) ? true : false;
    params.tx_sample_size = params.rx_sample_size = imx->sample_size * _BITS_BYTE;

    /* If NULL is passed in as the DLL name, this will cause the ado_mixer_dll() call
     * to look in the audio configuration file for the dll name (mixer_dll=xxxx)
     */
    if (ado_mixer_dll (card, NULL, I2S_CODEC_MINOR_VERSION, &params, &imx->callbacks, &imx->mixer) != EOK)
    {
        status = errno;
        if (status == ENOENT)
        {
            /* No mixer_dll specified, run in a no-codec configuration */
            if ((status = ado_mixer_create (card, NULL, &imx->mixer, NULL)) != EOK)
            {
                status = errno;
                ado_error_fmt("Unable to create mixer device - %s", strerror(status));
            }
        }
    }
    else if (ado_mixer_get_dll_handle(imx->mixer))
    {
        if (imx->callbacks.codec_set_default_group == NULL)
        {
            ado_error_fmt("Mixer DLL missing codec_set_defalut_group callback");
            status = ENOTSUP;
         }
    }

    return (status);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxesai/nto/aarch64/dll.le.mx8/mixer.c $ $Rev: 903313 $")
#endif
