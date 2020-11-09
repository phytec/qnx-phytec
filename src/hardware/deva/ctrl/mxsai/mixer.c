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

#include "mxsai.h"

static bool is_active ( HW_CONTEXT_T *hwc, int channel)
{
    uint32_t i;
    /* Active state is currently only tracked for capture */
    if (channel == ADO_PCM_CHANNEL_CAPTURE && hwc->num_rx_aif)
        for (i = 0; i < hwc->num_rx_aif; i++) {
            if (hwc->aif[i].cap_strm.active)
                return true;
        }

    return (false);
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

void codec_set_rate(HW_CONTEXT_T *hwc, uint32_t rate, int channel)
{
    if(hwc->callbacks.codec_set_rate)
        hwc->callbacks.codec_set_rate(hwc->mixer, rate, channel);
    return;
}

void
codec_set_default_group( HW_CONTEXT_T *hwc, ado_pcm_t *pcm, int channel, int index )
{
    if (hwc->callbacks.codec_set_default_group)
        hwc->callbacks.codec_set_default_group(pcm, hwc->mixer, channel, index);
}

int codec_mixer(ado_card_t * card, HW_CONTEXT_T * imx, ado_pcm_t *pcm)
{
    int status = EOK;
    uint32_t idx = 0;
    ado_mixer_dll_codec_params_t params = {0};
    /* Currently not used */
    (void)(pcm);

    params.hw_context = imx;
    params.is_active = is_active;

    if (imx->num_tx_aif) {
        int num_voices = 0;
        for (idx = 0; idx < imx->num_tx_aif; idx++)
        {
            num_voices += imx->aif[idx].play_strm.voices;
        }
        params.tx_voices = num_voices;
        params.tx_sample_size = imx->tx_cfg.sample_size == 2 ? 16 : 32;
        params.tx_sample_rate = imx->tx_cfg.sample_rate;
    }

    if (imx->num_rx_aif) {
        int num_voices = 0;
        for (idx = 0; idx < imx->num_rx_aif; idx++)
        {
            num_voices += imx->aif[idx].cap_strm.voices;
        }
        params.rx_voices = num_voices;
        params.rx_sample_size = imx->rx_cfg.sample_size == 2 ? 16 : 32;
        params.rx_sample_rate = imx->rx_cfg.sample_rate;
    }

    params.i2c_dev = imx->i2c_dev;
    params.i2c_addr = imx->i2c_addr;
    params.mclk = imx->sys_clk;
    params.clk_master = (imx->clk_mode == IMX_SAI_SLAVE) ? true : false;

    /* If NULL is passed in as the DLL name, this will cause the ado_mixer_dll() call
     * to look in the audio configuration file for the dll name (mixer_dll=xxxx)
     */
    status = ado_mixer_dll (card, NULL, I2S_CODEC_MINOR_VERSION, &params, &imx->callbacks, &imx->mixer);
    if (status != EOK) {
        status = errno;
        if (status == ENOENT) {
            // no mixer dll is found, need to open default one
            if ((status = ado_mixer_create (card, NULL, &imx->mixer, NULL)) != EOK)	{
                status = errno;
                ado_error_fmt("Unable to create mixer device - %s", strerror(status));
            }
        }
    } else {
        if (imx->callbacks.codec_set_default_group == NULL) {
            ado_error_fmt("Mixer DLL missing codec_set_defalut_group callback");
            status = ENOTSUP;
        }
    }

    return (status);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxsai/mixer.c $ $Rev: 905443 $")
#endif
