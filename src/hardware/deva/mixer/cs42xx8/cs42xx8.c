/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 * Copyright 2017-2019, QNX Software Systems.
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

struct cs42xx8_context;
#define  MIXER_CONTEXT_T struct cs42xx8_context

//#define  DUMP_OPTIONS  1

#include <audio_driver.h>
#include <hw/i2c.h>
#include <mixer/i2s_codec_dll.h>
#include "cs42xx8.h"


/**
 * CS42xx8 codec driver source file.
 *
 * @file       deva/codecs/cs42xx8/cs42xx8.c
 * @addtogroup cs42xx8
 * @{
 */

#define DEFAULT_CLK_RATIO  512
#define FORMAT_I2S         0
#define FORMAT_TDM         1

/** Channel mapping - 2 channels */
static snd_mixer_voice_t stereo_voices[2] = {
    { .voice = SND_MIXER_VOICE_LEFT,  .vindex = 0, .reserved = { 0 } },
    { .voice = SND_MIXER_VOICE_RIGHT, .vindex = 0, .reserved = { 0 } }
};

/** Channel mapping - 4 channels */
static snd_mixer_voice_t quad_voices[4] = {
    { .voice = SND_MIXER_VOICE_LEFT,       .vindex = 0, .reserved = { 0 } },
    { .voice = SND_MIXER_VOICE_RIGHT,      .vindex = 0, .reserved = { 0 } },
    { .voice = SND_MIXER_VOICE_REAR_LEFT,  .vindex = 0, .reserved = { 0 } },
    { .voice = SND_MIXER_VOICE_REAR_RIGHT, .vindex = 0, .reserved = { 0 } }
};

/** Channel mapping - 8 channels */
static snd_mixer_voice_t eight_voices[8] = {
    { .voice = SND_MIXER_VOICE_LEFT,       .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_RIGHT,      .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_CENTER,     .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_WOOFER,     .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_REAR_LEFT,  .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_REAR_RIGHT, .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_SURR_LEFT,  .vindex = 0, .reserved = {0} },
    { .voice = SND_MIXER_VOICE_SURR_RIGHT, .vindex = 0, .reserved = {0} }
};

/** Output Range */
static struct snd_mixer_element_volume1_range output_range[8] = {
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 255, .min_dB = -12750, .max_dB = 0, .dB_scale_factor = 100, .reserved = {0} }
};
/** Input Range */
static struct snd_mixer_element_volume1_range input_range[4] = {
    { .min = 0, .max = 176, .min_dB = -6400, .max_dB = 2400, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 176, .min_dB = -6400, .max_dB = 2400, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 176, .min_dB = -6400, .max_dB = 2400, .dB_scale_factor = 100, .reserved = {0} },
    { .min = 0, .max = 176, .min_dB = -6400, .max_dB = 2400, .dB_scale_factor = 100, .reserved = {0} }
};

/**
 * Writes data into external audio codec over i2c data bus.
 *
 * @param cs42xx8 Mixer context.
 * @param register_address Register address.
 * @param val Value to be written.
 * @param bitmask Bit mask for masking register bits.
 *
 * @return Execution status.
 */
static int cs42xx8_write(MIXER_CONTEXT_T * cs42xx8, uint8_t register_address, uint8_t val, uint8_t bitmask)
{
    int32_t status = EOK;

    if (cs42xx8->params.codec_write)
    {
        status = cs42xx8->params.codec_write(cs42xx8->params.hw_context, register_address, val, 0);
    }
    else
    {
        struct send {
            i2c_send_t hdr;
            uint8_t buf[2];
        } cs42xx8_wr_data;

        struct send_recv {
            i2c_sendrecv_t hdr;
            uint8_t buf[2];
        } cs42xx8_rd_data;

        uint8_t data_tmp;

        memset(&cs42xx8_wr_data, 0x0, sizeof(struct send));
        memset(&cs42xx8_rd_data, 0x0, sizeof(struct send_recv));

        cs42xx8_rd_data.buf[0] = register_address;
        cs42xx8_rd_data.hdr.send_len = 1;
        cs42xx8_rd_data.hdr.recv_len = 1;
        cs42xx8_rd_data.hdr.slave.addr = cs42xx8->i2c_addr;
        cs42xx8_rd_data.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
        cs42xx8_rd_data.hdr.stop = 0;

        if (devctl(cs42xx8->fd, DCMD_I2C_SENDRECV, &cs42xx8_rd_data, sizeof(cs42xx8_rd_data), NULL)) {
            ado_error("Failed to read codec reg values: %s\n", strerror(errno));
            return -1;
        }
        data_tmp = cs42xx8_rd_data.buf[0];
        /* Now that the register value is read, Set and clear the required bits only */
        cs42xx8_wr_data.buf[0] = register_address;
        cs42xx8_wr_data.buf[1] = (data_tmp & ~bitmask) | (val & bitmask);
        cs42xx8_wr_data.hdr.len = 2;
        cs42xx8_wr_data.hdr.slave.addr = cs42xx8->i2c_addr;
        cs42xx8_wr_data.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
        cs42xx8_wr_data.hdr.stop = 1;
        if (devctl(cs42xx8->fd, DCMD_I2C_SEND, &cs42xx8_wr_data, sizeof(cs42xx8_wr_data), NULL)) {
            ado_error_fmt("Failed to write to codec: %s\n", strerror(errno));
            return -1;
        }
    }
    return status;
}
#if 0
/**
 * Reads data from external audio codec over i2c data bus.
 *
 * @param cs42xx8 Mixer context.
 * @param register_address Register address.
 *
 * @return Read register content.
 */
static uint8_t cs42xx8_read(MIXER_CONTEXT_T * cs42xx8, uint8_t register_address)
{
    if (cs42xx8->params.codec_read)
    {
        uint32_t data;
        /* We only do error checking on the write call, if there are problems with
         * the I2C transport then we will catch it there.
         */
        cs42xx8->params.codec_read(cs42xx8->params.hw_context, register_address, &data, 0);
        return ((uint8_t) data);
    }
    else
    {
        struct send_recv {
            i2c_sendrecv_t hdr;
            uint8_t buf[2];
        } cs42xx8_rd_data;

        /* Read the Registers Current Value */
        cs42xx8_rd_data.buf[0] = register_address;
        cs42xx8_rd_data.hdr.send_len = 1;
        cs42xx8_rd_data.hdr.recv_len = 1;
        cs42xx8_rd_data.hdr.slave.addr = cs42xx8->i2c_addr;
        cs42xx8_rd_data.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
        cs42xx8_rd_data.hdr.stop = 1;

        if (devctl(cs42xx8->fd, DCMD_I2C_SENDRECV, &cs42xx8_rd_data, sizeof(cs42xx8_rd_data), NULL)) {
            ado_error("Failed to read codec: %s\n", strerror(errno));
        }
        return cs42xx8_rd_data.buf[0];
    }
}
#endif

/**
 * Controls analog in mute.
 *
 * @param cs42xx8       Mixer context.
 * @param element       Pointer to a mixer element.
 * @param set           When true a mute will be set. When false mute state will be got.
 * @param vol           Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case mute has been changed.
 */
static int32_t cs42xx8_ainx_mute_control(MIXER_CONTEXT_T * cs42xx8, ado_mixer_delement_t * element, uint8_t set,
                                         uint32_t * val, void *instance_data)
{
    int32_t altered = 0;
    int i, voice_offset, mixer_chn;
    uint32_t hw_mute_map = 0;                   /* Hardware mute bitmap */
    uint32_t mute_map = 0;                      /* Currently applied SND_MIXER_CHN* mute bitmap */
    uint32_t idx = (uintptr_t)instance_data;    /* Audio Interface index */
    uint32_t rx_voices = cs42xx8->params.rx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE];
    uint32_t offset = idx * rx_voices;          /* codec voice offset */
    uint32_t ainx_vol_ctl = AIN1_VOL_CTL + offset;
    const snd_pcm_chmap_t *chmap = (cs42xx8->params.get_chmap) ?
                                    cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_CAPTURE, idx) : NULL;

    for (i = 0, voice_offset = 0; i < rx_voices; i++)
    {
        if (!chmap || (chmap->pos[0] == SND_CHMAP_UNKNOWN) || (chmap->pos[0] == SND_CHMAP_NA))
        {
            /* If there is no chmap then use the chn_mask directly */
            while( !(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] & (1<<(i+voice_offset))))
            {
                /* Skip over holes in the channel mask */
                voice_offset++;
            }
            mixer_chn = (i+voice_offset);
        }
        else
        {
            /* Translate the chmap to a mixer_chn */
            mixer_chn = ado_chmap_to_mixer_chn ( chmap->pos[i] );
        }

        /* Current mute map translated to our SND_MIXER_CHN* mute mask */
        mute_map |= (cs42xx8->ain_mute[idx] & (1<<i)) ? (1<<mixer_chn) : 0;

        /* New hw_mute_map based on client requested mute levels */
        hw_mute_map |= (val[0] & (1<<mixer_chn)) ? (1 << i) : 0;

        if (set)
        {
            if (!(mute_map & (1<<mixer_chn)) && (hw_mute_map & (1<<i)))
            {
                altered = 1;
                /* 0x80 == -64dB closest to mute we can get via volume controls */
                cs42xx8->ain_mute[idx] |= (1<<i);
                cs42xx8_write(cs42xx8, ainx_vol_ctl + i, 0x80, 0xFF);
            }
            else if ((mute_map & (1<<mixer_chn)) && !(hw_mute_map & (1<<i)))
            {
                altered = 1;
                /* Restore configured volume level */
                cs42xx8->ain_mute[idx] &= ~(1<<i);
                cs42xx8_write(cs42xx8, ainx_vol_ctl + i, INPUT_VOL(cs42xx8->ain_vol[i+offset]), 0xFF);
            }
        }
    }

    if (!set)
    {
        val[0] = mute_map;
    }
    return altered;
}

/**
 * Controls analog input volume.
 *
 * @param cs42xx8       Mixer context.
 * @param element       Pointer to a mixer element.
 * @param set           When true a new volume will be set. When false volume will be got.
 * @param vol           Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t cs42xx8_ainx_vol_control(MIXER_CONTEXT_T * cs42xx8, ado_mixer_delement_t * element,
                                        uint8_t set, uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    uint32_t idx = (uintptr_t)instance_data;    /* Audio Interface index */
    uint32_t rx_voices = cs42xx8->params.rx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE];
    uint32_t offset = idx * rx_voices;          /* codec voice offset */
    uint32_t ainx_vol_ctl = AIN1_VOL_CTL + offset;
    int i, voice_offset, mixer_chn;
    const snd_pcm_chmap_t *chmap = (cs42xx8->params.get_chmap) ?
                                    cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_CAPTURE, idx) : NULL;

    for (i = 0, voice_offset = 0; i < rx_voices; i++)
    {
        if (!chmap || (chmap->pos[0] == SND_CHMAP_UNKNOWN) || (chmap->pos[0] == SND_CHMAP_NA))
        {
            /* If there is no chmap then use the chn_mask directly */
            while( !(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] & (1<<(i+voice_offset))))
            {
                /* Skip over holes in the channel mask */
                voice_offset++;
            }
            mixer_chn = (i+voice_offset);
        }
        else
        {
            /* Translate the chmap to a mixer_chn */
            mixer_chn = ado_chmap_to_mixer_chn ( chmap->pos[i] );
        }

        if (set)
        {
            if (cs42xx8->ain_vol[i+offset] != vol[mixer_chn])
            {
                altered = 1;
                cs42xx8->ain_vol[i+offset] = vol[mixer_chn];
                if (!(cs42xx8->ain_mute[idx] & (1<<mixer_chn))) {
                    /* Set volume */
                    cs42xx8_write(cs42xx8, ainx_vol_ctl + i, INPUT_VOL(cs42xx8->ain_vol[i+offset]), 0xFF);
                }
            }
        }
        else
        {
            vol[mixer_chn] = cs42xx8->ain_vol[i+offset];
        }
    }

    return altered;
}

/**
 * Controls analog out mute.
 *
 * @param cs42xx8       Mixer context.
 * @param element       Pointer to a mixer element.
 * @param set           When true a mute will be set. When false mute state will be got.
 * @param vol           Mute value.
 * @param instance_data Driver instance data. Index to virtual PCM device.
 *
 * @return Returns 1 in case mute has been changed.
 */
static int32_t cs42xx8_aoutx_mute_control(MIXER_CONTEXT_T * cs42xx8, ado_mixer_delement_t * element,
                                          uint8_t set, uint32_t * val, void *instance_data)
{
    int32_t altered = 0;
    int i, voice_offset, mixer_chn;
    uint32_t hw_mute_map = 0;                   /* Hardware mute bitmap */
    uint32_t mute_map = 0;                      /* Currently applied SND_MIXER_CHN* mute bitmap */
    uint32_t idx = (uintptr_t)instance_data;    /* Audio Interface index */
    uint32_t tx_voices = cs42xx8->params.tx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK];
    uint32_t offset = idx * tx_voices;          /* codec voice offset */
    const snd_pcm_chmap_t *chmap = (cs42xx8->params.get_chmap) ?
                                    cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_PLAYBACK, idx) : NULL;

    for (i = 0, voice_offset = 0; i < tx_voices; i++)
    {
        if (!chmap || (chmap->pos[0] == SND_CHMAP_UNKNOWN) || (chmap->pos[0] == SND_CHMAP_NA))
        {
            /* If there is no chmap then use the chn_mask directly */
            while( !(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] & (1<<(i+voice_offset))))
            {
                /* Skip over holes in the channel mask */
                voice_offset++;
            }
            mixer_chn = (i+voice_offset);
        }
        else
        {
            /* Translate the chmap to a mixer_chn */
            mixer_chn = ado_chmap_to_mixer_chn ( chmap->pos[i] );
        }

        /* Current mute map translated to our SND_MIXER_CHN* mute mask */
        mute_map |= (cs42xx8->aout_mute & (1<<(i+offset))) ? (1<<mixer_chn) : 0;

        /* New hw_mute_map based on client requested mute levels */
        hw_mute_map |= (val[0] & (1<<mixer_chn)) ? (1 << (i+offset)) : 0;
    }

    if (set)
    {
        altered = (val[0] != mute_map);
        if (altered)
        {
            cs42xx8->aout_mute &= ~(ADO_CONTIG_VOICE_MASK(tx_voices) << offset);
            cs42xx8->aout_mute |= hw_mute_map;
            /* Only apply the mute change in hardware if playback is active */
            if (cs42xx8->codec_playback_on == true) {
                cs42xx8_write(cs42xx8, DAC_CH_MUTE, cs42xx8->aout_mute, AOUT_MUTE_MASK);
            }
        }
    }
    else
    {
        val[0] = mute_map;
    }

    return altered;
}

/**
 * Controls analog output volume.
 *
 * @param cs42xx8       Mixer context.
 * @param element       Pointer to a mixer element.
 * @param set           When true a new volume will be set. When false volume will be got.
 * @param vol           Volume value.
 * @param instance_data Driver instance data. Index to virtual PCM device.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t cs42xx8_aoutx_vol_control(MIXER_CONTEXT_T * cs42xx8, ado_mixer_delement_t * element,
                                         uint8_t set, uint32_t * vol, void *instance_data)
{
    unsigned int voice_offset, mixer_chn;
    int i;
    int32_t altered = 0;
    uint32_t idx = (uintptr_t)instance_data;    /* Audio Interface index */
    uint32_t tx_voices = cs42xx8->params.tx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK];
    uint32_t offset = idx * tx_voices;          /* codec voice offset */
    uint32_t aoutx_vol_ctl = AOUT1_VOL_CTL + offset;
    const snd_pcm_chmap_t *chmap = (cs42xx8->params.get_chmap) ?
                                    cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_PLAYBACK, idx) : NULL;

    for (i = 0, voice_offset = 0; i < tx_voices; i++)
    {
        if (!chmap || (chmap->pos[0] == SND_CHMAP_UNKNOWN) || (chmap->pos[0] == SND_CHMAP_NA))
        {
            /* If there is no chmap then use the chn_mask directly */
            while( !(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] & (1<<(i+voice_offset))))
            {
                /* Skip over holes in the channel mask */
                voice_offset++;
            }
            mixer_chn = (i+voice_offset);
        }
        else
        {
            /* Translate the chmap to a mixer_chn */
            mixer_chn = ado_chmap_to_mixer_chn ( chmap->pos[i] );
        }
        if (set)
        {
            if (cs42xx8->aout_vol[i + offset] != vol[mixer_chn])
            {
                altered = 1;
                cs42xx8->aout_vol[i + offset] = vol[mixer_chn];
                /* Set volume */
                cs42xx8_write(cs42xx8, aoutx_vol_ctl + i, OUTPUT_VOL(cs42xx8->aout_vol[i+offset]), 0xFF);
            }
        }
        else
        {
            vol[mixer_chn] = cs42xx8->aout_vol[i + offset];
        }
    }

    return altered;
}

/**
 * Builds ado mixer according to codec capabilities.
 *
 * @param cs42xx8   Mixer context.
 * @param mixer     A pointer to the ado_mixer_t structure for the mixer.
 *
 * @return Execution status.
 */
static int32_t cs42xx8_mixer_build(MIXER_CONTEXT_T * cs42xx8, ado_mixer_t * mixer)
{
    ado_mixer_delement_t *play_vol;
    ado_mixer_delement_t *play_mute;
    ado_mixer_delement_t *capture_vol;
    ado_mixer_delement_t *capture_mute;
    ado_mixer_dgroup_t *play_grp;
    ado_mixer_dgroup_t *capture_grp;
    int error = 0;
    ado_mixer_delement_t *pre_elem = NULL, *elem = NULL;
    snd_mixer_voice_t *mixer_voices;
    uint32_t idx;
    char *name = NULL;

    /*#################### */
    /*## Playback Group ## */
    /*#################### */
    for (idx = 0; idx < cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK]; idx++)
    {
        int voices = 0;
        /* Create mixer channel mask based on chmap (PCM Channel map) */
        if (cs42xx8->params.get_chmap)
        {
            const snd_pcm_chmap_t *chmap = cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_PLAYBACK, idx);
            if (chmap != NULL)
            {
                /* Set the number of voices and channel mask based on the chmap */
                voices = chmap->channels;
                cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] = ado_chmap_to_mixer_chn_mask(chmap, voices);
            }
        }
        if (voices == 0)
        {
            /* If we could not get the voices from the chmap then evenly divided
             * the full number of voices into the number of individual devices
             */
            voices = cs42xx8->params.tx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK];
        }

        switch (voices)
        {
            case 2:
                mixer_voices = stereo_voices;
                if (cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] == 0)
                    cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] = SND_MIXER_CHN_MASK_STEREO;
                break;
            case 4:
                mixer_voices = quad_voices;
                if (cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] == 0)
                    cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] = SND_MIXER_CHN_MASK_4;
                break;
            case 8:
                mixer_voices = eight_voices;
                if (cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] == 0)
                    cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx] = SND_MIXER_CHN_MASK_7_1;
                break;
            default:
                ado_error_fmt("Invalid tx_voices %d", voices);
                return -1;
        }

        if (!error && (pre_elem = ado_mixer_element_pcm1(mixer, SND_MIXER_ELEMENT_PLAYBACK,
                                                         SND_MIXER_ETYPE_PLAYBACK1, 1, (int32_t*)&idx)) == NULL) {
            error++;
        }
        if (!error && (elem = ado_mixer_element_volume1(mixer, "PCM Volume", voices,
                                                        output_range, (void *) cs42xx8_aoutx_vol_control,
                                                        (void *)(uintptr_t)idx, NULL)) == NULL) {
            error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        play_vol = elem;
        pre_elem = elem;
        if (!error && (elem = ado_mixer_element_sw1(mixer, "PCM Mute", voices,
                                                    (void *) cs42xx8_aoutx_mute_control,
                                                    (void*)(uintptr_t)idx, NULL)) == NULL) {
            error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        play_mute = elem;
        pre_elem = elem;
        if (!error && (elem = ado_mixer_element_io(mixer, "PCM OUT", SND_MIXER_ETYPE_OUTPUT, 0,
                                                   voices, mixer_voices)) == NULL) {
            error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        if (!error && (play_grp = ado_mixer_playback_group_create(mixer, SND_MIXER_MASTER_OUT,
                                                                  cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK][idx],
                                                                  play_vol, play_mute)) == NULL) {
            error++;
        }
    }

    /*###################*/
    /*## Capture Group ##*/
    /*###################*/
    for (idx = 0; idx < cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE]; idx++)
    {
        int voices = 0;
        /* Create mixer channel mask based on chmap (PCM Channel map) */
        if (cs42xx8->params.get_chmap)
        {
            const snd_pcm_chmap_t *chmap = cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_CAPTURE, idx);
            if (chmap != NULL)
            {
                /* Set the number of voices and channel mask based on the chmap */
                voices = chmap->channels;
                cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] = ado_chmap_to_mixer_chn_mask(chmap, voices);
            }
        }
        if (voices == 0)
        {
            /* If we could not get the voices from the chmap then evenly divided
             * the full number of voices into the number of individual devices
             */
            voices = cs42xx8->params.rx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE];
        }

        switch (voices)
        {
            case 2:
                mixer_voices = stereo_voices;
                if (cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] == 0)
                    cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] = SND_MIXER_CHN_MASK_STEREO;
                break;
            case 4:
                mixer_voices = quad_voices;
                if (cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] == 0)
                    cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx] = SND_MIXER_CHN_MASK_4;
                break;
            default:
                ado_error_fmt("Invalid rx_voices %d", voices);
                return -1;
        }

        if (!error && (pre_elem = ado_mixer_element_pcm1(mixer, SND_MIXER_ELEMENT_CAPTURE,
                                                         SND_MIXER_ETYPE_CAPTURE1, 1, (int32_t*)&idx)) == NULL) {
            error++;
        }
        name = (voices == 4) ? "PCM In Volume" : ((idx == 0) ? "Line In Volume" : "Mic In Volume");
        if (!error && (elem = ado_mixer_element_volume1(mixer, name,
                                                        voices, input_range,
                                                        (void *) cs42xx8_ainx_vol_control,
                                                        (void *)(uintptr_t)idx, NULL)) == NULL) {
            error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        capture_vol = elem;
        pre_elem = elem;
        name = (voices == 4) ? "PCM In Mute" : ((idx == 0) ? "Line In Mute" : "Mic In Mute");
        if (!error && (elem = ado_mixer_element_sw1(mixer, name,
                                                    voices,
                                                    (void *) cs42xx8_ainx_mute_control,
                                                    (void *)(uintptr_t)idx, NULL)) == NULL) {
             error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        capture_mute = elem;
        pre_elem = elem;
        name = (voices == 4) ? "PCMIn" : ((idx == 0) ? "LineIn" : "MicIn");
        if (!error && (elem = ado_mixer_element_io(mixer, name, SND_MIXER_ETYPE_INPUT,
                                                   0, voices, mixer_voices)) == NULL) {
            error++;
        }
        if (!error && ado_mixer_element_route_add(mixer, pre_elem, elem) != 0) {
            error++;
        }
        name = (voices == 4) ? SND_MIXER_PCM_IN : ((idx == 0) ? SND_MIXER_LINE_IN : SND_MIXER_MIC_IN);
        if (!error && (capture_grp = ado_mixer_capture_group_create(mixer, name, cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE][idx],
                                                                    capture_vol, capture_mute, NULL, NULL)) == NULL) {
            error++;
        }
    }
    return (!error ? 0 : -1);
}

#if defined DUMP_OPTIONS
static void dump_options(const char** keys, const char** vals, int nOptions)
{
    int cnt;
    for (cnt = 0; cnt < nOptions; cnt++) {
        if ( vals[cnt] )
            ado_error_fmt("%s = %s", keys[cnt], vals[cnt]);
    }
}
#endif /* DUMP_OPTIONS */

/**
 * Gets cs42xx8 mode configuration according to required MCLK ratio
 * @param ratio MCLK ratio.
 * @return Mode configuration value.
 */
static uint32_t get_mode_bits(int ratio)
{
    switch (ratio) {
        case 256:
            return 0x0;
        case 384:
            return 0x2;
        case 512:
            return 0x4;
        case 768:
            return 0x6;
        case 1024:
            return 0x8;
        default:
            return 0x0;
    }
}
/** Codec config file opts */
static const char *opts[] = {
#define CLK_RATIO             0
    "cs42xx8_clk_ratio",
#define FORMAT                1
    "cs42xx8_format",
    NULL
};

#define N_OPTS ((sizeof(opts)/sizeof(opts[0])) - 1U)

static int32_t
parse_config_opts ( MIXER_CONTEXT_T *cs42xx8 )
{
    const ado_dict_t *dict = NULL;
    const char* optValues[N_OPTS] = {0};
    uint32_t tmp_reg;
    ado_card_t *card = ado_mixer_get_card(cs42xx8->mixer);

    if(!card)
    {
        printf("%s: could not get card from mixer\n", __FUNCTION__);
        return EINVAL;
    }
    /* Set default ratio between master and frame clock */
    cs42xx8->mode_bits = get_mode_bits(DEFAULT_CLK_RATIO);

    /* Format is I2S by default */
    cs42xx8->format_bits = (ADC_DIF_I2S | DAC_DIF_I2S);

    dict = ado_get_card_dict( card );
    if (dict) {
        ado_config_load_key_values(dict, opts, optValues, N_OPTS, 0, -1);

        if (optValues[CLK_RATIO])
        {
            tmp_reg = strtoul(optValues[CLK_RATIO], 0, 0);
            if (tmp_reg == 0)
            {
                tmp_reg = DEFAULT_CLK_RATIO;
                ado_error("CS42448: invalid clock ratio, using default ratio (512)");
            }
            cs42xx8->mode_bits = get_mode_bits(tmp_reg);

        }
        else
            goto fail;
        if (optValues[FORMAT])
        {
            tmp_reg = strtoul(optValues[FORMAT], 0, 0);
            if(tmp_reg == FORMAT_I2S)
            {
                /* Configure Interface Formats */
                cs42xx8->format_bits = (ADC_DIF_I2S | DAC_DIF_I2S);
            }
            else
            {
                cs42xx8->format_bits = (ADC_DIF_TDM | DAC_DIF_TDM);
            }
        }
        else
        {
            goto fail;
        }
#if defined DUMP_OPTIONS
        dump_options(opts, optValues, N_OPTS);
#endif
    } else {
        ado_debug(DB_LVL_MIXER, "No mixer dll in config file, using default value");
    }
    return EOK;
  fail:
    ado_error_fmt("Config files is missing mandatory entries");
    return EINVAL;
}

static void cs42xx8_close ( cs42xx8_context_t* cs42xx8 )
{
    if (cs42xx8->params.codec_close)
    {
        cs42xx8->params.codec_close(cs42xx8->params.hw_context);
    }
    else
    {
        close(cs42xx8->fd);
    }
}

/**
 * Resets the codec.
 *
 * @param cs42xx8 Mixer context.
 *
 * @return Execution status.
 */
static int32_t cs42xx8_reset(MIXER_CONTEXT_T * cs42xx8)
{
    int cnt;
    /* Power down */
    cs42xx8_write(cs42xx8, PWR_CTL, PDN, PDN); /* Power down Codec */
    cs42xx8_write(cs42xx8, PWR_CTL, 0xFE, 0xFE); /* Power down all DACs and ADCs */

    /* SOC is master, codec is slave */
    cs42xx8_write(cs42xx8, FN_MODE, ( cs42xx8->ms_bits| cs42xx8->mode_bits), 0xFE);

    /* Configure Interface Formats */
    cs42xx8_write(cs42xx8, IFACE_FMT, cs42xx8->format_bits, 0xFF);

    /* Configure ADC Control Register */
    cs42xx8_write(cs42xx8, ADC_CTL, (ADC1_SINGLE), 0xFF);

    /* Mute DACs immediatly (i.e. before we enable zero-cross) to minimize pops/clicks*/
    cs42xx8_write(cs42xx8, DAC_CH_MUTE, AOUT_MUTE_MASK, AOUT_MUTE_MASK);

    /* Configure Transition Control (Zero Cross & Soft Ramp ) */
    cs42xx8->codec_capture_on = true;
    cs42xx8_write(cs42xx8, TRANS_CTL, DAC_ZCROSS | DAC_SRAMP | ADC_MUTE_SP | ADC_ZCROSS, 0xFF);

    /* Initialize DAC Volume to 0dB */
    for (cnt = 0; cnt < CS42XX8_MAX_OUTPUTS; cnt++) {
        /* initialize tx volume registers */
        cs42xx8->aout_vol[cnt] = AOUT_0DB;
        cs42xx8_write(cs42xx8, AOUT1_VOL_CTL + cnt, OUTPUT_VOL(cs42xx8->aout_vol[cnt]), 0xFF);
    }

    /* Initialize ADC Volume to 0dB */
    for (cnt = 0; cnt < CS42XX8_MAX_INPUTS; cnt++) {
        cs42xx8->ain_vol[cnt] = AIN_0DB;
        cs42xx8_write(cs42xx8, AIN1_VOL_CTL + cnt, INPUT_VOL(cs42xx8->ain_vol[cnt]), 0xFF);
    }

    cs42xx8_write(cs42xx8, PWR_CTL, 0x0, PDN); /* Power up Codec */

    /* Power up DAC */
    cs42xx8_write(cs42xx8, PWR_CTL, 0, (PDN_DAC1 | PDN_DAC2 | PDN_DAC3 | PDN_DAC4));

    /* Power up ADC */
    cs42xx8_write(cs42xx8, PWR_CTL, 0, (PDN_ADC1 | PDN_ADC2 ));
    delay(400); /* Wait 400 ms for dc-blocking capacitors charge */

    /* Unmute DAC */
    cs42xx8->aout_mute = 0;
    cs42xx8->codec_playback_on = true;
    cs42xx8_write(cs42xx8, DAC_CH_MUTE, cs42xx8->aout_mute, AOUT_MUTE_MASK);

    return 0;
}

/**
 * Destroys codec context and Power downs the codec.
 *
 * @param cs42xx8 Mixer context.
 *
 * @return Execution status.
 */
static int32_t cs42xx8_destroy(MIXER_CONTEXT_T * cs42xx8)
{
    ado_debug(DB_LVL_MIXER, "Destroying CS42448 Codec");
    cs42xx8_write(cs42xx8, DAC_CH_MUTE, AOUT_MUTE_MASK, AOUT_MUTE_MASK); /* Mute DACs */
    cs42xx8_write(cs42xx8, PWR_CTL, 0xFE, 0xFE); /* Power down all DACs and ADCs */
    cs42xx8_write(cs42xx8, PWR_CTL, PDN, PDN); /* Power down Codec */
    cs42xx8_close(cs42xx8);
    ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
    ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
    ado_free(cs42xx8);
    return 0;
}

void cs42xx8_codec_on(ado_mixer_t * mixer, int channel)
{
    MIXER_CONTEXT_T * cs42xx8 = ado_mixer_get_context(mixer);

    if(!cs42xx8)
        return;

    /* Unmute DAC */
    if(channel == ADO_PCM_CHANNEL_PLAYBACK)
    {
        cs42xx8_write(cs42xx8, DAC_CH_MUTE, cs42xx8->aout_mute, AOUT_MUTE_MASK);
        cs42xx8->codec_playback_on = true;
    }
    else
    {
        /* Unmute ADC serial port */
        cs42xx8_write(cs42xx8, TRANS_CTL, 0, ADC_MUTE_SP);
        cs42xx8->codec_capture_on = true;
    }
}

void cs42xx8_codec_off(ado_mixer_t * mixer, int channel)
{
    MIXER_CONTEXT_T * cs42xx8 = ado_mixer_get_context(mixer);

    if(!cs42xx8)
        return;

    if(channel == ADO_PCM_CHANNEL_PLAYBACK)
    {
        /* Mute DAC */
        cs42xx8->codec_playback_on = false;
        cs42xx8_write(cs42xx8, DAC_CH_MUTE, AOUT_MUTE_MASK, AOUT_MUTE_MASK);
    }
    else
    {
        /* Mute ADC serial port */
        cs42xx8->codec_capture_on = false;
        cs42xx8_write(cs42xx8, TRANS_CTL, ADC_MUTE_SP, ADC_MUTE_SP);
    }
}

/**
 * Called by audio controller to associate a mixer element and group with a PCM device.
 *
 * @param pcm     Pointer to PCM data
 * @param mixer   Pointer to Mixer data
 * @param channel Channel type
 * @param index   Element index
 */
void codec_set_default_group(ado_pcm_t *pcm, ado_mixer_t *mixer, int channel, int index)
{
    MIXER_CONTEXT_T * cs42xx8 = ado_mixer_get_context(mixer);

    switch (channel) {
        case ADO_PCM_CHANNEL_PLAYBACK:
            ado_pcm_chn_mixer(pcm, ADO_PCM_CHANNEL_PLAYBACK, mixer,
                              ado_mixer_find_element(mixer, SND_MIXER_ETYPE_PLAYBACK1, SND_MIXER_ELEMENT_PLAYBACK, index),
                              ado_mixer_find_group(mixer, SND_MIXER_MASTER_OUT, index));
            break;
        case ADO_PCM_CHANNEL_CAPTURE:
        {
            uint32_t rx_voices = cs42xx8->params.rx_voices / cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE];
            ado_pcm_chn_mixer(pcm, ADO_PCM_CHANNEL_CAPTURE, mixer,
                              ado_mixer_find_element(mixer, SND_MIXER_ETYPE_CAPTURE1, SND_MIXER_ELEMENT_CAPTURE, index),
                              ado_mixer_find_group(mixer, (rx_voices == 4 ?
                                                           SND_MIXER_PCM_IN : index == 0 ? SND_MIXER_LINE_IN : SND_MIXER_MIC_IN), 0));
            break;
        }
        default:
            break;
    }
}

ado_dll_version_t version;
void
version (int *major, int *minor, char *date)
{
    *major = ADO_MAJOR_VERSION;
    *minor = I2S_CODEC_MINOR_VERSION;
    date = __DATE__;
    ado_debug(DB_LVL_DRIVER, "Date = %s", date);
}

ado_mixer_dll_init_t mixer_dll_init;

int
mixer_dll_init (MIXER_CONTEXT_T ** context, ado_mixer_t * mixer, void *params,
                void *raw_callbacks, int version)
{
    char i2c_port[20];
    unsigned int speed;
    cs42xx8_context_t *cs42xx8;
    ado_mixer_dll_codec_callbacks_t *callbacks = raw_callbacks;

    ado_debug(DB_LVL_MIXER, "Initializing CS42XX8 Codec");

    if ((cs42xx8 = (cs42xx8_context_t *) ado_calloc(1, sizeof (cs42xx8_context_t))) == NULL)
    {
        ado_error_fmt("CS42XX8: Failed to allocate device structure - %s", strerror(errno));
        return (-1);
    }
    *context = cs42xx8;
    ado_mixer_set_name (mixer, "CS42XX8");

    cs42xx8->mixer = mixer;
    memcpy (&cs42xx8->params, params, sizeof (cs42xx8->params));

    if (cs42xx8->params.is_active == NULL)
    {
        ado_error_fmt ("CS42XX8: is_active routine missing");
        ado_free(cs42xx8);
        return (-1);
    }

    if (cs42xx8->params.tx_voices > CS42XX8_MAX_OUTPUTS)
    {
        ado_error_fmt("Invalid tx_voice settings %d, must be < %d", cs42xx8->params.tx_voices, CS42XX8_MAX_OUTPUTS);
        return (-1);
    }
    if (cs42xx8->params.rx_voices > CS42XX8_MAX_INPUTS)
    {
        ado_error_fmt("Invalid rx_voice settings %d, must be < %d", cs42xx8->params.rx_voices, CS42XX8_MAX_INPUTS);
        return (-1);
    }

    if (cs42xx8->params.get_chmap)
    {
        /* Count the number of chmaps to determine the number of devices
         * and verify all devices have the same number of voices
         */
        int idx = 0;
        uint32_t voices = 0;
        const snd_pcm_chmap_t *chmap;
        while ((chmap = cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_PLAYBACK, idx)) != NULL)
        {
            /* Verify all devices have the same number of voices */
            if (idx == 0)
                voices = chmap->channels;
            else
            {
                if (chmap->channels != voices)
                {
                    ado_error_fmt("All playback devices must have the same number of channels");
                    return (-1);
                }
            }
            idx++;
        }
        voices = voices * idx;
        if (voices != cs42xx8->params.tx_voices)
        {
            ado_error_fmt("Channel map voices (%d) and params.tx_voices (%d) do not agree", voices, cs42xx8->params.tx_voices);
            return (-1);
        }
        cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK] = idx;
    }
    else
    {
        /* Default to one playback device if there are no chmaps */
        cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK] = 1;
    }
    cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK] = ado_calloc(cs42xx8->ndevs[SND_PCM_CHANNEL_PLAYBACK], sizeof(int32_t));

    if (cs42xx8->params.get_chmap)
    {
        /* Count the number of chmaps to determine the number of devices
         * and verify all devices have the same number of voices
         */
        int idx = 0;
        uint32_t voices = 0;
        const snd_pcm_chmap_t *chmap;
        while ((chmap = cs42xx8->params.get_chmap(cs42xx8->params.hw_context, ADO_PCM_CHANNEL_CAPTURE, idx)) != NULL)
        {
            if (idx == 0)
                voices = chmap->channels;
            else
            {
                if (chmap->channels != voices)
                {
                    ado_error_fmt("All capture devices must have the same number of channels");
                    return (-1);
                }
            }
            idx++;
        }
        voices = voices * idx;
        if (voices != cs42xx8->params.rx_voices)
        {
            ado_error_fmt("Channel map voices (%d) and params.rx_voices (%d) do not agree", voices, cs42xx8->params.rx_voices);
            return (-1);
        }
        cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE] = idx;
    }
    else
    {
        /* Default to one capture device if there are no chmaps */
        cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE] = 1;
    }
    cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE] = ado_calloc(cs42xx8->ndevs[SND_PCM_CHANNEL_CAPTURE], sizeof(int32_t));

    cs42xx8->i2c_dev = cs42xx8->params.i2c_dev;
    cs42xx8->i2c_addr = cs42xx8->params.i2c_addr;
    cs42xx8->ms_bits = ((cs42xx8->params.clk_master == true) ?
                        (ADC_FM_MASTER_SS | DAC_FM_MASTER_SS) :
                        (ADC_FM_SLAVE | DAC_FM_SLAVE));

    if (parse_config_opts(cs42xx8) != EOK)
    {
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
        ado_free(cs42xx8);
        return -1;
    }
    if (callbacks)
    {
        callbacks->codec_set_default_group = codec_set_default_group;
        callbacks->codec_on = cs42xx8_codec_on;
        callbacks->codec_off = cs42xx8_codec_off;
    }

    if (cs42xx8->params.codec_open)
    {
        if (cs42xx8->params.codec_open(cs42xx8->params.hw_context) != EOK)
        {
            ado_error_fmt("CS42XX8: codec open failed");
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
            ado_free(cs42xx8);
            return (-1);
        }
    }
    else
    {
        sprintf(i2c_port, "/dev/i2c%d", cs42xx8->i2c_dev);
        if ((cs42xx8->fd = open(i2c_port, O_RDWR)) < 0)
        {
            ado_error_fmt("CS42XX8: could not open i2c device - %s", strerror(errno));
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
            ado_free(cs42xx8);
            return (-1);
        }

        /* Set the bus speed to 400K */
        speed = 400000;
        if (devctl (cs42xx8->fd, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof (speed), NULL))
        {
            ado_error ("cs42xx8: Failed to set I2C bus speed");
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
            ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
            cs42xx8_close(cs42xx8);
            ado_free (cs42xx8);
            return -1;
        }
    }

    if (cs42xx8_mixer_build(cs42xx8, cs42xx8->mixer))
    {
        cs42xx8_close(cs42xx8);
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
        ado_free(cs42xx8);
        return (-1);
    }

    /* reset codec(s) */
    if(cs42xx8_reset(cs42xx8) == -1)
    {
        ado_error("cs42xx8: could not initialize codec");
        cs42xx8_close(cs42xx8);
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_PLAYBACK]);
        ado_free(cs42xx8->chn_mask[SND_PCM_CHANNEL_CAPTURE]);
        ado_free(cs42xx8);
        return (-1);
    }
    ado_mixer_set_reset_func(cs42xx8->mixer, cs42xx8_reset);
    ado_mixer_set_destroy_func(cs42xx8->mixer, cs42xx8_destroy);

    return (0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/mixer/cs42xx8/cs42xx8.c $ $Rev: 903313 $")
#endif
