/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 * Copyright 2018, QNX Software Systems.
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

struct wm8960_context;
#define MIXER_CONTEXT_T struct wm8960_context

#include <string.h>
#include <stdint.h>
#include <audio_driver.h>
#include <hw/i2c.h>
#include <mixer/i2s_codec_dll.h>
#include "wm8960.h"

/** One playback device, one capture device. */
static int32_t pcm_devices[1] = { 0 };

/** Channel mapping */
static snd_mixer_voice_t stereo_voices[2] = {
    {SND_MIXER_VOICE_LEFT, 0},
    {SND_MIXER_VOICE_RIGHT, 0}
};

/** Speaker Range */
static struct snd_mixer_element_volume1_range spk_range[2] = {
    {0, WM8960_SPKOUT_VOL_MAX - WM8960_SPKOUT_VOL_MIN, -7300, 600, 100},
    {0, WM8960_SPKOUT_VOL_MAX - WM8960_SPKOUT_VOL_MIN, -7300, 600, 100},
};

/** Headphone Range */
static struct snd_mixer_element_volume1_range hp_range[2] = {
    {0, WM8960_HPOUT_VOL_MAX - WM8960_HPOUT_VOL_MIN, -7300, 600, 100},
    {0, WM8960_HPOUT_VOL_MAX - WM8960_HPOUT_VOL_MIN, -7300, 600, 100},
};

/** DAC Master Range */
static struct snd_mixer_element_volume1_range master_range[2] = {
    {0, WM8960_DAC_VOL_MAX - WM8960_DAC_VOL_MIN, -12700, 0, 100},
    {0, WM8960_DAC_VOL_MAX - WM8960_DAC_VOL_MIN, -12700, 0, 100},
};

/** ADC IGAIN Range */
static struct snd_mixer_element_volume1_range adc_range[2] = {
    {0, WM8960_ADC_VOL_MAX - WM8960_ADC_VOL_MIN, -9700, 3000, 100},
    {0, WM8960_ADC_VOL_MAX - WM8960_ADC_VOL_MIN, -9700, 3000, 100},
};

/** Analog Input Range */
static struct snd_mixer_element_volume1_range input_range[2] = {
    {0, WM8960_INPUT_VOL_MAX - WM8960_INPUT_VOL_MIN, -1725, 3000, 100},
    {0, WM8960_INPUT_VOL_MAX - WM8960_INPUT_VOL_MIN, -1725, 3000, 100},
};

/**
 * Writes data into external audio codec over i2c data bus.
 *
 * @param mixer Mixer context.
 * @param reg WM8960 register address.
 * @param val Value to be written.
 *
 * @return Execution status.
 */
static int wm8960_write(MIXER_CONTEXT_T * mixer, uint32_t reg, uint32_t val)
{
    int status = EOK;
    uint32_t i = 0;

    struct {
        i2c_send_t hdr;
        uint16_t regval;
    } msg;

    msg.hdr.slave.addr = mixer->i2c_addr;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.len = 2;
    msg.hdr.stop = 1;
    msg.regval = (0xff & ((reg << 1) | (0x1 & (val >> 8)))) | ((0xff & val) << 8);

    do {
        status = devctl(mixer->i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL);
    } while ((status == EAGAIN) && (i++ < 10) && (usleep(10000) == 0));

    if (status != EOK) {
        ado_error_fmt("I2C_WRITE failed");
    } else {
        /* Save written value */
        mixer->regs[reg] = val;
    }
    return status;
}

/**
 * Provides a non-destructive way of modifying register contents.
 *
 * @param mixer Mixer context.
 * @param reg WM8960 register address.
 * @param mask The mask code for the bits want to change.
 * @param val Value to be written.
 *
 * @return Execution status.
 */
static int wm8960_update(MIXER_CONTEXT_T * mixer, uint32_t reg, uint32_t mask, uint32_t val, uint32_t write_reg)
{
    /* WM8960 can't accept read commands so we read from regs buffer */
    uint32_t current_value = mixer->regs[reg];

    /* Always update the regs buffer */
    current_value &= ~mask;

    if(write_reg == WM8960_UPDATE_AND_WRITE_REG) { /* If a bit isn't sticky we avoid an extra codec access */
        return wm8960_write(mixer, reg, current_value | val);
    } else if(write_reg == WM8960_UPDATE_ONLY) {
        mixer->regs[reg] = current_value | val;
        return EOK;
    }

    return -1;
}

/**
 * Controls Speaker mute.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a mute will be set. When false mute state will be got.
 * @param vol Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case mute has been changed.
 */
static int32_t wm8960_spk_mute_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                       uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        altered =
            (wm8960->spk_mute != (vol[0] & (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT))) ? 1 : 0;
        if (altered) {
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                wm8960->spk_mute |= SND_MIXER_CHN_MASK_FRONT_LEFT;
            } else {
                wm8960->spk_mute &= ~SND_MIXER_CHN_MASK_FRONT_LEFT;
            }

            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                wm8960->spk_mute |= SND_MIXER_CHN_MASK_FRONT_RIGHT;
            } else {
                wm8960->spk_mute &= ~SND_MIXER_CHN_MASK_FRONT_RIGHT;
            }

            /* We only apply the volume update bit when we write to the last channel
             * channel (right) so mute updates across all channels are synchronous
             */
            if (wm8960->spk_mute & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_L, WM8960_SPKOUT_VOL_CTRL_MASK,
                                       WM8960_SPKOUT_VOL_MUTE_VAL, WM8960_UPDATE_AND_WRITE_REG);
            } else {
                status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_L, WM8960_SPKOUT_VOL_CTRL_MASK,
                                       wm8960->spk_volume[0], WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
            if (wm8960->spk_mute & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R, WM8960_SPKOUT_VOL_CTRL_MASK,
                                       WM8960_SPKOUT_VOL_MUTE_VAL | WM8960_SPKOUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            } else {
                status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R, WM8960_SPKOUT_VOL_CTRL_MASK,
                                       wm8960->spk_volume[1] | WM8960_SPKOUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R, WM8960_SPKOUT_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
        }
    } else {
        vol[0] = wm8960->spk_mute;
    }

    return (altered);
}
/**
 * Controls Speaker volume.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new volume will be set. When false volume will be got.
 * @param vol Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_spk_vol_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                      uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        // This offset is used since spk_range was shifted to start from 0
        vol[0] += WM8960_SPKOUT_VOL_MIN;
        vol[1] += WM8960_SPKOUT_VOL_MIN;

        altered = ((vol[0] != ((wm8960->spk_volume[0]) & WM8960_SPKOUT_VOL_CTRL_MASK)) ||
                   (vol[1] != ((wm8960->spk_volume[1]) & WM8960_SPKOUT_VOL_CTRL_MASK)));

        if (altered) {
            /* We only apply the volume update bit when we write to the last channel
             * channel (right) so volume updates across all channels are synchronous
             */
            status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_L,
                                   WM8960_SPKOUT_VOL_CTRL_MASK, vol[0], WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R,
                                   WM8960_SPKOUT_VOL_CTRL_MASK | WM8960_SPKOUT_VOL_UPDATE_MASK,
                                   vol[1] | WM8960_SPKOUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R, WM8960_SPKOUT_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            /* Save volume */
            wm8960->spk_volume[0] = vol[0];
            wm8960->spk_volume[1] = vol[1];
        }
    } else {
        /* Get saved volume */
        vol[0] = (wm8960->spk_volume[0] & WM8960_SPKOUT_VOL_CTRL_MASK) - WM8960_SPKOUT_VOL_MIN;
        vol[1] = (wm8960->spk_volume[1] & WM8960_SPKOUT_VOL_CTRL_MASK) - WM8960_SPKOUT_VOL_MIN;
    }

    return (altered);
}

/**
 * Controls Headphone mute.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new mute will be set. When false mute status will be got.
 * @param vol Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_hp_mute_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                      uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        altered =
            (wm8960->hp_mute != (vol[0] & (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT))) ? 1 :
            0;

        if (altered) {
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                wm8960->hp_mute |= SND_MIXER_CHN_MASK_FRONT_LEFT;
            } else {
                wm8960->hp_mute &= ~SND_MIXER_CHN_MASK_FRONT_LEFT;
            }

            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                wm8960->hp_mute |= SND_MIXER_CHN_MASK_FRONT_RIGHT;
            } else {
                wm8960->hp_mute &= ~SND_MIXER_CHN_MASK_FRONT_RIGHT;
            }

            /* We only apply the volume update bit when we write to the last channel
             * channel (right) so mute updates across all channels are synchronous
             */
            if (wm8960->hp_mute & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                status = wm8960_update(wm8960, WM8960_HPOUT_VOL_L, WM8960_HPOUT_VOL_CTRL_MASK,
                                       WM8960_HPOUT_VOL_MUTE_VAL, WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_HPOUT_VOL_L, WM8960_HPOUT_VOL_CTRL_MASK,
                                       wm8960->hp_volume[0], WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
            if (wm8960->hp_mute & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                status = wm8960_update(wm8960, WM8960_HPOUT_VOL_R, WM8960_HPOUT_VOL_CTRL_MASK,
                                       WM8960_HPOUT_VOL_MUTE_VAL | WM8960_HPOUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_HPOUT_VOL_R, WM8960_HPOUT_VOL_CTRL_MASK,
                                       wm8960->hp_volume[1] | WM8960_HPOUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_HPOUT_VOL_R, WM8960_HPOUT_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
        }
    } else {
        vol[0] = wm8960->hp_mute;
    }

    return (altered);
}

/**
 * Controls Headphone volume.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new volume will be set. When false volume will be got.
 * @param vol Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_hp_vol_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                     uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        // This offset is used since spk_range was shifted to start from 0
        vol[0] += WM8960_HPOUT_VOL_MIN;
        vol[1] += WM8960_HPOUT_VOL_MIN;

        altered = ((vol[0] != ((wm8960->hp_volume[0]) & WM8960_HPOUT_VOL_CTRL_MASK))
                   || (vol[1] != ((wm8960->hp_volume[1]) & WM8960_HPOUT_VOL_CTRL_MASK)));

        if (altered) {
            /* Update Left and Right channel volume and set volume update bit */
            status = wm8960_update(wm8960, WM8960_HPOUT_VOL_L,
                                   WM8960_HPOUT_VOL_CTRL_MASK, vol[0], WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            status = wm8960_update(wm8960, WM8960_HPOUT_VOL_R,
                                   WM8960_HPOUT_VOL_CTRL_MASK | WM8960_HPOUT_VOL_UPDATE_MASK,
                                   vol[1] | WM8960_HPOUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_HPOUT_VOL_R, WM8960_HPOUT_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            /* Save volume */
            wm8960->hp_volume[0] = vol[0];
            wm8960->hp_volume[1] = vol[1];
        }
    } else {
        /* Get saved volume */
        vol[0] = (wm8960->hp_volume[0] & WM8960_HPOUT_VOL_CTRL_MASK) - WM8960_HPOUT_VOL_MIN;
        vol[1] = (wm8960->hp_volume[1] & WM8960_HPOUT_VOL_CTRL_MASK) - WM8960_HPOUT_VOL_MIN;
    }

    return (altered);
}

/**
 * Controls Master mute.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new mute will be set. When false mute state will be got.
 * @param vol Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case mute has been changed.
 */
static int32_t wm8960_dac_mute_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                       uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        altered =
            (wm8960->dac_mute != (vol[0] & (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT))) ? 1 :
            0;
        if (altered) {
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                wm8960->dac_mute |= SND_MIXER_CHN_MASK_FRONT_LEFT;
            } else {
                wm8960->dac_mute &= ~SND_MIXER_CHN_MASK_FRONT_LEFT;
            }

            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                wm8960->dac_mute |= SND_MIXER_CHN_MASK_FRONT_RIGHT;
            } else {
                wm8960->dac_mute &= ~SND_MIXER_CHN_MASK_FRONT_RIGHT;
            }

            /* Apply mute update */
            if (wm8960->dac_mute & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                status = wm8960_update(wm8960, WM8960_DAC_VOL_L, WM8960_DAC_VOL_CTRL_MASK,
                                       WM8960_DAC_VOL_MUTE_VAL, WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_DAC_VOL_L, WM8960_DAC_VOL_CTRL_MASK,
                                       wm8960->dac_volume[0], WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
            if (wm8960->dac_mute & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_CTRL_MASK,
                                       WM8960_DAC_VOL_MUTE_VAL | WM8960_DAC_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_CTRL_MASK,
                                       wm8960->dac_volume[1] | WM8960_DAC_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt(" mute update failed");
                return 0;
            }
        }
    } else {
        vol[0] = wm8960->dac_mute;
    }

    return (altered);
}

/**
 * Controls DAC volume.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new volume will be set. When false volume will be got.
 * @param vol Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_dac_vol_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                      uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        // This offset is used since dac_range was shifted to start from 0
        vol[0] += WM8960_DAC_VOL_MIN;
        vol[1] += WM8960_DAC_VOL_MIN;

        altered = ((vol[0] != ((wm8960->dac_volume[0]) & WM8960_DAC_VOL_CTRL_MASK))
                   || (vol[1] != ((wm8960->dac_volume[1]) & WM8960_DAC_VOL_CTRL_MASK)));

        if (altered) {
            /* Update Left and Right channel volume and set volume update bit */
            status = wm8960_update(wm8960, WM8960_DAC_VOL_L, WM8960_DAC_VOL_CTRL_MASK,
                                   vol[0], WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_CTRL_MASK | WM8960_DAC_VOL_UPDATE_MASK,
                                   vol[1] | WM8960_DAC_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            /* Clear volume update bit */
            status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("volume update failed");
                return 0;
            }
            wm8960->dac_volume[0] = vol[0];
            wm8960->dac_volume[1] = vol[1];
        }
    } else {
        /* Get saved volume */
        vol[0] = (wm8960->dac_volume[0] & WM8960_DAC_VOL_CTRL_MASK) - WM8960_DAC_VOL_MIN;
        vol[1] = (wm8960->dac_volume[1] & WM8960_DAC_VOL_CTRL_MASK) - WM8960_DAC_VOL_MIN;
    }

    return (altered);
}

/**
 * Controls ADC Mute.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new mute will be set. When false mute state will be got.
 * @param vol Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case mute has been changed.
 */
static int32_t wm8960_adc_mute_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                       uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        /* If we are Jointly-mute then translate mute status across all channels */
        if (wm8960->mic_left2right) {
            vol[0] = (vol[0] == 1) ? (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT) : 0;
        }
        altered = (wm8960->adc_mute != (vol[0] & (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT)) ? 1 : 0);
        if (altered) {
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                wm8960->adc_mute |= SND_MIXER_CHN_MASK_FRONT_LEFT;
            } else {
                wm8960->adc_mute &= ~SND_MIXER_CHN_MASK_FRONT_LEFT;
            }

            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                wm8960->adc_mute |= SND_MIXER_CHN_MASK_FRONT_RIGHT;
            } else {
                wm8960->adc_mute &= ~SND_MIXER_CHN_MASK_FRONT_RIGHT;
            }

            /* Apply mute update */
            if (wm8960->adc_mute & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                status = wm8960_update(wm8960, WM8960_ADC_L_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       WM8960_ADC_VOL_MUTE_VAL, WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_ADC_L_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       wm8960->adc_volume[0], WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
            if (wm8960->adc_mute & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                status = wm8960_update(wm8960, WM8960_ADC_R_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       WM8960_ADC_VOL_MUTE_VAL | WM8960_ADC_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            } else {
                // restore volume
                status = wm8960_update(wm8960, WM8960_ADC_R_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       wm8960->adc_volume[1] | WM8960_ADC_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            }
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Clear volume update bit since it is not sticky in the register */
            status = wm8960_update(wm8960, WM8960_ADC_R_VOL, WM8960_ADC_VOL_UPDATE_MASK, 0,
                                   WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
        }
    } else {
        vol[0] = wm8960->adc_mute;
    }

    return (altered);
}

/**
 * Controls ADC volume.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new volume will be set. When false volume will be got.
 * @param vol Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_adc_vol_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                      uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        // This offset is used since dac_range was shifted to start from 0
        if (wm8960->mic_left2right) {
            vol[0] += WM8960_ADC_VOL_MIN;
            altered = (vol[0] != ((wm8960->adc_volume[0]) & WM8960_ADC_VOL_CTRL_MASK));

            if (altered) {
                /* Update Left channel volume and set volume update bit */
                status = wm8960_update(wm8960, WM8960_ADC_L_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       vol[0] | WM8960_INPUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                /* Clear volume update bit */
                status = wm8960_update(wm8960, WM8960_ADC_L_VOL, WM8960_ADC_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                wm8960->adc_volume[0] = vol[0];
            }
        } else {
            // This offset is used since dac_range was shifted to start from 0
            vol[0] += WM8960_ADC_VOL_MIN;
            vol[1] += WM8960_ADC_VOL_MIN;

            altered = ((vol[0] != ((wm8960->adc_volume[0]) & WM8960_ADC_VOL_CTRL_MASK))
                       || (vol[1] != ((wm8960->adc_volume[1]) & WM8960_ADC_VOL_CTRL_MASK)));

            if (altered) {
                /* Update Left and Right channel volume and set volume update bit */
                status = wm8960_update(wm8960, WM8960_ADC_L_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                       vol[0], WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                status = wm8960_update(wm8960, WM8960_ADC_R_VOL, WM8960_ADC_VOL_CTRL_MASK,
                                   vol[1] | WM8960_ADC_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                /* Clear volume update bit */
                status = wm8960_update(wm8960, WM8960_ADC_R_VOL, WM8960_ADC_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                wm8960->adc_volume[0] = vol[0];
                wm8960->adc_volume[1] = vol[1];
            }
        }
    } else {
        /* Get saved volume */
        if (wm8960->mic_left2right) {
            vol[0] = (wm8960->adc_volume[0] & WM8960_ADC_VOL_CTRL_MASK) - WM8960_ADC_VOL_MIN;
        } else {
            vol[0] = (wm8960->adc_volume[0] & WM8960_ADC_VOL_CTRL_MASK) - WM8960_ADC_VOL_MIN;
            vol[1] = (wm8960->adc_volume[1] & WM8960_ADC_VOL_CTRL_MASK) - WM8960_ADC_VOL_MIN;
        }
    }

    return (altered);
}

/**
 * Controls Analog Input Mute.
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new mute will be set. When false mute state will be got.
 * @param vol Mute value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_input_mute_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                       uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        /* If we are Jointly-mute then translate mute status across all channels */
        if (wm8960->mic_left2right) {
            vol[0] = (vol[0] == 1) ? (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT) : 0;
        }
        altered = (wm8960->input_mute != (vol[0] & (SND_MIXER_CHN_MASK_FRONT_LEFT | SND_MIXER_CHN_MASK_FRONT_RIGHT))) ? 1 : 0;
        if (altered) {
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_LEFT) {
                wm8960->input_mute |= SND_MIXER_CHN_MASK_FRONT_LEFT;
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_L, WM8960_INPUT_VOL_MUTE_MASK,
                                       WM8960_INPUT_VOL_MUTE_MASK, WM8960_UPDATE_AND_WRITE_REG);
            } else {
                wm8960->input_mute &= ~SND_MIXER_CHN_MASK_FRONT_LEFT;
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_L, WM8960_INPUT_VOL_MUTE_MASK,
                                       0, WM8960_UPDATE_AND_WRITE_REG);
            }

            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Note: We only set the VOL_UPDATE_MASK bit when updating the Right channel value
             *       so that the Left and Right mute are applied synchronously
             */
            if (vol[0] & SND_MIXER_CHN_MASK_FRONT_RIGHT) {
                wm8960->input_mute |= SND_MIXER_CHN_MASK_FRONT_RIGHT;
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_R, WM8960_INPUT_VOL_MUTE_MASK,
                                       WM8960_INPUT_VOL_MUTE_MASK | WM8960_INPUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
            } else {
                wm8960->input_mute &= ~SND_MIXER_CHN_MASK_FRONT_RIGHT;
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_R,
                                       WM8960_INPUT_VOL_MUTE_MASK | WM8960_INPUT_VOL_UPDATE_MASK,
                                       WM8960_INPUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
            }

            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }

            /* Since the volume update bit is not sticky in the register we must clear our local copy of the bit */
            status = wm8960_update(wm8960, WM8960_INPUT_VOL_R, WM8960_INPUT_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY);
            if (status != EOK) {
                ado_error_fmt("mute update failed");
                return 0;
            }
        }
    } else {
        vol[0] = wm8960->input_mute;
    }

    return (altered);
}

/**
 * Controls Analog Input volume.
 *
 * @param wm8960 Mixer context.
 * @param element Pointer to a mixer element.
 * @param set When true a new volume will be set. When false volume will be got.
 * @param vol Volume value.
 * @param instance_data Driver instance data. This pointer is not used and shall be NULL.
 *
 * @return Returns 1 in case volume has been changed.
 */
static int32_t wm8960_input_vol_control(MIXER_CONTEXT_T * wm8960, ado_mixer_delement_t * element, uint8_t set,
                                      uint32_t * vol, void *instance_data)
{
    int32_t altered = 0;
    int status;

    if (set) {
        if (wm8960->mic_left2right) {
            // This offset is used since spk_range was shifted to start from 0
            vol[0] += WM8960_INPUT_VOL_MIN;
            altered = (vol[0] != (wm8960->input_volume[0] & WM8960_INPUT_VOL_CTRL_MASK));

            if (altered) {
                /* Update Left channel volume and set volume update bit */
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_L,
                                       WM8960_INPUT_VOL_CTRL_MASK | WM8960_INPUT_VOL_UPDATE_MASK, vol[0] | WM8960_INPUT_VOL_UPDATE_MASK,
                                       WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                /* Clear volume update bit */
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_L, WM8960_INPUT_VOL_UPDATE_MASK, 0,
                                       WM8960_UPDATE_ONLY);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                wm8960->input_volume[0] = vol[0];
            }
        } else {
            // This offset is used since spk_range was shifted to start from 0
            vol[0] += WM8960_INPUT_VOL_MIN;
            vol[1] += WM8960_INPUT_VOL_MIN;

            altered = ((vol[0] != (wm8960->input_volume[0] & WM8960_INPUT_VOL_CTRL_MASK)) ||
                       (vol[1] != (wm8960->input_volume[1] & WM8960_INPUT_VOL_CTRL_MASK)));

            if (altered) {
                /* Update Left and Right channel volume and set volume update bit */
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_L,
                                       WM8960_INPUT_VOL_CTRL_MASK | WM8960_INPUT_VOL_UPDATE_MASK, vol[0],
                                       WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_R,
                                       WM8960_INPUT_VOL_CTRL_MASK | WM8960_INPUT_VOL_UPDATE_MASK,
                                       vol[1] | WM8960_INPUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                /* Clear volume update bit */
                status = wm8960_update(wm8960, WM8960_INPUT_VOL_R, WM8960_INPUT_VOL_UPDATE_MASK, 0,
                                       WM8960_UPDATE_ONLY);
                if (status != EOK) {
                    ado_error_fmt("volume update failed");
                    return 0;
                }
                wm8960->input_volume[0] = vol[0];
                wm8960->input_volume[1] = vol[1];
            }
        }
    } else {
        /* Get saved volume */
        if (wm8960->mic_left2right) {
            vol[0] = (wm8960->input_volume[0] & WM8960_INPUT_VOL_CTRL_MASK) - WM8960_INPUT_VOL_MIN;
        } else {
            vol[0] = (wm8960->input_volume[0] & WM8960_INPUT_VOL_CTRL_MASK) - WM8960_INPUT_VOL_MIN;
            vol[1] = (wm8960->input_volume[1] & WM8960_INPUT_VOL_CTRL_MASK) - WM8960_INPUT_VOL_MIN;
        }
    }

    return (altered);
}
/**
 * Determine if the mic-bias is enabled.
 *
 * @param wm8960 Mixer context.
 * @param dswitch Opaque parameter.
 * @param cswitch Opaque parameter.
 * @param instance_data Optional data. By default NULL.
 *
 * @return Execution status.
 */
static int32_t wm8960_micbias_get(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t data;

    data = wm8960->regs[WM8960_PWR_MGMT_1] & WM8960_PWR_MGMT_1_MICB_MASK;
    cswitch->type = SND_SW_TYPE_BOOLEAN;
    cswitch->value.enable = data ? 1 : 0;

    return EOK;
}
/**
 * Set the micbias to a new value if that has been requested.
 *
 * @param wm8960 Mixer context.
 * @param dswitch Opaque parameter.
 * @param cswitch Opaque parameter.
 * @param instance_data Optional data. By default NULL.
 *
 * @return Returns 1 when altered.
 */
static int32_t wm8960_micbias_set(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t bias;
    int32_t altered = 0;
    int status;

    bias = wm8960->regs[WM8960_PWR_MGMT_1] & WM8960_PWR_MGMT_1_MICB_MASK;
    altered = (cswitch->value.enable != (bias ? 1 : 0));
    if (altered) {
        status = wm8960_update(wm8960, WM8960_PWR_MGMT_1, WM8960_PWR_MGMT_1_MICB_MASK,
                               cswitch->value.enable ? WM8960_PWR_MGMT_1_MICB_MASK : 0, WM8960_UPDATE_AND_WRITE_REG);

        if (status != EOK) {
            ado_error_fmt("micbias update failed");
            return 0;
        }
    }
    return (altered);
}

static int32_t wm8960_loopback_get(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t data;

    data = wm8960->regs[WM8960_AUDIO_INTERF_1] & WM8960_AUDIO_INTERF_1_LOOPBACK;
    cswitch->type = SND_SW_TYPE_BOOLEAN;
    cswitch->value.enable = data ? 1 : 0;

    return EOK;
}

static int32_t wm8960_loopback_set(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t loopback;
    int32_t altered = 0;
    int status;

    loopback = wm8960->regs[WM8960_AUDIO_INTERF_1] & WM8960_AUDIO_INTERF_1_LOOPBACK;
    altered = (cswitch->value.enable != (loopback ? 1 : 0));
    if (altered) {
        status = wm8960_update(wm8960, WM8960_AUDIO_INTERF_1, WM8960_AUDIO_INTERF_1_LOOPBACK,
                               cswitch->value.enable ? WM8960_AUDIO_INTERF_1_LOOPBACK : 0, WM8960_UPDATE_AND_WRITE_REG);

        if (status != EOK) {
            ado_error_fmt("loopback update failed");
            return 0;
        }
    }
    return (altered);
}

static int32_t wm8960_analog_loopback_get(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t data;

    data = wm8960->regs[WM8960_BYPASS_L] & WM8960_BYPASS_LB2LO;
    cswitch->type = SND_SW_TYPE_BOOLEAN;
    cswitch->value.enable = data ? 1 : 0;

    return EOK;
}

static int32_t wm8960_analog_loopback_set(MIXER_CONTEXT_T * wm8960, ado_dswitch_t * dswitch, snd_switch_t * cswitch,
                                  void *instance_data)
{
    uint32_t loopback;
    int32_t altered = 0;
    int status;

    loopback = wm8960->regs[WM8960_BYPASS_L] & WM8960_BYPASS_LB2LO;
    altered = (cswitch->value.enable != (loopback ? 1 : 0));
    if (altered) {
        status = wm8960_update(wm8960, WM8960_BYPASS_L, WM8960_BYPASS_LB2LO,
                               cswitch->value.enable ? WM8960_BYPASS_LB2LO : 0, WM8960_UPDATE_AND_WRITE_REG);
        if (status != EOK) {
            ado_error_fmt("Left loopback update failed");
            return 0;
        }
        status = wm8960_update(wm8960, WM8960_BYPASS_R, WM8960_BYPASS_RB2RO,
                               cswitch->value.enable ? WM8960_BYPASS_RB2RO : 0, WM8960_UPDATE_AND_WRITE_REG);
        if (status != EOK) {
            ado_error_fmt("Right loopback update failed");
            return 0;
        }
    }
    return (altered);
}

/**
 * Resets WM8960 codec
 *
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
static int wm8960_reset(MIXER_CONTEXT_T * wm8960)
{
    int status;
    /* Data written to reset registers isn't really important. */
    status = wm8960_write(wm8960, WM8960_SOFTWARE_RESET, 0);
    if (status != EOK) {
        ado_error_fmt("codec reset failed");
        return status;
    }
    /* Initialize register buffer to default reset values from RM */
    wm8960->regs[WM8960_INPUT_VOL_L] = WM8960_INPUT_VOL_DEFAULT; // default volume
    wm8960->regs[WM8960_INPUT_VOL_R] = WM8960_INPUT_VOL_DEFAULT; // default volume
    wm8960->input_volume[0] = WM8960_INPUT_VOL_DEFAULT;
    wm8960->input_volume[1] = WM8960_INPUT_VOL_DEFAULT;

    wm8960->regs[WM8960_HPOUT_VOL_L] = 0;
    wm8960->regs[WM8960_HPOUT_VOL_R] = 0;
    wm8960->hp_volume[0] = 0;
    wm8960->hp_volume[1] = 0;

    wm8960->regs[WM8960_CLOCKING_1] = 0;
    wm8960->regs[WM8960_ADCDAC_CTL_1] = WM8960_ADCDAC_CTL_1_DACMUTE_MASK;
    wm8960->regs[WM8960_ADCDAC_CTL_2] = 0;
    wm8960->regs[WM8960_AUDIO_INTERF_0] = WM8960_AUDIO_INTERF_0_WL_24 | WM8960_AUDIO_INTERF_0_FORMAT_I2S;
    wm8960->regs[WM8960_CLOCKING_2] = WM8960_CLOCKING_2_DCLKDIV_MASK;
    wm8960->regs[WM8960_AUDIO_INTERF_1] = 0;

    /* Mute DAC Output */
    if ((status = wm8960_write(wm8960, WM8960_DAC_VOL_L, 0)) != EOK) {
        return status;
    }
    if ((status = wm8960_write(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_UPDATE_MASK | 0)) != EOK) {
        return status;
    }
    /* The VOL_UPDATE bit is not sticky, so clear it in our local copy */
    if ((status = wm8960_update(wm8960, WM8960_DAC_VOL_R, WM8960_DAC_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY)) != EOK) {
        return status;
    }

    wm8960->regs[WM8960_DAC_3D_CTL] = 0;
    wm8960->regs[WM8960_ALC_CTL_1] = WM8960_ALC_CTL_1_DEFAULT;
    wm8960->regs[WM8960_ALC_CTL_2] = 0;
    wm8960->regs[WM8960_ALC_CTL_3] = WM8960_ALC_CTL_3_DECAY_192MS | WM8960_ALC_CTL_3_ATTACK_24MS;
    wm8960->regs[WM8960_NOISE_GATE] = 0;

    wm8960->regs[WM8960_ADC_L_VOL] = WM8960_ADC_VOL_DEFAULT;  // default volume
    wm8960->regs[WM8960_ADC_R_VOL] = WM8960_ADC_VOL_DEFAULT;  //default volume
    wm8960->adc_volume[0] = WM8960_ADC_VOL_DEFAULT;
    wm8960->adc_volume[1] = WM8960_ADC_VOL_DEFAULT;

    wm8960->regs[WM8960_ADDITIONAL_CTRL_1] = WM8960_ADDITIONAL_CTRL_1_TSDEN_MASK | WM8960_ADDITIONAL_CTRL_1_VSEL_MASK;
    wm8960->regs[WM8960_ADDITIONAL_CTRL_2] = 0;
    wm8960->regs[WM8960_PWR_MGMT_1] = 0;
    wm8960->regs[WM8960_PWR_MGMT_2] = 0;
    wm8960->regs[WM8960_ADDITIONAL_CTRL_3] = 0;
    wm8960->regs[WM8960_ANTI_POP_1] = 0;
    wm8960->regs[WM8960_ANTI_POP_2] = 0;
    wm8960->regs[WM8960_ADC_L_SIG_PATH] = WM8960_ADC_L_SIG_PATH_LMN1_MASK;
    wm8960->regs[WM8960_ADC_R_SIG_PATH] = WM8960_ADC_R_SIG_PATH_RMN1_MASK;
    wm8960->regs[WM8960_OUT_L_MIX] = WM8960_OUT_MIX_DEFAULT; // default volume
    wm8960->regs[WM8960_OUT_R_MIX] = WM8960_OUT_MIX_DEFAULT; // default volume
    wm8960->regs[WM8960_MONO_OUT_MIX_1] = 0;
    wm8960->regs[WM8960_MONO_OUT_MIX_2] = 0;
    wm8960->regs[WM8960_SPKOUT_VOL_L] = 0;
    wm8960->regs[WM8960_SPKOUT_VOL_R] = 0;
    wm8960->spk_volume[0] = 0;
    wm8960->spk_volume[1] = 0;

    /* Mute SPK Output */
    if ((status = wm8960_write(wm8960, WM8960_SPKOUT_VOL_L, 0)) != EOK) {
        return status;
    }
    if ((status = wm8960_write(wm8960, WM8960_SPKOUT_VOL_L, WM8960_SPKOUT_VOL_UPDATE_MASK | 0)) != EOK) {
        return status;
    }
    /* The VOL_UPDATE bit is not sticky, so clear it in our local copy */
    if ((status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_L, WM8960_SPKOUT_VOL_UPDATE_MASK, 0, WM8960_UPDATE_ONLY)) != EOK) {
        return status;
    }

    wm8960->regs[WM8960_OUT_3_VOL] = WM8960_OUT_3_VOL_MOUTVOL_MASK;
    wm8960->regs[WM8960_INPUT_L_BOOST_MIX] = 0;
    wm8960->regs[WM8960_INPUT_R_BOOST_MIX] = 0;
    wm8960->regs[WM8960_BYPASS_L] = WM8960_BYPASS_DEFAULT;    // default volume
    wm8960->regs[WM8960_BYPASS_R] = WM8960_BYPASS_DEFAULT;    // default volume
    wm8960->regs[WM8960_PWR_MGMT_3] = 0;
    wm8960->regs[WM8960_ADDITIONAL_CTRL_4] = WM8960_ADDITIONAL_CTRL_4_TSENSEN_MASK;
    wm8960->regs[WM8960_CLASSD_CTRL_1] = WM8960_CLASSD_CTRL_1_DEFAULT; // Reserved bits 5:0
    wm8960->regs[WM8960_CLASSD_CTRL_2] = WM8960_CLASSD_CTRL_2_DEFAULT; // Reserved bits 8:6
    wm8960->regs[WM8960_PLL_1] = WM8960_PLL_1_PLLN_DEFAULT;
    wm8960->regs[WM8960_PLL_2] = WM8960_PLL_2_PLLK_H_DEFAULT;
    wm8960->regs[WM8960_PLL_3] = WM8960_PLL_3_PLLK_L_DEFAULT;
    wm8960->regs[WM8960_PLL_4] = WM8960_PLL_4_PLLK_LL_DEFAULT;

    /* Set default codec sample. Driver will overwrite it later */
    wm8960->sample_rate = 48000;
    return (EOK);
}

/**
 * Initializes WM8960 subsystems like power, gains, volume, mute etc.
 *
 * @param mx Audio driver context.
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
static int wm8960_init_subsystems(MIXER_CONTEXT_T * wm8960)
{
    uint32_t vol[2];
    int status;
    uint32_t val;

    /* Master Clock enabled */
    val = 0;
    status = wm8960_write(wm8960, WM8960_PWR_MGMT_1, val);
    if (status != EOK) {
        return status;
    }
    delay(1); /* Ensure 1ms delay between enable of master clock and everything else */

    /* Set Class D amp AC and DC Gain */
#define WM8960_GAIN (WM8960_SPKVDD * 100) / WM8960_AVDD

#if WM8960_GAIN >= 180
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1P80 | WM8960_CLASSD_CTRL_2_DCGAIN_1P80
#elif WM8960_GAIN >= 167
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1P67 | WM8960_CLASSD_CTRL_2_DCGAIN_1P67
#elif WM8960_GAIN >= 152
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1P52 | WM8960_CLASSD_CTRL_2_DCGAIN_1P52
#elif WM8960_GAIN >= 140
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1P40 | WM8960_CLASSD_CTRL_2_DCGAIN_1P40
#elif WM8960_GAIN >= 127
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1P27 | WM8960_CLASSD_CTRL_2_DCGAIN_1P27
#else
#define WM8960_GAIN_VAL WM8960_CLASSD_CTRL_2_ACGAIN_1 | WM8960_CLASSD_CTRL_2_DCGAIN_1
#endif

    /* Anti-Pop */
    val = ANTI_POP_1_POBCTRL | ANTI_POP_1_BUFDCOPEN | ANTI_POP_1_BUFIOEN | ANTI_POP_1_SOFT_ST;
    status = wm8960_write(wm8960, WM8960_ANTI_POP_1, val);
    if (status != EOK) {
        return status;
    }

    /* Power on VMID */
    val = WM8960_PWR_MGMT_1_VMIDSEL_50k | WM8960_PWR_MGMT_1_AINL_MASK | WM8960_PWR_MGMT_1_AINR_MASK | WM8960_PWR_MGMT_1_ADCL_MASK | WM8960_PWR_MGMT_1_ADCR_MASK;
    if (wm8960->micbias) {
        val |= WM8960_PWR_MGMT_1_MICB_MASK;
    }
    status = wm8960_update(wm8960, WM8960_PWR_MGMT_1, val, val, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
    /* Delay for VMID ramp (POP suppression) */
    delay(700);

    /* Enable VREF */
    val = WM8960_PWR_MGMT_1_VREF_MASK;
    status = wm8960_update(wm8960, WM8960_PWR_MGMT_1, val, val, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    status = wm8960_update(wm8960, WM8960_CLASSD_CTRL_2,
                           WM8960_CLASSD_CTRL_2_DCGAIN_MASK | WM8960_CLASSD_CTRL_2_ACGAIN_MASK, WM8960_GAIN_VAL,
                           WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    /* Optimize Analog bias */
#if WM8960_AVDD <= WM8960_ANALOG_BIAS_THRESH
    status = wm8960_update(wm8960, WM8960_ADDITIONAL_CTRL_1,
                  WM8960_ADDITIONAL_CTRL_1_VSEL_MASK,
                  WM8960_ADDITIONAL_CTRL_1_VSEL_HBIAS,
                  WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
#endif

    /* enable Input PGA and Output Mixer */
    status = wm8960_write(
                 wm8960, WM8960_PWR_MGMT_3,
                 WM8960_PWR_MGMT_3_LMIC | WM8960_PWR_MGMT_3_RMIC | WM8960_PWR_MGMT_3_LOMIX | WM8960_PWR_MGMT_3_ROMIX);
    if (status != EOK) {
        return status;
    }

    /* Set routing on left and right output mix and default volume 0 dB */
    status = wm8960_write(wm8960, WM8960_OUT_L_MIX, WM8960_OUT_MIX_DAC2MIX);
    if (status != EOK) {
        return status;
    }
    status = wm8960_write(wm8960, WM8960_OUT_R_MIX, WM8960_OUT_MIX_DAC2MIX);
    if (status != EOK) {
        return status;
    }
    /* Enable Zero crossing and unmute Headphone out, set 0 dB */
    status = wm8960_write(wm8960, WM8960_HPOUT_VOL_L, WM8960_HPOUT_VOL_ZERO_MASK);
    if (status != EOK) {
        return status;
    }
    status = wm8960_write(wm8960, WM8960_HPOUT_VOL_R, WM8960_HPOUT_VOL_ZERO_MASK);
    if (status != EOK) {
        return status;
    }
    wm8960->hp_mute = 0;

    /* Set volume to 0 dB */
    vol[0] = WM8960_DAC_VOLUME_DEFAULT - WM8960_DAC_VOL_MIN;
    vol[1] = WM8960_DAC_VOLUME_DEFAULT - WM8960_DAC_VOL_MIN;
    wm8960_dac_vol_control(wm8960, NULL, 1, vol, NULL);

    /* Set volume to 0 dB. Step is 1 dB */
    vol[0] = hp_range[0].max - (hp_range[0].max_dB / 100);
    vol[1] = hp_range[1].max - (hp_range[1].max_dB / 100);
    wm8960_hp_vol_control(wm8960, NULL, 1, vol, NULL);

    /* Unmute DAC */
    status = wm8960_write(wm8960, WM8960_ADCDAC_CTL_1, 0);
    if (status != EOK) {
        return status;
    }
    wm8960->dac_mute = 0;

    /* Configure ADC left signal path
     * Connect LINPUT1 to the left input PGA
     * Connect Left input PGA to the Left input boost mixer
     */
    status = wm8960_write(wm8960, WM8960_ADC_L_SIG_PATH, WM8960_ADC_L_SIG_PATH_LMN1_MASK | WM8960_ADC_L_LMIC2B_MASK);
    if (status != EOK) {
        return status;
    }
    /* Un-mute and enable Zero Cross, volume to 0dB */
    status = wm8960_update(wm8960, WM8960_INPUT_VOL_L, WM8960_INPUT_VOL_ZERO_MASK | WM8960_INPUT_VOL_MUTE_MASK,
                           WM8960_INPUT_VOL_ZERO_MASK | WM8960_INPUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
    status = wm8960_update(wm8960, WM8960_INPUT_VOL_R, WM8960_INPUT_VOL_ZERO_MASK | WM8960_INPUT_VOL_MUTE_MASK,
                           WM8960_INPUT_VOL_ZERO_MASK | WM8960_INPUT_VOL_UPDATE_MASK, WM8960_UPDATE_AND_WRITE_REG);

    if (status != EOK) {
        return status;
    }
    wm8960->input_mute = 0;
    wm8960->adc_mute = 0;

    if (wm8960->mic_left2right) {
        /* Right input is not used */
        status = wm8960_write(wm8960, WM8960_ADC_R_SIG_PATH, 0);
        if (status != EOK) {
            return status;
        }
        /* Route the Left ADC output to both left and right */
        status = wm8960_update(wm8960, WM8960_ADDITIONAL_CTRL_1, WM8960_ADDITIONAL_CTRL_1_DATSEL_MASK,
                               WM8960_ADDITIONAL_CTRL_1_DATSEL_LEFT_LEFT, WM8960_UPDATE_AND_WRITE_REG);
        if (status != EOK) {
            return status;
        }
    }

    /* Set up Headphone detect debounce */
    status = wm8960_update(wm8960, WM8960_ADDITIONAL_CTRL_1, WM8960_ADDITIONAL_CTRL_1_TOEN_MASK,
                           WM8960_ADDITIONAL_CTRL_1_TOEN_MASK, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    /* Enable Capless mode headphone switch */
    status = wm8960_write(wm8960, WM8960_ADDITIONAL_CTRL_3, OUT3CAP);

    /* Set up Headphone detect to JD2 */
    status = wm8960_update(wm8960, WM8960_ADDITIONAL_CTRL_4, WM8960_ADDITIONAL_CTRL_4_HPSEL_MASK | WM8960_ADDITIONAL_CTRL_4_MBSEL,
                           WM8960_ADDITIONAL_CTRL_4_HPSEL_JD2 | WM8960_ADDITIONAL_CTRL_4_MBSEL, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
    /* HP detect pin is high with headphone jack connected */
    status = wm8960_update(wm8960, WM8960_ADDITIONAL_CTRL_2,
                           WM8960_ADDITIONAL_CTRL_2_HPSWEN_MASK | WM8960_ADDITIONAL_CTRL_2_HPSWPOL_MASK,
                           WM8960_ADDITIONAL_CTRL_2_HPSWEN_MASK, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    /* Enable Class D amp */
    status = wm8960_update(wm8960, WM8960_CLASSD_CTRL_1, WM8960_CLASSD_CTRL_1_SPKEN_MASK,
                           WM8960_CLASSD_CTRL_1_SPKEN_MASK, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    /* Enable Zero cross for Speakers */
    status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_L, WM8960_SPKOUT_VOL_ZERO_MASK, WM8960_SPKOUT_VOL_ZERO_MASK,
                           WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
    status = wm8960_update(wm8960, WM8960_SPKOUT_VOL_R, WM8960_SPKOUT_VOL_ZERO_MASK, WM8960_SPKOUT_VOL_ZERO_MASK,
                           WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }
    /* Set volume to 0 dB. Step is 1 dB */
    vol[0] = spk_range[0].max - (spk_range[0].max_dB / 100);
    vol[1] = spk_range[1].max - (spk_range[1].max_dB / 100);
    /* Unmute Speakers */
    wm8960_spk_vol_control(wm8960, NULL, 1, vol, NULL);
    wm8960->spk_mute = 0;

    /* Power on DACL, DACR, LOUT, ROUT, SPKL, SPKR */
    val = WM8960_PWR_MGMT_2_DACL | WM8960_PWR_MGMT_2_DACR | WM8960_PWR_MGMT_2_LOUT1 | WM8960_PWR_MGMT_2_ROUT1 | WM8960_PWR_MGMT_2_SPKL | WM8960_PWR_MGMT_2_SPKR;
    status = wm8960_update(wm8960, WM8960_PWR_MGMT_2, val, val, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    /* Disable Anti-Pop */
    val = ANTI_POP_1_POBCTRL | ANTI_POP_1_BUFDCOPEN | ANTI_POP_1_SOFT_ST;
    status = wm8960_update(wm8960, WM8960_ANTI_POP_1, val, 0, WM8960_UPDATE_AND_WRITE_REG);
    if (status != EOK) {
        return status;
    }

    return EOK;
}

/**
 * Initializes WM8960 audio data I2S interface.
 *
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
static int wm8960_init_interface(MIXER_CONTEXT_T *wm8960)
{
    int status;
    /* Set interface format, configure as Master or Slave */
    status = wm8960_write(wm8960, WM8960_AUDIO_INTERF_0,
                          ((wm8960->format == WM8960_I2S) ? WM8960_AUDIO_INTERF_0_FORMAT_I2S : WM8960_AUDIO_INTERF_0_FORMAT_DSP) |
                          ((wm8960->clk_mode == WM8960_MASTER) ? WM8960_AUDIO_INTERF_0_MS_MASK : 0));
    if (status != EOK) {
        return status;
    }

    /* Set sample size */
    switch (wm8960->sample_size) {
        case 16:
            status = wm8960_update(wm8960, WM8960_AUDIO_INTERF_0, WM8960_AUDIO_INTERF_0_WL_MASK,
                                   WM8960_AUDIO_INTERF_0_WL_16, WM8960_UPDATE_AND_WRITE_REG);
            break;
        case 24:
            status = wm8960_update(wm8960, WM8960_AUDIO_INTERF_0, WM8960_AUDIO_INTERF_0_WL_MASK,
                                   WM8960_AUDIO_INTERF_0_WL_24, WM8960_UPDATE_AND_WRITE_REG);
            break;
        case 32:
        default:
            status = wm8960_update(wm8960, WM8960_AUDIO_INTERF_0, WM8960_AUDIO_INTERF_0_WL_MASK,
                                   WM8960_AUDIO_INTERF_0_WL_32, WM8960_UPDATE_AND_WRITE_REG);
            break;
    }
    if (status != EOK) {
        return status;
    }

    /* Use DAC LRCK pin for ADC */
    if (wm8960->use_dac_lrck) {
        status = wm8960_update(wm8960, WM8960_AUDIO_INTERF_1, WM8960_AUDIO_INTERF_1_ALRCGPIO_MASK,
                               WM8960_AUDIO_INTERF_1_ALRCGPIO, WM8960_UPDATE_AND_WRITE_REG);
        if (status != EOK) {
            return status;
        }
    }

    return EOK;
}

/**
 * Calculates WM8960 ADC and DAC divider according to required sample rate.
 *
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
int wm8960_set_rate(MIXER_CONTEXT_T * wm8960, uint32_t sample_rate)
{

    int err = 0;
    int best_err = 0;
    uint32_t best_div;
    uint32_t div;
    uint32_t begin;
    int status = EOK;
    uint32_t sysclk_div = 0;
    uint32_t bclk_sample_size;
    uint32_t target_sysclk;

    wm8960->sample_rate = sample_rate;

    if (wm8960->sample_rate == 0 || wm8960->mclk == 0) {
        return EINVAL;
    }

    /* sysclk should be as close as possible to wm8960->sample_rate * 256 */
    target_sysclk = wm8960->sample_rate * 256;
    if (abs(wm8960->mclk - target_sysclk) <= abs((wm8960->mclk/2) - target_sysclk)) {
        wm8960->sysclk = wm8960->mclk;
    } else {
        /* sysclk can divide only by 2 */
        wm8960->sysclk = wm8960->mclk / 2;
        sysclk_div = 2 << 1;
    }

    div = wm8960->sysclk / wm8960->sample_rate / 256;

    // calculate more accurately
    if (div) {
        div *= 10;
    } else {
        /* div can't be zero */
        ado_error_fmt("Wrong div %u", div);
        return EINVAL;
    }

    /* Start with smaller div */
    if (div > 10) {
        div -= 10;
    }

    begin = div;
    best_div = div;
    do {

        err = wm8960->sample_rate - ((wm8960->sysclk * 10) / (div * 256));

        if (abs(err) <= abs(best_err)) {
            best_err = err;
            best_div = div;
        }

        if ((div == 10) || (div == 40) || (div % 10)) {
            div += 5;
        } else {
            div += 10;
        }

    } while (div <= (begin + 20) && (div <= 60));

    ado_debug(DB_LVL_MIXER, "calculated div=%u.%u with err=%i", best_div / 10, best_div % 10, best_err);

    switch (best_div) {
        case 10:
            /* for div = 1.0 writes 0 to the register */
            best_div -= 10;
            /* fallthrough */
        case 20: /* div = 2.0 writes 2 */
        case 30: /* div = 3.0 writes 3 */
        case 40: /* div = 4.0 writes 4 */
        case 60: /* div = 6.0 writes 6 */
            if (best_div) {
                best_div /= 10;
            }
            status = wm8960_update(wm8960, WM8960_CLOCKING_1, WM8960_CLOCKING_1_ADCDIV_MASK | WM8960_CLOCKING_1_DACDIV_MASK | WM8960_CLOCKING_1_SYSCLKDIV_MASK,
                                   best_div << WM8960_CLOCKING_1_ADCDIV_SHIFT | best_div << WM8960_CLOCKING_1_DACDIV_SHIFT | sysclk_div,
                                   WM8960_UPDATE_AND_WRITE_REG);
            break;
        case 15: /* div = 1.5 writes 1 */
        case 55: /* div = 5.5 writes 5 */
            best_div -= 5;
            best_div /= 10;
            status = wm8960_update(wm8960, WM8960_CLOCKING_1, WM8960_CLOCKING_1_ADCDIV_MASK | WM8960_CLOCKING_1_DACDIV_MASK | WM8960_CLOCKING_1_SYSCLKDIV_MASK,
                                   best_div << WM8960_CLOCKING_1_ADCDIV_SHIFT | best_div << WM8960_CLOCKING_1_DACDIV_SHIFT | sysclk_div,
                                   WM8960_UPDATE_AND_WRITE_REG);
            break;
        default:
            ado_error_fmt("Wrong div %u.%u", best_div / 10, best_div % 10);
            return EINVAL;
            break;
    }

    if (wm8960->clk_mode == WM8960_MASTER) {
        /* BCLK for Master mode */
        bclk_sample_size = wm8960->sample_size;

        div = (wm8960->sysclk * 10) / (wm8960->sample_rate * bclk_sample_size * 2);
        switch (div) {
            case 0:
                break;
            case 15:
            case 20:
            case 30:
            case 40:
            case 60:
                div = div / 10;
                break;
            case 80:
                div = 7;
                break;
            case 110:
                div = 8;
                break;
            case 120:
                div = 9;
                break;
            case 160:
                div = 10;
                break;
            case 220:
                div = 11;
                break;
            case 240:
                div = 12;
                break;
            case 320:
                div = 0xff;
                break;
            default:
                if (div > 40 && div < 60) {
                    div = 5;
                }
                break;
        }
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_BCLKDIV_MASK, div, WM8960_UPDATE_AND_WRITE_REG);
    }
    return status;
}

/**
 * Initializes WM8960 internal clocks.
 *
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
static int wm8960_init_clocks(MIXER_CONTEXT_T * wm8960)
{

    uint32_t div;
    int status;

    status = wm8960_set_rate(wm8960, wm8960->sample_rate);

    if (status != EOK) {
        return status;
    }

    /* Calculate Class D amp divider to divide SYSCLK to 700 - 800 kHz */
    div = wm8960->sysclk / 80000;

    if (div > 160) {
        ado_error_fmt("Wrong div %u", div);
        return EINVAL;
    } else if (div > 120) {
        /* set div to 16 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_16,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 16;
    } else if (div > 80) {
        /* set div to 12 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_12,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 12;
    } else if (div > 60) {
        /* set div to 8 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_8,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 8;
    } else if (div > 40) {
        /* set div to 6 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_6,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 6;
    } else if (div > 30) {
        /* set div to 4 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_4,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 4;
    } else if (div > 20) {
        /* set div to 3 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_3,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 3;
    } else {
        /* set div to 2 */
        status = wm8960_update(wm8960, WM8960_CLOCKING_2, WM8960_CLOCKING_2_DCLKDIV_MASK, WM8960_CLOCKING_2_DCLKDIV_2,
                           WM8960_UPDATE_AND_WRITE_REG);
        div = 2;
    }

    if (status != EOK) {
        return status;
    }

    /* Double check Class D amp. calculated clock */
    div = wm8960->sysclk / div;

    if ((div < 700000U) || (div > 800000U)) {
        ado_debug(DB_LVL_MIXER, "Warning Class D amp. clock is %u", div);
    }

    return EOK;
}

/**
 * Destroys WM8960 context and Power downs the codec.
 *
 * @param wm8960 Mixer context.
 *
 * @return Execution status.
 */
static int wm8960_destroy(MIXER_CONTEXT_T * wm8960)
{
    uint32_t val;

    /* Anti-Pop */
    val = ANTI_POP_1_POBCTRL | ANTI_POP_1_BUFDCOPEN | ANTI_POP_1_SOFT_ST;
    wm8960_write(wm8960, WM8960_ANTI_POP_1, val);

    /* Power down analog Outputs */
    wm8960_write(wm8960, WM8960_PWR_MGMT_2, 0);

    /* Disable VREF */
    wm8960_update(wm8960, WM8960_PWR_MGMT_1, WM8960_PWR_MGMT_1_VREF_MASK, 0, WM8960_UPDATE_AND_WRITE_REG);
    delay(100);

    /* Disable Anti Pop */
    wm8960_write(wm8960, WM8960_ANTI_POP_1, ANTI_POP_1_BUFIOEN);

    /* Power down the rest of the codec */
    wm8960_write(wm8960, WM8960_PWR_MGMT_1, 0);
    wm8960_write(wm8960, WM8960_CLASSD_CTRL_1, 0);
    wm8960_write(wm8960, WM8960_PWR_MGMT_3, 0);

    /* disconnect connect between SoC and codec */
    if (wm8960->i2c_fd != -1) {
        close(wm8960->i2c_fd);
    }

    ado_debug(DB_LVL_MIXER, "Destroying WM8960 mixer");
    ado_free(wm8960);

    return (0);
}

/**
 * Builds ado mixer according to WM8960 capabilities.
 *
 * @param wm8960 Mixer context.
 * @param mixer A pointer to the ado_mixer_t structure for the mixer.
 *
 * @return Execution status.
 */
int32_t build_wm8960_mixer(MIXER_CONTEXT_T * wm8960, ado_mixer_t * mixer)
{
    /* Speaker Output elements */
    ado_mixer_delement_t *spk_mute = NULL, *spk_vol = NULL, *spk_io = NULL, *spk_pcm = NULL;

    /* headphone Output elements */
    ado_mixer_delement_t *hp_mute = NULL, *hp_vol = NULL, *hp_io = NULL;

    /* DAC Master Output elements */
    ado_mixer_delement_t *master_mute = NULL, *master_vol = NULL, *master_io = NULL;

    /* Input elements */
    ado_mixer_delement_t *input_vol = NULL, *input_mute = NULL, *adc_mute = NULL, *adc_vol = NULL, *adc_pcm = NULL, *adc_io = NULL;

    /* ################# */
    /*  PLAYBACK GROUPS  */
    /* ################# */
    /* WM8960: Line Out Output */
    if ((spk_pcm = ado_mixer_element_pcm1(mixer, SND_MIXER_ELEMENT_PLAYBACK, SND_MIXER_ETYPE_PLAYBACK1, 1, &pcm_devices[0])) == NULL) {
        return (-1);
    }

    if ((spk_mute = ado_mixer_element_sw1(mixer, "Speaker Mute", 2, wm8960_spk_mute_control, NULL, NULL)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, spk_pcm, spk_mute) != 0) {
        return (-1);
    }

    if ((spk_vol = ado_mixer_element_volume1(mixer, "Speaker Volume", 2, spk_range, wm8960_spk_vol_control, NULL, NULL)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, spk_mute, spk_vol) != 0) {
        return (-1);
    }

    if ((spk_io = ado_mixer_element_io(mixer, "Speaker Out", SND_MIXER_ETYPE_OUTPUT, 0, 2, stereo_voices)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, spk_vol, spk_io) != 0) {
        return (-1);
    }

    if (ado_mixer_playback_group_create(mixer, SND_MIXER_FRONT_OUT, SND_MIXER_CHN_MASK_STEREO, spk_vol, spk_mute) == NULL) {
        return (-1);
    }

    /* WM8960: Headphone Output */
    if (ado_mixer_element_pcm1(mixer, SND_MIXER_ELEMENT_PLAYBACK, SND_MIXER_ETYPE_PLAYBACK1, 1, &pcm_devices[0]) == NULL) {
        return (-1);
    }

    if ((hp_mute = ado_mixer_element_sw1(mixer, "HP Mute", 2, wm8960_hp_mute_control, NULL, NULL)) == NULL) {
        return (-1);
    }

    if ((hp_vol = ado_mixer_element_volume1(mixer, "HP Volume", 2, hp_range, wm8960_hp_vol_control, NULL, NULL))
        == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, hp_mute, hp_vol) != 0) {
        return (-1);
    }

    if ((hp_io = ado_mixer_element_io(mixer, "HP OUT", SND_MIXER_ETYPE_OUTPUT, 0, 2, stereo_voices)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, hp_vol, hp_io) != 0) {
        return (-1);
    }

    if (ado_mixer_playback_group_create(mixer, SND_MIXER_HEADPHONE_OUT, SND_MIXER_CHN_MASK_STEREO, hp_vol, hp_mute) == NULL) {
        return (-1);
    }

    /* WM8960: DAC master Output */
    if (ado_mixer_element_pcm1(mixer, SND_MIXER_ELEMENT_PLAYBACK, SND_MIXER_ETYPE_PLAYBACK1, 1, &pcm_devices[0]) == NULL) {
        return (-1);
    }

    if ((master_mute = ado_mixer_element_sw1(mixer, "Master Mute", 2, wm8960_dac_mute_control, NULL, NULL))
        == NULL) {
        return (-1);
    }

    if ((master_vol = ado_mixer_element_volume1(mixer, "Master Volume", 2, master_range, wm8960_dac_vol_control, NULL, NULL)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, master_mute, master_vol) != 0) {
        return (-1);
    }

    if ((master_io = ado_mixer_element_io(mixer, "Master OUT", SND_MIXER_ETYPE_OUTPUT, 0, 2, stereo_voices)) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, master_vol, master_io) != 0) {
        return (-1);
    }

    if (ado_mixer_playback_group_create(mixer, SND_MIXER_MASTER_OUT, SND_MIXER_CHN_MASK_STEREO, master_vol, master_mute) == NULL) {
        return (-1);
    }

    /* ################ */
    /*  CAPTURE GROUPS  */
    /* ################ */

    if ((adc_io = ado_mixer_element_io(mixer, "MICIN", SND_MIXER_ETYPE_INPUT, 0, 2, stereo_voices)) == NULL) {
        return (-1);
    }
    /* Analog Input Gain Control */
    if (wm8960->mic_left2right) {
        /* Jointly-volume*/
        if ((input_vol = ado_mixer_element_volume1(mixer, "Mic Volume", 1, input_range, wm8960_input_vol_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    } else {
        if ((input_vol = ado_mixer_element_volume1(mixer, "Mic Volume", 2, input_range, wm8960_input_vol_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    }

    if (ado_mixer_element_route_add(mixer, adc_io, input_vol) != 0) {
        return (-1);
    }

    if (wm8960->mic_left2right) {
        /* Jointly-mute */
        if ((input_mute = ado_mixer_element_sw2(mixer, "Mic Mute", wm8960_input_mute_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    } else {
        if ((input_mute = ado_mixer_element_sw1(mixer, "Mic Mute", 2, wm8960_input_mute_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    }

    if (ado_mixer_element_route_add(mixer, input_vol, input_mute) != 0) {
        return (-1);
    }

    if (wm8960->mic_left2right) {
        /* Jointly-mute */
        if ((adc_mute = ado_mixer_element_sw2(mixer, "ADC Mute", wm8960_adc_mute_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    } else {
        if ((adc_mute = ado_mixer_element_sw1(mixer, "ADC Mute", 2, wm8960_adc_mute_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    }

    if (ado_mixer_element_route_add(mixer, input_mute, adc_mute) != 0) {
        return (-1);
    }

    if (wm8960->mic_left2right) {
        /* Jointly-volume */
        if ((adc_vol = ado_mixer_element_volume1(mixer, "ADC Volume", 1, adc_range, wm8960_adc_vol_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    }
    else {
        if ((adc_vol = ado_mixer_element_volume1(mixer, "ADC Volume", 2, adc_range, wm8960_adc_vol_control, NULL, NULL)) == NULL) {
            return (-1);
        }
    }

    if (ado_mixer_element_route_add(mixer, adc_mute, adc_vol) != 0) {
        return (-1);
    }

    if ((adc_pcm = ado_mixer_element_pcm1(mixer, "Mic PCM", SND_MIXER_ETYPE_CAPTURE1, 1, &pcm_devices[0])) == NULL) {
        return (-1);
    }

    if (ado_mixer_element_route_add(mixer, adc_vol, adc_pcm) != 0) {
        return (-1);
    }

    if (ado_mixer_capture_group_create(mixer, SND_MIXER_MIC_IN, SND_MIXER_CHN_MASK_STEREO, input_vol, input_mute, NULL, NULL) == NULL) {
        return (-1);
    }

    if (ado_mixer_capture_group_create(mixer, SND_MIXER_GRP_IGAIN, SND_MIXER_CHN_MASK_STEREO, adc_vol, adc_mute, NULL, NULL) == NULL) {
        return (-1);
    }

    /* ####################### */
    /* SWITCHES                */
    /* ####################### */
    if (ado_mixer_switch_new(mixer, "MIC BIAS", SND_SW_TYPE_BYTE, 0, wm8960_micbias_get, wm8960_micbias_set, NULL, NULL) == NULL) {
        return (-1);
    }

    if (ado_mixer_switch_new(mixer, "Digital Loopback", SND_SW_TYPE_BYTE, 0,
                             wm8960_loopback_get, wm8960_loopback_set, NULL, NULL) == NULL) {
        return (-1);
    }

    if (ado_mixer_switch_new(mixer, "Analog Loopback", SND_SW_TYPE_BYTE, 0,
                             wm8960_analog_loopback_get, wm8960_analog_loopback_set, NULL, NULL) == NULL) {
        return (-1);
    }


    return (0);
}

/** WM8960 command line opts */
static const char *wm8960_opts[] = {
#define MCLK         0
    "wm8960_mclk",
#define ADCLRC       1
    "wm8960_adclrc",
#define I2CDEV       2
    "wm8960_i2cdev",
#define I2CADDR      3
    "wm8960_i2caddr",
#define MIC_LEFT2RIGHT 4
    "wm8960_mic_left2right",
#define MICBIAS      5
    "wm8960_micbias",
#define CLK_MODE     6
    "wm8960_clk_mode",
#define FORMAT       7
    "wm8960_format",
    NULL
};


#define N_OPTS ((sizeof(wm8960_opts)/sizeof(wm8960_opts[0])) - 1U)

static void dump_options(const char** keys, const char** vals, int nOptions)
{
    int cnt;
    for (cnt = 0; cnt < nOptions; cnt++) {
        if ( vals[cnt] )
            ado_error_fmt("%s = %s", keys[cnt], vals[cnt]);
    }
}

/* Parse mixer options */
static int32_t
wm8960_parse_options (MIXER_CONTEXT_T * wm8960)
{
    const ado_dict_t *dict = NULL;
    const char* optValues[N_OPTS] = {0};
    ado_card_t *card = ado_mixer_get_card(wm8960->mixer);

    dict = ado_get_card_dict( card );

    wm8960->i2c_num = WM8960_DEFAULT_I2C_NUM;
    wm8960->i2c_addr = WM8960_DEFAULT_I2C_ADDR;
    wm8960->mclk = 0;
    wm8960->use_dac_lrck = 0;
    wm8960->clk_mode = WM8960_SLAVE;
    wm8960->format = WM8960_I2S;

    ado_config_load_key_values(dict, wm8960_opts, optValues, N_OPTS, 0, 1);

    if (optValues[MCLK])
    {
        wm8960->mclk = strtoul (optValues[MCLK], NULL, 0);
    }

    if (optValues[ADCLRC])
    {
        wm8960->use_dac_lrck = strtoul (optValues[ADCLRC], NULL, 0);
    }

    if (optValues[I2CDEV])
    {
        wm8960->i2c_num = strtoul (optValues[I2CDEV], NULL, 0);
    }

    if (optValues[I2CADDR])
    {
        wm8960->i2c_addr = strtoul (optValues[I2CADDR], NULL, 0);
    }

    if (optValues[MIC_LEFT2RIGHT])
    {
        unsigned int val = strtoul (optValues[MIC_LEFT2RIGHT], NULL, 0);
        wm8960->mic_left2right = (val == 0) ? false : true;
    }

    if (optValues[MICBIAS])
    {
        unsigned int val = strtoul (optValues[I2CADDR], NULL, 0);
        wm8960->micbias = (val == 0) ? false : true;
    }
    if (optValues[CLK_MODE])
    {
        if (strncasecmp(optValues[CLK_MODE], "master", sizeof("master")) == 0) {
            wm8960->clk_mode = WM8960_MASTER;
            ado_debug(DB_LVL_MIXER, "WM8960 clock mode = Master");
        } else if (strncasecmp(optValues[CLK_MODE], "slave", sizeof("slave")) == 0) {
            wm8960->clk_mode = WM8960_SLAVE;
            ado_debug(DB_LVL_MIXER, "WM8960 clock mode = Slave");
        } else {
            ado_error_fmt ("WM8960: Invalid clk_mode: %s", optValues[CLK_MODE]);
            return (-1);
        }
    }
    if (optValues[FORMAT])
    {
        if (strncasecmp(optValues[FORMAT], "I2S", sizeof("I2S")) == 0) {
            wm8960->format = WM8960_I2S;
            ado_debug(DB_LVL_MIXER, "WM8960 format = I2S");
        } else if (strncasecmp(optValues[FORMAT], "PCM", sizeof("PCM")) == 0) {
            wm8960->format = WM8960_PCM;
            ado_debug(DB_LVL_MIXER, "WM8960 format = PCM");
        } else {
            ado_error_fmt ("WM8960: Invalid format: %s", optValues[FORMAT]);
            return (-1);
        }
    }

    dump_options(wm8960_opts, optValues, N_OPTS);

    return (EOK);
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
    /* setup mixer controls for playback and capture */
    switch (channel) {
        case ADO_PCM_CHANNEL_PLAYBACK:
            ado_pcm_chn_mixer(pcm, ADO_PCM_CHANNEL_PLAYBACK, mixer,
                              ado_mixer_find_element(mixer, SND_MIXER_ETYPE_PLAYBACK1, SND_MIXER_ELEMENT_PLAYBACK, index),
                              ado_mixer_find_group(mixer, SND_MIXER_MASTER_OUT, index));
            break;
        case ADO_PCM_CHANNEL_CAPTURE:
            ado_pcm_chn_mixer(pcm, ADO_PCM_CHANNEL_CAPTURE, mixer,
                              ado_mixer_find_element(mixer, SND_MIXER_ETYPE_CAPTURE1, SND_MIXER_ELEMENT_CAPTURE, index),
                              ado_mixer_find_group(mixer, SND_MIXER_MIC_IN, index));
            break;
        default:
            break;
    }
}

ado_mixer_dll_init_t mixer_dll_init;
int
mixer_dll_init (MIXER_CONTEXT_T ** context, ado_mixer_t * mixer, void *params, void *raw_callbacks, int version)
{
    wm8960_context_t *wm8960;
    char i2c_port[MAX_I2C_PORT];
    ado_mixer_dll_codec_callbacks_t *callbacks = raw_callbacks;

    ado_debug(DB_LVL_MIXER, "Initializing WM8960 Codec");

    if ((wm8960 = (wm8960_context_t *)
        ado_calloc (1, sizeof (wm8960_context_t))) == NULL)
    {
        ado_error_fmt ("WM8960: Failed to allocate device structure - %s", strerror (errno));
        return (-1);
    }

    *context = wm8960;
    ado_mixer_set_name (mixer, "WM8960");

    wm8960->mixer = mixer;
    memcpy (&wm8960->params, params, sizeof (wm8960->params));
    wm8960->sample_rate = wm8960->params.tx_sample_rate;
    wm8960->sample_size = wm8960->params.tx_sample_size;

    if (wm8960_parse_options (wm8960)!=EOK)
    {
        ado_error_fmt ("WM8960: Failed to parse mixer options");
        ado_free (wm8960);
        return (-1);
    }

    if (callbacks)
    {
        callbacks->codec_set_default_group = codec_set_default_group;
    }

    if (wm8960->params.codec_open)
    {
        if (wm8960->params.codec_open(wm8960->params.hw_context) != EOK)
        {
            ado_error_fmt("WM8960: codec open failed");
            ado_free(wm8960);
            return (-1);
        }
    }
    else
    {
        snprintf(i2c_port, MAX_I2C_PORT - 1, "/dev/i2c%d", wm8960->i2c_num);
        if ((wm8960->i2c_fd = open(i2c_port, O_RDWR)) < 0)
        {
            ado_error_fmt("WM8960: could not open i2c device %s - %s", i2c_port, strerror(errno));
            ado_free(wm8960);
            return (-1);
        }
    }

    wm8960_reset(wm8960);
    wm8960_init_clocks(wm8960);
    wm8960_init_subsystems(wm8960);
    wm8960_init_interface(wm8960);

    if (build_wm8960_mixer (wm8960, wm8960->mixer))
    {
        ado_error_fmt ("WM8960: Failed to build mixer");
        if (wm8960->params.codec_close)
        {
            wm8960->params.codec_close(wm8960->params.hw_context);
        }
        else
        {
            close (wm8960->i2c_fd);
        }
        ado_free (wm8960);
        return (-1);
    }

    ado_mixer_set_reset_func (wm8960->mixer, wm8960_reset);
    ado_mixer_set_destroy_func (wm8960->mixer, wm8960_destroy);
    return (0);
}

ado_dll_version_t version;
void
version (int *major, int *minor, char *date)
{
    *major = ADO_MAJOR_VERSION;
    *minor = I2S_CODEC_MINOR_VERSION;
    date = __DATE__;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/mixer/wm8960/wm8960.c $ $Rev: 910920 $")
#endif
