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

#ifndef CS42XX8_H_
#define CS42XX8_H_

/**
 * CS42xx8 codec driver header file.
 *
 * @file       deva/mixer/cs42xx8/cs42xx8.h
 * @addtogroup cs42xx8
 * @{
 */

#include <stdint.h>

/*
 *  Definitions for Audio registers for CS42448 and CS42888
 */
#define CS42888                  1

#define PWR_CTL                  0x02
#define FN_MODE                  0x03
#define IFACE_FMT                0x04
#define ADC_CTL                  0x05
#define TRANS_CTL                0x06
#define DAC_CH_MUTE              0x07
#define AOUT1_VOL_CTL            0x08
#define AOUT2_VOL_CTL            0x09
#define AOUT3_VOL_CTL            0x0A
#define AOUT4_VOL_CTL            0x0B
#define AOUT5_VOL_CTL            0x0C
#define AOUT6_VOL_CTL            0x0D
#define AOUT7_VOL_CTL            0x0E
#define AOUT8_VOL_CTL            0x0F
#define AIN1_VOL_CTL             0x11
#define AIN2_VOL_CTL             0x12
#define AIN3_VOL_CTL             0x13
#define AIN4_VOL_CTL             0x14

#define AOUT_0DB 255
#define AIN_0DB  128

/* Bit definitions for Power Control Register */
#define PDN                     (1<<0)
#define PDN_DAC1                (1<<1)
#define PDN_DAC2                (1<<2)
#define PDN_DAC3                (1<<3)
#define PDN_DAC4                (1<<4)
#define PDN_ADC1                (1<<5)
#define PDN_ADC2                (1<<6)
#define PDN_ADC3                (1<<7)

/* Bit definitions for Functional Mode Register */
// 'SS' indicates Single-Speed mode (4 - 50kHZ sample rates)
#define ADC_FM_MASTER_SS        (0<<4)
#define ADC_FM_SLAVE            (3<<4)
#define DAC_FM_MASTER_SS        (0<<6)
#define DAC_FM_SLAVE            (3<<6)

/* Bit definitions for Interface Format Register */
#define ADC_DIF_LJ              (0)
#define ADC_DIF_I2S             (1<<0)
#define ADC_DIF_TDM             (6<<0)
#define DAC_DIF_LJ              (0<<3)
#define DAC_DIF_I2S             (1<<3)
#define DAC_DIF_TDM             (6<<3)

/* Bit definitions for ADC Control Register */
#define ADC1_SINGLE             (1<<4)
#define ADC2_SINGLE             (1<<3)
#define ADC3_SINGLE             (1<<2)  // does not exist on cs42888
/* Bit definitions for Transition Control Register */
#define ADC_ZCROSS              (1<<0)
#define ADC_SRAMP               (1<<1)
#define ADC_MUTE_SP             (1<<3)
#define DAC_ZCROSS              (1<<5)
#define DAC_SRAMP               (1<<6)

/* Bit definitions for DAC Channel Mute Register */
#define AOUT1_MUTE              (1<<0)
#define AOUT2_MUTE              (1<<1)
#define AOUT3_MUTE              (1<<2)
#define AOUT4_MUTE              (1<<3)
#define AOUT5_MUTE              (1<<4)
#define AOUT6_MUTE              (1<<5)
#define AOUT7_MUTE              (1<<6)
#define AOUT8_MUTE              (1<<7)
#define AOUT_MUTE_MASK          (0xFF)

#define MAX_STRLEN              50

#define OUTPUT_VOL(x) (255 - x)
#define INPUT_VOL(x) ((x >= 128) ? (x - 128) : (x + 128))

#define CS42XX8_MAX_OUTPUTS 8
#define CS42XX8_MAX_INPUTS  4

/** CS42xx8 context structure */
typedef struct cs42xx8_context{
    ado_mixer_t  *mixer;
    int          fd;
    ado_mixer_dll_codec_params_t   params;
    int i2c_dev;
    int i2c_addr;
    int mode_bits;
    int ms_bits;
    char mode[MAX_STRLEN];
    int format_bits;
    uint32_t ain_mute[CS42XX8_MAX_INPUTS];
    uint8_t ain_vol[CS42XX8_MAX_INPUTS];
    uint8_t aout_vol[CS42XX8_MAX_OUTPUTS];
    uint32_t aout_mute;
    bool codec_playback_on;
    bool codec_capture_on;
    uint32_t ndevs[SND_PCM_CHANNEL_MAX];
} cs42xx8_context_t;

void cs42xx8_mixer(ado_card_t * card, ado_mixer_t ** mixer, char * args, ado_pcm_t * pcm1);
void cs42xx8_set_default_group(ado_pcm_t *pcm, ado_mixer_t *mixer, int channel, int index);
void cs42xx8_codec_off(ado_mixer_t * mixer, int channel);
void cs42xx8_codec_on(ado_mixer_t * mixer, int channel);

void cs42xx8_playback_unmute(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_playback_mute(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_capture_unmute(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_capture_mute(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_playback_power_up(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_playback_power_down(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_capture_power_up(MIXER_CONTEXT_T * cs42xx8);
void cs42xx8_capture_power_down(MIXER_CONTEXT_T * cs42xx8);

#endif /* CS42XX8_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/deva/mixer/cs42xx8/cs42xx8.h $ $Rev: 893028 $")
#endif
