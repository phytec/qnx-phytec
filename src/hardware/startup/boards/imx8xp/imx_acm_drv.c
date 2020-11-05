/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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

#include "imx_acm_drv.h"
#include "startup.h"
#include <aarch64/mx8xp.h>

/**
 * Helper function. Reports wrong index.
 *
 * @param func String containing function name.
 */
static void imx_acm_wrong_index(const char * func)
{
    crash("%s :Wrong index", func);
}

/**
 * Sets Audio MUX.
 *
 * @param index Audio MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_aud_mux(unsigned index, imx_acm_aud_clk_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_AUD_CLK0_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 1:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_AUD_CLK1_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets MCLK MUX.
 *
 * @param index MCLK MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_mclk_mux(unsigned index, imx_acm_mclk_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_MCLKOUT0_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 1:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_MCLKOUT1_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}


/**
 * Sets ESAI MUX.
 *
 * @param index ESAI MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_esai_mux(unsigned index, imx_acm_esai_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_ESAI0_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets GPT MUX.
 *
 * @param index GPT MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_gpt_mux(unsigned index, imx_acm_gpt_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT0_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 1:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT1_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 2:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT2_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 3:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT3_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 4:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT4_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 5:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT5_MUX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets GPT MUX.
 *
 * @param index GPT MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_gpt_event(unsigned index, uint32_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT0_MUX_CLK_CTL + 0x4, val);
            break;
        case 1:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT1_MUX_CLK_CTL + 0x4, val);
            break;
        case 2:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT2_MUX_CLK_CTL + 0x4, val);
            break;
        case 3:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT3_MUX_CLK_CTL + 0x4, val);
            break;
        case 4:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT4_MUX_CLK_CTL + 0x4, val);
            break;
        case 5:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_GPT5_MUX_CLK_CTL + 0x4, val);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets SAI MUX.
 *
 * @param index SAI MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_sai_mux(unsigned index, imx_acm_sai_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI0_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 1:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI1_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 2:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI2_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 3:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI3_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 4:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI4_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        case 5:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SAI5_MCLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets SPDIF MUX.
 *
 * @param index SPDIF MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_spdif_mux(unsigned index, imx_acm_spdif_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_SPDIF0_TX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Sets MQS MUX.
 *
 * @param index MQS MUX register index.
 * @param val   Register value.
 */
void imx_acm_set_mqs_mux(unsigned index, imx_acm_mqs_mux_t val)
{
    switch (index) {
        case 0:
            IMX_OUT(IMX_ACM_BASE + IMX_ACM_MQS_TX_CLK_CTL, IMX_ACM_MUX_REG_VAL(val));
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            break;
    }
}

/**
 * Gets Aud MUX.
 *
 * @param index Aud MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_aud_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_AUD_CLK0_CTL);
            break;
        case 1:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_AUD_CLK1_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

/**
 * Gets MCLK MUX.
 *
 * @param index MCLK MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_mclk_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_MCLKOUT0_CTL);
            break;
        case 1:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_MCLKOUT1_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

/**
 * Gets ESAI MUX.
 *
 * @param index ESAI MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_esai_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_ESAI0_MCLK_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

/**
 * Gets GPT MUX.
 *
 * @param index GPT MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_gpt_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT0_MUX_CLK_CTL);
            break;
        case 1:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT1_MUX_CLK_CTL);
            break;
        case 2:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT2_MUX_CLK_CTL);
            break;
        case 3:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT3_MUX_CLK_CTL);
            break;
        case 4:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT4_MUX_CLK_CTL);
            break;
        case 5:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT5_MUX_CLK_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

uint32_t imx_acm_get_gpt_event(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT0_MUX_CLK_CTL + 0x4);
            break;
        case 1:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT1_MUX_CLK_CTL + 0x4);
            break;
        case 2:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT2_MUX_CLK_CTL + 0x4);
            break;
        case 3:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT3_MUX_CLK_CTL + 0x4);
            break;
        case 4:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT4_MUX_CLK_CTL + 0x4);
            break;
        case 5:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_GPT5_MUX_CLK_CTL + 0x4);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}


/**
 * Gets SAI MUX.
 *
 * @param index SAI MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_sai_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI0_MCLK_CTL);
            break;
        case 1:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI1_MCLK_CTL);
            break;
        case 2:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI2_MCLK_CTL);
            break;
        case 3:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI3_MCLK_CTL);
            break;
        case 4:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI4_MCLK_CTL);
            break;
        case 5:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SAI5_MCLK_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

/**
 * Gets SPDIF MUX.
 *
 * @param index SPDIF MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_spdif_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_SPDIF0_TX_CLK_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}

/**
 * Gets MQS MUX.
 *
 * @param index MQS MUX register index.
 *
 * @return Register value.
 */
uint32_t imx_acm_get_mqs_mux(unsigned index)
{
    switch (index) {
        case 0:
            return IMX_IN(IMX_ACM_BASE + IMX_ACM_MQS_TX_CLK_CTL);
            break;
        default:
            imx_acm_wrong_index(__FUNCTION__);
            return 0;
            break;
    }
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_acm_drv.c $ $Rev: 891625 $")
#endif
