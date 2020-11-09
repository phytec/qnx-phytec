/*
 * $QNXLicenseC:
 * Copyright 2010, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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

#include "mx8lpspi.h"

/**
 * @file       imx8lpspi/config.c
 * @addtogroup spi
 * @{
 */

#define MAX_DIVIDER 256         /* Possible max divider is 257 but odd values does not produce 1:1 ratio clock */
#define MIN_DIVIDER 2

/**
 * Save spi timing parameters. (dbt, sckps, pcssck)
 * dbt    - Delay between transfers value.
 * sckps  - Delay from the PCS assertion to the first SCK edge value.
 * pcssck - Delay from the last SCK edge to the PCS negation.
 *
 * @param dev - Low level driver handle
 * @param div - Current used divider value
 *
 * @return Register value of dbt, sckps, pcssck values.
 *
 */
static uint32_t spi_timing(imx_spi_t *dev, uint32_t div)
{
    uint32_t ccr_reg = 0;

    /* Save delay between transfers value */
    if (dev->dbt) {
        ccr_reg |= (((dev->dbt * div / 2) -1) << IMX_LPSPI_CCR_DBT_SHIFT) &
                   IMX_LPSPI_CCR_DBT_MASK;
    }
    /* Save delay from the PCS assertion to the first SCK edge value */
    if (dev->pcssck) {
        ccr_reg |= (((dev->pcssck * div / 2) - 1) << IMX_LPSPI_CCR_PCSSCK_SHIFT) &
                   IMX_LPSPI_CCR_PCSSCK_MASK;
    }
    /* Save delay from the last SCK edge to the PCS negation */
    if (dev->sckpcs) {
        ccr_reg |= (((dev->sckpcs * div / 2) - 1) << IMX_LPSPI_CCR_SCKPCS_SHIFT) &
                   IMX_LPSPI_CCR_SCKPCS_MASK;
    }
    return ccr_reg;
}

/**
 * Parse spi_cfg_t structure.
 *
 * @param hdl Low level driver handle.
 * @param cfg SPI configuration parameters.
 *
 * @return Register value of parsed spi_cfg_t structure
 */
int imxspi_cfg(void *hdl, spi_cfg_t *cfg)
{
    imx_spi_t   *dev = hdl;
    uint32_t    ccr_reg = 0;       /* Clock configuration register */
    uint32_t    div;

    if (cfg == NULL) {
        return 0;
    }
    if (((cfg->mode & SPI_MODE_CHAR_LEN_MASK) > 32) || ((cfg->mode & SPI_MODE_CHAR_LEN_MASK) < 1)) {
        return 0;
    }
    /* Baud rate = Peripheral input clock / (PRESCALE * (SCKDIV + 2)) */
    if (cfg->clock_rate < dev->periph_clock) {
        div = dev->periph_clock / cfg->clock_rate;
        if ((div <= MIN_DIVIDER)) {
            ccr_reg = (MIN_DIVIDER - 2) << IMX_LPSPI_CCR_SCKDIV_SHIFT;
            cfg->clock_rate = dev->periph_clock / MIN_DIVIDER;
            /* Save dbt, pcssck, sckpcs values */
            ccr_reg |= spi_timing(dev, MIN_DIVIDER);
            LOG_WARNING("(spi: 0x%X) Not possible to set requested clock rate (MIN): Current value: %d Hz",
                        dev->pbase, cfg->clock_rate);
        } else if (div >= MAX_DIVIDER) {
            ccr_reg = (MAX_DIVIDER - 2) << IMX_LPSPI_CCR_SCKDIV_SHIFT;
            cfg->clock_rate = dev->periph_clock / MAX_DIVIDER;
            /* Save dbt, pcssck, sckpcs values */
            ccr_reg |= spi_timing(dev, MAX_DIVIDER);
            LOG_WARNING("(spi: 0x%X) Not possible to set requested clock rate (MAX). Current value: %d Hz",
                        dev->pbase, cfg->clock_rate);
        } else {
            if (div % 2) {
                div++;
            }
            /* Save clock divider */
            ccr_reg = ((div - 2) << IMX_LPSPI_CCR_SCKDIV_SHIFT) & IMX_LPSPI_CCR_SCKDIV_MASK;
            cfg->clock_rate = dev->periph_clock / div;
            /* Save dbt, pcssck, sckpcs values */
            ccr_reg |= spi_timing(dev, div);
            if (dev->verbose > 1) {
                LOG_INFO("(spi: 0x%X) Clock rate: %d Hz", dev->pbase, cfg->clock_rate);
            }
        }
    }
    return ccr_reg;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/config.c $ $Rev: 853352 $")
#endif
