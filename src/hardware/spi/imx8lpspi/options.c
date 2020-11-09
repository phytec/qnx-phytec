/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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

#include <devctl.h>
#include <drvr/hwinfo.h>
#include <fcntl.h>
#include <hw/sysinfo.h>
#include <stdio.h>
#include <string.h>

#include "mx8lpspi.h"

/**
 * @file       imx8lpspi/options.c
 * @addtogroup spi
 * @{
 */

static char *spi_opts[] = {
    "base",         /* Base address for this SPI controller */
    "irq",          /* IRQ for this SPI interface */
    "clock",        /* SPI peripheral input clock */
    "dbt",          /* Delay between transfers. In terms of half SPI shift clock.*/
    "pcssck",       /* Delay from the PCS assertion to the first SCK edge. In terms of half SPI shift clock.*/
    "sckpcs",       /* Delay from the last SCK edge to the PCS negation */
    "rxedma",       /* Rx EDMA vector */
    "txedma",       /* Tx EDMA vector */
    "slave",        /* Slave mode enable */
    "hwi",          /* HWI peripheral index (0, 1, 2, 3) */
    "blksz",        /* Slave block size */
    "verbose",      /* Verbose mode */
    "pincfg",       /* Pin configuration */
    "csnum",        /* Default (initialization parameter) Slave CSx number */
    "mode",         /* Default (initialization parameter) SPI mode, for details see mode field from spi_cfg_t structure */
    "clock_rate",   /* Default (initialization parameter) SPI clock rate, for details see clock_rate field from spi_cfg_t structure */
    "smmu",         /* Default (initialization parameter) Determines whether to use SMMU smmu=0/off SMMU=1/on */
    NULL
};

/**
 * Load SPI parameters from hwi table.
 *
 * @param dev Low level driver handle.
 * @param hwi Peripheral HWI index (0, 1, 2, 3).
 */
static void spi_hwi_load(imx_spi_t *dev, uint32_t hwi)
{
    hwiattr_spi_t attr;

    unsigned hwi_off = hwi_find_bus(HWI_ITEM_BUS_SPI, hwi);
    if (hwi_off != HWI_NULL_OFF) {
        hwiattr_get_spi(hwi_off, &attr);
        dev->pbase = attr.common.location.base;
        if (attr.common.num_irq) {
            dev->irq = hwitag_find_ivec(hwi_off, NULL);
        }
        if (attr.num_clks) {
            hwi_tag *tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_inputclk, NULL);
            if (tag) {
                dev->periph_clock = tag->inputclk.clk;
            }
        }
    } else {
        LOG_ERROR("(spi  t%d::%s:%d) HW Info index not found in syspage - %d", pthread_self(), __func__, __LINE__, hwi);
    }
}

/**
 * Parse command line options.
 *
 * @param dev       Low level driver handle.
 * @param optstring Command line options.
 *
 * @return EOK or -1 if error
 */
int imxspi_options(imx_spi_t *dev, char *optstring)
{
    int      opt, rc = EOK;
    char     *options, *freeptr, *c, *value, *token;
    uint32_t blksz = 0;
    char     *endptr;
    int      ival;

    if (optstring == NULL) {
        return EOK;
    }
    freeptr = options = strdup(optstring);
    while (options && (*options != '\0')) {
        c = options;
        if ((opt = getsubopt(&options, spi_opts, &value)) == -1) {
            free(freeptr);
            LOG_ERROR("(spi  t%d::%s:%d) Unknown option %s", pthread_self(), __func__, __LINE__, c);
            return -1;
        }
        switch (opt) {
            case 0: /* base */
                dev->pbase = strtoul(value, 0, 0);
                continue;
            case 1: /* irq */
                dev->irq   = strtoul(value, 0, 0);
                continue;
            case 2: /* clock */
                dev->periph_clock = strtoul(value, 0, 0);
                continue;
            case 3: /* dbt */
                dev->dbt = strtoul(value, 0, 0);
                continue;
            case 4: /* pcssck */
                dev->pcssck = strtoul(value, 0, 0);
                continue;
            case 5: /* sckpcs */
                dev->sckpcs = strtoul(value, 0, 0);
                continue;
            case 6: /* rxedma */
                token = strsep(&value, ":");
                dev->rxedma_vect.edmanum = strtoul(token, 0, 0);
                token = strsep(&value, ":");
                dev->rxedma_vect.channelnum = strtoul(token, 0, 0);
                token = strsep(&value, ":");
                dev->rxedma_vect.irqnum = strtoul(token, 0, 0);
                continue;
            case 7: /* txedma */
                token = strsep(&value, ":");
                dev->txedma_vect.edmanum = strtoul(token, 0, 0);
                token = strsep(&value, ":");
                dev->txedma_vect.channelnum = strtoul(token, 0, 0);
                token = strsep(&value, ":");
                dev->txedma_vect.irqnum = strtoul(token, 0, 0);
                continue;
            case 8: /* slave */
                dev->slave = strtoul(value, 0, 0);
                continue;
            case 9: /* hwi */
                spi_hwi_load(dev, strtoul(value, 0, 0));
                continue;
            case 10: /* blksz */
                blksz = strtoul(value, 0, 0);
                dev->notify = blksz ? (blksz - 1) : 0;
                continue;
            case 11: /* verbose */
                dev->verbose = strtoul(value, 0, 0);
                continue;
            case 12: /* pincfg */
                dev->pincfg = strtoul(value, 0, 0);
                continue;
            case 13: /* csnum */
                dev->csnum = strtoul(value, 0, 0);
                continue;
            case 14: /* mode */
                dev->mode = strtoul(value, 0, 0);
                continue;
            case 15: /* clock_rate */
                dev->clock_rate = strtoul(value, 0, 0);
                continue;
            case 16: /* use_smmu */
                ival = strtol(value, &endptr, 0);
                if (strcmp(value, endptr) != 0 && (ival == 0 || ival == 1)) {
                    /* we were given smmu = 0/1 */
                    dev->use_smmu = ival;
                } else {
                    /* strtol failed, we have a str option or some other invalid numeric value */
                    if (strcmp(strlwr(value), "on") == 0) {
                        dev->use_smmu = 1;
                    } else if (strcmp(strlwr(value), "off") == 0) {
                        dev->use_smmu = 0;
                    } else {
                        dev->use_smmu = 0;
                        LOG_ERROR("(spi  t%d::%s:%d) Invalid SMMU Option", pthread_self(), __func__, __LINE__);
                    }
                }
                if ( dev->use_smmu == 0 ) {
                    LOG_INFO("(spi  t%d) SMMU OFF", pthread_self());
                } else {
                    LOG_INFO("(spi  t%d) SMMU ON", pthread_self());
                }
                continue;
        }
    }
    free(freeptr);
    return rc;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/options.c $ $Rev: 894855 $")
#endif
