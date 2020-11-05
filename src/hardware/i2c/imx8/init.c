/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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

#include "proto.h"


/**
 * Initializes I2C driver.
 *
 * @param argc Command-line arguments count.
 * @param argv Array of command-line arguments.
 *
 * @return Pointer to I2C driver device structure or NULL when fails.
 */
void *imx_init(int argc, char *argv[])
{
    uint32_t fifo;
    imx_dev_t *dev;
    /* Set thread privileges */
    if (-1 == ThreadCtl(_NTO_TCTL_IO_PRIV, 0)) {
        return NULL;
    }
    /* Allocates device structure */
    dev = calloc(sizeof(*dev), 1);
    if (!dev) {
        return NULL;
    }
    /* Parse command line options */
    if (-1 == imx_options(dev, argc, argv)) {
        free(dev);
        return NULL;
    }
    if (dev->physbase == 0) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO,
              "i2c_imx8 error : Invalid  I2C controller physical base address.");
    }
    /* Map peripheral registers */
    dev->regbase = mmap_device_memory(0, sizeof(imx_lpi2c_t),  PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, dev->physbase);
    if ((uintptr_t) dev->regbase == (uintptr_t) MAP_FAILED) {
        if (dev->verbose) {
            _i2c_slogf("i2c_imx8 mmap_device_memory error");
        }
        free(dev);
        return NULL;
    }

    /* Reset peripheral */
    imx_i2c_reset(dev);
    /* Initialize peripheral */
    dev->regbase->mcfgr0 = 0;
    dev->regbase->mcfgr1 = IMX_LPI2C_MCFGR1_PINCFG(0);
    dev->regbase->mcfgr2 = 0;
    dev->regbase->mcfgr3 = 0;
    /* Get TX FIFO size */
    fifo = 1 << (dev->regbase->param & 0xF);
    /* Initialize TX watermark */
    if (fifo > 8) {
        dev->regbase->mfcr |= fifo / 4 - 1;
    } else if (fifo > 4) {
        dev->regbase->mfcr |= fifo / 2 - 1;
    } else {
        /* Clear TX watermark bitfield, watermark level is 1 word */
        dev->regbase->mfcr &= ~IMX_LPI2C_PARAM_MTXFIFO_MASK;
    }
    dev->txfifo = fifo;
    dev->rxfifo = 1 << (dev->regbase->param >> IMX_LPI2C_PARAM_MRXFIFO_SHIFT);

    if (dev->int_mode) {
        /* Initialize interrupt handler */
        SIGEV_INTR_INIT(&dev->intrevent);
        dev->iid = InterruptAttach(dev->intr, handler, dev, sizeof(imx_dev_t), _NTO_INTR_FLAGS_TRK_MSK);
        if (dev->iid == -1) {
            if (dev->verbose) {
                _i2c_slogf("i2c_imx8 InterruptAttach error");
            }
            munmap_device_memory(dev->regbase, sizeof(imx_lpi2c_t));
            free(dev);
            return NULL;
        }
    }
    /* Disable all interrupts */
    dev->regbase->mier = 0;
    /* Enable I2C master mode */
    dev->regbase->mcr |= IMX_LPI2C_MCR_MEN_MASK | IMX_LPI2C_MCR_DBGEN_MASK;
    if (dev->verbose) {
        _i2c_slogf("Starting lpi2c base 0x%lx in %s mode", dev->physbase, dev->int_mode ? "interrupt" : "polling (DEPRECATED)");
    }
    return dev;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/init.c $ $Rev: 870816 $")
#endif
