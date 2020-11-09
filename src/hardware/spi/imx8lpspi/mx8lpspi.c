/*
 * $QNXLicenseC:
 * Copyright 2010, QNX Software Systems.
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

#include <devctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <atomic.h>

#include "mx8lpspi.h"

/**
 * @file       imx8lpspi/mx8lpspi.c
 * @addtogroup spi
 * @{
 */

spi_funcs_t spi_drv_entry = {
    sizeof(spi_funcs_t),
    imxspi_init,    /* init() */
    imxspi_dinit,   /* fini() */
    imxspi_drvinfo, /* drvinfo() */
    imxspi_devinfo, /* devinfo() */
    imxspi_setcfg,  /* setcfg() */
    imxspi_xfer,    /* xfer() */
    imxspi_dmaxfer  /* dma_xfer() */
};

spi_slave_funcs_t spi_slave_drv_entry = {
    sizeof(spi_slave_funcs_t),
    imxspi_init,     /* init() */
    imxspi_dinit,    /* fini() */
    imxspi_drvinfo,  /* drvinfo() */
    imxspi_devinfo,  /* devinfo() */
    imxspi_setcfg,   /* setcfg() */
    imxspi_xfer,     /* xfer() */
    NULL,            /* dma_xfer */
    imxspi_read,     /* read() */
    imxspi_write,    /* write() */
    imxspi_status,   /* status() */
    imxspi_error,    /* error() */
};

static spi_devinfo_t devlist[IMX_SPI_DEV_NUM];
static uint32_t devccr[IMX_SPI_DEV_NUM];
static uint32_t devcfgr0[IMX_SPI_DEV_NUM];
static uint32_t devcfgr1[IMX_SPI_DEV_NUM];
static uint32_t devtcr[IMX_SPI_DEV_NUM];

/**
 * Update SPI configuration parameters.
 *
 * @param dev    Low level driver handle.
 * @param device SSx device on SPI bus.
 */
static void spi_update_devcfg(imx_spi_t *dev, uint32_t device)
{
    devccr[device] = imxspi_cfg(dev, &devlist[device].cfg);
    /* TCR */
    /* CPOL */
    if ((devlist[device].cfg.mode & SPI_MODE_CKPOL_HIGH)) {
        devtcr[device] |= (0x1 << IMX_LPSPI_TCR_CPOL_SHIFT);
    } else {
        devtcr[device] &= ~(0x1 << IMX_LPSPI_TCR_CPOL_SHIFT);
    }
    /* CPHA */
    if (devlist[device].cfg.mode & SPI_MODE_CKPHASE_HALF) {
        devtcr[device] |= (0x1 << (IMX_LPSPI_TCR_CPHA_SHIFT));
    } else {
        devtcr[device] &= ~(0x1 << (IMX_LPSPI_TCR_CPHA_SHIFT));
    }
    /* MSB/LSB */
    if ((devlist[device].cfg.mode & SPI_MODE_BODER_MSB)) {
        devtcr[device] &= ~(0x1 << IMX_LPSPI_TCR_LSBF_SHIFT);
    } else {
        devtcr[device] |= (0x1 << IMX_LPSPI_TCR_LSBF_SHIFT);
    }
    /* FRAMESZ */
    devtcr[device] &= ~IMX_LPSPI_TCR_FRAMESZ_MASK;
    devtcr[device] |= ((((devlist[device].cfg.mode & SPI_MODE_CHAR_LEN_MASK) - 1) & IMX_LPSPI_TCR_FRAMESZ_MASK));
    /* PCS */
    devtcr[device] &= ~IMX_LPSPI_TCR_PCS_MASK;
    devtcr[device] |= (device << IMX_LPSPI_TCR_PCS_SHIFT) & IMX_LPSPI_TCR_PCS_MASK;
    /* CFGR1 */
    /* CS POL */
    if (devlist[device].cfg.mode & SPI_MODE_CSPOL_HIGH) {
        devcfgr1[device] |= (0x1 << (device + IMX_LPSPI_CFGR1_PCSPOL_SHIFT));
    } else {
        devcfgr1[device] &= ~(0x1 << (device + IMX_LPSPI_CFGR1_PCSPOL_SHIFT));
    }
    /* CFGR0 */
    /* HREQ */
    if (devlist[device].cfg.mode & SPI_MODE_RDY_MASK) {
        devcfgr0[device] |= (0x1 << (IMX_LPSPI_CFGR0_HREN_SHIFT));
        if (SPI_MODE_RDY_LEVEL) {
            devcfgr0[device] |= (0x1 << (IMX_LPSPI_CFGR0_HRPOL_SHIFT));
        } else {
            devcfgr0[device] &= ~(0x1 << (IMX_LPSPI_CFGR0_HRPOL_SHIFT));
        }
    } else {
        devcfgr0[device] &= ~(0x1 << (IMX_LPSPI_CFGR0_HREN_SHIFT));
    }
}

/**
 * Register dump. For debug purposes only.
 *
 * @param base Register bases address.
 * @param func Pointer to the function.
 * @param line Line of the code.
 */
void spi_reg_dump(uintptr_t base, const char *func, int line) {
    LOG_INFO("SPI reg. dump start - %s: line %d", func, line);
    LOG_INFO("IMX_LPSPI_VERID: 0x%X", in32(base + IMX_LPSPI_VERID));
    LOG_INFO("IMX_LPSPI_PARAM: 0x%X", in32(base + IMX_LPSPI_PARAM));
    LOG_INFO("IMX_LPSPI_CR: 0x%X", in32(base + IMX_LPSPI_CR));
    LOG_INFO("IMX_LPSPI_SR: 0x%X", in32(base + IMX_LPSPI_SR));
    LOG_INFO("IMX_LPSPI_IER: 0x%X", in32(base + IMX_LPSPI_IER));
    LOG_INFO("IMX_LPSPI_DER: 0x%X", in32(base + IMX_LPSPI_DER));
    LOG_INFO("IMX_LPSPI_CFGR0: 0x%X", in32(base + IMX_LPSPI_CFGR0));
    LOG_INFO("IMX_LPSPI_CFGR1: 0x%X", in32(base + IMX_LPSPI_CFGR1));
    LOG_INFO("IMX_LPSPI_DMR0: 0x%X", in32(base + IMX_LPSPI_DMR0));
    LOG_INFO("IMX_LPSPI_DMR0: 0x%X", in32(base + IMX_LPSPI_DMR1));
    LOG_INFO("IMX_LPSPI_CCR: 0x%X", in32(base + IMX_LPSPI_CCR));
    LOG_INFO("IMX_LPSPI_FCR: 0x%X", in32(base + IMX_LPSPI_FCR));
    LOG_INFO("IMX_LPSPI_FSR: 0x%X", in32(base + IMX_LPSPI_FSR));
    LOG_INFO("IMX_LPSPI_TCR: 0x%X", in32(base + IMX_LPSPI_TCR));
    LOG_INFO("IMX_LPSPI_RSR: 0x%X", in32(base + IMX_LPSPI_RSR));
    LOG_INFO("SPI reg. dump end - %s: line %d", func, line);
}

/**
 * Configure all necessary settings related to Slave RX DMA support.
 *
 * @param dev Low level driver handle.
 *
 * @retval EIO Execution error.
 * @retval EOK Everything is fine.
 */
static int spi_setup_slave_dma(imx_spi_t *dev, uint8_t spi_word)
{
    dma_transfer_t  tinfo;              /**< DMA tinfo structure */
    uint16_t        i;

    /* Size of SPI word in bytes */
    dev->dlen = (spi_word + 7) >> 3;
    /******/
    /* RX */
    /******/
    memset(&tinfo, 0, sizeof(tinfo));
    /* Source DMA parameters */
    if ((tinfo.src_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        return EIO;
    };
    /* Do not increment. Read from IMX_LPSPI_RDR. */
    tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
    tinfo.src_addrs->paddr = (uint32_t)dev->pbase + IMX_LPSPI_RDR;
    tinfo.src_addrs->len = dev->notify + 1;                                 /* Overall buffer size. */
    tinfo.src_fragments = 1;                                                /* No buffer fragmentation used. */
    tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;    /* Do not increment. */
    /* Destination DMA parameters */
    if ((tinfo.dst_addrs = calloc(1, sizeof(dma_addr_t) * IMX_SPI_SLAVE_DMA_BUF_FRGS)) == NULL) {
        free(tinfo.src_addrs);
        return EIO;
    }
    /* Destination DMA parameters */
    for (i = 0; i < IMX_SPI_SLAVE_DMA_BUF_FRGS; i++) {
        tinfo.dst_addrs[i].paddr = dev->sl_rx_pdmabuf + ((dev->notify + 1) * i);
    }
    for (i = 0; i < IMX_SPI_SLAVE_DMA_BUF_FRGS; i++) {
        tinfo.dst_addrs[i].len = dev->notify + 1;               /* Fragment size */
    }
    tinfo.dst_fragments = IMX_SPI_SLAVE_DMA_BUF_FRGS;           /* Fragmented buffer */
    tinfo.xfer_unit_size = spi_word;                            /* Size in bits! Pass spi_word. */
    tinfo.xfer_bytes = dev->dlen;                               /* SPI word per DMA minor transfer. */
    dev->dmafuncs.setup_xfer(dev->edma_rx_handler, &tinfo);
    dev->dmafuncs.xfer_start(dev->edma_rx_handler);
    free(tinfo.dst_addrs);
    free(tinfo.src_addrs);
    /******/
    /* TX */
    /******/
    memset(&tinfo, 0, sizeof(tinfo));
    /* Source DMA parameters */
    if ((tinfo.src_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        return EIO;
    }
    tinfo.src_addrs->paddr = dev->sl_tx_pdmabuf;
    tinfo.src_addrs->len = dev->notify + 1; /* Overall buffer size */
    tinfo.src_fragments = 1;                /* No buffer fragmentation used */
    /* Destination DMA parameters */
    if ((tinfo.dst_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        free(tinfo.src_addrs);
        return EIO;
    }
    tinfo.dst_addrs->paddr = (uint32_t)dev->pbase + IMX_LPSPI_TDR;
    tinfo.dst_addrs->len = dev->notify + 1;                                 /* Overall buffer size. */
    tinfo.dst_fragments = 1;                                                /* No buffer fragmentation used. */
    tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;    /* Do not increment. Write to HW FIFO. */
    tinfo.xfer_unit_size = spi_word;                                        /* Size in bits!. Pass spi_word. */
    tinfo.xfer_bytes = dev->dlen;                                           /* SPI word per DMA minor transfer. */
    dev->dmafuncs.setup_xfer(dev->edma_tx_handler, &tinfo);
    free(tinfo.dst_addrs);
    free(tinfo.src_addrs);

    return EOK;
}

/**
 * Configure SPI communication parameters. Intended for slave only.
 *
 * @param dev Low level driver handle.
 * @param id  Device identification.
 *
 * @retval EOK Everything is fine
 * @retval EIO Wrong device index used.
 */
static int spi_setup_slave(imx_spi_t *dev, uint32_t id)
{
    uintptr_t base = dev->vbase;  /**< Peripheral Base address */
    uint32_t spi_word;

    /* Slave index (ss0, ss1, ss2, ss3) */
    if (id >= IMX_SPI_DEV_NUM) {
        return EIO;
    }
    /* Size of SPI word in bits */
    spi_word = devlist[id].cfg.mode & SPI_MODE_CHAR_LEN_MASK;
    /* Size of SPI word in bytes */
    dev->dlen = (spi_word + 7) >> 3;
    dev->tlen = 0;
    /* Disable all interrupts */
    out32(base + IMX_LPSPI_IER, 0x0);
    /* Disable DMA requests */
    out32(base + IMX_LPSPI_DER, 0x0);
    /* Setup HREQ */
    out32(base + IMX_LPSPI_CFGR0, devcfgr0[id]);
    /* Setup FIFO water-marks */
    out32(base + IMX_LPSPI_FCR, (IMX_SPI_SLAVE_TX_FIFO_WTMK |
                                (IMX_SPI_SLAVE_RX_FIFO_WTMK << IMX_LPSPI_FCR_RXWATER_SHIFT)));
    /* CSPOL, Slave, PINCFG */
    out32(base + IMX_LPSPI_CFGR1, devcfgr1[id] | (dev->pincfg << IMX_LPSPI_CFGR1_PINCFG_SHIFT));
    /* Setup CPOL, CPHA, MSB/LSB, PCS, FRAMESZ */
    out32(base + IMX_LPSPI_TCR, devtcr[id]);
    if (dev->verbose > 1) {
        spi_reg_dump(base, __func__, __LINE__);
    }
    if (dev->dma) {
        /* Enable RX DMA */
        out32(dev->vbase + IMX_LPSPI_DER, IMX_LPSPI_DER_RDDE_MASK);
        /* Enable RX error interrupt */
        out32(dev->vbase + IMX_LPSPI_IER, IMX_LPSPI_IER_REIE_MASK);
    } else {
        /* Enable RX interrupts */
        out32(dev->vbase + IMX_LPSPI_IER, IMX_LPSPI_IER_RDIE_MASK | IMX_LPSPI_IER_REIE_MASK);
    }

    return EOK;
}

/**
 * Configure SPI communication parameters. Intended for master only.
 *
 * @param dev Low level driver handle.
 * @param id  Device identification.
 */
static void spi_setup_master(imx_spi_t *dev, uint32_t id, uint32_t watermark)
{
    uint32_t  cr;                   /**< CR register */
    uintptr_t base = dev->vbase;    /**< Peripheral Base address */

    /* Disable all interrupts */
    out32(base + IMX_LPSPI_IER, 0x0);
    /* Reset FIFOs */
    cr = in32(dev->vbase + IMX_LPSPI_CR);
    cr |= (IMX_LPSPI_CR_RRF_MASK | IMX_LPSPI_CR_RTF_MASK);
    out32(dev->vbase + IMX_LPSPI_CR, cr);
    /* Setup SPI clock, DBT, PCSSCK, SCKPCS */
    out32(base + IMX_LPSPI_CCR, devccr[id]);
    /* Setup HREQ */
    out32(base + IMX_LPSPI_CFGR0, devcfgr0[id]);
    /* Setup FIFO water-marks to 1 SPI word */
    out32(base + IMX_LPSPI_FCR, ((watermark + 1) | watermark << IMX_LPSPI_FCR_RXWATER_SHIFT));
    /* CSPOL, Master, PINCFG, SAMPLE - (Sampling on delayed edge.
     * Without SAMPLE bit set sampling on higher frequencies is not OK.
     * For lower frequencies it works too. So enabled always.) */
    out32(base + IMX_LPSPI_CFGR1, devcfgr1[id] | IMX_LPSPI_CFGR1_MASTER_MASK | IMX_LPSPI_CFGR1_SAMPLE_MASK |
                                  (dev->pincfg << IMX_LPSPI_CFGR1_PINCFG_SHIFT));
    /* Setup CPOL, CPHA, MSB/LSB, PCS, FRAMESZ */
    out32(base + IMX_LPSPI_TCR, devtcr[id] | IMX_LPSPI_TCR_CONT_MASK);
    if (dev->verbose > 1) {
        spi_reg_dump(base, __func__, __LINE__);
    }
    /* TCR is also part of the FIFO, so wait for TCR write */
    while ((in32(base + IMX_LPSPI_FSR) & IMX_LPSPI_FSR_TXCOUNT_MASK) != 0);
    /* Clean up the RXFIFO */
    while (in32(base + IMX_LPSPI_FSR) & IMX_LPSPI_FSR_RXCOUNT_MASK) {
        in32(base + IMX_LPSPI_RDR);
    }
}

/**
 * Reset and disable spi device.
 *
 * @param dev Low level driver handle.
 */
static void imxspi_reset(imx_spi_t *dev)
{
    uint32_t cr = 0;

    /* Reset all internal logic and registers, except the Control Register. Remains set until cleared by software.*/
    cr = in32(dev->vbase + IMX_LPSPI_CR);
    cr |= IMX_LPSPI_CR_RST_MASK;
    out32(dev->vbase + IMX_LPSPI_CR, cr);
    /* Software reset doesn't reset the CR, so manual reset the FIFOs */
    cr = in32(dev->vbase + IMX_LPSPI_CR);
    cr |= (IMX_LPSPI_CR_RRF_MASK | IMX_LPSPI_CR_RTF_MASK);
    out32(dev->vbase + IMX_LPSPI_CR, cr);
    /* Master logic is not reset and module is disabled.*/
    out32(dev->vbase + IMX_LPSPI_CR, 0);
}

/**
 * Initialization routine.
 *
 * @param hdl     Low level driver handle.
 * @param options Initialization options.
 */
void * imxspi_init(void *hdl, char *options)
{
    imx_spi_t           *dev;
    uintptr_t           base;
    uint32_t            rxfifo = 0;
    uint32_t            txfifo = 0;
    uint8_t             i;
    pthread_attr_t      attr;
    struct sched_param  param;
    int                 policy;

    dev = calloc(1, sizeof(imx_spi_t));
    if (dev == NULL) {
        return NULL;
    }
    /* Set defaults */
    dev->pbase = IMX_LPSPI0_BASE;                   /* Default base address physical value */
    dev->irq   = IMX_LPSPI0_IRQ;                    /* Default interrupt vector number */
    dev->periph_clock = 66500000;                   /* 66500000 MHz SPI shift clock */
    dev->dbt = 0;                                   /* No delay between transfers */
    dev->pcssck = 0;                                /* No delay of PCS assertion to the first SCK edge */
    dev->sckpcs = 0;                                /* No delay from the last SCK edge to the PCS negation */
    dev->rxedma_vect = (imx_edma_vect_t){0, 0, 0};  /* No RX DMA by default */
    dev->txedma_vect = (imx_edma_vect_t){0, 0, 0};  /* No TX DMA vector */
    dev->dma = 0;                                   /* No DMA by default */
    dev->slave = 0;                                 /* No slave mode by default */
    dev->err = 0;                                   /* No slave communication errors */
    dev->notify = 0;                                /* Slave packet size - 1. 0 = Notify after some data received */
    dev->verbose = 0;                               /* Verbose mode */
    dev->pincfg = 0;                                /* Pin configuration SDO = out, SDI = in */
    dev->filldata = 1;                              /* Variable to control TX DMA flow. */
    /* Defaults */
    dev->clock_rate = 5000000;                      /**< Default clock rate (master mode only) */
    dev->mode = 8 | SPI_MODE_BODER_MSB;             /**< Default SPI communication mode */
    dev->csnum = 0;                                 /**< Slave default device number */
    dev->use_smmu = 0;                              /**< Flag to use SMMU for DMA functions */

    if (imxspi_options(dev, options)) {
        goto fail0;
    }
    if (dev->pincfg > 3) {
        goto fail0;
    }
    /* Map in SPI registers */
    if ((base = mmap_device_io(IMX_LPSPI_SIZE, dev->pbase)) == (uintptr_t)MAP_FAILED) {
        goto fail0;
    }
    dev->vbase = base;
    /* Get SPI FIFO size */
    rxfifo = 1 << ((in32(base + IMX_LPSPI_PARAM) & IMX_LPSPI_PARAM_RXFIFO_MASK) >> IMX_LPSPI_PARAM_RXFIFO_SHIFT);
    txfifo = 1 << ((in32(base + IMX_LPSPI_PARAM) & IMX_LPSPI_PARAM_TXFIFO_MASK) >> IMX_LPSPI_PARAM_TXFIFO_SHIFT);
    dev->fifo_size = min(rxfifo, txfifo);
    /* Reset SPI */
    imxspi_reset(dev);
    if (dev->slave) {
        /* Set SPI to Slave mode */
        out32(base + IMX_LPSPI_CFGR1, (dev->pincfg << IMX_LPSPI_CFGR1_PINCFG_SHIFT));
    } else {
        /* Set SPI to Master mode */
        out32(base + IMX_LPSPI_CFGR1, IMX_LPSPI_CFGR1_MASTER_MASK | (dev->pincfg << IMX_LPSPI_CFGR1_PINCFG_SHIFT));
    }
    /* Calculate all device configurations here */
    for (i = 0; i < IMX_SPI_DEV_NUM; i++) {
        devlist[i].device = i;
        sprintf(devlist[i].name, "CSPI-DEV%d", i);
        devlist[i].cfg.mode = dev->mode;
        devlist[i].cfg.clock_rate = dev->clock_rate;
        spi_update_devcfg(dev, i);
    }
    /* Clear SPI interrupts */
    out32(base + IMX_LPSPI_SR, (IMX_LPSPI_SR_DMF_MASK | IMX_LPSPI_SR_REF_MASK | IMX_LPSPI_SR_TEF_MASK |
                                IMX_LPSPI_SR_TCF_MASK | IMX_LPSPI_SR_FCF_MASK | IMX_LPSPI_SR_WCF_MASK));
    /* Attach SPI interrupt/DMA request */
    if ((dev->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK)) == -1) {
        goto fail1;
    }
    if (imxspi_attach_intr(dev)) {
        LOG_ERROR("(spi  t%d::%s:%d) Interrupt attach fail", pthread_self(), __func__, __LINE__);
        goto fail2;
    }
    if ((dev->rxedma_vect.irqnum != 0) && (dev->txedma_vect.irqnum != 0)) {
        dev->dma = 1;
        if (imxspi_attach_edma(dev)) {
            LOG_ERROR("(spi  t%d::%s:%d) DMA attach fail", pthread_self(), __func__, __LINE__);
            if (dev->fd) {
                close(dev->fd);
            }
            goto fail2;
        }
    } else {
        /* Disable all DMA requests */
        out32(base + IMX_LPSPI_DER, 0);
    }
    /* Setup slave mode */
    if (dev->slave) {
        if ((dev->ibuff = buff_create(MAX_BUFF_BYTES)) == NULL) {
            LOG_ERROR("(spi  t%d::%s:%d) ibuff create failed", pthread_self(), __func__, __LINE__);
            goto fail3;
        }
        if ((dev->obuff = buff_create(MAX_BUFF_BYTES)) == NULL) {
            LOG_ERROR("(spi  t%d::%s:%d) obuff create failed", pthread_self(), __func__, __LINE__);
            goto fail3;
        }
        /* Configure thread parameters */
        pthread_attr_init(&attr);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_getschedparam(pthread_self(), &policy, &param);
        param.sched_priority = sched_get_priority_max(SCHED_RR);
        pthread_attr_setschedparam(&attr, &param);
        pthread_attr_setschedpolicy(&attr, SCHED_RR);
        /* Start slave thread */
        if (pthread_create(&dev->tid, &attr, imxspi_slave_thread, dev)) {
            LOG_ERROR("(spi  t%d::%s:%d) Slave thread create failed", pthread_self(), __func__, __LINE__);
            goto fail3;
        }
        if (dev->dma) {
            /* Create slave RX buffer */
            if ((dev->sl_rx_dmabuf = imxspi_edma_create_buffer(dev, &dev->sl_rx_pdmabuf,
                                                               (dev->notify + 1) * IMX_SPI_SLAVE_DMA_BUF_FRGS)) == NULL) {
                goto fail3;
            }
            /* Create slave TX buffer */
            if ((dev->sl_tx_dmabuf = imxspi_edma_create_buffer(dev, &dev->sl_tx_pdmabuf,
                                                               dev->notify + 1)) == NULL) {
                goto fail3;
            }
            /* Configure DMA for slave */
            if (spi_setup_slave_dma(dev, dev->mode & SPI_MODE_CHAR_LEN_MASK)) {
                goto fail3;
            }
        }
        if (spi_setup_slave(dev, dev->csnum)) {
            goto fail3;
        }
        imxspi_tx_lock_init(dev);
        imxspi_rx_lock_init(dev);
    }

    /* Enable SPI */
    dev->spi.hdl = hdl;
    out32(base + IMX_LPSPI_CR, IMX_LPSPI_CR_MEN_MASK);

    return dev;

fail3:
    if (dev->obuff != NULL) {
        _sfree(dev->obuff, sizeof(*dev->obuff));
    }
    if (dev->ibuff != NULL) {
        _sfree(dev->ibuff, sizeof(*dev->ibuff));
    }
fail2:
    ChannelDestroy(dev->chid);
fail1:
    munmap_device_io(dev->vbase, IMX_LPSPI_SIZE);
fail0:
    free(dev);
    return NULL;
}

/**
 * De-initialization routine.
 *
 * @param hdl Low level driver handle.
 */
void imxspi_dinit(void *hdl)
{
    imx_spi_t    *dev = hdl;

    /* Unmap the register, detach the interrupt and DMA */
    if (dev->slave) {
        if (dev->dma) {
            pthread_cancel(dev->tid);
        }
        imxspi_tx_lock_fini(dev);
        imxspi_rx_lock_fini(dev);
    }
    InterruptDetach(dev->iid);
    ConnectDetach(dev->coid);
    ConnectDetach(dev->erxcoid);
    ConnectDetach(dev->etxcoid);
    ChannelDestroy(dev->chid);

    /* Destroy SMMU object */
    if (dev->use_smmu == 1) {
       smmu_obj_destroy(dev->smmu_obj);
        dev->smmu_init--;
        /* Clean-up if last SMMU object */
        if ( dev->smmu_init == 0 ) {
            smmu_fini();
        }
    }

    /* Reset and Disable SPI */
    imxspi_reset(dev);

    munmap_device_io(dev->vbase, IMX_LPSPI_SIZE);
    munmap_device_memory(dev->dmabuf, IMX_SPI_DRV_DMA_BUF_SIZE);

    if (dev->slave) {
        munmap_device_memory(dev->sl_rx_dmabuf, (dev->notify + 1) * IMX_SPI_SLAVE_DMA_BUF_FRGS);
        munmap_device_memory(dev->sl_tx_dmabuf, dev->notify + 1);
        if (dev->obuff != NULL) {
            _sfree(dev->obuff, sizeof(*dev->obuff));
        }
        if (dev->ibuff != NULL) {
            _sfree(dev->ibuff, sizeof(*dev->ibuff));
        }
    }

    if (dev->fd) {
        close(dev->fd);
    }

    free(hdl);
}

/**
 * Get driver information.
 *
 * @param hdl  Low level driver handle.
 * @param info SPI driver info.
 *
 * @return     EOK always
 */
int imxspi_drvinfo(void *hdl, spi_drvinfo_t *info)
{
    info->version = (SPI_VERSION_MAJOR << SPI_VERMAJOR_SHIFT) | (SPI_VERSION_MINOR << SPI_VERMINOR_SHIFT) |
                    (SPI_REVISION << SPI_VERREV_SHIFT);
    strcpy(info->name, IMX_SPI_DRV);
    info->feature = 0;
    return (EOK);
}

/**
 * Set SPI configuration.
 *
 * @param hdl     Low level driver handle.
 * @param device  Connected device (SS device).
 * @param cfg     SPI configuration.
 *
 * @retval EOK    Everything is OK.
 * @retval EINVAL SS index, SPI words out of bounds.
 * @retval EIO    SPI configuration is NULL.
 */
int imxspi_setcfg(void *hdl, uint16_t device, spi_cfg_t *cfg)
{
    imx_spi_t   *dev = hdl;

    if (device >= IMX_SPI_DEV_NUM) {
        return (EINVAL);
    }
    if (cfg == NULL) {
        return EIO;
    }
    if (((cfg->mode & SPI_MODE_CHAR_LEN_MASK) > 32) || ((cfg->mode & SPI_MODE_CHAR_LEN_MASK) < 1)) {
        LOG_ERROR("(spi  t%d::%s:%d) Incorrect SPI word size", pthread_self(), __func__, __LINE__);
        return EINVAL;
    }
    memcpy(&devlist[device].cfg, cfg, sizeof(spi_cfg_t));
    spi_update_devcfg(dev, device);
    /* Configure Slave parameters */
    if (dev->slave) {
        if (dev->dma) {
            if ((dev->mode & SPI_MODE_CHAR_LEN_MASK) != (cfg->mode & SPI_MODE_CHAR_LEN_MASK)) {
                LOG_ERROR("(spi  t%d::%s:%d) SPI slave does not support SPI word reconfiguration", pthread_self(), __func__, __LINE__);
                return EIO;
            }
        }
        if (spi_setup_slave(dev, device)) {
            return EIO;
        }
    }

    return (EOK);
}

/**
 * Get device information (SS connected device).
 *
 * @param hdl    Low level driver handle.
 * @param device SS device id.
 * @param info   Device info structure.
 *
 * @retval EOK    Everything is OK.
 * @retval EINVAL SS index is out of bounds.
 */
int imxspi_devinfo(void *hdl, uint32_t device, spi_devinfo_t *info)
{
    unsigned id = device & SPI_DEV_ID_MASK;

    if (id < ARRAY_SIZE(devlist)) {
        memcpy(info, &devlist[id], sizeof(*info));
    } else {
        return EINVAL;
    }

    return EOK;
}

/**
 * SPI data exchange function (interrupt mode).
 *
 * @param hdl    Low level driver handle.
 * @param device Chip select device (0, 1, 2, 3).
 * @param buf    Pointer to data buffer.
 * @param len    Pointer to data size to send/receive.
 */
void * imxspi_xfer(void *hdl, uint32_t device, uint8_t *buf, int *len)
{
    imx_spi_t   *dev = hdl;
    uintptr_t   base = dev->vbase;
    uint32_t    ssid;
    uint32_t    data;
    uint32_t    spi_word;
    uint32_t    burst;

    /* Device associated with slave select (ss0, ss1, ss2, ss3) */
    ssid = device & SPI_DEV_ID_MASK;
    if (ssid >= IMX_SPI_DEV_NUM) {
        *len = -1;
        return buf;
    }
    spi_word = devlist[ssid].cfg.mode & SPI_MODE_CHAR_LEN_MASK;
    dev->xlen = *len;
    dev->rlen = 0;
    dev->tlen = 0;
    dev->pbuf = buf;
    dev->dlen = (spi_word + 7) >> 3;
    /* Configure water-mark value
     * -1 is for water-mark logic, RX flags is set when more data than water-mark is in RX FIFO.
     * -1 is for CONT mode - without CS toggle controller does not recognize last data. */
    burst = min((dev->xlen - dev->tlen) / dev->dlen, dev->fifo_size);
    if (burst > 1) {
        burst -= 2;
    }
    if (burst == 1) {
        burst -= 1;
    }
    if (dev->xlen % dev->dlen) {
        *len = -1;
        return buf;
    }
    /* Module busy */
    if (in32(base + IMX_LPSPI_SR) & IMX_LPSPI_SR_MBF_MASK) {
        *len = -1;
        LOG_INFO("(spi  t%d::%s:%d) XFER Busy", pthread_self(), __func__, __LINE__);
        return buf;
    }
    /* Estimate transfer time in us... The calculated dtime is only used for
     * the timeout, so it doesn't have to be that accurate.  At higher clock
     * rates, a calculated dtime of 0 would mess-up the timeout calculation, so
     * round up to 1 us
     */
    dev->dtime = dev->dlen * (8 + dev->dbt * 2) * 1000 * 1000 / devlist[ssid].cfg.clock_rate;
    if (dev->dtime == 0) {
        dev->dtime = 1;
    }
    /* Configure SPI communication parameters */
    spi_setup_master(dev, ssid, burst);
    /* Write data to TXFIFO */
    while ((dev->xlen > dev->tlen) && ((dev->tlen / dev->dlen) < dev->fifo_size)) {
        switch (dev->dlen) {
            case NBYTES_1:
                data = buf[dev->tlen];
                break;
            case NBYTES_2:
                data = *(uint16_t *)(&buf[dev->tlen]);
                break;
            case NBYTES_4:
                data = *(uint32_t *)(&buf[dev->tlen]);
                break;
            default:
                LOG_ERROR("(spi  t%d::%s:%d) Unsupported word length", pthread_self(), __func__, __LINE__);
                data = 0;
                break;
        }
        out32(base + IMX_LPSPI_TDR, data);
        dev->tlen += dev->dlen;
    }
    /* In case all data already sent toggle the CS */
    if (dev->tlen >= dev->xlen) {
        uint32_t tcr = in32(base + IMX_LPSPI_TCR);
        tcr &= ~IMX_LPSPI_TCR_CONT_MASK;
        out32(base + IMX_LPSPI_TCR, tcr);
    }
    /* Enable receive interrupt */
    out32(base + IMX_LPSPI_IER, IMX_LPSPI_IER_RDIE_MASK);
    /* Wait for exchange to finish */
    if (imxspi_wait(dev, dev->xlen)) {
        LOG_ERROR("(spi  t%d::%s:%d) XFER Interrupt Timeout", pthread_self(), __func__, __LINE__);
        dev->rlen = -1;
    }
    *len = dev->rlen;

    return buf;
}

/**
 * SPI data exchange function (dma mode).
 *
 * @param hdl    Low level driver handle.
 * @param device Chip select device (0, 1, 2, 3).
 * @param paddr  Pointer to the buffer to send/receive (physical address).
 * @param len    Size of data to send/receive.
 *
 * @return Execution status.
 */
int imxspi_dmaxfer(void *hdl, uint32_t device, spi_dma_paddr_t *paddr, int len)
{
    imx_spi_t       *dev = hdl;         /**< Low level driver handle */
    uint32_t        ssid;               /**< Device associated with slave select (ss0, ss1, ss2, ss3) */
    uint32_t        spi_word;           /**< SPI word size in bits */
    dma_transfer_t  tinfo;              /**< DMA tinfo structure */

    ssid = device & SPI_DEV_ID_MASK;
    /* Check if DMA is enabled and correct CS index is passed. */
    if ((ssid >= IMX_SPI_DEV_NUM) || (dev->dma == 0)) {
        return -1;
    }
    /* There are no data to send/receive */
    if ((paddr->wpaddr == 0) && (paddr->rpaddr == 0)) {
        return -1;
    }
    /* Module busy */
    if (in32(dev->vbase + IMX_LPSPI_SR) & IMX_LPSPI_SR_MBF_MASK) {
        LOG_INFO("(spi  t%d::%s:%d) XFER Busy", pthread_self(), __func__, __LINE__);
        return -1;
    }
    spi_word = devlist[ssid].cfg.mode & SPI_MODE_CHAR_LEN_MASK;
    dev->xlen = len;
    dev->rlen = 0;
    dev->tlen = 0;
    dev->dlen = (spi_word + 7) >> 3;
    /* Estimate transfer time in us... The calculated dtime is only used for
     * the timeout, so it doesn't have to be that accurate.  At higher clock
     * rates, a calculated dtime of 0 would mess-up the timeout calculation, so
     * round up to 1 us
     */
    dev->dtime = dev->dlen * (8 + dev->dbt *2) * 1000 * 1000 / devlist[ssid].cfg.clock_rate;
    if (dev->dtime == 0) {
        dev->dtime = 1;
    }
    /* Configure SPI communication parameters */
    spi_setup_master(dev, ssid, IMX_SPI_EDMA_FIFO_WATERMARK);
    /* Setup the DMA transfer */
    /* RX */
    memset(&tinfo, 0, sizeof(tinfo));
    /* Source DMA parameters */
    /* Do not increment. Read from IMX_LPSPI_RDR. */
    if ((tinfo.src_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        return -1;
    };
    tinfo.src_addrs->paddr = (uint32_t)dev->pbase + IMX_LPSPI_RDR;
    tinfo.src_addrs->len = len; /* Overall buffer size */
    tinfo.src_fragments = 1;
    tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;
    /* Destination DMA parameters */
    if ((tinfo.dst_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        free(tinfo.src_addrs);
        return -1;
    }
    if (paddr->rpaddr) {
        /* Use user RX buffer - DMA library has appropriate checks for address range */
        tinfo.dst_addrs->paddr = paddr->rpaddr;
    } else {
        /* Use DMA driver internal buffer, for details
         * see IMX_SPI_DRV_DMA_BUF_SIZE in mxlpspi.h */
         tinfo.dst_addrs->paddr = dev->pdmabuf;
         tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT;
    }
    tinfo.dst_addrs->len = len;         /* Overall buffer size */
    tinfo.dst_fragments = 1;            /* No buffer fragmentation used */
    tinfo.xfer_unit_size = spi_word;    /* Size in bits! Pass spi_word. */
    tinfo.xfer_bytes = dev->dlen;       /* SPI word per DMA minor transfer */
    dev->dmafuncs.setup_xfer(dev->edma_rx_handler, &tinfo);
    dev->dmafuncs.xfer_start(dev->edma_rx_handler);
    free(tinfo.dst_addrs);
    free(tinfo.src_addrs);
    /* TX */
    memset(&tinfo, 0, sizeof(tinfo));
    /* Source DMA parameters */
    if ((tinfo.src_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        return -1;
    }
    if (paddr->wpaddr) {
        /* Use user TX buffer - DMA library has appropriate checks for address range*/
        tinfo.src_addrs->paddr = paddr->wpaddr;
    } else {
        /* Use user RX buffer - DMA library has appropriate checks for address range */
        tinfo.src_addrs->paddr = paddr->rpaddr;
    }
    tinfo.src_addrs->len = len; /* Overall buffer size */
    tinfo.src_fragments = 1;
    /* Destination DMA parameters */
    if ((tinfo.dst_addrs = calloc(1, sizeof(dma_addr_t))) == NULL) {
        free(tinfo.src_addrs);
        return -1;
    }
    tinfo.dst_addrs->paddr = (uint32_t)dev->pbase + IMX_LPSPI_TDR;
    tinfo.dst_addrs->len = len;                     /* Overall buffer size */
    tinfo.dst_fragments = 1;                        /* No buffer fragmentation used */
    tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;   /* Do not increment. Write to IMX_LPSPI_TDR. */
    tinfo.xfer_unit_size = spi_word;                /* Size in bits!. Pass spi_word. */
    tinfo.xfer_bytes = dev->dlen;                   /* SPI word per DMA minor transfer. */
    dev->dmafuncs.setup_xfer(dev->edma_tx_handler, &tinfo);
    dev->dmafuncs.xfer_start(dev->edma_tx_handler);
    free(tinfo.dst_addrs);
    free(tinfo.src_addrs);
    /* Enable RX, TX DMA requests */
    out32(dev->vbase + IMX_LPSPI_DER, IMX_LPSPI_DER_RDDE_MASK | IMX_LPSPI_DER_TDDE_MASK);
    /* Wait for exchange to finish */
    if (imxspi_wait(dev, dev->xlen)) {
        LOG_ERROR("(spi  t%d::%s:%d) XFER DMA Timeout", pthread_self(), __func__, __LINE__);
        len = -1;
    }
    /* Disable RX, TX DMA requests */
    out32(dev->vbase + IMX_LPSPI_DER, 0);

    return len;
}

/**
 * SPI slave read.
 *
 * @param hdl    Low level driver handle.
 * @param device Chip select device (0, 1, 2, 3).
 * @param buf    Read buffer pointer.
 * @param len    Size of data to read.
 *
 * @return Current buffer or NULL if any error.
 */
void *imxspi_read(void *hdl, uint32_t device, uint8_t *buf, int *len)
{
    imx_spi_t *dev = hdl;
    uint32_t  id;
    uint32_t  nread = 0;
    char      ch;

    id = device & SPI_DEV_ID_MASK;
    if (id >= IMX_SPI_DEV_NUM) {
        LOG_ERROR("(spi  t%d::%s:%d) Not supported device = %d", pthread_self(), __func__, __LINE__, device);
        return NULL;
    }
    imxspi_rx_lock(dev);
    while (buff_waiting(dev->ibuff) && (nread < *len)) {
        if (buff_getc(dev->ibuff, &ch) == 0) {
            buf[nread] = ch;
            nread++;
        } else {
            imxspi_rx_unlock(dev);
            *len = nread;
            LOG_ERROR("(spi  t%d::%s:%d) Read fail", pthread_self(), __func__, __LINE__);
            return buf;
        }
    }
    imxspi_rx_unlock(dev);
    *len = nread;

    return buf;
}

/**
 * SPI slave write.
 *
 * @param hdl    Low level driver handle.
 * @param device Chip select device (0, 1, 2, 3).
 * @param buf    Write buffer pointer.
 * @param len    Size of data to write.
 *
 * @return Current buffer or NULL if any error.
 */
void *imxspi_write(void *hdl, uint32_t device, uint8_t *buf, int *len)
{
    imx_spi_t *dev = hdl;
    uint32_t  id;
    int       nwrite = 0;
    int       nlen = *len;

    *len = 0;
    id = device & SPI_DEV_ID_MASK;
    if (id >= IMX_SPI_DEV_NUM) {
        LOG_ERROR("(spi  t%d::%s:%d) Not supported device = %d", pthread_self(), __func__, __LINE__, device);
        return NULL;
    }
    imxspi_tx_lock(dev);
    if (buff_remaining(dev->obuff) < nlen) {
        imxspi_tx_unlock(dev);
        LOG_ERROR("(spi  t%d::%s:%d) Not enough buffer space left", pthread_self(), __func__, __LINE__);
        return NULL;
    }
    while (nwrite < nlen) {
        if (-1 == buff_putc(dev->obuff, buf[nwrite])) {
            imxspi_tx_unlock(dev);
            *len = nwrite;
            LOG_ERROR("(spi  t%d::%s:%d) Write fail", pthread_self(), __func__, __LINE__);
            return buf;
        } else {
            nwrite++;
        }
    }
    imxspi_tx_unlock(dev);
    *len = nwrite;
    if (!dev->dma) {
        /* Enable TX interrupts */
        out32(dev->vbase + IMX_LPSPI_IER, in32(dev->vbase + IMX_LPSPI_IER) | IMX_LPSPI_IER_TDIE_MASK);
    } else {
        if (dev->filldata) {
            imxspi_tx_lock(dev);
            buff_get(dev->obuff, dev->sl_tx_dmabuf, dev->notify + 1);
            imxspi_tx_unlock(dev);
            dev->dmafuncs.xfer_start(dev->edma_tx_handler);
            atomic_clr(&dev->filldata, 1);
            /* Enable TX DMA */
            out32(dev->vbase + IMX_LPSPI_DER, in32(dev->vbase + IMX_LPSPI_DER) | IMX_LPSPI_DER_TDDE_MASK);
            /* Enable TX error interrupts */
            out32(dev->vbase + IMX_LPSPI_IER, in32(dev->vbase + IMX_LPSPI_IER) | IMX_LPSPI_IER_TEIE_MASK);
        }
    }

    return buf;
}

/**
 * Return error status of the SPI slave instance.
 *
 * @param hdl    Low level driver handle.
 * @param err    Error mask. (SPI_SLAVE_HW_TX_FIFO_UNDERRUN, SPI_SLAVE_HW_RX_FIFO_OVERFLOW)
 *
 * @return Status of the Buffer or NULL if any error.
 */
int imxspi_error(void *hdl, int *err)
{
    imx_spi_t *dev = hdl;

    *err = dev->err;
    dev->err = 0;

    return EOK;
}

/**
 * Return status of the SPI slave buffer.
 *
 * @param hdl    Low level driver handle.
 * @param device Chip select device (0, 1, 2, 3).
 * @param status Buffer status.
 *
 * @return Status of the Buffer or NULL if any error.
 */
void *imxspi_status(void *hdl, uint32_t device, int *status)
{
    imx_spi_t *dev = hdl;
    uint32_t  id;

    id = device & SPI_DEV_ID_MASK;
    if (id >= IMX_SPI_DEV_NUM) {
        LOG_ERROR("(spi  t%d::%s:%d) Not supported device = %d", pthread_self(), __func__, __LINE__, device);
        return NULL;
    }
    /* Log communication error */
    if (dev->err) {
        if (dev->verbose > 0) {
            LOG_ERROR("(spi  t%d::%s:%d) Tx underrun | Rx overflow error: 0x%X", pthread_self(), __func__, __LINE__,
                      dev->err);
        }
        *status |= _NOTIFY_COND_OBAND;
    } else {
        *status &= ~_NOTIFY_COND_OBAND;
    }
    /* Set Rx buffer status */
    imxspi_rx_lock(dev);
    if (buff_waiting(dev->ibuff) > dev->notify) {
        *status |= _NOTIFY_COND_INPUT;
    } else {
        *status &= ~(_NOTIFY_COND_INPUT);
    }
    imxspi_rx_unlock(dev);
    /* Set Tx buffer status */
    imxspi_tx_lock(dev);
    if (buff_remaining(dev->obuff) > dev->notify) {
        *status |= _NOTIFY_COND_OUTPUT;
    } else {
        *status &= ~(_NOTIFY_COND_OUTPUT);
    }
    imxspi_tx_unlock(dev);

    return status;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/mx8lpspi.c $ $Rev: 894855 $")
#endif
