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

#ifndef MXLPSPI_H_
#define MXLPSPI_H_

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <hw/spi-master.h>
#include <hw/spi-slave.h>
#include <hw/dma.h>
#include <sys/slogcodes.h>

#include "buffer.h"
#include <aarch64/imx8_common/imx_lpspi.h>
#include <smmu.h>

/**
 * @file       imx8lpspi/mx8lpspi.h
 * @addtogroup spi
 * @{
 */

#ifndef IMX_LPSPI_SIZE
#define IMX_LPSPI_SIZE                0x10000
#endif

#ifndef IMX_LPSPI0_BASE
#define IMX_LPSPI0_BASE               0x5A000000
#endif

#ifndef IMX_LPSPI0_IRQ
#define IMX_LPSPI0_IRQ                248
#endif

#define ARRAY_SIZE(array) (sizeof(array)/sizeof((array)[0]))

#define IMX_SPI_DRV                   "IMX_LPSPI"
/** Number of CS devices */
#define IMX_SPI_DEV_NUM               2
/** Pulse priority */
#define IMX_SPI_PRIORITY              21
/** SPI interrupt */
#define IMX_SPI_EVENT                 1
/** SPI RX DMA */
#define IMX_SPI_RX_DMA_EVENT          2
/** SPI TX DMA */
#define IMX_SPI_TX_DMA_EVENT          3
/** DMA internal buffer size */
#define IMX_SPI_DRV_DMA_BUF_SIZE      4
/** SPI word length */
#define NBYTES_1                      1
#define NBYTES_2                      2
#define NBYTES_4                      4
/** SPI EDMA water-mark value (TX is +1)*/
#define IMX_SPI_EDMA_FIFO_WATERMARK   0
/** SPI SLAVE water-mark values */
#define IMX_SPI_SLAVE_RX_FIFO_WTMK    0
#define IMX_SPI_SLAVE_TX_FIFO_WTMK    60
/* SPI slave, number of DMA buffer fragments */
#define IMX_SPI_SLAVE_DMA_BUF_FRGS    8

/** EDMA vector structure - for details see: arm/imx/imx_edma_requests.h file */
typedef struct _imx_edma_vect_t {
    uint32_t        edmanum;            /**< EDMA number */
    uint32_t        channelnum;         /**< EDMA channel number */
    uint32_t        irqnum;             /**< EDMA IRQ number */
} imx_edma_vect_t;

/** SPI Low level driver structure */
typedef struct _imx_spi_t {
    SPIDEV          spi;                /**< It has to be the first element - High level handle */
    unsigned        pbase;              /**< Peripheral physical address */
    uintptr_t       vbase;              /**< Peripheral virtual address */
    int             irq;                /**< SPI interrupt HW number */
    int             iid;                /**< SPI interrupt virtual number */
    int             chid;               /**< Main SPI driver channel id */
    int             coid;               /**< Interrupt connection */
    int             tid;                /**< Thread id */
    int             erxcoid, etxcoid;   /**< DMA connections */
    uint32_t        periph_clock;       /**< SPI functional clock in Hz */
    uint32_t        dbt;                /**< Delay between transfers defined by user */
    uint32_t        pcssck;             /**< Delay from the PCS assertion to the first SCK edge */
    uint32_t        sckpcs;             /**< Delay from the last SCK edge to the PCS negation */
    uint8_t         *pbuf;              /**< Pointer to data in interrupt mode */
    uint32_t        fifo_size;          /**< In size of SPI words */
    int             xlen, tlen, rlen;   /**< Variables for correct interrupt handling */
    int             dlen;               /**< SPI word in bytes (data width) */
    int             dtime;              /**< Timeout: usec per data, for time out use */
    struct sigevent spievent;           /**< SPI interrupt event */
    uint32_t        verbose;            /**< Verbose mode */
    uint32_t        pincfg;             /**< Pin configuration */
    /* Slave */
    uint32_t        err;                /**< SPI communication error for slave mode */
    buffer_t        *ibuff;             /**< Slave input buffer pointer */
    buffer_t        *obuff;             /**< Slave output buffer pointer */
    uint32_t        slave;              /**< Master or Slave mode selection */
    uint16_t        notify;             /**< Packet (Block) size for slave communication - 1 */
    uint32_t        filldata;           /**< Slave TX buffer fill control */
    /* EDMA */
    int             fd;                 /**< File descriptor used for internal buffer allocation for systems with more than 4GB RAM */
    int             dma;                /**< DMA mode enabled/disabled */
    dma_functions_t dmafuncs;           /**< DMA API structure */
    struct sigevent spi_rxedma_event;   /**< DMA event for RX */
    struct sigevent spi_txedma_event;   /**< DMA event for TX */
    imx_edma_vect_t rxedma_vect;        /**< DMA vector for RX */
    imx_edma_vect_t txedma_vect;        /**< DMA vector for TX */
    void            *edma_rx_handler;   /**< DMA channel handler for RX */
    void            *edma_tx_handler;   /**< DMA channel handler for TX */
    void            *dmabuf;            /**< Used when no RX buffer passed from user (virtual) */
    uint32_t        pdmabuf;            /**< Used when no RX buffer passed from user (physical) */
    void            *sl_rx_dmabuf;      /**< RX buffer for slave DMA mode (virtual) */
    uint32_t        sl_rx_pdmabuf;      /**< RX buffer for slave DMA mode (physical) */
    void            *sl_tx_dmabuf;      /**< TX buffer for slave DMA mode (virtual) */
    uint32_t        sl_tx_pdmabuf;      /**< TX buffer for slave DMA mode (physical) */
    intrspin_t      rxspin;             /**< Rx Interrupt spinlock */
    intrspin_t      txspin;             /**< Rx Interrupt spinlock */
    pthread_mutex_t rxmutex;            /**< Rx mutex */
    pthread_mutex_t txmutex;            /**< Tx mutex */
    uint8_t         smmu_init;          /**< SMMU init flag, keeps track of how many devices are using SMMU */
    struct smmu_object *smmu_obj;       /** < SMMU object */

    /* Defaults */
    uint32_t        clock_rate;         /**< Default bit-rate */
    uint32_t        mode;               /**< Default communication mode */
    uint32_t        csnum;              /**< Default chip select device number */
    uint8_t         use_smmu;           /**< Flag to use SMMU for DMA functions */
} imx_spi_t;

int imxspi_options(imx_spi_t *dev, char *optstring);

/* Low level driver API */
void *imxspi_init(void *hdl, char *options);
void imxspi_dinit(void *hdl);
int imxspi_drvinfo(void *hdl, spi_drvinfo_t *info);
int imxspi_devinfo(void *hdl, uint32_t device, spi_devinfo_t *info);
int imxspi_setcfg(void *hdl, uint16_t device, spi_cfg_t *cfg);
void *imxspi_xfer(void *hdl, uint32_t device, uint8_t *buf, int *len);
int imxspi_dmaxfer(void *hdl, uint32_t device, spi_dma_paddr_t *paddr, int len);

/* Slave specific */
void *imxspi_read(void *hdl, uint32_t device, uint8_t *buf, int *len);
void *imxspi_write(void *hdl, uint32_t device, uint8_t *buf, int *len);
void *imxspi_status(void *hdl, uint32_t device, int *status);
int imxspi_error(void *hdl, int *err);
void *imxspi_slave_thread(void *arg);

/* Low level driver internal API */
int imxspi_attach_intr(imx_spi_t *dev);
int imxspi_wait(imx_spi_t *dev, int len);
int imxspi_cfg(void *hdl, spi_cfg_t *cfg);

/* EDMA interface registration */
int imxspi_attach_edma(imx_spi_t *dev);
void *imxspi_edma_create_buffer(imx_spi_t *dev, uint32_t *pbuff, uint32_t size);

/** Logger messages  */
/** Log ERROR message */
#define LOG_ERROR(msg, vars...) slogf(_SLOGC_CHAR, _SLOG_ERROR, msg, ##vars)
/** Log WARNING message */
#define LOG_WARNING(msg, vars...) slogf(_SLOGC_CHAR, _SLOG_WARNING, msg, ##vars)
/** Log INFO message */
#define LOG_INFO(msg, vars...) slogf(_SLOGC_CHAR, _SLOG_INFO, msg, ##vars)

/* Lock macros */
/* Rx */
#define imxspi_rx_lock_init(d)                                                 \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_init(&d->rxmutex, NULL);                             \
        } else {                                                               \
            memset(&d->rxspin, 0, sizeof(d->rxspin));                          \
        }                                                                      \
    }
#define imxspi_rx_lock_fini(d)                                                 \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_destroy(&d->rxmutex);                                \
        }                                                                      \
    }
#define imxspi_rx_lock(d)                                                      \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_lock(&d->rxmutex);                                   \
        } else {                                                               \
            InterruptLock(&d->rxspin);                                         \
        }                                                                      \
    }
#define imxspi_rx_unlock(d)                                                    \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_unlock(&d->rxmutex);                                 \
        } else {                                                               \
            InterruptUnlock(&d->rxspin);                                       \
        }                                                                      \
    }

/* Tx */
#define imxspi_tx_lock_init(d)                                                 \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_init(&d->txmutex, NULL);                             \
        } else {                                                               \
            memset(&d->txspin, 0, sizeof(d->txspin));                          \
        }                                                                      \
    }
#define imxspi_tx_lock_fini(d)                                                 \
if (d->slave) {                                                                \
    if (d->dma) {                                                              \
        pthread_mutex_destroy(&d->txmutex);                                    \
    }                                                                          \
}
#define imxspi_tx_lock(d)                                                      \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_lock(&d->txmutex);                                   \
        } else {                                                               \
            InterruptLock(&d->txspin);                                         \
        }                                                                      \
    }
#define imxspi_tx_unlock(d)                                                    \
    if (d->slave) {                                                            \
        if (d->dma) {                                                          \
            pthread_mutex_unlock(&d->txmutex);                                 \
        } else {                                                               \
            InterruptUnlock(&d->txspin);                                       \
        }                                                                      \
    }
/** @} */

#endif /* MXLPSPI_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/mx8lpspi.h $ $Rev: 894855 $")
#endif
