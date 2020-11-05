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

#include <atomic.h>
#include "mx8lpspi.h"

/**
 * @file       imx8lpspi/wait.c
 * @addtogroup spi
 * @{
 */

/* This thread is dedicated for slave */
void *imxspi_slave_thread(void *arg)
{
    struct _pulse pulse;
    imx_spi_t     *dev = arg;
    uint32_t      frag = 0; /* DMA buffer fragment index */
    int           ret;
    int           oleft;

    if (pthread_setname_np(gettid(), "imxlpspi_slthread")) {
        LOG_INFO("(spi  t%d::%s:%d) Set thread name failed", pthread_self(), __func__, __LINE__);
    }
    for (;;) {
        if (MsgReceivePulse(dev->chid, &pulse, sizeof(pulse), NULL) == -1) {
            LOG_ERROR("(spi  t%d::%s:%d) Slave thread pulse error", pthread_self(), __func__, __LINE__);
            break;
        }
        switch (pulse.code) {
            case IMX_SPI_RX_DMA_EVENT:
                /* Check DMA transfer status */
                if (dev->dmafuncs.xfer_complete(dev->edma_rx_handler)) {
                    dev->dmafuncs.xfer_abort(dev->edma_rx_handler);
                    LOG_ERROR("(spi  t%d::%s:%d) Slave Rx DMA aborted", pthread_self(), __func__, __LINE__);
                    dev->err |= SPI_SLAVE_RX_DMA_ABORT;
                    iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                    break;
                }
                imxspi_rx_lock(dev);
                if (buff_remaining(dev->ibuff) < (dev->notify + 1)) {
                    imxspi_rx_unlock(dev);
                    LOG_ERROR("(spi  t%d::%s:%d) RX buffer overflow, packet discarded", pthread_self(), __func__, __LINE__);
                    dev->err |= SPI_SLAVE_RX_DMA_OVERFLOW;
                    iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                } else {
                    ret = buff_put(dev->ibuff, (char *)dev->sl_rx_dmabuf + (dev->notify + 1) * frag, dev->notify + 1);
                    imxspi_rx_unlock(dev);
                    iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_INPUT);
                    if (ret == -1) {
                        dev->err |= SPI_SLAVE_RX_DMA_OVERFLOW;
                        LOG_ERROR("(spi  t%d::%s:%d) RX buffer overrun, no buffer space", pthread_self(), __func__, __LINE__);
                        iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                    }
                }
                frag = (frag + 1) % IMX_SPI_SLAVE_DMA_BUF_FRGS;
                break;
            case IMX_SPI_TX_DMA_EVENT:
                /* Check DMA transfer status */
                if (dev->dmafuncs.xfer_complete(dev->edma_tx_handler)) {
                    dev->dmafuncs.xfer_abort(dev->edma_tx_handler);
                    LOG_ERROR("(spi  t%d::%s:%d) Slave Tx DMA aborted", pthread_self(), __func__, __LINE__);
                    dev->err |= SPI_SLAVE_TX_DMA_ABORT;
                    iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                    break;
                }
                imxspi_tx_lock(dev);
                if (!buff_waiting(dev->obuff)) {
                    imxspi_tx_unlock(dev);
                    /* Set flag for write method to trigger new DMA transfer there. */
                    atomic_set(&dev->filldata, 1);
                    /* Disable HW requests and error interrupts */
                    out32(dev->vbase + IMX_LPSPI_DER, in32(dev->vbase + IMX_LPSPI_DER) & ~IMX_LPSPI_DER_TDDE_MASK);
                    out32(dev->vbase + IMX_LPSPI_IER, in32(dev->vbase + IMX_LPSPI_IER) & ~IMX_LPSPI_IER_TEIE_MASK);
                    /* Notify user application that Tx ring buffer can accept new data */
                    iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OUTPUT);
                } else {
                    /* Copy data into DMA buffer */
                    oleft = buff_get(dev->obuff, dev->sl_tx_dmabuf, dev->notify + 1);
                    if (oleft == -1) {
                        dev->err |= SPI_SLAVE_TX_DMA_UNDERRUN;
                        LOG_ERROR("(spi  t%d::%s:%d) TX buffer underrun, no data", pthread_self(), __func__, __LINE__);
                        iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                    }
                    /* Notify user application that Tx ring buffer can accept new data */
                    if (buff_remaining(dev->obuff) > dev->notify) {
                        iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OUTPUT);
                    }
                    imxspi_tx_unlock(dev);
                    /* Trigger new transfer if there are still data to send */
                    dev->dmafuncs.xfer_start(dev->edma_tx_handler);
                }
                break;
            case IMX_SPI_EVENT:
                if (!dev->dma) {
                    if (buff_waiting(dev->ibuff) > dev->notify) {
                        iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_INPUT);
                    }
                    if (buff_remaining(dev->obuff) > dev->notify) {
                        iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OUTPUT);
                    }
                }
                if (dev->err & SPI_SLAVE_HW_ERROR) {
                   if (dev->verbose > 0) {
                        LOG_ERROR("(spi  t%d::%s:%d) Tx underrun | Rx overflow error: 0x%X", pthread_self(), __func__, __LINE__,
                                  dev->err);
                   }
                   iofunc_notify_trigger(dev->spi.notify, 1, IOFUNC_NOTIFY_OBAND);
                }
                break;
            default:
                LOG_ERROR("(spi  t%d::%s:%d) Unknown event received: %d", pthread_self(), __func__, __LINE__, pulse.code);
                break;
        }
    }

    return NULL;
}

/**
 * Wait for exchange to finish.
 *
 * @param dev Low level driver handle.
 * @param len Transfer length.
 *
 * @return EOK if everything is OK, -1 otherwise.
 */
int imxspi_wait(imx_spi_t *dev, int len)
{
    struct _pulse pulse;
    uint64_t      to;
    uint32_t      tcr, der;

    while (1) {
        if (len) {
            to = dev->dtime;
            to *= len * 1000 * 50;    /* 50 times for time out */
            TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
        }
        if (MsgReceivePulse(dev->chid, &pulse, sizeof(pulse), NULL) == -1) {
            return -1;
        }
        if (pulse.code == IMX_SPI_EVENT) {
            return EOK;
        }
        if (pulse.code == IMX_SPI_TX_DMA_EVENT) {
            /* Check DMA transfer status */
            if (dev->dmafuncs.xfer_complete(dev->edma_tx_handler)) {
                return -1;
            }
            /* Disable DMA request */
            der = in32(dev->vbase + IMX_LPSPI_DER);
            der &= ~IMX_LPSPI_DER_TDDE_MASK;
            out32(dev->vbase + IMX_LPSPI_DER, der);
            /* Toggle the CS line if all data is sent. */
            tcr = in32(dev->vbase + IMX_LPSPI_TCR);
            tcr &= ~(IMX_LPSPI_TCR_CONTC_MASK | IMX_LPSPI_TCR_CONT_MASK);
            out32(dev->vbase + IMX_LPSPI_TCR, tcr);
        }
        if (pulse.code == IMX_SPI_RX_DMA_EVENT) {
            /* Check DMA transfer status */
            if (dev->dmafuncs.xfer_complete(dev->edma_rx_handler)) {
                return -1;
            }
            /* Disable DMA request */
            out32(dev->vbase + IMX_LPSPI_DER, 0);
            return EOK;
        }
    }
    return EOK;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/imx8lpspi/wait.c $ $Rev: 870334 $")
#endif
