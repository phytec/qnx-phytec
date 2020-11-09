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

#include "mx8lpspi.h"

/**
 * @file       imx8lpspi/intr.c
 * @addtogroup spi
 * @{
 */

/**
 * Get number of words in TXFIFO.
 *
 * @param base Peripheral base address.
 *
 * @return Number of words in Tx FIFO.
 */
static inline uint32_t imxspi_get_tx_fifo_count(uintptr_t base)
{
    return ((in32(base + IMX_LPSPI_FSR) & IMX_LPSPI_FSR_TXCOUNT_MASK) >> IMX_LPSPI_FSR_TXCOUNT_SHIFT);
}

/**
 * Get number of words in RXFIFO.
 *
 * @param base Peripheral base address.
 *
 * @return Number of received words.
 */
static inline uint32_t imxspi_get_rx_fifo_count(uintptr_t base)
{
    return ((in32(base + IMX_LPSPI_FSR) & IMX_LPSPI_FSR_RXCOUNT_MASK) >> IMX_LPSPI_FSR_RXCOUNT_SHIFT);
}

/**
 * SPI interrupt routine.
 *
 * @param area Low level driver handle.
 * @param id   ID returned by InterruptAttach.
 *
 * @return EOK or SPI interrupt event.
 */
static const struct sigevent * spi_intr(void *area, int id)
{
    imx_spi_t *dev = area;
    uintptr_t base = dev->vbase;
    char      ch = 0;
    uint32_t  data, tcr, burst, sr;
    uint32_t  txdc, rxdc;                       /* Slave Tx, Rx data count variables */

    /* Save communication status */
    sr = in32(base + IMX_LPSPI_SR);
    /* Clear flags */
    out32(base + IMX_LPSPI_SR, sr);

    if (!dev->slave) {
        /* Process the RX master interrupt only when: appropriate flag is set
         *                                            there is buffer pointer
         *                                            interrupt is enabled */
        if ((sr & IMX_LPSPI_SR_RDF_MASK) && (dev->pbuf != NULL) &&
            (in32(dev->vbase + IMX_LPSPI_IER) & IMX_LPSPI_IER_RDIE_MASK)) {
            if (dev->rlen < dev->xlen) {
                /* Read out available data */
                while ((imxspi_get_rx_fifo_count(base)) && (dev->rlen < dev->xlen)) {
                    data = in32(base + IMX_LPSPI_RDR);
                    switch (dev->dlen) {
                        case NBYTES_1:
                            dev->pbuf[dev->rlen++] = (uint8_t)data;
                            break;
                        case NBYTES_2:
                            *(uint16_t *)(&dev->pbuf[dev->rlen]) = (uint16_t)data;
                            dev->rlen += 2;
                            break;
                        case NBYTES_4:
                            *(uint32_t *)(&dev->pbuf[dev->rlen]) = data;
                            dev->rlen += 4;
                            break;
                        default:
                            break;
                    }
                    /* Are there still some data to send? */
                    if (dev->tlen < dev->xlen) {
                        /* If Rx data retrieved there is available room in TX, so fill it. */
                        switch (dev->dlen) {
                            case NBYTES_1:
                                data = dev->pbuf[dev->tlen];
                                break;
                            case NBYTES_2:
                                data = *(uint16_t *)(&dev->pbuf[dev->tlen]);
                                break;
                            case NBYTES_4:
                                data = *(uint32_t *)(&dev->pbuf[dev->tlen]);
                                break;
                            default:
                                data = 0;
                                break;
                        }
                        out32(base + IMX_LPSPI_TDR, data);
                        dev->tlen += dev->dlen;
                    }
                }
            }
            /* Handle Rx watermark value. Otherwise there is no Rx interrupt
             * because the Rx count is not greater than rxWatermark.
             */
            if (dev->rlen < dev->xlen) {
                burst = min((dev->xlen - dev->tlen) / dev->dlen, dev->fifo_size);
                if (burst > 1) {
                    burst -= 2;
                }
                if (burst == 1) {
                    burst -= 1;
                }
                out32(base + IMX_LPSPI_FCR, (burst | burst << IMX_LPSPI_FCR_RXWATER_SHIFT));
            }
            /* Quit continuous mode */
            if (dev->tlen >= dev->xlen) {
                tcr = in32(base + IMX_LPSPI_TCR);
                tcr &= ~(IMX_LPSPI_TCR_CONTC_MASK | IMX_LPSPI_TCR_CONT_MASK);
                out32(base + IMX_LPSPI_TCR, tcr);
            }
            /* Transfer done */
            if (dev->rlen >= dev->xlen) {
                out32(base + IMX_LPSPI_IER, 0);
                return (&dev->spievent);
            }
        }
    } else {
        /* Slave */
        /* Save err flag value */
        /* SR is set regardless of IER bit value. dev->err is set if the error interrupt is enabled. */
        if (sr & in32(dev->vbase + IMX_LPSPI_IER) & (IMX_LPSPI_SR_REF_MASK | IMX_LPSPI_SR_TEF_MASK)) {
            dev->err |= sr & in32(dev->vbase + IMX_LPSPI_IER) & (IMX_LPSPI_SR_REF_MASK | IMX_LPSPI_SR_TEF_MASK);
            if (dev->dma) {
                return (&dev->spievent);
            }
        }
        if (!dev->dma) {
            /* Tx */
            imxspi_tx_lock(dev);
            /* Fill all available space in FIFO */
            txdc = min(dev->fifo_size - imxspi_get_tx_fifo_count(base), buff_waiting(dev->obuff));
            txdc = min(32, txdc);
            switch (dev->dlen) {
                case NBYTES_1:
                    while (txdc) {
                        buff_getc(dev->obuff, &ch);
                        data = ch;
                        out32(dev->vbase + IMX_LPSPI_TDR, data);    /* Write data to FIFO */
                        txdc--;                                     /* Substract SPI word */
                    }
                    break;
                case NBYTES_2:
                    while (txdc) {
                        buff_getc(dev->obuff, &ch);
                        data = ch;
                        buff_getc(dev->obuff, &ch);
                        data |= (ch << 8);
                        out32(dev->vbase + IMX_LPSPI_TDR, data);    /* Write data to FIFO */
                        txdc--;                                     /* Substract SPI word */
                    }
                    break;
                case NBYTES_4:
                    while (txdc) {
                        buff_getc(dev->obuff, &ch);
                        data = ch;
                        buff_getc(dev->obuff, &ch);
                        data |= (ch << 8);
                        buff_getc(dev->obuff, &ch);
                        data |= (ch << 16);
                        buff_getc(dev->obuff, &ch);
                        data |= (ch << 24);
                        out32(dev->vbase + IMX_LPSPI_TDR, data);    /* Write data to FIFO */
                        txdc--;                                     /* Substract SPI word */
                    }
                    break;
                default:
                    data = 0;
                    out32(dev->vbase + IMX_LPSPI_TDR, data);    /* Write data to FIFO */
                    txdc--;                                     /* Substract SPI word */
                    break;
            }
            imxspi_tx_unlock(dev);
            /* Rx */
            /* Read size of data in FIFO */
            imxspi_rx_lock(dev);
            rxdc = imxspi_get_rx_fifo_count(base);
            rxdc = min(32, rxdc);
            switch (dev->dlen) {
                case NBYTES_1:
                    while (rxdc) {
                        data = in32(base + IMX_LPSPI_RDR);          /* Read SPI word */
                        rxdc--;                                     /* Substract SPI word */
                        buff_putc(dev->ibuff, (uint8_t)(data));
                    }
                    break;
                case NBYTES_2:
                    while (rxdc) {
                        data = in32(base + IMX_LPSPI_RDR);          /* Read SPI word */
                        rxdc--;
                        buff_putc(dev->ibuff, (uint8_t)(data));
                        buff_putc(dev->ibuff, (uint8_t)(data >> 8));
                    }
                    break;
                case NBYTES_4:
                    while (rxdc) {
                        data = in32(base + IMX_LPSPI_RDR);          /* Read SPI word */
                        rxdc--;                                     /* Substract SPI word */
                        buff_putc(dev->ibuff, (uint8_t)(data));
                        buff_putc(dev->ibuff, (uint8_t)(data >> 8));
                        buff_putc(dev->ibuff, (uint8_t)(data >> 16));
                        buff_putc(dev->ibuff, (uint8_t)(data >> 24));
                    }
                    break;
                default:
                    break;
            }
            imxspi_rx_unlock(dev);

            imxspi_tx_lock(dev);
            /* Disable Tx interrupt when no data to send. */
            if (!buff_waiting(dev->obuff)) {
                imxspi_tx_unlock(dev);
                /* Disable Tx interrupt */
                out32(dev->vbase + IMX_LPSPI_IER, in32(dev->vbase + IMX_LPSPI_IER) & ~(IMX_LPSPI_IER_TDIE_MASK |
                                                                                       IMX_LPSPI_IER_TEIE_MASK));
            }
            imxspi_tx_unlock(dev);
            return (&dev->spievent);
        }
    }

    return NULL;
}

/**
 * Attach SPI interrupt routine.
 *
 * @param dev Low level driver handle.
 *
 * @return EOK or -1 otherwise.
 */
int imxspi_attach_intr(imx_spi_t *dev)
{
    if ((dev->coid = ConnectAttach(0, 0, dev->chid, _NTO_SIDE_CHANNEL, 0)) == -1) {
        return -1;
    }
    SIGEV_PULSE_INIT(&dev->spievent, dev->coid, IMX_SPI_PRIORITY, IMX_SPI_EVENT, NULL);
    /* Attach SPI interrupt */
    dev->iid = InterruptAttach(dev->irq, spi_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
    if (dev->iid != -1) {
        return EOK;
    }

    ConnectDetach(dev->coid);

    return -1;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/intr.c $ $Rev: 909766 $")
#endif
