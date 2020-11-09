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
 * Receives data from I2C bus in interrupt mode.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length.
 * @param stop Determines if STOP condition is sent.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
i2c_status_t imx_recv_int(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    imx_dev_t *dev = hdl;
    i2c_status_t status = I2C_STATUS_DONE;
    uint64_t ntime;
    int interr;
    uint32_t count;
    uint32_t entries;
    uint32_t i = 0;

    /* Send slave address */
    imx_send_addr(dev, dev->slave_addr, dev->slave_addr_fmt, 1);

    dev->rxbuf = buf;
    dev->rxlen = len;
    dev->rxstop = stop;
    dev->rxbytes_read = 0;
    dev->msr = 0;

    /* Configure RX FIFO watermark */
    if (dev->rxlen > dev->rxfifo) {
        /* To FIFO / 2 */
        dev->regbase->mfcr = (dev->regbase->mfcr & IMX_LPI2C_MFCR_TXWATER_MASK) | ((dev->rxfifo / 2 - 1) <<
                                                                                   IMX_LPI2C_MFCR_RXWATER_SHIFT);
    } else {
        dev->regbase->mfcr = (dev->regbase->mfcr & IMX_LPI2C_MFCR_TXWATER_MASK) | ((dev->rxlen - 1) <<
                                                                                   IMX_LPI2C_MFCR_RXWATER_SHIFT);
    }
    /* Get number of free entries in TX fifo */
    count = imx_get_txfifo_avail(dev);
    if (!count) {
        _i2c_slogf("lpi2c-imx recv txfifo count = 0");
        return I2C_STATUS_ERROR;
    }
    /* Calculate number of entries to write to the TX FIFO */
    entries = len / 256 + ((len % 256) ? 1 : 0);
    /* Allocate command buffer */
    if (count < entries) {
        dev->cmdlen = entries - count;
        dev->cmdbuf = calloc(dev->cmdlen * sizeof(uint8_t), 1);
        if (dev->cmdbuf == NULL) {
            _i2c_slogf("lpi2c-imx recv cmd buf allocation failed");
            return I2C_STATUS_ERROR;
        }
        dev->cmd_ix = 0;
    }
    /* Fill TX FIFO by command data */
    for (i = 0; (i < count) && (i < entries) && len; i++) {
        if (len > 256) {
            imx_write_byte(dev, 255, IMX_I2C_RX_DATA);
            len -= 256;
        } else {
            imx_write_byte(dev, len - 1, IMX_I2C_RX_DATA);
            len = 0;
        }
    }
    /* Fill command buffer */
    for (i = 0; len && (i < dev->cmdlen); i++) {
        if (len > 256) {
            dev->cmdbuf[i] = 255;
            len -= 256;
        } else {
            dev->cmdbuf[i] = len - 1;
            len = 0;
        }
    }
    atomic_set(&dev->regbase->mier, IMX_LPI2C_MIER_RDIE_MASK | IMX_LPI2C_MIER_FEIE_MASK | IMX_LPI2C_MIER_ALIE_MASK |
               IMX_LPI2C_MIER_NDIE_MASK);
    do {
        ntime = dev->timeout;
        interr = EOK;
        /* Set timeout for InterruptWait */
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_INTR, NULL, &ntime, NULL);
        /* Wait for Rx interupt */
        interr = InterruptWait_r(0, NULL);
    } while ((dev->state != IMX_LPI2C_DONE) && (dev->state != IMX_LPI2C_ERROR) && (interr == EOK));

    if ((dev->state == IMX_LPI2C_ERROR) || (interr != EOK)) {
        dev->regbase->mier = 0;
        if (dev->verbose) {
            _i2c_slogf("lpi2c-imx recv interrupt error %u, msr 0x%x, state %u, bytes_read %u, reg_msr 0x%x", interr, dev->msr,
                       dev->state, dev->rxbytes_read, dev->regbase->msr);
        }
        /* Reset FIFOs */
        dev->regbase->mcr |= (IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
        dev->regbase->msr = dev->regbase->msr | 1;
        status = imx_i2c_flags2status(dev, dev->msr);
        dev->msr = 0;
        /* Handle common error */
        status = (status == I2C_STATUS_DONE) ? I2C_STATUS_ERROR : status;
    }
    if (dev->cmdbuf) {
        free((void *)dev->cmdbuf);
    }
    dev->cmdlen = 0;
    dev->cmdbuf = NULL;
    dev->rxbuf = NULL;
    return status;
}

/**
 * Receives data from I2C bus in polling mode.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length.
 * @param stop Determines if STOP condition is sent.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
i2c_status_t imx_recv_noint(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    imx_dev_t *dev = hdl;
    i2c_status_t status;
    uint16_t rd;
    uint8_t i;
    unsigned count;

    if (len <= 0) {
        return I2C_STATUS_DONE;
    }
    /* Check max buffer len. Actually we can send only 256 bytes long buffer at once. */
    if (len > 256) {
        return I2C_STATUS_ERROR;
    }
    /* Send slave address */
    status = imx_send_addr(dev, dev->slave_addr, dev->slave_addr_fmt, 1);

    if (status != I2C_STATUS_DONE) {
        if (dev->verbose) {
            _i2c_slogf("i2c_imx8 recv address error");
        }
        return status;
    }

    /* Send buffer len with receive command to FIFO */
    status = imx_send_byte(dev, len - 1, IMX_I2C_RX_DATA);

    for (i = 0; i < len;) {
        /* Clear flags */
        if ((status = imx_i2c_clear_flags(dev)) != I2C_STATUS_DONE) {
            return status;
        }

        /* Receive data from FIFO */
        while ((dev->regbase->msr & IMX_LPI2C_MSR_RDF_MASK) && (i < len)) {
            /* Read FIFO */
            rd = dev->regbase->mrdr;
            /* Check FIFO empty flag */
            if (!(rd & IMX_LPI2C_MRDR_RXEMPTY_MASK)) {
                /* Store data into buffer */
                *((uint8_t *)buf + i++) = rd;
            }
        }
        /* Wait while FIFO is empty */
        if (i < len) {
            count = dev->timeout / 10000;
            while (!(dev->regbase->msr & IMX_LPI2C_MSR_RDF_MASK) && (count--)) {
                nanospin_ns(10000);
            }
            /* Check timout */
            if (count == 0) {
                if (dev->verbose) {
                    _i2c_slogf("i2c-imx8 recv blocking mode error");
                }
                status = imx_i2c_clear_flags(dev);
                return ((status != I2C_STATUS_DONE) ? status : I2C_STATUS_ERROR);
            }
        }
    }
    /* Send stop */
    if (stop) {
        status = imx_stop(dev);
    }

    return status;
}

/**
 * Receives data from I2C bus.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length.
 * @param stop Determines if STOP condition is sent.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
i2c_status_t imx_recv(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    imx_dev_t *dev = hdl;
    i2c_status_t status;
    uint8_t max_retries = dev->max_retries;

    if (!len || !buf) {
        return I2C_STATUS_ERROR;
    }
    do {
        /* Determine interrupt mode or polling mode */
        if (dev->int_mode) {
            status = imx_recv_int(hdl, buf, len, stop);
        } else {
            status = imx_recv_noint(hdl, buf, len, stop);
        }
        if ((status == I2C_STATUS_DONE) || (status & I2C_STATUS_ARBL)) {
            /* Exit immediately if done or arbitration is lost */
            break;
        }
        if (max_retries != 0) {
            /* Wait some time then repeat */
            delay(10);
        }
        /* Repeat transfer if max_retries is non zero value. Usable if NACK is expected during a communication. */
    } while (max_retries--);
    return status;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/i2c/imx8/recv.c $ $Rev: 870816 $")
#endif
