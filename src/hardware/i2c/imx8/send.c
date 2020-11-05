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
 * Sends data over I2C bus in interrupt mode.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length in bytes.
 * @param stop Determines whether STOP condition is sent after data transfer.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
static i2c_status_t imx_send_int(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    imx_dev_t *dev = hdl;
    i2c_status_t status = I2C_STATUS_DONE;
    uint64_t ntime;
    int interr;

    /* Send slave address */
    imx_send_addr(dev, dev->slave_addr, dev->slave_addr_fmt, 0);

    dev->txlen = len;
    dev->txbuf = buf;
    dev->txstop = stop;
    dev->txbytes_read = 0;
    dev->msr = 0;

    /* Handle by interrupt */
    atomic_set(&dev->regbase->mier, IMX_LPI2C_MIER_TDIE_MASK | IMX_LPI2C_MIER_FEIE_MASK | IMX_LPI2C_MIER_ALIE_MASK |
               IMX_LPI2C_MIER_NDIE_MASK);
    do {
        ntime = dev->timeout;
        interr = EOK;
        /* Set timeout for InterruptWait */
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_INTR, NULL, &ntime, NULL);
        /* Wait for TX interrupt */
        interr = InterruptWait_r(0, NULL);
    } while ((dev->state != IMX_LPI2C_DONE) && (dev->state != IMX_LPI2C_ERROR) && (interr == EOK));

    if ((dev->state == IMX_LPI2C_ERROR) || (interr != EOK)) {
        dev->regbase->mier = 0;
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 send interrupt error %u, msr 0x%x, state %u, bytes_read %u, reg_msr 0x%x", interr, dev->msr,
                       dev->state, dev->txbytes_read, dev->regbase->msr);
        }
        /* Reset FIFOs */
        dev->regbase->mcr |= (IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
        /* Clear all flags */
        dev->regbase->msr = dev->regbase->msr | 1;
        status = imx_i2c_flags2status(dev, dev->msr);
        dev->msr = 0;
        /* Handle common error */
        status = (status == I2C_STATUS_DONE) ? I2C_STATUS_ERROR : status;
    }
    dev->txbuf = NULL;
    return status;
}

/**
 * Sends data over I2C bus in polling mode.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length in bytes.
 * @param stop Determines whether STOP condition is sent after data transfer.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
static i2c_status_t imx_send_noint(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    imx_dev_t *dev = hdl;
    i2c_status_t status;

    if (len <= 0) {
        return I2C_STATUS_DONE;
    }
    /* Send slave address */
    status = imx_send_addr(dev, dev->slave_addr, dev->slave_addr_fmt, 0);

    if (status != I2C_STATUS_DONE) {
        if (dev->verbose) {
            _i2c_slogf("i2c_imx8 send address error");
        }
        return status;
    }
    /* Send data to slave */
    while (len > 0) {
        /* Send byte by blocking transfer */
        status = imx_send_byte(dev, *(uint8_t *) buf, IMX_I2C_TX_DATA);
        if (status != I2C_STATUS_DONE) {
            if (dev->verbose) {
                _i2c_slogf("i2c_imx8 send byte error");
            }
            return status;
        }
        ++buf;
        --len;
    }
    /* Send stop */
    if (stop) {
        status = imx_stop(dev);
    }
    return status;
}

/**
 * Sends data over I2C bus.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length in bytes.
 * @param stop Determines whether STOP condition is sent after data transfer.
 *
 * @return Execution status.
 * @retval I2C_STATUS_DONE  Success.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_ARBL  I2C arbitration lost error.
 * @retval I2C_STATUS_NACK  Slave no-acknowledge.
 */
i2c_status_t imx_send(void *hdl, void *buf, unsigned int len, unsigned int stop)
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
            status = imx_send_int(hdl, buf, len, stop);
        } else {
            status = imx_send_noint(hdl, buf, len, stop);
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
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/send.c $ $Rev: 893360 $")
#endif
