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
 * Private function. Prints and returns I2C flags.
 *
 * @param dev Pointer to I2C driver device structure.
 * @param msr MSR register content.
 *
 * @return Execution status.
 */
i2c_status_t imx_i2c_flags2status(imx_dev_t *dev, uint32_t msr)
{
    i2c_status_t status = I2C_STATUS_DONE;

    /* Internal error , pin low timeout */
    if (msr & IMX_LPI2C_MSR_PLTF_MASK) {
        status = I2C_STATUS_ERROR;
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 status PLTF - pin low timeout.");
        }
    }
    /* This is mostly internal error, FIFO error */
    if (msr & IMX_LPI2C_MSR_FEF_MASK) {
        status = I2C_STATUS_ERROR;
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 status FEF - FIFO error.");
        }
    }
    /* NACK overwrites previous errors, because
     * this error is known for I2C library */
    if (msr & IMX_LPI2C_MSR_NDF_MASK) {
        status = I2C_STATUS_NACK;
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 status NDF - NACK detected!");
        }
    }
    /* Arbitration lost overwrites previous internal errors, because
     * this error is known for I2C library */
    if (msr & IMX_LPI2C_MSR_ALF_MASK) {
        status = I2C_STATUS_ARBL;
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 status ALF - Arbitration lost!");
        }
    }

    return status;
}

/**
 * Private function. Clears I2C status flags.
 *
 * @param dev Pointer to I2C driver device structure.
 *
 * @return Execution status.
 */
i2c_status_t imx_i2c_clear_flags(imx_dev_t *dev)
{
    i2c_status_t status = I2C_STATUS_DONE;
    uint8_t max_retries = 10;

    if (dev->regbase->msr & (IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_NDF_MASK | IMX_LPI2C_MSR_ALF_MASK |
                             IMX_LPI2C_MSR_PLTF_MASK)) {

        status = imx_i2c_flags2status(dev, dev->regbase->msr);

        if (dev->regbase->msr & (IMX_LPI2C_MSR_NDF_MASK | IMX_LPI2C_MSR_FEF_MASK)) {
            /* Wait on stop because it is sent automatically */
            while (!(dev->regbase->msr & IMX_LPI2C_MSR_SDF_MASK) && max_retries--) {
                delay(1);
            }
        }

        /* Clear all flags */
        dev->regbase->msr = dev->regbase->msr | 1;

        /* Reset FIFOs */
        dev->regbase->mcr |= (IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
    }
    return status;
}
/**
 * Private function. Resets I2C peripheral.
 *
 * @param dev Pointer to I2C driver device structure.
 */
void imx_i2c_reset(imx_dev_t *dev)
{
    dev->regbase->mcr = (IMX_LPI2C_MCR_RST_MASK | IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
    nanospin_ns(100);
    dev->regbase->mcr = 0;
}

/**
 * Private function. Waits while I2C line is busy.
 *
 * @param dev Pointer to I2C driver device structure.
 *
 * @return Execution status.
 * @retval I2C_STATUS_ERROR Busy timeout.
 * @retval 0                Success.
 */
inline i2c_status_t imx_wait_busy(imx_dev_t *dev)
{
    unsigned timeout = dev->timeout / 10000;
    /* Check bus busy and master busy flags */
    while ((dev->regbase->msr & (IMX_LPI2C_MSR_BBF_MASK | IMX_LPI2C_MSR_MBF_MASK)) && --timeout) {
        nanospin_ns(10000);
    }
    if (timeout == 0) {
        if (dev->verbose) {
            _i2c_slogf("i2c_imx8 busy timedout (%x %x)", dev->regbase->msr, dev->regbase->mfsr);
        }
        return (I2C_STATUS_BUSY);
    }
    return (I2C_STATUS_DONE);
}

/**
 * Private function. Waits while I2C TX FIFO is full.
 *
 * @param dev Pointer to I2C driver device structure.
 *
 * @return Execution status.
 * @retval I2C_STATUS_ERROR Busy timeout.
 * @retval 0                Success.
 */
i2c_status_t imx_wait_tx_fifo(imx_dev_t *dev)
{
    i2c_status_t status;
    unsigned count = dev->timeout / 10000;

    while (!(dev->regbase->msr & IMX_LPI2C_MSR_TDF_MASK) && (count--)) {
        nanospin_ns(10000);
    }
    /* Check timeout */
    if (count == 0) {
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 wait fifo error");
        }
        status = imx_i2c_clear_flags(dev);
        return ((status != I2C_STATUS_DONE) ? status : I2C_STATUS_ERROR);
    }
    return I2C_STATUS_DONE;
}

/**
 * Private function. Writes one byte to TX FIFO.
 *
 * @param dev  Pointer to I2C driver device structure.
 * @param data Byte to send.
 * @param cmd  I2C command.
 */
inline void imx_write_byte(imx_dev_t *dev, uint8_t data, imx_i2c_cmd_t cmd)
{
    /* Send data to FIFO */
    dev->regbase->mtdr = cmd | data;
}

/**
 * Private function. Sends one byte on I2C bus.
 *
 * @param dev  Pointer to I2C driver device structure.
 * @param data Byte to send.
 * @param cmd  I2C command.
 *
 * @return Execution status.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_DONE  Success.
 */
i2c_status_t imx_send_byte(imx_dev_t *dev, uint8_t data, imx_i2c_cmd_t cmd)
{
    i2c_status_t status;
    /* Clear flags */
    if ((status = imx_i2c_clear_flags(dev)) != I2C_STATUS_DONE) {
        return status;
    }
    /* Wait for ready */
    status = imx_wait_tx_fifo(dev);
    if (status != I2C_STATUS_DONE) {
        return status;
    }
    /* Send data to FIFO */
    imx_write_byte(dev, data, cmd);
    return I2C_STATUS_DONE;
}

/**
 * Private function. Sends slave address on I2C bus.
 *
 * @param dev     Pointer to I2C driver device structure.
 * @param addr    Slave device address.
 * @param fmt     Slave device address format.
 * @param read    Determines read (IMX_LPI2C_ADDR_RD) or write (IMX_LPI2C_ADDR_WR) on I2C bus.
 *
 * @return Execution status.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_DONE  Success.
 */
i2c_status_t imx_send_addr(imx_dev_t *dev, unsigned addr, i2c_addrfmt_t fmt,  int read)
{
    imx_i2c_cmd_t cmd = IMX_I2C_START;
    i2c_status_t status = I2C_STATUS_DONE;

    if (dev->int_mode) {
        if (fmt == I2C_ADDRFMT_7BIT) {
            /* Send whole address in one byte */
            imx_write_byte(dev, (addr << 1) | read, cmd);
        } else {
            /* Extract address into 2 bytes and send according to I2C bus specification */
            imx_write_byte(dev,  IMX_LPI2C_XADDR1(addr), cmd);
            imx_write_byte(dev, IMX_LPI2C_XADDR2(addr), IMX_I2C_TX_DATA);
            /* For RD bit set we need re-send first part of address according to I2C bus spec. */
            if (read) {
                imx_write_byte(dev, IMX_LPI2C_XADDR1(addr) | read, cmd);
            }
        }
    } else {
        if (dev->stopped) {
            status = imx_wait_busy(dev);
            if (status != I2C_STATUS_DONE) {
                return status;
            }
            dev->stopped = 0;
        }
        /* Disable auto stop functionality */
        dev->regbase->mcfgr1 &= ~IMX_LPI2C_MCFGR1_AUTOSTOP_MASK;

        if (fmt == I2C_ADDRFMT_7BIT) {
            /* Send whole address in one byte */
            status = imx_send_byte(dev, (addr << 1) | read, cmd);
        } else {
            /* Extract address into 2 bytes and send according to I2C bus specification */
            status = imx_send_byte(dev,  IMX_LPI2C_XADDR1(addr), cmd);
            status |= imx_send_byte(dev, IMX_LPI2C_XADDR2(addr), IMX_I2C_TX_DATA);
            /* For RD bit set we need re-send first part of address according to I2C bus spec. */
            if (read) {
                status |= imx_send_byte(dev, IMX_LPI2C_XADDR1(addr) | read, cmd);
            }
        }
    }
    dev->state = IMX_LPI2C_START;
    return status;
}

/**
 * Private function. Sends STOP command on I2C bus.
 *
 * @param dev     Pointer to I2C driver device structure.
 *
 * @return Execution status.
 * @retval I2C_STATUS_ERROR I2C error.
 * @retval I2C_STATUS_DONE  Success.
 */
i2c_status_t imx_stop(imx_dev_t *dev)
{
    i2c_status_t status = imx_wait_tx_fifo(dev);
    unsigned count;

    if (status != I2C_STATUS_DONE) {
        return status;
    }

    dev->stopped = 1;
    /* Send stop command */
    status = imx_send_byte(dev, 0, IMX_I2C_STOP);

    if (status != I2C_STATUS_DONE) {
        return status;
    }

    count = dev->timeout / 10000;
    while (!(dev->regbase->msr & IMX_LPI2C_MSR_SDF_MASK) && (count--)) {
        nanospin_ns(10000);
    }
    /* Check timout */
    if (count == 0) {
        if (dev->verbose) {
            _i2c_slogf("i2c-imx8 stop error");
        }
        status = imx_i2c_clear_flags(dev);
        return ((status != I2C_STATUS_DONE) ? status : I2C_STATUS_ERROR);
    }

    /* Clear I2C flags */
    status = imx_i2c_clear_flags(dev);
    return status;
}

/**
 * Get number of available entries in TX FIFO
 *
 * @param dev Pointer to I2C driver device structure.
 *
 * @return    Number of free entries in TX FIFO.
 */
uint32_t imx_get_txfifo_avail(imx_dev_t *dev)
{
    /* Get number of free bytes in the FIFO, keep at least 1 byte free */
    uint32_t avail = dev->txfifo - ((dev->txfifo < 8) ? 1 : 4) - (dev->regbase->mfsr & IMX_LPI2C_MFSR_TXCOUNT_MASK);
    return (avail > dev->txfifo ? 0 : avail);
}

/**
 * LPI2C IRQ handler
 *
 * @param area Device data.
 * @param id   Interrupt ID.
 * @return     Pointer to sigevent.
 */
const struct sigevent* handler(void* area, __attribute__((unused)) int id)
{
    imx_dev_t *dev = (imx_dev_t *)area;
    uint32_t msr;
    uint32_t rd;
    uint32_t max_retries = 10;
    uint32_t count;

    if (dev) {
        /* Check interrupt flags only for enabled interrupts */
        msr = dev->regbase->msr;
        if (msr & dev->regbase->mier) {
            /* Check errors */
            if (msr & (IMX_LPI2C_MSR_NDF_MASK | IMX_LPI2C_MSR_ALF_MASK | IMX_LPI2C_MSR_FEF_MASK)) {
                /* Reported errors are ORed because after NACK usually follows FIFO error which rewrites NACK */
                dev->msr |= msr;
                imx_lpi2c_state_t state = IMX_LPI2C_ERROR;
                uint32_t mier = 0;

                if (msr & (IMX_LPI2C_MSR_NDF_MASK | IMX_LPI2C_MSR_FEF_MASK)) {
                    /* Wait on stop because it is sent automatically */
                    if (!(msr & IMX_LPI2C_MSR_SDF_MASK)) {
                        state = IMX_LPI2C_ERROR_ON_STOP;
                        /* Enable only stop detect */
                        mier = IMX_LPI2C_MIER_SDIE_MASK;
                    }
                }
                /* Disable interrupts or enable only stop detect */
                dev->regbase->mier = mier;
                dev->state = state;
                /* Check stop detect */
            } else if (msr & IMX_LPI2C_MSR_SDF_MASK) {
                /* Stop detect flag */
                dev->state = (dev->state == IMX_LPI2C_ERROR_ON_STOP) ? IMX_LPI2C_ERROR : IMX_LPI2C_DONE;
                /* Disable interrupts */
                dev->regbase->mier = 0;
                /* Reset FIFOs */
                dev->regbase->mcr |= (IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
                /* Check receive flag */
            } else if (msr & IMX_LPI2C_MSR_RDF_MASK & dev->regbase->mier) {
                /* Check command buffer and fill the TX FIFO from the buffer
                 * or disable TX interrupt.
                 */
                if (dev->cmdbuf && dev->cmdlen && (dev->cmd_ix < dev->cmdlen)) {
                    count = imx_get_txfifo_avail(dev);
                    while (count-- && (dev->cmd_ix < dev->cmdlen)) {
                        imx_write_byte(dev, dev->cmdbuf[dev->cmd_ix++], IMX_I2C_RX_DATA);
                    }
                }
                /* Clear flag by reading RX FIFO, flag is cleared when FIFO is empty.
                 * Loop is limited by max_retries because it can be dangerous to execute infinite loop
                 * in the IRQ handler even if I2C transfers are slow enough.
                 * */
                while ((dev->regbase->msr & IMX_LPI2C_MSR_RDF_MASK) && max_retries--) {
                    /* Get number of bytes in the FIFO */
                    count = dev->regbase->mfsr >> IMX_LPI2C_MFSR_RXCOUNT_SHIFT;
                    /* Receive all data from the FIFO */
                    while (count-- && (dev->rxbytes_read < dev->rxlen)) {
                        /* Read the FIFO */
                        rd = dev->regbase->mrdr;
                        /* Check FIFO empty flag */
                        if (!(rd & IMX_LPI2C_MRDR_RXEMPTY_MASK) && dev->rxbuf) {
                            /* Store data into the buffer */
                            *(dev->rxbuf) = rd;
                            dev->rxbytes_read++;
                            dev->rxbuf++;
                        }
                    }
                    if (dev->rxbytes_read == dev->rxlen) {
                        /* We are done */
                        if (dev->rxstop) {
                            /* Enable stop detect and keep errors enabled */
                            dev->regbase->mier = IMX_LPI2C_MIER_SDIE_MASK | IMX_LPI2C_MIER_FEIE_MASK | IMX_LPI2C_MIER_ALIE_MASK |
                                                 IMX_LPI2C_MIER_NDIE_MASK;
                            imx_write_byte(dev, 0, IMX_I2C_STOP);
                            dev->state = IMX_LPI2C_STOP_SENT;
                        } else {
                            /* Disable interrupts */
                            dev->regbase->mier = 0;
                            dev->state = IMX_LPI2C_DONE;
                        }
                        /* Break the loop and finish */
                        break;
                    } else {
                        /* We are not done so reconfigure RX FIFO watermark */
                        /* Use rd as temporary var */
                        rd = dev->rxlen - dev->rxbytes_read;
                        if (rd > dev->rxfifo) {
                            /* To FIFO / 2 */
                            dev->regbase->mfcr = (dev->regbase->mfcr & IMX_LPI2C_MFCR_TXWATER_MASK) | (dev->rxfifo / 2 - 1) <<
                                                 IMX_LPI2C_MFCR_RXWATER_SHIFT;
                        } else {
                            dev->regbase->mfcr = (dev->regbase->mfcr & IMX_LPI2C_MFCR_TXWATER_MASK) | ((uint64_t)rd - 1) <<
                                                 (IMX_LPI2C_MFCR_RXWATER_SHIFT);
                        }
                    }
                }
                if (dev->state != IMX_LPI2C_DONE) {
                    /* We are not done */
                    dev->regbase->msr = msr;
                    return NULL;
                }
                /* Check TX side */
            } else if (msr & IMX_LPI2C_MSR_TDF_MASK & dev->regbase->mier) {
                if ((dev->state  == IMX_LPI2C_WAIT_TX_DONE) || (dev->txbuf == NULL)) {
                    /* Transaction finished */
                    dev->regbase->mier = 0;
                    dev->state = IMX_LPI2C_DONE;
                } else {
                    /* Write task */
                    /* Clear flag by writing data to the TX FIFO.
                     * Loop is limited by max_retries because it can be dangerous to execute infinite loop
                     * in the IRQ handler even if I2C transfers are slow enough.
                     * */
                    while ((dev->regbase->msr & IMX_LPI2C_MSR_TDF_MASK) && max_retries--) {
                        count = imx_get_txfifo_avail(dev);
                        /* Write data to the FIFO */
                        while (count-- && (dev->txbytes_read < dev->txlen)) {
                            /* Write the FIFO */
                            imx_write_byte(dev, *dev->txbuf, IMX_I2C_TX_DATA);
                            dev->txbuf++;
                            dev->txbytes_read++;
                        }
                        if (dev->txbytes_read == dev->txlen) {
                            /* We are done */
                            if (dev->txstop) {
                                dev->regbase->mier = IMX_LPI2C_MIER_SDIE_MASK | IMX_LPI2C_MIER_FEIE_MASK | IMX_LPI2C_MIER_ALIE_MASK |
                                                     IMX_LPI2C_MIER_NDIE_MASK;
                                imx_write_byte(dev, 0, IMX_I2C_STOP);
                                dev->state = IMX_LPI2C_STOP_SENT;
                            } else {
                                /* We need at least 4 bytes for repeated start + 10 bit address + receive command.
                                 * We can finish if FIFO has at least 8 bytes or wait on next TDF.
                                 */
                                if (dev->txfifo >= 8) {
                                    /* Finish, we have space enough for repeated start */
                                    dev->regbase->mier = 0;
                                    dev->state = IMX_LPI2C_DONE;
                                } else {
                                    /* Wait on TX FIFO watermark */
                                    dev->state = IMX_LPI2C_WAIT_TX_DONE;
                                }
                            }
                            /* Break the loop and finish */
                            break;
                        }
                    }
                    if (dev->state != IMX_LPI2C_DONE) {
                        /* We are not done */
                        dev->regbase->msr = msr;
                        return NULL;
                    }
                }
            }
            /* Clear flags and finish */
            dev->regbase->msr = msr;
            return &dev->intrevent;
        }
    }
    return NULL;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/common.c $ $Rev: 893360 $")
#endif
