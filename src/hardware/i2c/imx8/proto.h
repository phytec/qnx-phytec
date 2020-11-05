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

#ifndef __PROTO_H_INCLUDED
#define __PROTO_H_INCLUDED

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <atomic.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <hw/inout.h>
#include <hw/i2c.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <aarch64/imx8_common/imx_lpi2c.h>
#include <hw/imx8_lpi2c_cmd.h>
#include <aarch64/mx8x.h>


#define _i2c_slogf(msg, ...)    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, msg, ##__VA_ARGS__)
/* Defines permissible error between required and calculated I2C SCL frequency in Hz */
#define IMX_I2C_SCL_MAX_ERR     1000
/* Defines default input clock frequency in Hz */
#define IMX_I2C_INPUT_CLOCK     24000000

/** I2C states */
typedef enum _imx_lpi2c_state {
    IMX_LPI2C_STOP_SENT,            /**< IMX_LPI2C_STOP_SENT */
    IMX_LPI2C_ERROR_ON_STOP,        /**< IMX_LPI2C_ERROR_ON_STOP */
    IMX_LPI2C_RECV,                 /**< IMX_LPI2C_RECV */
    IMX_LPI2C_SEND,                 /**< IMX_LPI2C_SEND */
    IMX_LPI2C_START,                /**< IMX_LPI2C_START */
    IMX_LPI2C_DONE,                 /**< IMX_LPI2C_DONE */
    IMX_LPI2C_WAIT_TX_DONE,         /**< IMX_LPI2C_WAIT_TX_DONE */
    IMX_LPI2C_ERROR,                /**< IMX_LPI2C_ERROR */
} imx_lpi2c_state_t;

/** I2C driver device structure. */
typedef struct _imx_dev {
    imx_lpi2c_t *regbase;           /**< Virtual address of I2C peripheral registers mapped by mmap_device_memory() */
    paddr_t physbase;               /**< Base address of I2C peripheral registers */
    int intr;                       /**< I2C interrupt event number from reference manual */
    int iid;                        /**< Interrupt event ID returned by InterruptAttachEvent() */
    struct sigevent intrevent;      /**< sigevent structure which is delivered when I2C interrupt occurs */
    unsigned slave_addr;            /**< Slave device address */
    i2c_addrfmt_t slave_addr_fmt;   /**< Determines slave address format 7-bit or 10-bit */
    unsigned input_clk;             /**< I2C peripheral input clock frequency in Hz */
    unsigned speed;                 /**< I2C bus speed in Hz */
    int verbose;                    /**< Driver verbose mode */
    int int_mode;                   /**< Driver uses interrupts */
    int nack;                       /**< Determines if ACK from slave is required or not */
    int stopped;                    /**< Determines if I2C device is in idle state and shall not be busy */
    uint64_t timeout;               /**< Transaction timeout in ns */
    volatile imx_lpi2c_state_t state; /**< LPI2C state used by IRQ handler */
    volatile uint32_t msr;            /**< Holds errors reported during last transfer. It is copy of MSR register */
    volatile uint8_t *txbuf;          /**< Pointer to TX buffer used by IRQ handler */
    uint32_t txlen;                   /**< TX buffer len used by IRQ handler */
    uint8_t txstop;                   /**< TX stop flag. Used by IRQ handler. If set IRQ handler will generate stop on end of packet */
    volatile uint32_t txbytes_read;   /**< TX bytes read from buffer. Used by IRQ handler */
    uint8_t txfifo;                   /**< TX FIFO size. */
    volatile uint8_t *rxbuf;          /**< Pointer to RX buffer used by IRQ handler */
    uint32_t rxlen;                   /**< RX buffer len used by IRQ handler */
    uint8_t rxstop;                   /**< RX stop flag. Used by IRQ handler. If set IRQ handler will generate stop on end of packet */
    volatile uint32_t rxbytes_read;   /**< RX bytes read from RX FIFO. Used by IRQ handler */
    uint8_t rxfifo;                   /**< RX FIFO size. */
    uint8_t max_retries;              /**< Max retry count when NACK is detected. */
    volatile uint8_t *cmdbuf;         /**< Command buffer for receive interrupt */
    volatile uint32_t cmdlen;         /**< Command buffer len */
    volatile uint32_t cmd_ix;         /**< Command buffer index */
} imx_dev_t;

#define IMX_I2C_REVMAJOR(rev)      (((rev) >> 8) & 0xff)
#define IMX_I2C_REVMINOR(rev)      ((rev) & 0xff)

void *imx_init(int argc, char *argv[]);
void imx_fini(void *hdl);
int imx_options(imx_dev_t *dev, int argc, char *argv[]);
int imx_set_slave_addr(void *hdl, unsigned int addr, i2c_addrfmt_t fmt);
int imx_set_bus_speed(void *hdl, unsigned int speed, unsigned int *ospeed);
int imx_version_info(i2c_libversion_t *version);
int imx_driver_info(void *hdl, i2c_driver_info_t *info);
i2c_status_t imx_recv(void *hdl, void *buf, unsigned int len, unsigned int stop);
i2c_status_t imx_send(void *hdl, void *buf, unsigned int len, unsigned int stop);
i2c_status_t imx_send_addr(imx_dev_t *dev, unsigned addr, i2c_addrfmt_t fmt,  int read);
i2c_status_t imx_send_byte(imx_dev_t *dev, uint8_t data, imx_i2c_cmd_t cmd);
void imx_i2c_reset(imx_dev_t *dev);
i2c_status_t imx_wait_busy(imx_dev_t *dev);
i2c_status_t imx_wait_tx_fifo(imx_dev_t *dev);
i2c_status_t imx_stop(imx_dev_t *dev);
i2c_status_t imx_i2c_clear_flags(imx_dev_t *dev);
int imx_ctl(void *hdl, int cmd, void *msg, int msglen, int *nbytes, int *info);
const struct sigevent* handler(void* area, int id);
i2c_status_t imx_i2c_flags2status(imx_dev_t *dev, uint32_t msr);
void imx_write_byte(imx_dev_t *dev, uint8_t data, imx_i2c_cmd_t cmd);
uint32_t imx_get_txfifo_avail(imx_dev_t *dev);
#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/proto.h $ $Rev: 870816 $")
#endif
