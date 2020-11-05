/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * Copyright 2018, QNX Software Systems.
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

#include "imx_lpi2c_drv.h"
#include "startup.h"

/**
 * Private function. Clears I2C status flags.
 *
 * @param base LPI2C base address.
 */
static void imx_lpi2c_clear_flags(imx_base_t base)
{
    uint32_t msr = IMX_IN(base + IMX_LPI2C_MSR);

    msr |= (IMX_LPI2C_MSR_DMF_MASK | IMX_LPI2C_MSR_PLTF_MASK | IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_ALF_MASK |
            IMX_LPI2C_MSR_NDF_MASK | IMX_LPI2C_MSR_SDF_MASK | IMX_LPI2C_MSR_EPF_MASK);

    IMX_OUT(base + IMX_LPI2C_MSR, msr);
}
/**
 * Private function. Resets I2C peripheral.
 *
 * @param base LPI2C base address.
 */
static void imx_lpi2c_reset(imx_base_t base)
{
    IMX_OUT(base + IMX_LPI2C_MSR, IMX_LPI2C_MCR_RST_MASK | IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
    imx_usleep(1000);
    IMX_OUT(base + IMX_LPI2C_MSR, 0);
}

/**
 * Private function. Waits while I2C line is busy.
 *
 * @param base LPI2C base address.
 *
 * @return Execution status.
 */
static int imx_lpi2c_wait_busy(imx_base_t base)
{
    /* Wait for 5000 us */
    int timeout = 5000;
    /* Check bus and master busy flags */
    while ((IMX_IN(base + IMX_LPI2C_MSR) & (IMX_LPI2C_MSR_BBF_MASK | IMX_LPI2C_MSR_MBF_MASK)) && --timeout) {
        imx_usleep(1);
    }

    if (timeout <= 0) {
        if (debug_flag) {
            kprintf("%s error 1\n", __FUNCTION__);
        }
        imx_lpi2c_reset(base);
        return -1;
    }

    /* Wait for 1 ms */
    timeout = 1000;
    /* Check TX FIFO */
    while (((IMX_IN(base + IMX_LPI2C_MFSR) & IMX_LPI2C_MFSR_TXCOUNT_MASK) != 0x00) && --timeout) {
        imx_usleep(1);
    }

    if (timeout <= 0) {
        if (debug_flag) {
            kprintf("%s error 2\n", __FUNCTION__);
        }
        imx_lpi2c_reset(base);
        return -1;
    }
    return (0);
}

/**
 * Private function. Waits while I2C TX FIFO is full.
 *
 * @param base LPI2C base address.
 *
 * @return Execution status.
 */
static inline int imx_lpi2c_wait_tx_fifo(imx_base_t base)
{
    int timeout = 1000;
    while (!(IMX_IN(base + IMX_LPI2C_MSR) & IMX_LPI2C_MSR_TDF_MASK) && --timeout) {
        imx_usleep(1);
    }
    if (timeout <= 0) {
        if (debug_flag) {
            kprintf("%s error\n", __FUNCTION__);
        }
        imx_lpi2c_reset(base);
        return -1;
    }
    return 0;
}

/**
 * Private function. Sends one byte on I2C bus.
 *
 * @param base LPI2C base address.
 * @param data Byte to send.
 * @param cmd I2C command type.
 *
 * @return Execution status.
 */
static int imx_lpi2c_send_byte(imx_base_t base, uint8_t data, imx_i2c_cmd_t cmd)
{
    if (imx_lpi2c_wait_tx_fifo(base)) {
        return -1;
    }

    imx_lpi2c_clear_flags(base);

    IMX_OUT(base + IMX_LPI2C_MTDR, cmd | data);

    return 0;
}

/**
 * Private function. Sends slave address on I2C bus.
 *
 * @param base LPI2C base address.
 * @param addr    Slave device address.
 * @param fmt     Slave device address format.
 * @param read    Determines read (IMX_LPI2C_ADDR_RD) or write (IMX_LPI2C_ADDR_WR) on I2C bus.
 *
 * @return Execution status.
 */
static int imx_lpi2c_send_addr(imx_base_t base, unsigned addr, imx_lpi2c_addr_fmt_t fmt,  int read)
{
    int status;

    IMX_OUT(base + IMX_LPI2C_MCFGR1, IMX_IN(base + IMX_LPI2C_MCFGR1) & (~IMX_LPI2C_MCFGR1_AUTOSTOP_MASK));

    if (fmt == IMX_LPI2C_ADDR_7BIT) {
        status = imx_lpi2c_send_byte(base, (addr << 1) | read, IMX_I2C_START);
    } else {
        status = imx_lpi2c_send_byte(base,  IMX_LPI2C_XADDR1(addr), IMX_I2C_START);
        status |= imx_lpi2c_send_byte(base, IMX_LPI2C_XADDR2(addr), IMX_I2C_TX_DATA);
        if (read) {
            status |= imx_lpi2c_send_byte(base, IMX_LPI2C_XADDR1(addr) | read, IMX_I2C_START);
        }
    }

    return status;
}

static int imx_lpi2c_stop(imx_base_t base)
{
    int status = imx_lpi2c_send_byte(base, 0, IMX_I2C_STOP);

    if (status) {
        if (debug_flag) {
            kprintf("%s sendbyte error\n", __FUNCTION__);
        }
        return status;
    }

    while (!(IMX_IN(base + IMX_LPI2C_MSR) & IMX_LPI2C_MSR_SDF_MASK));

    imx_lpi2c_clear_flags(base);

    return status;

}

/**
 * Sets i2c bus speed.
 * @param base LPI2C base address.
 * @param input_clk Source clock frequency in Hz.
 * @param speed  Required speed.
 * @param ospeed Pointer to variable where real calculated speed is returned.
 *
 * @return  Execution status.
 * @retval -1 Fail.
 * @retval 0  Success.
 */
int imx_lpi2c_set_bus_speed(imx_base_t base, unsigned int input_clk, unsigned int speed, unsigned int *ospeed)
{
    if (speed > 400000) {
        return -1;
    }

    uint8_t prescaler = 8;
    uint8_t clk;
    uint32_t real_rate = 0;
    uint8_t filtscl = 0;
    uint32_t err;
    uint32_t best_err = speed;
    uint32_t best_prescaler = 0;
    uint32_t best_clk = 0;
    uint8_t timeout = 0;
    uint8_t master_en = 0;
    uint32_t reg_val = 0;
    int status;
    uint8_t presc_val, clklo, sethold, datavd, busidle;

    status = imx_lpi2c_wait_busy(base);

    if (status) {
        kprintf("%s wait busy error\n", __FUNCTION__);
        return status;
    }

    /* Disable master if enabled */
    if (IMX_IN(base + IMX_LPI2C_MCR) & IMX_LPI2C_MCR_MEN_MASK) {
        while ((IMX_IN(base + IMX_LPI2C_MSR) & IMX_LPI2C_MSR_MBF_MASK) && (timeout++ < 100)) {
            imx_usleep(1000);
        }
        IMX_OUT(base + IMX_LPI2C_MCR, IMX_IN(base + IMX_LPI2C_MCR) & ~(IMX_LPI2C_MCR_MEN_MASK));
        master_en = 1;
    }

    while (prescaler > 0) {
        presc_val = 1 << (--prescaler);
        clk = 32;
        while (clk > 1) {
            /* clklo to clkhi ratio is 2:1 for high speed modes. We use it also for standard modes */
            real_rate = input_clk / ((3 * (clk--) + 2) * presc_val + (2 + filtscl) / presc_val);

            err = (real_rate > speed) ? (real_rate - speed) : (speed - real_rate);

            if ((err < best_err) && (real_rate <= speed)) {
                best_prescaler = prescaler;
                best_clk = clk;
                best_err = err;
                if (err <= IMX_I2C_SCL_MAX_ERR) {
                    break;
                }
            }
        }
    }

    /* clklo to clkhi ratio is 2:1 */
    clklo = 2 * best_clk;
    /* clklo minimum value is 0x3 */
    if (clklo < 3) {
        clklo = 3;
    }
    /* sethold minimum value is 0x2 */
    sethold = (best_clk < 2) ? 2 : best_clk;
    datavd = best_clk / 2;
    if (datavd < 1) {
        datavd = 1;
    }
    IMX_OUT(base + IMX_LPI2C_MCCR0, ((datavd) << IMX_LPI2C_MCCR0_DATAVD_SHIFT) | ((sethold) <<
                                                                                  IMX_LPI2C_MCCR0_SETHOLD_SHIFT)
            | ((best_clk) << IMX_LPI2C_MCCR0_CLKHI_SHIFT) | ((clklo) << IMX_LPI2C_MCCR0_CLKLO_SHIFT));

    busidle = (clklo + sethold + 2) * 2;

    reg_val = IMX_IN(base + IMX_LPI2C_MCFGR1);
    reg_val &= ~(IMX_LPI2C_MCFGR1_PRESCALE_MASK);
    IMX_OUT(base + IMX_LPI2C_MCFGR1, reg_val | (best_prescaler << IMX_LPI2C_MCFGR1_PRESCALE_SHIFT));

    reg_val = IMX_IN(base + IMX_LPI2C_MCFGR2);
    reg_val &= ~(IMX_LPI2C_MCFGR2_BUSIDLE_MASK) | ~(IMX_LPI2C_MCFGR2_FILTSDA_MASK) | ~(IMX_LPI2C_MCFGR2_FILTSCL_MASK);
    IMX_OUT(base + IMX_LPI2C_MCFGR2, reg_val | (busidle << IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT) |
            (filtscl << IMX_LPI2C_MCFGR2_FILTSDA_SHIFT)
            | (filtscl << IMX_LPI2C_MCFGR2_FILTSCL_SHIFT));

    if (master_en) {
        IMX_OUT(base + IMX_LPI2C_MCR, IMX_IN(base + IMX_LPI2C_MCR) | IMX_LPI2C_MCR_MEN_MASK);
    }

    if (ospeed) {
        presc_val = 1 << ((IMX_IN(base + IMX_LPI2C_MCFGR1) & IMX_LPI2C_MCFGR1_PRESCALE_MASK) >>
                          IMX_LPI2C_MCFGR1_PRESCALE_SHIFT);

        clk = ((IMX_IN(base + IMX_LPI2C_MCCR0) & IMX_LPI2C_MCCR0_CLKLO_MASK) >> IMX_LPI2C_MCCR0_CLKLO_SHIFT)
              + ((IMX_IN(base + IMX_LPI2C_MCCR0) & IMX_LPI2C_MCCR0_CLKHI_MASK) >> IMX_LPI2C_MCCR0_CLKHI_SHIFT);

        real_rate = input_clk / ((clk + 2U) * presc_val + (2U + ((IMX_IN(base + IMX_LPI2C_MCFGR2)
                                                                & IMX_LPI2C_MCFGR2_FILTSCL_MASK) >> IMX_LPI2C_MCFGR2_FILTSCL_SHIFT)) / presc_val);;

        *ospeed = real_rate;
    }

    return 0;
}

/**
 * Initializes I2C driver.
 *
 * @param base LPI2C base address.
 * @param source_clock  Source clock frequency in Hz.
 * @param speed I2C bus speed in Hz.
 * @param real_speed Returns calculated I2C bus speed in Hz.
 *
 * @return Execution status.
 */
int imx_lpi2c_init(imx_base_t base, uint32_t source_clock, uint32_t speed, uint32_t * real_speed)
{
    imx_lpi2c_reset(base);

    IMX_OUT(base + IMX_LPI2C_MCFGR0, 0);
    IMX_OUT(base + IMX_LPI2C_MCFGR1, IMX_LPI2C_MCFGR1_PINCFG(0));
    IMX_OUT(base + IMX_LPI2C_MCFGR2, 0);
    IMX_OUT(base + IMX_LPI2C_MCFGR3, 0);

    /* Disable all interrupts */
    IMX_OUT(base + IMX_LPI2C_MIER, 0);

    imx_lpi2c_set_bus_speed(base, source_clock, speed, real_speed);

    /* Enable I2C master mode */
    IMX_OUT(base + IMX_LPI2C_MCR, IMX_IN(base + IMX_LPI2C_MCR) | IMX_LPI2C_MCR_MEN_MASK | IMX_LPI2C_MCR_DBGEN_MASK);

    return 0;
}

/**
 * Receives data from I2C bus.
 *
 * @param base LPI2C base address.
 * @param slave_addr Slave address.
 * @param slave_addr_fmt Slave address format, 7 or 10 bit.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length.
 * @param stop Determines if STOP condition is sent.
 *
 * @return Execution status.
 */
int imx_lpi2c_recv(imx_base_t base, uint16_t slave_addr, imx_lpi2c_addr_fmt_t slave_addr_fmt, uint8_t *buf,
                   unsigned int len, unsigned int stop)
{
    int status;
    uint16_t rd;
    uint8_t i;

    if (len <= 0) {
        return 0;
    }

    if (len > 256) {
        return -1;
    }

    status = imx_lpi2c_send_addr(base, slave_addr, slave_addr_fmt, 1);

    if (status) {
        return status;
    }

    imx_lpi2c_clear_flags(base);
    status = imx_lpi2c_send_byte(base, len - 1, IMX_I2C_RX_DATA);

    for (i = 0; i < len;) {

        if (IMX_IN(base + IMX_LPI2C_MSR) & 0x3C00) {
            return -1;
        }

        while ((IMX_IN(base + IMX_LPI2C_MSR) & IMX_LPI2C_MSR_RDF_MASK) && (i < len)) {
            rd = IMX_IN(base + IMX_LPI2C_MRDR);
            if (!(rd & IMX_LPI2C_MRDR_RXEMPTY_MASK)) {
                *((uint8_t *)buf + i++) = rd;
            }
        }

        if (i < len) {
            while (!(IMX_IN(base + IMX_LPI2C_MSR) & IMX_LPI2C_MSR_RDF_MASK));
        }
    }

    if (stop) {
        status = imx_lpi2c_stop(base);
    }

    return status;
}

/**
 * Sends data over I2C bus.
 *
 * @param base LPI2C base address.
 * @param slave_addr Slave address.
 * @param slave_addr_fmt Slave address format, 7 or 10 bit.
 * @param buf  Pointer to data buffer.
 * @param len  Data buffer length in bytes.
 * @param stop Determines whether STOP condition is sent after data transfer.
 *
 * @return Execution status.
 */
int imx_lpi2c_send(imx_base_t base, uint16_t slave_addr, imx_lpi2c_addr_fmt_t slave_addr_fmt, uint8_t *buf,
                   unsigned int len, unsigned int stop)
{
    int status;

    if (len <= 0) {
        return 0;
    }

    /* Send master blocking data to slave */
    status = imx_lpi2c_send_addr(base, slave_addr, slave_addr_fmt, 0);

    if (status) {
        if (debug_flag) {
            kprintf("%s sendaddr 0x%x error 1\n", __FUNCTION__, slave_addr);
        }
        return status;
    }

    while (len > 0) {
        status = imx_lpi2c_send_byte(base, *(uint8_t *) buf, IMX_I2C_TX_DATA);
        if (status) {
            if (debug_flag) {
                kprintf("%s sendaddr 0x%x error 2\n", __FUNCTION__, slave_addr);
            }
            return status;
        }
        if (IMX_IN(base + IMX_LPI2C_MSR) & 0x3C00) {
            if (debug_flag) {
                kprintf("imx_lpi2c_send msr error %u\n", IMX_IN(base + IMX_LPI2C_MSR));
            }
        }
        ++buf;
        --len;
    }

    if (stop) {
        status = imx_lpi2c_stop(base);
    }

    return status;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_lpi2c_drv.c $ $Rev: 891625 $")
#endif
