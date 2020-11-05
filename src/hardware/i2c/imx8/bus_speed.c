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

#include <stdbool.h>
#include "proto.h"


/**
 * Sets i2c bus speed.
 * @param hdl    Pointer to I2C device structure.
 * @param speed  Required speed.
 * @param ospeed Pointer to variable where real calculated speed is returned.
 *
 * @return  Execution status.
 * @retval -1 Fail.
 * @retval 0  Success.
 */
int imx_set_bus_speed(void *hdl, unsigned int speed, unsigned int *ospeed)
{
    imx_dev_t *dev = hdl;

    uint8_t         prescaler = 0;
    uint8_t         clk;
    uint32_t        real_rate = 0;
    uint8_t         filtscl = 0;
    uint32_t        scl_latency = 0;
    uint8_t         timeout = 0;
    uint8_t         master_en = 0;
    uint32_t        reg_val = 0;
    i2c_status_t    status;
    uint8_t         presc_val;
    uint8_t         clklo, clkhi, sethold, datavd, ratio;
    uint16_t        busidle;
    bool            success = false;

    if (speed > 1000000) {
        _i2c_slogf("lpi2c_imx driver does not support frequencies above the Fast-mode Plus range.");
        return -1;
    }

    if (speed != dev->speed) {
        /* Wait for busy I2C bus */
        status = imx_wait_busy(dev);
        status |= imx_wait_tx_fifo(dev);

        if (status != I2C_STATUS_DONE) {
            return -1;
        }
        dev->speed = speed;

        /* Disable master if enabled */
        if (dev->regbase->mcr & IMX_LPI2C_MCR_MEN_MASK) {
            while ((dev->regbase->msr & IMX_LPI2C_MSR_MBF_MASK) && (timeout++ < 100)) {
                delay(1);
            }
            dev->regbase->mcr &= ~(IMX_LPI2C_MCR_MEN_MASK);
            master_en = 1;
        }

        /* I2C baud rate calculation is using 3:2 tlow:tHigh ratio */
        ratio = 5;

        /* Iterates over prescaler to find best clock settings */
        while (prescaler < 8) {
            presc_val = 1 << prescaler;

            /* Calculate clk and round-up to have frequencies =< speed */
            clk = (dev->input_clk + (speed * presc_val) - 1) / (speed * presc_val) - 2;

            /* Divide clk into requested ratio of clkhi (2) and round-up */
            clkhi = ((clk * 2) + ratio - 1) / ratio;
            clklo = clk - clkhi;
            scl_latency = (2 + filtscl) / (1 << prescaler);
            if (clkhi >= scl_latency) {
                /* Extract scl_latency from clkhi */
                clkhi -= scl_latency;
                if ((clklo < 64) && (clklo > 2) && (clkhi >= 1)) {
                    success = true;
                    break;
                }
            }
            prescaler++;
        }

        if (!success) {
            _i2c_slogf("lpi2c_imx - Not possible to calculate requested frequency: %d Hz.", speed);
            return -1;
        }
        if (dev->verbose) {
            _i2c_slogf("lpi2c_imx - clk tlow:thigh clk %u ratio - %u : %u ", clk, clklo, clkhi + scl_latency);
        }
        /* According to RM sethold minimum value is 0x2 */
        sethold = clkhi + scl_latency;
        if (sethold < 2) {
            sethold =  2;
        }
        /* According to RM datavd minimum value is 0x1 */
        datavd = clkhi / 2;
        if (datavd < 1) {
            datavd = 1;
        }
        busidle = (clklo + sethold + 2) * 2;
        /* MCFGR0 */
        dev->regbase->mccr0 = IMX_LPI2C_MCCR0_DATAVD(datavd) | IMX_LPI2C_MCCR0_SETHOLD(sethold) |
                              IMX_LPI2C_MCCR0_CLKHI(clkhi) | IMX_LPI2C_MCCR0_CLKLO(clklo);
        /* MCFGR1 */
        reg_val = dev->regbase->mcfgr1;
        reg_val &= ~(IMX_LPI2C_MCFGR1_PRESCALE_MASK);
        dev->regbase->mcfgr1 = reg_val | IMX_LPI2C_MCFGR1_PRESCALE(prescaler);
        /* MCFGR2 */
        dev->regbase->mcfgr2 = IMX_LPI2C_MCFGR2_FILTSDA(filtscl) | IMX_LPI2C_MCFGR2_BUSIDLE(busidle) |
                               IMX_LPI2C_MCFGR2_FILTSCL(filtscl);
        /* Re-enable master */
        if (master_en) {
            dev->regbase->mcr |= (IMX_LPI2C_MCR_MEN_MASK);
        }
    }

    /* Calculates speed from register settings */
    if (ospeed || dev->verbose) {
        /* Get prescaler value */
        presc_val = 1 << ((dev->regbase->mcfgr1 & IMX_LPI2C_MCFGR1_PRESCALE_MASK) >> IMX_LPI2C_MCFGR1_PRESCALE_SHIFT);
        /* Get clko and clkhi */
        clk = ((dev->regbase->mccr0 & IMX_LPI2C_MCCR0_CLKLO_MASK) >> IMX_LPI2C_MCCR0_CLKLO_SHIFT)
              + ((dev->regbase->mccr0 & IMX_LPI2C_MCCR0_CLKHI_MASK) >> IMX_LPI2C_MCCR0_CLKHI_SHIFT);

        /* Calculate real rate */
        filtscl = (dev->regbase->mcfgr2 & IMX_LPI2C_MCFGR2_FILTSCL_MASK) >> IMX_LPI2C_MCFGR2_FILTSCL_SHIFT;
        scl_latency = (2 + filtscl) / presc_val;
        real_rate = dev->input_clk / ((clk + 2U + scl_latency) * presc_val);

        /* Prints calculated speed */
        if (dev->verbose) {
            _i2c_slogf("lpi2c_imx - rate set %u Hz", real_rate);
        }
        /* Returns calculated speed */
        if (ospeed) {
            *ospeed = real_rate;
        }
    }
    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/bus_speed.c $ $Rev: 869802 $")
#endif
