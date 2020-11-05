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
#ifndef IMX_LPI2C_DRV_H_
#define IMX_LPI2C_DRV_H_

#include <stdint.h>
#include <aarch64/imx8_common/imx_lpi2c.h>
#include "imx_startup.h"

typedef enum {
    IMX_LPI2C_ADDR_7BIT,
    IMX_LPI2C_ADDR_10BIT
} imx_lpi2c_addr_fmt_t;

#define IMX_I2C_SCL_MAX_ERR 100

int imx_lpi2c_set_bus_speed(imx_base_t base, unsigned int input_clk, unsigned int speed, unsigned int *ospeed);
int imx_lpi2c_recv(imx_base_t base, uint16_t slave_addr, imx_lpi2c_addr_fmt_t slave_addr_fmt, uint8_t *buf,
                   unsigned int len, unsigned int stop);
int imx_lpi2c_send(imx_base_t base, uint16_t slave_addr, imx_lpi2c_addr_fmt_t slave_addr_fmt, uint8_t *buf,
                   unsigned int len, unsigned int stop);
int imx_lpi2c_init(imx_base_t base, uint32_t source_clock, uint32_t speed, uint32_t * real_speed);

#endif /* IMX_LPI2C_DRV_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_lpi2c_drv.h $ $Rev: 869196 $")
#endif
