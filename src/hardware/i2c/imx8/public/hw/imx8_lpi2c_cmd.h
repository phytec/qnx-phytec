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

#ifndef IMX_LPI2C_CMD_H_
#define IMX_LPI2C_CMD_H_

#include <hw/i2c.h>

/* Configures IGNACK bit in the MCFGR1 register. When set, the received NACK field is ignored and assumed to be ACK.
 * This bit is required to be set in Ultra-Fast Mode.*/
#define DCMD_I2C_SET_IGNACK          __DIOT (_DCMD_I2C, 12, uint32_t)
/* Configures the pin mode in the MCFGR1 register. */
#define DCMD_I2C_SET_PINCFG          __DIOT (_DCMD_I2C, 13, imx_lpi2c_pincfg_t)

/**
 * Pin configuration values for DCMD_I2C_SET_PINCFG DCMD command.
 */
typedef enum _imx_lpi2c_pincfg {
    IMX_LPI2C_2PIN_OD = 0,     /**< LPI2C configured for 2-pin open drain mode. */
    IMX_LPI2C_2PIN_OUT = 1,    /**< LPI2C configured for 2-pin output only mode (ultra-fast mode). */
    IMX_LPI2C_2PIN_PP = 2,     /**< LPI2C configured for 2-pin push-pull mode. */
    IMX_LPI2C_4PIN_PP = 3,     /**< LPI2C configured for 4-pin push-pull mode. */
    IMX_LPI2C_2PIN_OD_SEP = 4, /**< LPI2C configured for 2-pin open drain mode with separate LPI2C slave. */
    IMX_LPI2C_2PIN_OUT_SEP = 5,/**< LPI2C configured for 2-pin output only mode (ultra-fast mode) with separate LPI2C slave. */
    IMX_LPI2C_2PIN_PP_SEP = 6, /**< LPI2C configured for 2-pin push-pull mode with separate LPI2C slave. */
    IMX_LPI2C_4PIN_PP_INV = 7  /**< LPI2C configured for 4-pin push-pull mode (inverted outputs). */
} imx_lpi2c_pincfg_t;

#endif /* IMX_LPI2C_CMD_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/public/hw/imx8_lpi2c_cmd.h $ $Rev: 869802 $")
#endif
