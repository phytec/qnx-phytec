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
 * Handles a driver-specific devctl().
 *
 * @param hdl Driver handle.
 * @param cmd Device command.
 * @param msg Message buffer.
 * @param msglen Length of message buffer in bytes.
 * @param nbytes Bytes to return (<= msglen).
 * @param info Extra status information returned by devctl.
 *
 * @return EOK when success.
 */
int imx_ctl(void *hdl, int cmd, void *msg, int msglen, int *nbytes, int *info)
{
    imx_dev_t *dev = hdl;
    imx_lpi2c_pincfg_t cfg;

    switch (cmd) {
        /* IGNACK is required for ultra fast mode */
        case DCMD_I2C_SET_IGNACK:
            if (msg) {
                dev->nack = *(uint32_t *)msg;
                if (dev->nack) {
                    dev->regbase->mcfgr1 |= IMX_LPI2C_MCFGR1_IGNACK_MASK;
                } else {
                    dev->regbase->mcfgr1 &= ~IMX_LPI2C_MCFGR1_IGNACK_MASK;
                }
            }
            break;
        /* Reconfigures I2C pins */
        case DCMD_I2C_SET_PINCFG:
            if (msg) {
                cfg = (*(imx_lpi2c_pincfg_t *)msg);
                dev->regbase->mcfgr1 = (dev->regbase->mcfgr1 & (~IMX_LPI2C_MCFGR1_PINCFG_MASK)) |
                                       (IMX_LPI2C_MCFGR1_PINCFG(cfg) & IMX_LPI2C_MCFGR1_PINCFG_MASK);
            }
            break;
        default:
            if (dev->verbose) {
                _i2c_slogf("i2c_imx8 ctl unknown command %u", cmd);
            }
            break;
    }
    return EOK;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/i2c/imx8/ctl.c $ $Rev: 869802 $")
#endif
