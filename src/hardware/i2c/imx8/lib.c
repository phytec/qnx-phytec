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
 * Used by higher-level code to access the hardware-specific functions.
 *
 * @param funcs   The function table to fill in.
 * @param tabsize The size of the structure that funcs points to, in bytes.
 *
 * @return Execution status.
 * @retval 0 Success.
 */
int i2c_master_getfuncs(i2c_master_funcs_t *funcs, int tabsize)
{
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, init, imx_init, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, fini, imx_fini, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, send, imx_send, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, recv, imx_recv, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, set_slave_addr, imx_set_slave_addr, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, set_bus_speed, imx_set_bus_speed, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, version_info, imx_version_info, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, driver_info, imx_driver_info, tabsize);
    I2C_ADD_FUNC(i2c_master_funcs_t, funcs, ctl, imx_ctl, tabsize);
    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/lib.c $ $Rev: 869802 $")
#endif
